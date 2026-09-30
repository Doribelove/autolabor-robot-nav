#!/usr/bin/env python3
"""Build the deterministic V3-14-I1 pure-offline integrity closure."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
MODULE_ABS = W / (
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v3_14_action_binding.py"
)
SPEC = importlib.util.spec_from_file_location("v3_14_action_binding", MODULE_ABS)
binding = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(binding)


CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_14_i1_action_binding_integrity_contract.yaml"
)
MODULE = Path(
    "src/tools/thesis_experiment/src/thesis_experiment/"
    "v3_14_action_binding.py"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_14_i1_action_binding_integrity.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_14_i1_action_binding_integrity.py"
)
ROOT = Path("artifacts/v3/multiactor/v3_14_i1_action_binding_integrity")
REPORT = ROOT / "integrity_report.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def write_or_check(path, payload, check):
    target = W / path
    if check:
        if not target.is_file() or target.read_bytes() != payload:
            raise RuntimeError("{} differs".format(path))
        return
    if target.exists():
        raise RuntimeError("{} exists".format(path))
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(target))


def expect_rejected(callback):
    try:
        callback()
    except binding.ActionBindingError:
        return True
    raise RuntimeError("invalid action binding was accepted")


def fixture_row(candidate, sequence):
    return {
        "sequence": sequence,
        "block_index": 1,
        "seed": 0,
        "scene_id": "offline_identity_fixture",
        "forced_candidate_id": candidate,
        "behavior_propensity": 0.20,
    }


def build_report(contract):
    historical = contract["historical_failure"]
    for path_key, hash_key in (
        ("terminal_failure_path", "terminal_failure_sha256"),
        ("independent_review_path", "independent_review_sha256"),
        ("matrix_runner_path", "matrix_runner_sha256"),
        ("delegated_preflight_runner_path", "delegated_preflight_runner_sha256"),
    ):
        if sha(read(Path(historical[path_key]))) != historical[hash_key]:
            raise RuntimeError("historical hash differs: {}".format(path_key))
    failure = yaml.safe_load(read(Path(historical["terminal_failure_path"])))
    if failure["scientific_episode_count"] != 0:
        raise RuntimeError("historical scientific count differs")
    mismatches = failure["observed_mismatches"]
    if len(mismatches) != 2 or any(
        item["scheduled_candidate_id"] == item["runtime_candidate_id"]
        for item in mismatches
    ):
        raise RuntimeError("historical mismatch replay differs")
    matrix_source = read(Path(historical["matrix_runner_path"])).decode("utf-8")
    delegated_source = read(
        Path(historical["delegated_preflight_runner_path"])
    ).decode("utf-8")
    delegated_call_detected = "R5.run_episode(row, scene, root, ros_port)" in matrix_source
    hardcoded_token = (
        '"forced_candidate_id:=recovery_evasive_right_v120_r3"'
    )
    hardcoded_r3_detected = hardcoded_token in delegated_source
    if not delegated_call_detected or not hardcoded_r3_detected:
        raise RuntimeError("historical binding cause not reproduced")

    candidates = contract["frozen_candidate_bank"]["candidate_ids"]
    positive_launch = 0
    positive_runtime = 0
    launch_substitution_rejections = 0
    runtime_substitution_rejections = 0
    for sequence, candidate in enumerate(candidates, 1):
        row = fixture_row(candidate, sequence)
        command = ["roslaunch", "m2_gazebo", "fixture.launch"]
        command += binding.build_launch_binding_args(row, candidates)
        binding.verify_preconsumption_binding(row, command, candidates)
        positive_launch += 1
        report = dict(row)
        binding.verify_runtime_report_binding(row, report, candidates)
        positive_runtime += 1
        for other in candidates:
            if other == candidate:
                continue
            mutated = [
                (
                    "forced_candidate_id:={}".format(other)
                    if token.startswith("forced_candidate_id:=")
                    else token
                )
                for token in command
            ]
            expect_rejected(
                lambda row=row, command=mutated: binding.verify_preconsumption_binding(
                    row, command, candidates
                )
            )
            launch_substitution_rejections += 1
            bad_report = dict(report)
            bad_report["forced_candidate_id"] = other
            expect_rejected(
                lambda row=row, report=bad_report: binding.verify_runtime_report_binding(
                    row, report, candidates
                )
            )
            runtime_substitution_rejections += 1
    base_row = fixture_row(candidates[0], 1)
    base_command = ["roslaunch"] + binding.build_launch_binding_args(
        base_row, candidates
    )
    missing = [
        token for token in base_command
        if not token.startswith("forced_candidate_id:=")
    ]
    duplicate = list(base_command) + [
        "forced_candidate_id:={}".format(candidates[0])
    ]
    missing_rejected = expect_rejected(
        lambda: binding.verify_preconsumption_binding(base_row, missing, candidates)
    )
    duplicate_rejected = expect_rejected(
        lambda: binding.verify_preconsumption_binding(base_row, duplicate, candidates)
    )
    checks = {
        "historical_failure_and_review_hashes_match": True,
        "historical_two_mismatches_replayed": True,
        "historical_zero_scientific_units_replayed": True,
        "matrix_to_single_action_preflight_delegation_detected": delegated_call_detected,
        "delegated_runner_hardcoded_r3_detected": hardcoded_r3_detected,
        "all_candidate_launch_bindings_pass": positive_launch == len(candidates),
        "all_candidate_runtime_bindings_pass": positive_runtime == len(candidates),
        "all_cross_candidate_launch_substitutions_rejected": launch_substitution_rejections == 20,
        "all_cross_candidate_runtime_substitutions_rejected": runtime_substitution_rejections == 20,
        "missing_candidate_override_rejected": missing_rejected,
        "duplicate_candidate_override_rejected": duplicate_rejected,
        "execution_authority_remains_zero": not any(
            contract["authority"][key]
            for key in (
                "execution_authorized",
                "predictor_fit_authorized",
                "actual_TEB_execution_authorized",
                "perception_execution_authorized",
                "training_authorized",
                "hardware_device_access_authorized",
                "real_vehicle_authorized",
            )
        ),
    }
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-14-I1",
        "status": (
            "v3_14_i1_action_binding_fail_closed_integrity_closure_pass"
            if all(checks.values())
            else "v3_14_i1_action_binding_integrity_closure_fail"
        ),
        "historical_stage_preserved": "V3-13-Q1-R19-R6",
        "historical_terminal_failure_sha256": historical[
            "terminal_failure_sha256"
        ],
        "candidate_count": len(candidates),
        "positive_preconsumption_checks": positive_launch,
        "positive_runtime_report_checks": positive_runtime,
        "cross_candidate_preconsumption_rejections": launch_substitution_rejections,
        "cross_candidate_runtime_rejections": runtime_substitution_rejections,
        "checks": checks,
        "execution_ready": False,
        "execution_authorized": False,
        "evidence_budget_authorized": 0,
        "predictor_fit_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "next_safe_entry": (
            "independent_fresh_seed_multi_action_support_design_and_release"
        ),
    }


def build_outputs():
    contract = yaml.safe_load(read(CONTRACT))
    report = build_report(contract)
    historical = contract["historical_failure"]
    resources = [
        CONTRACT,
        MODULE,
        BUILDER,
        REVIEWER,
        Path(historical["terminal_failure_path"]),
        Path(historical["independent_review_path"]),
        Path(historical["matrix_runner_path"]),
        Path(historical["delegated_preflight_runner_path"]),
    ]
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-14-I1",
        "status": "closed_pure_offline_dependency_freeze",
        "resource_count": len(resources),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in resources
        ],
    }
    outputs = {
        REPORT: yaml_bytes(report),
        DEPENDENCIES: yaml_bytes(dependencies),
    }
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in resources
    )
    manifest += "".join(
        "{}  {}\n".format(sha(payload), path)
        for path, payload in outputs.items()
    )
    outputs[MANIFEST] = manifest.encode("utf-8")
    return outputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, args.check)
    report = yaml.safe_load(outputs[REPORT])
    print(
        "status={} report_sha256={} manifest_sha256={}".format(
            report["status"], sha(outputs[REPORT]), sha(outputs[MANIFEST])
        )
    )


if __name__ == "__main__":
    main()
