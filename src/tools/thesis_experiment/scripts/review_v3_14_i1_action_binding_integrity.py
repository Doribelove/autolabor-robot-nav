#!/usr/bin/env python3
"""Independently review the V3-14-I1 action-binding closure."""

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


ROOT = Path("artifacts/v3/multiactor/v3_14_i1_action_binding_integrity")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_14_i1_action_binding_integrity_contract.yaml"
)
REPORT = ROOT / "integrity_report.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
REVIEW = ROOT / "independent_review.yaml"
TERMINAL = ROOT / "terminal_freeze.sha256"


def payload(path):
    return (W / path).read_bytes()


def digest(value):
    return hashlib.sha256(value).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def write_or_check(path, value, check):
    target = W / path
    if check:
        if not target.is_file() or target.read_bytes() != value:
            raise RuntimeError("{} differs".format(path))
        return
    if target.exists():
        raise RuntimeError("{} exists".format(path))
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(value)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(target))


def rejected(callback):
    try:
        callback()
    except binding.ActionBindingError:
        return True
    return False


def review_value():
    contract = yaml.safe_load(payload(CONTRACT))
    report = yaml.safe_load(payload(REPORT))
    dependencies = yaml.safe_load(payload(DEPENDENCIES))
    manifest_lines = payload(MANIFEST).decode("utf-8").splitlines()
    manifest = {}
    for line in manifest_lines:
        expected, path = line.split("  ", 1)
        if path in manifest:
            raise RuntimeError("duplicate manifest path")
        manifest[path] = expected
        if digest(payload(Path(path))) != expected:
            raise RuntimeError("manifest hash drift: {}".format(path))
    for resource in dependencies["resources"]:
        if digest(payload(Path(resource["path"]))) != resource["sha256"]:
            raise RuntimeError("dependency hash drift")
    candidates = contract["frozen_candidate_bank"]["candidate_ids"]
    positive = 0
    negative = 0
    for sequence, candidate in enumerate(candidates, 1):
        row = {
            "sequence": sequence,
            "block_index": 2,
            "seed": 0,
            "scene_id": "independent_offline_fixture",
            "forced_candidate_id": candidate,
            "behavior_propensity": 0.20,
        }
        command = ["roslaunch"] + binding.build_launch_binding_args(
            row, candidates
        )
        binding.verify_preconsumption_binding(row, command, candidates)
        binding.verify_runtime_report_binding(row, dict(row), candidates)
        positive += 2
        wrong = candidates[(sequence % len(candidates))]
        bad_command = [
            (
                "forced_candidate_id:={}".format(wrong)
                if token.startswith("forced_candidate_id:=")
                else token
            )
            for token in command
        ]
        bad_report = dict(row)
        bad_report["forced_candidate_id"] = wrong
        if not rejected(
            lambda row=row, command=bad_command: binding.verify_preconsumption_binding(
                row, command, candidates
            )
        ):
            raise RuntimeError("independent launch mismatch accepted")
        if not rejected(
            lambda row=row, report=bad_report: binding.verify_runtime_report_binding(
                row, report, candidates
            )
        ):
            raise RuntimeError("independent runtime mismatch accepted")
        negative += 2
    authority = contract["authority"]
    zero_authority = (
        authority["offline_only"]
        and authority["evidence_budget_authorized"] == 0
        and not any(
            authority[key]
            for key in authority
            if key not in ("offline_only", "evidence_budget_authorized")
        )
    )
    checks = {
        "builder_report_passed": report["status"].endswith("closure_pass"),
        "all_builder_checks_true": all(report["checks"].values()),
        "manifest_rehashed": True,
        "dependency_freeze_rehashed": True,
        "independent_positive_binding_checks": positive == 10,
        "independent_mismatch_rejections": negative == 10,
        "zero_execution_training_hardware_authority": zero_authority,
        "historical_R19_R6_remains_nonreusable": not contract[
            "historical_preservation"
        ]["reuse_R18_or_R19_identity_seed_budget"],
    }
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-14-I1",
        "status": (
            "v3_14_i1_independent_action_binding_integrity_review_pass"
            if all(checks.values())
            else "v3_14_i1_independent_action_binding_integrity_review_fail"
        ),
        "integrity_report_sha256": digest(payload(REPORT)),
        "evidence_manifest_sha256": digest(payload(MANIFEST)),
        "checks": checks,
        "execution_ready": False,
        "execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    value = review_value()
    review_payload = yaml_bytes(value)
    terminal_payload = (
        "{}  {}\n{}  {}\n{}  {}\n".format(
            digest(payload(MANIFEST)), MANIFEST,
            digest(payload(REPORT)), REPORT,
            digest(review_payload), REVIEW,
        )
    ).encode("utf-8")
    write_or_check(REVIEW, review_payload, args.check)
    write_or_check(TERMINAL, terminal_payload, args.check)
    print(
        "status={} review_sha256={} terminal_sha256={}".format(
            value["status"], digest(review_payload), digest(terminal_payload)
        )
    )


if __name__ == "__main__":
    main()
