#!/usr/bin/env python3
"""Build the immutable V3-14-S1 schedule, closure, and simulation release."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path("config/thesis_experiments/v3/v3_14_s1_multiaction_support_contract.yaml")
AUTHORIZATION = Path("experiments/manifests/v3/v3_14_s1_simulation_authorization.yaml")
ROOT = Path("artifacts/v3/multiactor/v3_14_s1_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "release_manifest.sha256"

RESOURCES = [
    CONTRACT,
    AUTHORIZATION,
    Path("artifacts/v3/multiactor/v3_14_i1_action_binding_integrity/terminal_freeze.sha256"),
    Path("artifacts/v3/multiactor/v3_14_i1_action_binding_integrity/independent_review.yaml"),
    Path("artifacts/v3/multiactor/v3_13_q1_r19_r5_r1_assessment.yaml"),
    Path("artifacts/v3/multiactor/v3_13_q1_r19_r5_r1_assessment_review.yaml"),
    Path("config/thesis_experiments/v3/v3_13_d0_multiactor_foundation_contract.yaml"),
    Path("config/thesis_experiments/v3/v3_13_q1_r17_goal_aligned_recovery_contract.yaml"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_14_action_binding.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_14_s1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_14_s1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_14_s1_multiaction_support.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_14_s1_multiaction_support.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_14_s1_result.py"),
    Path("src/application/v3_scope_planner/scripts/v3_scope_tolerance_repaired_prefix_aligned_bank_clone_episode.py"),
    Path("src/simulation/m2_gazebo/launch/m2_v3_13_tolerance_repaired_prefix_aligned_bank_clone_episode.launch"),
    Path("devel/lib/libv3_reactive_actor_plugin.so"),
    Path("devel/lib/libm2_ackermann_plugin.so"),
]


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


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


def rotated_actions(actions, block_index):
    ordered = list(actions)
    shift = (block_index - 1) % len(ordered)
    ordered = ordered[shift:] + ordered[:shift]
    if block_index % 2 == 0:
        ordered.reverse()
    return ordered


def schedule_rows(contract):
    population = contract["population"]
    actions = population["actions"]
    rows = []
    for offset, action in enumerate(actions, 1):
        rows.append({
            "sequence": offset,
            "phase": "preflight",
            "block_index": 1,
            "seed": int(contract["preflight"]["seed"]),
            "scene_id": contract["preflight"]["scene_id"],
            "actor_policies_evaluator_only": list(contract["preflight"]["actor_policies_evaluator_only"]),
            "branch_target_sim_s": float(population["branch_target_sim_s"]),
            "pre_action_wait_s": float(population["nominal_wait_depth_s"]),
            "forced_candidate_id": action,
            "behavior_propensity": float(population["behavior_propensity_each"]),
            "attempt": 1,
        })
    sequence = len(rows)
    strata = population["scene_policy_strata"]
    for block_index, seed in enumerate(contract["evidence"]["seeds"], 1):
        stratum = strata[(block_index - 1) % len(strata)]
        for action in rotated_actions(actions, block_index):
            sequence += 1
            rows.append({
                "sequence": sequence,
                "phase": "evidence",
                "block_index": block_index,
                "seed": int(seed),
                "scene_id": stratum["scene_id"],
                "actor_policies_evaluator_only": list(stratum["actor_policies_evaluator_only"]),
                "branch_target_sim_s": float(population["branch_target_sim_s"]),
                "pre_action_wait_s": float(population["nominal_wait_depth_s"]),
                "forced_candidate_id": action,
                "behavior_propensity": float(population["behavior_propensity_each"]),
                "attempt": 1,
            })
    return rows


def load_binding():
    path = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_14_action_binding.py"
    spec = importlib.util.spec_from_file_location("v3_14_s1_binding", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def build_outputs():
    contract = yaml.safe_load(read(CONTRACT))
    authorization = yaml.safe_load(read(AUTHORIZATION))
    rows = schedule_rows(contract)
    if len(rows) != 45 or len([r for r in rows if r["phase"] == "preflight"]) != 5:
        raise RuntimeError("schedule size differs")
    if {r["seed"] for r in rows} != {7719, *range(7721, 7729)}:
        raise RuntimeError("fresh seed roster differs")
    if min(r["seed"] for r in rows) <= 7704:
        raise RuntimeError("historical seed reuse")
    actions = set(contract["population"]["actions"])
    for phase, blocks in (("preflight", [1]), ("evidence", range(1, 9))):
        for block_index in blocks:
            block = [r for r in rows if r["phase"] == phase and r["block_index"] == block_index]
            if {r["forced_candidate_id"] for r in block} != actions:
                raise RuntimeError("incomplete action support")
    binding = load_binding()
    for row in rows:
        command = ["roslaunch", "m2_gazebo", contract["execution"]["launch"]]
        command += binding.build_launch_binding_args(row, tuple(actions))
        binding.verify_preconsumption_binding(row, command, tuple(actions))
    if not authorization["execution_authorized"] or authorization["authorized_scope"]["exact_units"] != 45:
        raise RuntimeError("authorization differs")
    upstream = contract["upstream"]
    expected_hashes = {
        RESOURCES[2]: upstream["v3_14_i1_terminal_freeze_sha256"],
        RESOURCES[3]: upstream["v3_14_i1_independent_review_sha256"],
        RESOURCES[4]: upstream["r19_r5_r1_assessment_sha256"],
        RESOURCES[5]: upstream["r19_r5_r1_review_sha256"],
    }
    for path, expected in expected_hashes.items():
        if sha(read(path)) != expected:
            raise RuntimeError("upstream drift: {}".format(path))
    schedule = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-14-S1",
        "status": "immutable_exact_schedule", "row_count": 45,
        "preflight_units": 5, "evidence_units": 40, "rows": rows,
    }
    schedule_payload = yaml_bytes(schedule)
    dependencies = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-14-S1",
        "status": "closed_runtime_and_analysis_dependency_freeze", "resource_count": len(RESOURCES),
        "resources": [{"path": str(path), "sha256": sha(read(path))} for path in RESOURCES],
    }
    dependency_payload = yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-14-S1",
        "status": "fresh_bounded_simulation_release",
        "contract_sha256": sha(read(CONTRACT)), "authorization_sha256": sha(read(AUTHORIZATION)),
        "exact_schedule_sha256": sha(schedule_payload), "dependency_freeze_sha256": sha(dependency_payload),
        "authorized_units": 45, "preflight_units": 5, "evidence_units": 40,
        "fresh_seeds": [7719, *range(7721, 7729)], "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "preflight_gate_required_before_evidence": True, "execution_authorized": True,
        "predictor_fit_authorized": False, "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False, "training_authorized": False,
        "hardware_device_access_authorized": False, "real_vehicle_authorized": False,
    }
    release_payload = yaml_bytes(release)
    outputs = {SCHEDULE: schedule_payload, DEPENDENCIES: dependency_payload, RELEASE: release_payload}
    manifest = "".join("{}  {}\n".format(sha(read(path)), path) for path in RESOURCES)
    manifest += "".join("{}  {}\n".format(sha(payload), path) for path, payload in outputs.items())
    outputs[MANIFEST] = manifest.encode("utf-8")
    return outputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, args.check)
    print("release_sha256={} manifest_sha256={}".format(sha(outputs[RELEASE]), sha(outputs[MANIFEST])))


if __name__ == "__main__":
    main()
