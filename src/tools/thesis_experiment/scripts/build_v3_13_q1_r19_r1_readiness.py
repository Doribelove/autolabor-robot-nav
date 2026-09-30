#!/usr/bin/env python3
"""Build immutable R19-R1 boundary-atomic preflight release."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r1_boundary_atomic_preflight_contract.yaml"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_q1_r19_r1_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
RESOURCES = [
    CONTRACT,
    Path(
        "artifacts/v3/multiactor/"
        "v3_13_q1_r19_d1_terminal_atomicity_diagnosis/"
        "terminal_atomicity_diagnosis.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/"
        "v3_13_q1_r19_d1_terminal_atomicity_diagnosis/"
        "evidence_manifest.sha256"
    ),
    Path(
        "artifacts/v3/multiactor/"
        "v3_13_q1_r19_d1_terminal_atomicity_diagnosis/"
        "independent_review.yaml"
    ),
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_q1_r17_goal_aligned_recovery_contract.yaml"
    ),
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_d0_multiactor_foundation_contract.yaml"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "goal_aligned_execution_adapter_r1.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "progress_recovery_candidate_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "execution_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "multi_actor_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_goal_aligned_boundary_atomic_preflight_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_goal_aligned_recovery_preflight_episode_r1.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_progress_recovery_preflight_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_sequential_clone_episode_v2.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_sequential_clone_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_clone_episode.py"
    ),
    Path(
        "src/simulation/m2_gazebo/launch/"
        "m2_v3_13_goal_aligned_boundary_atomic_preflight.launch"
    ),
    Path("src/application/v3_scope_planner/CMakeLists.txt"),
    Path("src/simulation/m2_gazebo/launch/spawn_m2.launch"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("devel/lib/libv3_reactive_actor_plugin.so"),
    Path("src/simulation/m2_gazebo/src/m2_ackermann_plugin.cpp"),
    Path("devel/lib/libm2_ackermann_plugin.so"),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "v3_13_multiactor_design.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r1_actual_teb_matrix.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_13_q1_r19_r1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r19_r1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r19_r1_boundary_atomic_preflight.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_13_q1_r19_r1_boundary_atomic_preflight.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r19_r1_preflight_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_13_q1_r19_r1_boundary_atomic_preflight.py"
    ),
]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(path):
    target = W / path
    require(target.is_file(), "required file absent: {}".format(path))
    return target.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def write_or_check(path, payload, check):
    target = W / path
    if check:
        require(target.is_file(), "output absent")
        require(target.read_bytes() == payload, "output drift")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    require(not target.exists(), "refusing overwrite")
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(target))


def build_outputs():
    contract = yaml.safe_load(read(CONTRACT))
    upstream = contract["upstream"]
    for key, path in (
        ("R19_D1_diagnosis_sha256", RESOURCES[1]),
        ("R19_D1_manifest_sha256", RESOURCES[2]),
        ("R19_D1_review_sha256", RESOURCES[3]),
        ("R17_geometry_sha256", RESOURCES[4]),
        ("nonrecursive_adapter_sha256", RESOURCES[6]),
    ):
        require(sha(read(path)) == upstream[key], "{} differs".format(path))
    row = {
        "sequence": 1,
        "block_index": 1,
        "seed": 7677,
        "scene_id": "crossing_and_oncoming",
        "actor_policies_evaluator_only": ["hesitant", "cautious"],
        "branch_target_sim_s": 12.5,
        "pre_action_wait_s": 5.0,
        "forced_candidate_id": "recovery_evasive_right_v120_r3",
        "behavior_propensity": 1.0,
        "attempt": 1,
        "non_evidence_preflight": True,
    }
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R1",
        "status": "exact_fresh_boundary_atomic_preflight_schedule",
        "row_count": 1,
        "rows": [row],
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R1",
        "status": "closed_dependency_freeze",
        "resource_count": len(RESOURCES),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in RESOURCES
        ],
        "explicit_thesis_workspace_overlay_required": True,
        "credential_environment_scrub_required": True,
    }
    schedule_payload = yaml_bytes(schedule)
    dependency_payload = yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R1",
        "status": "unique_fresh_boundary_atomic_preflight_release",
        "contract_sha256": sha(read(CONTRACT)),
        "schedule_sha256": sha(schedule_payload),
        "dependency_freeze_sha256": sha(dependency_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/"
            "v3_13_q1_r19_r1_boundary_atomic_preflight"
        ),
        "authorized_units": 1,
        "authorized_seed": 7677,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "execution_authorized": True,
        "evidence_claim_authorized": False,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    return {
        SCHEDULE: schedule_payload,
        DEPENDENCIES: dependency_payload,
        RELEASE: yaml_bytes(release),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, args.check)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in RESOURCES + list(outputs)
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, args.check)
    print(
        "release_sha256={} manifest_sha256={}".format(
            sha(outputs[RELEASE]), sha(manifest)
        )
    )


if __name__ == "__main__":
    main()
