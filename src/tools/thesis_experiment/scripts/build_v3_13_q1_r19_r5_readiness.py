#!/usr/bin/env python3
"""Build immutable one-unit R19-R5 tolerance preflight release."""

import argparse
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r6_readiness.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r19_r5_builder_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r5_terminal_tolerance_preflight_contract.yaml"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_q1_r19_r5_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
RESOURCES = [
    CONTRACT,
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r19_r4_assessment/"
        "support_assessment.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r19_r4_assessment/"
        "evidence_freeze.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r19_r4_assessment/"
        "independent_result_review.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/"
        "v3_13_q1_r19_r4_d1_terminal_margin_diagnosis/"
        "terminal_margin_diagnosis.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/"
        "v3_13_q1_r19_r4_d1_terminal_margin_diagnosis/"
        "evidence_manifest.sha256"
    ),
    Path(
        "artifacts/v3/multiactor/"
        "v3_13_q1_r19_r4_d1_terminal_margin_diagnosis/"
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
        "goal_aligned_execution_adapter_r2.py"
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
        "v3_scope_tolerance_repaired_prefix_aligned_bank_clone_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_prefix_aligned_boundary_atomic_bank_clone_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_boundary_atomic_recovery_bank_clone_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_goal_aligned_boundary_atomic_preflight_episode.py"
    ),
    Path(
        "src/simulation/m2_gazebo/launch/"
        "m2_v3_13_tolerance_repaired_prefix_aligned_bank_clone_episode.launch"
    ),
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
        "build_v3_13_q1_r19_r5_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r19_r5_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r19_r5_terminal_tolerance_preflight.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_13_q1_r19_r5_terminal_tolerance_preflight.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r19_r5_preflight_result.py"
    ),
]


def build_outputs():
    contract = yaml.safe_load(BASE.read(CONTRACT))
    upstream = contract["upstream"]
    for key, path in (
        ("R19_R4_assessment_sha256", RESOURCES[1]),
        ("R19_R4_freeze_sha256", RESOURCES[2]),
        ("R19_R4_review_sha256", RESOURCES[3]),
        ("R19_R4_D1_diagnosis_sha256", RESOURCES[4]),
        ("R19_R4_D1_manifest_sha256", RESOURCES[5]),
        ("R19_R4_D1_review_sha256", RESOURCES[6]),
        ("R17_geometry_sha256", RESOURCES[7]),
    ):
        BASE.require(
            BASE.sha(BASE.read(path)) == upstream[key],
            "{} differs".format(path),
        )
    row = {
        "sequence": 1,
        "block_index": 1,
        "seed": 7695,
        "scene_id": "occluded_staggered_corridor",
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
        "stage": "V3-13-Q1-R19-R5",
        "status": "exact_fresh_terminal_tolerance_preflight_schedule",
        "row_count": 1,
        "rows": [row],
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R5",
        "status": "closed_dependency_freeze",
        "resource_count": len(RESOURCES),
        "resources": [
            {"path": str(path), "sha256": BASE.sha(BASE.read(path))}
            for path in RESOURCES
        ],
        "explicit_thesis_workspace_overlay_required": True,
        "credential_environment_scrub_required": True,
    }
    schedule_payload = BASE.yaml_bytes(schedule)
    dependency_payload = BASE.yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R5",
        "status": "unique_fresh_terminal_tolerance_preflight_release",
        "contract_sha256": BASE.sha(BASE.read(CONTRACT)),
        "schedule_sha256": BASE.sha(schedule_payload),
        "dependency_freeze_sha256": BASE.sha(dependency_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/"
            "v3_13_q1_r19_r5_terminal_tolerance_preflight"
        ),
        "authorized_units": 1,
        "authorized_seed": 7695,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "execution_authorized": True,
        "predictor_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    return {
        SCHEDULE: schedule_payload,
        DEPENDENCIES: dependency_payload,
        RELEASE: BASE.yaml_bytes(release),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        BASE.write_or_check(path, payload, args.check)
    manifest = "".join(
        "{}  {}\n".format(BASE.sha(BASE.read(path)), path)
        for path in RESOURCES + list(outputs)
    ).encode("utf-8")
    BASE.write_or_check(MANIFEST, manifest, args.check)
    print(
        "release_sha256={} manifest_sha256={}".format(
            BASE.sha(outputs[RELEASE]), BASE.sha(manifest)
        )
    )


if __name__ == "__main__":
    main()
