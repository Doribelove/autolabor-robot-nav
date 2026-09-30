#!/usr/bin/env python3
"""Build immutable 40-unit R19-R4 support release."""

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
    "v3_13_q1_r19_r4_builder_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r4_prefix_aligned_support_contract.yaml"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_q1_r19_r4_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
RESOURCES = [
    CONTRACT,
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r19_r3_assessment/"
        "preflight_assessment.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r19_r3_assessment/"
        "evidence_freeze.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r19_r3_assessment/"
        "independent_result_review.yaml"
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
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_goal_aligned_recovery_bank_clone_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_progress_recovery_bank_clone_episode.py"
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
        "m2_v3_13_prefix_aligned_boundary_atomic_bank_clone_episode.launch"
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
        "run_v3_13_q1_r6_recovery_support.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r19_r2_boundary_atomic_support.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r19_r3_prefix_matching_preflight.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_13_q1_r19_r4_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r19_r4_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r19_r4_prefix_aligned_support.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_13_q1_r19_r4_prefix_aligned_support.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r19_r4_support_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_13_q1_r19_r4_support_design.py"
    ),
]


def build_outputs():
    contract = yaml.safe_load(BASE.read(CONTRACT))
    upstream = contract["upstream"]
    for key, path in (
        ("R19_R3_assessment_sha256", RESOURCES[1]),
        ("R19_R3_freeze_sha256", RESOURCES[2]),
        ("R19_R3_review_sha256", RESOURCES[3]),
        ("R17_geometry_sha256", RESOURCES[4]),
        ("nonrecursive_adapter_sha256", RESOURCES[6]),
    ):
        BASE.require(
            BASE.sha(BASE.read(path)) == upstream[key],
            "{} differs".format(path),
        )
    rows = BASE.schedule_rows(contract)
    BASE.require(len(rows) == 40, "schedule row count")
    seeds = {row["seed"] for row in rows}
    BASE.require(seeds == set(range(7687, 7695)), "fresh seed roster")
    BASE.require(
        not seeds.intersection(set(range(7601, 7687))),
        "historical seed reused",
    )
    expected = set(contract["population"]["actions"])
    for block_index in range(1, 9):
        BASE.require(
            {
                row["forced_candidate_id"]
                for row in rows
                if row["block_index"] == block_index
            }
            == expected,
            "block support differs",
        )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R4",
        "status": "exact_fresh_prefix_aligned_support_schedule",
        "row_count": 40,
        "matched_context_blocks": 8,
        "rows": rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R4",
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
        "stage": "V3-13-Q1-R19-R4",
        "status": "unique_fresh_prefix_aligned_support_release",
        "contract_sha256": BASE.sha(BASE.read(CONTRACT)),
        "schedule_sha256": BASE.sha(schedule_payload),
        "dependency_freeze_sha256": BASE.sha(dependency_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/"
            "v3_13_q1_r19_r4_prefix_aligned_support"
        ),
        "authorized_units": 40,
        "authorized_seed_range": [7687, 7694],
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "execution_authorized": True,
        "supervised_new_head_fit_authorized": False,
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
