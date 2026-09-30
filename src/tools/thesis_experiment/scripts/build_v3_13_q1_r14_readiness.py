#!/usr/bin/env python3
"""Build the immutable 40-unit R14 qualification release."""

import argparse
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r8_readiness.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r8_frozen_builder", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r14_progress_predictor_qualification_contract.yaml"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_q1_r14_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
RESOURCES = [
    CONTRACT,
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_q1_r13_progress_predictor_fit_contract.yaml"
    ),
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_q1_r10_progress_recovery_geometry_contract.yaml"
    ),
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_d0_multiactor_foundation_contract.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r13_predictor/"
        "expanded_progress_predictor.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r13_predictor/"
        "internal_cross_validation.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r13_predictor/"
        "evidence_manifest.sha256"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r13_predictor/"
        "independent_review.yaml"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "progress_recovery_candidate_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "multi_actor_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "multi_actor_core_v1.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "execution_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_progress_predictor_qualification_episode.py"
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
        "m2_v3_13_progress_predictor_qualification.launch"
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
        "run_v3_13_q1_r12_progress_support.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_13_q1_r14_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r14_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r14_progress_qualification.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_13_q1_r14_progress_qualification.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r14_qualification_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_13_q1_r14_qualification_design.py"
    ),
]


def schedule_rows(contract):
    population = contract["qualification_population"]
    strata = population["scene_policy_strata"]
    actions = population["actions"]
    rows = []
    sequence = 0
    for block_index, seed in enumerate(
        population["seeds"], start=1
    ):
        stratum = strata[(block_index - 1) % len(strata)]
        ordered = list(actions)
        shift = (block_index + 2) % len(ordered)
        ordered = ordered[shift:] + ordered[:shift]
        if block_index % 2:
            ordered.reverse()
        for action in ordered:
            sequence += 1
            rows.append(
                {
                    "sequence": sequence,
                    "block_index": block_index,
                    "seed": int(seed),
                    "scene_id": stratum["scene_id"],
                    "actor_policies_evaluator_only": list(
                        stratum["actor_policies_evaluator_only"]
                    ),
                    "branch_target_sim_s": 12.5,
                    "pre_action_wait_s": 5.0,
                    "forced_candidate_id": action,
                    "behavior_propensity": 0.20,
                    "attempt": 1,
                }
            )
    return rows


def build_outputs():
    contract = yaml.safe_load(BASE.read(CONTRACT))
    upstream = contract["upstream"]
    for key, path in (
        ("R13_fit_contract_sha256", RESOURCES[1]),
        ("geometry_contract_sha256", RESOURCES[2]),
        ("predictor_sha256", RESOURCES[4]),
        ("internal_CV_sha256", RESOURCES[5]),
        ("R13_manifest_sha256", RESOURCES[6]),
        ("R13_review_sha256", RESOURCES[7]),
    ):
        BASE.require(
            BASE.sha(BASE.read(path)) == upstream[key],
            "{} differs".format(path),
        )
    rows = schedule_rows(contract)
    BASE.require(len(rows) == 40, "schedule row count")
    seeds = {row["seed"] for row in rows}
    BASE.require(seeds == set(range(7650, 7658)), "seed roster")
    BASE.require(
        not seeds.intersection(set(range(7601, 7650))),
        "historical seed reused",
    )
    expected = set(contract["qualification_population"]["actions"])
    for block_index in range(1, 9):
        BASE.require(
            {
                row["forced_candidate_id"] for row in rows
                if row["block_index"] == block_index
            }
            == expected,
            "block support differs",
        )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R14",
        "status": "exact_fresh_progress_qualification_schedule_frozen",
        "row_count": 40,
        "matched_context_blocks": 8,
        "rows": rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R14",
        "status": "closed_dependency_freeze",
        "resource_count": len(RESOURCES),
        "resources": [
            {"path": str(path), "sha256": BASE.sha(BASE.read(path))}
            for path in RESOURCES
        ],
    }
    schedule_payload = BASE.yaml_bytes(schedule)
    dependency_payload = BASE.yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R14",
        "status": "unique_fresh_progress_predictor_qualification_release",
        "contract_sha256": BASE.sha(BASE.read(CONTRACT)),
        "predictor_sha256": upstream["predictor_sha256"],
        "schedule_sha256": BASE.sha(schedule_payload),
        "dependency_freeze_sha256": BASE.sha(dependency_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/"
            "v3_13_q1_r14_progress_qualification"
        ),
        "authorized_units": 40,
        "authorized_seed_range": [7650, 7657],
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "execution_authorized": True,
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
