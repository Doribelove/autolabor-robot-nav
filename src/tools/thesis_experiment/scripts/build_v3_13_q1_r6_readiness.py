#!/usr/bin/env python3
"""Build the immutable 40-unit V3-13-Q1-R6 support release."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r6_recovery_bank_support_contract.yaml"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r6_readiness"
)
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
RESOURCES = [
    CONTRACT,
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_q1_r4_r1_recovery_candidate_geometry_contract.yaml"
    ),
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_d0_multiactor_foundation_contract.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r5_assessment/"
        "preflight_assessment.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r5_assessment/"
        "evidence_freeze.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r5_assessment/"
        "independent_result_review.yaml"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "recovery_candidate_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "multi_actor_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "execution_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_recovery_bank_clone_episode.py"
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
        "m2_v3_13_recovery_bank_clone_episode.launch"
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
        "build_v3_13_q1_r6_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r6_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r6_recovery_support.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_13_q1_r6_recovery_support.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r6_support_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_13_q1_r6_support_design.py"
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
        require(target.is_file(), "output absent: {}".format(path))
        require(target.read_bytes() == payload, "output drift: {}".format(path))
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    require(not target.exists(), "refusing overwrite: {}".format(path))
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(target))


def schedule_rows(contract):
    population = contract["population"]
    strata = population["scene_policy_strata"]
    seeds = population["seeds"]
    actions = population["actions"]
    rows = []
    sequence = 0
    for block_index, seed in enumerate(seeds, start=1):
        stratum = strata[(block_index - 1) % len(strata)]
        ordered = list(actions)
        shift = (block_index - 1) % len(ordered)
        ordered = ordered[shift:] + ordered[:shift]
        if block_index % 2 == 0:
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
    contract = yaml.safe_load(read(CONTRACT))
    upstream = contract["upstream"]
    for key, path in (
        ("geometry_contract_sha256", RESOURCES[1]),
        ("r5_assessment_sha256", RESOURCES[3]),
        ("r5_freeze_sha256", RESOURCES[4]),
        ("r5_review_sha256", RESOURCES[5]),
    ):
        require(
            sha(read(path)) == upstream[key],
            "{} differs".format(path),
        )
    rows = schedule_rows(contract)
    require(len(rows) == 40, "schedule row count")
    require(
        {row["seed"] for row in rows} == set(range(7617, 7625)),
        "fresh seed roster",
    )
    require(
        not {row["seed"] for row in rows}.intersection(
            set(range(7601, 7617))
        ),
        "C1 seed reused",
    )
    for block_index in range(1, 9):
        block = [
            row for row in rows if row["block_index"] == block_index
        ]
        require(
            {row["forced_candidate_id"] for row in block}
            == set(contract["population"]["actions"]),
            "block support differs",
        )
    require(
        contract["authority"][
            "exact_bounded_simulation_support_authorized"
        ]
        is True,
        "simulation authority absent",
    )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R6",
        "status": "exact_fresh_five_action_support_schedule_frozen",
        "row_count": 40,
        "matched_context_blocks": 8,
        "rows": rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R6",
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
        "stage": "V3-13-Q1-R6",
        "status": "unique_fresh_five_action_support_release",
        "contract_sha256": sha(read(CONTRACT)),
        "schedule_sha256": sha(schedule_payload),
        "dependency_freeze_sha256": sha(dependency_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/"
            "v3_13_q1_r6_recovery_support"
        ),
        "authorized_units": 40,
        "authorized_seed_range": [7617, 7624],
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
