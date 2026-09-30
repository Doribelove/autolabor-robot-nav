#!/usr/bin/env python3
"""Build the immutable 16-unit Q1-R3 midpoint clone release."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r3_midpoint_clone_contract.yaml"
)
D0 = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r3_readiness"
)
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
RESOURCES = [
    CONTRACT,
    D0,
    Path(
        "artifacts/v3/multiactor/v3_13_s3_predictor/"
        "candidate_conditioned_recovery_predictor.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r2_diagnostic/"
        "offline_root_cause.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r2_diagnostic/"
        "evidence_freeze.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r2_diagnostic/"
        "independent_result_review.yaml"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_13_q1_r3_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r3_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r3_midpoint_clones.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_13_q1_r3_midpoint_clones.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r3_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r1_actual_teb_matrix.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "v3_13_multiactor_design.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_midpoint_clone_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_clone_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/"
        "v3_scope_planner/multi_actor_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/"
        "v3_scope_planner/multi_actor_core_v1.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/"
        "v3_scope_planner/execution_core.py"
    ),
    Path(
        "src/simulation/m2_gazebo/launch/"
        "m2_v3_13_midpoint_clone_episode.launch"
    ),
    Path("src/simulation/m2_gazebo/launch/spawn_m2.launch"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("devel/lib/libv3_reactive_actor_plugin.so"),
    Path("src/simulation/m2_gazebo/src/m2_ackermann_plugin.cpp"),
    Path("devel/lib/libm2_ackermann_plugin.so"),
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
        require(target.read_bytes() == payload, "output drift")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    require(not target.exists(), "refusing overwrite")
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(target))


def schedule_rows(contract):
    rows = []
    sequence = 0
    for block_index, (stratum, seed) in enumerate(
        zip(
            contract["population"]["scene_policy_strata"],
            contract["population"]["seeds"],
        ),
        start=1,
    ):
        actions = list(contract["population"]["actions"])
        if block_index % 2 == 0:
            actions.reverse()
        for action in actions:
            sequence += 1
            rows.append(
                {
                    "sequence": sequence,
                    "block_index": block_index,
                    "seed": int(seed),
                    "scene_id": stratum["scene_id"],
                    "actor_policies_evaluator_only": list(
                        stratum[
                            "actor_policies_evaluator_only"
                        ]
                    ),
                    "branch_target_sim_s": 10.5,
                    "forced_candidate_id": action,
                    "behavior_propensity": 0.25,
                    "attempt": 1,
                }
            )
    return rows


def build_outputs():
    contract = yaml.safe_load(read(CONTRACT))
    require(
        sha(
            read(
                Path(
                    "artifacts/v3/multiactor/"
                    "v3_13_q1_r2_diagnostic/offline_root_cause.yaml"
                )
            )
        )
        == contract["upstream"]["diagnostic_analysis_sha256"],
        "diagnostic analysis differs",
    )
    rows = schedule_rows(contract)
    require(len(rows) == 16, "row count")
    require(
        {row["seed"] for row in rows} == set(range(7332, 7336)),
        "seed roster",
    )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R3",
        "status": "exact_midpoint_clone_schedule_frozen",
        "row_count": 16,
        "matched_context_blocks": 4,
        "rows": rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R3",
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
        "stage": "V3-13-Q1-R3",
        "status": "unique_fresh_midpoint_clone_release",
        "contract_sha256": sha(read(CONTRACT)),
        "schedule_sha256": sha(schedule_payload),
        "dependency_freeze_sha256": sha(dependency_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/v3_13_q1_r3_execution"
        ),
        "authorized_units": 16,
        "authorized_seed_range": [7332, 7335],
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
        RELEASE: yaml_bytes(release),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        write_or_check(path, payload, arguments.check)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in RESOURCES + list(outputs)
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, arguments.check)
    print(
        "status=v3_13_q1_r3_readiness_{} release_sha256={} "
        "manifest_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(read(RELEASE)),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
