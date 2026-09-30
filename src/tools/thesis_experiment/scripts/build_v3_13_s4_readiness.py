#!/usr/bin/env python3
"""Build the fresh V3-13-S4 predictor qualification release."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_s4_recovery_predictor_qualification_contract.yaml"
)
S2_CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_s2_synchronized_sequential_clone_contract.yaml"
)
SCENE_CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_c1_multiactor_clone_contract.yaml"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/build_v3_13_s4_readiness.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/review_v3_13_s4_readiness.py"
)
RUNNER = Path(
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_s4_recovery_qualification.py"
)
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_s4_recovery_qualification.py"
)
RESULT_REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_s4_recovery_result.py"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_s4_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"


RESOURCES = [
    CONTRACT,
    S2_CONTRACT,
    SCENE_CONTRACT,
    BUILDER,
    REVIEWER,
    RUNNER,
    ASSESSOR,
    RESULT_REVIEWER,
    Path(
        "src/tools/thesis_experiment/scripts/"
        "v3_13_multiactor_design.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_c1_multiactor_clones.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_s2_sequential_clones.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_sequential_clone_episode_v2.py"
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
        "src/simulation/m2_gazebo/launch/"
        "m2_v3_13_sequential_clone_episode_v2.launch"
    ),
    Path("src/simulation/m2_gazebo/launch/spawn_m2.launch"),
    Path(
        "src/simulation/m2_gazebo/src/"
        "v3_reactive_actor_plugin.cpp"
    ),
    Path("devel/lib/libv3_reactive_actor_plugin.so"),
    Path("src/simulation/m2_gazebo/src/m2_ackermann_plugin.cpp"),
    Path("devel/lib/libm2_ackermann_plugin.so"),
    Path(
        "artifacts/v3/multiactor/v3_13_s3_predictor/"
        "candidate_conditioned_recovery_predictor.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_s3_predictor/"
        "internal_cross_validation.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_s3_predictor/"
        "independent_review.yaml"
    ),
]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def read(relative):
    path = W / relative
    require(path.is_file(), "required file absent: {}".format(relative))
    return path.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def load_yaml(relative):
    return yaml.safe_load(read(relative))


def schedule_rows(contract, s2_contract):
    population = contract["qualification_population"]
    target_times = s2_contract["repair"][
        "branch_target_sim_times_s"
    ]
    rows = []
    sequence = 0
    block_index = 0
    for stratum in population["scene_policy_strata"]:
        for label in population["branch_time_labels"]:
            seed = int(population["seeds"][block_index])
            block_index += 1
            actions = list(population["actions"])
            if block_index % 2 == 0:
                actions.reverse()
            for action in actions:
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "block_index": block_index,
                        "seed": seed,
                        "scene_id": stratum["scene_id"],
                        "actor_policies_evaluator_only": list(
                            stratum[
                                "actor_policies_evaluator_only"
                            ]
                        ),
                        "branch_time_label": label,
                        "pre_action_wait_s": (
                            1.0 if label == "early" else 5.0
                        ),
                        "branch_target_sim_s": float(
                            target_times[label]
                        ),
                        "forced_candidate_id": action,
                        "behavior_propensity": 0.25,
                        "attempt": 1,
                    }
                )
    return rows


def write_or_check(relative, payload, check):
    path = W / relative
    if check:
        require(path.is_file(), "output absent: {}".format(relative))
        require(path.read_bytes() == payload, "output drift: {}".format(relative))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    require(not path.exists(), "refusing overwrite: {}".format(relative))
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def build_outputs():
    contract = load_yaml(CONTRACT)
    s2_contract = load_yaml(S2_CONTRACT)
    predictor = Path(contract["upstream"]["predictor_path"])
    require(
        sha(read(predictor))
        == contract["upstream"]["predictor_sha256"],
        "predictor differs",
    )
    rows = schedule_rows(contract, s2_contract)
    require(len(rows) == 32, "schedule row count")
    seeds = {int(row["seed"]) for row in rows}
    require(seeds == set(range(7251, 7259)), "seed roster")
    require(
        not seeds.intersection(set(range(6801, 7249))),
        "historical V3 seed reused",
    )
    require(
        not seeds.intersection(set(range(7601, 7617))),
        "C1 seed reused",
    )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S4",
        "status": "exact_fresh_recovery_qualification_schedule_frozen",
        "row_count": len(rows),
        "matched_context_blocks": 8,
        "rows": rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S4",
        "status": "closed_dependency_freeze",
        "resource_count": len(RESOURCES),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in RESOURCES
        ],
        "credential_environment_scrub_required": True,
    }
    schedule_payload = yaml_bytes(schedule)
    dependencies_payload = yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S4",
        "status": "unique_fresh_recovery_qualification_release",
        "contract_sha256": sha(read(CONTRACT)),
        "predictor_sha256": sha(read(predictor)),
        "schedule_sha256": sha(schedule_payload),
        "dependency_freeze_sha256": sha(dependencies_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/v3_13_s4_execution"
        ),
        "authorized_units": 32,
        "authorized_seed_range": [7251, 7258],
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "execution_authorized": True,
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "hardware_device_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    return {
        SCHEDULE: schedule_payload,
        DEPENDENCIES: dependencies_payload,
        RELEASE: yaml_bytes(release),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    arguments = parser.parse_args()
    outputs = build_outputs()
    for relative, payload in outputs.items():
        write_or_check(relative, payload, arguments.check)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in RESOURCES + list(outputs)
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, arguments.check)
    print(
        "status=v3_13_s4_readiness_{} release_sha256={} "
        "manifest_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(read(RELEASE)),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
