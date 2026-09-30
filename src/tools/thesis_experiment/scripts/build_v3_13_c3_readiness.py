#!/usr/bin/env python3
"""Build the V3-13-C3 fresh predictor qualification release."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_c3_predictor_qualification_contract.yaml"
)
SCENE_CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_c1_multiactor_clone_contract.yaml"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/build_v3_13_c3_readiness.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/review_v3_13_c3_readiness.py"
)
RUNNER = Path(
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_c3_predictor_qualification.py"
)
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_c3_predictor_qualification.py"
)
RESULT_REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_c3_predictor_result.py"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_c3_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"

RESOURCES = [
    CONTRACT,
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
        "m2_v3_13_multiactor_clone_episode.launch"
    ),
    Path("src/simulation/m2_gazebo/launch/spawn_m2.launch"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("devel/lib/libv3_reactive_actor_plugin.so"),
    Path("src/simulation/m2_gazebo/src/m2_ackermann_plugin.cpp"),
    Path("devel/lib/libm2_ackermann_plugin.so"),
    Path(
        "artifacts/v3/multiactor/v3_13_c2_predictor/"
        "candidate_conditioned_predictor.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_c2_predictor/"
        "internal_cross_validation.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_c2_predictor/"
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


def schedule_rows(contract, scene_contract):
    seeds = contract["qualification_population"]["seeds"]
    scenes = scene_contract["scene_population"]
    policies = scene_contract["actor_population"][
        "evaluator_policy_pairs"
    ]
    require(len(seeds) == len(scenes) * len(policies), "seed count")
    actions = contract["qualification_population"]["actions"]
    rows = []
    sequence = 0
    block_index = 0
    for scene in scenes:
        for pair in policies:
            seed = int(seeds[block_index])
            block_index += 1
            order = list(actions)
            if block_index % 2 == 0:
                order.reverse()
            for candidate_id in order:
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "block_index": block_index,
                        "seed": seed,
                        "scene_id": scene["scene_id"],
                        "actor_policies_evaluator_only": list(pair),
                        "method": "fresh_predictor_qualification_clone",
                        "forced_candidate_id": candidate_id,
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
    scene_contract = load_yaml(SCENE_CONTRACT)
    require(
        sha(read(SCENE_CONTRACT))
        == contract["upstream"]["scene_contract_sha256"],
        "scene contract differs",
    )
    predictor = Path(contract["upstream"]["predictor_path"])
    require(
        sha(read(predictor)) == contract["upstream"]["predictor_sha256"],
        "predictor differs",
    )
    rows = schedule_rows(contract, scene_contract)
    require(len(rows) == 64, "schedule row count")
    require(
        not ({row["seed"] for row in rows} & set(range(6801, 7131))),
        "old seed reused",
    )
    require(
        not ({row["seed"] for row in rows} & set(range(7601, 7617))),
        "C1 seed reused",
    )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C3",
        "status": "exact_fresh_qualification_schedule_frozen",
        "row_count": len(rows),
        "rows": rows,
    }
    dependencies = {
        "schema_version": "1.0",
        "stage": "V3-13-C3",
        "status": "closed_dependency_freeze",
        "resource_count": len(RESOURCES),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in RESOURCES
        ],
    }
    schedule_payload = yaml_bytes(schedule)
    dependencies_payload = yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C3",
        "status": "unique_fresh_predictor_qualification_release",
        "contract_sha256": sha(read(CONTRACT)),
        "schedule_sha256": sha(schedule_payload),
        "dependency_freeze_sha256": sha(dependencies_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/v3_13_c3_execution"
        ),
        "authorized_units": 64,
        "authorized_seed_range": [7201, 7216],
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "execution_authorized": True,
        "training_authorized": False,
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
        "status=v3_13_c3_readiness_{} release_sha256={} manifest_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(read(RELEASE)),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()

