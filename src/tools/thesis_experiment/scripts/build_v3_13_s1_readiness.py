#!/usr/bin/env python3
"""Build the one-shot V3-13-S1 sequential-clone execution release."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_s1_sequential_clone_contract.yaml"
)
SCENE_CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_c1_multiactor_clone_contract.yaml"
)
FOUNDATION_BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_s1_foundation.py"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_s1_readiness.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_s1_readiness.py"
)
RUNNER = Path(
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_s1_sequential_clones.py"
)
ASSESSOR = Path(
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_s1_sequential_clones.py"
)
RESULT_REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_s1_sequential_result.py"
)
FOUNDATION = Path(
    "artifacts/v3/multiactor/v3_13_s1_foundation"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_s1_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"


RESOURCES = [
    CONTRACT,
    SCENE_CONTRACT,
    FOUNDATION_BUILDER,
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
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_sequential_clone_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/"
        "v3_scope_planner/multi_actor_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/"
        "v3_scope_planner/execution_core.py"
    ),
    Path(
        "src/simulation/m2_gazebo/launch/"
        "m2_v3_13_sequential_clone_episode.launch"
    ),
    Path("src/simulation/m2_gazebo/launch/spawn_m2.launch"),
    Path(
        "src/simulation/m2_gazebo/src/"
        "v3_reactive_actor_plugin.cpp"
    ),
    Path("devel/lib/libv3_reactive_actor_plugin.so"),
    Path("src/simulation/m2_gazebo/src/m2_ackermann_plugin.cpp"),
    Path("devel/lib/libm2_ackermann_plugin.so"),
    FOUNDATION / "future_exact_schedule.yaml",
    FOUNDATION / "offline_wait_state_readiness.yaml",
    FOUNDATION / "evidence_manifest.sha256",
    FOUNDATION / "independent_review.yaml",
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


def load_foundation_builder():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s1_release_foundation", W / FOUNDATION_BUILDER
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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
    foundation = load_foundation_builder()
    replayed = foundation.build_outputs()
    for relative, payload in replayed.items():
        require(read(relative) == payload, "foundation replay differs")
    foundation_review = load_yaml(FOUNDATION / "independent_review.yaml")
    require(
        foundation_review["status"]
        == "v3_13_s1_independent_offline_foundation_review_pass",
        "foundation review did not pass",
    )
    future_schedule = load_yaml(FOUNDATION / "future_exact_schedule.yaml")
    rows = future_schedule["rows"]
    require(len(rows) == 32, "schedule row count")
    require(
        len(
            {
                (
                    row["seed"],
                    row["pre_action_wait_s"],
                    row["forced_candidate_id"],
                )
                for row in rows
            }
        )
        == 32,
        "schedule identity collision",
    )
    seeds = {int(row["seed"]) for row in rows}
    require(seeds == set(range(7221, 7229)), "fresh seed roster differs")
    require(
        not seeds.intersection(set(range(6801, 7217))),
        "historical V3 seed reused",
    )
    require(7299 not in seeds, "C1 preflight seed reused")
    require(
        not seeds.intersection(set(range(7601, 7617))),
        "C1 evidence seed reused",
    )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S1",
        "status": "exact_sequential_clone_schedule_frozen",
        "row_count": len(rows),
        "matched_context_blocks": 8,
        "rows": rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependency_rows = [
        {"path": str(path), "sha256": sha(read(path))}
        for path in RESOURCES
    ]
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S1",
        "status": "closed_dependency_freeze",
        "resource_count": len(dependency_rows),
        "resources": dependency_rows,
        "stable_workspace_path_in_resources": False,
        "credential_environment_scrub_required": True,
    }
    schedule_payload = yaml_bytes(schedule)
    dependencies_payload = yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S1",
        "status": "unique_bounded_sequential_clone_execution_release",
        "user_continuation_instruction_date": "2026-07-25",
        "zero_authority_design_contract_sha256": sha(read(CONTRACT)),
        "foundation_manifest_sha256": sha(
            read(FOUNDATION / "evidence_manifest.sha256")
        ),
        "foundation_review_sha256": sha(
            read(FOUNDATION / "independent_review.yaml")
        ),
        "exact_schedule_sha256": sha(schedule_payload),
        "dependency_freeze_sha256": sha(dependencies_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/v3_13_s1_execution"
        ),
        "authorized_units": 32,
        "authorized_seed_range": [7221, 7228],
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "execution_authorized": True,
        "predictor_fit_authorized_after_collection_gate_only": True,
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "perception_claim_authorized": False,
        "hardware_device_access_authorized": False,
        "real_vehicle_authorized": False,
        "stable_workspace_write_authorized": False,
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
        "status=v3_13_s1_readiness_{} release_sha256={} "
        "manifest_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(read(RELEASE)),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
