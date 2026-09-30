#!/usr/bin/env python3
"""Build the exact V3-13-C1 execution release after preflight."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_c1_multiactor_clone_contract.yaml"
)
D0_CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
DESIGN = Path(
    "src/tools/thesis_experiment/scripts/v3_13_multiactor_design.py"
)
RUNNER = Path(
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_c1_multiactor_clones.py"
)
PREFLIGHT_RUNNER = Path(
    "src/tools/thesis_experiment/scripts/run_v3_13_c1_preflight.py"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/build_v3_13_c1_readiness.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/review_v3_13_c1_readiness.py"
)
TEST = Path(
    "src/tools/thesis_experiment/tests/test_v3_13_c1_design.py"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_c1_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
PREFLIGHT = Path("artifacts/v3/multiactor/v3_13_c1_preflight")


STATIC_RESOURCES = [
    CONTRACT,
    D0_CONTRACT,
    DESIGN,
    RUNNER,
    PREFLIGHT_RUNNER,
    BUILDER,
    REVIEWER,
    TEST,
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
        "v3_scope_planner/execution_core.py"
    ),
    Path(
        "src/simulation/m2_gazebo/launch/"
        "m2_v3_13_multiactor_clone_episode.launch"
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
        "artifacts/v3/multiactor/v3_13_d0_foundation/"
        "evidence_manifest.sha256"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_d0_foundation/"
        "independent_review.yaml"
    ),
    PREFLIGHT / "preflight_identity.yaml",
    PREFLIGHT / "attempt_journal.jsonl",
    PREFLIGHT / "preflight_result.yaml",
    PREFLIGHT / "episode/world.sdf",
    PREFLIGHT / "episode/scene_spec.yaml",
    PREFLIGHT / "episode/actor_specs.yaml",
    PREFLIGHT / "episode/episode_report.yaml",
    PREFLIGHT / "episode/matrix_receipt.yaml",
    PREFLIGHT / "episode/roslaunch.log",
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


def load_design():
    spec = importlib.util.spec_from_file_location(
        "v3_13_c1_readiness_design", W / DESIGN
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
    preflight = load_yaml(PREFLIGHT / "preflight_result.yaml")
    require(
        preflight["status"] == "integration_preflight_pass",
        "preflight did not pass",
    )
    require(preflight["evidence_units_created"] == 0, "preflight evidence leak")
    require(
        preflight[
            "result_cannot_enter_predictor_fit_or_performance_analysis"
        ],
        "preflight exclusion absent",
    )
    require(
        preflight["contract_sha256"] == sha(read(CONTRACT)),
        "preflight contract differs",
    )
    design = load_design()
    rows = design.build_schedule(contract)
    require(len(rows) == 64, "schedule row count differs")
    require(
        len({(row["seed"], row["forced_candidate_id"]) for row in rows})
        == 64,
        "schedule identity collision",
    )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C1",
        "status": "exact_schedule_frozen",
        "row_count": len(rows),
        "paired_blocks": 16,
        "rows": rows,
        "retry_resume_or_replacement_authorized": False,
    }
    dependency_rows = [
        {"path": str(path), "sha256": sha(read(path))}
        for path in STATIC_RESOURCES
    ]
    dependencies = {
        "schema_version": "1.0",
        "stage": "V3-13-C1",
        "status": "closed_dependency_freeze",
        "resource_count": len(dependency_rows),
        "resources": dependency_rows,
        "stable_workspace_path_in_resources": False,
        "credential_names_absent_from_preflight_log": all(
            token not in read(PREFLIGHT / "episode/roslaunch.log")
            for token in (
                b"OPENAI_API_KEY",
                b"GITHUB_TOKEN",
                b"GH_TOKEN",
                b"AWS_SECRET_ACCESS_KEY",
            )
        ),
    }
    require(
        dependencies["credential_names_absent_from_preflight_log"],
        "credential name present in preflight log",
    )
    schedule_payload = yaml_bytes(schedule)
    dependencies_payload = yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C1",
        "status": "unique_bounded_execution_release",
        "user_instruction_date": "2026-07-25",
        "contract_sha256": sha(read(CONTRACT)),
        "exact_schedule_sha256": sha(schedule_payload),
        "dependency_freeze_sha256": sha(dependencies_payload),
        "preflight_result_sha256": sha(
            read(PREFLIGHT / "preflight_result.yaml")
        ),
        "canonical_output_root": (
            "artifacts/v3/multiactor/v3_13_c1_execution"
        ),
        "authorized_units": 64,
        "authorized_seed_range": [7601, 7616],
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "execution_authorized": True,
        "training_authorized": False,
        "actual_teb_comparison_authorized": False,
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
    manifest_resources = STATIC_RESOURCES + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(sha(read(path)), path)
        for path in manifest_resources
    ).encode("utf-8")
    write_or_check(MANIFEST, manifest, arguments.check)
    print(
        "status=v3_13_c1_readiness_{} release_sha256={} manifest_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(read(RELEASE)),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()

