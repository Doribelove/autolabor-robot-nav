#!/usr/bin/env python3
"""Build the four-unit non-evidence V3-13-S2 preflight release."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_s2_synchronized_sequential_clone_contract.yaml"
)
BUILDER = Path(
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_s2_preflight_readiness.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_s2_preflight_readiness.py"
)
RUNNER = Path(
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_s2_preflight.py"
)
RESULT_REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_s2_preflight_result.py"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_s2_preflight_readiness"
)
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "preflight_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"


RESOURCES = [
    CONTRACT,
    BUILDER,
    REVIEWER,
    RUNNER,
    RESULT_REVIEWER,
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_13_s2_design.py"
    ),
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
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_sequential_clone_episode_v2.py"
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
        "artifacts/v3/multiactor/v3_13_s1_failure/"
        "failure_attribution.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_s1_failure/"
        "terminal_evidence_freeze.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_s1_failure/"
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
    authority = contract["authority"]
    require(
        authority["exact_non_evidence_preflight_authorized"],
        "preflight not authorized",
    )
    require(
        int(authority["authorized_preflight_units"]) == 4,
        "preflight unit count",
    )
    require(
        not authority["evidence_execution_authorized"],
        "evidence unexpectedly authorized",
    )
    require(not authority["training_authorized"], "training boundary")
    failure = load_yaml(
        Path(
            "artifacts/v3/multiactor/v3_13_s1_failure/"
            "failure_attribution.yaml"
        )
    )
    require(
        failure["status"]
        == "terminal_sequential_collection_integrity_gate_fail",
        "S1 failure not frozen",
    )
    dependency_rows = [
        {"path": str(path), "sha256": sha(read(path))}
        for path in RESOURCES
    ]
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S2-PREFLIGHT",
        "status": "closed_preflight_dependency_freeze",
        "resource_count": len(dependency_rows),
        "resources": dependency_rows,
        "credential_environment_scrub_required": True,
    }
    dependencies_payload = yaml_bytes(dependencies)
    preflight = contract["integration_preflight"]
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S2-PREFLIGHT",
        "status": "unique_non_evidence_preflight_release",
        "contract_sha256": sha(read(CONTRACT)),
        "dependency_freeze_sha256": sha(dependencies_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/v3_13_s2_preflight"
        ),
        "authorized_units": 4,
        "authorized_seed": int(preflight["seed"]),
        "authorized_branch_target_sim_s": float(
            preflight["branch_target_sim_s"]
        ),
        "attempts_per_identity": 1,
        "retry_resume_or_replacement_authorized": False,
        "execution_authorized": True,
        "evidence_identity": False,
        "predictor_fit_or_claim_use_authorized": False,
        "policy_or_ranker_training_authorized": False,
        "hardware_device_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    return {
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
        "status=v3_13_s2_preflight_readiness_{} "
        "release_sha256={} manifest_sha256={}".format(
            "reproducible" if arguments.check else "built",
            sha(read(RELEASE)),
            sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
