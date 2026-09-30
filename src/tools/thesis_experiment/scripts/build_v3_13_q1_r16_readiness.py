#!/usr/bin/env python3
"""Build the immutable two-unit R16 paired preflight release."""

import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r16_actual_teb_paired_preflight_contract.yaml"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r16_readiness"
)
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
RESOURCES = [
    CONTRACT,
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_q1_r15_progress_planner_integration_contract.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r15_planner_audit/"
        "progress_planner_audit.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r15_planner_audit/"
        "evidence_manifest.sha256"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r15_planner_audit/"
        "independent_review.yaml"
    ),
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_d0_multiactor_foundation_contract.yaml"
    ),
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_q1_r10_progress_recovery_geometry_contract.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/v3_13_q1_r13_predictor/"
        "expanded_progress_predictor.yaml"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "progress_expanded_recovery_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "progress_recovery_candidate_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "expanded_recovery_core.py"
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
        "v3_scope_progress_actual_teb_preflight_episode.py"
    ),
    Path(
        "devel/lib/v3_scope_planner/"
        "v3_scope_progress_actual_teb_preflight_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_actual_teb_fair_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_multiactor_clone_episode.py"
    ),
    Path(
        "src/simulation/m2_gazebo/launch/"
        "m2_v3_13_progress_actual_teb_preflight.launch"
    ),
    Path("src/simulation/m2_gazebo/launch/spawn_m2.launch"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("devel/lib/libv3_reactive_actor_plugin.so"),
    Path("src/simulation/m2_gazebo/src/m2_ackermann_plugin.cpp"),
    Path("devel/lib/libm2_ackermann_plugin.so"),
    Path(
        "src/navigation_arena/forks/navigation/local_planner/teb/"
        "src/teb_local_planner_ros.cpp"
    ),
    Path("devel/lib/libteb_local_planner.so"),
    Path("src/simulation/m2_gazebo/config/fixed_teb.yaml"),
    Path("src/simulation/m2_gazebo/config/move_base.yaml"),
    Path("src/simulation/m2_gazebo/config/costmap_common.yaml"),
    Path("src/simulation/m2_gazebo/config/local_costmap.yaml"),
    Path("src/simulation/m2_gazebo/config/global_costmap.yaml"),
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
        "run_v3_13_q1_actual_teb_matrix.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_13_q1_r16_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r16_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r16_actual_teb_preflight.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_13_q1_r16_actual_teb_preflight.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r16_actual_teb_preflight_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_13_q1_r16_actual_teb_preflight.py"
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
        require(target.is_file(), "output absent")
        require(target.read_bytes() == payload, "output drift")
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    require(not target.exists(), "refusing overwrite")
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_bytes(payload)
    os.replace(str(temporary), str(target))


def rows(contract):
    population = contract["preflight_population"]
    return [
        {
            "sequence": sequence,
            "block_index": 1,
            "seed": int(population["seed"]),
            "scene_id": population["scene_id"],
            "actor_policies_evaluator_only": list(
                population["actor_policies_evaluator_only"]
            ),
            "method": method,
            "attempt": 1,
        }
        for sequence, method in enumerate(
            population["method_order"], start=1
        )
    ]


def build_outputs():
    contract = yaml.safe_load(read(CONTRACT))
    upstream = contract["upstream"]
    for key, path in (
        ("R15_contract_sha256", RESOURCES[1]),
        ("R15_audit_sha256", RESOURCES[2]),
        ("R15_manifest_sha256", RESOURCES[3]),
        ("R15_review_sha256", RESOURCES[4]),
        ("D0_contract_sha256", RESOURCES[5]),
        ("geometry_contract_sha256", RESOURCES[6]),
        ("predictor_sha256", RESOURCES[7]),
    ):
        require(sha(read(path)) == upstream[key], "{} differs".format(path))
    baseline = contract["methods"]["primary_baseline"]
    for key, path in (
        ("config_sha256", Path(baseline["config_path"])),
        ("built_plugin_sha256", Path(baseline["built_plugin_path"])),
        ("source_sha256", Path(baseline["source_path"])),
    ):
        require(
            sha(read(path)) == baseline[key],
            "actual TEB identity differs: {}".format(path),
        )
    schedule_rows = rows(contract)
    require(len(schedule_rows) == 2, "schedule row count")
    require(
        [row["method"] for row in schedule_rows]
        == ["actual_teb", "v3_13_analytic"],
        "method order differs",
    )
    require(
        {row["seed"] for row in schedule_rows} == {7666},
        "seed differs",
    )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R16",
        "status": "exact_fresh_actual_teb_paired_preflight_schedule",
        "row_count": 2,
        "paired_blocks": 1,
        "rows": schedule_rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R16",
        "status": "closed_dependency_freeze",
        "resource_count": len(RESOURCES),
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in RESOURCES
        ],
    }
    schedule_payload = yaml_bytes(schedule)
    dependency_payload = yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R16",
        "status": "unique_fresh_actual_teb_paired_preflight_release",
        "contract_sha256": sha(read(CONTRACT)),
        "schedule_sha256": sha(schedule_payload),
        "dependency_freeze_sha256": sha(dependency_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/"
            "v3_13_q1_r16_actual_teb_preflight"
        ),
        "authorized_units": 2,
        "authorized_seed": 7666,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "execution_authorized": True,
        "qualification_execution_authorized": False,
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
