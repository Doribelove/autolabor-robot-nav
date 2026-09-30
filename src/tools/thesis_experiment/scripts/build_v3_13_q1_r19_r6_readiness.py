#!/usr/bin/env python3
"""Build immutable 40-unit R19-R6 support release."""

import argparse
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r6_readiness.py"
)
SPEC = importlib.util.spec_from_file_location("r19_r6_builder_base", BASE_PATH)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r6_tolerance_repaired_support_contract.yaml"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_q1_r19_r6_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
RESOURCES = [
    CONTRACT,
    Path(
        "artifacts/v3/multiactor/"
        "v3_13_q1_r19_r5_r1_assessment.yaml"
    ),
    Path(
        "artifacts/v3/multiactor/"
        "v3_13_q1_r19_r5_r1_assessment_review.yaml"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "goal_aligned_execution_adapter_r2.py"
    ),
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_tolerance_repaired_prefix_aligned_bank_clone_episode.py"
    ),
    Path(
        "src/simulation/m2_gazebo/launch/"
        "m2_v3_13_tolerance_repaired_prefix_aligned_bank_clone_episode.launch"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r19_r6_tolerance_repaired_support.py"
    ),
    Path("devel/lib/libv3_reactive_actor_plugin.so"),
    Path("devel/lib/libm2_ackermann_plugin.so"),
]


def build_outputs():
    contract = yaml.safe_load(BASE.read(CONTRACT))
    upstream = contract["upstream"]
    for key, path in (
        ("R19_R5_R1_assessment_sha256", RESOURCES[1]),
        ("R19_R5_R1_review_sha256", RESOURCES[2]),
    ):
        BASE.require(
            BASE.sha(BASE.read(path)) == upstream[key],
            "{} differs".format(path),
        )
    rows = BASE.schedule_rows(contract)
    BASE.require(len(rows) == 40, "schedule row count")
    seeds = {row["seed"] for row in rows}
    BASE.require(seeds == set(range(7697, 7705)), "fresh seeds differ")
    expected = set(contract["population"]["actions"])
    for block_index in range(1, 9):
        BASE.require(
            {
                row["forced_candidate_id"] for row in rows
                if row["block_index"] == block_index
            } == expected,
            "support differs",
        )
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R6",
        "status": "exact_fresh_tolerance_repaired_support_schedule",
        "row_count": 40,
        "matched_context_blocks": 8,
        "rows": rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R6",
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
        "stage": "V3-13-Q1-R19-R6",
        "status": "unique_fresh_tolerance_repaired_support_release",
        "contract_sha256": BASE.sha(BASE.read(CONTRACT)),
        "schedule_sha256": BASE.sha(schedule_payload),
        "dependency_freeze_sha256": BASE.sha(dependency_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/"
            "v3_13_q1_r19_r6_tolerance_repaired_support"
        ),
        "authorized_units": 40,
        "authorized_seed_range": [7697, 7704],
        "attempts_per_identity": 1,
        "execution_authorized": True,
        "supervised_new_head_fit_authorized": False,
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
