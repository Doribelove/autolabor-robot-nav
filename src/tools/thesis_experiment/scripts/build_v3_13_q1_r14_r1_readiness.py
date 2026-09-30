#!/usr/bin/env python3
"""Build the immutable fresh 40-unit R14-R1 repair release."""

import argparse
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r14_readiness.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r14_frozen_builder", PARENT_PATH
)
PARENT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PARENT)
BASE = PARENT.BASE
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r14_r1_progress_predictor_qualification_contract.yaml"
)
ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r14_r1_readiness"
)
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
REPAIR_FREEZE = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r14_terminal_assessment_failure.yaml"
)
REPAIR_REVIEW = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r14_terminal_assessment_failure_review.yaml"
)
RESOURCES = list(PARENT.RESOURCES) + [
    REPAIR_FREEZE,
    REPAIR_REVIEW,
    CONTRACT,
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_13_q1_r8_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_13_q1_r8_recovery_qualification.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_13_q1_r14_r1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r14_r1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r14_r1_progress_qualification.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_13_q1_r14_r1_progress_qualification.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r14_r1_qualification_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_13_q1_r14_r1_qualification_design.py"
    ),
]


def build_outputs():
    contract = yaml.safe_load(BASE.read(CONTRACT))
    repair = contract["repair_scope"]
    BASE.require(
        BASE.sha(BASE.read(REPAIR_FREEZE))
        == repair["terminal_failure_freeze_sha256"],
        "terminal failure freeze differs",
    )
    BASE.require(
        BASE.sha(BASE.read(REPAIR_REVIEW))
        == repair["terminal_failure_review_sha256"],
        "terminal failure review differs",
    )
    upstream = contract["upstream"]
    for key, path in (
        ("R13_fit_contract_sha256", PARENT.RESOURCES[1]),
        ("geometry_contract_sha256", PARENT.RESOURCES[2]),
        ("predictor_sha256", PARENT.RESOURCES[4]),
        ("internal_CV_sha256", PARENT.RESOURCES[5]),
        ("R13_manifest_sha256", PARENT.RESOURCES[6]),
        ("R13_review_sha256", PARENT.RESOURCES[7]),
    ):
        BASE.require(
            BASE.sha(BASE.read(path)) == upstream[key],
            "{} differs".format(path),
        )
    rows = PARENT.schedule_rows(contract)
    BASE.require(len(rows) == 40, "schedule row count")
    seeds = {row["seed"] for row in rows}
    BASE.require(seeds == set(range(7658, 7666)), "seed roster")
    BASE.require(
        not seeds.intersection(set(range(7601, 7658))),
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
        "stage": "V3-13-Q1-R14-R1",
        "status": "exact_fresh_stage_repair_schedule_frozen",
        "row_count": 40,
        "matched_context_blocks": 8,
        "rows": rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R14-R1",
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
        "stage": "V3-13-Q1-R14-R1",
        "status": "unique_fresh_stage_repair_qualification_release",
        "contract_sha256": BASE.sha(BASE.read(CONTRACT)),
        "predictor_sha256": upstream["predictor_sha256"],
        "schedule_sha256": BASE.sha(schedule_payload),
        "dependency_freeze_sha256": BASE.sha(dependency_payload),
        "canonical_output_root": (
            "artifacts/v3/multiactor/"
            "v3_13_q1_r14_r1_progress_qualification"
        ),
        "authorized_units": 40,
        "authorized_seed_range": [7658, 7665],
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
        "only_repaired_factor": (
            "released_summary_stage_and_status_compatibility"
        ),
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
