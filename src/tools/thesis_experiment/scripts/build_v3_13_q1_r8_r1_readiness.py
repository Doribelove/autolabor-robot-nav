#!/usr/bin/env python3
"""Build the immutable R8-R1 repaired qualification release."""

import argparse
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r8_readiness.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r8_frozen_builder", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r8_r1_runner_repair_contract.yaml"
)
ROOT = Path("artifacts/v3/multiactor/v3_13_q1_r8_r1_readiness")
SCHEDULE = ROOT / "exact_schedule.yaml"
DEPENDENCIES = ROOT / "dependency_freeze.yaml"
RELEASE = ROOT / "execution_release.yaml"
MANIFEST = ROOT / "evidence_manifest.sha256"
FROZEN_READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r8_readiness"
)
FROZEN_EXECUTION = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r8_recovery_qualification"
)
RESOURCES = BASE.RESOURCES + [
    FROZEN_READINESS / "exact_schedule.yaml",
    FROZEN_READINESS / "dependency_freeze.yaml",
    FROZEN_READINESS / "execution_release.yaml",
    FROZEN_READINESS / "evidence_manifest.sha256",
    FROZEN_READINESS / "independent_review.yaml",
    CONTRACT,
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r8_r1_recovery_qualification.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_13_q1_r8_r1_recovery_qualification.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r8_r1_qualification_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_13_q1_r8_r1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_13_q1_r8_r1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_13_q1_r8_r1_runner_repair.py"
    ),
]


def build_outputs():
    contract = yaml.safe_load(BASE.read(CONTRACT))
    repair = contract["repair"]
    BASE.require(
        BASE.sha(BASE.read(FROZEN_READINESS / "execution_release.yaml"))
        == repair["frozen_failed_release_sha256"],
        "frozen release differs",
    )
    BASE.require(
        BASE.sha(BASE.read(FROZEN_READINESS / "independent_review.yaml"))
        == repair["frozen_failed_review_sha256"],
        "frozen review differs",
    )
    BASE.require(
        not FROZEN_EXECUTION.exists(),
        "failed R8 release was executed; seed reauthorization forbidden",
    )
    rows = BASE.schedule_rows(contract)
    BASE.require(len(rows) == 40, "schedule row count")
    frozen_rows = yaml.safe_load(
        BASE.read(FROZEN_READINESS / "exact_schedule.yaml")
    )["rows"]
    BASE.require(rows == frozen_rows, "semantic schedule changed")
    schedule = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R8-R1",
        "status": "exact_unconsumed_r8_schedule_reauthorized",
        "row_count": 40,
        "matched_context_blocks": 8,
        "rows": rows,
        "attempts_per_identity": 1,
        "retry_resume_replacement_or_selective_exclusion_authorized": False,
    }
    dependencies = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R8-R1",
        "status": "closed_repair_dependency_freeze",
        "resource_count": len(RESOURCES),
        "resources": [
            {"path": str(path), "sha256": BASE.sha(BASE.read(path))}
            for path in RESOURCES
        ],
        "frozen_failed_release_execute_invoked": False,
        "runner_arity_repair_only": True,
        "semantic_gate_or_model_change_count": 0,
    }
    schedule_payload = BASE.yaml_bytes(schedule)
    dependency_payload = BASE.yaml_bytes(dependencies)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R8-R1",
        "status": "unique_repaired_expanded_predictor_qualification_release",
        "contract_sha256": BASE.sha(BASE.read(CONTRACT)),
        "predictor_sha256": contract["upstream"]["predictor_sha256"],
        "schedule_sha256": BASE.sha(schedule_payload),
        "dependency_freeze_sha256": BASE.sha(dependency_payload),
        "frozen_failed_release_sha256": repair[
            "frozen_failed_release_sha256"
        ],
        "canonical_output_root": (
            "artifacts/v3/multiactor/"
            "v3_13_q1_r8_r1_recovery_qualification"
        ),
        "authorized_units": 40,
        "authorized_seed_range": [7625, 7632],
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
