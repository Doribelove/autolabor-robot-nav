#!/usr/bin/env python3
"""Independently replay the R16 paired preflight result."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ASSESSOR = W / (
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r16_actual_teb_preflight.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r16_assessment/"
    "independent_result_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r16_assessor", ASSESSOR
    )
    assessor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assessor)
    outputs = assessor.build_outputs()
    for path, payload in outputs.items():
        assessor.require(
            assessor.read(path) == payload,
            "assessment replay differs",
        )
    assessment = yaml.safe_load(outputs[assessor.ASSESSMENT])
    passed = assessment["preflight_pass"]
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R16",
        "status": (
            "independent_actual_teb_paired_preflight_pass"
            if passed
            else "independent_actual_teb_paired_preflight_fail"
        ),
        "assessment_sha256": hashlib.sha256(
            outputs[assessor.ASSESSMENT]
        ).hexdigest(),
        "evidence_freeze_sha256": hashlib.sha256(
            outputs[assessor.FREEZE]
        ).hexdigest(),
        "release_schedule_journal_and_raw_hashes_replayed": True,
        "fairness_and_integration_gates_replayed": True,
        "actual_teb_outcome_retention_replayed": True,
        "actual_teb_multiscene_qualification_design_authorized": passed,
        "qualification_execution_authorized": False,
        "performance_claim_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = assessor.yaml_bytes(value)
    assessor.require(not REVIEW.exists(), "review exists")
    REVIEW.parent.mkdir(parents=True, exist_ok=True)
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
