#!/usr/bin/env python3
"""Independently replay the terminal R8-R2 qualification."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ASSESSOR = W / (
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r8_r2_recovery_qualification.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r8_r2_assessment/"
    "independent_result_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r8_r2_assessor", ASSESSOR
    )
    wrapper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(wrapper)
    assessor = wrapper.BASE
    outputs = assessor.build_outputs()
    for path, payload in outputs.items():
        assessor.require(
            assessor.read(path) == payload,
            "qualification replay differs: {}".format(path),
        )
    assessment = yaml.safe_load(outputs[assessor.ASSESSMENT])
    passed = assessment["qualification_pass"]
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R8-R2",
        "status": (
            "independent_expanded_predictor_qualification_pass"
            if passed
            else "independent_expanded_predictor_qualification_fail"
        ),
        "assessment_sha256": hashlib.sha256(
            outputs[assessor.ASSESSMENT]
        ).hexdigest(),
        "evidence_freeze_sha256": hashlib.sha256(
            outputs[assessor.FREEZE]
        ).hexdigest(),
        "raw_hash_schedule_and_journal_replayed": True,
        "fresh_seed_recovery_replayed": True,
        "matching_support_and_predictor_metrics_replayed": True,
        "expanded_recovery_planner_integration_design_authorized": passed,
        "direct_actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    if REVIEW.exists():
        raise RuntimeError("independent result review exists")
    REVIEW.parent.mkdir(parents=True, exist_ok=True)
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
