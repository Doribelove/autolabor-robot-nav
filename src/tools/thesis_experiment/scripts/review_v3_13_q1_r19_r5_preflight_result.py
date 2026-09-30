#!/usr/bin/env python3
"""Independently replay the terminal R19-R5 preflight."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ASSESSOR = W / (
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r19_r5_terminal_tolerance_preflight.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r19_r5_assessment/"
    "independent_result_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r19_r5_assessor", ASSESSOR
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
    passed = assessment["all_preflight_gates_pass"]
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R5",
        "status": (
            "independent_fresh_terminal_tolerance_preflight_pass"
            if passed
            else "independent_fresh_terminal_tolerance_preflight_fail"
        ),
        "assessment_sha256": hashlib.sha256(
            outputs[assessor.ASSESSMENT]
        ).hexdigest(),
        "evidence_freeze_sha256": hashlib.sha256(
            outputs[assessor.FREEZE]
        ).hexdigest(),
        "raw_hashes_terminal_and_safety_gates_replayed": True,
        "fresh_tolerance_repaired_support_design_authorized": passed,
        "predictor_fit_authorized": False,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    if REVIEW.exists():
        raise RuntimeError("review exists")
    REVIEW.parent.mkdir(parents=True, exist_ok=True)
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
