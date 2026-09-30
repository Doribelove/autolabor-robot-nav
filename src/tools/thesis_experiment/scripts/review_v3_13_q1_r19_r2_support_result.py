#!/usr/bin/env python3
"""Independently replay terminal R19-R2 support."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ASSESSOR = W / (
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r19_r2_boundary_atomic_support.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r19_r2_assessment/"
    "independent_result_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r19_r2_assessor", ASSESSOR
    )
    assessor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assessor)
    outputs = assessor.build_outputs()
    for path, payload in outputs.items():
        assessor.BASE.require(
            assessor.BASE.read(path) == payload,
            "support replay differs",
        )
    assessment = yaml.safe_load(
        outputs[assessor.BASE.ASSESSMENT]
    )
    passed = assessment["collection_gate_pass"]
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R2",
        "status": (
            "independent_boundary_atomic_support_assessment_pass"
            if passed
            else "independent_boundary_atomic_support_assessment_fail"
        ),
        "assessment_sha256": hashlib.sha256(
            outputs[assessor.BASE.ASSESSMENT]
        ).hexdigest(),
        "evidence_freeze_sha256": hashlib.sha256(
            outputs[assessor.BASE.FREEZE]
        ).hexdigest(),
        "raw_hash_matching_guard_and_boundary_gates_replayed": True,
        "r3_assertive_head_fit_authorized": passed,
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
