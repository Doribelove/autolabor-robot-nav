#!/usr/bin/env python3
"""Independent replay of the terminal Q1-R6 support result."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ASSESSOR = W / (
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r6_recovery_support.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r6_assessment/"
    "independent_result_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r6_assessor", ASSESSOR
    )
    assessor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assessor)
    outputs = assessor.build_outputs()
    for path, payload in outputs.items():
        assessor.require(
            assessor.read(path) == payload,
            "assessment replay differs: {}".format(path),
        )
    assessment = yaml.safe_load(outputs[assessor.ASSESSMENT])
    passed = assessment["collection_gate_pass"]
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R6",
        "status": (
            "independent_five_action_support_result_pass"
            if passed
            else "independent_five_action_support_result_fail"
        ),
        "assessment_sha256": hashlib.sha256(
            outputs[assessor.ASSESSMENT]
        ).hexdigest(),
        "evidence_freeze_sha256": hashlib.sha256(
            outputs[assessor.FREEZE]
        ).hexdigest(),
        "journal_raw_hash_and_schedule_replayed": True,
        "five_action_positive_propensity_replayed": True,
        "matching_and_finite_trace_gates_replayed": True,
        "unsafe_outcomes_retained": True,
        "new_candidate_assertive_head_fit_authorized": passed,
        "all_other_S3_heads_must_remain_byte_equivalent": True,
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
        raise RuntimeError("independent review exists")
    REVIEW.parent.mkdir(parents=True, exist_ok=True)
    REVIEW.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
