#!/usr/bin/env python3
"""Independent replay of Q1-R3 midpoint qualification."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ASSESSOR = W / (
    "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_q1_r3_midpoint_clones.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r3_assessment/"
    "independent_result_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r3_assessor", ASSESSOR
    )
    assessor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assessor)
    outputs = assessor.build_outputs()
    for path, payload in outputs.items():
        if (W / path).read_bytes() != payload:
            raise RuntimeError("assessment replay differs")
    assessment = yaml.safe_load(outputs[assessor.ASSESSMENT])
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R3-ASSESSMENT",
        "status": "midpoint_qualification_independent_review_pass",
        "assessment_sha256": hashlib.sha256(
            outputs[assessor.ASSESSMENT]
        ).hexdigest(),
        "evidence_freeze_sha256": hashlib.sha256(
            outputs[assessor.FREEZE]
        ).hexdigest(),
        "qualification_pass": assessment["qualification_pass"],
        "new_actual_TEB_preflight_design_authorized": assessment[
            "new_actual_TEB_preflight_design_authorized"
        ],
        "direct_actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = assessor.yaml_bytes(value)
    if REVIEW.exists():
        raise RuntimeError("review exists")
    REVIEW.write_bytes(payload)
    print(
        "status={} qualification_pass={} review_sha256={}".format(
            value["status"],
            value["qualification_pass"],
            hashlib.sha256(payload).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
