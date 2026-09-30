#!/usr/bin/env python3
"""Independently review V3-07-A1 result."""

import hashlib
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
ASSESSOR = W / "src/tools/thesis_experiment/scripts/assess_v3_07_a1_identifiability.py"


def load_assessor():
    spec = importlib.util.spec_from_file_location("v3_07_a1_assessor_review", ASSESSOR)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    a = load_assessor()
    m = a.load_module()
    value = a.assess(m)
    value.pop("_evidence")
    payload = m.yaml_bytes(value)
    m.require(
        (W / m.ASSESSMENT_RELATIVE).read_bytes() == payload,
        "A1 assessment replay differs",
    )
    passed = value["overall_pass"] is True
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-07-A1",
        "status": (
            "v3_07_a1_identifiability_result_review_pass"
            if passed
            else "v3_07_a1_identifiability_failure_review_pass"
        ),
        "assessment_sha256": hashlib.sha256(payload).hexdigest(),
        "evidence_freeze_sha256": m.sha256_bytes(
            m.read_file_once(W / m.EVIDENCE_FREEZE_RELATIVE)
        ),
        "qualified_for_spatial_predictor_data_design": passed,
        "formal_result": False,
        "simulation_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }
    review_payload = m.yaml_bytes(review)
    a.write_once(W / m.RESULT_REVIEW_RELATIVE, review_payload)
    terminal = (
        "\n".join(
            "{}  {}".format(
                m.sha256_bytes(m.read_file_once(W / path)), path
            )
            for path in [
                m.ASSESSMENT_RELATIVE,
                m.EVIDENCE_FREEZE_RELATIVE,
                m.RESULT_REVIEW_RELATIVE,
            ]
        )
        + "\n"
    ).encode()
    a.write_once(W / m.TERMINAL_FREEZE_RELATIVE, terminal)
    print(
        "status={} review_sha256={}".format(
            review["status"], hashlib.sha256(review_payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
