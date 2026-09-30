#!/usr/bin/env python3
"""Independent replay of the V3-13-S2 synchronized assessment."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
ASSESSOR_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_s2_sequential_clones.py"
)
REVIEW_PATH = (
    W
    / "artifacts/v3/multiactor/v3_13_s2_assessment/"
    "independent_result_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s2_result_assessor", ASSESSOR_PATH
    )
    assessor = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(assessor)
    outputs = assessor.build_outputs()
    for relative, payload in outputs.items():
        assessor.require(
            assessor.read(relative) == payload,
            "assessment replay differs",
        )
    assessment = assessor.load_yaml(assessor.ASSESSMENT)
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S2-ASSESSMENT",
        "status": "v3_13_s2_independent_sequential_result_review_pass",
        "assessment_sha256": assessor.sha(
            assessor.read(assessor.ASSESSMENT)
        ),
        "evidence_freeze_sha256": assessor.sha(
            assessor.read(assessor.FREEZE)
        ),
        "all_32_attempt_one_reports_replayed": True,
        "clock_coordinate_speed_matching_replayed": True,
        "matched_post_stop_four_action_support_replayed": True,
        "assertive_recovery_predictor_fit_authorized": assessment[
            "assertive_recovery_predictor_fit_authorized"
        ],
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "performance_claim_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = assessor.yaml_bytes(value)
    assessor.require(not REVIEW_PATH.exists(), "review exists")
    REVIEW_PATH.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
