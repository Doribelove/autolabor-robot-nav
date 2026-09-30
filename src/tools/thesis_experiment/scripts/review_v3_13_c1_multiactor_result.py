#!/usr/bin/env python3
"""Independent replay of the V3-13-C1 clone assessment."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
ASSESSOR_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "assess_v3_13_c1_multiactor_clones.py"
)
REVIEW_PATH = (
    W
    / "artifacts/v3/multiactor/v3_13_c1_assessment/"
    "independent_result_review.yaml"
)


def load_assessor():
    spec = importlib.util.spec_from_file_location(
        "v3_13_c1_result_assessor", ASSESSOR_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    assessor = load_assessor()
    outputs = assessor.build_outputs()
    for relative, payload in outputs.items():
        assessor.require(
            assessor.read(relative) == payload,
            "assessment replay differs: {}".format(relative),
        )
    assessment = assessor.load_yaml(assessor.ASSESSMENT)
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C1-ASSESSMENT",
        "status": "v3_13_c1_independent_clone_result_review_pass",
        "assessment_sha256": assessor.sha(
            assessor.read(assessor.ASSESSMENT)
        ),
        "evidence_freeze_sha256": assessor.sha(
            assessor.read(assessor.FREEZE)
        ),
        "all_64_reports_replayed": True,
        "all_receipt_hashes_replayed": True,
        "journal_schedule_and_summary_replayed": True,
        "complete_positive_action_support_replayed": True,
        "action_conditioned_actor_response_observed": (
            assessment["actor_endpoint_spread_positive_rate"] > 0.0
        ),
        "predictor_fit_authorized": assessment[
            "predictor_fit_authorized"
        ],
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "performance_claim_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = assessor.yaml_bytes(value)
    assessor.require(not REVIEW_PATH.exists(), "result review exists")
    REVIEW_PATH.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()

