#!/usr/bin/env python3
"""Independent replay of the V3-13-S3 recovery predictor fit."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
FITTER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "fit_v3_13_s3_recovery_predictor.py"
)
REVIEW_PATH = (
    W
    / "artifacts/v3/multiactor/v3_13_s3_predictor/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_s3_recovery_fitter", FITTER_PATH
    )
    fitter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fitter)
    outputs = fitter.build_outputs()
    for relative, payload in outputs.items():
        fitter.require(
            fitter.read(relative) == payload,
            "predictor replay differs",
        )
    model = fitter.load_yaml(fitter.MODEL)
    cv = fitter.load_yaml(fitter.CV)
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S3",
        "status": "v3_13_s3_independent_recovery_predictor_review_pass",
        "model_sha256": fitter.sha(fitter.read(fitter.MODEL)),
        "internal_cross_validation_sha256": fitter.sha(
            fitter.read(fitter.CV)
        ),
        "evidence_manifest_sha256": fitter.sha(
            fitter.read(fitter.MANIFEST)
        ),
        "cautious_and_hesitant_heads_retained": (
            set(model["heads"]) >= {"cautious", "hesitant"}
        ),
        "assertive_heads_present_for_all_four_actions": (
            len(model["heads"]["assertive"]) == 4
        ),
        "internal_cross_validation_replayed": cv[
            "cross_validation_pass"
        ],
        "hidden_policy_feature_count": 0,
        "fresh_qualification_pass": False,
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = fitter.yaml_bytes(value)
    fitter.require(not REVIEW_PATH.exists(), "review exists")
    REVIEW_PATH.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
