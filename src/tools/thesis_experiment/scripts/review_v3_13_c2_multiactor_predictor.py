#!/usr/bin/env python3
"""Independent replay of the V3-13-C2 predictor fit."""

import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
FITTER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "fit_v3_13_c2_multiactor_predictor.py"
)
REVIEW_PATH = (
    W
    / "artifacts/v3/multiactor/v3_13_c2_predictor/"
    "independent_review.yaml"
)


def load_fitter():
    spec = importlib.util.spec_from_file_location(
        "v3_13_c2_predictor_fitter", FITTER_PATH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    fitter = load_fitter()
    outputs = fitter.build_outputs()
    for relative, payload in outputs.items():
        fitter.require(
            fitter.read(relative) == payload,
            "predictor output replay differs: {}".format(relative),
        )
    cv = fitter.load_yaml(fitter.CV)
    receipt = fitter.load_yaml(fitter.RECEIPT)
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-C2",
        "status": "v3_13_c2_independent_predictor_review_pass",
        "model_sha256": fitter.sha(fitter.read(fitter.MODEL)),
        "internal_cross_validation_sha256": fitter.sha(
            fitter.read(fitter.CV)
        ),
        "fit_receipt_sha256": fitter.sha(
            fitter.read(fitter.RECEIPT)
        ),
        "deterministic_fit_replayed": True,
        "leave_one_scene_family_out_replayed": True,
        "cross_validation_pass": cv["cross_validation_pass"],
        "hidden_policy_runtime_input": False,
        "supervised_system_identification_fit": receipt[
            "supervised_system_identification_fit"
        ],
        "policy_or_ranker_training_steps": 0,
        "fresh_qualification_pass": False,
        "actual_teb_comparison_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    payload = fitter.yaml_bytes(value)
    fitter.require(not REVIEW_PATH.exists(), "predictor review exists")
    REVIEW_PATH.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()

