#!/usr/bin/env python3
"""Independently replay the R13 progress predictor fit."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
FITTER = W / (
    "src/tools/thesis_experiment/scripts/"
    "fit_v3_13_q1_r13_progress_predictor.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r13_predictor/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r13_fitter", FITTER
    )
    fitter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fitter)
    outputs = fitter.build_outputs()
    for path, payload in outputs.items():
        fitter.BASE.require(
            fitter.BASE.read(path) == payload,
            "fit replay differs",
        )
    model = yaml.safe_load(outputs[fitter.BASE.MODEL])
    cv = yaml.safe_load(outputs[fitter.BASE.CV])
    retained = fitter.BASE.load_yaml(fitter.BASE.S3_MODEL)
    candidate_id = "recovery_evasive_right_v120_r2"
    fitter.BASE.require(cv["cross_validation_pass"], "internal CV failed")
    fitter.BASE.require(
        all(
            model["heads"][mode][candidate] == head
            for mode, heads in retained["heads"].items()
            for candidate, head in heads.items()
        ),
        "retained R7 head differs",
    )
    fitter.BASE.require(
        set(model["heads"]["assertive"])
        == set(retained["heads"]["assertive"]) | {candidate_id},
        "assertive head roster differs",
    )
    manifest = "".join(
        "{}  {}\n".format(
            fitter.BASE.sha(fitter.BASE.read(path)), path
        )
        for path in fitter.resources(outputs)
    ).encode("utf-8")
    fitter.BASE.require(
        fitter.BASE.read(fitter.BASE.MANIFEST) == manifest,
        "manifest differs",
    )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R13",
        "status": "independent_progress_predictor_fit_review_pass",
        "model_sha256": hashlib.sha256(
            outputs[fitter.BASE.MODEL]
        ).hexdigest(),
        "internal_cross_validation_sha256": hashlib.sha256(
            outputs[fitter.BASE.CV]
        ).hexdigest(),
        "evidence_manifest_sha256": hashlib.sha256(
            manifest
        ).hexdigest(),
        "retained_R7_heads_byte_equivalent": True,
        "exactly_one_assertive_candidate_head_added": True,
        "internal_CV_replayed": True,
        "fresh_qualification_design_authorized": True,
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "policy_or_ranker_training_authorized": False,
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
