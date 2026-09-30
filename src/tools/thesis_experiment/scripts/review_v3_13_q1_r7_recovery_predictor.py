#!/usr/bin/env python3
"""Independent replay of the V3-13-Q1-R7 model fit."""

import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
FITTER = W / (
    "src/tools/thesis_experiment/scripts/"
    "fit_v3_13_q1_r7_recovery_predictor.py"
)
REVIEW = W / (
    "artifacts/v3/multiactor/v3_13_q1_r7_predictor/"
    "independent_review.yaml"
)


def main():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r7_fitter", FITTER
    )
    fitter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(fitter)
    outputs = fitter.build_outputs()
    for path, payload in outputs.items():
        fitter.require(
            fitter.read(path) == payload,
            "fit replay differs: {}".format(path),
        )
    model = yaml.safe_load(outputs[fitter.MODEL])
    cv = yaml.safe_load(outputs[fitter.CV])
    s3 = fitter.load_yaml(fitter.S3_MODEL)
    candidate_id = (
        "recovery_evasive_right_v120_r1"
    )
    fitter.require(
        cv["cross_validation_pass"], "internal CV failed"
    )
    fitter.require(
        all(
            model["heads"][mode][candidate] == head
            for mode, heads in s3["heads"].items()
            for candidate, head in heads.items()
        ),
        "retained S3 head differs",
    )
    fitter.require(
        set(model["heads"]["assertive"])
        == set(s3["heads"]["assertive"]) | {candidate_id},
        "assertive head roster differs",
    )
    resources = [
        fitter.CONTRACT,
        fitter.C2_FITTER,
        fitter.S3_MODEL,
        fitter.FITTER,
        fitter.REVIEWER,
        fitter.TEST,
    ] + sorted(
        path.relative_to(fitter.W)
        for path in (fitter.W / fitter.R6_EPISODES).glob(
            "*_recovery_evasive_right_v120_r1/episode_report.yaml"
        )
    ) + list(outputs)
    manifest = "".join(
        "{}  {}\n".format(fitter.sha(fitter.read(path)), path)
        for path in resources
    ).encode("utf-8")
    fitter.require(
        fitter.read(fitter.MANIFEST) == manifest,
        "manifest differs",
    )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R7",
        "status": "independent_new_candidate_predictor_fit_review_pass",
        "model_sha256": hashlib.sha256(
            outputs[fitter.MODEL]
        ).hexdigest(),
        "internal_cross_validation_sha256": hashlib.sha256(
            outputs[fitter.CV]
        ).hexdigest(),
        "evidence_manifest_sha256": hashlib.sha256(
            manifest
        ).hexdigest(),
        "retained_S3_heads_byte_equivalent": True,
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
