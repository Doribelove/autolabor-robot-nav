#!/usr/bin/env python3
"""Fit only the new R13 progress-candidate assertive head."""

import argparse
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "fit_v3_13_q1_r7_recovery_predictor.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r7_frozen_fitter", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
BASE.CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_13_q1_r13_progress_predictor_fit_contract.yaml"
)
BASE.S3_MODEL = Path(
    "artifacts/v3/multiactor/v3_13_q1_r7_predictor/"
    "expanded_recovery_predictor.yaml"
)
BASE.R6_EPISODES = Path(
    "artifacts/v3/multiactor/v3_13_q1_r12_progress_support/episodes"
)
BASE.FITTER = Path(
    "src/tools/thesis_experiment/scripts/"
    "fit_v3_13_q1_r13_progress_predictor.py"
)
BASE.REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_r13_progress_predictor.py"
)
BASE.TEST = Path(
    "src/tools/thesis_experiment/tests/"
    "test_v3_13_q1_r13_predictor_fit.py"
)
BASE.ROOT = Path(
    "artifacts/v3/multiactor/v3_13_q1_r13_predictor"
)
BASE.MODEL = BASE.ROOT / "expanded_progress_predictor.yaml"
BASE.CV = BASE.ROOT / "internal_cross_validation.yaml"
BASE.RECEIPT = BASE.ROOT / "fit_receipt.yaml"
BASE.MANIFEST = BASE.ROOT / "evidence_manifest.sha256"


def build_outputs():
    outputs = BASE.build_outputs()
    model = yaml.safe_load(outputs[BASE.MODEL])
    model["stage"] = "V3-13-Q1-R13"
    model["model_id"] = "v3_13_q1_r13_expanded_progress_predictor_1"
    model["retained_R7_predictor_sha256"] = model.pop(
        "retained_S3_predictor_sha256"
    )
    cv = yaml.safe_load(outputs[BASE.CV])
    cv["stage"] = "V3-13-Q1-R13"
    cv["status"] = "progress_candidate_internal_CV_pass"
    receipt = yaml.safe_load(outputs[BASE.RECEIPT])
    receipt["stage"] = "V3-13-Q1-R13"
    receipt["status"] = "deterministic_progress_candidate_head_fit_pass"
    receipt["R7_predictor_sha256"] = receipt.pop(
        "S3_predictor_sha256"
    )
    receipt["retained_R7_heads_byte_equivalent"] = receipt.pop(
        "retained_S3_heads_byte_equivalent"
    )
    model_payload = BASE.yaml_bytes(model)
    cv_payload = BASE.yaml_bytes(cv)
    receipt["model_sha256"] = BASE.sha(model_payload)
    receipt["internal_cross_validation_sha256"] = BASE.sha(cv_payload)
    return {
        BASE.MODEL: model_payload,
        BASE.CV: cv_payload,
        BASE.RECEIPT: BASE.yaml_bytes(receipt),
    }


def resources(outputs):
    candidate_id = "recovery_evasive_right_v120_r2"
    return [
        BASE.CONTRACT,
        BASE.C2_FITTER,
        BASE.S3_MODEL,
        BASE.FITTER,
        BASE.REVIEWER,
        BASE.TEST,
    ] + sorted(
        path.relative_to(W)
        for path in (W / BASE.R6_EPISODES).glob(
            "*_{}/episode_report.yaml".format(candidate_id)
        )
    ) + list(outputs)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build_outputs()
    for path, payload in outputs.items():
        BASE.write_or_check(path, payload, args.check)
    manifest = "".join(
        "{}  {}\n".format(BASE.sha(BASE.read(path)), path)
        for path in resources(outputs)
    ).encode("utf-8")
    BASE.write_or_check(BASE.MANIFEST, manifest, args.check)
    print(
        "model_sha256={} cv_sha256={} manifest_sha256={}".format(
            BASE.sha(outputs[BASE.MODEL]),
            BASE.sha(outputs[BASE.CV]),
            BASE.sha(manifest),
        )
    )


if __name__ == "__main__":
    main()
