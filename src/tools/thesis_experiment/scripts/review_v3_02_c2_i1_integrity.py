#!/usr/bin/env python3
"""Review the C2 post-assessment evidence-manifest integrity repair."""

import argparse
import importlib.util
import json
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_c2.py"
ASSESSOR_PATH = W / "src/tools/thesis_experiment/scripts/assess_v3_02_t1.py"
FITTER_PATH = W / "src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"
CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_02_c2_i1_assessment_integrity_repair_contract.yaml"
)
FAILURE_RECORD = Path(
    "artifacts/v3/prediction/v3_02_c2/"
    "v3_02_c2_post_assessment_failure_record.yaml"
)
OUTPUT = Path(
    "artifacts/v3/prediction/v3_02_c2/"
    "v3_02_c2_i1_independent_integrity_review.yaml"
)


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)


def review():
    v = load(V3_PATH, "v3_02_c2_i1_v3")
    assessor = load(ASSESSOR_PATH, "v3_02_c2_i1_assessor")
    fitter = load(FITTER_PATH, "v3_02_c2_i1_fitter")
    contract = v.load_yaml_relative(W, CONTRACT)
    failure = v.load_yaml_relative(W, FAILURE_RECORD)
    assessment_bytes = v.read_file_once(W / v.ASSESSMENT_RELATIVE)
    v.require(
        contract["stage"] == "V3-02-C2-I1"
        and contract["status"] == "post_failure_pure_offline_repair_frozen"
        and contract["single_factor_repair"]["factor_count"] == 1
        and contract["single_factor_repair"]["simulation_rerun"] is False,
        "repair contract differs",
    )
    v.require(
        v.sha256_bytes(assessment_bytes)
        == contract["failure"]["immutable_assessment_sha256"]
        == failure["assessment"]["sha256"],
        "immutable assessment changed",
    )
    rebuilt = assessor.assess(v, fitter)
    stored = v.load_yaml_relative(W, v.ASSESSMENT_RELATIVE)
    v.require(rebuilt == stored and stored["overall_pass"] is True, "replay differs")
    journal = [
        json.loads(line)
        for line in (W / v.JOURNAL_RELATIVE)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.strip()
    ]
    v.require(
        journal[-1]["event"] == "execution_complete"
        and journal[-1]["consumed_units"] == 36
        and journal[-1]["completed_units"] == 36
        and journal[-1]["retry_units"] == 0,
        "execution journal differs",
    )
    evidence_lines = (
        (W / v.EVIDENCE_FREEZE_RELATIVE)
        .read_text(encoding="utf-8")
        .splitlines()
    )
    model_line = "{}  {}".format(
        stored["frozen_model_sha256"],
        v.load_yaml_relative(W, v.CONTRACT_RELATIVE)["frozen_model"]["path"],
    )
    v.require(
        model_line in evidence_lines
        and not any("v3_02_a1_r1_candidate_predictor" in row for row in evidence_lines),
        "evidence manifest model binding differs",
    )
    result_review = v.load_yaml_relative(W, v.INDEPENDENT_RESULT_REVIEW_RELATIVE)
    v.require(
        result_review["overall_pass"] is True
        and result_review["assessment_sha256"] == v.sha256_bytes(assessment_bytes)
        and result_review["evidence_freeze_sha256"]
        == v.sha256_bytes(v.read_file_once(W / v.EVIDENCE_FREEZE_RELATIVE)),
        "result review differs",
    )
    resources = [
        CONTRACT,
        FAILURE_RECORD,
        Path("src/tools/thesis_experiment/scripts/assess_v3_02_t1.py"),
        Path("src/tools/thesis_experiment/scripts/review_v3_02_c2_i1_integrity.py"),
        v.ASSESSMENT_RELATIVE,
        v.EVIDENCE_FREEZE_RELATIVE,
        v.INDEPENDENT_RESULT_REVIEW_RELATIVE,
    ]
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-02-C2-I1",
        "review_id": "v3_scope_v3_02_c2_i1_integrity_review_1",
        "status": "v3_02_c2_i1_pure_offline_integrity_repair_pass",
        "checks": {
            "immutable_assessment_unchanged": True,
            "all_36_raw_traces_replayed": True,
            "correct_frozen_model_bound": True,
            "obsolete_model_not_bound": True,
            "metrics_and_gates_unchanged": True,
            "simulation_rerun": False,
        },
        "assessment_sha256": v.sha256_bytes(assessment_bytes),
        "evidence_freeze_sha256": v.sha256_bytes(
            v.read_file_once(W / v.EVIDENCE_FREEZE_RELATIVE)
        ),
        "result_review_sha256": v.sha256_bytes(
            v.read_file_once(W / v.INDEPENDENT_RESULT_REVIEW_RELATIVE)
        ),
        "resource_receipts": [v.receipt(W, item) for item in resources],
        "execution_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }, v


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    value, v = review()
    payload = v.yaml_bytes(value)
    path = W / OUTPUT
    if args.check:
        v.require(path.is_file() and path.read_bytes() == payload, "review differs")
    else:
        write_once(path, payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], v.sha256_bytes(payload)
        )
    )


if __name__ == "__main__":
    main()
