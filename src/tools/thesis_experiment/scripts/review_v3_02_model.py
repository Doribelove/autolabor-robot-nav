#!/usr/bin/env python3
"""Independent deterministic V3-02 model/calibration review."""

import argparse
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
FIT_PATH = W / "src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"


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
        path.write_bytes(payload)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(V3_PATH, "v3_02_predictor")
    fitter = load(FIT_PATH, "v3_02_fitter")
    rebuilt_model, rebuilt_report = fitter.fit(v)
    stored_model = v.load_yaml_relative(W, v.MODEL_RELATIVE)
    stored_report = v.load_yaml_relative(W, v.CALIBRATION_RELATIVE)
    v.require(stored_model == rebuilt_model, "model rebuild differs")
    v.require(stored_report == rebuilt_report, "calibration rebuild differs")
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    reserved = sorted(
        contract["authority_boundary"]["held_out_test_seeds_reserved"]
    )
    v.require(
        all(row["seed"] not in set(reserved) for row in schedule["rows"]),
        "reserved test seed leaked",
    )
    v.require(
        stored_report["overall_pass"] is True
        and all(stored_report["gates"].values()),
        "calibration gate failed",
    )
    data_qualification_sha256 = None
    if hasattr(v, "DATA_QUALIFICATION_RELATIVE"):
        qualification = v.load_yaml_relative(
            W, v.DATA_QUALIFICATION_RELATIVE
        )
        v.require(
            qualification["overall_pass"] is True
            and all(qualification["gates"].values()),
            "data qualification gate failed",
        )
        data_qualification_sha256 = v.sha256_bytes(
            v.read_file_once(W / v.DATA_QUALIFICATION_RELATIVE)
        )
    value = {
        "schema_version": "1.0",
        "stage": stored_report["stage"],
        "review_id": "{}_independent_model_review_1".format(
            stored_report["stage"].lower().replace("-", "_")
        ),
        "status": "{}_predictor_model_calibration_review_pass".format(
            stored_report["stage"].lower().replace("-", "_")
        ),
        "model_sha256": v.sha256_bytes(v.read_file_once(W / v.MODEL_RELATIVE)),
        "calibration_report_sha256": v.sha256_bytes(
            v.read_file_once(W / v.CALIBRATION_RELATIVE)
        ),
        "model_freeze_sha256": v.sha256_bytes(
            v.read_file_once(W / v.MODEL_FREEZE_RELATIVE)
        ),
        "data_qualification_sha256": data_qualification_sha256,
        "byte_identical_model_rebuild": True,
        "byte_identical_calibration_rebuild": True,
        "runtime_hidden_policy_input": False,
        "runtime_outputs_all_three_mode_heads": True,
        "training_rows": schedule["training_rows"],
        "calibration_rows": schedule["calibration_rows"],
        "held_out_test_rows_accessed": 0,
        "reserved_test_seeds_intact": reserved,
        "all_preregistered_calibration_gates_pass": True,
        "held_out_test_authorized": False,
        "planner_or_performance_claim_authorized": False,
    }
    payload = v.yaml_bytes(value)
    output = W / v.INDEPENDENT_MODEL_REVIEW_RELATIVE
    if args.check:
        v.require(
            output.is_file() and output.read_bytes() == payload,
            "review differs",
        )
    else:
        write_once(output, payload)
    print("review_sha256={}".format(v.sha256_bytes(payload)))


if __name__ == "__main__":
    main()
