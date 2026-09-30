#!/usr/bin/env python3
"""Independent replay review of the V3-02-T1 held-out result."""

import argparse
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_t1.py"
ASSESS_PATH = W / "src/tools/thesis_experiment/scripts/assess_v3_02_t1.py"
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
    v = load(V3_PATH, "v3_02_t1")
    assessor = load(ASSESS_PATH, "v3_02_t1_assessor")
    fitter = load(FIT_PATH, "v3_02_fitter")
    rebuilt = assessor.assess(v, fitter)
    stored = v.load_yaml_relative(W, v.ASSESSMENT_RELATIVE)
    v.require(rebuilt == stored, "held-out assessment replay differs")
    v.require(stored["overall_pass"] is True, "held-out result did not pass")
    value = {
        "schema_version": "1.0",
        "stage": stored["stage"],
        "review_id": "{}_independent_result_review_1".format(
            stored["stage"].lower().replace("-", "_")
        ),
        "status": "{}_independent_heldout_review_pass".format(
            stored["stage"].lower().replace("-", "_")
        ),
        "assessment_sha256": v.sha256_bytes(
            v.read_file_once(W / v.ASSESSMENT_RELATIVE)
        ),
        "evidence_freeze_sha256": v.sha256_bytes(
            v.read_file_once(W / v.EVIDENCE_FREEZE_RELATIVE)
        ),
        "raw_episode_count_replayed": stored["execution"]["completed_units"],
        "point_count_recomputed": int(
            v.load_yaml_relative(W, v.CONTRACT_RELATIVE)["heldout_gates"][
                "point_count"
            ]
        ),
        "frozen_model_unchanged": True,
        "all_preregistered_gates_pass": True,
        "overall_pass": True,
        "claim_scope": "heldout_parameterized_crossing_prediction_only",
        "planner_or_performance_claim_authorized": False,
    }
    payload = v.yaml_bytes(value)
    output = W / v.INDEPENDENT_RESULT_REVIEW_RELATIVE
    if args.check:
        v.require(output.is_file() and output.read_bytes() == payload, "review differs")
    else:
        write_once(output, payload)
    print("review_sha256={}".format(v.sha256_bytes(payload)))


if __name__ == "__main__":
    main()
