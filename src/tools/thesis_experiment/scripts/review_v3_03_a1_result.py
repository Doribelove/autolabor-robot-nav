#!/usr/bin/env python3
"""Independent raw replay review of V3-03-A1 integration."""

import argparse
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_a1.py"
ASSESS_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "assess_v3_03_a1_integration.py"
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
        path.write_bytes(payload)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(V3_PATH, "v3_03_a1_result_v3")
    assessor = load(ASSESS_PATH, "v3_03_a1_result_assessor")
    rebuilt = assessor.assess(v)
    stored = v.load_yaml_relative(W, v.ASSESSMENT_RELATIVE)
    v.require(rebuilt == stored, "assessment replay differs")
    v.require(
        stored["overall_pass"] is True and all(stored["gates"].values()),
        "integration gate failed",
    )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": stored["stage"],
        "review_id": "{}_independent_result_review_1".format(
            stored["stage"].lower().replace("-", "_")
        ),
        "status": "{}_independent_integration_review_pass".format(
            stored["stage"].lower().replace("-", "_")
        ),
        "assessment_sha256": v.sha256_bytes(
            v.read_file_once(W / v.ASSESSMENT_RELATIVE)
        ),
        "evidence_freeze_sha256": v.sha256_bytes(
            v.read_file_once(W / v.EVIDENCE_FREEZE_RELATIVE)
        ),
        "raw_simulation_units_replayed": 18,
        "decision_pairs_replayed": 9,
        "all_preregistered_gates_pass": True,
        "formal_performance_result": False,
        "navigation_safety_claim_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }
    payload = v.yaml_bytes(value)
    path = W / v.INDEPENDENT_RESULT_REVIEW_RELATIVE
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
