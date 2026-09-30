#!/usr/bin/env python3
"""Independent result replay for V3-03-C1."""

import argparse
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c1.py"
ASSESSOR_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "assess_v3_03_c1_closed_loop.py"
)
RUNNER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "run_v3_03_c1_closed_loop.py"
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
    v = load(V3_PATH, "v3_03_c1_result_v3")
    assessor = load(ASSESSOR_PATH, "v3_03_c1_result_assessor")
    runner = load(RUNNER_PATH, "v3_03_c1_result_runner")
    rebuilt = assessor.assess(v, runner)
    rebuilt.pop("_evidence_resources")
    stored = v.load_yaml_relative(W, v.ASSESSMENT_RELATIVE)
    v.require(rebuilt == stored, "assessment replay differs")
    v.require(
        stored["overall_pass"] is True and all(stored["gates"].values()),
        "integration result did not pass",
    )
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": stored["stage"],
        "review_id": "{}_independent_result_review_1".format(
            stored["stage"].lower().replace("-", "_")
        ),
        "status": "{}_independent_result_review_pass".format(
            stored["stage"].lower().replace("-", "_")
        ),
        "assessment_sha256": v.sha256_bytes(
            v.read_file_once(W / v.ASSESSMENT_RELATIVE)
        ),
        "evidence_freeze_sha256": v.sha256_bytes(
            v.read_file_once(W / v.EVIDENCE_FREEZE_RELATIVE)
        ),
        "episode_count_replayed": stored["metrics"]["episode_count"],
        "decision_count_replayed": stored["metrics"]["decision_count"],
        "all_preregistered_gates_pass": True,
        "overall_pass": True,
        "claim_scope": (
            "fresh_segmented_gazebo_closed_loop_crossing_integration_only"
        ),
        "continuous_runtime_or_performance_claim_authorized": False,
        "real_vehicle_authorized": False,
    }
    payload = v.yaml_bytes(value)
    output = W / v.INDEPENDENT_RESULT_REVIEW_RELATIVE
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
