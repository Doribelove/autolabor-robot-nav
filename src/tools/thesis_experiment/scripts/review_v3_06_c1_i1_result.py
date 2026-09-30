#!/usr/bin/env python3
"""Independent review for V3-06-C1-I1."""

import argparse
import hashlib
import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ASSESSOR_PATH = W / "src/tools/thesis_experiment/scripts/assess_v3_06_c1_i1_result.py"


def load_assessor():
    spec = importlib.util.spec_from_file_location("v3_06_c1_i1_review", ASSESSOR_PATH)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


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
    assessor = load_assessor()
    value = assessor.assess()
    stored = (W / assessor.REPORT).read_bytes()
    expected = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, width=1000000
    ).encode("utf-8")
    if stored != expected:
        raise RuntimeError("I1 assessment replay differs")
    if not (
        value["integrity"]["only_latency_gate_failed"]
        and value["final_selection"]["all_candidate_levels_disqualified"]
        and not value["final_selection"]["fresh_qualification_authorized"]
    ):
        raise RuntimeError("I1 disposition differs")
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-C1-I1",
        "review_id": "v3_06_c1_i1_independent_review_1",
        "status": "v3_06_c1_i1_latency_failure_review_pass",
        "report_sha256": sha(stored),
        "completed_units": 36,
        "only_failing_shared_gate": "planner_latency_pass",
        "candidate_selected": False,
        "fresh_qualification_authorized": False,
        "simulation_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }
    payload = yaml.safe_dump(
        review, sort_keys=False, allow_unicode=True, width=1000000
    ).encode("utf-8")
    output = W / assessor.REVIEW
    freeze = (
        "{}  {}\n{}  {}\n".format(
            sha(stored),
            assessor.REPORT.as_posix(),
            sha(payload),
            assessor.REVIEW.as_posix(),
        )
    ).encode("utf-8")
    if args.check:
        if output.read_bytes() != payload:
            raise RuntimeError("I1 review differs")
        if (W / assessor.TERMINAL).read_bytes() != freeze:
            raise RuntimeError("I1 terminal freeze differs")
    else:
        write_once(output, payload)
        write_once(W / assessor.TERMINAL, freeze)
    print("status={} review_sha256={}".format(review["status"], sha(payload)))


if __name__ == "__main__":
    main()
