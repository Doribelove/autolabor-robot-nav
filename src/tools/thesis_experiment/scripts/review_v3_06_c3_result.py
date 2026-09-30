#!/usr/bin/env python3
"""Independent result review for V3-06-C3."""

import argparse
import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_c3.py"
ASSESSOR_PATH = W / "src/tools/thesis_experiment/scripts/assess_v3_06_c3_qualification.py"
BASE_PATH = W / "src/tools/thesis_experiment/scripts/assess_v3_03_c1_closed_loop.py"
RUNNER_PATH = W / "src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
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
    v = load(V3_PATH, "v3_06_c3_review_v")
    assessor = load(ASSESSOR_PATH, "v3_06_c3_review_assessor")
    base = load(BASE_PATH, "v3_06_c3_review_base")
    runner = load(RUNNER_PATH, "v3_06_c3_review_runner")
    value = assessor.assess(v, base, runner)
    value.pop("_evidence_resources")
    stored = v.read_file_once(W / v.ASSESSMENT_RELATIVE)
    v.require(v.yaml_bytes(value) == stored, "assessment replay differs")
    freeze = v.read_file_once(W / v.EVIDENCE_FREEZE_RELATIVE)
    for line in freeze.decode("utf-8").splitlines():
        digest, path = line.split("  ", 1)
        v.require(sha(v.read_file_once(W / path)) == digest, "evidence differs")
    qualified = value["qualified_for_fresh_formal_evaluation"]
    review = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-C3",
        "review_id": "v3_06_c3_independent_result_review_1",
        "status": (
            "v3_06_c3_qualification_review_pass"
            if qualified else "v3_06_c3_qualification_failure_review_pass"
        ),
        "assessment_sha256": sha(stored),
        "evidence_freeze_sha256": sha(freeze),
        "completed_units": 36,
        "qualified_for_fresh_formal_evaluation": qualified,
        "formal_result": False,
        "simulation_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }
    payload = v.yaml_bytes(review)
    output = W / v.INDEPENDENT_RESULT_REVIEW_RELATIVE
    resources = [
        v.ASSESSMENT_RELATIVE,
        v.EVIDENCE_FREEZE_RELATIVE,
        v.INDEPENDENT_RESULT_REVIEW_RELATIVE,
    ]
    terminal = (
        "\n".join(
            "{}  {}".format(
                sha(v.read_file_once(W / item)), item.as_posix()
            )
            for item in resources[:-1]
        )
        + "\n{}  {}\n".format(sha(payload), resources[-1].as_posix())
    ).encode("utf-8")
    if args.check:
        v.require(output.read_bytes() == payload, "review differs")
        v.require(
            v.read_file_once(W / v.TERMINAL_FREEZE_RELATIVE) == terminal,
            "terminal differs",
        )
    else:
        write_once(output, payload)
        write_once(W / v.TERMINAL_FREEZE_RELATIVE, terminal)
    print(
        "status={} qualified={} review_sha256={}".format(
            review["status"], qualified, sha(payload)
        )
    )


if __name__ == "__main__":
    main()
