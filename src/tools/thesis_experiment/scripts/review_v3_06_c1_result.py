#!/usr/bin/env python3
"""Independent result review for V3-06-C1."""

import argparse
import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_c1.py"
ASSESSOR_PATH = W / "src/tools/thesis_experiment/scripts/assess_v3_06_c1_calibration.py"
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


def review(v, assessor, base, runner):
    computed = assessor.assess(v, base, runner)
    computed.pop("_evidence_resources")
    stored = v.read_file_once(W / v.ASSESSMENT_RELATIVE)
    v.require(v.yaml_bytes(computed) == stored, "assessment replay differs")
    freeze = v.read_file_once(W / v.EVIDENCE_FREEZE_RELATIVE)
    for line in freeze.decode("utf-8").splitlines():
        digest, path = line.split("  ", 1)
        v.require(
            sha(v.read_file_once(W / path)) == digest,
            "evidence resource differs",
        )
    selected = computed["selection"]["selected_method"]
    if selected:
        v.require(
            computed["candidate_effects"][selected]["eligible"] is True,
            "selection gate differs",
        )
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-C1",
        "review_id": "v3_06_c1_independent_result_review_1",
        "status": (
            "v3_06_c1_development_selection_review_pass"
            if selected
            else "v3_06_c1_no_selection_review_pass"
        ),
        "assessment_sha256": sha(stored),
        "evidence_freeze_sha256": sha(freeze),
        "completed_units": computed["execution"]["completed_units"],
        "selected_method": selected,
        "selected_new_candidate_ids": computed["selection"][
            "selected_new_candidate_ids"
        ],
        "formal_result": False,
        "fresh_qualification_required": bool(selected),
        "simulation_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(V3_PATH, "v3_06_c1_review_v3")
    assessor = load(ASSESSOR_PATH, "v3_06_c1_review_assessor")
    base = load(BASE_PATH, "v3_06_c1_review_base")
    runner = load(RUNNER_PATH, "v3_06_c1_review_runner")
    value = review(v, assessor, base, runner)
    payload = v.yaml_bytes(value)
    output = W / v.INDEPENDENT_RESULT_REVIEW_RELATIVE
    resources = [
        v.ASSESSMENT_RELATIVE,
        v.EVIDENCE_FREEZE_RELATIVE,
        v.INDEPENDENT_RESULT_REVIEW_RELATIVE,
    ]
    if args.check:
        v.require(
            output.is_file() and output.read_bytes() == payload,
            "review differs",
        )
        expected = (
            "\n".join(
                "{}  {}".format(
                    sha(v.read_file_once(W / item)), item.as_posix()
                )
                for item in resources
            )
            + "\n"
        ).encode("utf-8")
        v.require(
            v.read_file_once(W / v.TERMINAL_FREEZE_RELATIVE) == expected,
            "terminal freeze differs",
        )
    else:
        write_once(output, payload)
        terminal = (
            "\n".join(
                "{}  {}".format(
                    sha(v.read_file_once(W / item)), item.as_posix()
                )
                for item in resources
            )
            + "\n"
        ).encode("utf-8")
        write_once(W / v.TERMINAL_FREEZE_RELATIVE, terminal)
    print(
        "status={} selected={} review_sha256={}".format(
            value["status"], value["selected_method"] or "none", sha(payload)
        )
    )


if __name__ == "__main__":
    main()
