#!/usr/bin/env python3
"""Independent result replay and terminal closure for V3-05-A0."""

import argparse
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_05_a0.py"
ASSESSOR_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "assess_v3_05_a0_formal.py"
)
BASE_ASSESSOR_PATH = (
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


def build(v, assessor, base_assessor, runner):
    rebuilt = assessor.assess(v, base_assessor, runner)
    rebuilt.pop("_evidence_resources")
    stored = v.load_yaml_relative(W, v.ASSESSMENT_RELATIVE)
    v.require(rebuilt == stored, "formal assessment replay differs")
    v.require(
        stored["formal_result"] is True
        and stored["execution"]["authorized_units"] == 270
        and stored["execution"]["consumed_units"] == 270
        and stored["execution"]["completed_units"] == 270
        and stored["formal_gates"]["complete_paired_matrix_pass"] is True
        and stored["formal_gates"]["shared_safety_integrity_pass"] is True,
        "formal matrix/integrity gate failed",
    )
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": stored["stage"],
        "review_id": "{}_independent_result_review_1".format(
            stored["stage"].lower().replace("-", "_")
        ),
        "status": (
            "{}_independent_positive_result_review_pass".format(
                stored["stage"].lower().replace("-", "_")
            )
            if stored["performance_improvement_demonstrated"]
            else "{}_independent_valid_negative_result_review_pass".format(
                stored["stage"].lower().replace("-", "_")
            )
        ),
        "assessment_sha256": v.sha256_bytes(
            v.read_file_once(W / v.ASSESSMENT_RELATIVE)
        ),
        "evidence_freeze_sha256": v.sha256_bytes(
            v.read_file_once(W / v.EVIDENCE_FREEZE_RELATIVE)
        ),
        "formal_result": True,
        "performance_improvement_demonstrated": stored[
            "performance_improvement_demonstrated"
        ],
        "statistical_unit_count": stored["primary_statistics"][
            "statistical_unit_count"
        ],
        "point_relative_reduction": stored["primary_statistics"][
            "point_relative_reduction"
        ],
        "one_sided_lower_relative_reduction": stored[
            "primary_statistics"
        ]["one_sided_lower_relative_reduction"],
        "complete_paired_matrix": True,
        "shared_safety_integrity_pass": True,
        "actual_teb_claim": False,
        "general_navigation_or_deployment_claim": False,
        "real_vehicle_authorized": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(V3_PATH, "v3_05_a0_review_v3")
    assessor = load(ASSESSOR_PATH, "v3_05_a0_review_assessor")
    base_assessor = load(
        BASE_ASSESSOR_PATH, "v3_05_a0_review_base_assessor"
    )
    runner = load(RUNNER_PATH, "v3_05_a0_review_runner")
    value = build(v, assessor, base_assessor, runner)
    payload = v.yaml_bytes(value)
    output = W / v.INDEPENDENT_RESULT_REVIEW_RELATIVE
    terminal_resources = [
        v.RELEASE_RELATIVE,
        v.REVIEW_RELATIVE,
        v.READINESS_FREEZE_RELATIVE,
        v.AUTHORIZATION_RELATIVE,
        v.JOURNAL_RELATIVE,
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
                    v.sha256_bytes(v.read_file_once(W / item)),
                    item.as_posix(),
                )
                for item in terminal_resources
            )
            + "\n"
        ).encode("utf-8")
        v.require(
            (W / v.TERMINAL_FREEZE_RELATIVE).read_bytes() == expected,
            "terminal freeze differs",
        )
    else:
        write_once(output, payload)
        freeze = (
            "\n".join(
                "{}  {}".format(
                    v.sha256_bytes(v.read_file_once(W / item)),
                    item.as_posix(),
                )
                for item in terminal_resources
            )
            + "\n"
        ).encode("utf-8")
        write_once(W / v.TERMINAL_FREEZE_RELATIVE, freeze)
    print(
        "status={} review_sha256={}".format(
            value["status"], v.sha256_bytes(payload)
        )
    )


if __name__ == "__main__":
    main()
