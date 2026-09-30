#!/usr/bin/env python3
"""Independent replay and terminal closure for the negative V3-03-C4 result."""

import argparse
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c4.py"
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
OUTPUT_RELATIVE = Path(
    "artifacts/v3/planner/v3_03_c4/execution/"
    "v3_03_c4_independent_failure_review.yaml"
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


def build(v, assessor, runner):
    rebuilt = assessor.assess(v, runner)
    rebuilt.pop("_evidence_resources")
    stored = v.load_yaml_relative(W, v.ASSESSMENT_RELATIVE)
    v.require(rebuilt == stored, "assessment replay differs")
    failed_gates = sorted(
        name for name, passed in stored["gates"].items() if not passed
    )
    v.require(
        stored["overall_pass"] is False
        and failed_gates == ["goal_success_pass"],
        "negative gate pattern differs",
    )
    v.require(
        stored["execution"]
        == {
            "authorized_units": 9,
            "consumed_units": 9,
            "completed_units": 9,
            "retry_units": 0,
        },
        "execution accounting differs",
    )
    metrics = stored["metrics"]
    v.require(
        metrics["goal_success_rate"] == 2.0 / 3.0
        and metrics["selected_candidate_accepted_rate"] == 1.0
        and metrics["certified_stop_available_rate"] == 1.0
        and metrics["exact_core_replay_rate"] == 1.0
        and metrics["motion_recovery_after_clear_rate"] == 1.0
        and metrics["conservative_clearance_violation_count"] == 0,
        "negative result metrics differ",
    )
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-03-C4",
        "review_id": "v3_scope_v3_03_c4_independent_failure_review_1",
        "status": "v3_03_c4_independent_negative_result_review_pass",
        "assessment_sha256": v.sha256_bytes(
            v.read_file_once(W / v.ASSESSMENT_RELATIVE)
        ),
        "evidence_freeze_sha256": v.sha256_bytes(
            v.read_file_once(W / v.EVIDENCE_FREEZE_RELATIVE)
        ),
        "authorized_consumed_completed": [9, 9, 9],
        "failed_gates": failed_gates,
        "goal_success_rate": metrics["goal_success_rate"],
        "all_safety_integrity_replay_gates_pass": True,
        "failure_classification": (
            "bounded_goal_efficiency_failure_after_safe_yield_and_recovery"
        ),
        "retry_or_resume_authorized": False,
        "gate_change_authorized": False,
        "performance_claim_authorized": False,
        "real_vehicle_authorized": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(V3_PATH, "v3_03_c4_failure_v3")
    assessor = load(ASSESSOR_PATH, "v3_03_c4_failure_assessor")
    runner = load(RUNNER_PATH, "v3_03_c4_failure_runner")
    value = build(v, assessor, runner)
    payload = v.yaml_bytes(value)
    output = W / OUTPUT_RELATIVE
    terminal_resources = [
        v.RELEASE_RELATIVE,
        v.REVIEW_RELATIVE,
        v.READINESS_FREEZE_RELATIVE,
        v.AUTHORIZATION_RELATIVE,
        v.JOURNAL_RELATIVE,
        v.ASSESSMENT_RELATIVE,
        v.EVIDENCE_FREEZE_RELATIVE,
        OUTPUT_RELATIVE,
        Path(
            "src/tools/thesis_experiment/scripts/"
            "review_v3_03_c4_failure.py"
        ),
    ]
    if args.check:
        v.require(
            output.is_file() and output.read_bytes() == payload,
            "failure review differs",
        )
        freeze = v.read_file_once(W / v.TERMINAL_FREEZE_RELATIVE)
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
        v.require(freeze == expected, "terminal freeze differs")
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
