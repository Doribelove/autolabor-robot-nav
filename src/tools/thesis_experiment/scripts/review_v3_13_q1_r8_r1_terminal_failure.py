#!/usr/bin/env python3
"""Freeze the presemantic R8-R1 runner failure without retrying it."""

import hashlib
import json
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r8_r1_readiness"
)
EXECUTION = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r8_r1_recovery_qualification"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r8_r1_terminal_failure_review.yaml"
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUTPUT.exists():
        raise RuntimeError("terminal review exists")
    journal = EXECUTION / "attempt_journal.jsonl"
    events = [
        json.loads(line)
        for line in journal.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    if [event["event"] for event in events] != [
        "execution_opened",
        "unit_consumed",
        "terminal_failure",
    ]:
        raise RuntimeError("unexpected journal shape")
    failure = events[-1]
    if failure["sequence"] != 1:
        raise RuntimeError("unexpected failure sequence")
    if failure["remaining_units_forfeited"] != 39:
        raise RuntimeError("unexpected forfeiture")
    if not failure["error"].startswith(
        "TypeError:repaired_run_episode() missing 1 required"
    ):
        raise RuntimeError("unexpected failure cause")
    if (EXECUTION / "episodes").exists():
        raise RuntimeError("semantic episode root unexpectedly exists")
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R8-R1",
        "status": "terminal_presemantic_runner_failure_frozen",
        "execution_release_sha256": sha(
            READINESS / "execution_release.yaml"
        ),
        "readiness_review_sha256": sha(
            READINESS / "independent_review.yaml"
        ),
        "exact_schedule_sha256": sha(
            EXECUTION / "exact_schedule.yaml"
        ),
        "attempt_journal_sha256": sha(journal),
        "failure_cause": "incorrect_duplicate_argument_adapter",
        "authorized_units": 40,
        "consumed_units": 1,
        "completed_units": 0,
        "forfeited_units": 39,
        "semantic_episode_started": False,
        "ros_or_gazebo_started": False,
        "seeds7625_through_7632_reusable": False,
        "retry_or_resume_authorized": False,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    OUTPUT.write_bytes(payload)
    print(
        "status={} review_sha256={}".format(
            value["status"], hashlib.sha256(payload).hexdigest()
        )
    )


if __name__ == "__main__":
    main()
