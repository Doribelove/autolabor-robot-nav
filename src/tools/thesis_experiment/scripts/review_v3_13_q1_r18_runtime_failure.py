#!/usr/bin/env python3
"""Independently review the frozen R18 runtime adapter failure."""

import hashlib
import json
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
FREEZE = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r18_terminal_runtime_failure.yaml"
)
JOURNAL = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r18_goal_aligned_preflight/attempt_journal.jsonl"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r18_terminal_runtime_failure_review.yaml"
)


def main():
    if OUTPUT.exists():
        raise RuntimeError("failure review exists")
    freeze = yaml.safe_load(FREEZE.read_bytes())
    events = [
        json.loads(line)
        for line in JOURNAL.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    failure = events[-1]
    checks = {
        "terminal_journal_shape": [event["event"] for event in events]
        == ["execution_opened", "unit_consumed", "terminal_failure"],
        "seed7667_consumed_once": failure["row"]["seed"] == 7667
        and failure["row"]["attempt"] == 1,
        "recursion_observed": (
            "RecursionError: maximum recursion depth exceeded"
            in failure["error"]
        ),
        "report_absent": not (
            JOURNAL.parent
            / "episodes/001_01_7667_recovery_evasive_right_v120_r3/"
            "episode_report.yaml"
        ).exists(),
        "no_scientific_claim": (
            not freeze["scientific_outcome_available"]
            and not freeze["r3_preflight_qualified"]
        ),
        "seed_and_retry_closed": (
            not freeze["seed7667_reusable"]
            and not freeze["retry_or_resume_authorized"]
        ),
    }
    if not all(checks.values()):
        raise RuntimeError("failure review check failed")
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R18",
        "status": "independent_terminal_runtime_adapter_failure_review_pass",
        "failure_freeze_sha256": hashlib.sha256(
            FREEZE.read_bytes()
        ).hexdigest(),
        "checks": checks,
        "fresh_single_factor_adapter_repair_design_authorized": True,
        "simulation_execution_authorized": False,
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
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
