#!/usr/bin/env python3
"""Freeze the terminal package-resolution failure of V3-13-Q1 preflight."""

import hashlib
import json
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = Path("artifacts/v3/multiactor/v3_13_q1_preflight")
READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_preflight_readiness"
)
SCRIPT = Path(
    "src/tools/thesis_experiment/scripts/"
    "freeze_v3_13_q1_preflight_failure.py"
)
REVIEWER = Path(
    "src/tools/thesis_experiment/scripts/"
    "review_v3_13_q1_preflight_failure.py"
)
REPORT = ROOT / "terminal_failure_freeze.yaml"


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def build():
    events = [
        json.loads(line)
        for line in read(ROOT / "attempt_journal.jsonl")
        .decode("utf-8")
        .splitlines()
        if line.strip()
    ]
    if [event["event"] for event in events] != [
        "execution_opened",
        "unit_consumed",
        "terminal_failure",
    ]:
        raise RuntimeError("terminal journal differs")
    failure = events[-1]
    if "neither a launch file" not in failure["error"]:
        raise RuntimeError("failure attribution differs")
    episode = ROOT / "episodes/001_01_7300_v3_13_analytic"
    resources = [
        READINESS / "execution_release.yaml",
        READINESS / "independent_review.yaml",
        ROOT / "exact_schedule.yaml",
        ROOT / "attempt_journal.jsonl",
        episode / "world.sdf",
        episode / "scene_spec.yaml",
        episode / "actor_specs.yaml",
        episode / "roslaunch.log",
        SCRIPT,
        REVIEWER,
    ]
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-PREFLIGHT",
        "status": "terminal_presemantic_package_resolution_failure",
        "authorized_units": 2,
        "consumed_units": 1,
        "completed_units": 0,
        "forfeited_units": 1,
        "consumed_identity": failure["row"],
        "semantic_execution_reached": False,
        "root_cause": (
            "runner inherited a foreign ROS_PACKAGE_PATH and resolved an "
            "m2_gazebo package that did not contain the released launch"
        ),
        "repair_boundary": {
            "new_stage_and_fresh_preflight_seed_required": True,
            "explicit_thesis_workspace_overlay_required": True,
            "planner_actor_TEB_config_or_gate_change_authorized": False,
        },
        "retry_resume_or_seed_reuse_authorized": False,
        "qualification_seed_consumed": False,
        "resources": [
            {"path": str(path), "sha256": sha(read(path))}
            for path in resources
        ],
        "policy_or_ranker_training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def main():
    payload = build()
    path = W / REPORT
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("failure freeze drift")
        status = "reproducible"
    else:
        temporary = path.with_suffix(".yaml.tmp")
        temporary.write_bytes(payload)
        os.replace(str(temporary), str(path))
        status = "written"
    print(
        "status={} freeze_sha256={}".format(status, sha(payload))
    )


if __name__ == "__main__":
    main()
