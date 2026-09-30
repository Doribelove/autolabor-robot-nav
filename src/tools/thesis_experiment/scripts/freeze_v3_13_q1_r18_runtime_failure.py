#!/usr/bin/env python3
"""Freeze and attribute the terminal R18 recursive-adapter failure."""

import hashlib
import json
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
READINESS = Path(
    "artifacts/v3/multiactor/v3_13_q1_r18_readiness"
)
EXECUTION = Path(
    "artifacts/v3/multiactor/v3_13_q1_r18_goal_aligned_preflight"
)
EPISODE = EXECUTION / (
    "episodes/001_01_7667_recovery_evasive_right_v120_r3"
)
OUTPUT = Path(
    "artifacts/v3/multiactor/"
    "v3_13_q1_r18_terminal_runtime_failure.yaml"
)
RESOURCES = [
    Path(
        "config/thesis_experiments/v3/"
        "v3_13_q1_r18_goal_aligned_recovery_preflight_contract.yaml"
    ),
    READINESS / "exact_schedule.yaml",
    READINESS / "dependency_freeze.yaml",
    READINESS / "execution_release.yaml",
    READINESS / "evidence_manifest.sha256",
    READINESS / "independent_review.yaml",
    EXECUTION / "exact_schedule.yaml",
    EXECUTION / "attempt_journal.jsonl",
    EPISODE / "world.sdf",
    EPISODE / "scene_spec.yaml",
    EPISODE / "actor_specs.yaml",
    EPISODE / "roslaunch.log",
    Path(
        "src/application/v3_scope_planner/scripts/"
        "v3_scope_goal_aligned_recovery_preflight_episode.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "goal_aligned_execution_core.py"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/"
        "execution_core.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_13_q1_r18_goal_aligned_preflight.py"
    ),
]


def read(path):
    target = W / path
    if not target.is_file():
        raise RuntimeError("required file absent: {}".format(path))
    return target.read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def main():
    target = W / OUTPUT
    if target.exists():
        raise RuntimeError("terminal failure freeze exists")
    events = [
        json.loads(line)
        for line in read(EXECUTION / "attempt_journal.jsonl")
        .decode("utf-8")
        .splitlines()
        if line.strip()
    ]
    if [event["event"] for event in events] != [
        "execution_opened",
        "unit_consumed",
        "terminal_failure",
    ]:
        raise RuntimeError("unexpected journal shape")
    failure = events[-1]
    if failure["row"]["seed"] != 7667 or failure["sequence"] != 1:
        raise RuntimeError("unexpected failed identity")
    if "RecursionError: maximum recursion depth exceeded" not in failure[
        "error"
    ]:
        raise RuntimeError("unexpected failure cause")
    if (W / EPISODE / "episode_report.yaml").exists():
        raise RuntimeError("unexpected semantic report")
    value = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R18",
        "status": "terminal_runtime_adapter_recursion_failure_frozen",
        "failure_class": "pretrace_execution_adapter_integration",
        "confirmed_cause": (
            "The runtime replaced execution_core.pure_pursuit_command "
            "with observed_guard, while the frozen goal-aligned helper "
            "references that same execution_core module and called the "
            "replacement again, creating recursion on the first branch "
            "command."
        ),
        "failure_observed_sim_s": 12.54,
        "authorized_units": 1,
        "consumed_units": 1,
        "completed_units": 0,
        "remaining_units": 0,
        "seed7667_reusable": False,
        "retry_or_resume_authorized": False,
        "gazebo_started": True,
        "closed_loop_candidate_trace_started": False,
        "episode_report_present": False,
        "scientific_outcome_available": False,
        "r3_preflight_qualified": False,
        "repair_boundary": {
            "new_stage_and_fresh_seed_required": True,
            "only_execution_adapter_recursion_may_change": True,
            "candidate_geometry_model_actor_policies_and_gates_frozen": True,
            "adapter_equivalence_to_frozen_guard_required_offline": True,
        },
        "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "resources": [
            {
                "path": str(path),
                "sha256": sha(read(path)),
                "size_bytes": len(read(path)),
            }
            for path in RESOURCES
        ],
    }
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(payload)
    print(
        "status={} freeze_sha256={}".format(
            value["status"], sha(payload)
        )
    )


if __name__ == "__main__":
    main()
