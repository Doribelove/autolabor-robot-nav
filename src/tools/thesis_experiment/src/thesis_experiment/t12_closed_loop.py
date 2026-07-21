"""Paired evaluator for the T12 no-training Gazebo safety repair study."""

import csv
from pathlib import Path
from typing import Any, Dict, List, Mapping, Tuple

import yaml


METHODS = ("old_full_safety", "t12_safety", "projection_only")
SEEDS = (101, 102)


class T12ClosedLoopError(ValueError):
    pass


def _rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _summary(rows: List[Mapping[str, str]]) -> Dict[str, Any]:
    count = len(rows)
    if not count:
        raise T12ClosedLoopError("cannot summarize zero episodes")
    reason = lambda name: sum(row["termination_reason"] == name for row in rows)
    known_reasons = {"goal", "emergency_stop", "collision", "timeout",
                     "planner_failure", "interface_fault"}
    return {
        "episode_count": count,
        "success_count": reason("goal"),
        "success_rate": reason("goal") / float(count),
        "emergency_stop_count": reason("emergency_stop"),
        "emergency_stop_rate": reason("emergency_stop") / float(count),
        "collision_count": reason("collision"),
        "collision_rate": reason("collision") / float(count),
        "timeout_count": reason("timeout"),
        "interface_fault_count": reason("interface_fault"),
        "unknown_termination_count": sum(
            row["termination_reason"] not in known_reasons for row in rows),
        "planner_failure_count": sum(int(row["planner_failure_count"]) for row in rows),
        "mean_navigation_time_s": sum(float(row["navigation_time"]) for row in rows) / count,
        "mean_min_clearance_m": sum(float(row["min_obstacle_distance"]) for row in rows) / count,
    }


def evaluate_closed_loop(root: Any) -> Dict[str, Any]:
    root = Path(root)
    summaries, all_rows = {}, {}
    for method in METHODS:
        method_rows = []
        for seed in SEEDS:
            run_id = "t12_{}_seed{}".format(method, seed)
            run = root / "runs" / run_id
            report_path = run / "t11_run_report.yaml"
            if not report_path.is_file():
                raise T12ClosedLoopError("missing run report: {}".format(run_id))
            state = yaml.safe_load(report_path.read_text(encoding="utf-8"))
            if state.get("task") != "T12" or state.get("passed") is not True:
                raise T12ClosedLoopError("invalid T12 run: {}".format(run_id))
            rows = _rows(run / "episodes.csv")
            if len(rows) != 10:
                raise T12ClosedLoopError("{} must contain exactly 10 episodes".format(run_id))
            if len({row["scene_id"] for row in rows}) != 10:
                raise T12ClosedLoopError("{} scene pairing is incomplete".format(run_id))
            method_rows.extend(rows)
        all_rows[method] = method_rows
        summaries[method] = _summary(method_rows)

    old = summaries["old_full_safety"]
    new = summaries["t12_safety"]
    projection = summaries["projection_only"]
    paired_keys = {
        method: {(row["training_seed"], row["seed"], row["scene_id"])
                 for row in rows}
        for method, rows in all_rows.items()
    }
    pairing_complete = all(paired_keys[method] == paired_keys[METHODS[0]]
                           for method in METHODS[1:])
    gates = {
        "exact_60_episode_complete_pairing": pairing_complete and
            all(item["episode_count"] == 20 for item in summaries.values()),
        "new_emergency_rate_reduced_by_20pp":
            old["emergency_stop_rate"] - new["emergency_stop_rate"] >= 0.20,
        "new_emergency_rate_at_most_quarter_of_old":
            new["emergency_stop_rate"] <= 0.25 * old["emergency_stop_rate"],
        "collision_not_increased_vs_old":
            new["collision_count"] <= old["collision_count"],
        "collision_not_increased_vs_projection":
            new["collision_count"] <= projection["collision_count"],
        "success_recovered_by_20pp":
            new["success_rate"] - old["success_rate"] >= 0.20,
        "success_within_15pp_of_projection":
            new["success_rate"] + 0.15 >= projection["success_rate"],
        "new_planner_failures_zero": new["planner_failure_count"] == 0,
        "new_interface_faults_zero": new["interface_fault_count"] == 0,
        "new_termination_reasons_complete": new["unknown_termination_count"] == 0,
    }
    passed = all(gates.values())
    return {
        "schema_version": "1.0", "task": "T12", "study": "closed_loop_no_training",
        "status": "passed" if passed else "failed", "passed": passed,
        "training_performed": False, "simulation_only": True,
        "methods": summaries, "gates": gates,
        "effect": {
            "emergency_rate_reduction_percentage_points": 100.0 * (
                old["emergency_stop_rate"] - new["emergency_stop_rate"]),
            "success_rate_change_percentage_points": 100.0 * (
                new["success_rate"] - old["success_rate"]),
        },
        "decision": ("safety_repair_validated_no_training_may_proceed" if passed else
                     "do_not_train_continue_safety_repair"),
    }
