"""Strict paired evaluator for selected residual SAC and its two anchor baselines."""

import csv
from pathlib import Path
from typing import Any, Dict, List, Mapping, Tuple

import yaml


METHODS = ("selected_sac", "zero_residual", "teb_tuned")
SEEDS = (101, 102)


class T12ResidualPairError(ValueError):
    pass


def _read_csv(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _mean(values):
    values = list(values)
    return float(sum(values) / len(values)) if values else None


def _return_by_episode(run: Path) -> Dict[str, float]:
    result: Dict[str, float] = {}
    for row in _read_csv(run / "steps.csv"):
        result[row["episode_id"]] = result.get(row["episode_id"], 0.0) + float(row["reward_total"])
    return result


def _summary(rows: List[Mapping[str, Any]]) -> Dict[str, Any]:
    count = len(rows)
    if not count:
        raise T12ResidualPairError("empty method")
    return {
        "episode_count": count,
        "goal": sum(row["termination_reason"] == "goal" for row in rows),
        "success_rate": sum(row["termination_reason"] == "goal" for row in rows) / float(count),
        "collision": sum(row["termination_reason"] == "collision" for row in rows),
        "emergency_stop": sum(row["termination_reason"] == "emergency_stop" for row in rows),
        "planner_failure": sum(row["termination_reason"] == "planner_failure" for row in rows),
        "interface_fault": sum(row["termination_reason"] == "interface_fault" for row in rows),
        "mean_return": _mean(float(row["_return"]) for row in rows),
        "mean_navigation_time_s": _mean(float(row["navigation_time"]) for row in rows),
        "mean_path_length_m": _mean(float(row["path_length"]) for row in rows),
        "projection_intervention_count": sum(int(row["projection_intervention_count"])
                                             for row in rows),
        "safety_intervention_count": sum(int(row["safety_filter_intervention_count"])
                                         for row in rows),
    }


def _paired(left: Mapping[Tuple[str, str, str], Mapping[str, Any]],
            right: Mapping[Tuple[str, str, str], Mapping[str, Any]]) -> Dict[str, Any]:
    if set(left) != set(right):
        raise T12ResidualPairError("paired keys differ")
    keys = sorted(left)
    return_delta = {key: float(left[key]["_return"]) - float(right[key]["_return"])
                    for key in keys}
    goal_keys = [key for key in keys if left[key]["termination_reason"] == "goal" and
                 right[key]["termination_reason"] == "goal"]
    return {
        "pair_count": len(keys),
        "mean_return_delta": _mean(return_delta.values()),
        "return_delta_by_training_seed": {
            str(seed): _mean(value for key, value in return_delta.items() if int(key[0]) == seed)
            for seed in SEEDS
        },
        "both_goal_pair_count": len(goal_keys),
        "mean_navigation_time_reduction_s_on_both_goal": _mean(
            float(right[key]["navigation_time"]) - float(left[key]["navigation_time"])
            for key in goal_keys),
        "mean_path_length_reduction_m_on_both_goal": _mean(
            float(right[key]["path_length"]) - float(left[key]["path_length"])
            for key in goal_keys),
    }


def evaluate(root: Any) -> Dict[str, Any]:
    root = Path(root)
    rows_by_method: Dict[str, List[Dict[str, Any]]] = {}
    keyed: Dict[str, Dict[Tuple[str, str, str], Dict[str, Any]]] = {}
    run_integrity = {}
    for method in METHODS:
        method_rows: List[Dict[str, Any]] = []
        for seed in SEEDS:
            run_id = "t12e_{}_seed{}".format(method, seed)
            run = root / "runs" / run_id
            report = yaml.safe_load((run / "t11_run_report.yaml").read_text(encoding="utf-8"))
            if report.get("task") != "T12" or report.get("passed") is not True:
                raise T12ResidualPairError("invalid run {}".format(run_id))
            rows = _read_csv(run / "episodes.csv")
            if len(rows) != 4 or len({row["scene_id"] for row in rows}) != 4:
                raise T12ResidualPairError("{} does not contain four unique scenes".format(run_id))
            returns = _return_by_episode(run)
            for row in rows:
                row["_return"] = returns[row["episode_id"]]
            method_rows.extend(rows)
            run_integrity[run_id] = {
                "valid": True,
                "episode_count": 4,
                "evaluation_policy": report.get("evaluation_policy"),
                "selected_timesteps": report.get("selected_timesteps"),
            }
        rows_by_method[method] = method_rows
        keyed[method] = {
            (row["training_seed"], row["seed"], row["scene_id"]): row
            for row in method_rows
        }
        if len(keyed[method]) != 8:
            raise T12ResidualPairError("duplicate pairing key for {}".format(method))

    pairing_complete = all(set(keyed[method]) == set(keyed[METHODS[0]])
                           for method in METHODS[1:])
    summaries = {method: _summary(rows) for method, rows in rows_by_method.items()}
    learned = _paired(keyed["selected_sac"], keyed["zero_residual"])
    engineering = _paired(keyed["selected_sac"], keyed["teb_tuned"])
    per_seed_summaries = {
        method: {
            str(seed): _summary([row for row in rows if int(row["training_seed"]) == seed])
            for seed in SEEDS
        }
        for method, rows in rows_by_method.items()
    }
    stable_return = all(
        learned["return_delta_by_training_seed"][str(seed)] > 0.0 for seed in SEEDS)
    stable_success = all(
        per_seed_summaries["selected_sac"][str(seed)]["success_rate"] >=
        per_seed_summaries["zero_residual"][str(seed)]["success_rate"]
        for seed in SEEDS)
    selected = summaries["selected_sac"]
    zero = summaries["zero_residual"]
    safety_no_regression = (
        selected["collision"] <= zero["collision"] and
        selected["emergency_stop"] <= zero["emergency_stop"] and
        selected["interface_fault"] <= zero["interface_fault"])
    gates = {
        "exact_24_episode_complete_pairing": pairing_complete and
            all(item["episode_count"] == 8 for item in summaries.values()),
        "run_integrity_complete": len(run_integrity) == 6,
        "selected_vs_zero_return_positive_each_training_seed": stable_return,
        "selected_success_not_lower_each_training_seed": stable_success,
        "selected_safety_not_worse_than_zero": safety_no_regression,
    }
    integrity_passed = gates["exact_24_episode_complete_pairing"] and gates["run_integrity_complete"]
    stable_gain = stable_return and stable_success and safety_no_regression
    return {
        "schema_version": "1.0", "task": "T12",
        "study": "residual_selected_zero_teb_tuned_small_paired_evaluation",
        "formal_result": False, "training_performed": False, "simulation_only": True,
        "integrity_passed": integrity_passed, "stable_learning_gain_supported": stable_gain,
        "methods": summaries, "per_training_seed": per_seed_summaries,
        "paired_effects": {"selected_minus_zero_residual": learned,
                           "selected_minus_teb_tuned": engineering},
        "gates": gates, "runs": run_integrity,
        "decision": ("eligible_for_budget_expansion_review_not_automatic_expansion"
                     if stable_gain else
                     "single_factor_amendment_required_before_new_small_budget_pilot"),
    }
