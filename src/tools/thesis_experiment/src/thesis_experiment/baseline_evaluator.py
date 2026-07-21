"""Strict, algorithm-agnostic T08 baseline matrix evaluator."""

import csv
import math
from pathlib import Path
from statistics import median
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import yaml


class BaselineEvaluationError(ValueError):
    """Raised when a baseline contract or result matrix is not auditable."""


ALGORITHMS = ("Fixed-DWA", "TEB-Default", "TEB-Tuned", "Rule-TEB")
METRICS = (
    "navigation_time", "path_length", "smoothness", "min_obstacle_distance",
    "near_collision_time_ratio", "parameter_total_variation",
)


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise BaselineEvaluationError("{} must be numeric".format(label))
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise BaselineEvaluationError("{} must be numeric".format(label))
    if not math.isfinite(number):
        raise BaselineEvaluationError("{} must be finite".format(label))
    return number


def _boolean(value: Any, label: str) -> bool:
    if value in (True, "true", "True", "1", 1):
        return True
    if value in (False, "false", "False", "0", 0):
        return False
    raise BaselineEvaluationError("{} must be a bool".format(label))


def load_baseline_contract(path: Any) -> Mapping[str, Any]:
    source = Path(path)
    try:
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise BaselineEvaluationError("cannot read baseline contract: {}".format(exc))
    validate_baseline_contract(data)
    return data


def validate_baseline_contract(data: Any) -> Mapping[str, Any]:
    if not isinstance(data, dict):
        raise BaselineEvaluationError("baseline contract must be a mapping")
    if data.get("schema_version") != "1.0" or data.get("status") != "frozen_gazebo_t08":
        raise BaselineEvaluationError("baseline contract must be frozen_gazebo_t08 schema 1.0")
    if data.get("simulation_only") is not True or data.get("real_vehicle_use_forbidden") is not True:
        raise BaselineEvaluationError("T08 contract must be simulation-only and forbid real use")
    algorithms = data.get("algorithms")
    if not isinstance(algorithms, list) or tuple(item.get("name") for item in algorithms) != ALGORITHMS:
        raise BaselineEvaluationError("algorithms must use the frozen T08 order")
    if any(not isinstance(item, dict) or not item.get("config_version") for item in algorithms):
        raise BaselineEvaluationError("every algorithm needs an immutable config_version")
    matrix = data.get("evaluation_matrix")
    if not isinstance(matrix, dict) or not isinstance(matrix.get("scenes"), list):
        raise BaselineEvaluationError("evaluation_matrix.scenes is required")
    if not matrix.get("seeds") or len(set(matrix["seeds"])) != len(matrix["seeds"]):
        raise BaselineEvaluationError("evaluation seeds must be non-empty and unique")
    scene_ids = []
    for scene in matrix["scenes"]:
        if not isinstance(scene, dict):
            raise BaselineEvaluationError("scene must be a mapping")
        required = ("scene_id", "split", "layout", "goal", "timeout_s")
        if any(key not in scene for key in required):
            raise BaselineEvaluationError("scene lacks required fields")
        if scene["layout"] not in ("clear", "obstacle", "corridor"):
            raise BaselineEvaluationError("unknown scene layout")
        scene_ids.append(scene["scene_id"])
    if len(scene_ids) != len(set(scene_ids)):
        raise BaselineEvaluationError("scene IDs must be unique")
    if data.get("reference_algorithm") != "TEB-Default":
        raise BaselineEvaluationError("T08 paired reference must be TEB-Default")
    return data


def read_episode_rows(paths: Iterable[Any]) -> List[Dict[str, str]]:
    rows = []
    for path in paths:
        source = Path(path)
        try:
            with source.open("r", encoding="utf-8", newline="") as handle:
                rows.extend(dict(row) for row in csv.DictReader(handle))
        except OSError as exc:
            raise BaselineEvaluationError("cannot read {}: {}".format(source, exc))
    return rows


def _expected_keys(contract: Mapping[str, Any]) -> Tuple[Tuple[str, str, int], ...]:
    return tuple(
        (algorithm, scene["scene_id"], int(seed))
        for algorithm in ALGORITHMS
        for scene in contract["evaluation_matrix"]["scenes"]
        for seed in contract["evaluation_matrix"]["seeds"]
    )


def evaluate_baselines(
    rows: Sequence[Mapping[str, Any]], contract: Mapping[str, Any]
) -> Dict[str, Any]:
    """Validate a complete paired matrix and compute deterministic summaries."""

    validate_baseline_contract(contract)
    expected = set(_expected_keys(contract))
    indexed: Dict[Tuple[str, str, int], Mapping[str, Any]] = {}
    for position, row in enumerate(rows, 1):
        try:
            key = (str(row["algorithm"]), str(row["scene_id"]), int(row["seed"]))
        except (KeyError, TypeError, ValueError):
            raise BaselineEvaluationError("row {} lacks a valid algorithm/scene/seed".format(position))
        if key not in expected:
            raise BaselineEvaluationError("unexpected evaluation key {}".format(key))
        if key in indexed:
            raise BaselineEvaluationError("duplicate evaluation key {}".format(key))
        if _boolean(row.get("success"), "{}.success".format(key)) != (
                str(row.get("termination_reason")) == "goal"):
            raise BaselineEvaluationError("{} success/termination mismatch".format(key))
        for metric in METRICS:
            _finite(row.get(metric), "{}.{}".format(key, metric))
        indexed[key] = row
    missing = sorted(expected - set(indexed))
    if missing:
        raise BaselineEvaluationError("incomplete baseline matrix; missing {}".format(missing))

    by_algorithm = {}
    for algorithm in ALGORITHMS:
        selected = [indexed[key] for key in sorted(indexed) if key[0] == algorithm]
        successes = [_boolean(row["success"], "success") for row in selected]
        collisions = [_boolean(row["collision"], "collision") for row in selected]
        by_algorithm[algorithm] = {
            "episode_count": len(selected),
            "success_count": sum(successes),
            "failure_count": len(selected) - sum(successes),
            "success_rate": sum(successes) / float(len(selected)),
            "collision_count": sum(collisions),
            "termination_reasons": {
                reason: sum(str(row["termination_reason"]) == reason for row in selected)
                for reason in sorted(set(str(row["termination_reason"]) for row in selected))
            },
            "metrics": {
                metric: {
                    "mean": sum(_finite(row[metric], metric) for row in selected) / len(selected),
                    "median": median(_finite(row[metric], metric) for row in selected),
                }
                for metric in METRICS
            },
        }

    reference = contract["reference_algorithm"]
    paired = {}
    for algorithm in ALGORITHMS:
        if algorithm == reference:
            continue
        pairs = []
        for scene in contract["evaluation_matrix"]["scenes"]:
            for seed in contract["evaluation_matrix"]["seeds"]:
                key = (algorithm, scene["scene_id"], int(seed))
                ref_key = (reference, scene["scene_id"], int(seed))
                row, ref = indexed[key], indexed[ref_key]
                pairs.append({
                    "scene_id": scene["scene_id"], "seed": int(seed),
                    "success_delta": int(_boolean(row["success"], "success")) -
                                     int(_boolean(ref["success"], "success")),
                    "navigation_time_delta_s": _finite(row["navigation_time"], "navigation_time") -
                                               _finite(ref["navigation_time"], "navigation_time"),
                    "path_length_delta_m": _finite(row["path_length"], "path_length") -
                                             _finite(ref["path_length"], "path_length"),
                    "clearance_delta_m": _finite(row["min_obstacle_distance"], "clearance") -
                                          _finite(ref["min_obstacle_distance"], "clearance"),
                })
        paired[algorithm] = {"reference": reference, "pair_count": len(pairs), "pairs": pairs}

    return {
        "schema_version": "1.0", "status": "valid",
        "complete_matrix": True, "failure_rows_retained": True,
        "episode_count": len(rows), "expected_episode_count": len(expected),
        "algorithms": by_algorithm, "paired_against_reference": paired,
    }

