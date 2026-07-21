"""Unified, ROS-free episode evaluator for all five V2 foundation families."""

import csv
import hashlib
import math
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence

from .v2_scene import SCENE_FAMILIES, V2SceneError, canonical_sha256


TRACE_COLUMNS = (
    "stamp_s", "x_m", "y_m", "yaw_rad", "linear_velocity_mps",
    "angular_velocity_radps", "commanded_speed_mps", "clearance_m",
    "goal_distance_m", "collision", "goal_reached", "contact_count",
    "topology_id", "global_replan_count", "recovery_count", "gear",
    "predicted_ttc_s",
)
GEARS = ("REVERSE", "NEUTRAL", "FORWARD")


class V2EvaluationError(ValueError):
    """Raised for ambiguous traces, corrupted instances, or metric input drift."""


def _finite(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise V2EvaluationError("{} must be numeric".format(context))
    result = float(value)
    if not math.isfinite(result):
        raise V2EvaluationError("{} must be finite".format(context))
    return result


def _bool_text(value: str, context: str) -> bool:
    normalized = value.strip().lower()
    if normalized in ("true", "1"):
        return True
    if normalized in ("false", "0"):
        return False
    raise V2EvaluationError("{} must be true/false or 1/0".format(context))


def _integer_text(value: str, context: str) -> int:
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise V2EvaluationError("{} must be an integer".format(context)) from exc
    if result < 0:
        raise V2EvaluationError("{} must be non-negative".format(context))
    return result


def load_v2_trace(path: Any) -> List[Dict[str, Any]]:
    source = Path(path)
    try:
        with source.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            if tuple(reader.fieldnames or ()) != TRACE_COLUMNS:
                raise V2EvaluationError("trace columns/order drifted")
            raw_rows = list(reader)
    except OSError as exc:
        raise V2EvaluationError("cannot read trace: {}".format(exc)) from exc
    rows = []
    last_stamp = None
    for index, raw in enumerate(raw_rows):
        label = "trace[{}]".format(index)
        row = {
            key: _finite(float(raw[key]), label + "." + key)
            for key in (
                "stamp_s", "x_m", "y_m", "yaw_rad", "linear_velocity_mps",
                "angular_velocity_radps", "commanded_speed_mps", "clearance_m",
                "goal_distance_m",
            )
        }
        row["collision"] = _bool_text(raw["collision"], label + ".collision")
        row["goal_reached"] = _bool_text(raw["goal_reached"], label + ".goal_reached")
        for key in ("contact_count", "global_replan_count", "recovery_count"):
            row[key] = _integer_text(raw[key], label + "." + key)
        row["topology_id"] = raw["topology_id"].strip()
        row["gear"] = raw["gear"].strip()
        if row["gear"] not in GEARS:
            raise V2EvaluationError("{}.gear is invalid".format(label))
        ttc = raw["predicted_ttc_s"].strip()
        row["predicted_ttc_s"] = None if ttc == "" else _finite(
            float(ttc), label + ".predicted_ttc_s"
        )
        if row["clearance_m"] < 0.0 or row["goal_distance_m"] < 0.0:
            raise V2EvaluationError("{} distances must be non-negative".format(label))
        if row["predicted_ttc_s"] is not None and row["predicted_ttc_s"] < 0.0:
            raise V2EvaluationError("{} TTC must be non-negative".format(label))
        if last_stamp is not None and row["stamp_s"] <= last_stamp:
            raise V2EvaluationError("trace timestamps must be strictly increasing")
        if rows:
            for key in ("contact_count", "global_replan_count", "recovery_count"):
                if row[key] < rows[-1][key]:
                    raise V2EvaluationError("{} must be monotonic".format(key))
        last_stamp = row["stamp_s"]
        rows.append(row)
    if len(rows) < 2:
        raise V2EvaluationError("trace requires at least two rows")
    return rows


def _wrapped_delta(after: float, before: float) -> float:
    return math.atan2(math.sin(after - before), math.cos(after - before))


def _point_segment_distance(px: float, py: float, start: Sequence[float],
                            end: Sequence[float]) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    denominator = dx * dx + dy * dy
    if denominator <= 1e-12:
        return math.hypot(px - start[0], py - start[1])
    ratio = max(0.0, min(1.0, ((px - start[0]) * dx + (py - start[1]) * dy) /
                              denominator))
    return math.hypot(px - (start[0] + ratio * dx),
                      py - (start[1] + ratio * dy))


def _lateral_errors(rows: Sequence[Mapping[str, Any]], centerline: Sequence[Sequence[float]]) -> List[float]:
    return [
        min(_point_segment_distance(row["x_m"], row["y_m"], before, after)
            for before, after in zip(centerline, centerline[1:]))
        for row in rows
    ]


def _stop_count(rows: Sequence[Mapping[str, Any]]) -> int:
    moving = abs(rows[0]["linear_velocity_mps"]) >= 0.08
    stops = 0
    for row in rows[1:]:
        speed = abs(row["linear_velocity_mps"])
        if moving and speed <= 0.03:
            stops += 1
            moving = False
        elif not moving and speed >= 0.08:
            moving = True
    return stops


def _gear_metrics(rows: Sequence[Mapping[str, Any]]) -> Dict[str, int]:
    previous = next((row["gear"] for row in rows if row["gear"] != "NEUTRAL"), None)
    switches = violations = 0
    for row in rows:
        gear = row["gear"]
        if gear == "NEUTRAL" or previous is None or gear == previous:
            if previous is None and gear != "NEUTRAL":
                previous = gear
            continue
        switches += 1
        if abs(row["linear_velocity_mps"]) > 0.03:
            violations += 1
        previous = gear
    return {"gear_switch_count": switches, "gear_switch_while_moving_count": violations}


def evaluate_v2_episode(
    instance: Mapping[str, Any], rows: Sequence[Mapping[str, Any]], trace_sha256: str
) -> Dict[str, Any]:
    """Evaluate one episode without making its labels available to a policy."""

    if not isinstance(instance, dict) or set(instance) != {
        "schema_version", "generator", "scene", "instance_sha256"
    }:
        raise V2EvaluationError("compiled scene instance keys drifted")
    unhashed = {key: instance[key] for key in ("schema_version", "generator", "scene")}
    if canonical_sha256(unhashed) != instance["instance_sha256"]:
        raise V2EvaluationError("compiled scene instance hash mismatch")
    if not isinstance(trace_sha256, str) or len(trace_sha256) != 64:
        raise V2EvaluationError("raw trace SHA256 is required")
    scene = instance["scene"]
    family = scene.get("family")
    if family not in SCENE_FAMILIES:
        raise V2EvaluationError("scene family is invalid")
    if len(rows) < 2:
        raise V2EvaluationError("trace requires at least two rows")
    timestamps = [_finite(row["stamp_s"], "row.stamp_s") for row in rows]
    if any(after <= before for before, after in zip(timestamps, timestamps[1:])):
        raise V2EvaluationError("trace timestamps must be strictly increasing")

    navigation_time = timestamps[-1] - timestamps[0]
    path_length = 0.0
    reverse_distance = 0.0
    weighted_speed = 0.0
    for before, after in zip(rows, rows[1:]):
        dt = after["stamp_s"] - before["stamp_s"]
        distance = math.hypot(after["x_m"] - before["x_m"], after["y_m"] - before["y_m"])
        path_length += distance
        if before["gear"] == "REVERSE" or before["linear_velocity_mps"] < -0.01:
            reverse_distance += distance
        weighted_speed += 0.5 * (
            abs(before["linear_velocity_mps"]) + abs(after["linear_velocity_mps"])
        ) * dt
    minimum_clearance = min(row["clearance_m"] for row in rows)
    collision = any(row["collision"] or row["contact_count"] > 0 for row in rows)
    stopped = abs(rows[-1]["linear_velocity_mps"]) <= scene["success"]["stopped_speed_max_mps"]
    success = bool(rows[-1]["goal_reached"] and stopped and not collision)
    if collision:
        termination = "COLLISION"
    elif success:
        termination = "SUCCESS"
    elif navigation_time >= scene["timeout_s"]:
        termination = "TIMEOUT"
    else:
        termination = "ABORTED"
    common = {
        "success": success,
        "collision": collision,
        "navigation_time_s": navigation_time,
        "path_length_m": path_length,
        "mean_abs_speed_mps": weighted_speed / navigation_time,
        "minimum_clearance_m": minimum_clearance,
        "stop_count": _stop_count(rows),
        "reverse_distance_m": reverse_distance,
    }
    centerline = scene["layout"]["reference_centerline"]
    lateral = _lateral_errors(rows, centerline)
    heading_variation = sum(abs(_wrapped_delta(after["yaw_rad"], before["yaw_rad"]))
                            for before, after in zip(rows, rows[1:]))
    net_heading = abs(_wrapped_delta(rows[-1]["yaw_rad"], rows[0]["yaw_rad"]))
    heading_oscillation = max(0.0, heading_variation - net_heading)
    if family == "CRUISE":
        decelerations = 0
        armed = True
        for before, after in zip(rows, rows[1:]):
            drop = abs(before["linear_velocity_mps"]) - abs(after["linear_velocity_mps"])
            unnecessary = (
                drop >= 0.10 and after["commanded_speed_mps"] >= 0.50
                and after["clearance_m"] >= 2.0
                and (after["predicted_ttc_s"] is None or after["predicted_ttc_s"] >= 5.0)
            )
            if unnecessary and armed:
                decelerations += 1
                armed = False
            elif drop <= 0.02:
                armed = True
        family_metrics = {
            "lateral_rms_m": math.sqrt(sum(value * value for value in lateral) / len(lateral)),
            "heading_oscillation_rad": heading_oscillation,
            "unnecessary_deceleration_count": decelerations,
        }
    elif family == "DYNAMIC":
        ttc_values = [row["predicted_ttc_s"] for row in rows
                      if row["predicted_ttc_s"] is not None]
        interaction = [row["clearance_m"] for row in rows
                       if row["predicted_ttc_s"] is not None and row["predicted_ttc_s"] <= 5.0]
        family_metrics = {
            "minimum_predicted_ttc_s": min(ttc_values) if ttc_values else None,
            "stop_count": common["stop_count"],
            "interaction_clearance_m": min(interaction) if interaction else minimum_clearance,
        }
    elif family == "STATIC_DENSE":
        topology = [row["topology_id"] for row in rows if row["topology_id"]]
        family_metrics = {
            "topology_switch_count": sum(after != before for before, after in zip(topology, topology[1:])),
            "global_replan_count": rows[-1]["global_replan_count"],
            "recovery_count": rows[-1]["recovery_count"],
        }
    elif family == "CORRIDOR":
        family_metrics = {
            "lateral_rms_m": math.sqrt(sum(value * value for value in lateral) / len(lateral)),
            "heading_oscillation_rad": heading_oscillation,
            "emergency_contact_count": rows[-1]["contact_count"],
        }
    else:
        family_metrics = _gear_metrics(rows)
        family_metrics["reverse_distance_m"] = reverse_distance
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "evaluator_id": "fam_teb_v2_02_evaluator_1",
        "formal_result": False,
        "runtime_ready": False,
        "scene_id": scene["scene_id"],
        "family": family,
        "split": scene["split"],
        "seed": scene["seed"],
        "instance_sha256": instance["instance_sha256"],
        "raw_trace_sha256": trace_sha256,
        "termination": termination,
        "metrics": {"common": common, "family": family_metrics},
    }


def trace_sha256(path: Any) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
