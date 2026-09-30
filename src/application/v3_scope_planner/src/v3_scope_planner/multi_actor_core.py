"""Deterministic V3-13 multi-actor candidate bank and hard verifier."""

import copy
import hashlib
import json
import math
from typing import Any, Dict, Mapping, Sequence


class MultiActorPlannerError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise MultiActorPlannerError(message)


def finite(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def digest(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _point(
    time_s: float,
    x_m: float,
    y_m: float,
    yaw_rad: float,
    speed_mps: float,
    acceleration_mps2: float,
    steering_angle_rad: float,
    steering_rate_radps: float,
    curvature_inv_m: float,
) -> Dict[str, float]:
    return {
        "time_from_start_s": round(float(time_s), 9),
        "x_m": round(float(x_m), 12),
        "y_m": round(float(y_m), 12),
        "yaw_rad": round(float(yaw_rad), 12),
        "longitudinal_velocity_mps": round(float(speed_mps), 12),
        "longitudinal_acceleration_mps2": round(
            float(acceleration_mps2), 12
        ),
        "steering_angle_rad": round(float(steering_angle_rad), 12),
        "steering_rate_radps": round(float(steering_rate_radps), 12),
        "curvature_inv_m": round(float(curvature_inv_m), 12),
    }


def _reference_steering(
    x_m: float, direction: float, generator: Mapping[str, Any]
) -> float:
    outbound = [float(x) for x in generator["outbound_x_range_m"]]
    returning = [float(x) for x in generator["return_x_range_m"]]
    peak = direction * float(generator["steering_peak_rad"])
    if outbound[0] <= x_m < outbound[1]:
        phase = (x_m - outbound[0]) / (outbound[1] - outbound[0])
        return peak * math.sin(2.0 * math.pi * phase)
    if returning[0] <= x_m < returning[1]:
        phase = (x_m - returning[0]) / (returning[1] - returning[0])
        return -peak * math.sin(2.0 * math.pi * phase)
    return 0.0


def _generate_candidate(
    snapshot: Mapping[str, Any],
    candidate_spec: Mapping[str, Any],
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    generator = contract["candidate_generator"]
    vehicle = contract["vehicle"]
    dt = float(generator["sample_period_s"])
    count = int(round(float(generator["horizon_s"]) / dt))
    x_m = float(snapshot["ego"]["x_m"])
    y_m = float(snapshot["ego"]["y_m"])
    yaw_rad = float(snapshot["ego"]["yaw_rad"])
    speed_mps = float(snapshot["ego"]["speed_mps"])
    steering_rad = 0.0
    kind = candidate_spec["kind"]
    direction = float(candidate_spec.get("direction", 0.0))
    target_speed = float(candidate_spec["target_speed_mps"])
    points = []
    for index in range(count + 1):
        curvature = math.tan(steering_rad) / float(vehicle["wheelbase_m"])
        if kind == "spatial":
            desired_steering = _reference_steering(
                x_m, direction, generator
            )
        else:
            desired_steering = 0.0
        maximum_steering_delta = (
            float(vehicle["maximum_steering_rate_radps"]) * dt
        )
        next_steering = steering_rad + max(
            -maximum_steering_delta,
            min(
                maximum_steering_delta,
                desired_steering - steering_rad,
            ),
        )
        next_steering = max(
            -float(vehicle["maximum_steering_angle_rad"]),
            min(
                float(vehicle["maximum_steering_angle_rad"]),
                next_steering,
            ),
        )
        if target_speed > speed_mps:
            next_speed = min(
                target_speed,
                speed_mps
                + float(vehicle["maximum_acceleration_mps2"]) * dt,
            )
        elif target_speed < speed_mps:
            next_speed = max(
                target_speed,
                speed_mps
                - float(vehicle["maximum_deceleration_mps2"]) * dt,
            )
        else:
            next_speed = speed_mps
        acceleration = (
            (next_speed - speed_mps) / dt if index < count else 0.0
        )
        steering_rate = (
            (next_steering - steering_rad) / dt
            if index < count
            else 0.0
        )
        points.append(
            _point(
                index * dt,
                x_m,
                y_m,
                yaw_rad,
                speed_mps,
                acceleration,
                steering_rad,
                steering_rate,
                curvature,
            )
        )
        if index == count:
            break
        average_speed = 0.5 * (speed_mps + next_speed)
        average_steering = 0.5 * (steering_rad + next_steering)
        ds = average_speed * dt
        x_m += ds * math.cos(yaw_rad)
        y_m += ds * math.sin(yaw_rad)
        yaw_rad += (
            ds
            * math.tan(average_steering)
            / float(vehicle["wheelbase_m"])
        )
        speed_mps = next_speed
        steering_rad = next_steering
    value = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "candidate_id": candidate_spec["candidate_id"],
        "generator_id": generator["generator_id"],
        "kind": kind,
        "certified_stop": bool(candidate_spec["certified_stop"]),
        "target_speed_mps": target_speed,
        "points": points,
    }
    value["candidate_digest"] = digest(value)
    return value


def generate_candidate_set(
    snapshot: Mapping[str, Any], contract: Mapping[str, Any]
) -> Dict[str, Any]:
    candidates = [
        _generate_candidate(snapshot, spec, contract)
        for spec in contract["candidate_generator"]["candidates"]
    ]
    identifiers = [item["candidate_id"] for item in candidates]
    require(len(identifiers) == len(set(identifiers)), "DUPLICATE_CANDIDATE_ID")
    require(
        sum(item["certified_stop"] for item in candidates) == 1,
        "CERTIFIED_STOP_CARDINALITY",
    )
    value = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "world_model_seq": int(snapshot["world_model_seq"]),
        "generator_id": contract["candidate_generator"]["generator_id"],
        "candidates": candidates,
    }
    value["candidate_set_digest"] = digest(value)
    return value


def validate_candidate(
    candidate: Mapping[str, Any], contract: Mapping[str, Any]
) -> Sequence[str]:
    failures = []
    vehicle = contract["vehicle"]
    generator = contract["candidate_generator"]
    dt = float(generator["sample_period_s"])
    expected = int(round(float(generator["horizon_s"]) / dt)) + 1
    points = candidate.get("points", [])
    if len(points) != expected:
        return ["POINT_COUNT_INVALID"]
    fields = (
        "time_from_start_s",
        "x_m",
        "y_m",
        "yaw_rad",
        "longitudinal_velocity_mps",
        "longitudinal_acceleration_mps2",
        "steering_angle_rad",
        "steering_rate_radps",
        "curvature_inv_m",
    )
    if any(
        not finite(point.get(field))
        for point in points
        for field in fields
    ):
        return ["NONFINITE_KINEMATICS"]
    body = copy.deepcopy(dict(candidate))
    claimed = body.pop("candidate_digest", "")
    if digest(body) != claimed:
        failures.append("CANDIDATE_DIGEST_MISMATCH")
    for index, point in enumerate(points):
        if abs(float(point["time_from_start_s"]) - index * dt) > 1e-8:
            failures.append("TIME_GRID_INVALID")
            break
        if (
            float(point["longitudinal_velocity_mps"]) < -1e-9
            or float(point["longitudinal_velocity_mps"])
            > float(vehicle["maximum_speed_mps"]) + 1e-9
            or float(point["longitudinal_acceleration_mps2"])
            > float(vehicle["maximum_acceleration_mps2"]) + 1e-9
            or float(point["longitudinal_acceleration_mps2"])
            < -float(vehicle["maximum_deceleration_mps2"]) - 1e-9
            or abs(float(point["steering_angle_rad"]))
            > float(vehicle["maximum_steering_angle_rad"]) + 1e-9
            or abs(float(point["steering_rate_radps"]))
            > float(vehicle["maximum_steering_rate_radps"]) + 1e-9
            or abs(float(point["curvature_inv_m"]))
            > float(vehicle["maximum_curvature_inv_m"]) + 1e-9
        ):
            failures.append("KINEMATIC_BOUND")
            break
    residual_limit = float(generator["kinematic_residual_max_m_rad"])
    for left, right in zip(points[:-1], points[1:]):
        average_speed = 0.5 * (
            float(left["longitudinal_velocity_mps"])
            + float(right["longitudinal_velocity_mps"])
        )
        average_steering = 0.5 * (
            float(left["steering_angle_rad"])
            + float(right["steering_angle_rad"])
        )
        ds = average_speed * dt
        predicted_x = float(left["x_m"]) + ds * math.cos(
            float(left["yaw_rad"])
        )
        predicted_y = float(left["y_m"]) + ds * math.sin(
            float(left["yaw_rad"])
        )
        predicted_yaw = float(left["yaw_rad"]) + (
            ds
            * math.tan(average_steering)
            / float(vehicle["wheelbase_m"])
        )
        residual = max(
            abs(predicted_x - float(right["x_m"])),
            abs(predicted_y - float(right["y_m"])),
            abs(predicted_yaw - float(right["yaw_rad"])),
        )
        if residual > residual_limit:
            failures.append("KINEMATIC_RESIDUAL")
            break
    return sorted(set(failures))


def infer_observable_mode(
    coordinate_speed_mps: float, contract: Mapping[str, Any]
) -> str:
    gate = contract["predictor"]["mode_inference"]
    cautious_max = float(gate["cautious_max_speed_mps"])
    hesitant_max = float(gate["hesitant_max_speed_mps"])
    ambiguity = float(gate["ambiguity_half_width_mps"])
    require(finite(coordinate_speed_mps), "NONFINITE_MODE_FEATURE")
    speed = float(coordinate_speed_mps)
    if speed <= cautious_max - ambiguity:
        return "cautious"
    if cautious_max + ambiguity <= speed <= hesitant_max - ambiguity:
        return "hesitant"
    if speed >= hesitant_max + ambiguity:
        return "assertive"
    raise MultiActorPlannerError("MODE_AMBIGUOUS")


def predict_actor_tube(
    actor: Mapping[str, Any],
    candidate: Mapping[str, Any],
    model: Mapping[str, Any],
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    required = (
        "track_id",
        "coordinate_m",
        "coordinate_speed_mps",
        "path_angle_deg",
        "path_origin_x_m",
        "path_origin_y_m",
        "age_s",
    )
    require(
        all(finite(actor.get(field)) for field in required if field != "track_id"),
        "NONFINITE_ACTOR_TRACK",
    )
    require("actor_policy" not in actor, "HIDDEN_POLICY_RUNTIME_LEAK")
    require("hidden_type" not in actor, "HIDDEN_POLICY_RUNTIME_LEAK")
    mode = infer_observable_mode(
        float(actor["coordinate_speed_mps"]), contract
    )
    require(mode in model["heads"], "UNSUPPORTED_OBSERVABLE_MODE:" + mode)
    head = model["heads"][mode][candidate["candidate_id"]]
    initial_coordinate = float(actor["coordinate_m"])
    initial_speed = max(0.0, float(actor["coordinate_speed_mps"]))
    speed_scale = float(head["speed_scale"])
    speed_bias = float(head["speed_bias_mps"])
    effective_speed = max(0.0, initial_speed * speed_scale + speed_bias)
    radius = float(model["calibration"]["coordinate_radius_m"])
    growth = float(model["calibration"]["coordinate_growth_mps"])
    samples = []
    for point in candidate["points"]:
        time_s = float(point["time_from_start_s"])
        center = initial_coordinate + effective_speed * time_s
        half_width = radius + growth * time_s
        samples.append(
            {
                "time_s": time_s,
                "lower_coordinate_m": center - half_width,
                "upper_coordinate_m": center + half_width,
            }
        )
    value = {
        "track_id": actor["track_id"],
        "candidate_id": candidate["candidate_id"],
        "observable_mode": mode,
        "effective_speed_mps": effective_speed,
        "samples": samples,
    }
    value["tube_digest"] = digest(value)
    return value


def _point_segment_distance(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    dx = bx - ax
    dy = by - ay
    denominator = dx * dx + dy * dy
    if denominator <= 1e-18:
        return math.hypot(px - ax, py - ay)
    ratio = ((px - ax) * dx + (py - ay) * dy) / denominator
    ratio = max(0.0, min(1.0, ratio))
    return math.hypot(px - (ax + ratio * dx), py - (ay + ratio * dy))


def _rectangle_distance(
    x_m: float, y_m: float, rectangle: Mapping[str, Any]
) -> float:
    dx = max(
        float(rectangle["x_min_m"]) - x_m,
        0.0,
        x_m - float(rectangle["x_max_m"]),
    )
    dy = max(
        float(rectangle["y_min_m"]) - y_m,
        0.0,
        y_m - float(rectangle["y_max_m"]),
    )
    return math.hypot(dx, dy)


def evaluate_candidate(
    snapshot: Mapping[str, Any],
    candidate: Mapping[str, Any],
    model: Mapping[str, Any],
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    failures = list(validate_candidate(candidate, contract))
    verifier = contract["hard_verifier"]
    if candidate["scene_snapshot_id"] != snapshot["scene_snapshot_id"]:
        failures.append("SCENE_SNAPSHOT_MISMATCH")
    if not all(
        snapshot.get("health", {}).get(field) is True
        for field in verifier["required_health_fields"]
    ):
        failures.append("HEALTH_INVALID")
    if any(
        float(actor["age_s"]) > float(verifier["maximum_track_age_s"])
        for actor in snapshot["actors"]
    ):
        failures.append("ACTOR_TRACK_STALE")
    actor_tubes = []
    minimum_dynamic_clearance = float("inf")
    for actor in sorted(snapshot["actors"], key=lambda item: item["track_id"]):
        try:
            tube = predict_actor_tube(actor, candidate, model, contract)
            actor_tubes.append(tube)
            angle = math.radians(float(actor["path_angle_deg"]))
            direction_x = math.cos(angle)
            direction_y = math.sin(angle)
            origin_x = float(actor["path_origin_x_m"])
            origin_y = float(actor["path_origin_y_m"])
            for point, sample in zip(candidate["points"], tube["samples"]):
                lower = float(sample["lower_coordinate_m"])
                upper = float(sample["upper_coordinate_m"])
                clearance = _point_segment_distance(
                    float(point["x_m"]),
                    float(point["y_m"]),
                    origin_x + lower * direction_x,
                    origin_y + lower * direction_y,
                    origin_x + upper * direction_x,
                    origin_y + upper * direction_y,
                ) - float(verifier["combined_actor_ego_radius_m"])
                minimum_dynamic_clearance = min(
                    minimum_dynamic_clearance, clearance
                )
        except (MultiActorPlannerError, KeyError, TypeError, ValueError) as error:
            failures.append("ACTOR_TUBE:" + str(error))
    if minimum_dynamic_clearance < 0.0:
        failures.append("DYNAMIC_OCCUPANCY_INTERSECTION")
    scene = snapshot["scene"]
    minimum_static_clearance = float("inf")
    half_width = scene.get("corridor_half_width_m")
    ego_half_width = float(contract["vehicle"]["ego_half_width_m"])
    if half_width is not None:
        for point in candidate["points"]:
            clearance = (
                float(half_width)
                - abs(float(point["y_m"]))
                - ego_half_width
            )
            minimum_static_clearance = min(
                minimum_static_clearance, clearance
            )
        if minimum_static_clearance < 0.0:
            failures.append("CORRIDOR_CONTAINMENT")
    for rectangle in scene.get("static_rectangles", []):
        for point in candidate["points"]:
            clearance = _rectangle_distance(
                float(point["x_m"]), float(point["y_m"]), rectangle
            ) - float(contract["vehicle"]["ego_circumscribed_radius_m"])
            minimum_static_clearance = min(
                minimum_static_clearance, clearance
            )
        if minimum_static_clearance < 0.0:
            failures.append("STATIC_OBSTACLE_INTERSECTION")
    if math.isinf(minimum_static_clearance):
        minimum_static_clearance = 1000.0
    if math.isinf(minimum_dynamic_clearance):
        minimum_dynamic_clearance = 1000.0
    terminal = candidate["points"][-1]
    progress = float(terminal["x_m"]) - float(
        candidate["points"][0]["x_m"]
    )
    acceleration_cost = sum(
        abs(float(point["longitudinal_acceleration_mps2"]))
        for point in candidate["points"]
    ) / len(candidate["points"])
    steering_cost = sum(
        abs(float(point["steering_angle_rad"]))
        for point in candidate["points"]
    ) / len(candidate["points"])
    clearance = min(minimum_dynamic_clearance, minimum_static_clearance)
    ranker = contract["analytic_ranker"]
    score = (
        float(ranker["progress_weight"]) * progress
        + float(ranker["clearance_weight"]) * min(clearance, 3.0)
        - float(ranker["acceleration_cost_weight"]) * acceleration_cost
        - float(ranker["steering_cost_weight"]) * steering_cost
    )
    value = {
        "candidate_id": candidate["candidate_id"],
        "accepted": not failures,
        "verdict": "ACCEPTED" if not failures else "REJECTED",
        "rejection_codes": sorted(set(failures)),
        "minimum_dynamic_clearance_m": minimum_dynamic_clearance,
        "minimum_static_clearance_m": minimum_static_clearance,
        "minimum_clearance_m": clearance,
        "progress_m": progress,
        "acceleration_cost": acceleration_cost,
        "steering_cost": steering_cost,
        "analytic_score": score,
        "actor_tubes": actor_tubes,
    }
    value["evaluation_digest"] = digest(value)
    return value


def plan(
    snapshot: Mapping[str, Any],
    model: Mapping[str, Any],
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    require(
        int(contract["minimum_actor_count"])
        <= len(snapshot.get("actors", []))
        <= int(contract["maximum_actor_count"]),
        "ACTOR_COUNT_OUT_OF_SCOPE",
    )
    track_ids = [actor["track_id"] for actor in snapshot["actors"]]
    require(len(track_ids) == len(set(track_ids)), "DUPLICATE_TRACK_ID")
    require(
        all(
            "actor_policy" not in actor and "hidden_type" not in actor
            for actor in snapshot["actors"]
        ),
        "HIDDEN_POLICY_RUNTIME_LEAK",
    )
    candidate_set = generate_candidate_set(snapshot, contract)
    evaluations = [
        evaluate_candidate(snapshot, candidate, model, contract)
        for candidate in candidate_set["candidates"]
    ]
    accepted = [
        item
        for item in evaluations
        if item["accepted"]
        and not next(
            candidate
            for candidate in candidate_set["candidates"]
            if candidate["candidate_id"] == item["candidate_id"]
        )["certified_stop"]
    ]
    fallback_used = False
    if accepted:
        selected = sorted(
            accepted,
            key=lambda item: (
                -float(item["analytic_score"]),
                item["candidate_id"],
            ),
        )[0]
    else:
        stop_ids = {
            candidate["candidate_id"]
            for candidate in candidate_set["candidates"]
            if candidate["certified_stop"]
        }
        safe_stops = [
            item
            for item in evaluations
            if item["accepted"] and item["candidate_id"] in stop_ids
        ]
        require(safe_stops, "NO_SAFE_CANDIDATE_OR_CERTIFIED_STOP")
        selected = sorted(safe_stops, key=lambda item: item["candidate_id"])[0]
        fallback_used = True
    decision = {
        "policy_id": contract["analytic_ranker"]["ranker_id"],
        "selected_candidate_id": selected["candidate_id"],
        "selection_propensity": 1.0,
        "safety_fallback_used": fallback_used,
        "candidate_set_digest": candidate_set["candidate_set_digest"],
        "world_model_actor_order_invariant": True,
    }
    decision["decision_digest"] = digest(decision)
    return {
        "candidate_set": candidate_set,
        "evaluations": evaluations,
        "decision": decision,
    }


def validate_cloned_intervention_block(
    rows: Sequence[Mapping[str, Any]],
    candidate_ids: Sequence[str],
) -> Dict[str, Any]:
    expected = set(candidate_ids)
    require(len(rows) == len(expected), "CLONE_ROW_COUNT")
    require(
        {row["candidate_id"] for row in rows} == expected,
        "CLONE_ACTION_SUPPORT",
    )
    initial_digests = {row["initial_state_digest"] for row in rows}
    require(len(initial_digests) == 1, "CLONE_INITIAL_STATE_MISMATCH")
    propensities = [float(row["behavior_propensity"]) for row in rows]
    require(all(value > 0.0 for value in propensities), "ZERO_PROPENSITY")
    require(abs(sum(propensities) - 1.0) <= 1e-12, "PROPENSITY_SUM")
    require(
        all(row.get("hidden_policy_runtime_input") is False for row in rows),
        "HIDDEN_POLICY_RUNTIME_LEAK",
    )
    require(
        all(
            row.get("evaluator_hidden_type") is not None
            for row in rows
        ),
        "EVALUATOR_STRATUM_ABSENT",
    )
    value = {
        "row_count": len(rows),
        "candidate_support_count": len(expected),
        "minimum_propensity": min(propensities),
        "initial_state_digest": next(iter(initial_digests)),
        "full_action_support": True,
        "hidden_label_firewall": True,
    }
    value["block_digest"] = digest(value)
    return value
