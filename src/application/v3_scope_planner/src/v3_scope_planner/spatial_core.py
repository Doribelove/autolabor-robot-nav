"""Deterministic spatial-candidate V3 planner without mutating frozen core.py."""

import copy
import math
from typing import Any, Dict, Mapping, Sequence

from . import core


PlannerError = core.PlannerError
require = core.require
finite = core.finite
digest = core.digest


def infer_mode(speed: float, contract: Mapping[str, Any]) -> str:
    gate = contract["predictor_runtime"]["mode_inference"]
    lower = float(gate["cautious_hesitant_threshold_mps"])
    upper = float(gate["hesitant_assertive_threshold_mps"])
    ambiguity = float(gate["ambiguity_half_width_mps"])
    require(finite(speed), "NONFINITE_MODE_FEATURE")
    if speed < lower - ambiguity:
        return "cautious"
    if lower + ambiguity < speed < upper - ambiguity:
        return "hesitant"
    if speed > upper + ambiguity:
        return "assertive"
    raise PlannerError("MODE_AMBIGUOUS")


def _point(
    time_s, x, y, yaw, speed, acceleration, steering, steering_rate, curvature
):
    return {
        "time_from_start_s": round(float(time_s), 9),
        "x_m": round(float(x), 12),
        "y_m": round(float(y), 12),
        "yaw_rad": round(float(yaw), 12),
        "longitudinal_velocity_mps": round(float(speed), 12),
        "longitudinal_acceleration_mps2": round(float(acceleration), 12),
        "steering_angle_rad": round(float(steering), 12),
        "steering_rate_radps": round(float(steering_rate), 12),
        "curvature_inv_m": round(float(curvature), 12),
    }


def _longitudinal_candidate(snapshot, contract, candidate_id, target_speed,
                            certified_stop=False, cleared_straight=False):
    generator = contract["candidate_generator"]
    dt = float(generator["sample_period_s"])
    count = int(round(float(generator["decision_horizon_s"]) / dt))
    x = float(snapshot["ego"]["x_m"])
    y = float(snapshot["ego"]["y_m"])
    yaw = float(snapshot["ego"]["yaw_rad"])
    speed = float(snapshot["ego"]["speed_mps"])
    points = []
    for index in range(count + 1):
        if target_speed > speed:
            next_speed = min(
                target_speed,
                speed + float(generator["maximum_acceleration_mps2"]) * dt,
            )
        elif target_speed < speed:
            next_speed = max(
                target_speed,
                speed - float(generator["maximum_deceleration_mps2"]) * dt,
            )
        else:
            next_speed = speed
        acceleration = (next_speed - speed) / dt if index < count else 0.0
        points.append(
            _point(index * dt, x, y, yaw, speed, acceleration, 0.0, 0.0, 0.0)
        )
        if index == count:
            break
        average_speed = 0.5 * (speed + next_speed)
        x += average_speed * math.cos(yaw) * dt
        y += average_speed * math.sin(yaw) * dt
        speed = next_speed
    value = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "candidate_id": candidate_id,
        "generator_id": generator["generator_id"],
        "certified_stop": bool(certified_stop),
        "cleared_straight": bool(cleared_straight),
        "target_speed_mps": float(target_speed),
        "points": points,
    }
    value["candidate_digest"] = digest(value)
    return value


def _reference_steering(x_m, generator):
    outbound = [float(x) for x in generator["outbound_x_range_m"]]
    returning = [float(x) for x in generator["return_x_range_m"]]
    peak = float(generator["steering_peak_rad"])
    if outbound[0] <= x_m < outbound[1]:
        phase = (x_m - outbound[0]) / (outbound[1] - outbound[0])
        return peak * math.sin(2.0 * math.pi * phase)
    if returning[0] <= x_m < returning[1]:
        phase = (x_m - returning[0]) / (returning[1] - returning[0])
        return -peak * math.sin(2.0 * math.pi * phase)
    return 0.0


def _spatial_candidate(snapshot, contract):
    generator = contract["candidate_generator"]
    spatial = contract["spatial_template"]
    vehicle = spatial["vehicle"]
    shape = spatial["generator"]
    dt = float(generator["sample_period_s"])
    count = int(round(float(generator["decision_horizon_s"]) / dt))
    x = float(snapshot["ego"]["x_m"])
    y = float(snapshot["ego"]["y_m"])
    yaw = float(snapshot["ego"]["yaw_rad"])
    speed = float(snapshot["ego"]["speed_mps"])
    steering = 0.0
    points = []
    for index in range(count + 1):
        curvature = math.tan(steering) / float(vehicle["wheelbase_m"])
        desired = _reference_steering(x, shape)
        maximum_delta = float(vehicle["maximum_steering_rate_radps"]) * dt
        next_steering = steering + max(
            -maximum_delta, min(maximum_delta, desired - steering)
        )
        next_steering = max(
            -float(vehicle["maximum_steering_angle_rad"]),
            min(float(vehicle["maximum_steering_angle_rad"]), next_steering),
        )
        target_speed = float(shape["target_speed_mps"])
        next_speed = min(
            target_speed,
            speed + float(vehicle["maximum_acceleration_mps2"]) * dt,
        )
        acceleration = (next_speed - speed) / dt if index < count else 0.0
        steering_rate = (next_steering - steering) / dt if index < count else 0.0
        points.append(
            _point(
                index * dt, x, y, yaw, speed, acceleration, steering,
                steering_rate, curvature,
            )
        )
        if index == count:
            break
        average_speed = 0.5 * (speed + next_speed)
        average_steering = 0.5 * (steering + next_steering)
        ds = average_speed * dt
        x += ds * math.cos(yaw)
        y += ds * math.sin(yaw)
        yaw += ds * math.tan(average_steering) / float(vehicle["wheelbase_m"])
        speed = next_speed
        steering = next_steering
    value = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "candidate_id": generator["spatial_candidate_id"],
        "generator_id": generator["generator_id"],
        "certified_stop": False,
        "cleared_straight": False,
        "target_speed_mps": float(shape["target_speed_mps"]),
        "points": points,
    }
    value["candidate_digest"] = digest(value)
    return value


def actor_cleared(snapshot, contract):
    verifier = contract["hard_verifier"]
    actor = snapshot.get("actor", {})
    return (
        finite(actor.get("coordinate_m"))
        and float(actor["coordinate_m"])
        >= float(verifier["cleared_actor_minimum_coordinate_m"])
        and actor.get(verifier["cleared_actor_required_flag"]) is True
    )


def generate_candidate_set(snapshot, contract):
    generator = contract["candidate_generator"]
    candidates = [
        _longitudinal_candidate(
            snapshot, contract, generator["certified_stop_candidate_id"], 0.0,
            certified_stop=True,
        ),
        _spatial_candidate(snapshot, contract),
    ]
    if actor_cleared(snapshot, contract):
        candidates.append(
            _longitudinal_candidate(
                snapshot, contract, generator["cleared_straight_candidate_id"],
                float(generator["maximum_speed_mps"]), cleared_straight=True,
            )
        )
    value = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "world_model_seq": int(snapshot["world_model_seq"]),
        "generator_id": generator["generator_id"],
        "candidates": candidates,
    }
    value["candidate_set_digest"] = digest(value)
    return value


def _features(snapshot):
    coordinate = float(snapshot["actor"]["coordinate_m"])
    speed = float(snapshot["actor"]["speed_mps"])
    return [1.0, coordinate, speed, coordinate * coordinate,
            speed * speed, coordinate * speed]


def predict_spatial_tube(snapshot, candidate, model, contract, mode):
    runtime = contract["predictor_runtime"]
    bounds = runtime["ood_bounds"]
    values = {
        "actor_coordinate_m": float(snapshot["actor"]["coordinate_m"]),
        "actor_speed_mps": float(snapshot["actor"]["speed_mps"]),
        "actor_path_angle_deg": float(snapshot["actor"]["path_angle_deg"]),
    }
    ood = [
        key for key, value in values.items()
        if value < float(bounds[key][0]) - 1e-12
        or value > float(bounds[key][1]) + 1e-12
    ]
    if ood:
        raise PlannerError("OOD:" + ",".join(sorted(ood)))
    require(
        mode in runtime["spatial_supported_modes"],
        "UNSUPPORTED_MODE_ACTION:" + mode,
    )
    x = _features(snapshot)
    head = model["supported_action_heads"][mode][candidate["candidate_id"]]
    radius = float(model["calibration"]["radius_m"])
    margin = float(runtime["additional_coordinate_margin_m"])
    anchors = [{
        "time_s": 0.0,
        "lower_coordinate_m": x[1],
        "upper_coordinate_m": x[1],
    }]
    for time_s, key in zip(
        runtime["relative_horizons_s"], ("t1_5", "t2_5", "t3_5", "t4_5")
    ):
        prediction = sum(
            float(a) * float(b)
            for a, b in zip(head[key]["coefficients"], x)
        )
        require(finite(prediction), "NONFINITE_PREDICTION")
        anchors.append({
            "time_s": float(time_s),
            "lower_coordinate_m": prediction - radius - margin,
            "upper_coordinate_m": prediction + radius + margin,
            "prediction_m": prediction,
        })
    samples = []
    dt = float(contract["candidate_generator"]["sample_period_s"])
    horizon = float(contract["candidate_generator"]["decision_horizon_s"])
    for index in range(int(round(horizon / dt)) + 1):
        time_s = index * dt
        if time_s <= 4.0 + 1e-12:
            right_index = min(4, int(math.ceil(time_s)))
            left_index = max(0, right_index - 1)
            ratio = 0.0 if right_index == left_index else time_s - left_index
            lower = float(anchors[left_index]["lower_coordinate_m"]) + ratio * (
                float(anchors[right_index]["lower_coordinate_m"])
                - float(anchors[left_index]["lower_coordinate_m"])
            )
            upper = float(anchors[left_index]["upper_coordinate_m"]) + ratio * (
                float(anchors[right_index]["upper_coordinate_m"])
                - float(anchors[left_index]["upper_coordinate_m"])
            )
        else:
            minimum_speed = float(
                runtime["post_horizon_certified_minimum_speed_mps"][mode]
            )
            lower = float(anchors[-1]["lower_coordinate_m"]) + minimum_speed * (
                time_s - 4.0
            )
            upper = float(runtime["certified_path_coordinate_upper_m"])
        samples.append({
            "time_s": round(time_s, 9),
            "lower_coordinate_m": min(lower, upper),
            "upper_coordinate_m": max(lower, upper),
        })
    value = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "candidate_id": candidate["candidate_id"],
        "predictor_id": model["model_id"],
        "inferred_mode": mode,
        "features": x,
        "anchors": anchors,
        "samples": samples,
    }
    value["occupancy_tube_digest"] = digest(value)
    return value


def _initial_support(snapshot, contract):
    support = contract["candidate_generator"]["spatial_initial_support"]
    values = {
        "x_m": float(snapshot["ego"]["x_m"]),
        "y_m": float(snapshot["ego"]["y_m"]),
        "yaw_rad": float(snapshot["ego"]["yaw_rad"]),
        "speed_mps": float(snapshot["ego"]["speed_mps"]),
    }
    return all(
        float(support[key][0]) <= value <= float(support[key][1])
        for key, value in values.items()
    )


def _kinematic_failures(candidate, contract):
    generator = contract["candidate_generator"]
    failures = []
    points = candidate.get("points", [])
    expected = int(round(float(generator["decision_horizon_s"])
                         / float(generator["sample_period_s"]))) + 1
    if len(points) != expected:
        return ["POINT_COUNT_INVALID"]
    fields = (
        "time_from_start_s", "x_m", "y_m", "yaw_rad",
        "longitudinal_velocity_mps", "longitudinal_acceleration_mps2",
        "steering_angle_rad", "steering_rate_radps", "curvature_inv_m",
    )
    if any(not finite(point.get(field)) for point in points for field in fields):
        return ["NONFINITE_KINEMATICS"]
    if any(
        float(point["longitudinal_velocity_mps"]) < -1e-9
        or float(point["longitudinal_velocity_mps"])
        > float(generator["maximum_speed_mps"]) + 1e-9
        or float(point["longitudinal_acceleration_mps2"])
        > float(generator["maximum_acceleration_mps2"]) + 1e-9
        or float(point["longitudinal_acceleration_mps2"])
        < -float(generator["maximum_deceleration_mps2"]) - 1e-9
        or abs(float(point["steering_angle_rad"]))
        > float(generator["maximum_steering_angle_rad"]) + 1e-9
        or abs(float(point["steering_rate_radps"]))
        > float(generator["maximum_steering_rate_radps"]) + 1e-9
        or abs(float(point["curvature_inv_m"]))
        > float(generator["maximum_curvature_inv_m"]) + 1e-9
        for point in points
    ):
        failures.append("KINEMATIC_BOUND")
    body = dict(candidate)
    claimed = body.pop("candidate_digest", "")
    if digest(body) != claimed:
        failures.append("CANDIDATE_DIGEST_MISMATCH")
    return failures


def _path_segment_clearance(candidate, snapshot, contract, lower, upper):
    angle = math.radians(float(snapshot["actor"]["path_angle_deg"]))
    dx, dy = math.cos(angle), math.sin(angle)
    radius = float(contract["hard_verifier"]["conservative_combined_radius_m"])
    return min(
        core._point_segment_distance(
            float(point["x_m"]), float(point["y_m"]),
            lower * dx, lower * dy, upper * dx, upper * dy,
        ) - radius
        for point in candidate["points"]
    )


def evaluate_candidate_set(snapshot, candidate_set, model, contract):
    body = dict(candidate_set)
    claimed_set_digest = body.pop("candidate_set_digest", "")
    set_valid = digest(body) == claimed_set_digest
    verifier = contract["hard_verifier"]
    health_ok = all(
        snapshot.get("health", {}).get(field) is True
        for field in verifier["required_health_fields"]
    )
    freshness_ok = finite(snapshot.get("snapshot_age_s")) and float(
        snapshot["snapshot_age_s"]
    ) <= float(verifier["maximum_snapshot_age_s"])
    latency_ok = (
        finite(snapshot.get("predictor_latency_ms"))
        and finite(snapshot.get("verifier_latency_ms"))
        and float(snapshot["predictor_latency_ms"])
        <= float(verifier["maximum_predictor_latency_ms"])
        and float(snapshot["verifier_latency_ms"])
        <= float(verifier["maximum_verifier_latency_ms"])
    )
    try:
        mode = infer_mode(float(snapshot["actor"]["speed_mps"]), contract)
        mode_error = ""
    except (PlannerError, KeyError, TypeError, ValueError) as exc:
        mode = ""
        mode_error = str(exc)
    evaluations = []
    for candidate in candidate_set["candidates"]:
        failures = list(_kinematic_failures(candidate, contract))
        if not set_valid:
            failures.append("CANDIDATE_SET_DIGEST_MISMATCH")
        if candidate["scene_snapshot_id"] != snapshot["scene_snapshot_id"]:
            failures.append("SCENE_SNAPSHOT_MISMATCH")
        tube = None
        if candidate["candidate_id"] == contract["candidate_generator"][
            "spatial_candidate_id"
        ]:
            if not _initial_support(snapshot, contract):
                failures.append("SPATIAL_INITIAL_STATE_UNSUPPORTED")
            if not health_ok:
                failures.append("HEALTH_INVALID")
            if not freshness_ok:
                failures.append("SNAPSHOT_STALE")
            if not latency_ok:
                failures.append("LATENCY_BUDGET_EXCEEDED")
            if mode_error:
                failures.append("MODE_INVALID:" + mode_error)
            else:
                try:
                    tube = predict_spatial_tube(
                        snapshot, candidate, model, contract, mode
                    )
                except (PlannerError, KeyError, TypeError, ValueError) as exc:
                    failures.append("PREDICTOR_INVALID:" + str(exc))
            if tube is not None:
                angle = math.radians(float(snapshot["actor"]["path_angle_deg"]))
                dx, dy = math.cos(angle), math.sin(angle)
                radius = float(verifier["conservative_combined_radius_m"])
                clearances = [
                    core._point_segment_distance(
                        float(point["x_m"]), float(point["y_m"]),
                        float(occupancy["lower_coordinate_m"]) * dx,
                        float(occupancy["lower_coordinate_m"]) * dy,
                        float(occupancy["upper_coordinate_m"]) * dx,
                        float(occupancy["upper_coordinate_m"]) * dy,
                    ) - radius
                    for point, occupancy in zip(candidate["points"], tube["samples"])
                ]
                clearance = min(clearances)
                if clearance < 0.0:
                    failures.append("OCCUPANCY_INTERSECTION")
            else:
                clearance = -float("inf")
        elif candidate.get("cleared_straight"):
            if not actor_cleared(snapshot, contract):
                failures.append("CLEARED_ACTOR_CERTIFICATE_MISSING")
            clearance = _path_segment_clearance(
                candidate, snapshot, contract,
                float(snapshot["actor"]["coordinate_m"]),
                float(contract["predictor_runtime"][
                    "certified_path_coordinate_upper_m"
                ]),
            )
            if clearance < 0.0:
                failures.append("CLEARED_STRAIGHT_INTERSECTION")
        else:
            clearance = _path_segment_clearance(
                candidate, snapshot, contract,
                float(snapshot["actor"]["coordinate_m"]),
                float(contract["predictor_runtime"][
                    "certified_path_coordinate_upper_m"
                ]),
            )
            if clearance < 0.0:
                failures.append("BACKUP_CLEARANCE_INVALID")
        failures = sorted(set(failures))
        progress = float(candidate["points"][-1]["x_m"]) - float(
            candidate["points"][0]["x_m"]
        )
        acceleration_cost = sum(
            abs(float(point["longitudinal_acceleration_mps2"]))
            for point in candidate["points"]
        ) / len(candidate["points"])
        ranker = contract["analytic_ranker"]
        score = (
            float(ranker["progress_weight"]) * progress
            + float(ranker["clearance_weight"]) * min(max(clearance, -5.0), 5.0)
            - float(ranker["acceleration_cost_weight"]) * acceleration_cost
        )
        evaluations.append({
            "candidate_id": candidate["candidate_id"],
            "accepted": not failures,
            "verdict": "ACCEPTED" if not failures else "REJECTED",
            "rejection_codes": failures,
            "minimum_clearance_m": clearance,
            "progress_m": progress,
            "acceleration_cost": acceleration_cost,
            "analytic_score": score,
            "occupancy_tube_digest": (
                tube["occupancy_tube_digest"] if tube is not None else ""
            ),
            "inferred_mode": mode,
        })
    accepted = [value for value in evaluations if value["accepted"]]
    stop_id = contract["candidate_generator"]["certified_stop_candidate_id"]
    if accepted:
        selected = sorted(
            accepted, key=lambda value: (-value["analytic_score"],
                                         value["candidate_id"])
        )[0]
    else:
        selected = next(value for value in evaluations
                        if value["candidate_id"] == stop_id)
    fallback = selected["candidate_id"] == stop_id
    decision = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "candidate_set_digest": claimed_set_digest,
        "selected_candidate_id": selected["candidate_id"],
        "policy_kind": "spatial_analytic_safe_candidate_ranker",
        "policy_artifact_digest": digest(contract["analytic_ranker"]),
        "safety_fallback_used": fallback,
        "fallback_reason": "CERTIFIED_STOP_FAIL_CLOSED" if fallback else "",
    }
    decision["decision_digest"] = digest(decision)
    return {
        "candidate_set": candidate_set,
        "evaluations": evaluations,
        "decision": decision,
        "integrity": {
            "candidate_set_digest_valid": set_valid,
            "health_ok": health_ok,
            "freshness_ok": freshness_ok,
            "latency_ok": latency_ok,
            "hidden_policy_runtime_input": False,
        },
    }


def plan(snapshot, model, contract):
    return evaluate_candidate_set(
        snapshot, generate_candidate_set(snapshot, contract), model, contract
    )
