"""Deterministic V3-SCOPE crossing candidate generator and safety planner."""

import hashlib
import json
import math
from typing import Any, Dict, Iterable, Mapping, Sequence


class PlannerError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise PlannerError(message)


def finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")


def digest(value: Any) -> str:
    return hashlib.sha256(canonical_bytes(value)).hexdigest()


def _profile(
    elapsed: float,
    x0: float,
    initial_speed: float,
    target_speed: float,
    maximum_acceleration: float,
    maximum_deceleration: float,
) -> Sequence[float]:
    if target_speed > initial_speed:
        acceleration = maximum_acceleration
    elif target_speed < initial_speed:
        acceleration = -maximum_deceleration
    else:
        acceleration = 0.0
    ramp = (
        abs(target_speed - initial_speed) / abs(acceleration)
        if acceleration
        else 0.0
    )
    active = min(elapsed, ramp)
    x = x0 + initial_speed * active + 0.5 * acceleration * active * active
    speed = initial_speed + acceleration * active
    if elapsed > ramp:
        x += target_speed * (elapsed - ramp)
        speed = target_speed
    return x, speed, acceleration if elapsed < ramp - 1e-12 else 0.0


def _candidate(
    snapshot: Mapping[str, Any],
    target_speed: float,
    candidate_id: str,
    certified_stop: bool,
    teb_fallback: bool,
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    generator = contract["candidate_generator"]
    horizon = float(generator["decision_horizon_s"])
    step = float(generator["sample_period_s"])
    count = int(round(horizon / step))
    ego = snapshot["ego"]
    states = [
        (
            0.0,
            float(ego["x_m"]),
            float(ego["speed_mps"]),
        )
    ]
    accelerations = []
    x = float(ego["x_m"])
    speed = float(ego["speed_mps"])
    for index in range(1, count + 1):
        if target_speed > speed:
            new_speed = min(
                target_speed,
                speed
                + float(generator["maximum_acceleration_mps2"]) * step,
            )
        elif target_speed < speed:
            new_speed = max(
                target_speed,
                speed
                - float(generator["maximum_deceleration_mps2"]) * step,
            )
        else:
            new_speed = speed
        acceleration = (new_speed - speed) / step
        x += 0.5 * (speed + new_speed) * step
        speed = new_speed
        accelerations.append(acceleration)
        states.append((index * step, x, speed))
    points = []
    for index, (elapsed, x, speed) in enumerate(states):
        acceleration = (
            accelerations[index] if index < len(accelerations) else 0.0
        )
        point = {
            "time_from_start_s": round(elapsed, 9),
            "x_m": round(x, 12),
            "y_m": round(float(ego["y_m"]), 12),
            "yaw_rad": round(float(ego["yaw_rad"]), 12),
            "longitudinal_velocity_mps": round(speed, 12),
            "longitudinal_acceleration_mps2": round(acceleration, 12),
            "steering_angle_rad": 0.0,
            "steering_rate_radps": 0.0,
            "curvature_inv_m": 0.0,
        }
        points.append(point)
    value = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "candidate_id": candidate_id,
        "generator_id": generator["generator_id"],
        "certified_stop": certified_stop,
        "teb_fallback": teb_fallback,
        "target_speed_mps": target_speed,
        "points": points,
    }
    value["candidate_digest"] = digest(value)
    return value


def generate_candidate_set(
    snapshot: Mapping[str, Any], contract: Mapping[str, Any]
) -> Dict[str, Any]:
    generator = contract["candidate_generator"]
    candidates = []
    for target in generator["target_speeds_mps"]:
        speed = float(target)
        candidate_id = (
            generator["certified_stop_candidate_id"]
            if speed == 0.0
            else "feasible_v{:03d}".format(int(round(speed * 100.0)))
        )
        candidates.append(
            _candidate(
                snapshot,
                speed,
                candidate_id,
                speed == 0.0,
                False,
                contract,
            )
        )
    candidates.append(
        _candidate(
            snapshot,
            0.8,
            generator["teb_fallback_candidate_id"],
            False,
            True,
            contract,
        )
    )
    require(
        len({item["candidate_id"] for item in candidates}) == len(candidates),
        "candidate ids are not unique",
    )
    body = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "world_model_seq": int(snapshot["world_model_seq"]),
        "generator_id": generator["generator_id"],
        "vehicle_model_digest": digest(
            {
                "maximum_speed_mps": generator["maximum_speed_mps"],
                "maximum_acceleration_mps2": generator[
                    "maximum_acceleration_mps2"
                ],
                "maximum_deceleration_mps2": generator[
                    "maximum_deceleration_mps2"
                ],
                "maximum_steering_angle_rad": generator[
                    "maximum_steering_angle_rad"
                ],
                "maximum_steering_rate_radps": generator[
                    "maximum_steering_rate_radps"
                ],
                "maximum_curvature_inv_m": generator[
                    "maximum_curvature_inv_m"
                ],
            }
        ),
        "candidates": candidates,
    }
    body["candidate_set_digest"] = digest(body)
    return body


def validate_candidate(
    candidate: Mapping[str, Any],
    snapshot: Mapping[str, Any],
    contract: Mapping[str, Any],
) -> Sequence[str]:
    failures = []
    generator = contract["candidate_generator"]
    if candidate.get("scene_snapshot_id") != snapshot.get("scene_snapshot_id"):
        failures.append("SCENE_SNAPSHOT_MISMATCH")
    body = dict(candidate)
    claimed_digest = body.pop("candidate_digest", "")
    if digest(body) != claimed_digest:
        failures.append("CANDIDATE_DIGEST_MISMATCH")
    points = candidate.get("points", [])
    expected_count = int(
        round(
            float(generator["decision_horizon_s"])
            / float(generator["sample_period_s"])
        )
    ) + 1
    if len(points) != expected_count:
        failures.append("POINT_COUNT_INVALID")
        return failures
    numeric_fields = (
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
        for field in numeric_fields
    ):
        failures.append("NONFINITE_KINEMATICS")
        return failures
    tolerance = 1e-8
    for left, right in zip(points, points[1:]):
        dt = float(right["time_from_start_s"]) - float(
            left["time_from_start_s"]
        )
        if dt <= 0.0:
            failures.append("NONMONOTONIC_TIME")
            break
        average_speed = 0.5 * (
            float(left["longitudinal_velocity_mps"])
            + float(right["longitudinal_velocity_mps"])
        )
        expected_dx = (
            average_speed * math.cos(float(left["yaw_rad"])) * dt
        )
        expected_dy = (
            average_speed * math.sin(float(left["yaw_rad"])) * dt
        )
        if (
            abs(float(right["x_m"]) - float(left["x_m"]) - expected_dx)
            > tolerance
            or abs(float(right["y_m"]) - float(left["y_m"]) - expected_dy)
            > tolerance
        ):
            failures.append("BICYCLE_INTEGRATION_MISMATCH")
            break
        acceleration = (
            float(right["longitudinal_velocity_mps"])
            - float(left["longitudinal_velocity_mps"])
        ) / dt
        if acceleration > float(generator["maximum_acceleration_mps2"]) + tolerance:
            failures.append("ACCELERATION_LIMIT")
            break
        if acceleration < -float(generator["maximum_deceleration_mps2"]) - tolerance:
            failures.append("DECELERATION_LIMIT")
            break
    if any(
        float(point["longitudinal_velocity_mps"])
        < -tolerance
        or float(point["longitudinal_velocity_mps"])
        > float(generator["maximum_speed_mps"]) + tolerance
        or abs(float(point["steering_angle_rad"]))
        > float(generator["maximum_steering_angle_rad"]) + tolerance
        or abs(float(point["steering_rate_radps"]))
        > float(generator["maximum_steering_rate_radps"]) + tolerance
        or abs(float(point["curvature_inv_m"]))
        > float(generator["maximum_curvature_inv_m"]) + tolerance
        for point in points
    ):
        failures.append("KINEMATIC_BOUND")
    if candidate.get("certified_stop"):
        terminal_stop_time = (
            float(points[0]["longitudinal_velocity_mps"])
            / float(generator["maximum_deceleration_mps2"])
        )
        stop_points = [
            point
            for point in points
            if float(point["time_from_start_s"])
            >= terminal_stop_time - tolerance
        ]
        if (
            not stop_points
            or max(
                abs(float(point["longitudinal_velocity_mps"]))
                for point in stop_points
            )
            > tolerance
            or max(float(point["x_m"]) for point in stop_points)
            - min(float(point["x_m"]) for point in stop_points)
            > tolerance
        ):
            failures.append("CERTIFIED_STOP_TERMINAL_INVARIANT")
    return sorted(set(failures))


def _features(snapshot: Mapping[str, Any], candidate: Mapping[str, Any]):
    coordinate = float(snapshot["actor"]["coordinate_m"])
    actor_speed = float(snapshot["actor"]["speed_mps"])
    target = float(candidate["target_speed_mps"])
    return [
        1.0,
        coordinate,
        actor_speed,
        target,
        coordinate * coordinate,
        actor_speed * actor_speed,
        target * target,
        coordinate * actor_speed,
        coordinate * target,
        actor_speed * target,
    ]


def _predict(
    model: Mapping[str, Any],
    policy: str,
    horizon: str,
    features: Sequence[float],
) -> float:
    coefficients = model["mode_heads"][policy][horizon]["coefficients"]
    value = sum(float(a) * float(b) for a, b in zip(coefficients, features))
    physical = model.get("physical_coordinate_clip")
    if physical:
        value = max(float(features[1]), value)
        value = min(float(physical["upper_bound_m"]), value)
    return value


def infer_observed_mode(
    actor_speed_mps: float, mode_inference: Mapping[str, Any]
) -> str:
    lower = float(mode_inference["cautious_hesitant_threshold_mps"])
    upper = float(mode_inference["hesitant_assertive_threshold_mps"])
    ambiguity = float(mode_inference["ambiguity_half_width_mps"])
    require(finite(actor_speed_mps), "NONFINITE_MODE_FEATURE")
    if actor_speed_mps < lower - ambiguity:
        return "cautious"
    if lower + ambiguity < actor_speed_mps < upper - ambiguity:
        return "hesitant"
    if actor_speed_mps > upper + ambiguity:
        return "assertive"
    raise PlannerError("MODE_AMBIGUOUS")


def actor_cleared(
    snapshot: Mapping[str, Any], contract: Mapping[str, Any]
) -> bool:
    cleared = contract["predictor_runtime"].get("cleared_actor_bypass")
    if not cleared:
        return False
    coordinate = snapshot.get("actor", {}).get("coordinate_m")
    speed = snapshot.get("actor", {}).get("speed_mps")
    return (
        finite(coordinate)
        and finite(speed)
        and float(coordinate) >= float(cleared["minimum_coordinate_m"])
        and float(speed) >= float(cleared["minimum_speed_mps"])
        and snapshot["actor"].get(
            cleared["required_snapshot_actor_flag"]
        )
        is True
    )


def actor_post_conflict_reachable(
    snapshot: Mapping[str, Any], contract: Mapping[str, Any]
) -> bool:
    reachable = contract["predictor_runtime"].get(
        "post_conflict_reachable_bypass"
    )
    if not reachable:
        return False
    actor = snapshot.get("actor", {})
    coordinate = actor.get("coordinate_m")
    speed = actor.get("speed_mps")
    angle = actor.get("path_angle_deg")
    support = reachable["path_angle_support_deg"]
    return (
        finite(coordinate)
        and finite(speed)
        and finite(angle)
        and float(coordinate)
        >= float(reachable["minimum_coordinate_m"])
        and float(coordinate)
        < float(
            contract["predictor_runtime"]["cleared_actor_bypass"][
                "minimum_coordinate_m"
            ]
        )
        and float(speed)
        >= float(reachable["certified_minimum_speed_mps"])
        and float(support[0]) <= float(angle) <= float(support[1])
        and actor.get(reachable["required_snapshot_actor_flag"]) is True
    )


def _post_conflict_reachable_lower_coordinate(
    coordinate: float,
    speed: float,
    elapsed: float,
    reachable: Mapping[str, Any],
) -> float:
    minimum_speed = float(reachable["certified_minimum_speed_mps"])
    maximum_deceleration = float(
        reachable["certified_maximum_deceleration_mps2"]
    )
    initial_speed = max(float(speed), minimum_speed)
    deceleration_time = max(
        0.0, (initial_speed - minimum_speed) / maximum_deceleration
    )
    active = min(float(elapsed), deceleration_time)
    progress = (
        initial_speed * active
        - 0.5 * maximum_deceleration * active * active
        + minimum_speed * max(0.0, float(elapsed) - deceleration_time)
    )
    return float(coordinate) + progress


def predict_occupancy_tube(
    snapshot: Mapping[str, Any],
    candidate: Mapping[str, Any],
    model: Mapping[str, Any],
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    runtime = contract["predictor_runtime"]
    cleared = runtime.get("cleared_actor_bypass")
    coordinate = float(snapshot["actor"]["coordinate_m"])
    actor_speed = float(snapshot["actor"]["speed_mps"])
    if actor_cleared(snapshot, contract):
        step = float(contract["candidate_generator"]["sample_period_s"])
        horizon = float(contract["candidate_generator"]["decision_horizon_s"])
        upper = max(
            coordinate,
            float(cleared["certified_path_coordinate_upper_m"]),
        )
        samples = [
            {
                "time_s": round(index * step, 9),
                "lower_coordinate_m": coordinate,
                "upper_coordinate_m": upper,
            }
            for index in range(int(round(horizon / step)) + 1)
        ]
        value = {
            "schema_version": "1.0",
            "scene_snapshot_id": snapshot["scene_snapshot_id"],
            "candidate_id": candidate["candidate_id"],
            "predictor_id": model["model_id"],
            "mode_inference_id": "",
            "inferred_mode": "",
            "active_mode_heads": [],
            "features": _features(snapshot, candidate),
            "anchors": [],
            "samples": samples,
            "safety_certified_cleared_bypass": True,
            "cleared_bypass_id": cleared["cleared_bypass_id"],
        }
        value["occupancy_tube_digest"] = digest(value)
        return value
    reachable = runtime.get("post_conflict_reachable_bypass")
    if actor_post_conflict_reachable(snapshot, contract):
        step = float(contract["candidate_generator"]["sample_period_s"])
        horizon = float(contract["candidate_generator"]["decision_horizon_s"])
        upper = float(reachable["certified_path_coordinate_upper_m"])
        samples = []
        for index in range(int(round(horizon / step)) + 1):
            time_s = round(index * step, 9)
            lower = _post_conflict_reachable_lower_coordinate(
                coordinate, actor_speed, time_s, reachable
            )
            samples.append(
                {
                    "time_s": time_s,
                    "lower_coordinate_m": min(lower, upper),
                    "upper_coordinate_m": upper,
                }
            )
        value = {
            "schema_version": "1.0",
            "scene_snapshot_id": snapshot["scene_snapshot_id"],
            "candidate_id": candidate["candidate_id"],
            "predictor_id": model["model_id"],
            "mode_inference_id": "",
            "inferred_mode": "",
            "active_mode_heads": [],
            "features": _features(snapshot, candidate),
            "anchors": [],
            "samples": samples,
            "safety_certified_cleared_bypass": False,
            "cleared_bypass_id": "",
            "safety_certified_post_conflict_reachable": True,
            "post_conflict_reachable_bypass_id": reachable[
                "post_conflict_reachable_bypass_id"
            ],
        }
        value["occupancy_tube_digest"] = digest(value)
        return value
    bounds = runtime["ood_bounds"]
    values = {
        "actor_coordinate_m": coordinate,
        "actor_speed_mps": actor_speed,
        "candidate_target_speed_mps": float(candidate["target_speed_mps"]),
        "actor_path_angle_deg": float(snapshot["actor"]["path_angle_deg"]),
    }
    ood = [
        name
        for name, value in values.items()
        if value < float(bounds[name][0]) - 1e-12
        or value > float(bounds[name][1]) + 1e-12
    ]
    if ood:
        raise PlannerError("OOD:" + ",".join(sorted(ood)))
    features = _features(snapshot, candidate)
    radius = float(model["calibration"]["radius_m"])
    horizons = ("t1_5", "t2_5", "t3_5", "t4_5")
    mode_inference = runtime.get("mode_inference")
    inferred_mode = None
    active_modes = list(runtime["mode_heads"])
    if mode_inference:
        inferred_mode = infer_observed_mode(
            float(snapshot["actor"]["speed_mps"]), mode_inference
        )
        active_modes = [inferred_mode]
    anchors = [
        {
            "time_s": 0.0,
            "lower_coordinate_m": features[1],
            "upper_coordinate_m": features[1],
        }
    ]
    for relative_time, horizon in zip(
        runtime["relative_horizons_s"], horizons
    ):
        modes = {
            policy: _predict(model, policy, horizon, features)
            for policy in active_modes
        }
        require(all(finite(value) for value in modes.values()), "NONFINITE_PREDICTION")
        anchors.append(
            {
                "time_s": float(relative_time),
                "lower_coordinate_m": min(modes.values()) - radius,
                "upper_coordinate_m": max(modes.values()) + radius,
                "mode_predictions_m": modes,
            }
        )
    margin = float(runtime["additional_coordinate_margin_m"])
    step = float(contract["candidate_generator"]["sample_period_s"])
    horizon = float(contract["candidate_generator"]["decision_horizon_s"])
    samples = []
    for index in range(int(round(horizon / step)) + 1):
        time_s = min(index * step, horizon)
        right_index = min(int(math.ceil(time_s)), 4)
        left_index = max(0, right_index - 1)
        if right_index == left_index:
            ratio = 0.0
        else:
            ratio = (time_s - float(left_index)) / (
                float(right_index - left_index)
            )
        left = anchors[left_index]
        right = anchors[right_index]
        lower = float(left["lower_coordinate_m"]) + ratio * (
            float(right["lower_coordinate_m"])
            - float(left["lower_coordinate_m"])
        )
        upper = float(left["upper_coordinate_m"]) + ratio * (
            float(right["upper_coordinate_m"])
            - float(left["upper_coordinate_m"])
        )
        if time_s > 0.0:
            lower -= margin
            upper += margin
        samples.append(
            {
                "time_s": round(time_s, 9),
                "lower_coordinate_m": lower,
                "upper_coordinate_m": upper,
            }
        )
    value = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "candidate_id": candidate["candidate_id"],
        "predictor_id": model["model_id"],
        "mode_inference_id": (
            mode_inference["mode_inference_id"] if mode_inference else ""
        ),
        "inferred_mode": inferred_mode or "",
        "active_mode_heads": active_modes,
        "features": features,
        "anchors": anchors,
        "samples": samples,
        "safety_certified_cleared_bypass": False,
        "cleared_bypass_id": "",
    }
    value["occupancy_tube_digest"] = digest(value)
    return value


def _point_segment_distance(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    vx, vy = bx - ax, by - ay
    denominator = vx * vx + vy * vy
    if denominator <= 1e-18:
        return math.hypot(px - ax, py - ay)
    ratio = ((px - ax) * vx + (py - ay) * vy) / denominator
    ratio = min(max(ratio, 0.0), 1.0)
    return math.hypot(px - (ax + ratio * vx), py - (ay + ratio * vy))


def _combined_radius(contract: Mapping[str, Any]) -> float:
    verifier = contract["hard_verifier"]
    ego_radius = math.hypot(
        0.5 * float(verifier["ego_footprint_length_m"]),
        0.5 * float(verifier["ego_footprint_width_m"]),
    )
    actor_radius = math.hypot(
        0.5 * float(verifier["actor_footprint_length_m"]),
        0.5 * float(verifier["actor_footprint_width_m"]),
    )
    return (
        ego_radius
        + actor_radius
        + float(verifier["tracking_error_margin_m"])
        + float(verifier["occupancy_margin_m"])
    )


def _backup_invariant(
    snapshot: Mapping[str, Any],
    candidate: Mapping[str, Any],
    contract: Mapping[str, Any],
) -> Sequence[float]:
    angle = math.radians(float(snapshot["actor"]["path_angle_deg"]))
    dx, dy = math.cos(angle), math.sin(angle)
    radius = _combined_radius(contract)
    clearance = min(
        abs(float(point["x_m"]) * dy - float(point["y_m"]) * dx) - radius
        for point in candidate["points"]
    )
    verifier = contract["hard_verifier"]
    free_distance = abs(float(snapshot["ego"]["x_m"])) - radius
    speed = float(snapshot["ego"]["speed_mps"])
    braking_required = (
        speed * float(verifier["braking_response_latency_s"])
        + speed * speed
        / (2.0 * float(verifier["certified_braking_deceleration_mps2"]))
    )
    return clearance, free_distance - braking_required


def _commitment_braking_margin(
    candidate: Mapping[str, Any], contract: Mapping[str, Any]
) -> float:
    verifier = contract["hard_verifier"]
    horizon = float(verifier["control_commitment_horizon_s"])
    radius = _combined_radius(contract)
    margins = []
    for point in candidate["points"]:
        if float(point["time_from_start_s"]) > horizon + 1e-12:
            continue
        speed = float(point["longitudinal_velocity_mps"])
        free_distance = abs(float(point["x_m"])) - radius
        braking_required = (
            speed * float(verifier["braking_response_latency_s"])
            + speed
            * speed
            / (
                2.0
                * float(verifier["certified_braking_deceleration_mps2"])
            )
        )
        margins.append(free_distance - braking_required)
    require(margins, "COMMITMENT_HORIZON_EMPTY")
    return min(margins)


def evaluate_candidate_set(
    snapshot: Mapping[str, Any],
    candidate_set: Mapping[str, Any],
    model: Mapping[str, Any],
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    set_body = dict(candidate_set)
    claimed_set_digest = set_body.pop("candidate_set_digest", "")
    set_valid = digest(set_body) == claimed_set_digest
    candidates = candidate_set.get("candidates", [])
    stop_candidates = [
        item for item in candidates if item.get("certified_stop") is True
    ]
    stop_roster_valid = len(stop_candidates) == 1
    snapshot_valid = (
        candidate_set.get("scene_snapshot_id")
        == snapshot.get("scene_snapshot_id")
    )
    verifier = contract["hard_verifier"]
    health_ok = all(
        snapshot.get("health", {}).get(field) is True
        for field in verifier["required_health_fields"]
    )
    freshness_ok = (
        finite(snapshot.get("snapshot_age_s"))
        and float(snapshot["snapshot_age_s"])
        <= float(verifier["maximum_snapshot_age_s"])
    )
    latency_ok = (
        finite(snapshot.get("predictor_latency_ms"))
        and finite(snapshot.get("verifier_latency_ms"))
        and float(snapshot["predictor_latency_ms"])
        <= float(verifier["maximum_predictor_latency_ms"])
        and float(snapshot["verifier_latency_ms"])
        <= float(verifier["maximum_verifier_latency_ms"])
    )
    cleared_certified = actor_cleared(snapshot, contract)
    evaluations = []
    for candidate in candidates:
        rejection_codes = list(
            validate_candidate(candidate, snapshot, contract)
        )
        if not set_valid:
            rejection_codes.append("CANDIDATE_SET_DIGEST_MISMATCH")
        if not snapshot_valid:
            rejection_codes.append("SCENE_SNAPSHOT_MISMATCH")
        if not stop_roster_valid:
            rejection_codes.append("CERTIFIED_STOP_ROSTER_INVALID")
        clearance, braking_margin = _backup_invariant(
            snapshot, candidate, contract
        )
        commitment_braking_margin = braking_margin
        if "control_commitment_horizon_s" in verifier:
            commitment_braking_margin = _commitment_braking_margin(
                candidate, contract
            )
        if (
            not cleared_certified
            and braking_margin < float(verifier["minimum_braking_margin_m"])
        ):
            rejection_codes.append("BRAKING_DOMAIN_INVALID")
        if (
            not cleared_certified
            and "control_commitment_horizon_s" in verifier
            and commitment_braking_margin
            < float(verifier["minimum_braking_margin_m"])
        ):
            rejection_codes.append("COMMITMENT_BRAKING_DOMAIN_INVALID")
        tube = None
        if not candidate.get("certified_stop"):
            if not health_ok:
                rejection_codes.append("HEALTH_INVALID")
            if not freshness_ok:
                rejection_codes.append("SNAPSHOT_STALE")
            if not latency_ok:
                rejection_codes.append("LATENCY_BUDGET_EXCEEDED")
            try:
                tube = predict_occupancy_tube(
                    snapshot, candidate, model, contract
                )
            except (PlannerError, KeyError, TypeError, ValueError) as exc:
                rejection_codes.append("PREDICTOR_INVALID:" + str(exc))
            if tube is not None:
                maximum_width = max(
                    float(item["upper_coordinate_m"])
                    - float(item["lower_coordinate_m"])
                    for item in tube["samples"]
                )
                if (
                    not tube.get("safety_certified_cleared_bypass", False)
                    and not tube.get(
                        "safety_certified_post_conflict_reachable", False
                    )
                    and maximum_width
                    > float(
                        contract["predictor_runtime"][
                            "maximum_full_tube_width_m"
                        ]
                    )
                ):
                    rejection_codes.append("UNCERTAINTY_TUBE_TOO_WIDE")
                angle = math.radians(
                    float(snapshot["actor"]["path_angle_deg"])
                )
                dx, dy = math.cos(angle), math.sin(angle)
                radius = _combined_radius(contract)
                clearances = []
                for point, occupancy in zip(
                    candidate["points"], tube["samples"]
                ):
                    lower = float(occupancy["lower_coordinate_m"])
                    upper = float(occupancy["upper_coordinate_m"])
                    clearances.append(
                        _point_segment_distance(
                            float(point["x_m"]),
                            float(point["y_m"]),
                            lower * dx,
                            lower * dy,
                            upper * dx,
                            upper * dy,
                        )
                        - radius
                    )
                clearance = min(clearances)
                if clearance < 0.0:
                    rejection_codes.append("OCCUPANCY_INTERSECTION")
        else:
            if not cleared_certified and clearance < 0.0:
                rejection_codes.append("BACKUP_CLEARANCE_INVALID")
        rejection_codes = sorted(set(rejection_codes))
        accepted = not rejection_codes
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
            + float(ranker["clearance_weight"])
            * min(max(clearance, -5.0), 5.0)
            - float(ranker["acceleration_cost_weight"]) * acceleration_cost
            - (
                float(ranker["teb_fallback_penalty"])
                if candidate.get("teb_fallback")
                else 0.0
            )
        )
        evaluations.append(
            {
                "candidate_id": candidate["candidate_id"],
                "accepted": accepted,
                "verdict": "ACCEPTED" if accepted else "REJECTED",
                "rejection_codes": rejection_codes,
                "minimum_clearance_m": clearance,
                "braking_margin_m": braking_margin,
                "commitment_braking_margin_m": (
                    commitment_braking_margin
                ),
                "progress_m": progress,
                "acceleration_cost": acceleration_cost,
                "analytic_score": score,
                "occupancy_tube_digest": (
                    tube["occupancy_tube_digest"] if tube else ""
                ),
            }
        )
    accepted = [item for item in evaluations if item["accepted"]]
    if accepted:
        ranked = sorted(
            accepted,
            key=lambda item: (-item["analytic_score"], item["candidate_id"]),
        )
        selected = ranked[0]
        fallback_reason = (
            "ONLY_CERTIFIED_STOP_ACCEPTED"
            if selected["candidate_id"]
            == contract["candidate_generator"][
                "certified_stop_candidate_id"
            ]
            else ""
        )
    elif stop_candidates:
        selected = next(
            item
            for item in evaluations
            if item["candidate_id"]
            == stop_candidates[0]["candidate_id"]
        )
        fallback_reason = "EMERGENCY_STOP_UNVERIFIED"
    else:
        raise PlannerError("CERTIFIED_STOP_MISSING")
    decision = {
        "schema_version": "1.0",
        "scene_snapshot_id": snapshot["scene_snapshot_id"],
        "candidate_set_digest": claimed_set_digest,
        "selected_candidate_id": selected["candidate_id"],
        "policy_kind": "analytic_safe_candidate_ranker",
        "policy_artifact_digest": digest(contract["analytic_ranker"]),
        "safety_fallback_used": bool(fallback_reason),
        "fallback_reason": fallback_reason,
    }
    decision["decision_digest"] = digest(decision)
    return {
        "candidate_set": candidate_set,
        "evaluations": evaluations,
        "decision": decision,
        "integrity": {
            "candidate_set_digest_valid": set_valid,
            "snapshot_id_valid": snapshot_valid,
            "certified_stop_roster_valid": stop_roster_valid,
            "health_ok": health_ok,
            "freshness_ok": freshness_ok,
            "latency_ok": latency_ok,
        },
    }


def plan(
    snapshot: Mapping[str, Any],
    model: Mapping[str, Any],
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    return evaluate_candidate_set(
        snapshot, generate_candidate_set(snapshot, contract), model, contract
    )
