"""Deterministic V3-07 mirrored spatial Ackermann candidates."""

import hashlib
import json
import math
from typing import Any, Dict, Mapping


class SpatialCandidateError(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise SpatialCandidateError(message)


def digest(value):
    return hashlib.sha256(
        json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode()
    ).hexdigest()


def reference_steering(x_m: float, side: float, generator: Mapping[str, Any]):
    outbound = [float(x) for x in generator["outbound_x_range_m"]]
    returning = [float(x) for x in generator["return_x_range_m"]]
    peak = float(generator["steering_peak_rad"])
    if outbound[0] <= x_m < outbound[1]:
        phase = (x_m - outbound[0]) / (outbound[1] - outbound[0])
        return side * peak * math.sin(2.0 * math.pi * phase)
    if returning[0] <= x_m < returning[1]:
        phase = (x_m - returning[0]) / (returning[1] - returning[0])
        return -side * peak * math.sin(2.0 * math.pi * phase)
    return 0.0


def generate(
    candidate_id: str,
    side: float,
    contract: Mapping[str, Any],
) -> Dict[str, Any]:
    require(side in (-1.0, 1.0), "side differs")
    vehicle = contract["vehicle"]
    generator = contract["generator"]
    dt = float(generator["integration_period_s"])
    duration = float(generator["rollout_duration_s"])
    count = int(round(duration / dt))
    x, y, yaw = [float(v) for v in generator["initial_pose_m_rad"]]
    speed = float(generator["initial_speed_mps"])
    steering = float(generator["initial_steering_angle_rad"])
    target_speed = float(generator["target_speed_mps"])
    wheelbase = float(vehicle["wheelbase_m"])
    points = []
    for index in range(count + 1):
        curvature = math.tan(steering) / wheelbase
        points.append(
            {
                "time_from_start_s": round(index * dt, 9),
                "x_m": round(x, 12),
                "y_m": round(y, 12),
                "yaw_rad": round(yaw, 12),
                "longitudinal_velocity_mps": round(speed, 12),
                "longitudinal_acceleration_mps2": 0.0,
                "steering_angle_rad": round(steering, 12),
                "steering_rate_radps": 0.0,
                "curvature_inv_m": round(curvature, 12),
            }
        )
        if index == count:
            break
        if target_speed > speed:
            next_speed = min(
                target_speed,
                speed + float(vehicle["maximum_acceleration_mps2"]) * dt,
            )
        else:
            next_speed = max(
                target_speed,
                speed - float(vehicle["maximum_deceleration_mps2"]) * dt,
            )
        desired = reference_steering(x, side, generator)
        maximum_delta = float(vehicle["maximum_steering_rate_radps"]) * dt
        next_steering = steering + max(
            -maximum_delta, min(maximum_delta, desired - steering)
        )
        next_steering = max(
            -float(vehicle["maximum_steering_angle_rad"]),
            min(float(vehicle["maximum_steering_angle_rad"]), next_steering),
        )
        average_speed = 0.5 * (speed + next_speed)
        average_steering = 0.5 * (steering + next_steering)
        curvature = math.tan(average_steering) / wheelbase
        ds = average_speed * dt
        x += ds * math.cos(yaw)
        y += ds * math.sin(yaw)
        yaw += ds * curvature
        points[-1]["longitudinal_acceleration_mps2"] = round(
            (next_speed - speed) / dt, 12
        )
        points[-1]["steering_rate_radps"] = round(
            (next_steering - steering) / dt, 12
        )
        speed = next_speed
        steering = next_steering
    value = {
        "schema_version": "1.0",
        "candidate_id": candidate_id,
        "generator_id": generator["generator_id"],
        "side": "left" if side > 0.0 else "right",
        "target_speed_mps": target_speed,
        "points": points,
    }
    value["candidate_digest"] = digest(value)
    return value


def audit(candidate: Mapping[str, Any], contract: Mapping[str, Any]):
    vehicle = contract["vehicle"]
    corridor = contract["corridor"]
    points = candidate["points"]
    integration_errors = []
    speed_violations = acceleration_violations = 0
    steering_violations = steering_rate_violations = curvature_violations = 0
    corridor_violations = 0
    center_limit = float(corridor["half_width_m"]) - 0.5 * float(
        vehicle["ego_footprint_width_m"]
    )
    for point in points:
        speed_violations += (
            float(point["longitudinal_velocity_mps"]) < -1e-12
            or float(point["longitudinal_velocity_mps"])
            > float(vehicle["maximum_speed_mps"]) + 1e-12
        )
        acceleration = float(point["longitudinal_acceleration_mps2"])
        acceleration_violations += (
            acceleration > float(vehicle["maximum_acceleration_mps2"]) + 1e-12
            or acceleration < -float(vehicle["maximum_deceleration_mps2"]) - 1e-12
        )
        steering_violations += (
            abs(float(point["steering_angle_rad"]))
            > float(vehicle["maximum_steering_angle_rad"]) + 1e-12
        )
        steering_rate_violations += (
            abs(float(point["steering_rate_radps"]))
            > float(vehicle["maximum_steering_rate_radps"]) + 1e-12
        )
        curvature_violations += (
            abs(float(point["curvature_inv_m"]))
            > float(vehicle["maximum_curvature_inv_m"]) + 1e-12
        )
        corridor_violations += abs(float(point["y_m"])) > center_limit + 1e-12
    for left, right in zip(points, points[1:]):
        dt = float(right["time_from_start_s"]) - float(left["time_from_start_s"])
        average_speed = 0.5 * (
            float(left["longitudinal_velocity_mps"])
            + float(right["longitudinal_velocity_mps"])
        )
        expected_x = float(left["x_m"]) + average_speed * math.cos(
            float(left["yaw_rad"])
        ) * dt
        expected_y = float(left["y_m"]) + average_speed * math.sin(
            float(left["yaw_rad"])
        ) * dt
        integration_errors.append(
            math.hypot(
                float(right["x_m"]) - expected_x,
                float(right["y_m"]) - expected_y,
            )
        )
    eligible_terminal = [
        point
        for point in points
        if float(point["x_m"]) >= float(corridor["terminal_goal_x_m"])
    ]
    terminal = eligible_terminal[0] if eligible_terminal else points[-1]
    return {
        "point_count": len(points),
        "maximum_bicycle_integration_error_m": max(integration_errors),
        "speed_limit_violation_count": speed_violations,
        "acceleration_limit_violation_count": acceleration_violations,
        "steering_limit_violation_count": steering_violations,
        "steering_rate_limit_violation_count": steering_rate_violations,
        "curvature_limit_violation_count": curvature_violations,
        "corridor_violation_count": corridor_violations,
        "maximum_absolute_center_y_m": max(
            abs(float(point["y_m"])) for point in points
        ),
        "maximum_absolute_yaw_rad": max(
            abs(float(point["yaw_rad"])) for point in points
        ),
        "terminal_goal_x_reached": bool(eligible_terminal),
        "terminal_center_error_m": abs(float(terminal["y_m"])),
        "terminal_yaw_error_rad": abs(float(terminal["yaw_rad"])),
        "final_x_m": float(points[-1]["x_m"]),
        "final_y_m": float(points[-1]["y_m"]),
        "final_yaw_rad": float(points[-1]["yaw_rad"]),
    }
