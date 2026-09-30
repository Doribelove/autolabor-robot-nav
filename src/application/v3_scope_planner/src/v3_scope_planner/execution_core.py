"""Pure helpers for fair M2 execution of frozen V3 candidate trajectories.

This module deliberately does not change the frozen V3 planning semantics.
It only converts an already-selected, Ackermann-feasible path into Twist
commands for the common simulation vehicle used by both V3 and TEB.
"""

import math


def angle_delta(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


def quaternion_yaw(orientation):
    return math.atan2(
        2.0
        * (
            float(orientation.w) * float(orientation.z)
            + float(orientation.x) * float(orientation.y)
        ),
        1.0
        - 2.0
        * (
            float(orientation.y) * float(orientation.y)
            + float(orientation.z) * float(orientation.z)
        ),
    )


def nearest_path_index(points, x_m, y_m, minimum_index=0):
    if not points:
        raise ValueError("empty path")
    lower = max(0, min(int(minimum_index), len(points) - 1))
    return min(
        range(lower, len(points)),
        key=lambda index: (
            (float(points[index]["x_m"]) - float(x_m)) ** 2
            + (float(points[index]["y_m"]) - float(y_m)) ** 2,
            index,
        ),
    )


def tracking_error(points, x_m, y_m, minimum_index=0):
    index = nearest_path_index(points, x_m, y_m, minimum_index)
    error = math.hypot(
        float(points[index]["x_m"]) - float(x_m),
        float(points[index]["y_m"]) - float(y_m),
    )
    return index, error


def pure_pursuit_command(
    points,
    x_m,
    y_m,
    yaw_rad,
    minimum_index=0,
    lookahead_m=0.70,
    maximum_speed_mps=1.20,
    maximum_yaw_rate_radps=1.00,
    wheelbase_m=0.65,
    maximum_steering_angle_rad=0.4964,
):
    """Return a bounded (speed, yaw-rate, path-index, error) command."""

    nearest, error = tracking_error(
        points, x_m, y_m, max(0, int(minimum_index) - 4)
    )
    target = nearest
    for index in range(nearest, len(points)):
        if (
            math.hypot(
                float(points[index]["x_m"]) - float(x_m),
                float(points[index]["y_m"]) - float(y_m),
            )
            >= float(lookahead_m)
        ):
            target = index
            break
        target = index

    target_point = points[target]
    dx = float(target_point["x_m"]) - float(x_m)
    dy = float(target_point["y_m"]) - float(y_m)
    distance = max(math.hypot(dx, dy), 1.0e-6)
    heading_error = angle_delta(math.atan2(dy, dx), float(yaw_rad))
    curvature = 2.0 * math.sin(heading_error) / distance

    desired_speed = min(
        float(maximum_speed_mps),
        max(0.0, float(target_point["longitudinal_velocity_mps"])),
    )
    if abs(heading_error) > 0.75:
        desired_speed *= 0.55
    elif abs(heading_error) > 0.45:
        desired_speed *= 0.75
    if error > 0.45:
        desired_speed *= 0.65

    steering_limit_curvature = (
        math.tan(float(maximum_steering_angle_rad)) / float(wheelbase_m)
    )
    curvature = max(
        -steering_limit_curvature, min(steering_limit_curvature, curvature)
    )
    yaw_rate = desired_speed * curvature
    yaw_rate = max(
        -float(maximum_yaw_rate_radps),
        min(float(maximum_yaw_rate_radps), yaw_rate),
    )
    return {
        "linear_x_mps": desired_speed,
        "angular_z_radps": yaw_rate,
        "nearest_index": nearest,
        "target_index": target,
        "tracking_error_m": error,
        "heading_error_rad": heading_error,
    }


def paired_reduction_percent(treatment_s, baseline_s):
    if len(treatment_s) != len(baseline_s) or not treatment_s:
        raise ValueError("paired nonempty samples required")
    if any(float(value) <= 0.0 for value in baseline_s):
        raise ValueError("baseline times must be positive")
    treatment_mean = sum(map(float, treatment_s)) / len(treatment_s)
    baseline_mean = sum(map(float, baseline_s)) / len(baseline_s)
    return 100.0 * (baseline_mean - treatment_mean) / baseline_mean
