"""Non-recursive adapter for the frozen goal-aligned execution guard."""

import math

from v3_scope_planner import execution_core as base


# Bind the original executor at import time. Runtime instrumentation may replace
# the public module attribute, but this callable remains the frozen dependency.
_FROZEN_BASE_COMMAND = base.pure_pursuit_command


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
    terminal_tolerance_m=0.12,
):
    """Apply the frozen guard without recursively using a patched module."""
    command = _FROZEN_BASE_COMMAND(
        points,
        x_m,
        y_m,
        yaw_rad,
        minimum_index=minimum_index,
        lookahead_m=lookahead_m,
        maximum_speed_mps=maximum_speed_mps,
        maximum_yaw_rate_radps=maximum_yaw_rate_radps,
        wheelbase_m=wheelbase_m,
        maximum_steering_angle_rad=maximum_steering_angle_rad,
    )
    terminal = points[-1]
    terminal_distance = math.hypot(
        float(terminal["x_m"]) - float(x_m),
        float(terminal["y_m"]) - float(y_m),
    )
    previous = points[-2]
    tangent_x = float(terminal["x_m"]) - float(previous["x_m"])
    tangent_y = float(terminal["y_m"]) - float(previous["y_m"])
    tangent_norm = max(math.hypot(tangent_x, tangent_y), 1.0e-12)
    tangent_x /= tangent_norm
    tangent_y /= tangent_norm
    terminal_plane_progress = (
        (float(x_m) - float(terminal["x_m"])) * tangent_x
        + (float(y_m) - float(terminal["y_m"])) * tangent_y
    )
    terminal_window = command["nearest_index"] >= len(points) - 3
    terminal_reached = (
        terminal_window
        and (
            terminal_distance <= float(terminal_tolerance_m)
            or terminal_plane_progress >= 0.0
        )
    )
    if terminal_reached:
        command["linear_x_mps"] = 0.0
        command["angular_z_radps"] = 0.0
    command["terminal_distance_m"] = terminal_distance
    command["terminal_plane_progress_m"] = terminal_plane_progress
    command["terminal_window"] = terminal_window
    command["terminal_reached_fail_closed"] = terminal_reached
    return command
