"""R3 terminal adapter with the preregistered 0.15 m stop tolerance."""

from v3_scope_planner import goal_aligned_execution_adapter_r1 as r1


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
    """Use the frozen nonrecursive guard with one changed tolerance."""
    return r1.pure_pursuit_command(
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
        terminal_tolerance_m=0.15,
    )
