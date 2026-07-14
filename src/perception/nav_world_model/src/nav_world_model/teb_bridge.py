"""Coordinate conversion and filtering for the tracker-to-TEB obstacle bridge."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class FixedFrameObstacle:
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float


def local_track_to_fixed(
    *, track_id, local_x, local_y, relative_vx, relative_vy, radius,
    robot_x, robot_y, robot_yaw, robot_linear_velocity,
):
    """Recover absolute fixed-frame position/velocity from a relative track."""

    cosine, sine = math.cos(robot_yaw), math.sin(robot_yaw)
    fixed_x = robot_x + cosine * local_x - sine * local_y
    fixed_y = robot_y + sine * local_x + cosine * local_y
    relative_fixed_vx = cosine * relative_vx - sine * relative_vy
    relative_fixed_vy = sine * relative_vx + cosine * relative_vy
    return FixedFrameObstacle(
        track_id=int(track_id),
        x=float(fixed_x),
        y=float(fixed_y),
        vx=float(relative_fixed_vx + robot_linear_velocity * cosine),
        vy=float(relative_fixed_vy + robot_linear_velocity * sine),
        radius=max(0.05, float(radius)),
    )
