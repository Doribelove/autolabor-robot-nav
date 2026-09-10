"""ROS-independent geometry/lifecycle helpers for pending-litter cleaning."""

import math
from typing import Iterable, Optional, Tuple


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def normalize_angle(angle: float) -> float:
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return math.atan2(math.sin(angle), math.cos(angle))


def stand_off_pose(
    robot_x: float,
    robot_y: float,
    target_x: float,
    target_y: float,
    stand_off_m: float,
) -> Tuple[float, float, float, float]:
    """Return goal x/y/yaw and current target distance.

    The goal lies on the current robot-to-target ray and faces the target.  It
    never asks move_base to drive over the litter coordinate.
    """

    values = (robot_x, robot_y, target_x, target_y, stand_off_m)
    if not _finite(values) or stand_off_m <= 0.0:
        raise ValueError("stand-off geometry is invalid")
    dx = target_x - robot_x
    dy = target_y - robot_y
    distance = math.hypot(dx, dy)
    if distance <= stand_off_m:
        raise ValueError("target is already inside the visual handoff distance")
    unit_x = dx / distance
    unit_y = dy / distance
    return (
        target_x - stand_off_m * unit_x,
        target_y - stand_off_m * unit_y,
        math.atan2(dy, dx),
        distance,
    )


def map_target_to_wheel_odom(
    robot_map_x: float,
    robot_map_y: float,
    robot_map_yaw: float,
    target_map_x: float,
    target_map_y: float,
    robot_odom_x: float,
    robot_odom_y: float,
    robot_odom_yaw: float,
) -> Tuple[float, float]:
    """Freeze a map point in the independent wheel-odometry frame.

    No map->wheel-odom TF is assumed.  The current base pose shared by the two
    coordinate systems supplies the single rigid conversion used for the
    remainder of the cleaning attempt.
    """

    values = (
        robot_map_x,
        robot_map_y,
        robot_map_yaw,
        target_map_x,
        target_map_y,
        robot_odom_x,
        robot_odom_y,
        robot_odom_yaw,
    )
    if not _finite(values):
        raise ValueError("map-to-wheel-odometry inputs must be finite")
    dx = target_map_x - robot_map_x
    dy = target_map_y - robot_map_y
    forward = math.cos(robot_map_yaw) * dx + math.sin(robot_map_yaw) * dy
    lateral = -math.sin(robot_map_yaw) * dx + math.cos(robot_map_yaw) * dy
    return (
        robot_odom_x
        + math.cos(robot_odom_yaw) * forward
        - math.sin(robot_odom_yaw) * lateral,
        robot_odom_y
        + math.sin(robot_odom_yaw) * forward
        + math.cos(robot_odom_yaw) * lateral,
    )


def newest_goal_id(
    previous_ids: Iterable[str], current_ids: Iterable[str]
) -> Optional[str]:
    """Return the unique newly issued simple-goal ID, otherwise fail closed."""

    previous = {str(value) for value in previous_ids}
    added = sorted({str(value) for value in current_ids} - previous)
    return added[0] if len(added) == 1 else None


def matches_cleaning_completion(evidence, sequence_floor, object_id, target_x, target_y):
    """Require this attempt's stopped, forward-coverage evidence for this point."""
    if not isinstance(evidence, dict) or sequence_floor is None:
        return False
    try:
        distance = float(evidence["minimum_distance_m"])
        x, y = float(evidence["target_x"]), float(evidence["target_y"])
        return (
            int(evidence["sequence"]) > sequence_floor
            and str(evidence["object_id"]) == str(object_id)
            and evidence.get("covered") is True
            and int(evidence["eligible_segments"]) > 0
            and all(math.isfinite(v) for v in (distance, x, y))
            and 0.0 <= distance <= 0.10
            and math.hypot(x - target_x, y - target_y) <= 1e-6
        )
    except (KeyError, TypeError, ValueError, OverflowError):
        return False
