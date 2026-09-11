"""ROS-independent geometry/lifecycle helpers for pending-litter cleaning."""

import math
from typing import Iterable, Optional, Tuple


def target_geometry(robot_pose, target_x, target_y):
    x, y, yaw = robot_pose
    if not _finite((x, y, yaw, target_x, target_y)):
        raise ValueError("non-finite target geometry")
    dx, dy = target_x - x, target_y - y
    return math.hypot(dx, dy), math.degrees(normalize_angle(math.atan2(dy, dx) - yaw))


def approach_candidates(robot_pose, target_x, target_y, radius=1.5):
    """Bounded, target-facing poses. Near/behind targets still get navigation.

    These are candidate goals, not straight-line driving commands. The caller
    must check the current costmap footprint and query the normal planner.
    """
    x, y, yaw = robot_pose
    if not _finite((x, y, yaw, target_x, target_y, radius)) or not 1.0 <= radius <= 2.0:
        raise ValueError("invalid approach geometry")
    bearing = math.atan2(target_y - y, target_x - x) if math.hypot(target_x-x, target_y-y) > 1e-6 else yaw
    for offset in (0, 30, -30, 60, -60, 90, -90, 180):
        angle = bearing + math.radians(offset)
        yield (target_x - radius * math.cos(angle),
               target_y - radius * math.sin(angle), normalize_angle(angle))


def goal_subscriber_error(subscribers, bridge, observers):
    actual = set(subscribers)
    if bridge not in actual:
        return "缺少导航执行桥：" + bridge
    unexpected = actual - {bridge} - set(observers)
    if unexpected:
        return "导航目标存在未授权订阅者：" + ", ".join(sorted(unexpected))
    return ""


def footprint_is_free(grid, pose, footprint):
    """Conservative convex footprint/cell SAT check on an OccupancyGrid.

    Tests every intersected cell, including polygon interior and boundary;
    unknown/outside/inscribed/lethal space fails closed. No planner settings
    are modified. The caller supplies the actual padded robot footprint.
    """
    info = grid.info
    resolution = float(info.resolution)
    width, height = int(info.width), int(info.height)
    if (resolution <= 0 or not math.isfinite(resolution) or width <= 0 or height <= 0
            or len(grid.data) != width * height or len(footprint) < 3):
        return False
    ox, oy = info.origin.position.x, info.origin.position.y
    q = info.origin.orientation
    origin_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
    x, y, yaw = pose
    if not _finite((x, y, yaw, ox, oy, origin_yaw)):
        return False
    polygon = []
    for fx, fy in footprint:
        if not _finite((fx, fy)):
            return False
        dx, dy = x + math.cos(yaw)*fx - math.sin(yaw)*fy - ox, y + math.sin(yaw)*fx + math.cos(yaw)*fy - oy
        polygon.append(((math.cos(origin_yaw)*dx+math.sin(origin_yaw)*dy)/resolution,
                        (-math.sin(origin_yaw)*dx+math.cos(origin_yaw)*dy)/resolution))
    turns = []
    for i in range(len(polygon)):
        a, b, c = polygon[i-2], polygon[i-1], polygon[i]
        turns.append((b[0]-a[0])*(c[1]-b[1])-(b[1]-a[1])*(c[0]-b[0]))
    if (not any(abs(t) > 1e-9 for t in turns) or
            (any(t > 1e-9 for t in turns) and any(t < -1e-9 for t in turns))):
        return False  # Degenerate/non-convex footprints are not SAT-safe.
    lo_x, hi_x = min(p[0] for p in polygon), max(p[0] for p in polygon)
    lo_y, hi_y = min(p[1] for p in polygon), max(p[1] for p in polygon)
    if lo_x < 0 or lo_y < 0 or hi_x >= width or hi_y >= height:
        return False
    axes = [(1., 0.), (0., 1.)]
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        axes.append((a[1]-b[1], b[0]-a[0]))
    projected = [(ax, ay, min(ax*x+ay*y for x,y in polygon), max(ax*x+ay*y for x,y in polygon)) for ax,ay in axes]
    for cy in range(max(0, math.floor(lo_y)-1), math.floor(hi_y)+1):
        for cx in range(max(0, math.floor(lo_x)-1), math.floor(hi_x)+1):
            cost = grid.data[cy*width+cx]
            if 0 <= cost < 99:
                continue
            corners = ((cx,cy), (cx+1,cy), (cx+1,cy+1), (cx,cy+1))
            if all(not (max(ax*x+ay*y for x,y in corners) < low - 1e-9 or
                        min(ax*x+ay*y for x,y in corners) > high + 1e-9)
                   for ax,ay,low,high in projected):
                return False
    return True


def matches_visual_completion(evidence, sequence_floor, object_id, target_x, target_y, request_id):
    if not isinstance(evidence, dict) or sequence_floor is None or not request_id:
        return False
    try:
        return (int(evidence['sequence']) > sequence_floor
                and evidence.get('request_id') == request_id
                and evidence.get('completion_policy') == 'visual_servo'
                and evidence.get('stopped') is True
                and str(evidence['object_id']) == str(object_id)
                and int(evidence['acquired_object_id']) > 0
                and math.hypot(float(evidence['target_x'])-target_x,
                               float(evidence['target_y'])-target_y) <= 1e-6
                and 0.05 <= float(evidence['blind_target_m']) <= 0.50
                and float(evidence['blind_target_m']) - 0.005 <= float(evidence['forward_m']) <= 0.55
                and float(evidence['forward_m']) <= float(evidence['path_m']) <= 0.55)
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


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
