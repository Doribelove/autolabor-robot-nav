"""Shared cleaning footprint and wheel-frame geometry."""
import math
from typing import Tuple


def _finite(values):
    return all(math.isfinite(float(v)) for v in values)


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


