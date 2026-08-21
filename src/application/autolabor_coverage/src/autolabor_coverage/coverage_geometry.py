#!/usr/bin/env python3
"""Dependency-free geometry used by the static-map coverage manager.

The planner deliberately works from the immutable OccupancyGrid rather than a
live costmap. Dynamic obstacles remain the responsibility of move_base and the
mission retry state machine.
"""

from dataclasses import dataclass
from collections import deque
import hashlib
import math


EPSILON = 1.0e-9


@dataclass(frozen=True)
class Point:
    x: float
    y: float


@dataclass(frozen=True)
class Swath:
    start: Point
    end: Point
    scan_v: float
    length: float


@dataclass
class CoveragePlan:
    angle: float
    spacing: float
    swaths: list
    requested_area: float
    reachable_area: float
    unreachable_area: float
    score: float


class GridMap:
    def __init__(self, width, height, resolution, origin_x, origin_y, data):
        self.width = int(width)
        self.height = int(height)
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.data = tuple(int(value) for value in data)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("map dimensions must be positive")
        if not math.isfinite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("map resolution must be positive")
        if len(self.data) != self.width * self.height:
            raise ValueError("map data length does not match dimensions")

    def world_to_cell(self, x, y):
        mx = int(math.floor((x - self.origin_x) / self.resolution))
        my = int(math.floor((y - self.origin_y) / self.resolution))
        if mx < 0 or my < 0 or mx >= self.width or my >= self.height:
            return None
        return mx, my

    def is_free(self, x, y):
        cell = self.world_to_cell(x, y)
        if cell is None:
            return False
        mx, my = cell
        return self.data[my * self.width + mx] == 0

    def digest(self):
        digest = hashlib.sha256()
        digest.update(
            ("%d:%d:%.9f:%.9f:%.9f:" % (
                self.width,
                self.height,
                self.resolution,
                self.origin_x,
                self.origin_y,
            )).encode("ascii")
        )
        digest.update(bytes((value + 1) & 0xFF for value in self.data))
        return digest.hexdigest()

    def reachable_free_cells(self, seed):
        """Return the 4-connected known-free component containing ``seed``."""
        cell = self.world_to_cell(seed.x, seed.y)
        if cell is None:
            return set()
        seed_x, seed_y = cell
        if self.data[seed_y * self.width + seed_x] != 0:
            nearest = None
            maximum_radius = max(1, int(math.ceil(2.0 / self.resolution)))
            for radius in range(1, maximum_radius + 1):
                for x in range(max(0, seed_x - radius),
                               min(self.width, seed_x + radius + 1)):
                    for y in (seed_y - radius, seed_y + radius):
                        if (0 <= y < self.height and
                                self.data[y * self.width + x] == 0):
                            nearest = (x, y)
                            break
                    if nearest:
                        break
                if not nearest:
                    for y in range(max(0, seed_y - radius + 1),
                                   min(self.height, seed_y + radius)):
                        for x in (seed_x - radius, seed_x + radius):
                            if (0 <= x < self.width and
                                    self.data[y * self.width + x] == 0):
                                nearest = (x, y)
                                break
                        if nearest:
                            break
                if nearest:
                    break
            if nearest is None:
                return set()
            cell = nearest
        reachable = {cell}
        queue = deque([cell])
        while queue:
            x, y = queue.popleft()
            for other in ((x - 1, y), (x + 1, y),
                          (x, y - 1), (x, y + 1)):
                other_x, other_y = other
                if (other in reachable or other_x < 0 or other_y < 0 or
                        other_x >= self.width or other_y >= self.height or
                        self.data[other_y * self.width + other_x] != 0):
                    continue
                reachable.add(other)
                queue.append(other)
        return reachable


def polygon_area(points):
    return abs(signed_polygon_area(points))


def signed_polygon_area(points):
    if len(points) < 3:
        return 0.0
    total = 0.0
    for index, point in enumerate(points):
        other = points[(index + 1) % len(points)]
        total += point.x * other.y - other.x * point.y
    return 0.5 * total


def _orientation(a, b, c):
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)


def _on_segment(a, b, point):
    return (
        min(a.x, b.x) - EPSILON <= point.x <= max(a.x, b.x) + EPSILON
        and min(a.y, b.y) - EPSILON <= point.y <= max(a.y, b.y) + EPSILON
        and abs(_orientation(a, b, point)) <= EPSILON
    )


def segments_intersect(a, b, c, d):
    ab_c = _orientation(a, b, c)
    ab_d = _orientation(a, b, d)
    cd_a = _orientation(c, d, a)
    cd_b = _orientation(c, d, b)
    if ((ab_c > EPSILON and ab_d < -EPSILON) or
            (ab_c < -EPSILON and ab_d > EPSILON)) and (
            (cd_a > EPSILON and cd_b < -EPSILON) or
            (cd_a < -EPSILON and cd_b > EPSILON)):
        return True
    return (
        (abs(ab_c) <= EPSILON and _on_segment(a, b, c))
        or (abs(ab_d) <= EPSILON and _on_segment(a, b, d))
        or (abs(cd_a) <= EPSILON and _on_segment(c, d, a))
        or (abs(cd_b) <= EPSILON and _on_segment(c, d, b))
    )


def validate_polygon(points, minimum_area=0.5):
    if len(points) < 3:
        return False, "coverage region needs at least three points"
    for point in points:
        if not math.isfinite(point.x) or not math.isfinite(point.y):
            return False, "coverage region contains a non-finite point"
    for index, point in enumerate(points):
        other = points[(index + 1) % len(points)]
        if math.hypot(point.x - other.x, point.y - other.y) < 1.0e-3:
            return False, "coverage region contains duplicate adjacent points"
    edge_count = len(points)
    for first in range(edge_count):
        a = points[first]
        b = points[(first + 1) % edge_count]
        for second in range(first + 1, edge_count):
            if second in (first, first + 1) or (
                    first == 0 and second == edge_count - 1):
                continue
            c = points[second]
            d = points[(second + 1) % edge_count]
            if segments_intersect(a, b, c, d):
                return False, "coverage region is self-intersecting"
    area = polygon_area(points)
    if area < minimum_area:
        return False, "coverage region area is too small"
    return True, "ok"


def point_in_polygon(point, polygon):
    inside = False
    previous = polygon[-1]
    for current in polygon:
        if _on_segment(previous, current, point):
            return True
        crosses = ((current.y > point.y) != (previous.y > point.y))
        if crosses:
            x_at_y = ((previous.x - current.x) * (point.y - current.y) /
                      (previous.y - current.y) + current.x)
            if point.x < x_at_y:
                inside = not inside
        previous = current
    return inside


def rotate_to_scan(point, angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return Point(cosine * point.x + sine * point.y,
                 -sine * point.x + cosine * point.y)


def rotate_from_scan(point, angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return Point(cosine * point.x - sine * point.y,
                 sine * point.x + cosine * point.y)


def scanline_intersections(scan_polygon, scan_v):
    intersections = []
    previous = scan_polygon[-1]
    for current in scan_polygon:
        low = min(previous.y, current.y)
        high = max(previous.y, current.y)
        if high - low > EPSILON and low <= scan_v < high:
            ratio = (scan_v - previous.y) / (current.y - previous.y)
            intersections.append(previous.x + ratio * (current.x - previous.x))
        previous = current
    intersections.sort()
    if len(intersections) % 2:
        intersections = intersections[:-1]
    return list(zip(intersections[0::2], intersections[1::2]))


def sample_path(start, end, spacing):
    distance = math.hypot(end.x - start.x, end.y - start.y)
    count = max(1, int(math.ceil(distance / max(spacing, 1.0e-3))))
    return [
        Point(start.x + (end.x - start.x) * index / count,
              start.y + (end.y - start.y) * index / count)
        for index in range(count + 1)
    ]


class CoveragePlanner:
    def __init__(self, grid_map, footprint_front=0.62, footprint_rear=0.62,
                 footprint_half_width=0.45, minimum_swath_length=1.2,
                 angle_step_degrees=15.0):
        self.grid = grid_map
        self.front = float(footprint_front)
        self.rear = float(footprint_rear)
        self.half_width = float(footprint_half_width)
        self.minimum_swath_length = float(minimum_swath_length)
        self.angle_step = math.radians(float(angle_step_degrees))

    def _pose_is_free(self, point, angle, cache):
        key = (
            int(round((point.x - self.grid.origin_x) / self.grid.resolution)),
            int(round((point.y - self.grid.origin_y) / self.grid.resolution)),
            int(round(angle * 10000.0)),
        )
        if key in cache:
            return cache[key]
        longitudinal_step = max(self.grid.resolution, 0.10)
        lateral_step = max(self.grid.resolution, 0.10)
        x_count = max(1, int(math.ceil((self.front + self.rear) / longitudinal_step)))
        y_count = max(1, int(math.ceil(2.0 * self.half_width / lateral_step)))
        cosine = math.cos(angle)
        sine = math.sin(angle)
        for x_index in range(x_count + 1):
            local_x = -self.rear + (self.front + self.rear) * x_index / x_count
            for y_index in range(y_count + 1):
                local_y = -self.half_width + 2.0 * self.half_width * y_index / y_count
                world_x = point.x + cosine * local_x - sine * local_y
                world_y = point.y + sine * local_x + cosine * local_y
                if not self.grid.is_free(world_x, world_y):
                    cache[key] = False
                    return False
        cache[key] = True
        return True

    @staticmethod
    def _candidate_angles(points, angle_step):
        angles = set()
        count = max(1, int(round(math.pi / angle_step)))
        for index in range(count):
            angles.add(round(index * math.pi / count, 7))
        for index, point in enumerate(points):
            other = points[(index + 1) % len(points)]
            if math.hypot(other.x - point.x, other.y - point.y) < 0.2:
                continue
            angle = math.atan2(other.y - point.y, other.x - point.x) % math.pi
            angles.add(round(angle, 7))
        return sorted(angles)

    def _plan_angle(self, polygon, operation_width, spacing, angle,
                    reachable_cells=None):
        scan_polygon = [rotate_to_scan(point, angle) for point in polygon]
        minimum_v = min(point.y for point in scan_polygon)
        maximum_v = max(point.y for point in scan_polygon)
        if maximum_v - minimum_v < self.grid.resolution:
            return []
        first_v = minimum_v + 0.5 * operation_width
        last_v = maximum_v - 0.5 * operation_width
        if first_v > last_v:
            scan_values = [0.5 * (minimum_v + maximum_v)]
        else:
            count = max(1, int(math.floor((last_v - first_v) / spacing)) + 1)
            scan_values = [first_v + index * spacing for index in range(count)]
            if last_v - scan_values[-1] > 0.55 * spacing:
                scan_values.append(last_v)
        cache = {}
        swaths = []
        sample_step = self.grid.resolution
        for scan_v in scan_values:
            for minimum_u, maximum_u in scanline_intersections(scan_polygon, scan_v):
                if maximum_u - minimum_u < self.minimum_swath_length:
                    continue
                sample_count = max(1, int(math.ceil((maximum_u - minimum_u) / sample_step)))
                run_start = None
                previous_point = None
                for index in range(sample_count + 1):
                    scan_u = minimum_u + (maximum_u - minimum_u) * index / sample_count
                    point = rotate_from_scan(Point(scan_u, scan_v), angle)
                    available = self._pose_is_free(point, angle, cache)
                    if available and run_start is None:
                        run_start = point
                    if available:
                        previous_point = point
                    if (not available or index == sample_count) and run_start is not None:
                        run_end = previous_point
                        length = math.hypot(run_end.x - run_start.x,
                                            run_end.y - run_start.y)
                        start_cell = self.grid.world_to_cell(run_start.x, run_start.y)
                        end_cell = self.grid.world_to_cell(run_end.x, run_end.y)
                        connected = (
                            reachable_cells is None or
                            (start_cell in reachable_cells and
                             end_cell in reachable_cells)
                        )
                        if length >= self.minimum_swath_length and connected:
                            swaths.append(Swath(run_start, run_end, scan_v, length))
                        run_start = None
                        previous_point = None
        return swaths

    def plan(self, polygon, operation_width, overlap_ratio, reachable_seed=None):
        valid, reason = validate_polygon(polygon)
        if not valid:
            raise ValueError(reason)
        if not math.isfinite(operation_width) or not 0.30 <= operation_width <= 3.0:
            raise ValueError("operation width must be in [0.30, 3.00] m")
        if not math.isfinite(overlap_ratio) or not 0.0 <= overlap_ratio <= 0.5:
            raise ValueError("overlap ratio must be in [0.0, 0.5]")
        spacing = operation_width * (1.0 - overlap_ratio)
        area = polygon_area(polygon)
        reachable_cells = (
            self.grid.reachable_free_cells(reachable_seed)
            if reachable_seed is not None else None
        )
        if reachable_seed is not None and not reachable_cells:
            raise ValueError("vehicle is outside the known free static map")
        best = None
        for angle in self._candidate_angles(polygon, self.angle_step):
            swaths = self._plan_angle(
                polygon, operation_width, spacing, angle, reachable_cells)
            if not swaths:
                continue
            path_length = sum(swath.length for swath in swaths)
            reachable = min(area, path_length * operation_width)
            unreachable = max(0.0, area - reachable)
            split_penalty = max(0, len(swaths) - int(math.ceil(area / max(
                operation_width * max(math.sqrt(area), 1.0), EPSILON))))
            score = (path_length + len(swaths) * math.pi * 1.2 +
                     split_penalty * 2.0 + unreachable * 100.0)
            candidate = CoveragePlan(
                angle=angle,
                spacing=spacing,
                swaths=swaths,
                requested_area=area,
                reachable_area=reachable,
                unreachable_area=unreachable,
                score=score,
            )
            if best is None or candidate.score < best.score:
                best = candidate
        if best is None:
            raise ValueError("selected region has no footprint-safe swath")
        return best


def order_swaths(swaths, current, spacing, minimum_turning_radius):
    if not swaths:
        return []
    ordered_by_v = sorted(swaths, key=lambda swath: (
        round(swath.scan_v / max(spacing, EPSILON)),
        min(swath.start.x, swath.end.x),
    ))
    stride = max(1, int(math.ceil(2.0 * minimum_turning_radius /
                                  max(spacing, EPSILON))))
    index_order = []
    for residue in range(stride):
        index_order.extend(range(residue, len(ordered_by_v), stride))
    # The region is not a geofence, so move_base may transit through any known
    # free cells.  Rotate the turn-friendly row sequence to the endpoint nearest
    # the vehicle; this provides a deterministic, low-cost coverage entry.
    first_position = min(
        range(len(index_order)),
        key=lambda position: min(
            math.hypot(current.x - ordered_by_v[index_order[position]].start.x,
                       current.y - ordered_by_v[index_order[position]].start.y),
            math.hypot(current.x - ordered_by_v[index_order[position]].end.x,
                       current.y - ordered_by_v[index_order[position]].end.y),
        ),
    )
    index_order = index_order[first_position:] + index_order[:first_position]
    route = []
    cursor = current
    for index in index_order:
        swath = ordered_by_v[index]
        start_distance = math.hypot(cursor.x - swath.start.x,
                                    cursor.y - swath.start.y)
        end_distance = math.hypot(cursor.x - swath.end.x,
                                  cursor.y - swath.end.y)
        if end_distance < start_distance:
            swath = Swath(swath.end, swath.start, swath.scan_v, swath.length)
        route.append(swath)
        cursor = swath.end
    return route
