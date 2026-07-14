"""Deterministic local geometry, clustering, tracking, and prediction core.

This module intentionally has no ROS imports. Gazebo/Pedsim truth and scene labels
are not accepted inputs; those belong only in external evaluation code.
"""

from dataclasses import dataclass, field
import math
from typing import Dict, Iterable, List, Optional, Sequence, Tuple


class ScanValidationError(ValueError):
    """Raised when LaserScan metadata is inconsistent or unsafe to consume."""


@dataclass(frozen=True)
class Point2:
    x: float
    y: float


@dataclass(frozen=True)
class ScanFrame:
    stamp_s: float
    frame_id: str
    angle_min: float
    angle_max: float
    angle_increment: float
    range_min: float
    range_max: float
    ranges: Tuple[float, ...]


@dataclass(frozen=True)
class Detection:
    x: float
    y: float
    radius: float
    point_count: int


@dataclass(frozen=True)
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    linear_velocity: float = 0.0


@dataclass(frozen=True)
class Prediction:
    time_from_start_s: float
    x: float
    y: float
    vx: float
    vy: float
    position_variance: float
    confidence: float


@dataclass(frozen=True)
class TrackEstimate:
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    confidence: float
    age_s: float
    miss_count: int
    last_update_s: float
    motion_class: str
    predictions: Tuple[Prediction, ...]


@dataclass
class _Track:
    track_id: int
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    created_s: float
    updated_s: float
    hits: int = 1
    miss_count: int = 0


@dataclass(frozen=True)
class GeometryEstimate:
    front_clearance_m: float
    left_clearance_m: float
    right_clearance_m: float
    rear_clearance_m: float
    footprint_clearance_m: float
    obstacle_density: float
    static_persistence: float
    corridor_width_m: float
    corridor_axis_yaw_rad: float
    corridor_parallel_confidence: float
    corridor_center_offset_m: float
    dead_end_score: float
    path_curvature: float
    signed_cross_track_error_m: float
    signed_heading_error_rad: float
    goal_direction_stability: float
    front_covered: bool
    left_covered: bool
    right_covered: bool
    rear_covered: bool


def _finite(value: float) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _clamp(value: float, minimum: float = 0.0, maximum: float = 1.0) -> float:
    return max(minimum, min(maximum, value))


def validate_scan(scan: ScanFrame, ray_count_required: Optional[int] = None) -> None:
    values = (
        scan.stamp_s,
        scan.angle_min,
        scan.angle_max,
        scan.angle_increment,
        scan.range_min,
        scan.range_max,
    )
    if not all(_finite(value) for value in values):
        raise ScanValidationError("scan metadata must be finite")
    if not scan.frame_id:
        raise ScanValidationError("scan frame_id is required")
    if scan.stamp_s < 0.0:
        raise ScanValidationError("scan stamp cannot be negative")
    if scan.angle_increment <= 0.0 or scan.angle_max <= scan.angle_min:
        raise ScanValidationError("scan angles must be strictly increasing")
    if scan.range_min < 0.0 or scan.range_max <= scan.range_min:
        raise ScanValidationError("scan range bounds are invalid")
    if len(scan.ranges) < 2:
        raise ScanValidationError("scan requires at least two rays")
    expected_max = scan.angle_min + scan.angle_increment * (len(scan.ranges) - 1)
    tolerance = max(1.0e-6, abs(scan.angle_increment) * 0.51)
    if abs(expected_max - scan.angle_max) > tolerance:
        raise ScanValidationError("angle range and ray count are inconsistent")
    if ray_count_required is not None and len(scan.ranges) != ray_count_required:
        raise ScanValidationError("scan ray count drifted")


def scan_to_local_points(scan: ScanFrame) -> Tuple[Optional[Point2], ...]:
    """Convert valid rays while preserving invalid-ray breaks for clustering."""

    validate_scan(scan)
    points: List[Optional[Point2]] = []
    for index, raw_range in enumerate(scan.ranges):
        if not _finite(raw_range):
            points.append(None)
            continue
        distance = float(raw_range)
        if distance < scan.range_min or distance > scan.range_max:
            points.append(None)
            continue
        angle = scan.angle_min + index * scan.angle_increment
        points.append(Point2(distance * math.cos(angle), distance * math.sin(angle)))
    return tuple(points)


def transform_points(
    points: Sequence[Optional[Point2]], tx: float, ty: float, yaw: float
) -> Tuple[Optional[Point2], ...]:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    transformed: List[Optional[Point2]] = []
    for point in points:
        if point is None:
            transformed.append(None)
        else:
            transformed.append(
                Point2(
                    tx + cosine * point.x - sine * point.y,
                    ty + sine * point.x + cosine * point.y,
                )
            )
    return tuple(transformed)


def _distance(first: Point2, second: Point2) -> float:
    return math.hypot(first.x - second.x, first.y - second.y)


def _cluster_ordered_points(
    points: Sequence[Optional[Point2]], maximum_gap_m: float
) -> List[List[Point2]]:
    clusters: List[List[Point2]] = []
    current: List[Point2] = []
    for point in points:
        if point is None:
            if current:
                clusters.append(current)
                current = []
            continue
        if current and _distance(current[-1], point) > maximum_gap_m:
            clusters.append(current)
            current = []
        current.append(point)
    if current:
        clusters.append(current)
    if len(clusters) >= 2 and points and points[0] is not None and points[-1] is not None:
        if _distance(clusters[-1][-1], clusters[0][0]) <= maximum_gap_m:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()
    return clusters


def extract_detections(
    fixed_points: Sequence[Optional[Point2]],
    maximum_gap_m: float,
    minimum_points: int,
    maximum_diameter_m: float,
    maximum_range_m: float,
    origin: Point2,
) -> Tuple[Detection, ...]:
    detections: List[Detection] = []
    for cluster in _cluster_ordered_points(fixed_points, maximum_gap_m):
        if len(cluster) < minimum_points:
            continue
        xs = [point.x for point in cluster]
        ys = [point.y for point in cluster]
        diameter = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        if diameter > maximum_diameter_m:
            continue
        x = sum(xs) / len(xs)
        y = sum(ys) / len(ys)
        if math.hypot(x - origin.x, y - origin.y) > maximum_range_m:
            continue
        radius = max(0.08, min(maximum_diameter_m * 0.5, diameter * 0.5))
        detections.append(Detection(x=x, y=y, radius=radius, point_count=len(cluster)))
    return tuple(detections)


def _sector_clearance(
    scan: ScanFrame, center: float, half_width: float
) -> Tuple[float, bool]:
    values: List[float] = []
    covered = False
    for index, raw_range in enumerate(scan.ranges):
        angle = _wrap(scan.angle_min + index * scan.angle_increment - center)
        if abs(angle) <= half_width:
            covered = True
            if _finite(raw_range) and scan.range_min <= raw_range <= scan.range_max:
                values.append(float(raw_range))
    return (min(values) if values else scan.range_max, covered)


def _line_fit(points: Sequence[Point2]) -> Optional[Tuple[float, float, float]]:
    if len(points) < 5:
        return None
    mean_x = sum(point.x for point in points) / len(points)
    mean_y = sum(point.y for point in points) / len(points)
    variance_x = sum((point.x - mean_x) ** 2 for point in points)
    if variance_x <= 1.0e-8:
        return None
    slope = sum((point.x - mean_x) * (point.y - mean_y) for point in points) / variance_x
    intercept = mean_y - slope * mean_x
    residual = math.sqrt(
        sum((point.y - (slope * point.x + intercept)) ** 2 for point in points)
        / len(points)
    )
    return slope, intercept, residual


def _corridor_geometry(
    points: Sequence[Point2],
    minimum_x: float,
    maximum_x: float,
    minimum_side_distance: float,
    maximum_side_distance: float,
    maximum_residual_m: float,
    maximum_slope_difference: float,
) -> Tuple[float, float, float, float]:
    left = [
        point for point in points
        if minimum_x <= point.x <= maximum_x
        and minimum_side_distance <= point.y <= maximum_side_distance
    ]
    right = [
        point for point in points
        if minimum_x <= point.x <= maximum_x
        and -maximum_side_distance <= point.y <= -minimum_side_distance
    ]
    left_fit, right_fit = _line_fit(left), _line_fit(right)
    if left_fit is None or right_fit is None:
        return 0.0, 0.0, 0.0, 0.0
    left_slope, left_intercept, left_residual = left_fit
    right_slope, right_intercept, right_residual = right_fit
    width = left_intercept - right_intercept
    if width <= 0.0:
        return 0.0, 0.0, 0.0, 0.0
    residual_score = _clamp(
        1.0 - max(left_residual, right_residual) / max(maximum_residual_m, 1.0e-6)
    )
    parallel_score = _clamp(
        1.0 - abs(left_slope - right_slope) / max(maximum_slope_difference, 1.0e-6)
    )
    support_score = _clamp(min(len(left), len(right)) / 20.0)
    confidence = residual_score * parallel_score * support_score
    axis_yaw = math.atan(0.5 * (left_slope + right_slope))
    center_offset = 0.5 * (left_intercept + right_intercept)
    return width, axis_yaw, confidence, center_offset


def compute_path_metrics(
    robot: RobotState, path: Sequence[Point2]
) -> Tuple[float, float, float, float]:
    if len(path) < 2:
        return 0.0, 0.0, 0.0, 0.0
    nearest_index = min(
        range(len(path)),
        key=lambda index: math.hypot(path[index].x - robot.x, path[index].y - robot.y),
    )
    start_index = min(nearest_index, len(path) - 2)
    first, second = path[start_index], path[start_index + 1]
    heading = math.atan2(second.y - first.y, second.x - first.x)
    dx, dy = robot.x - first.x, robot.y - first.y
    cross_track = -math.sin(heading) * dx + math.cos(heading) * dy
    heading_error = _wrap(heading - robot.yaw)
    headings: List[float] = []
    for index in range(start_index, min(len(path) - 1, start_index + 5)):
        delta_x = path[index + 1].x - path[index].x
        delta_y = path[index + 1].y - path[index].y
        if math.hypot(delta_x, delta_y) > 1.0e-6:
            headings.append(math.atan2(delta_y, delta_x))
    curvature = 0.0
    if len(headings) >= 2:
        distances = [
            math.hypot(path[index + 1].x - path[index].x,
                       path[index + 1].y - path[index].y)
            for index in range(start_index, min(len(path) - 1, start_index + len(headings)))
        ]
        arc_length = max(1.0e-6, sum(distances))
        curvature = sum(abs(_wrap(headings[index + 1] - headings[index]))
                        for index in range(len(headings) - 1)) / arc_length
    mean_cos = sum(math.cos(value) for value in headings) / max(1, len(headings))
    mean_sin = sum(math.sin(value) for value in headings) / max(1, len(headings))
    stability = _clamp(math.hypot(mean_cos, mean_sin))
    return curvature, cross_track, heading_error, stability


def compute_local_geometry(
    scan: ScanFrame,
    local_points: Sequence[Optional[Point2]],
    tracks: Sequence[TrackEstimate],
    path_metrics: Tuple[float, float, float, float],
    sector_half_width_rad: float,
    density_radius_m: float,
    robot_radius_m: float,
    corridor_parameters: Dict[str, float],
) -> GeometryEstimate:
    validate_scan(scan)
    front, front_covered = _sector_clearance(scan, 0.0, sector_half_width_rad)
    left, left_covered = _sector_clearance(scan, math.pi * 0.5, sector_half_width_rad)
    right, right_covered = _sector_clearance(scan, -math.pi * 0.5, sector_half_width_rad)
    rear, rear_covered = _sector_clearance(scan, math.pi, sector_half_width_rad)
    valid_points = [point for point in local_points if point is not None]
    close_points = [point for point in valid_points if math.hypot(point.x, point.y) <= density_radius_m]
    obstacle_density = len(close_points) / max(1, len(scan.ranges))
    minimum_range = min(
        (math.hypot(point.x, point.y) for point in valid_points),
        default=scan.range_max,
    )
    stationary = sum(track.motion_class == "STATIONARY" for track in tracks)
    static_persistence = stationary / max(1, len(tracks)) if tracks else 0.0
    width, axis, confidence, offset = _corridor_geometry(
        valid_points,
        corridor_parameters["minimum_x_m"],
        corridor_parameters["maximum_x_m"],
        corridor_parameters["minimum_side_distance_m"],
        corridor_parameters["maximum_side_distance_m"],
        corridor_parameters["maximum_residual_m"],
        corridor_parameters["maximum_slope_difference"],
    )
    front_block = _clamp((corridor_parameters["dead_end_front_distance_m"] - front)
                         / corridor_parameters["dead_end_front_distance_m"])
    side_enclosure = _clamp(
        (corridor_parameters["dead_end_side_distance_m"] - min(left, right))
        / corridor_parameters["dead_end_side_distance_m"]
    )
    dead_end_score = 0.7 * front_block + 0.3 * side_enclosure
    curvature, cross_track, heading_error, stability = path_metrics
    return GeometryEstimate(
        front_clearance_m=front,
        left_clearance_m=left,
        right_clearance_m=right,
        rear_clearance_m=rear,
        footprint_clearance_m=max(0.0, minimum_range - robot_radius_m),
        obstacle_density=obstacle_density,
        static_persistence=static_persistence,
        corridor_width_m=width,
        corridor_axis_yaw_rad=axis,
        corridor_parallel_confidence=confidence,
        corridor_center_offset_m=offset,
        dead_end_score=dead_end_score,
        path_curvature=curvature,
        signed_cross_track_error_m=cross_track,
        signed_heading_error_rad=heading_error,
        goal_direction_stability=stability,
        front_covered=front_covered,
        left_covered=left_covered,
        right_covered=right_covered,
        rear_covered=rear_covered,
    )


class MultiObjectTracker:
    """Greedy gated alpha-beta tracker with deterministic constant-velocity prediction."""

    def __init__(
        self,
        association_gate_m: float,
        alpha: float,
        beta: float,
        minimum_confirmed_hits: int,
        maximum_misses: int,
        maximum_dt_s: float,
        stationary_speed_max_mps: float,
        dynamic_speed_min_mps: float,
        prediction_horizon_s: float,
        prediction_step_s: float,
        confidence_decay_per_s: float,
        crossing_lateral_speed_min_mps: float,
        crossing_path_half_width_m: float,
    ):
        if not 0.0 < alpha <= 1.0 or not 0.0 <= beta <= 1.0:
            raise ValueError("alpha/beta gains are invalid")
        if association_gate_m <= 0.0 or maximum_dt_s <= 0.0:
            raise ValueError("tracker distances and dt must be positive")
        if minimum_confirmed_hits < 1 or maximum_misses < 0:
            raise ValueError("tracker hit/miss limits are invalid")
        if prediction_step_s <= 0.0 or prediction_horizon_s < prediction_step_s:
            raise ValueError("prediction horizon/step are invalid")
        self.association_gate_m = association_gate_m
        self.alpha = alpha
        self.beta = beta
        self.minimum_confirmed_hits = minimum_confirmed_hits
        self.maximum_misses = maximum_misses
        self.maximum_dt_s = maximum_dt_s
        self.stationary_speed_max_mps = stationary_speed_max_mps
        self.dynamic_speed_min_mps = dynamic_speed_min_mps
        self.prediction_horizon_s = prediction_horizon_s
        self.prediction_step_s = prediction_step_s
        self.confidence_decay_per_s = confidence_decay_per_s
        self.crossing_lateral_speed_min_mps = crossing_lateral_speed_min_mps
        self.crossing_path_half_width_m = crossing_path_half_width_m
        self._tracks: Dict[int, _Track] = {}
        self._next_id = 1
        self.last_update_s: Optional[float] = None

    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1
        self.last_update_s = None

    def update(
        self,
        detections: Sequence[Detection],
        stamp_s: float,
        robot: RobotState,
    ) -> Tuple[TrackEstimate, ...]:
        if not _finite(stamp_s) or stamp_s < 0.0:
            raise ValueError("tracker stamp is invalid")
        if self.last_update_s is not None and stamp_s <= self.last_update_s:
            raise ValueError("tracker stamps must be strictly increasing")
        candidate_pairs: List[Tuple[float, int, int]] = []
        predicted_positions: Dict[int, Tuple[float, float, float]] = {}
        for track_id, track in self._tracks.items():
            dt = min(self.maximum_dt_s, max(1.0e-3, stamp_s - track.updated_s))
            predicted_x = track.x + track.vx * dt
            predicted_y = track.y + track.vy * dt
            predicted_positions[track_id] = (predicted_x, predicted_y, dt)
            for detection_index, detection in enumerate(detections):
                distance = math.hypot(detection.x - predicted_x, detection.y - predicted_y)
                if distance <= self.association_gate_m:
                    candidate_pairs.append((distance, track_id, detection_index))
        assigned_tracks, assigned_detections = set(), set()
        assignments: List[Tuple[int, int]] = []
        for _, track_id, detection_index in sorted(candidate_pairs):
            if track_id in assigned_tracks or detection_index in assigned_detections:
                continue
            assigned_tracks.add(track_id)
            assigned_detections.add(detection_index)
            assignments.append((track_id, detection_index))
        for track_id, detection_index in assignments:
            track = self._tracks[track_id]
            detection = detections[detection_index]
            predicted_x, predicted_y, dt = predicted_positions[track_id]
            residual_x = detection.x - predicted_x
            residual_y = detection.y - predicted_y
            track.x = predicted_x + self.alpha * residual_x
            track.y = predicted_y + self.alpha * residual_y
            track.vx += self.beta * residual_x / dt
            track.vy += self.beta * residual_y / dt
            track.radius = 0.7 * track.radius + 0.3 * detection.radius
            track.updated_s = stamp_s
            track.hits += 1
            track.miss_count = 0
        for track_id, track in list(self._tracks.items()):
            if track_id in assigned_tracks:
                continue
            predicted_x, predicted_y, _ = predicted_positions[track_id]
            track.x, track.y = predicted_x, predicted_y
            track.updated_s = stamp_s
            track.miss_count += 1
            if track.miss_count > self.maximum_misses:
                del self._tracks[track_id]
        for detection_index, detection in enumerate(detections):
            if detection_index in assigned_detections:
                continue
            track_id = self._next_id
            self._next_id += 1
            self._tracks[track_id] = _Track(
                track_id=track_id,
                x=detection.x,
                y=detection.y,
                vx=0.0,
                vy=0.0,
                radius=detection.radius,
                created_s=stamp_s,
                updated_s=stamp_s,
            )
        self.last_update_s = stamp_s
        return tuple(
            self._estimate(track, stamp_s, robot)
            for track in sorted(self._tracks.values(), key=lambda item: item.track_id)
            if track.hits >= self.minimum_confirmed_hits
        )

    def _estimate(self, track: _Track, stamp_s: float, robot: RobotState) -> TrackEstimate:
        confidence = _clamp(
            (track.hits / max(1.0, self.minimum_confirmed_hits + 2.0))
            * math.exp(-0.35 * track.miss_count)
        )
        motion_class = self._classify(track, robot)
        predictions: List[Prediction] = []
        step_count = int(round(self.prediction_horizon_s / self.prediction_step_s))
        for index in range(1, step_count + 1):
            horizon = index * self.prediction_step_s
            predictions.append(
                Prediction(
                    time_from_start_s=horizon,
                    x=track.x + track.vx * horizon,
                    y=track.y + track.vy * horizon,
                    vx=track.vx,
                    vy=track.vy,
                    position_variance=0.02 + 0.08 * horizon * horizon,
                    confidence=_clamp(confidence * math.exp(-self.confidence_decay_per_s * horizon)),
                )
            )
        return TrackEstimate(
            track_id=track.track_id,
            x=track.x,
            y=track.y,
            vx=track.vx,
            vy=track.vy,
            radius=track.radius,
            confidence=confidence,
            age_s=max(0.0, stamp_s - track.created_s),
            miss_count=track.miss_count,
            last_update_s=track.updated_s,
            motion_class=motion_class,
            predictions=tuple(predictions),
        )

    def _classify(self, track: _Track, robot: RobotState) -> str:
        speed = math.hypot(track.vx, track.vy)
        if speed <= self.stationary_speed_max_mps:
            return "STATIONARY"
        cosine, sine = math.cos(robot.yaw), math.sin(robot.yaw)
        relative_x = cosine * (track.x - robot.x) + sine * (track.y - robot.y)
        relative_y = -sine * (track.x - robot.x) + cosine * (track.y - robot.y)
        robot_vx = robot.linear_velocity * cosine
        robot_vy = robot.linear_velocity * sine
        relative_vx_world = track.vx - robot_vx
        relative_vy_world = track.vy - robot_vy
        relative_vx = cosine * relative_vx_world + sine * relative_vy_world
        relative_vy = -sine * relative_vx_world + cosine * relative_vy_world
        if relative_x > 0.0 and abs(relative_vy) >= self.crossing_lateral_speed_min_mps:
            crossing_time = -relative_y / relative_vy
            crossing_x = relative_x + relative_vx * crossing_time
            if 0.0 <= crossing_time <= self.prediction_horizon_s and crossing_x > -self.crossing_path_half_width_m:
                return "CROSSING"
        if relative_x > 0.0 and relative_vx < -self.dynamic_speed_min_mps:
            if abs(relative_y) <= max(self.crossing_path_half_width_m, track.radius * 2.0):
                return "HEAD_ON"
        if relative_x > 0.0 and abs(relative_y) <= self.crossing_path_half_width_m:
            if 0.0 <= relative_vx + robot.linear_velocity < max(robot.linear_velocity, 0.1):
                return "FOLLOWING"
        radial_rate = (
            relative_x * relative_vx + relative_y * relative_vy
        ) / max(1.0e-6, math.hypot(relative_x, relative_y))
        if radial_rate > self.dynamic_speed_min_mps:
            return "DEPARTING"
        return "UNKNOWN"
