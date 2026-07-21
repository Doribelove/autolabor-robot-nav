"""Pure helpers for relative-motion TTC and footprint-clearance evidence.

The runtime world model never receives scene labels or Gazebo truth.  The
oriented-box helper is intentionally generic so an external evaluator can use
truth poses without feeding them back into the policy path.
"""

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple


TTC_OBSERVED_CONFLICT = "OBSERVED_CONFLICT"
TTC_NO_CONFLICT = "NO_CONFLICT_IN_HORIZON"
TTC_TRACKER_INVALID = "TRACKER_INVALID"


@dataclass(frozen=True)
class RelativeTrack:
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    confidence: float
    motion_class: str = "UNKNOWN"


@dataclass(frozen=True)
class ClearanceEvidence:
    signed_clearance_m: float
    clipped_clearance_m: float
    raw_range_m: float
    ray_index: int
    ray_angle_rad: float
    footprint_boundary_range_m: float


def _finite(value) -> bool:
    return isinstance(value, (int, float)) and math.isfinite(float(value))


def relative_collision_ttc(
    track: RelativeTrack,
    *,
    robot_radius_m: float,
    horizon_s: float,
    minimum_confidence: float,
    minimum_relative_speed_mps: float = 0.05,
) -> Optional[float]:
    """Return first circle-envelope contact time for a relative track.

    ``x/y`` and ``vx/vy`` are all in the current robot frame and velocity is
    relative velocity.  UNKNOWN tracks are accepted when their kinematics are
    informative; only explicitly stationary/departing tracks are excluded.
    """

    values = (
        track.x, track.y, track.vx, track.vy, track.radius,
        track.confidence, robot_radius_m, horizon_s, minimum_confidence,
        minimum_relative_speed_mps,
    )
    if not all(_finite(value) for value in values):
        return None
    if horizon_s <= 0.0 or robot_radius_m <= 0.0 or track.radius < 0.0:
        return None
    if track.confidence < minimum_confidence:
        return None
    if track.motion_class in ("STATIONARY", "DEPARTING"):
        return None
    speed_squared = track.vx * track.vx + track.vy * track.vy
    if speed_squared < minimum_relative_speed_mps * minimum_relative_speed_mps:
        return None
    interaction_radius = robot_radius_m + track.radius
    c = track.x * track.x + track.y * track.y - interaction_radius * interaction_radius
    if c <= 0.0:
        return 0.0
    b = 2.0 * (track.x * track.vx + track.y * track.vy)
    if b >= 0.0:
        return None
    discriminant = b * b - 4.0 * speed_squared * c
    if discriminant < 0.0:
        return None
    root = (-b - math.sqrt(discriminant)) / (2.0 * speed_squared)
    return root if 0.0 <= root <= horizon_s else None


def earliest_relative_ttc(
    tracks: Iterable[RelativeTrack],
    *,
    robot_radius_m: float = 0.62,
    horizon_s: float = 5.0,
    minimum_confidence: float = 0.45,
) -> Optional[float]:
    values = [
        value for value in (
            relative_collision_ttc(
                track,
                robot_radius_m=robot_radius_m,
                horizon_s=horizon_s,
                minimum_confidence=minimum_confidence,
            )
            for track in tracks
        )
        if value is not None
    ]
    return min(values) if values else None


def classify_ttc_evidence(
    *, tracker_message_count: int, healthy_tracker_sample_count: int,
    finite_ttc_sample_count: int,
) -> str:
    if tracker_message_count <= 0 or healthy_tracker_sample_count <= 0:
        return TTC_TRACKER_INVALID
    if finite_ttc_sample_count > 0:
        return TTC_OBSERVED_CONFLICT
    return TTC_NO_CONFLICT


def rectangular_footprint_clearance(
    scan,
    *,
    half_length_m: float = 0.52,
    half_width_m: float = 0.35,
) -> ClearanceEvidence:
    """Return signed LaserScan-to-rectangle clearance without hiding intrusion.

    ``scan`` only needs the standard LaserScan angle/range attributes.  A
    negative result means the reported range endpoint lies inside the footprint
    rectangle; the clipped value preserves the legacy non-negative metric.
    """

    if half_length_m <= 0.0 or half_width_m <= 0.0:
        raise ValueError("footprint half extents must be positive")
    best = None
    for index, raw in enumerate(scan.ranges):
        if not _finite(raw):
            continue
        distance = float(raw)
        if distance < float(scan.range_min) or distance > float(scan.range_max):
            continue
        angle = float(scan.angle_min) + index * float(scan.angle_increment)
        cosine, sine = math.cos(angle), math.sin(angle)
        x_limit = half_length_m / abs(cosine) if abs(cosine) > 1.0e-12 else float("inf")
        y_limit = half_width_m / abs(sine) if abs(sine) > 1.0e-12 else float("inf")
        boundary = min(x_limit, y_limit)
        signed = distance - boundary
        candidate = ClearanceEvidence(
            signed_clearance_m=signed,
            clipped_clearance_m=max(0.0, signed),
            raw_range_m=distance,
            ray_index=index,
            ray_angle_rad=angle,
            footprint_boundary_range_m=boundary,
        )
        if best is None or candidate.signed_clearance_m < best.signed_clearance_m:
            best = candidate
    if best is not None:
        return best
    maximum = float(scan.range_max)
    return ClearanceEvidence(maximum, maximum, maximum, -1, 0.0, 0.0)


def _rectangle_vertices(x, y, yaw, half_length, half_width):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    points = []
    for local_x, local_y in (
        (-half_length, -half_width), (half_length, -half_width),
        (half_length, half_width), (-half_length, half_width),
    ):
        points.append((
            x + cosine * local_x - sine * local_y,
            y + sine * local_x + cosine * local_y,
        ))
    return tuple(points)


def _axes(vertices):
    result = []
    for first, second in zip(vertices, vertices[1:] + vertices[:1]):
        dx, dy = second[0] - first[0], second[1] - first[1]
        length = math.hypot(dx, dy)
        result.append((-dy / length, dx / length))
    return result[:2]


def _overlap(first, second):
    for axis in _axes(first) + _axes(second):
        first_projection = [x * axis[0] + y * axis[1] for x, y in first]
        second_projection = [x * axis[0] + y * axis[1] for x, y in second]
        if max(first_projection) < min(second_projection) or max(second_projection) < min(first_projection):
            return False
    return True


def _point_segment_distance(point, first, second):
    dx, dy = second[0] - first[0], second[1] - first[1]
    denominator = dx * dx + dy * dy
    if denominator <= 1.0e-18:
        return math.hypot(point[0] - first[0], point[1] - first[1])
    t = max(0.0, min(1.0, (
        (point[0] - first[0]) * dx + (point[1] - first[1]) * dy
    ) / denominator))
    return math.hypot(point[0] - (first[0] + t * dx), point[1] - (first[1] + t * dy))


def oriented_box_clearance(
    first_pose: Tuple[float, float, float],
    first_size: Tuple[float, float],
    second_pose: Tuple[float, float, float],
    second_size: Tuple[float, float],
) -> float:
    """Euclidean clearance between two oriented 2-D rectangles (zero on overlap)."""

    if min(first_size + second_size) <= 0.0:
        raise ValueError("box dimensions must be positive")
    first = _rectangle_vertices(
        first_pose[0], first_pose[1], first_pose[2], first_size[0] / 2.0, first_size[1] / 2.0
    )
    second = _rectangle_vertices(
        second_pose[0], second_pose[1], second_pose[2], second_size[0] / 2.0, second_size[1] / 2.0
    )
    if _overlap(first, second):
        return 0.0
    distances = []
    for point in first:
        distances.extend(
            _point_segment_distance(point, edge_start, edge_end)
            for edge_start, edge_end in zip(second, second[1:] + second[:1])
        )
    for point in second:
        distances.extend(
            _point_segment_distance(point, edge_start, edge_end)
            for edge_start, edge_end in zip(first, first[1:] + first[:1])
        )
    return min(distances)
