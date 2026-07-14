"""Deterministic observation construction and fixed-length history windows."""

import math
from dataclasses import dataclass
from typing import Dict, List, Mapping, Optional, Sequence, Tuple

from .timing import MonotonicRosTime, SynchronizationError, synchronize_stamps


class StateError(ValueError):
    """Raised when an observation cannot produce a trustworthy state."""


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise StateError("{} must be numeric".format(name))
    number = float(value)
    if not math.isfinite(number):
        raise StateError("{} must be finite".format(name))
    return number


def lidar_sector_quantiles(
    ranges: Sequence[object],
    range_min: float,
    range_max: float,
    sector_count: int = 36,
    quantile: float = 0.1,
) -> Tuple[float, ...]:
    """Return robust low quantiles for equal angular sectors.

    Invalid rays are ignored; a sector with no valid ray is represented by
    ``range_max``.  Each input ray belongs to exactly one sector.
    """

    minimum = _finite(range_min, "range_min")
    maximum = _finite(range_max, "range_max")
    if minimum < 0.0 or maximum <= minimum:
        raise StateError("laser range bounds are invalid")
    if isinstance(sector_count, bool) or not isinstance(sector_count, int) or sector_count <= 0:
        raise StateError("sector_count must be a positive integer")
    q = _finite(quantile, "quantile")
    if q < 0.0 or q > 1.0:
        raise StateError("quantile must be within [0, 1]")
    if len(ranges) < sector_count:
        raise StateError("ranges must contain at least one ray per sector")
    sectors: List[List[float]] = [[] for _ in range(sector_count)]
    count = len(ranges)
    for index, raw in enumerate(ranges):
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            continue
        ray = float(raw)
        if not math.isfinite(ray) or ray < minimum or ray > maximum:
            continue
        sector_index = min(sector_count - 1, (index * sector_count) // count)
        sectors[sector_index].append(ray)
    result = []
    for values in sectors:
        if not values:
            result.append(maximum)
            continue
        ordered = sorted(values)
        position = q * (len(ordered) - 1)
        lower = int(math.floor(position))
        upper = int(math.ceil(position))
        fraction = position - lower
        result.append(ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction)
    return tuple(result)


@dataclass(frozen=True)
class DirectionalScanCoverage:
    """Whether the scan contains a small angular band around four body directions."""

    front: bool
    left: bool
    right: bool
    rear: bool


@dataclass(frozen=True)
class ScanAngularMetadata:
    """Validated LaserScan geometry retained alongside the legacy V1 vector."""

    stamp: float
    frame_id: str
    angle_min: float
    angle_max: float
    angle_increment: float
    ray_count: int

    def __post_init__(self) -> None:
        stamp = _finite(self.stamp, "scan stamp")
        angle_min = _finite(self.angle_min, "angle_min")
        angle_max = _finite(self.angle_max, "angle_max")
        increment = _finite(self.angle_increment, "angle_increment")
        if stamp < 0.0:
            raise StateError("scan stamp must be non-negative")
        if not isinstance(self.frame_id, str) or not self.frame_id.strip():
            raise StateError("scan frame_id must be a non-empty string")
        if isinstance(self.ray_count, bool) or not isinstance(self.ray_count, int):
            raise StateError("scan ray_count must be an integer")
        if self.ray_count <= 0:
            raise StateError("scan ray_count must be positive")
        if angle_max <= angle_min or increment <= 0.0:
            raise StateError("LaserScan angles must be finite and increasing")
        expected_last = angle_min + increment * max(0, self.ray_count - 1)
        tolerance = max(1e-6, 1.5 * increment)
        if abs(expected_last - angle_max) > tolerance:
            raise StateError(
                "LaserScan angle bounds/increment/ray_count are inconsistent"
            )
        if angle_max - angle_min > 2.0 * math.pi + 2.0 * increment:
            raise StateError("LaserScan angular span exceeds one revolution")
        object.__setattr__(self, "stamp", stamp)
        object.__setattr__(self, "frame_id", self.frame_id.strip())
        object.__setattr__(self, "angle_min", angle_min)
        object.__setattr__(self, "angle_max", angle_max)
        object.__setattr__(self, "angle_increment", increment)

    @property
    def angular_span(self) -> float:
        return self.angle_max - self.angle_min

    @property
    def full_circle(self) -> bool:
        tolerance = max(1e-6, 2.0 * self.angle_increment)
        return self.angular_span + self.angle_increment >= 2.0 * math.pi - tolerance

    def covers_direction(self, center_rad: float, half_width_rad: float = 0.0) -> bool:
        """Return true only when the requested body-frame angular band is observed."""

        center = _finite(center_rad, "direction center")
        half_width = _finite(half_width_rad, "direction half width")
        if half_width < 0.0 or half_width > math.pi:
            raise StateError("direction half width must be within [0, pi]")
        if self.full_circle:
            return True
        tolerance = max(1e-6, self.angle_increment)
        for turns in range(-2, 3):
            adjusted = center + turns * 2.0 * math.pi
            if (
                adjusted - half_width >= self.angle_min - tolerance
                and adjusted + half_width <= self.angle_max + tolerance
            ):
                return True
        return False

    def sector_center_angles(self, sector_count: int) -> Tuple[float, ...]:
        if (
            isinstance(sector_count, bool)
            or not isinstance(sector_count, int)
            or sector_count <= 0
        ):
            raise StateError("sector_count must be a positive integer")
        last_index = max(0, self.ray_count - 1)
        return tuple(
            self.angle_min
            + ((index + 0.5) / float(sector_count)) * last_index * self.angle_increment
            for index in range(sector_count)
        )

    def sector_indices(
        self, sector_count: int, center_rad: float, half_width_rad: float
    ) -> Tuple[int, ...]:
        """Map a body-frame angular cone to legacy equal-index lidar sectors."""

        center = _finite(center_rad, "sector direction center")
        half_width = _finite(half_width_rad, "sector direction half width")
        if half_width < 0.0 or half_width > math.pi:
            raise StateError("sector direction half width must be within [0, pi]")
        centers = self.sector_center_angles(sector_count)
        selected = tuple(
            index
            for index, angle in enumerate(centers)
            if abs(math.atan2(math.sin(angle - center), math.cos(angle - center)))
            <= half_width + 1e-12
        )
        if selected or not self.covers_direction(center, 0.0):
            return selected
        nearest = min(
            range(sector_count),
            key=lambda index: abs(
                math.atan2(
                    math.sin(centers[index] - center),
                    math.cos(centers[index] - center),
                )
            ),
        )
        return (nearest,)

    def directional_coverage(
        self, probe_half_width_rad: float = math.radians(5.0)
    ) -> DirectionalScanCoverage:
        return DirectionalScanCoverage(
            front=self.covers_direction(0.0, probe_half_width_rad),
            left=self.covers_direction(math.pi / 2.0, probe_half_width_rad),
            right=self.covers_direction(-math.pi / 2.0, probe_half_width_rad),
            rear=self.covers_direction(math.pi, probe_half_width_rad),
        )


@dataclass(frozen=True)
class StateFrame:
    timestamp: float
    vector: Tuple[float, ...]
    valid: bool
    invalid_reasons: Tuple[str, ...]
    named_features: Mapping[str, float]
    scan_metadata: Optional[ScanAngularMetadata] = None
    directional_scan_coverage: Optional[DirectionalScanCoverage] = None


class StateBuilder:
    """Build one synchronized frame from lidar plus named semantic features."""

    def __init__(
        self,
        feature_order: Sequence[str],
        required_streams: Sequence[str],
        sector_count: int = 36,
        lidar_quantile: float = 0.1,
        max_sync_skew_s: float = 0.1,
    ) -> None:
        if len(set(feature_order)) != len(feature_order):
            raise StateError("feature_order contains duplicates")
        self.feature_order = tuple(feature_order)
        self.required_streams = tuple(required_streams)
        self.sector_count = sector_count
        self.lidar_quantile = lidar_quantile
        self.max_sync_skew_s = max_sync_skew_s
        self._clock = MonotonicRosTime()

    def build(
        self,
        stamps: Mapping[str, object],
        ranges: Sequence[object],
        range_min: float,
        range_max: float,
        features: Mapping[str, object],
        validity: Optional[Mapping[str, bool]] = None,
        scan_metadata: Optional[ScanAngularMetadata] = None,
    ) -> StateFrame:
        synced = synchronize_stamps(stamps, self.required_streams, self.max_sync_skew_s)
        self._clock.observe("state", synced.observation_time)
        missing = [name for name in self.feature_order if name not in features]
        if missing:
            raise StateError("missing features: {}".format(", ".join(missing)))
        named: Dict[str, float] = {
            name: _finite(features[name], name) for name in self.feature_order
        }
        lidar = lidar_sector_quantiles(
            ranges, range_min, range_max, self.sector_count, self.lidar_quantile
        )
        coverage = None
        if scan_metadata is not None:
            if not isinstance(scan_metadata, ScanAngularMetadata):
                raise StateError("scan_metadata must be ScanAngularMetadata")
            if scan_metadata.ray_count != len(ranges):
                raise StateError("LaserScan ray_count does not match ranges")
            if "scan" in stamps:
                scan_stamp = _finite(stamps["scan"], "scan timestamp")
                if abs(scan_metadata.stamp - scan_stamp) > self.max_sync_skew_s:
                    raise StateError("LaserScan metadata stamp is not synchronized")
            coverage = scan_metadata.directional_coverage()
        invalid = tuple(
            sorted(name for name, is_valid in (validity or {}).items() if is_valid is not True)
        )
        return StateFrame(
            timestamp=synced.observation_time,
            vector=lidar + tuple(named[name] for name in self.feature_order),
            valid=not invalid,
            invalid_reasons=invalid,
            named_features=named,
            scan_metadata=scan_metadata,
            directional_scan_coverage=coverage,
        )

    def reset_time_epoch(self) -> None:
        self._clock.reset()


class HistoryWindow:
    """A strictly ordered, fixed-``K`` state history."""

    def __init__(self, k: int = 4) -> None:
        if isinstance(k, bool) or not isinstance(k, int) or k <= 0:
            raise StateError("k must be a positive integer")
        self.k = k
        self._frames: List[StateFrame] = []
        self._dimension: Optional[int] = None

    @property
    def ready(self) -> bool:
        return len(self._frames) == self.k

    @property
    def frames(self) -> Tuple[StateFrame, ...]:
        return tuple(self._frames)

    def append(self, frame: StateFrame) -> None:
        if self._frames and frame.timestamp <= self._frames[-1].timestamp:
            raise StateError("history timestamps must be strictly increasing")
        if self._dimension is None:
            self._dimension = len(frame.vector)
        elif len(frame.vector) != self._dimension:
            raise StateError("state vector dimension changed")
        self._frames.append(frame)
        if len(self._frames) > self.k:
            del self._frames[0]

    def stacked(self, require_valid: bool = True) -> Tuple[float, ...]:
        if not self.ready:
            raise StateError("history window is not ready: {}/{}".format(len(self._frames), self.k))
        if require_valid and not all(frame.valid for frame in self._frames):
            raise StateError("history window contains invalid state")
        return tuple(value for frame in self._frames for value in frame.vector)

    def clear(self) -> None:
        self._frames = []
        self._dimension = None
