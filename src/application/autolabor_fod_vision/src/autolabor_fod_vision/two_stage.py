"""ROS-independent primitives for the detect-and-classify FOD backend.

The module deliberately contains no model or ROS imports.  It is used by the
live node and by deterministic unit tests for latest-frame delivery, robust
depth clustering, classification voting, and world-object re-identification.
"""

from collections import deque
from dataclasses import dataclass, field
import math
import threading
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


MATERIAL_CLASSES: Tuple[str, ...] = (
    "metal",
    "plastic",
    "paper",
    "glass",
    "kitchen_waste",
)

BBox = Tuple[float, float, float, float]


class LatestFrameSlot:
    """A blocking single-element slot where a new frame replaces the old one."""

    def __init__(self):
        self._condition = threading.Condition()
        self._item = None
        self._stopped = False
        self.received = 0
        self.overwritten = 0

    def put(self, item) -> bool:
        """Store *item* and return True when an unread item was overwritten."""
        with self._condition:
            if self._stopped:
                return False
            overwritten = self._item is not None
            self.received += 1
            if overwritten:
                self.overwritten += 1
            self._item = item
            self._condition.notify()
            return overwritten

    def take(self, timeout: Optional[float] = None):
        with self._condition:
            if self._item is None and not self._stopped:
                self._condition.wait(timeout=timeout)
            if self._item is None:
                return None
            item = self._item
            self._item = None
            return item

    def clear(self) -> None:
        with self._condition:
            self._item = None

    def stop(self) -> None:
        with self._condition:
            self._stopped = True
            self._item = None
            self._condition.notify_all()

    @property
    def pending(self) -> int:
        with self._condition:
            return int(self._item is not None)


def clip_bbox(bbox: BBox, width: int, height: int) -> BBox:
    x1, y1, x2, y2 = [float(value) for value in bbox]
    x1 = max(0.0, min(float(max(0, width - 1)), x1))
    y1 = max(0.0, min(float(max(0, height - 1)), y1))
    x2 = max(x1 + 1.0, min(float(width), x2))
    y2 = max(y1 + 1.0, min(float(height), y2))
    return x1, y1, x2, y2


def context_crop(
    image_bgr: np.ndarray, bbox: BBox, context_fraction: float = 0.20
) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
    """Return an in-memory crop expanded on every side by the box fraction."""
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("context_crop expects a HxWx3 image")
    if not 0.0 <= context_fraction <= 1.0:
        raise ValueError("context_fraction must be in [0, 1]")
    height, width = image_bgr.shape[:2]
    x1, y1, x2, y2 = clip_bbox(bbox, width, height)
    pad_x = (x2 - x1) * context_fraction
    pad_y = (y2 - y1) * context_fraction
    left = max(0, int(math.floor(x1 - pad_x)))
    top = max(0, int(math.floor(y1 - pad_y)))
    right = min(width, int(math.ceil(x2 + pad_x)))
    bottom = min(height, int(math.ceil(y2 + pad_y)))
    if right <= left or bottom <= top:
        raise ValueError("expanded crop is empty")
    return image_bgr[top:bottom, left:right], (left, top, right, bottom)


def appearance_histogram(crop_bgr: np.ndarray) -> np.ndarray:
    """Small normalized HSV histogram used only as a reclassification hint."""
    if crop_bgr.size == 0:
        return np.zeros(64, dtype=np.float32)
    hsv = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2HSV)
    histogram = cv2.calcHist([hsv], [0, 1], None, [8, 8], [0, 180, 0, 256])
    vector = histogram.astype(np.float32).reshape(-1)
    norm = float(np.linalg.norm(vector))
    if norm > 1e-12:
        vector /= norm
    return vector


def crop_sharpness(crop_bgr: np.ndarray) -> float:
    """Return Laplacian variance used to skip visibly blurred vote samples."""
    if crop_bgr.ndim != 3 or crop_bgr.shape[2] != 3 or crop_bgr.size == 0:
        return 0.0
    gray = cv2.cvtColor(crop_bgr, cv2.COLOR_BGR2GRAY)
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def cosine_distance(left: Optional[np.ndarray], right: Optional[np.ndarray]) -> float:
    if left is None or right is None or left.size == 0 or right.size == 0:
        return 0.0
    denominator = float(np.linalg.norm(left) * np.linalg.norm(right))
    if denominator <= 1e-12:
        return 0.0
    similarity = float(np.dot(left, right) / denominator)
    return max(0.0, min(2.0, 1.0 - similarity))


def bbox_iou(left: BBox, right: BBox) -> float:
    lx1, ly1, lx2, ly2 = left
    rx1, ry1, rx2, ry2 = right
    intersection = max(0.0, min(lx2, rx2) - max(lx1, rx1)) * max(
        0.0, min(ly2, ry2) - max(ly1, ry1)
    )
    left_area = max(0.0, lx2 - lx1) * max(0.0, ly2 - ly1)
    right_area = max(0.0, rx2 - rx1) * max(0.0, ry2 - ry1)
    union = left_area + right_area - intersection
    return intersection / union if union > 1e-12 else 0.0


def _linear_assignment(cost: np.ndarray) -> List[Tuple[int, int]]:
    """Hungarian assignment for a finite rectangular cost matrix."""
    if cost.ndim != 2 or cost.size == 0:
        return []
    original_rows, original_columns = cost.shape
    transposed = original_rows > original_columns
    matrix = cost.T.copy() if transposed else cost.copy()
    rows, columns = matrix.shape
    u = np.zeros(rows + 1, dtype=np.float64)
    v = np.zeros(columns + 1, dtype=np.float64)
    p = np.zeros(columns + 1, dtype=np.int64)
    way = np.zeros(columns + 1, dtype=np.int64)
    for row in range(1, rows + 1):
        p[0] = row
        min_value = np.full(columns + 1, np.inf, dtype=np.float64)
        used = np.zeros(columns + 1, dtype=bool)
        column0 = 0
        while True:
            used[column0] = True
            row0 = p[column0]
            delta = np.inf
            column1 = 0
            for column in range(1, columns + 1):
                if used[column]:
                    continue
                current = matrix[row0 - 1, column - 1] - u[row0] - v[column]
                if current < min_value[column]:
                    min_value[column] = current
                    way[column] = column0
                if min_value[column] < delta:
                    delta = min_value[column]
                    column1 = column
            if not math.isfinite(float(delta)):
                break
            for column in range(columns + 1):
                if used[column]:
                    u[p[column]] += delta
                    v[column] -= delta
                else:
                    min_value[column] -= delta
            column0 = column1
            if p[column0] == 0:
                break
        while column0:
            column1 = way[column0]
            p[column0] = p[column1]
            column0 = column1
    assignments: List[Tuple[int, int]] = []
    for column in range(1, columns + 1):
        if p[column] == 0:
            continue
        row_index = int(p[column] - 1)
        column_index = int(column - 1)
        assignments.append(
            (column_index, row_index) if transposed else (row_index, column_index)
        )
    return [
        (row, column)
        for row, column in assignments
        if row < original_rows and column < original_columns
    ]


class ImageTrackFallback:
    """Fallback IDs for rare frames where BoT-SORT returns no box IDs."""

    def __init__(self, buffer_frames: int = 45, minimum_iou: float = 0.15):
        self.buffer_frames = max(1, int(buffer_frames))
        self.minimum_iou = float(minimum_iou)
        self._next_id = 1
        self._tracks: Dict[int, Tuple[BBox, int]] = {}

    def update(self, boxes: Sequence[BBox], frame_index: int) -> List[int]:
        stale = [
            track_id
            for track_id, (_, seen_frame) in self._tracks.items()
            if frame_index - seen_frame > self.buffer_frames
        ]
        for track_id in stale:
            self._tracks.pop(track_id, None)
        track_ids = sorted(self._tracks)
        assigned = [0] * len(boxes)
        if track_ids and boxes:
            costs = np.full((len(boxes), len(track_ids)), 1e6, dtype=np.float64)
            for box_index, box in enumerate(boxes):
                for track_index, track_id in enumerate(track_ids):
                    overlap = bbox_iou(box, self._tracks[track_id][0])
                    if overlap >= self.minimum_iou:
                        costs[box_index, track_index] = 1.0 - overlap
            for box_index, track_index in _linear_assignment(costs):
                if costs[box_index, track_index] >= 1e5:
                    continue
                assigned[box_index] = track_ids[track_index]
        for index, box in enumerate(boxes):
            if assigned[index] == 0:
                assigned[index] = self._next_id
                self._next_id += 1
            self._tracks[assigned[index]] = (box, frame_index)
        return assigned

    def reset(self) -> None:
        self._tracks.clear()
        self._next_id = 1


@dataclass(frozen=True)
class DepthClusterEstimate:
    valid: bool
    depth_m: float = float("nan")
    mad_m: float = float("nan")
    sample_count: int = 0
    valid_fraction: float = 0.0
    center_u: float = float("nan")
    center_v: float = float("nan")
    camera_point: Tuple[float, float, float] = (
        float("nan"),
        float("nan"),
        float("nan"),
    )
    separated_from_background: bool = False
    reason: str = ""


def _invalid_depth(reason: str, count: int = 0, fraction: float = 0.0):
    return DepthClusterEstimate(
        valid=False, sample_count=count, valid_fraction=fraction, reason=reason
    )


def _depth_cluster_score(
    component_fraction: float,
    compactness: float,
    center_distance: float,
    center_coverage: float,
    mad_m: float,
    median_depth_m: float,
) -> float:
    """Rank a current-frame depth component by monotonic geometric support."""
    area_score = min(1.0, max(0.0, float(component_fraction)) / 0.70)
    compactness_score = min(1.0, max(0.0, float(compactness)))
    normalized_center_distance = max(0.0, float(center_distance)) / 0.30
    centroid_score = math.exp(-(normalized_center_distance**2))
    center_coverage_score = min(1.0, max(0.0, float(center_coverage)))
    # A surrounding background ring can have a centroid at the box center even
    # though none of its pixels cover the detected object.  Give actual support
    # inside the center window substantially more weight than centroid position.
    center_score = 0.20 * centroid_score + 0.80 * center_coverage_score
    dispersion_scale = max(0.015, 0.015 * max(0.0, float(median_depth_m)))
    dispersion_score = math.exp(-max(0.0, float(mad_m)) / dispersion_scale)
    return (
        0.30 * area_score
        + 0.25 * compactness_score
        + 0.35 * center_score
        + 0.10 * dispersion_score
    )


def estimate_clustered_depth(
    depth_m: np.ndarray,
    bbox: BBox,
    camera_matrix: Sequence[float],
    minimum_depth_m: float = 0.30,
    maximum_depth_m: float = 15.0,
    inset_fraction: float = 0.10,
    minimum_samples: int = 24,
    minimum_valid_fraction: float = 0.12,
    aggregation: str = "median",
) -> DepthClusterEstimate:
    """Choose a spatially connected 3-D candidate, never simply the nearest layer.

    Candidate components are built from locally coherent registered-depth bands.
    Ranking uses monotonic pixel support, center coverage, compactness and
    dispersion.  Absolute range is intentionally absent from the score.
    """
    if depth_m.ndim != 2:
        return _invalid_depth("depth image is not single-channel")
    if len(camera_matrix) != 9:
        return _invalid_depth("camera matrix must contain 9 values")
    fx, fy = float(camera_matrix[0]), float(camera_matrix[4])
    cx, cy = float(camera_matrix[2]), float(camera_matrix[5])
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)) or fx <= 0 or fy <= 0:
        return _invalid_depth("camera intrinsics are invalid")
    if aggregation not in ("median", "mean"):
        return _invalid_depth("unsupported depth aggregation")
    height, width = depth_m.shape
    x1, y1, x2, y2 = clip_bbox(bbox, width, height)
    inset_x = (x2 - x1) * max(0.0, min(0.45, inset_fraction))
    inset_y = (y2 - y1) * max(0.0, min(0.45, inset_fraction))
    left = max(0, int(math.floor(x1 + inset_x)))
    top = max(0, int(math.floor(y1 + inset_y)))
    right = min(width, int(math.ceil(x2 - inset_x)))
    bottom = min(height, int(math.ceil(y2 - inset_y)))
    if right - left < 3 or bottom - top < 3:
        return _invalid_depth("inset detection box is too small")
    region = np.asarray(depth_m[top:bottom, left:right], dtype=np.float32)
    area = int(region.size)
    stride = max(1, int(math.ceil(math.sqrt(max(1.0, area / 16000.0)))))
    sampled = region[::stride, ::stride]
    valid = (
        np.isfinite(sampled)
        & (sampled >= float(minimum_depth_m))
        & (sampled <= float(maximum_depth_m))
    )
    valid_count = int(np.count_nonzero(valid))
    valid_fraction = valid_count / float(max(1, sampled.size))
    if valid_count < int(minimum_samples):
        return _invalid_depth("too few valid depth samples", valid_count, valid_fraction)
    if valid_fraction < float(minimum_valid_fraction):
        return _invalid_depth("valid depth fraction is too low", valid_count, valid_fraction)

    values = sampled[valid].astype(np.float64)
    low, high = np.percentile(values, [1.0, 99.0])
    robust = values[(values >= low) & (values <= high)]
    if robust.size < minimum_samples:
        return _invalid_depth("depth values are dominated by outliers", valid_count, valid_fraction)
    reference = float(np.median(robust))
    band_width = max(0.035, min(0.14, 0.025 * reference))
    span = max(band_width, float(high - low))
    bin_count = max(1, min(64, int(math.ceil(span / band_width))))
    histogram, edges = np.histogram(robust, bins=bin_count, range=(low, high + 1e-6))
    peak_indices = list(np.argsort(histogram)[::-1][: min(8, bin_count)])
    candidate_centers = [
        0.5 * (float(edges[index]) + float(edges[index + 1]))
        for index in peak_indices
        if histogram[index] >= max(3, minimum_samples // 3)
    ]
    candidate_centers.extend(float(value) for value in np.percentile(robust, [20, 40, 60, 80]))

    region_height, region_width = sampled.shape
    target_center = np.asarray(
        [0.5 * (region_width - 1), 0.5 * (region_height - 1)], dtype=np.float64
    )
    center_left = max(0, int(math.floor(0.30 * region_width)))
    center_top = max(0, int(math.floor(0.30 * region_height)))
    center_right = min(region_width, int(math.ceil(0.70 * region_width)))
    center_bottom = min(region_height, int(math.ceil(0.70 * region_height)))
    center_valid_count = int(
        np.count_nonzero(valid[center_top:center_bottom, center_left:center_right])
    )
    diagonal = max(1.0, math.hypot(region_width, region_height))
    best = None
    seen_centers: List[float] = []
    kernel = np.ones((3, 3), np.uint8)
    for center in candidate_centers:
        if any(abs(center - previous) < band_width * 0.35 for previous in seen_centers):
            continue
        seen_centers.append(center)
        tolerance = max(band_width * 1.15, 0.02 * center)
        layer = valid & (np.abs(sampled - center) <= tolerance)
        layer = cv2.morphologyEx(layer.astype(np.uint8), cv2.MORPH_CLOSE, kernel)
        component_count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            layer, connectivity=8
        )
        for component in range(1, component_count):
            component_size = int(stats[component, cv2.CC_STAT_AREA])
            if component_size < minimum_samples:
                continue
            component_mask = (labels == component) & valid
            component_values = sampled[component_mask].astype(np.float64)
            if component_values.size < minimum_samples:
                continue
            median = float(np.median(component_values))
            deviations = np.abs(component_values - median)
            mad = float(np.median(deviations))
            keep_threshold = max(0.025, 3.5 * mad)
            kept_mask = component_mask & (np.abs(sampled - median) <= keep_threshold)
            rows, columns = np.nonzero(kept_mask)
            kept_values = sampled[kept_mask].astype(np.float64)
            if kept_values.size < minimum_samples:
                continue
            centroid = np.asarray([float(np.mean(columns)), float(np.mean(rows))])
            center_distance = float(np.linalg.norm(centroid - target_center)) / diagonal
            component_fraction = float(kept_values.size) / float(valid_count)
            component_width = max(1, int(stats[component, cv2.CC_STAT_WIDTH]))
            component_height = max(1, int(stats[component, cv2.CC_STAT_HEIGHT]))
            compactness = min(
                1.0,
                float(kept_values.size) / float(component_width * component_height),
            )
            center_coverage = (
                float(
                    np.count_nonzero(
                        kept_mask[
                            center_top:center_bottom, center_left:center_right
                        ]
                    )
                )
                / float(center_valid_count)
                if center_valid_count > 0
                else 0.0
            )
            touches = sum(
                (
                    int(stats[component, cv2.CC_STAT_LEFT]) <= 0,
                    int(stats[component, cv2.CC_STAT_TOP]) <= 0,
                    int(stats[component, cv2.CC_STAT_LEFT]) + component_width
                    >= region_width,
                    int(stats[component, cv2.CC_STAT_TOP]) + component_height
                    >= region_height,
                )
            )
            other_values = sampled[valid & ~kept_mask].astype(np.float64)
            separation_threshold = max(0.035, 0.02 * median)
            separated = bool(
                other_values.size >= minimum_samples
                and abs(float(np.median(other_values)) - median) >= separation_threshold
            )
            plane_like = bool(
                component_fraction >= 0.68
                and touches >= 3
                and not separated
                and mad <= max(0.012, 0.006 * median)
            )
            score = _depth_cluster_score(
                component_fraction,
                compactness,
                center_distance,
                center_coverage,
                mad,
                median,
            ) - (0.40 if plane_like else 0.0)
            candidate = (
                score,
                plane_like,
                separated,
                rows,
                columns,
                kept_values,
                mad,
            )
            if best is None or score > best[0]:
                best = candidate
    if best is None:
        return _invalid_depth("no spatially coherent depth cluster", valid_count, valid_fraction)
    _, plane_like, separated, rows, columns, kept_values, mad = best
    if plane_like:
        return _invalid_depth(
            "candidate is indistinguishable from a locally flat surface",
            int(kept_values.size),
            valid_fraction,
        )
    aggregate = float(np.median(kept_values) if aggregation == "median" else np.mean(kept_values))
    global_u = left + columns.astype(np.float64) * stride
    global_v = top + rows.astype(np.float64) * stride
    x_values = (global_u - cx) * kept_values / fx
    y_values = (global_v - cy) * kept_values / fy
    if aggregation == "median":
        point = (
            float(np.median(x_values)),
            float(np.median(y_values)),
            aggregate,
        )
    else:
        point = (
            float(np.mean(x_values)),
            float(np.mean(y_values)),
            aggregate,
        )
    return DepthClusterEstimate(
        valid=True,
        depth_m=aggregate,
        mad_m=float(mad),
        sample_count=int(kept_values.size),
        valid_fraction=valid_fraction,
        center_u=float(np.median(global_u)),
        center_v=float(np.median(global_v)),
        camera_point=point,
        separated_from_background=bool(separated),
        reason="cluster selected by geometry",
    )


@dataclass
class ClassificationVote:
    probabilities: np.ndarray
    confidence: float
    frame_index: int


@dataclass(frozen=True)
class DepthHistorySample:
    depth_m: float
    frame_index: int
    timestamp: float
    separated_from_background: bool


@dataclass(frozen=True)
class WorldHistorySample:
    position: np.ndarray
    frame: str
    frame_index: int
    timestamp: float


@dataclass
class ObjectObservation:
    track_id: int
    bbox: BBox
    detect_confidence: float
    depth_valid: bool = False
    depth_m: float = float("nan")
    world_position: Optional[np.ndarray] = None
    world_frame: str = ""
    appearance: Optional[np.ndarray] = None
    classification_probabilities: Optional[np.ndarray] = None


@dataclass
class WorldObject:
    object_id: int
    current_track_id: int
    world_position: Optional[np.ndarray]
    world_frame: str
    bbox: BBox
    first_seen: float
    last_seen: float
    votes: Deque[ClassificationVote]
    stable_material: str = "unknown"
    classify_confidence: float = float("nan")
    state: str = "ACTIVE"
    depth_valid: bool = False
    depth_m: float = float("nan")
    last_classified_frame: int = -1000000
    appearance: Optional[np.ndarray] = None
    depth_samples: Deque[DepthHistorySample] = field(default_factory=deque)
    world_samples: Deque[WorldHistorySample] = field(default_factory=deque)
    depth_locked: bool = False
    stable_depth_m: float = float("nan")
    depth_lock_stamp: float = float("nan")
    depth_separated_from_background: bool = False
    last_depth_cluster_frame: int = -1000000
    depth_validation_failures: int = 0
    world_locked: bool = False
    world_lock_stamp: float = float("nan")

    def aggregate_probabilities(self) -> Optional[np.ndarray]:
        if not self.votes:
            return None
        weighted = np.zeros(len(MATERIAL_CLASSES), dtype=np.float64)
        total_weight = 0.0
        for vote in self.votes:
            weight = max(1e-6, float(vote.confidence))
            weighted += vote.probabilities.astype(np.float64) * weight
            total_weight += weight
        if total_weight <= 0.0:
            return None
        weighted /= total_weight
        normalizer = float(np.sum(weighted))
        if normalizer > 0.0:
            weighted /= normalizer
        return weighted.astype(np.float32)


class WorldObjectMap:
    """One-to-one world association with classification history on object_id."""

    VALID_STATES = {
        "ACTIVE",
        "LOST",
        "REIDENTIFIED",
        "CONFIRMED",
        "CLEANED",
        "EXPIRED",
    }

    def __init__(
        self,
        max_world_distance_m: float = 0.30,
        memory_timeout_sec: float = 30.0,
        vote_window: int = 5,
        minimum_stable_votes: int = 3,
        stable_confidence: float = 0.55,
        reclassify_interval_frames: int = 5,
        appearance_change_threshold: float = 0.35,
        depth_lock_samples: int = 5,
        depth_lock_min_inliers: int = 3,
        depth_outlier_mad_scale: float = 3.0,
        depth_outlier_min_m: float = 0.08,
        depth_validation_interval_frames: int = 12,
        depth_validation_max_abs_change_m: float = 0.15,
        depth_validation_max_relative_change: float = 0.10,
        depth_validation_failures_before_reacquire: int = 2,
        depth_bbox_area_change_ratio: float = 0.40,
        world_lock_samples: int = 3,
        world_outlier_mad_scale: float = 3.0,
        world_outlier_min_m: float = 0.08,
    ):
        self.max_world_distance_m = float(max_world_distance_m)
        self.memory_timeout_sec = float(memory_timeout_sec)
        self.vote_window = max(1, int(vote_window))
        self.minimum_stable_votes = max(1, int(minimum_stable_votes))
        self.stable_confidence = float(stable_confidence)
        self.reclassify_interval_frames = max(1, int(reclassify_interval_frames))
        self.appearance_change_threshold = float(appearance_change_threshold)
        self.depth_lock_samples = max(3, int(depth_lock_samples))
        self.depth_lock_min_inliers = max(
            2, min(self.depth_lock_samples, int(depth_lock_min_inliers))
        )
        self.depth_outlier_mad_scale = max(0.0, float(depth_outlier_mad_scale))
        self.depth_outlier_min_m = max(0.0, float(depth_outlier_min_m))
        self.depth_validation_interval_frames = max(
            1, int(depth_validation_interval_frames)
        )
        self.depth_validation_max_abs_change_m = max(
            0.0, float(depth_validation_max_abs_change_m)
        )
        self.depth_validation_max_relative_change = max(
            0.0, float(depth_validation_max_relative_change)
        )
        self.depth_validation_failures_before_reacquire = max(
            1, int(depth_validation_failures_before_reacquire)
        )
        self.depth_bbox_area_change_ratio = max(
            0.0, float(depth_bbox_area_change_ratio)
        )
        self.world_lock_samples = max(1, int(world_lock_samples))
        self.world_outlier_mad_scale = max(0.0, float(world_outlier_mad_scale))
        self.world_outlier_min_m = max(0.0, float(world_outlier_min_m))
        self.objects: Dict[int, WorldObject] = {}
        self._next_object_id = 1

    @staticmethod
    def _area(box: BBox) -> float:
        return max(1.0, (box[2] - box[0]) * (box[3] - box[1]))

    def target_for_track(
        self, track_id: int, timestamp: float, maximum_age_sec: float = 2.0
    ) -> Optional[WorldObject]:
        """Return the freshest live object currently owned by *track_id*."""
        candidates = [
            target
            for target in self.objects.values()
            if target.current_track_id == int(track_id)
            and target.state not in ("CLEANED", "EXPIRED")
            and 0.0 <= float(timestamp) - target.last_seen <= maximum_age_sec
        ]
        return max(candidates, key=lambda target: target.last_seen) if candidates else None

    def should_sample_depth(
        self,
        target: WorldObject,
        bbox: BBox,
        appearance: Optional[np.ndarray],
        frame_index: int,
    ) -> bool:
        """Request full clustering only while acquiring or periodically validating."""
        if not target.depth_locked:
            return True
        if (
            int(frame_index) - target.last_depth_cluster_frame
            >= self.depth_validation_interval_frames
        ):
            return True
        old_area = self._area(target.bbox)
        new_area = self._area(bbox)
        area_change = abs(new_area - old_area) / max(old_area, new_area)
        if area_change >= self.depth_bbox_area_change_ratio:
            return True
        if (
            appearance is not None
            and target.appearance is not None
            and cosine_distance(appearance, target.appearance)
            >= self.appearance_change_threshold
        ):
            return True
        return False

    def clear_depth_lock(self, target: WorldObject, clear_world: bool = False) -> None:
        target.depth_samples.clear()
        target.depth_locked = False
        target.stable_depth_m = float("nan")
        target.depth_lock_stamp = float("nan")
        target.depth_separated_from_background = False
        target.depth_validation_failures = 0
        if clear_world:
            target.world_samples.clear()
            target.world_locked = False
            target.world_position = None
            target.world_frame = ""
            target.world_lock_stamp = float("nan")

    def _recompute_depth_lock(self, target: WorldObject) -> bool:
        if len(target.depth_samples) < self.depth_lock_samples:
            return False
        samples = list(target.depth_samples)
        values = np.asarray([sample.depth_m for sample in samples], dtype=np.float64)
        median = float(np.median(values))
        deviations = np.abs(values - median)
        mad = float(np.median(deviations))
        threshold = max(
            self.depth_outlier_min_m,
            self.depth_outlier_mad_scale * mad,
        )
        # With only five samples, a large outlier can also inflate MAD.  Cap
        # the inlier band at the same absolute/relative change that would
        # invalidate a locked depth during a later validation frame.
        threshold = min(
            threshold,
            max(
                self.depth_validation_max_abs_change_m,
                self.depth_validation_max_relative_change * abs(median),
            ),
        )
        inlier_indices = np.flatnonzero(deviations <= threshold)
        if len(inlier_indices) < self.depth_lock_min_inliers:
            return False
        was_locked = target.depth_locked
        target.stable_depth_m = float(np.mean(values[inlier_indices]))
        target.depth_lock_stamp = max(
            samples[index].timestamp for index in inlier_indices
        )
        target.depth_separated_from_background = all(
            samples[index].separated_from_background for index in inlier_indices
        )
        target.depth_locked = True
        return not was_locked

    def _record_world_sample(
        self,
        target: WorldObject,
        world_position: np.ndarray,
        world_frame: str,
        timestamp: float,
        frame_index: int,
    ) -> bool:
        if target.world_locked:
            return False
        position = np.asarray(world_position, dtype=np.float64).reshape(-1)
        if position.shape != (3,) or not np.all(np.isfinite(position)) or not world_frame:
            return False
        if target.world_samples and target.world_samples[-1].frame != world_frame:
            target.world_samples.clear()
            target.world_locked = False
        if (
            target.world_samples
            and target.world_samples[-1].frame_index == int(frame_index)
        ):
            return False
        target.world_samples.append(
            WorldHistorySample(
                position=position.copy(),
                frame=str(world_frame),
                frame_index=int(frame_index),
                timestamp=float(timestamp),
            )
        )
        if len(target.world_samples) < self.world_lock_samples:
            return False
        samples = list(target.world_samples)
        positions = np.stack([sample.position for sample in samples], axis=0)
        center = np.median(positions, axis=0)
        distances = np.linalg.norm(positions - center, axis=1)
        distance_median = float(np.median(distances))
        distance_mad = float(np.median(np.abs(distances - distance_median)))
        threshold = max(
            self.world_outlier_min_m,
            self.world_outlier_mad_scale * distance_mad,
        )
        inliers = np.flatnonzero(distances <= threshold)
        minimum_inliers = max(1, min(self.world_lock_samples, 2))
        if len(inliers) < minimum_inliers:
            return False
        was_locked = target.world_locked
        target.world_position = np.mean(positions[inliers], axis=0)
        target.world_frame = str(world_frame)
        target.world_lock_stamp = max(samples[index].timestamp for index in inliers)
        target.world_locked = True
        return not was_locked

    def record_depth_observation(
        self,
        target: WorldObject,
        estimate: DepthClusterEstimate,
        timestamp: float,
        frame_index: int,
        world_position: Optional[np.ndarray] = None,
        world_frame: str = "",
    ) -> str:
        """Update the five-frame robust depth lock and three-frame world lock."""
        if not estimate.valid or not math.isfinite(estimate.depth_m):
            raise ValueError("record_depth_observation requires a valid estimate")
        target.last_depth_cluster_frame = int(frame_index)
        target.depth_validation_failures = 0
        event = "VALIDATED" if target.depth_locked else "ACQUIRING"
        if target.depth_locked:
            tolerance = max(
                self.depth_validation_max_abs_change_m,
                self.depth_validation_max_relative_change * target.stable_depth_m,
            )
            if abs(float(estimate.depth_m) - target.stable_depth_m) > tolerance:
                self.clear_depth_lock(target, clear_world=True)
                event = "REACQUIRING"
        if (
            not target.depth_samples
            or target.depth_samples[-1].frame_index != int(frame_index)
        ):
            target.depth_samples.append(
                DepthHistorySample(
                    depth_m=float(estimate.depth_m),
                    frame_index=int(frame_index),
                    timestamp=float(timestamp),
                    separated_from_background=bool(
                        estimate.separated_from_background
                    ),
                )
            )
        if self._recompute_depth_lock(target):
            event = "LOCKED"
        if world_position is not None:
            if self._record_world_sample(
                target,
                world_position,
                world_frame,
                timestamp,
                frame_index,
            ):
                event = "WORLD_LOCKED" if event == "VALIDATED" else event
        return event

    def note_depth_failure(self, target: WorldObject, frame_index: int) -> bool:
        """Return True when repeated failed validation forces reacquisition."""
        target.last_depth_cluster_frame = int(frame_index)
        if not target.depth_locked:
            return False
        target.depth_validation_failures += 1
        if (
            target.depth_validation_failures
            < self.depth_validation_failures_before_reacquire
        ):
            return False
        self.clear_depth_lock(target, clear_world=True)
        return True

    def _association_cost(
        self, observation: ObjectObservation, target: WorldObject, timestamp: float
    ) -> float:
        age = max(0.0, timestamp - target.last_seen)
        if age > self.memory_timeout_sec or target.state in ("CLEANED", "EXPIRED"):
            return 1e6
        area_ratio = self._area(observation.bbox) / self._area(target.bbox)
        size_cost = abs(math.log(max(1e-6, area_ratio)))
        if size_cost > 1.6:
            return 1e6
        appearance_cost = (
            cosine_distance(observation.appearance, target.appearance)
            if observation.appearance is not None and target.appearance is not None
            else 0.0
        )
        classification_cost = 0.0
        target_probabilities = target.aggregate_probabilities()
        if (
            observation.classification_probabilities is not None
            and target_probabilities is not None
        ):
            observation_probabilities = np.asarray(
                observation.classification_probabilities, dtype=np.float64
            ).reshape(-1)
            if (
                observation_probabilities.shape != (len(MATERIAL_CLASSES),)
                or not np.all(np.isfinite(observation_probabilities))
                or np.any(observation_probabilities < 0.0)
                or float(np.sum(observation_probabilities)) <= 0.0
            ):
                return 1e6
            observation_probabilities /= float(
                np.sum(observation_probabilities)
            )
            classification_cost = 0.5 * float(
                np.sum(
                    np.abs(
                        observation_probabilities
                        - target_probabilities.astype(np.float64)
                    )
                )
            )
            if classification_cost > 0.65:
                return 1e6
        if (
            observation.world_position is not None
            and target.world_position is not None
            and observation.world_frame
            and observation.world_frame == target.world_frame
        ):
            distance = float(
                np.linalg.norm(observation.world_position - target.world_position)
            )
            if not math.isfinite(distance) or distance > self.max_world_distance_m:
                return 1e6
            return (
                distance / max(1e-6, self.max_world_distance_m)
                + 0.16 * size_cost
                + 0.08 * min(1.0, age / max(1.0, self.memory_timeout_sec))
                + 0.10 * min(1.0, appearance_cost)
                + 0.12 * classification_cost
            )
        if observation.track_id == target.current_track_id and age <= 2.0:
            overlap = bbox_iou(observation.bbox, target.bbox)
            if overlap < 0.02 and size_cost > 0.8:
                return 1e6
            return (
                0.55
                + 0.25 * (1.0 - overlap)
                + 0.12 * size_cost
                + 0.08 * min(1.0, appearance_cost)
                + 0.10 * classification_cost
            )
        # A very short, visually unambiguous track break can survive a missing
        # source-stamped transform.  This is intentionally strict and never
        # makes the current observation's depth/world position valid.
        if (
            age <= 0.75
            and observation.appearance is not None
            and target.appearance is not None
        ):
            overlap = bbox_iou(observation.bbox, target.bbox)
            if appearance_cost <= 0.12 and overlap >= 0.15 and size_cost <= 0.45:
                return (
                    0.82
                    + 0.25 * (1.0 - overlap)
                    + 0.20 * appearance_cost
                    + 0.10 * classification_cost
                )
        return 1e6

    def associate(
        self, observations: Sequence[ObjectObservation], timestamp: float
    ) -> List[WorldObject]:
        timestamp = float(timestamp)
        for target in self.objects.values():
            age = max(0.0, timestamp - target.last_seen)
            if target.state == "CLEANED":
                continue
            target.state = "EXPIRED" if age > self.memory_timeout_sec else "LOST"
        candidates = [
            target
            for target in self.objects.values()
            if target.state not in ("CLEANED", "EXPIRED")
        ]
        assignments: Dict[int, WorldObject] = {}
        if observations and candidates:
            costs = np.full(
                (len(observations), len(candidates)), 1e6, dtype=np.float64
            )
            for observation_index, observation in enumerate(observations):
                for target_index, target in enumerate(candidates):
                    costs[observation_index, target_index] = self._association_cost(
                        observation, target, timestamp
                    )
            for observation_index, target_index in _linear_assignment(costs):
                if costs[observation_index, target_index] < 1e5:
                    assignments[observation_index] = candidates[target_index]

        output: List[WorldObject] = []
        for index, observation in enumerate(observations):
            target = assignments.get(index)
            if target is None:
                target = WorldObject(
                    object_id=self._next_object_id,
                    current_track_id=int(observation.track_id),
                    world_position=(
                        None
                        if observation.world_position is None
                        else observation.world_position.astype(np.float64).copy()
                    ),
                    world_frame=str(observation.world_frame),
                    bbox=observation.bbox,
                    first_seen=timestamp,
                    last_seen=timestamp,
                    votes=deque(maxlen=self.vote_window),
                    depth_samples=deque(maxlen=self.depth_lock_samples),
                    world_samples=deque(maxlen=self.world_lock_samples),
                    depth_valid=bool(observation.depth_valid),
                    depth_m=float(observation.depth_m),
                )
                self.objects[target.object_id] = target
                self._next_object_id += 1
            else:
                reidentified = target.current_track_id != int(observation.track_id)
                target.current_track_id = int(observation.track_id)
                target.bbox = observation.bbox
                target.last_seen = timestamp
                target.depth_valid = bool(observation.depth_valid)
                target.depth_m = float(observation.depth_m)
                if observation.world_position is not None and not target.world_locked:
                    if (
                        target.world_position is not None
                        and target.world_frame == observation.world_frame
                    ):
                        target.world_position = (
                            0.65 * target.world_position
                            + 0.35 * observation.world_position.astype(np.float64)
                        )
                    else:
                        target.world_position = observation.world_position.astype(
                            np.float64
                        ).copy()
                    target.world_frame = str(observation.world_frame)
                target.state = "REIDENTIFIED" if reidentified else "ACTIVE"
            target.bbox = observation.bbox
            target.last_seen = timestamp
            target.depth_valid = bool(observation.depth_valid)
            target.depth_m = float(observation.depth_m)
            if target.state != "REIDENTIFIED":
                target.state = (
                    "CONFIRMED"
                    if self.classification_is_stable(target)
                    else "ACTIVE"
                )
            output.append(target)
        return output

    def should_classify(
        self, target: WorldObject, appearance: np.ndarray, frame_index: int
    ) -> bool:
        if not target.votes:
            return True
        if frame_index - target.last_classified_frame < self.reclassify_interval_frames:
            return False
        aggregate = target.aggregate_probabilities()
        confidence = float(np.max(aggregate)) if aggregate is not None else 0.0
        history_incomplete = len(target.votes) < self.minimum_stable_votes
        low_confidence = confidence < self.stable_confidence
        appearance_changed = (
            target.appearance is not None
            and cosine_distance(target.appearance, appearance)
            >= self.appearance_change_threshold
        )
        return history_incomplete or low_confidence or appearance_changed

    def classification_is_stable(self, target: WorldObject) -> bool:
        aggregate = target.aggregate_probabilities()
        confidence = float(np.max(aggregate)) if aggregate is not None else 0.0
        return (
            len(target.votes) >= self.minimum_stable_votes
            and confidence >= self.stable_confidence
        )

    def add_classification(
        self,
        target: WorldObject,
        probabilities: Sequence[float],
        appearance: np.ndarray,
        frame_index: int,
    ) -> None:
        vector = np.asarray(probabilities, dtype=np.float32).reshape(-1)
        if vector.shape != (len(MATERIAL_CLASSES),):
            raise ValueError("classification probability vector must have length 5")
        if not np.all(np.isfinite(vector)) or np.any(vector < 0.0):
            raise ValueError("classification probabilities must be finite and non-negative")
        total = float(np.sum(vector))
        if total <= 0.0:
            raise ValueError("classification probabilities sum to zero")
        vector = vector / total
        confidence = float(np.max(vector))
        target.votes.append(
            ClassificationVote(vector, confidence, int(frame_index))
        )
        target.last_classified_frame = int(frame_index)
        target.appearance = appearance.astype(np.float32).copy()
        aggregate = target.aggregate_probabilities()
        if aggregate is None:
            target.stable_material = "unknown"
            target.classify_confidence = float("nan")
            return
        winner = int(np.argmax(aggregate))
        target.classify_confidence = float(aggregate[winner])
        target.stable_material = MATERIAL_CLASSES[winner]
        if self.classification_is_stable(target):
            target.state = "CONFIRMED"

    def mark_cleaned(self, object_id: int) -> bool:
        target = self.objects.get(int(object_id))
        if target is None:
            return False
        target.state = "CLEANED"
        return True

    def reset(self) -> None:
        self.objects.clear()
        self._next_object_id = 1
