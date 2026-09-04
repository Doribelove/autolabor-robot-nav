"""ROS-independent robust depth extraction for YOLO detections."""

from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class DepthEstimate:
    valid: bool
    depth_m: float
    mad_m: float
    sample_count: int
    valid_fraction: float


def nearest_synchronized_message(
    messages: Iterable[object],
    source_stamp_sec: float,
    source_frame: str,
    tolerance_sec: float,
) -> Tuple[Optional[object], float]:
    """Return the sensor message nearest to one RGB source timestamp.

    The helper intentionally requires the same optical frame and never falls
    back to the newest message.  It is ROS-import-free so the timestamp
    matching contract can be unit-tested with lightweight message doubles.
    """

    if (
        not math.isfinite(float(source_stamp_sec))
        or float(source_stamp_sec) <= 0.0
        or not str(source_frame)
        or not math.isfinite(float(tolerance_sec))
        or float(tolerance_sec) < 0.0
    ):
        return None, float("nan")

    nearest = None
    nearest_delta = float("inf")
    for message in messages:
        try:
            if str(message.header.frame_id) != str(source_frame):
                continue
            stamp_sec = float(message.header.stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            continue
        if not math.isfinite(stamp_sec) or stamp_sec <= 0.0:
            continue
        delta = abs(stamp_sec - float(source_stamp_sec))
        if delta < nearest_delta:
            nearest = message
            nearest_delta = delta

    if nearest is None or nearest_delta > float(tolerance_sec):
        return None, float("nan")
    return nearest, float(nearest_delta)


def _invalid(sample_count: int = 0, valid_fraction: float = 0.0) -> DepthEstimate:
    return DepthEstimate(
        valid=False,
        depth_m=float("nan"),
        mad_m=float("nan"),
        sample_count=int(sample_count),
        valid_fraction=float(valid_fraction),
    )


def _clipped_bounds(
    bbox: Tuple[float, float, float, float], width: int, height: int
) -> Tuple[int, int, int, int]:
    xmin, ymin, xmax, ymax = bbox
    x1 = max(0, min(width, int(math.floor(xmin))))
    y1 = max(0, min(height, int(math.floor(ymin))))
    x2 = max(x1, min(width, int(math.ceil(xmax)) + 1))
    y2 = max(y1, min(height, int(math.ceil(ymax)) + 1))
    return x1, y1, x2, y2


def _sampling_mask(
    shape: Tuple[int, int],
    bbox: Tuple[float, float, float, float],
    polygon: Sequence[Tuple[float, float]],
    bbox_inset_fraction: float,
):
    height, width = shape
    x1, y1, x2, y2 = _clipped_bounds(bbox, width, height)
    if x2 <= x1 or y2 <= y1:
        return (x1, y1, x2, y2), np.zeros((0, 0), dtype=bool)

    roi_height = y2 - y1
    roi_width = x2 - x1
    mask = np.zeros((roi_height, roi_width), dtype=np.uint8)
    if len(polygon) >= 3:
        points = np.asarray(polygon, dtype=np.float64)
        if points.ndim == 2 and points.shape[1] == 2 and np.isfinite(points).all():
            points[:, 0] = np.clip(points[:, 0] - x1, 0, roi_width - 1)
            points[:, 1] = np.clip(points[:, 1] - y1, 0, roi_height - 1)
            cv2.fillPoly(mask, [np.rint(points).astype(np.int32)], 1)
            # Remove one boundary pixel where registered depth commonly mixes
            # foreground and background. Keep very small masks untouched.
            if roi_width >= 7 and roi_height >= 7 and np.count_nonzero(mask) >= 49:
                mask = cv2.erode(mask, np.ones((3, 3), dtype=np.uint8), iterations=1)
            return (x1, y1, x2, y2), mask.astype(bool)

    inset_x = int(round(roi_width * bbox_inset_fraction))
    inset_y = int(round(roi_height * bbox_inset_fraction))
    if 2 * inset_x >= roi_width:
        inset_x = 0
    if 2 * inset_y >= roi_height:
        inset_y = 0
    mask[inset_y : roi_height - inset_y, inset_x : roi_width - inset_x] = 1
    return (x1, y1, x2, y2), mask.astype(bool)


def estimate_detection_depth(
    depth_image: np.ndarray,
    bbox: Tuple[float, float, float, float],
    polygon: Sequence[Tuple[float, float]] = (),
    min_depth_m: float = 0.30,
    max_depth_m: float = 15.0,
    min_samples: int = 20,
    min_valid_fraction: float = 0.20,
    bbox_inset_fraction: float = 0.18,
) -> DepthEstimate:
    """Estimate one object's axial depth in metres.

    A segmentation mask is preferred when present. Detect-only models use the
    inset centre of the bounding box. NaN/Inf and out-of-range ZED samples are
    rejected, followed by a median/MAD outlier pass.
    """

    array = np.asarray(depth_image)
    if array.ndim != 2 or array.size == 0:
        return _invalid()
    if not (
        math.isfinite(min_depth_m)
        and math.isfinite(max_depth_m)
        and 0.0 < min_depth_m < max_depth_m
        and min_samples > 0
        and 0.0 <= min_valid_fraction <= 1.0
        and 0.0 <= bbox_inset_fraction < 0.5
    ):
        raise ValueError("invalid depth-estimator configuration")

    bounds, support_mask = _sampling_mask(
        array.shape, bbox, polygon, bbox_inset_fraction
    )
    x1, y1, x2, y2 = bounds
    support_count = int(np.count_nonzero(support_mask))
    if support_count == 0:
        return _invalid()

    roi = array[y1:y2, x1:x2]
    samples = np.asarray(roi[support_mask], dtype=np.float64)
    valid = np.isfinite(samples) & (samples >= min_depth_m) & (samples <= max_depth_m)
    samples = samples[valid]
    sample_count = int(samples.size)
    valid_fraction = sample_count / float(support_count)
    if sample_count < min_samples or valid_fraction < min_valid_fraction:
        return _invalid(sample_count, valid_fraction)

    median = float(np.median(samples))
    absolute_deviation = np.abs(samples - median)
    mad = float(np.median(absolute_deviation))
    # Preserve genuinely flat surfaces while rejecting isolated flying pixels.
    cutoff = max(0.03, 3.0 * 1.4826 * mad)
    inliers = samples[absolute_deviation <= cutoff]
    if inliers.size >= min_samples:
        samples = inliers
        sample_count = int(samples.size)
        median = float(np.median(samples))
        mad = float(np.median(np.abs(samples - median)))

    return DepthEstimate(
        valid=True,
        depth_m=median,
        mad_m=mad,
        sample_count=sample_count,
        valid_fraction=valid_fraction,
    )
