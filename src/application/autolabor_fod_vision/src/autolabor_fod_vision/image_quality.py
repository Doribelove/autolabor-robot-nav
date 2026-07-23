"""Image-quality measurements and conservative camera-control decisions.

This module deliberately has no ROS dependency so that the control policy can
be tested with synthetic images before it is allowed to change camera state.
"""

from dataclasses import dataclass
from typing import List

import cv2
import numpy as np


@dataclass(frozen=True)
class NormalizedRoi:
    """A rectangular region expressed as fractions of image dimensions."""

    x_min: float = 0.05
    x_max: float = 0.95
    y_min: float = 0.25
    y_max: float = 1.0

    def __post_init__(self):
        if not (
            0.0 <= self.x_min < self.x_max <= 1.0
            and 0.0 <= self.y_min < self.y_max <= 1.0
        ):
            raise ValueError(
                "ROI must satisfy 0 <= min < max <= 1 on both axes"
            )


@dataclass(frozen=True)
class ImageQualityMetrics:
    p10: float
    median: float
    p90: float
    p99: float
    dark_fraction: float
    bright_fraction: float
    sharpness: float
    mean_b: float
    mean_g: float
    mean_r: float
    roi_x: int
    roi_y: int
    roi_width: int
    roi_height: int


@dataclass(frozen=True)
class ControllerConfig:
    target_median: float = 115.0
    median_tolerance: float = 8.0
    dark_threshold: int = 5
    bright_threshold: int = 250
    max_dark_fraction: float = 0.08
    max_bright_fraction: float = 0.02
    dynamic_range_dark_fraction: float = 0.08
    dynamic_range_bright_fraction: float = 0.02
    sharpness_warn_threshold: float = 45.0
    color_mean_spread_warn: float = 45.0
    controller_gain: float = 0.60
    max_exposure_step_ratio: float = 1.35
    exposure_min_us: float = 200.0
    exposure_max_us: float = 5000.0
    gain_min: float = 0.0
    gain_max: float = 12.0
    gain_step: float = 0.5

    def __post_init__(self):
        if not 0.0 < self.target_median < 255.0:
            raise ValueError("target_median must be between 0 and 255")
        if self.median_tolerance < 0.0:
            raise ValueError("median_tolerance must not be negative")
        if not 0 <= self.dark_threshold < self.bright_threshold <= 255:
            raise ValueError("dark/bright thresholds must be ordered in 0..255")
        for name, value in (
            ("max_dark_fraction", self.max_dark_fraction),
            ("max_bright_fraction", self.max_bright_fraction),
            ("dynamic_range_dark_fraction", self.dynamic_range_dark_fraction),
            (
                "dynamic_range_bright_fraction",
                self.dynamic_range_bright_fraction,
            ),
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError("{} must be in 0..1".format(name))
        if not 0.0 < self.controller_gain <= 1.0:
            raise ValueError("controller_gain must be in (0, 1]")
        if self.max_exposure_step_ratio <= 1.0:
            raise ValueError("max_exposure_step_ratio must be greater than 1")
        if not 0.0 < self.exposure_min_us <= self.exposure_max_us:
            raise ValueError("configured exposure limits are invalid")
        if not 0.0 <= self.gain_min <= self.gain_max:
            raise ValueError("configured gain limits are invalid")
        if self.gain_step <= 0.0:
            raise ValueError("gain_step must be positive")


@dataclass(frozen=True)
class ImagingControlBounds:
    exposure_min_us: float
    exposure_max_us: float
    gain_min: float
    gain_max: float

    def __post_init__(self):
        if not 0.0 < self.exposure_min_us <= self.exposure_max_us:
            raise ValueError("hardware exposure limits are invalid")
        if not 0.0 <= self.gain_min <= self.gain_max:
            raise ValueError("hardware gain limits are invalid")


@dataclass(frozen=True)
class ControlRecommendation:
    exposure_time_us: float
    gain: float
    changed: bool
    reason: str


def measure_image_quality(
    image_bgr: np.ndarray,
    roi: NormalizedRoi = NormalizedRoi(),
    dark_threshold: int = 5,
    bright_threshold: int = 250,
    max_sample_width: int = 320,
) -> ImageQualityMetrics:
    """Measure exposure, clipping, colour balance, and edge sharpness."""
    if not isinstance(image_bgr, np.ndarray):
        raise TypeError("image_bgr must be a numpy array")
    if image_bgr.ndim != 3 or image_bgr.shape[2] != 3:
        raise ValueError("image_bgr must have shape HxWx3")
    height, width = image_bgr.shape[:2]
    if height <= 0 or width <= 0:
        raise ValueError("image dimensions must be positive")
    if not 0 <= dark_threshold < bright_threshold <= 255:
        raise ValueError("dark/bright thresholds must be ordered in 0..255")
    if max_sample_width <= 0:
        raise ValueError("max_sample_width must be positive")

    x0 = min(width - 1, int(np.floor(roi.x_min * width)))
    x1 = min(width, max(x0 + 1, int(np.ceil(roi.x_max * width))))
    y0 = min(height - 1, int(np.floor(roi.y_min * height)))
    y1 = min(height, max(y0 + 1, int(np.ceil(roi.y_max * height))))
    crop = image_bgr[y0:y1, x0:x1]

    if crop.shape[1] > max_sample_width:
        scale = float(max_sample_width) / float(crop.shape[1])
        sample = cv2.resize(
            crop,
            (max_sample_width, max(1, int(round(crop.shape[0] * scale)))),
            interpolation=cv2.INTER_AREA,
        )
    else:
        sample = crop

    if sample.dtype != np.uint8:
        sample = np.clip(sample, 0, 255).astype(np.uint8)
    gray = cv2.cvtColor(sample, cv2.COLOR_BGR2GRAY)
    percentiles = np.percentile(gray, [10, 50, 90, 99])
    means = np.mean(sample, axis=(0, 1))

    return ImageQualityMetrics(
        p10=float(percentiles[0]),
        median=float(percentiles[1]),
        p90=float(percentiles[2]),
        p99=float(percentiles[3]),
        dark_fraction=float(np.mean(gray <= dark_threshold)),
        bright_fraction=float(np.mean(gray >= bright_threshold)),
        sharpness=float(cv2.Laplacian(gray, cv2.CV_64F).var()),
        mean_b=float(means[0]),
        mean_g=float(means[1]),
        mean_r=float(means[2]),
        roi_x=x0,
        roi_y=y0,
        roi_width=x1 - x0,
        roi_height=y1 - y0,
    )


def quality_flags(
    metrics: ImageQualityMetrics, config: ControllerConfig
) -> List[str]:
    """Return human-readable conditions that can hurt detector input quality."""
    flags = []
    if (
        metrics.dark_fraction >= config.dynamic_range_dark_fraction
        and metrics.bright_fraction >= config.dynamic_range_bright_fraction
    ):
        flags.append("dynamic_range_conflict")
    else:
        if metrics.median < config.target_median - config.median_tolerance:
            flags.append("underexposed")
        elif metrics.median > config.target_median + config.median_tolerance:
            flags.append("overexposed")
        if metrics.dark_fraction > config.max_dark_fraction:
            flags.append("shadow_clipping")
        if metrics.bright_fraction > config.max_bright_fraction:
            flags.append("highlight_clipping")
    if metrics.sharpness < config.sharpness_warn_threshold:
        flags.append("low_sharpness")
    if (
        max(metrics.mean_b, metrics.mean_g, metrics.mean_r)
        - min(metrics.mean_b, metrics.mean_g, metrics.mean_r)
        > config.color_mean_spread_warn
    ):
        flags.append("large_color_cast")
    return flags


class ExposureGainController:
    """Rate-limited exposure-first controller with clipping safeguards."""

    def __init__(self, config: ControllerConfig):
        self.config = config

    @staticmethod
    def _clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    @staticmethod
    def _different(first: float, second: float, absolute: float) -> bool:
        return abs(first - second) > max(absolute, 0.001 * abs(first))

    def recommend(
        self,
        metrics: ImageQualityMetrics,
        exposure_time_us: float,
        gain: float,
        hardware: ImagingControlBounds,
    ) -> ControlRecommendation:
        config = self.config
        exposure_min = max(config.exposure_min_us, hardware.exposure_min_us)
        exposure_max = min(config.exposure_max_us, hardware.exposure_max_us)
        gain_min = max(config.gain_min, hardware.gain_min)
        gain_max = min(config.gain_max, hardware.gain_max)
        if exposure_min > exposure_max:
            raise ValueError(
                "configured and hardware exposure limits do not intersect"
            )
        if gain_min > gain_max:
            raise ValueError("configured and hardware gain limits do not intersect")

        bounded_exposure = self._clamp(
            float(exposure_time_us), exposure_min, exposure_max
        )
        bounded_gain = self._clamp(float(gain), gain_min, gain_max)
        if self._different(bounded_exposure, exposure_time_us, 1.0) or (
            self._different(bounded_gain, gain, 0.01)
        ):
            return ControlRecommendation(
                bounded_exposure, bounded_gain, True, "clamp_to_safe_limits"
            )

        if (
            metrics.dark_fraction >= config.dynamic_range_dark_fraction
            and metrics.bright_fraction >= config.dynamic_range_bright_fraction
        ):
            return ControlRecommendation(
                bounded_exposure,
                bounded_gain,
                False,
                "dynamic_range_conflict",
            )

        low = config.target_median - config.median_tolerance
        high = config.target_median + config.median_tolerance
        if low <= metrics.median <= high:
            return ControlRecommendation(
                bounded_exposure, bounded_gain, False, "brightness_in_deadband"
            )

        raw_ratio = config.target_median / max(1.0, metrics.median)
        damped_ratio = 1.0 + config.controller_gain * (raw_ratio - 1.0)
        damped_ratio = self._clamp(
            damped_ratio,
            1.0 / config.max_exposure_step_ratio,
            config.max_exposure_step_ratio,
        )

        if metrics.median < low:
            if bounded_exposure < exposure_max - 1.0:
                new_exposure = min(
                    exposure_max, bounded_exposure * damped_ratio
                )
                return ControlRecommendation(
                    new_exposure, bounded_gain, True, "increase_exposure"
                )
            if bounded_gain < gain_max - 0.01:
                return ControlRecommendation(
                    bounded_exposure,
                    min(gain_max, bounded_gain + config.gain_step),
                    True,
                    "increase_gain",
                )
            return ControlRecommendation(
                bounded_exposure,
                bounded_gain,
                False,
                "underexposed_at_limits",
            )

        # When the image is too bright, remove noisy analogue gain before
        # shortening exposure.
        if bounded_gain > gain_min + 0.01:
            return ControlRecommendation(
                bounded_exposure,
                max(gain_min, bounded_gain - config.gain_step),
                True,
                "decrease_gain",
            )
        if bounded_exposure > exposure_min + 1.0:
            return ControlRecommendation(
                max(exposure_min, bounded_exposure * damped_ratio),
                bounded_gain,
                True,
                "decrease_exposure",
            )
        return ControlRecommendation(
            bounded_exposure, bounded_gain, False, "overexposed_at_limits"
        )
