#!/usr/bin/env python3

import unittest

import numpy as np

from autolabor_fod_vision.image_quality import (
    ControllerConfig,
    ExposureGainController,
    ImagingControlBounds,
    NormalizedRoi,
    measure_image_quality,
    quality_flags,
)


HARDWARE = ImagingControlBounds(
    exposure_min_us=20.0,
    exposure_max_us=100000.0,
    gain_min=0.0,
    gain_max=24.0,
)


def metrics_for(level):
    image = np.full((120, 160, 3), level, dtype=np.uint8)
    return measure_image_quality(image, max_sample_width=160)


class ImageQualityMeasurementTest(unittest.TestCase):
    def test_uniform_image_metrics(self):
        metrics = metrics_for(100)
        self.assertAlmostEqual(metrics.median, 100.0)
        self.assertAlmostEqual(metrics.dark_fraction, 0.0)
        self.assertAlmostEqual(metrics.bright_fraction, 0.0)
        self.assertAlmostEqual(metrics.sharpness, 0.0)

    def test_ground_roi_excludes_bright_upper_area(self):
        image = np.full((100, 120, 3), 90, dtype=np.uint8)
        image[:50, :] = 255
        metrics = measure_image_quality(
            image,
            roi=NormalizedRoi(0.0, 1.0, 0.5, 1.0),
            max_sample_width=120,
        )
        self.assertAlmostEqual(metrics.median, 90.0)
        self.assertAlmostEqual(metrics.bright_fraction, 0.0)

    def test_invalid_roi_is_rejected(self):
        with self.assertRaises(ValueError):
            NormalizedRoi(0.7, 0.2, 0.0, 1.0)


class ExposureGainControllerTest(unittest.TestCase):
    def setUp(self):
        self.config = ControllerConfig()
        self.controller = ExposureGainController(self.config)

    def test_deadband_keeps_controls(self):
        result = self.controller.recommend(
            metrics_for(115), 2500.0, 0.0, HARDWARE
        )
        self.assertFalse(result.changed)
        self.assertEqual(result.reason, "brightness_in_deadband")

    def test_dark_image_increases_exposure_with_rate_limit(self):
        result = self.controller.recommend(
            metrics_for(20), 1000.0, 0.0, HARDWARE
        )
        self.assertTrue(result.changed)
        self.assertEqual(result.reason, "increase_exposure")
        self.assertAlmostEqual(result.exposure_time_us, 1350.0)
        self.assertAlmostEqual(result.gain, 0.0)

    def test_dark_image_uses_gain_after_exposure_cap(self):
        result = self.controller.recommend(
            metrics_for(20), self.config.exposure_max_us, 1.0, HARDWARE
        )
        self.assertEqual(result.reason, "increase_gain")
        self.assertAlmostEqual(result.exposure_time_us, 5000.0)
        self.assertAlmostEqual(result.gain, 1.5)

    def test_bright_image_removes_gain_before_exposure(self):
        result = self.controller.recommend(
            metrics_for(220), 2500.0, 3.0, HARDWARE
        )
        self.assertEqual(result.reason, "decrease_gain")
        self.assertAlmostEqual(result.exposure_time_us, 2500.0)
        self.assertAlmostEqual(result.gain, 2.5)

    def test_bright_image_reduces_exposure_at_minimum_gain(self):
        result = self.controller.recommend(
            metrics_for(220), 2500.0, 0.0, HARDWARE
        )
        self.assertEqual(result.reason, "decrease_exposure")
        self.assertLess(result.exposure_time_us, 2500.0)
        self.assertGreaterEqual(result.exposure_time_us, 2500.0 / 1.35)

    def test_dynamic_range_conflict_does_not_chase_brightness(self):
        image = np.zeros((100, 100, 3), dtype=np.uint8)
        image[:, 50:] = 255
        metrics = measure_image_quality(
            image,
            roi=NormalizedRoi(0.0, 1.0, 0.0, 1.0),
            max_sample_width=100,
        )
        result = self.controller.recommend(metrics, 2500.0, 0.0, HARDWARE)
        self.assertFalse(result.changed)
        self.assertEqual(result.reason, "dynamic_range_conflict")
        self.assertIn("dynamic_range_conflict", quality_flags(metrics, self.config))

    def test_native_auto_value_is_clamped_to_motion_safe_cap(self):
        result = self.controller.recommend(
            metrics_for(115), 20000.0, 18.0, HARDWARE
        )
        self.assertTrue(result.changed)
        self.assertEqual(result.reason, "clamp_to_safe_limits")
        self.assertAlmostEqual(result.exposure_time_us, 5000.0)
        self.assertAlmostEqual(result.gain, 12.0)


if __name__ == "__main__":
    unittest.main()
