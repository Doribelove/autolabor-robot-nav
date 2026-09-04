#!/usr/bin/env python3

import math
import unittest

import numpy as np

from autolabor_fod_vision.depth_fusion import (
    estimate_detection_depth,
    nearest_synchronized_message,
)


class _Stamp:
    def __init__(self, value):
        self.value = value

    def to_sec(self):
        return self.value


class _Message:
    def __init__(self, stamp, frame_id):
        self.header = type("Header", (), {})()
        self.header.stamp = _Stamp(stamp)
        self.header.frame_id = frame_id


class DepthFusionTest(unittest.TestCase):
    def test_timestamp_match_selects_source_frame_not_latest_sensor_frame(self):
        messages = [
            _Message(10.00, "camera"),
            _Message(10.04, "camera"),
            _Message(14.00, "camera"),
        ]
        selected, delta = nearest_synchronized_message(
            messages, 10.01, "camera", 0.06
        )

        self.assertIs(selected, messages[0])
        self.assertAlmostEqual(delta, 0.01, places=6)

    def test_timestamp_match_rejects_wrong_frame_and_out_of_tolerance(self):
        wrong_frame, wrong_delta = nearest_synchronized_message(
            [_Message(10.00, "other_camera")], 10.00, "camera", 0.06
        )
        stale, stale_delta = nearest_synchronized_message(
            [_Message(10.20, "camera")], 10.00, "camera", 0.06
        )

        self.assertIsNone(wrong_frame)
        self.assertTrue(math.isnan(wrong_delta))
        self.assertIsNone(stale)
        self.assertTrue(math.isnan(stale_delta))

    def test_detect_box_uses_inset_median_and_rejects_invalid_pixels(self):
        depth = np.full((80, 100), 6.0, dtype=np.float32)
        depth[20:61, 25:76] = 2.25
        depth[30, 40] = np.nan
        depth[31, 41] = np.inf
        depth[32, 42] = 14.0

        estimate = estimate_detection_depth(
            depth,
            (20.0, 15.0, 80.0, 65.0),
            min_samples=20,
            bbox_inset_fraction=0.20,
        )

        self.assertTrue(estimate.valid)
        self.assertAlmostEqual(estimate.depth_m, 2.25, places=3)
        self.assertLess(estimate.mad_m, 0.01)
        self.assertGreater(estimate.sample_count, 100)

    def test_segmentation_polygon_excludes_bbox_background(self):
        depth = np.full((80, 100), 8.0, dtype=np.float32)
        polygon = [(30.0, 25.0), (70.0, 25.0), (65.0, 60.0), (35.0, 60.0)]
        # Fill the same polygon in the synthetic depth map.
        import cv2

        cv2.fillPoly(
            depth,
            [np.rint(np.asarray(polygon)).astype(np.int32)],
            1.40,
        )
        estimate = estimate_detection_depth(
            depth,
            (20.0, 15.0, 80.0, 65.0),
            polygon=polygon,
            min_samples=20,
        )

        self.assertTrue(estimate.valid)
        self.assertAlmostEqual(estimate.depth_m, 1.40, places=2)

    def test_sparse_depth_is_explicitly_invalid(self):
        depth = np.full((40, 60), np.nan, dtype=np.float32)
        depth[18:20, 28:30] = 1.0
        estimate = estimate_detection_depth(
            depth,
            (10.0, 10.0, 50.0, 30.0),
            min_samples=20,
            min_valid_fraction=0.20,
        )

        self.assertFalse(estimate.valid)
        self.assertTrue(math.isnan(estimate.depth_m))
        self.assertEqual(estimate.sample_count, 4)

    def test_out_of_range_depth_is_invalid(self):
        depth = np.full((40, 60), 20.0, dtype=np.float32)
        estimate = estimate_detection_depth(
            depth,
            (10.0, 10.0, 50.0, 30.0),
            max_depth_m=15.0,
        )
        self.assertFalse(estimate.valid)


if __name__ == "__main__":
    unittest.main()
