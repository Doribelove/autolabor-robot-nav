#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

from autolabor_fod_vision.camera_info import (
    load_camera_calibration,
    uncalibrated,
)


CALIBRATION_YAML = """\
image_width: 640
image_height: 480
camera_name: test_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [400.0, 0.0, 320.0, 0.0, 410.0, 240.0, 0.0, 0.0, 1.0]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.1, -0.2, 0.0, 0.0, 0.0]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [400.0, 0.0, 320.0, 0.0, 0.0, 410.0, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
"""


class CameraInfoTest(unittest.TestCase):
    def test_uncalibrated_is_explicit(self):
        calibration = uncalibrated(1280, 720, "camera")
        self.assertFalse(calibration.calibrated)
        self.assertEqual(calibration.k, [0.0] * 9)

    def test_load_and_scale(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "camera.yaml"
            path.write_text(CALIBRATION_YAML, encoding="utf-8")
            calibration = load_camera_calibration("file://" + str(path))
        self.assertTrue(calibration.calibrated)
        scaled = calibration.scaled(1280, 960)
        self.assertEqual(scaled.k[0], 800.0)
        self.assertEqual(scaled.k[2], 640.0)
        self.assertEqual(scaled.k[4], 820.0)
        self.assertEqual(scaled.k[5], 480.0)

    def test_invalid_file_rejected(self):
        with self.assertRaises(FileNotFoundError):
            load_camera_calibration("/tmp/does-not-exist-fod-camera.yaml")


if __name__ == "__main__":
    unittest.main()
