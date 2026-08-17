#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import unittest

from sensor_msgs.msg import LaserScan


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "dual_laser_fusion.py"
SPEC = importlib.util.spec_from_file_location("dual_laser_fusion", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def one_point_scan(angle, distance, intensity=1.0):
    scan = LaserScan()
    scan.angle_min = angle
    scan.angle_max = angle
    scan.angle_increment = 1.0
    scan.range_min = 0.02
    scan.range_max = 12.0
    scan.ranges = [distance]
    scan.intensities = [intensity]
    return scan


class FusionMathTest(unittest.TestCase):
    def setUp(self):
        self.increment = math.radians(0.5)
        self.ranges = [math.inf] * 720
        self.intensities = [0.0] * 720

    def add(self, scan, x=0.0, y=0.0, yaw=0.0):
        MODULE.add_scan_to_bins(
            scan,
            x,
            y,
            yaw,
            -math.pi,
            self.increment,
            self.ranges,
            self.intensities,
        )

    def test_translation_and_rotation(self):
        self.add(one_point_scan(math.pi / 2.0, 1.0), x=0.5, yaw=-math.pi / 2.0)
        forward_index = 360
        self.assertAlmostEqual(self.ranges[forward_index], 1.5, places=6)

    def test_nearest_return_wins(self):
        self.add(one_point_scan(0.0, 2.0, 2.0))
        self.add(one_point_scan(0.0, 1.0, 7.0))
        forward_index = 360
        self.assertAlmostEqual(self.ranges[forward_index], 1.0, places=6)
        self.assertEqual(self.intensities[forward_index], 7.0)

    def test_invalid_returns_are_ignored(self):
        scan = one_point_scan(0.0, float("inf"))
        self.add(scan)
        self.assertTrue(all(math.isinf(value) for value in self.ranges))


if __name__ == "__main__":
    unittest.main()

