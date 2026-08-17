#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import unittest

from sensor_msgs.msg import LaserScan
import yaml


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "optional_laserscan_fusion.py"
SPEC = importlib.util.spec_from_file_location("optional_laserscan_fusion", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def scan(ranges, angle_min=-math.pi / 2.0, increment=math.pi / 4.0):
    message = LaserScan()
    message.header.frame_id = "base_link"
    message.angle_min = angle_min
    message.angle_increment = increment
    message.angle_max = angle_min + (len(ranges) - 1) * increment
    message.range_min = 0.5
    message.range_max = 12.0
    message.ranges = list(ranges)
    message.intensities = [float(index) for index in range(len(ranges))]
    return message


class AvoidanceScanFusionMathTest(unittest.TestCase):
    def test_primary_is_unchanged_without_finite_optional_returns(self):
        primary = scan([1.0, 2.0, math.inf, 4.0, 5.0])
        optional = scan([math.inf] * 5)
        output = MODULE.merge_scans(primary, optional)
        self.assertEqual(output.ranges, primary.ranges)
        self.assertEqual(output.intensities, primary.intensities)

    def test_closest_return_wins(self):
        primary = scan([math.inf, 3.0, 4.0, 5.0, math.inf])
        optional = scan([math.inf, 2.0, 6.0, 1.0, math.inf])
        optional.intensities = [10.0, 11.0, 12.0, 13.0, 14.0]
        output = MODULE.merge_scans(primary, optional)
        self.assertEqual(output.ranges, [math.inf, 2.0, 4.0, 1.0, math.inf])
        self.assertEqual(output.intensities[1], 11.0)
        self.assertEqual(output.intensities[3], 13.0)

    def test_optional_scan_is_rebinned_to_primary_resolution(self):
        primary = scan([math.inf] * 5)
        optional = scan([2.0, 1.0, 3.0], increment=math.pi / 2.0)
        output = MODULE.merge_scans(primary, optional)
        self.assertEqual(output.ranges[0], 2.0)
        self.assertEqual(output.ranges[2], 1.0)
        self.assertEqual(output.ranges[4], 3.0)

    def test_invalid_optional_ranges_are_ignored(self):
        primary = scan([2.0, 2.0, 2.0, 2.0, 2.0])
        optional = scan([math.nan, 0.1, 13.0, math.inf, 1.0])
        output = MODULE.merge_scans(primary, optional)
        self.assertEqual(output.ranges[:4], primary.ranges[:4])
        self.assertEqual(output.ranges[4], 1.0)

    def test_optional_lidar_preserves_useful_near_field(self):
        primary = scan([math.inf] * 5)
        optional = scan([math.inf, math.inf, 0.2, math.inf, math.inf])
        optional.range_min = 0.02
        optional.range_max = 13.0
        output = MODULE.merge_scans(primary, optional)
        self.assertEqual(output.ranges[2], 0.2)
        self.assertEqual(output.range_min, 0.02)
        self.assertEqual(output.range_max, 13.0)
        self.assertEqual(primary.range_min, 0.5)


class AvoidanceCostmapFailClosedContractTest(unittest.TestCase):
    def test_all_dingo_obstacle_sources_require_fresh_scan(self):
        workspace_root = pathlib.Path(__file__).resolve().parents[4]
        config_dir = (
            workspace_root
            / "src/navigation_arena/arena-rosnav-3D/arena_navigation"
            / "arena_local_planer/model_based/conventional/config/dingo"
        )
        paths = (
            config_dir / "costmap_common_params_nomap.yaml",
            config_dir / "local_costmap_params_nomap.yaml",
            config_dir / "global_costmap_params_nomap.yaml",
        )
        for path in paths:
            with self.subTest(path=path.name), path.open(encoding="utf-8") as stream:
                config = yaml.safe_load(stream)
            obstacle_layer = config.get("obstacles_layer")
            if obstacle_layer is None:
                costmap = config.get("local_costmap", config.get("global_costmap"))
                obstacle_layer = costmap["obstacles_layer"]
            scan_source = obstacle_layer["scan"]
            self.assertEqual(scan_source["topic"], "/scan")
            self.assertEqual(scan_source["expected_update_rate"], 0.3)


if __name__ == "__main__":
    unittest.main()
