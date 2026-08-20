#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import tempfile
import unittest

import yaml
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import LaserScan


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "fused_scan_mapper.py"
SPEC = importlib.util.spec_from_file_location("fused_scan_mapper", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def scan(ranges, angle_min=-math.pi / 2.0, increment=math.pi / 2.0):
    message = LaserScan()
    message.header.frame_id = "base_link"
    message.angle_min = angle_min
    message.angle_increment = increment
    message.angle_max = angle_min + (len(ranges) - 1) * increment
    message.range_min = 0.1
    message.range_max = 12.0
    message.ranges = list(ranges)
    return message


class SparseOccupancyMapperTest(unittest.TestCase):
    def test_bresenham_includes_both_endpoints(self):
        self.assertEqual(
            [(0, 0), (1, 0), (2, 1), (3, 1)],
            list(MODULE.bresenham(0, 0, 3, 1)),
        )

    def test_scan_marks_free_ray_and_occupied_endpoint(self):
        mapper = MODULE.SparseOccupancyMapper(
            resolution=1.0,
            beam_stride=1,
            scan_stride=1,
            min_range=0.1,
            max_range=10.0,
            free_space_range=5.0,
            base_offset_x=0.0,
        )
        self.assertTrue(mapper.integrate_scan(scan([3.0], angle_min=0.0), 0.0, 0.0, 0.0))
        self.assertLess(mapper.cells[(1, 0)], 0.0)
        self.assertLess(mapper.cells[(2, 0)], 0.0)
        self.assertGreater(mapper.cells[(3, 0)], 0.0)

    def test_base_offset_is_rotated_with_fast_lio_pose(self):
        mapper = MODULE.SparseOccupancyMapper(
            resolution=0.5,
            beam_stride=1,
            scan_stride=1,
            min_range=0.1,
            max_range=10.0,
            free_space_range=5.0,
            base_offset_x=-0.5,
        )
        mapper.integrate_scan(scan([1.0], angle_min=0.0), 2.0, 3.0, math.pi / 2.0)
        # body=(2,3), base=(2,2.5), one metre forward at yaw=+90 -> (2,3.5)
        self.assertGreater(mapper.cells[(4, 7)], 0.0)
        self.assertAlmostEqual(2.0, mapper.final_base_pose[0])
        self.assertAlmostEqual(2.5, mapper.final_base_pose[1])

    def test_save_produces_map_server_files(self):
        mapper = MODULE.SparseOccupancyMapper(
            resolution=0.5,
            beam_stride=1,
            scan_stride=1,
            min_range=0.1,
            max_range=10.0,
            free_space_range=5.0,
            base_offset_x=0.0,
        )
        mapper.integrate_scan(scan([2.0], angle_min=0.0), 0.0, 0.0, 0.0)
        with tempfile.TemporaryDirectory() as output_dir:
            result = mapper.save(output_dir, padding_m=0.5)
            pgm = pathlib.Path(result["pgm"])
            config = yaml.safe_load(pathlib.Path(result["yaml"]).read_text())
            metadata = yaml.safe_load(pathlib.Path(result["metadata"]).read_text())
            self.assertTrue(pgm.read_bytes().startswith(b"P5\n"))
            self.assertEqual("map.pgm", config["image"])
            self.assertEqual(0.5, config["resolution"])
            self.assertEqual("complete", metadata["status"])
            self.assertEqual(1, metadata["integrated_scans"])
            self.assertEqual("/dual_lidar/scan", metadata["scan_topic"])
            self.assertEqual("dual_ld19_only", metadata["occupancy_source"])

    def test_quaternion_yaw(self):
        quaternion = Quaternion(z=math.sin(math.pi / 4.0), w=math.cos(math.pi / 4.0))
        self.assertAlmostEqual(math.pi / 2.0, MODULE.yaw_from_quaternion(quaternion))


if __name__ == "__main__":
    unittest.main()
