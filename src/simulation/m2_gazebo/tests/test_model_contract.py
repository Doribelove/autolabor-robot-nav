#!/usr/bin/env python3
import math
import os
import subprocess
import sys
import unittest
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_DIR, "src"))

from m2_gazebo.sensor_transport import noisy_range, release_time, stopping_distance


class ModelContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        xacro_file = os.path.join(PACKAGE_DIR, "urdf", "m2.urdf.xacro")
        rendered = subprocess.check_output(["xacro", "--inorder", xacro_file], text=True)
        cls.robot = ET.fromstring(rendered)
        with open(os.path.join(PACKAGE_DIR, "config", "simulation_candidates.yaml"), encoding="utf-8") as stream:
            cls.candidates = yaml.safe_load(stream)

    def test_required_links_and_joints(self):
        links = {link.attrib["name"] for link in self.robot.findall("link")}
        joints = {joint.attrib["name"] for joint in self.robot.findall("joint")}
        self.assertTrue({"base_link", "laser_link", "imu_link"}.issubset(links))
        self.assertTrue({
            "front_left_steer_joint", "front_right_steer_joint",
            "front_left_wheel_joint", "front_right_wheel_joint",
            "rear_left_wheel_joint", "rear_right_wheel_joint",
        }.issubset(joints))
        child_links = {child.attrib["link"] for child in self.robot.findall("joint/child")}
        self.assertNotIn("base_link", child_links, "base_link must be the URDF root for odom -> base_link")

    def test_required_gazebo_plugins(self):
        filenames = {plugin.attrib.get("filename") for plugin in self.robot.iter("plugin")}
        self.assertIn("libm2_ackermann_plugin.so", filenames)
        self.assertIn("libgazebo_ros_laser.so", filenames)
        self.assertIn("libgazebo_ros_bumper.so", filenames)

    def test_every_candidate_is_explicitly_uncalibrated(self):
        self.assertFalse(self.candidates["calibrated"])
        self.assertEqual("simulation_candidate", self.candidates["status"])
        for section in ("geometry", "dynamics", "sensors"):
            for item in self.candidates[section].values():
                self.assertFalse(item["calibrated"])
                self.assertEqual("simulation_candidate", item["status"])

    def test_turning_radius_derivation(self):
        geometry = self.candidates["geometry"]
        dynamics = self.candidates["dynamics"]
        expected = math.atan(
            geometry["wheelbase_m"]["value"] /
            dynamics["min_turning_radius_m"]["value"]
        )
        self.assertAlmostEqual(expected, dynamics["max_center_steering_angle_rad"]["value"], places=4)

    def test_v2_dynamics_candidates_are_ordered_and_nonzero(self):
        dynamics = self.candidates["dynamics"]
        self.assertGreater(dynamics["speed_time_constant_s"]["value"], 0.0)
        self.assertGreater(dynamics["steering_time_constant_s"]["value"], 0.0)
        service = dynamics["max_deceleration_mps2"]["value"]
        brake = dynamics["max_brake_deceleration_mps2"]["value"]
        emergency = dynamics["max_emergency_deceleration_mps2"]["value"]
        self.assertGreaterEqual(brake, service)
        self.assertGreaterEqual(emergency, brake)
        self.assertGreater(dynamics["command_delay_s"]["value"], 0.0)
        self.assertLessEqual(
            dynamics["command_jitter_s"]["value"],
            dynamics["command_delay_s"]["value"],
        )

    def test_v2_sensor_delay_noise_and_stopping_helpers_are_deterministic(self):
        first = release_time(1.0, 0.06, 0.01, 42, 7)
        self.assertEqual(first, release_time(1.0, 0.06, 0.01, 42, 7))
        self.assertGreaterEqual(first, 1.05)
        self.assertLessEqual(first, 1.07)
        measurement = noisy_range(5.0, 0.1, 30.0, 0.01, 42, 3, 100)
        self.assertEqual(
            measurement, noisy_range(5.0, 0.1, 30.0, 0.01, 42, 3, 100)
        )
        self.assertGreater(stopping_distance(1.0, 0.08, 2.4), 0.20)


if __name__ == "__main__":
    unittest.main()
