#!/usr/bin/env python3

import math
import threading
import time
import unittest

import rospy
import rostest
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class AvoidanceScanFusionContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("avoidance_scan_fusion_contract", anonymous=True)
        cls.condition = threading.Condition()
        cls.outputs = []
        cls.active_states = []
        cls.modes = []
        cls.primary_publisher = rospy.Publisher(
            "/test/mid360_scan", LaserScan, queue_size=1)
        cls.optional_publisher = rospy.Publisher(
            "/test/dual_scan", LaserScan, queue_size=1)
        cls.output_subscriber = rospy.Subscriber(
            "/test/avoidance_scan", LaserScan, cls._output_callback)
        cls.active_subscriber = rospy.Subscriber(
            "/avoidance/dual_lidar_active", Bool, cls._active_callback)
        cls.mode_subscriber = rospy.Subscriber(
            "/avoidance/source_mode", String, cls._mode_callback)

        deadline = time.time() + 5.0
        while time.time() < deadline and (
                cls.primary_publisher.get_num_connections() == 0
                or cls.optional_publisher.get_num_connections() == 0):
            time.sleep(0.05)
        if (cls.primary_publisher.get_num_connections() == 0
                or cls.optional_publisher.get_num_connections() == 0):
            raise RuntimeError("test publishers did not connect to scan fusion")

    @classmethod
    def _output_callback(cls, message):
        with cls.condition:
            cls.outputs.append(message)
            cls.condition.notify_all()

    @classmethod
    def _active_callback(cls, message):
        with cls.condition:
            cls.active_states.append(message.data)
            cls.condition.notify_all()

    @classmethod
    def _mode_callback(cls, message):
        with cls.condition:
            cls.modes.append(message.data)
            cls.condition.notify_all()

    def setUp(self):
        with self.condition:
            self.outputs.clear()
            self.active_states.clear()
            self.modes.clear()

    @staticmethod
    def make_scan(ranges):
        message = LaserScan()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "base_link"
        message.angle_min = -math.pi / 2.0
        message.angle_increment = math.pi / 4.0
        message.angle_max = message.angle_min + (len(ranges) - 1) * message.angle_increment
        message.range_min = 0.5
        message.range_max = 12.0
        message.ranges = list(ranges)
        message.intensities = [0.0] * len(ranges)
        return message

    def wait_output(self, timeout=3.0):
        deadline = time.time() + timeout
        with self.condition:
            while not self.outputs and time.time() < deadline:
                self.condition.wait(deadline - time.time())
            self.assertTrue(self.outputs, "timed out waiting for avoidance scan")
            return self.outputs[-1]

    def publish_primary_until_received(self, ranges):
        deadline = time.time() + 3.0
        while time.time() < deadline:
            self.primary_publisher.publish(self.make_scan(ranges))
            try:
                return self.wait_output(timeout=0.25)
            except AssertionError:
                continue
        self.fail("timed out waiting for avoidance scan")

    def wait_status(self, active, mode, timeout=1.0):
        deadline = time.time() + timeout
        with self.condition:
            while time.time() < deadline and (
                    active not in self.active_states or mode not in self.modes):
                self.condition.wait(deadline - time.time())
            self.assertIn(active, self.active_states)
            self.assertIn(mode, self.modes)

    def test_01_no_optional_scan_is_primary_passthrough(self):
        source_ranges = [1.0, 2.0, math.inf, 4.0, 5.0]
        output = self.publish_primary_until_received(source_ranges)
        self.assertEqual(list(output.ranges), source_ranges)
        self.wait_status(False, "mid360")

    def test_02_recent_optional_scan_adds_closer_obstacles(self):
        self.optional_publisher.publish(
            self.make_scan([math.inf, 1.0, 3.0, math.inf, math.inf]))
        time.sleep(0.03)
        output = self.publish_primary_until_received(
            [math.inf, 2.0, 2.0, 4.0, math.inf])
        self.assertEqual(output.ranges[1], 1.0)
        self.assertEqual(output.ranges[2], 2.0)
        self.wait_status(True, "mid360+dual_ld19")

    def test_03_stale_optional_scan_falls_back_to_primary(self):
        self.optional_publisher.publish(
            self.make_scan([1.0, 1.0, 1.0, 1.0, 1.0]))
        time.sleep(0.25)
        source_ranges = [5.0, 5.0, 5.0, 5.0, 5.0]
        output = self.publish_primary_until_received(source_ranges)
        self.assertEqual(list(output.ranges), source_ranges)
        self.wait_status(False, "mid360")


if __name__ == "__main__":
    rostest.rosrun(
        "robot_bringup",
        "avoidance_scan_fusion_contract",
        AvoidanceScanFusionContract,
    )
