#!/usr/bin/env python3

import math
import threading
import time
import unittest

import rospy
import rostest
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Bool, Header


class OptionalCloudEnhancerContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("optional_cloud_enhancer_contract", anonymous=True)
        cls.condition = threading.Condition()
        cls.clouds = []
        cls.statuses = []
        cls.cloud_pub = rospy.Publisher("/test/mid_cloud", PointCloud2, queue_size=1)
        cls.scan_pub = rospy.Publisher("/test/dual_scan", LaserScan, queue_size=1)
        cls.cloud_sub = rospy.Subscriber(
            "/test/enhanced_cloud", PointCloud2, cls._cloud_callback)
        cls.status_sub = rospy.Subscriber(
            "/dual_lidar/enhancement_active", Bool, cls._status_callback)

        deadline = time.time() + 5.0
        while time.time() < deadline and (
                cls.cloud_pub.get_num_connections() == 0
                or cls.scan_pub.get_num_connections() == 0):
            time.sleep(0.05)
        if cls.cloud_pub.get_num_connections() == 0 or cls.scan_pub.get_num_connections() == 0:
            raise RuntimeError("test publishers did not connect to enhancer")

    @classmethod
    def _cloud_callback(cls, message):
        with cls.condition:
            cls.clouds.append(message)
            cls.condition.notify_all()

    @classmethod
    def _status_callback(cls, message):
        with cls.condition:
            cls.statuses.append(message.data)
            cls.condition.notify_all()

    def setUp(self):
        with self.condition:
            self.clouds.clear()
            self.statuses.clear()

    @staticmethod
    def make_cloud():
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("intensity", 12, PointField.FLOAT32, 1),
        ]
        return point_cloud2.create_cloud(
            Header(stamp=rospy.Time.now(), frame_id="body"),
            fields,
            [(2.0, 3.0, 4.0, 5.0)],
        )

    @staticmethod
    def make_scan(frame_id="body"):
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = frame_id
        scan.angle_min = 0.0
        scan.angle_max = 0.0
        scan.angle_increment = math.radians(0.5)
        scan.range_min = 0.02
        scan.range_max = 12.0
        scan.ranges = [1.0]
        scan.intensities = [9.0]
        return scan

    def wait_cloud(self, timeout=3.0):
        deadline = time.time() + timeout
        with self.condition:
            while not self.clouds and time.time() < deadline:
                self.condition.wait(deadline - time.time())
            self.assertTrue(self.clouds, "timed out waiting for enhanced cloud")
            return self.clouds[-1]

    def wait_status(self, expected, timeout=1.0):
        deadline = time.time() + timeout
        with self.condition:
            while expected not in self.statuses and time.time() < deadline:
                self.condition.wait(deadline - time.time())
            self.assertIn(expected, self.statuses)

    def publish_cloud_until_received(self):
        deadline = time.time() + 3.0
        while time.time() < deadline:
            self.cloud_pub.publish(self.make_cloud())
            try:
                return self.wait_cloud(timeout=0.25)
            except AssertionError:
                continue
        self.fail("timed out waiting for enhanced cloud")

    def test_01_no_scan_is_byte_exact_passthrough(self):
        source = self.make_cloud()
        self.cloud_pub.publish(source)
        output = self.wait_cloud()
        self.assertEqual(output.width, source.width)
        self.assertEqual(output.fields, source.fields)
        self.assertEqual(bytes(output.data), bytes(source.data))
        self.wait_status(False)

    def test_02_live_scan_appends_horizontal_point(self):
        self.scan_pub.publish(self.make_scan())
        time.sleep(0.03)
        output = self.publish_cloud_until_received()
        self.assertEqual(output.width, 2)
        points = list(point_cloud2.read_points(
            output, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[1][0], 1.0, places=5)
        self.assertAlmostEqual(points[1][1], 0.0, places=5)
        self.assertAlmostEqual(points[1][2], 0.0, places=5)
        self.assertAlmostEqual(points[1][3], 9.0, places=5)
        self.wait_status(True)

    def test_03_stale_scan_returns_to_passthrough(self):
        self.scan_pub.publish(self.make_scan())
        time.sleep(0.25)
        output = self.publish_cloud_until_received()
        self.assertEqual(output.width, 1)
        self.wait_status(False)

    def test_04_base_link_scan_is_transformed_into_body_cloud(self):
        self.scan_pub.publish(self.make_scan(frame_id="base_link"))
        time.sleep(0.05)
        output = self.publish_cloud_until_received()
        points = list(point_cloud2.read_points(
            output, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        self.assertEqual(len(points), 2)
        self.assertAlmostEqual(points[1][0], 1.0, places=5)
        self.assertAlmostEqual(points[1][1], 0.0, places=5)
        self.assertAlmostEqual(points[1][2], -0.6, places=5)
        self.assertAlmostEqual(points[1][3], 9.0, places=5)
        self.wait_status(True)


if __name__ == "__main__":
    rostest.rosrun(
        "autolabor_dual_lidar",
        "optional_cloud_enhancer_contract",
        OptionalCloudEnhancerContract,
    )
