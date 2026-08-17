#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2


class LivoxCustomToPointCloudContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("livox_custom_to_pointcloud_contract", anonymous=True)
        cls.condition = threading.Condition()
        cls.clouds = []
        cls.publisher = rospy.Publisher("/test/livox_custom", CustomMsg, queue_size=1)
        cls.subscriber = rospy.Subscriber(
            "/test/livox_cloud", PointCloud2, cls._cloud_callback)

        deadline = time.time() + 5.0
        while time.time() < deadline and cls.publisher.get_num_connections() == 0:
            time.sleep(0.05)
        if cls.publisher.get_num_connections() == 0:
            raise RuntimeError("test publisher did not connect to converter")

    @classmethod
    def _cloud_callback(cls, message):
        with cls.condition:
            cls.clouds.append(message)
            cls.condition.notify_all()

    @staticmethod
    def point(x, y, z, reflectivity=1, tag=0x00, line=0):
        return CustomPoint(
            offset_time=0,
            x=x,
            y=y,
            z=z,
            reflectivity=reflectivity,
            tag=tag,
            line=line,
        )

    def wait_cloud(self, timeout=3.0):
        deadline = time.time() + timeout
        with self.condition:
            while not self.clouds and time.time() < deadline:
                self.condition.wait(deadline - time.time())
            self.assertTrue(self.clouds, "timed out waiting for converted cloud")
            return self.clouds[-1]

    def test_transform_and_livox_validity_filters(self):
        message = CustomMsg()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "livox_frame"
        message.points = [
            # +90 deg yaw and translation (1,2,3): (x,y) -> (1-y,2+x).
            self.point(-2.0, 1.0, 0.0, reflectivity=9),  # target (0,0), excluded
            self.point(-1.5, 0.25, 0.0, reflectivity=8), # target (+0.75,+0.5), excluded boundary
            self.point(-2.5, 1.75, 0.0, reflectivity=7), # target (-0.75,-0.5), excluded boundary
            self.point(-2.0, 0.2, 0.0),                 # target (0.8,0), kept outside +X
            self.point(-2.0, 1.8, 0.0),                 # target (-0.8,0), kept outside -X
            self.point(-1.4, 1.0, 0.0),                 # target (0,0.6), kept outside +Y
            self.point(-2.6, 1.0, 0.0),                 # target (0,-0.6), kept outside -Y
            self.point(0.0, -9.0, 0.0),                 # target (10,2), kept below max range
            self.point(0.0, -12.1, 0.0),                # above the original 12 m max range
            self.point(0.1, 0.0, 0.0),       # below min_range
            self.point(1.0, 0.0, 0.0, tag=0x30),  # invalid spatial tag
            self.point(1.0, 0.0, 0.0, line=4),    # outside MID360's four lines
        ]
        message.point_num = len(message.points)
        self.publisher.publish(message)

        cloud = self.wait_cloud()
        self.assertEqual(cloud.header.frame_id, "base_link")
        self.assertEqual(cloud.header.stamp, message.header.stamp)
        points = list(point_cloud2.read_points(
            cloud, field_names=("x", "y", "z", "intensity"), skip_nans=True))
        self.assertEqual(len(points), 5)
        # The exclusion crop is evaluated after translation/rotation in base_link.
        self.assertAlmostEqual(points[0][0], 0.8, places=5)
        self.assertAlmostEqual(points[0][1], 0.0, places=5)
        self.assertAlmostEqual(points[0][2], 3.0, places=5)
        self.assertAlmostEqual(points[1][0], -0.8, places=5)
        self.assertAlmostEqual(points[1][1], 0.0, places=5)
        self.assertAlmostEqual(points[2][0], 0.0, places=5)
        self.assertAlmostEqual(points[2][1], 0.60, places=5)
        self.assertAlmostEqual(points[3][0], 0.0, places=5)
        self.assertAlmostEqual(points[3][1], -0.60, places=5)
        self.assertAlmostEqual(points[4][0], 10.0, places=5)
        self.assertAlmostEqual(points[4][1], 2.0, places=5)


if __name__ == "__main__":
    rostest.rosrun(
        "robot_bringup",
        "livox_custom_to_pointcloud_contract",
        LivoxCustomToPointCloudContract,
    )
