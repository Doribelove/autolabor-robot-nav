#!/usr/bin/env python3
"""ROS graph test for serialized GPS/FOD ownership and completion recovery."""

import threading
import time
import unittest

import rospy
import rostest
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse


class FodNavigationModeRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.RLock()
        cls.latest_output = Twist()
        cls.latest_state = ""
        cls.pause_requests = []
        cls.visual_requests = []
        cls.cancel_count = 0

        cls.output_sub = rospy.Subscriber(
            "/test/cmd_vel", Twist, cls._output_cb, queue_size=20
        )
        cls.state_sub = rospy.Subscriber(
            "/fod_navigation_mode_test/state",
            String,
            cls._state_cb,
            queue_size=20,
        )
        cls.cancel_sub = rospy.Subscriber(
            "/test/move_base/cancel",
            GoalID,
            cls._cancel_cb,
            queue_size=20,
        )
        cls.gps_pub = rospy.Publisher(
            "/test/cmd_vel_gps", Twist, queue_size=1
        )
        cls.fod_pub = rospy.Publisher(
            "/test/cmd_vel_fod", Twist, queue_size=1
        )
        cls.odom_pub = rospy.Publisher("/test/odom", Odometry, queue_size=1)
        cls.visual_state_pub = rospy.Publisher(
            "/test/fod/state", String, queue_size=1, latch=True
        )
        cls.pause_service = rospy.Service(
            "/test/gps/set_paused", SetBool, cls._pause_cb
        )
        cls.visual_service = rospy.Service(
            "/test/fod/set_enabled", SetBool, cls._visual_cb
        )
        cls.publish_timer = rospy.Timer(
            rospy.Duration(0.05), cls._publish_inputs
        )
        cls.visual_state_pub.publish(String(data="DISABLED"))

        rospy.wait_for_service(
            "/fod_navigation_mode_test/set_fod_enabled", timeout=5.0
        )
        cls.mode_proxy = rospy.ServiceProxy(
            "/fod_navigation_mode_test/set_fod_enabled", SetBool
        )
        cls._wait_until(lambda: cls.latest_state == "GPS_ACTIVE", 5.0)
        cls._wait_until(
            lambda: abs(cls.latest_output.linear.x - 1.0) < 1e-6,
            5.0,
        )

    @classmethod
    def tearDownClass(cls):
        cls.publish_timer.shutdown()
        for item in (
            cls.output_sub,
            cls.state_sub,
            cls.cancel_sub,
            cls.gps_pub,
            cls.fod_pub,
            cls.odom_pub,
            cls.visual_state_pub,
            cls.pause_service,
            cls.visual_service,
        ):
            item.shutdown() if hasattr(item, "shutdown") else item.unregister()

    @classmethod
    def _output_cb(cls, msg):
        with cls.lock:
            cls.latest_output = msg

    @classmethod
    def _state_cb(cls, msg):
        with cls.lock:
            cls.latest_state = msg.data

    @classmethod
    def _cancel_cb(cls, _msg):
        with cls.lock:
            cls.cancel_count += 1

    @classmethod
    def _pause_cb(cls, request):
        with cls.lock:
            cls.pause_requests.append(bool(request.data))
        return SetBoolResponse(success=True, message="fake GPS pause accepted")

    @classmethod
    def _visual_cb(cls, request):
        with cls.lock:
            cls.visual_requests.append(bool(request.data))
        if not request.data:
            cls.visual_state_pub.publish(String(data="DISABLED"))
        return SetBoolResponse(success=True, message="fake visual request accepted")

    @classmethod
    def _publish_inputs(cls, _event):
        gps = Twist()
        gps.linear.x = 1.0
        gps.angular.z = 0.1
        cls.gps_pub.publish(gps)

        fod = Twist()
        fod.linear.x = 0.2
        fod.angular.z = -0.02
        cls.fod_pub.publish(fod)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        cls.odom_pub.publish(odom)

    @staticmethod
    def _wait_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.02)
        raise AssertionError("condition did not become true within {:.1f}s".format(timeout))

    def test_abort_stays_stopped_and_complete_resumes_gps(self):
        response = self.mode_proxy(True)
        self.assertTrue(response.success, response.message)
        self._wait_until(lambda: self.latest_state == "FOD_ACTIVE", 2.0)
        self._wait_until(
            lambda: abs(self.latest_output.linear.x - 0.2) < 1e-6,
            2.0,
        )
        with self.lock:
            self.assertEqual(self.pause_requests, [True])
            self.assertEqual(self.visual_requests, [True])
            self.assertGreaterEqual(self.cancel_count, 1)

        self.visual_state_pub.publish(String(data="ABORT"))
        self._wait_until(lambda: self.latest_state == "FOD_ABORTED", 2.0)
        self._wait_until(
            lambda: abs(self.latest_output.linear.x) < 1e-9,
            2.0,
        )
        rospy.sleep(0.3)
        with self.lock:
            self.assertEqual(self.pause_requests, [True])

        response = self.mode_proxy(False)
        self.assertTrue(response.success, response.message)
        self._wait_until(lambda: self.latest_state == "GPS_ACTIVE", 2.0)
        self._wait_until(
            lambda: abs(self.latest_output.linear.x - 1.0) < 1e-6,
            2.0,
        )
        with self.lock:
            self.assertEqual(self.pause_requests[-1], False)
            self.assertEqual(self.visual_requests[-1], False)

        response = self.mode_proxy(True)
        self.assertTrue(response.success, response.message)
        self._wait_until(lambda: self.latest_state == "FOD_ACTIVE", 2.0)
        self.visual_state_pub.publish(String(data="COMPLETE"))
        self._wait_until(lambda: self.latest_state == "GPS_ACTIVE", 4.0)
        self._wait_until(
            lambda: abs(self.latest_output.linear.x - 1.0) < 1e-6,
            2.0,
        )
        with self.lock:
            self.assertEqual(self.pause_requests[-2:], [True, False])
            self.assertEqual(self.visual_requests[-2:], [True, False])


if __name__ == "__main__":
    rospy.init_node("fake_mode_world", anonymous=False)
    rostest.rosrun(
        "autolabor_fod_control",
        "fod_navigation_mode_integration",
        FodNavigationModeRosTest,
    )
