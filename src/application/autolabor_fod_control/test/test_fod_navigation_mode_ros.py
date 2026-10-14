#!/usr/bin/env python3
"""ROS graph test for serialized GPS/FOD ownership and completion recovery."""

import json
from pathlib import Path
import sys
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
from autolabor_fod_msgs.msg import FodDetection, FodDetectionArray

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "src" / "SweepDeviceControl"))
from test_sweep_test import DeviceServer


class FodNavigationModeRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.lock = threading.RLock()
        cls.latest_output = Twist()
        cls.latest_state = ""
        cls.latest_status = {}
        cls.output_history = []
        cls.unexpected_faults = []
        cls.gps_x = 0.0
        cls.final_x = 20.0
        cls.route_active = True
        cls.route_state = "ACTIVE_FINAL"
        cls.publish_detections = True
        cls.target_visible = False
        cls.target_class = "Metal"
        cls.target_confidence = 0.9
        cls.target_depth_valid = True
        cls.target_age_sec = 0.0
        cls.drop_target_when_paused = False
        cls.sweep_states_at_visual_enable = []
        cls.pause_requests = []
        cls.visual_requests = []
        cls.cancel_count = 0
        cls.retain_gps_route = rospy.get_param("~retain_gps_route", True)
        cls.sweeper = DeviceServer(port=int(rospy.get_param("~sweep_port")))

        cls.output_sub = rospy.Subscriber(
            "/test/cmd_vel", Twist, cls._output_cb, queue_size=20
        )
        cls.state_sub = rospy.Subscriber(
            "/fod_navigation_mode_test/state",
            String,
            cls._state_cb,
            queue_size=20,
        )
        cls.status_sub = rospy.Subscriber(
            "/fod_navigation_mode_test/status", String, cls._status_cb, queue_size=20)
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
        cls.gps_pose_pub = rospy.Publisher("/test/gps/odom", Odometry, queue_size=1)
        cls.route_pub = rospy.Publisher("/test/gps/route_status", String, queue_size=1, latch=True)
        cls.detections_pub = rospy.Publisher("/test/detections", FodDetectionArray, queue_size=1)
        cls.visual_state_pub = rospy.Publisher(
            "/test/fod/state", String, queue_size=1, latch=True
        )
        # The indoor test deliberately has no GNSS pause service at all.
        cls.pause_service = (
            rospy.Service("/test/gps/set_paused", SetBool, cls._pause_cb)
            if cls.retain_gps_route else None
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
            cls.status_sub,
            cls.cancel_sub,
            cls.gps_pub,
            cls.fod_pub,
            cls.odom_pub,
            cls.gps_pose_pub,
            cls.route_pub,
            cls.detections_pub,
            cls.visual_state_pub,
            cls.pause_service,
            cls.visual_service,
        ):
            if item is None:
                continue
            item.shutdown() if hasattr(item, "shutdown") else item.unregister()
        cls.sweeper.__exit__()

    @classmethod
    def _output_cb(cls, msg):
        with cls.lock:
            cls.latest_output = msg
            cls.output_history.append(msg.linear.x)

    @classmethod
    def _state_cb(cls, msg):
        with cls.lock:
            cls.latest_state = msg.data
            if msg.data == "FAULT_STOP":
                cls.unexpected_faults.append(msg.data)

    @classmethod
    def _status_cb(cls, msg):
        with cls.lock:
            cls.latest_status = json.loads(msg.data)

    @classmethod
    def _cancel_cb(cls, _msg):
        with cls.lock:
            cls.cancel_count += 1

    @classmethod
    def _pause_cb(cls, request):
        with cls.lock:
            cls.pause_requests.append(bool(request.data))
            if request.data and cls.drop_target_when_paused:
                cls.target_visible = False
                cls.drop_target_when_paused = False
        return SetBoolResponse(success=True, message="fake GPS pause accepted")

    @classmethod
    def _visual_cb(cls, request):
        with cls.lock:
            cls.visual_requests.append(bool(request.data))
            if request.data:
                cls.sweep_states_at_visual_enable.append(cls.sweeper.state)
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
        fod.linear.x = 0.6
        fod.angular.z = -0.02
        cls.fod_pub.publish(fod)

        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        cls.odom_pub.publish(odom)

        with cls.lock:
            gps_x, final_x = cls.gps_x, cls.final_x
            visible, name = cls.target_visible, cls.target_class
            confidence, age = cls.target_confidence, cls.target_age_sec
            depth_valid = cls.target_depth_valid
            route_active, route_state = cls.route_active, cls.route_state
            publish_detections = cls.publish_detections
        cls.route_pub.publish(String(data=json.dumps(dict(
            active=route_active, paused=False, state=route_state, frame_id="camera_init",
            final_x=final_x, final_y=0.0, subgoal_x=gps_x + 0.1, subgoal_y=0.0))))
        gps_pose = Odometry()
        gps_pose.header.stamp = rospy.Time.now()
        gps_pose.header.frame_id = "camera_init"
        gps_pose.pose.pose.position.x = gps_x
        cls.gps_pose_pub.publish(gps_pose)
        detections = FodDetectionArray()
        detections.header.stamp = rospy.Time.now() - rospy.Duration(age)
        detections.image_width = 1280
        detections.image_height = 720
        if visible:
            item = FodDetection()
            item.class_name = name
            item.confidence = confidence
            item.track_id = 1
            item.depth_valid = depth_valid
            item.depth_m = 2.0 if depth_valid else 0.0
            item.range_m = 2.1 if depth_valid else 0.0
            item.bbox.x_offset = 590
            item.bbox.y_offset = 280
            item.bbox.width = 100
            item.bbox.height = 80
            item.anchor_px.x = 640
            item.anchor_px.y = 360
            detections.detections = [item]
        if publish_detections:
            cls.detections_pub.publish(detections)

    @staticmethod
    def _wait_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if predicate():
                return
            rospy.sleep(0.02)
        raise AssertionError("condition did not become true within {:.1f}s".format(timeout))

    def _conditions(self, **changes):
        with self.lock:
            for key, value in changes.items():
                setattr(type(self), key, value)
        rospy.sleep(0.25)

    def _assert_gps_undisturbed(self, sweep_switches=0):
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            with self.lock:
                self.assertEqual(self.latest_state, "GPS_ACTIVE")
                self.assertAlmostEqual(self.latest_output.linear.x, 1.0)
                self.assertEqual(self.pause_requests, [])
                self.assertEqual(self.visual_requests, [])
                self.assertEqual(self.cancel_count, 0)
                self.assertEqual(self.sweeper.switches, sweep_switches)
                self.assertTrue(all(abs(x - 1.0) < 1e-6 for x in self.output_history),
                                "GPS output was interrupted while only requesting sweeping")
            rospy.sleep(0.02)

    def test_proximity_and_detection_then_recovery_lifecycle(self):
        with self.lock:
            type(self).output_history = []
        switches_before_visual = 0
        cycles_before_visual = 0
        if self.retain_gps_route:
            # A visible nearby target alone never takes control without arming.
            self._conditions(gps_x=10.1, target_visible=True)
            self._assert_gps_undisturbed()
            # Nearby rolling subgoal must not substitute for the final goal.
            self._conditions(gps_x=0.0)
            response = self.mode_proxy(True)
            self.assertTrue(response.success, response.message)
            self._wait_until(lambda: self.latest_status.get("recovery_armed"), 2.0)
            self._assert_gps_undisturbed()
            # The requested boundary is strictly less than 10 m, even with FOD.
            self._conditions(gps_x=10.0)
            self._assert_gps_undisturbed()
            self.assertEqual(self.sweeper.hydraulic, [])
            # With no camera stream at all, enter the area and start sweeping
            # while continuing every GPS command without pause/cancel/zero.
            self._conditions(target_visible=False)
            self._conditions(publish_detections=False)
            self._conditions(gps_x=10.1)
            self._wait_until(lambda: self.latest_status.get("recovery_sweep_ready"), 5.0)
            self.assertEqual(self.sweeper.state, 1)
            self.assertEqual([(a, d) for a, d, _ in self.sweeper.hydraulic], [("extend", 3)])
            self._assert_gps_undisturbed(sweep_switches=1)
            self._conditions(publish_detections=True)
            for change in (dict(target_visible=True, target_depth_valid=False),
                           dict(target_depth_valid=True, target_class="Stone"),
                           dict(target_class="Metal", target_confidence=0.1),
                           dict(target_confidence=0.9, target_visible=False)):
                self._conditions(**change)
                self._assert_gps_undisturbed(sweep_switches=1)
            # GPS-only collection completes with no visual takeover and closes
            # the automatically started sweeper at the normal route end.
            self._conditions(route_active=False, route_state="COMPLETE")
            self._wait_until(lambda: self.sweeper.state == 0, 5.0)
            self._assert_hydraulic_cycles(1)
            self.assertFalse(self.latest_status.get("recovery_armed"))
            self._assert_gps_undisturbed(sweep_switches=2)
            # Use a newly accepted final target rather than the arrived one.
            self._conditions(route_active=True, route_state="ACTIVE_FINAL", final_x=30.0)
            self.assertTrue(self.mode_proxy(True).success)
            self._assert_gps_undisturbed(sweep_switches=2)
            self._conditions(target_visible=False, target_age_sec=0.0)
            self._conditions(gps_x=20.1)
            self._wait_until(lambda: self.latest_status.get("recovery_sweep_ready"), 5.0)
            self._assert_gps_undisturbed(sweep_switches=3)
            # Canceling closes our sweeper without pausing GPS navigation.
            self.assertTrue(self.mode_proxy(False).success)
            self.assertEqual(self.sweeper.state, 0)
            self._assert_hydraulic_cycles(2)
            self._conditions(target_visible=True)
            self._assert_gps_undisturbed(sweep_switches=4)
            switches_before_visual = 4
            cycles_before_visual = 2
            # Neither an old source timestamp nor a gap in detection messages
            # expires the last actual target. A new empty frame still clears it.
            self._conditions(target_age_sec=2.0)
            self._conditions(publish_detections=False)
            rospy.sleep(.6)
            self.assertTrue(self.latest_status.get("fod_target_available"))
        # Indoor mode retains immediate handoff without a GPS route.
        response = self.mode_proxy(True)
        self.assertTrue(response.success, response.message)
        self._wait_until(lambda: self.latest_state == "FOD_ACTIVE", 6.0)
        self._wait_until(
            lambda: abs(self.latest_output.linear.x - 0.6) < 1e-6, 2.0)
        self.assertEqual(self.sweeper.state, 1)
        self.assertEqual(self.sweep_states_at_visual_enable, [1])
        self._assert_hydraulic_cycles(cycles_before_visual, active=True)
        self.assertFalse(self.latest_status.get("recovery_armed"))
        # After handoff, loss/occlusion belongs to the original visual workflow.
        self._conditions(target_visible=False, target_age_sec=0.0, publish_detections=True)
        self.assertEqual(self.latest_state, "FOD_ACTIVE")

        self.visual_state_pub.publish(String(data="ABORT"))
        self._wait_until(lambda: self.latest_state == "FOD_ABORTED", 2.0)
        self._wait_until(lambda: abs(self.latest_output.linear.x) < 1e-9, 2.0)
        self._wait_until(lambda: self.sweeper.state == 0, 5.0)
        self.assertTrue(self.mode_proxy(False).success)
        self._assert_hydraulic_cycles(cycles_before_visual + 1)
        self._wait_until(lambda: self.latest_state == "GPS_ACTIVE", 3.0)
        self._wait_until(lambda: abs(self.latest_output.linear.x - 1.0) < 1e-6, 2.0)

        # COMPLETE consumes this one request and stays stopped even while
        # other targets and nonzero GPS/FOD commands continue to arrive.
        self._conditions(target_visible=True)
        response = self.mode_proxy(True)
        self.assertTrue(response.success, response.message)
        self._wait_until(lambda: self.latest_state == "FOD_ACTIVE", 6.0)
        self.visual_state_pub.publish(String(data="COMPLETE"))
        self._wait_until(lambda: self.latest_state == "FOD_COMPLETE_STOP", 3.0)
        self._wait_until(lambda: self.sweeper.state == 0, 5.0)
        pauses_at_complete = list(self.pause_requests)
        starts_at_complete = list(self.visual_requests)
        for _ in range(8):
            rospy.sleep(0.1)
            self.assertEqual(self.latest_state, "FOD_COMPLETE_STOP")
            self.assertAlmostEqual(self.latest_output.linear.x, 0.0)
            self.assertAlmostEqual(self.latest_output.angular.z, 0.0)
            self.assertFalse(self.latest_status.get("recovery_armed"))
            self.assertEqual(self.pause_requests, pauses_at_complete)
            self.assertEqual(self.visual_requests, starts_at_complete)
        self.assertTrue(self.mode_proxy(False).success)
        self._assert_hydraulic_cycles(cycles_before_visual + 2)
        self._wait_until(lambda: self.latest_state == "GPS_ACTIVE", 6.0)
        self._wait_until(
            lambda: abs(self.latest_output.linear.x - 1.0) < 1e-6,
            2.0,
        )
        with self.lock:
            self.assertEqual(self.pause_requests[-2:], [True, False] if self.retain_gps_route else [])
            self.assertEqual(self.visual_requests[-2:], [True, False])
        self.assertEqual(self.sweeper.state, 0)
        self._conditions(target_visible=True)
        rospy.sleep(0.5)
        self.assertEqual(self.latest_state, "GPS_ACTIVE")
        self.assertFalse(self.latest_status.get("recovery_armed"))
        self.assertEqual(self.sweeper.switches, switches_before_visual + 4)

        # The automatic cycle also owns a motor already switched on manually:
        # no redundant ON toggle, but completion stops it and retracts once.
        self.sweeper.state = 1
        self.assertTrue(self.mode_proxy(True).success)
        self._wait_until(lambda: self.latest_state == "FOD_ACTIVE", 6.0)
        self.visual_state_pub.publish(String(data="COMPLETE"))
        self._wait_until(lambda: self.latest_state == "FOD_COMPLETE_STOP", 3.0)
        self._wait_until(lambda: self.sweeper.state == 0, 5.0)
        self.assertEqual(self.sweeper.switches, switches_before_visual + 5)
        self.assertTrue(all(state == 1 for state in self.sweep_states_at_visual_enable))
        self.assertTrue(self.mode_proxy(False).success)
        self._assert_hydraulic_cycles(cycles_before_visual + 3)
        self._wait_until(lambda: self.latest_state == "GPS_ACTIVE", 3.0)

        if self.retain_gps_route:
            # Losing FOD during handoff resumes GPS with the automatically
            # started sweeper kept ON, rather than defeating GPS fallback.
            self.sweeper.state = 0
            previous_visual_starts = self.visual_requests.count(True)
            previous_pauses = len(self.pause_requests)
            self._conditions(drop_target_when_paused=True)
            self.assertTrue(self.mode_proxy(True).success)
            self._wait_until(lambda: len(self.pause_requests) > previous_pauses, 6.0)
            self._wait_until(lambda: self.latest_state == "GPS_ACTIVE" and
                             self.latest_status.get("recovery_armed"), 4.0)
            self._wait_until(lambda: abs(self.latest_output.linear.x - 1.0) < 1e-6, 2.0)
            self.assertEqual(self.visual_requests.count(True), previous_visual_starts)
            self.assertEqual(self.sweeper.state, 1)
            self.assertEqual(self.sweeper.switches, switches_before_visual + 6)
            self._assert_hydraulic_cycles(cycles_before_visual + 3, active=True)
            self.assertTrue(self.mode_proxy(False).success)
            self.assertEqual(self.sweeper.state, 0)
            self.assertEqual(self.sweeper.switches, switches_before_visual + 7)
            self._assert_hydraulic_cycles(cycles_before_visual + 4)

            # A sweeper that does not start reports its failure without
            # converting GPS fallback into a fault stop or retrying toggles.
            self.sweeper.behavior = "unchanged"
            self._conditions(target_visible=False)
            pauses = len(self.pause_requests)
            visual_starts = self.visual_requests.count(True)
            self.assertTrue(self.mode_proxy(True).success)
            self._wait_until(lambda: "sweeper start failed" in
                             self.latest_status.get("reason", ""), 10.0)
            self.assertEqual(self.latest_state, "GPS_ACTIVE")
            self.assertAlmostEqual(self.latest_output.linear.x, 1.0)
            self.assertEqual(len(self.pause_requests), pauses)
            self.assertEqual(self.visual_requests.count(True), visual_starts)
            switches = self.sweeper.switches
            rospy.sleep(0.5)
            self.assertEqual(self.sweeper.switches, switches)
            self.assertTrue(self.mode_proxy(False).success)
            self._assert_hydraulic_cycles(cycles_before_visual + 5)

        self.assertEqual(self.unexpected_faults, [], "unexpected command-graph fault during recovery")

    def _assert_hydraulic_cycles(self, completed, active=False):
        expected = [("extend", 3), ("retract", 3)] * completed
        if active:
            expected.append(("extend", 3))
        self._wait_until(lambda: len(self.sweeper.hydraulic) >= len(expected), 2.0)
        self.assertEqual([(a, d) for a, d, _ in self.sweeper.hydraulic], expected)


if __name__ == "__main__":
    rospy.init_node("fake_mode_world", anonymous=False)
    rostest.rosrun(
        "autolabor_fod_control",
        "fod_navigation_mode_integration",
        FodNavigationModeRosTest,
    )
