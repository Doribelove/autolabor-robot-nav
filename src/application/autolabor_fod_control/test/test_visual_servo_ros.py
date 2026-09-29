#!/usr/bin/env python3
"""Isolated ROS graph test for the complete visual recovery mode."""

import json
import math
import threading
import time
import unittest
from collections import deque

import rospy
import rostest
from autolabor_canbus_driver.msg import (
    CanBusMessage,
    ChassisParameter,
    ChassisStatusInfo,
)
from autolabor_canbus_driver.srv import (
    CanBusService,
    CanBusServiceResponse,
    ChassisParameterServer,
    ChassisParameterServerResponse,
)
from autolabor_fod_msgs.msg import FodDetection, FodDetectionArray
from geometry_msgs.msg import Point32, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, RegionOfInterest
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool

from autolabor_fod_control.visual_servo import local_displacement


MODEL_SHA = "7bf99d4c61343e8cdb37289f2eece6cf18342b508f9b7f80723592edce398500"
WIDTH = 1280
HEIGHT = 1024
CX = 620.043
WHEELBASE = 0.65


class FakeWorldIntegrationTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.travel_m = 0.0
        self.command = Twist()
        self.command_history = deque(maxlen=100)
        self.actual_linear_x = 0.0
        self.simulate_chassis_inertia = (
            self._testMethodName in (
                "test_continuous_eighty_cm_with_chassis_inertia",
                "test_early_loss_and_returning_boxes_complete_one_crossing",
            )
        )
        self.first_zero_forward_m = None
        self.early_loss_case = (
            self._testMethodName == "test_early_loss_and_returning_boxes_complete_one_crossing"
        )
        self.loss_at_travel_m = 0.15 if self.early_loss_case else 0.50
        self.first_loss_stamp = None
        self.selected_target_ids = set()
        self.last_visible_fraction = None
        self.intermediate_zero_commands = []
        self.minimum_angular_command = 0.0
        self.maximum_angular_command = 0.0
        self.maximum_linear_command = 0.0
        self.loss_pose = None
        self.loss_odom_release_monotonic = None
        self.pending_odom = []
        self.detection_frame_count = 0
        self.saw_sync_wait = False
        self.latest_reason = ""
        self.freeze_wheel_after_loss = (
            self._testMethodName == "test_stale_wheel_feedback_cannot_authorize_blind_motion"
        )
        self.freeze_odom_after_loss = (
            self._testMethodName == "test_stale_odom_feedback_cannot_authorize_blind_motion"
        )
        self.inject_transient_invalid_wheel = (
            self._testMethodName == "test_transient_invalid_wheel_feedback_is_latched"
        )
        self.inject_transient_chassis_emergency = (
            self._testMethodName == "test_transient_chassis_emergency_is_latched"
        )
        self.inject_transient_raw_can_fault = (
            self._testMethodName == "test_transient_raw_can_fault_is_latched"
        )
        self.inject_transient_steer_center_bias = (
            self._testMethodName
            == "test_transient_steer_center_bias_message_is_latched"
        )
        self.transient_fault_injected = False
        self.states = []

        self.detection_pub = rospy.Publisher(
            "/fod/detections", FodDetectionArray, queue_size=5
        )
        self.camera_pub = rospy.Publisher(
            "/fod_camera/camera_info", CameraInfo, queue_size=5
        )
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.wheel_pub = rospy.Publisher(
            "/m2_driver/wheel_angle", Float64, queue_size=10
        )
        self.chassis_pub = rospy.Publisher(
            "/m2_driver/chassis_info", ChassisStatusInfo, queue_size=5
        )
        self.timeout_pub = rospy.Publisher(
            "/m2_driver/control_timeout", Bool, queue_size=5
        )
        self.canbus_pub = rospy.Publisher("/canbus_msg", CanBusMessage, queue_size=20)
        self.cmd_sub = rospy.Subscriber(
            "/cmd_vel", Twist, self._command_cb, queue_size=20
        )
        self.state_sub = rospy.Subscriber(
            "/fod_visual_servo/state", String, self._state_cb, queue_size=20
        )
        self.status_sub = rospy.Subscriber(
            "/fod_visual_servo/status", String, self._status_cb, queue_size=20
        )
        self.canbus_server = rospy.Service(
            "/canbus_server", CanBusService, self._canbus_service_cb
        )
        self.parameter_server = rospy.Service(
            "/m2_driver/chassis_parameter",
            ChassisParameterServer,
            self._parameter_service_cb,
        )
        self.last_tick = time.monotonic()
        self.timer = rospy.Timer(rospy.Duration(0.04), self._publish_world)

    def tearDown(self):
        try:
            rospy.ServiceProxy("/fod_visual_servo/set_enabled", SetBool)(False)
        except (rospy.ROSException, rospy.ServiceException):
            pass
        self.timer.shutdown()
        for subscriber in (self.cmd_sub, self.state_sub, self.status_sub):
            subscriber.unregister()
        for publisher in (
            self.detection_pub,
            self.camera_pub,
            self.odom_pub,
            self.wheel_pub,
            self.chassis_pub,
            self.timeout_pub,
            self.canbus_pub,
        ):
            publisher.unregister()
        self.canbus_server.shutdown("integration test teardown")
        self.parameter_server.shutdown("integration test teardown")

    def _command_cb(self, msg):
        with self.lock:
            self.command = msg
            self.command_history.append((time.monotonic(), msg))
            if self.loss_pose is not None:
                forward, _ = local_displacement(
                    *self.loss_pose, self.x, self.y)
                if abs(msg.linear.x) < 1e-6 and self.first_zero_forward_m is None:
                    self.first_zero_forward_m = forward
                if forward < 0.795 and abs(msg.linear.x) < 1e-6:
                    self.intermediate_zero_commands.append(forward)
            self.minimum_angular_command = min(
                self.minimum_angular_command, float(msg.angular.z)
            )
            self.maximum_angular_command = max(
                self.maximum_angular_command, float(msg.angular.z)
            )
            self.maximum_linear_command = max(
                self.maximum_linear_command, float(msg.linear.x)
            )

    def _state_cb(self, msg):
        with self.lock:
            if not self.states or self.states[-1] != msg.data:
                self.states.append(msg.data)

    def _status_cb(self, msg):
        try:
            status = json.loads(msg.data)
            reason = str(status.get("reason", ""))
        except (TypeError, ValueError):
            return
        with self.lock:
            self.latest_reason = reason
            if status.get("target_locked"):
                self.selected_target_ids.add(status["target_track_id"])
            if status.get("target_visible"):
                self.last_visible_fraction = status["vertical_fraction"]
            if "waiting for synchronized odometry" in reason:
                self.saw_sync_wait = True

    def _canbus_service_cb(self, request):
        with self.lock:
            travel = self.travel_m
        if (
            self.inject_transient_steer_center_bias
            and not self.transient_fault_injected
            and travel > 0.05
        ):
            # Create a true one-shot publisher only after _check_graph has run
            # and while this service keeps the control tick blocked.  Unregister
            # it before returning so a periodic graph poll cannot be the reason
            # for ABORT; the message-generation latch itself must catch it.
            self.transient_fault_injected = True
            publisher = rospy.Publisher(
                "/m2_driver/steer_center_bias", Float64, queue_size=1
            )
            deadline = time.monotonic() + 1.0
            while (
                publisher.get_num_connections() < 1
                and time.monotonic() < deadline
                and not rospy.is_shutdown()
            ):
                rospy.sleep(0.01)
            publisher.publish(Float64(data=-0.4))
            rospy.sleep(0.12)
            publisher.unregister()
        if (
            self.inject_transient_chassis_emergency
            and not self.transient_fault_injected
            and travel > 0.05
        ):
            # The controller's control tick is blocked in this service call,
            # while its subscriber threads remain live.  Publish unsafe then
            # safe and leave enough time for both callbacks to run before the
            # service returns.  The subsequent abort must therefore come from
            # the generation latch, not from merely sampling the unsafe value.
            self.transient_fault_injected = True
            emergency = ChassisStatusInfo()
            emergency.hard_emergency = True
            self.chassis_pub.publish(emergency)
            self.chassis_pub.publish(ChassisStatusInfo())
            rospy.sleep(0.12)
        for query in request.requests:
            if (
                self.inject_transient_raw_can_fault
                and not self.transient_fault_injected
                and travel > 0.05
                and query.msg_type == 0x17
            ):
                unsafe = CanBusMessage()
                unsafe.node_type = query.node_type
                unsafe.node_seq = query.node_seq
                unsafe.msg_type = query.msg_type
                unsafe.payload = [1]
                self.canbus_pub.publish(unsafe)
                self.transient_fault_injected = True
            response = CanBusMessage()
            response.node_type = query.node_type
            response.node_seq = query.node_seq
            response.msg_type = query.msg_type
            response.payload = [0x10] if query.msg_type == 0x80 else [0]
            self.canbus_pub.publish(response)
        return CanBusServiceResponse()

    @staticmethod
    def _parameter_service_cb(_request):
        parameters = ChassisParameter()
        parameters.max_speed = 1.60
        parameters.max_steer = 0.50
        parameters.robot_width = 0.80
        parameters.robot_length = WHEELBASE
        parameters.wheel_radius = 0.16
        return ChassisParameterServerResponse(
            success=True,
            parameters=parameters,
            message="fake parameters",
        )

    def _publish_world(self, _event):
        now_monotonic = time.monotonic()
        dt = min(0.10, max(0.0, now_monotonic - self.last_tick))
        self.last_tick = now_monotonic
        with self.lock:
            commanded_linear_x = float(self.command.linear.x)
            commanded_angular_z = float(self.command.angular.z)
            if self.simulate_chassis_inertia:
                delayed = [command for stamp, command in self.command_history
                           if stamp <= now_monotonic - 0.12]
                if delayed:
                    commanded_linear_x = delayed[-1].linear.x
                    commanded_angular_z = delayed[-1].angular.z
                else:
                    commanded_linear_x = commanded_angular_z = 0.0
            # Model both physical M2 observations: 0.12 m/s can turn the front
            # wheel yet fails to produce sustained odometry, while 0.20 m/s
            # has already started the same chassis in straight calibration.
            if 0.0 < abs(commanded_linear_x) < 0.20:
                linear_x = 0.0
                angular_z = 0.0
            else:
                linear_x = commanded_linear_x
                angular_z = commanded_angular_z
            if self.simulate_chassis_inertia:
                # Measured chassis motion cannot change instantaneously with
                # a Twist: include 120 ms response and 1 m/s^2 acceleration /
                # braking, rather than hiding the real stop-and-restart bug.
                target_linear_x = linear_x
                self.actual_linear_x += max(-dt, min(
                    dt, target_linear_x - self.actual_linear_x))
                linear_x = self.actual_linear_x
                angular_z = (commanded_angular_z * linear_x / commanded_linear_x
                             if abs(commanded_linear_x) > 1e-5 else 0.0)
            self.yaw += angular_z * dt
            self.x += linear_x * math.cos(self.yaw) * dt
            self.y += linear_x * math.sin(self.yaw) * dt
            self.travel_m += max(0.0, linear_x) * dt
            x = self.x
            y = self.y
            yaw = self.yaw
            travel = self.travel_m

        stamp = rospy.Time.now()
        self._publish_camera(stamp)
        self._publish_detection(stamp, travel, x, y, yaw)
        visible = travel < self.loss_at_travel_m
        publish_odom = True
        with self.lock:
            if not visible and self.loss_odom_release_monotonic is None:
                # Keep the first-loss source stamp ahead of odometry long
                # enough for APPROACH -> BLIND_ADVANCE to happen first.
                # This exercises the bounded pending interpolation path that a
                # same-timer detector/odom simulation would otherwise miss.
                self.loss_odom_release_monotonic = now_monotonic + 0.24
            if (
                self.loss_odom_release_monotonic is not None
                and now_monotonic < self.loss_odom_release_monotonic
            ):
                publish_odom = False
        if self.freeze_odom_after_loss and not visible:
            publish_odom = False
        if publish_odom:
            # Model delayed delivery with original source timestamps, rather
            # than silently dropping every sample across the braking event.
            # Otherwise interpolation of the 0.24 s gap invents a different
            # loss pose (centimetres at 0.6 m/s) and invalidates the assertion
            # against this world's exact first-loss position.
            for sample in self.pending_odom:
                self._publish_odometry(*sample)
            self.pending_odom.clear()
            self._publish_odometry(stamp, x, y, yaw, linear_x, angular_z)
        elif not self.freeze_odom_after_loss:
            self.pending_odom.append((stamp, x, y, yaw, linear_x, angular_z))
        wheel_angle = (
            math.atan(angular_z * WHEELBASE / linear_x)
            if abs(linear_x) > 1e-5
            else 0.0
        )
        if not (self.freeze_wheel_after_loss and not visible):
            if (
                self.inject_transient_invalid_wheel
                and not self.transient_fault_injected
                and travel > 0.05
            ):
                self.wheel_pub.publish(Float64(data=float("nan")))
                self.transient_fault_injected = True
            self.wheel_pub.publish(Float64(data=wheel_angle))
        self.chassis_pub.publish(ChassisStatusInfo())
        self.timeout_pub.publish(Bool(data=False))

    def _publish_camera(self, stamp):
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = "fod_camera_optical_frame"
        message.width = WIDTH
        message.height = HEIGHT
        message.K = [800.0, 0.0, CX, 0.0, 800.0, 528.0, 0.0, 0.0, 1.0]
        message.P = [800.0, 0.0, CX, 0.0, 0.0, 800.0, 528.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_pub.publish(message)

    def _publish_detection(self, stamp, travel, x, y, yaw):
        message = FodDetectionArray()
        message.header.stamp = stamp
        message.header.frame_id = "fod_camera_optical_frame"
        message.image_width = WIDTH
        message.image_height = HEIGHT
        message.model_name = "best.pt"
        message.model_sha256 = MODEL_SHA
        message.model_task = "detect"
        message.inference_ms = 20.0
        self.detection_frame_count += 1
        visible = travel < self.loss_at_travel_m
        if not visible and self.first_loss_stamp is None:
            self.first_loss_stamp = stamp.to_sec()
        # Reproduce the real abort: loss while still near mid-image, followed
        # by the same box blinking back after loss confirmation, with two
        # unrelated objects still visible. Finish once, without switching IDs.
        returning_box = (self.early_loss_case and self.first_loss_stamp is not None
                         and stamp.to_sec() - self.first_loss_stamp > 0.40
                         and self.detection_frame_count % 3 == 0)
        off_center_case = (
            self._testMethodName
            == "test_off_center_target_is_acquired_and_steered_toward"
        )
        # Intermittent loss during off-centre steering must complete the same
        # one-shot advance; returning boxes must not abort or extend it.
        simulate_dropout = off_center_case and travel < 0.20 and (
            8 <= self.detection_frame_count <= 19
            or (
                self.detection_frame_count > 19
                and self.detection_frame_count % 8 == 0
            )
        )
        if (visible and not simulate_dropout) or returning_box:
            q = 0.50 + 0.42 * min(1.0, travel / 0.35)
            if self.early_loss_case:
                q = 0.50 + min(0.10, travel * 0.2)
            if off_center_case:
                initial_offset = -0.58 * 0.5 * WIDTH
            else:
                initial_offset = 80.0
            u = CX + initial_offset * max(0.0, 1.0 - travel / 0.25)
            v = q * (HEIGHT - 1)
            item = FodDetection()
            item.class_id = 0
            item.class_name = "Metal"
            item.confidence = 0.90
            item.track_id = 1
            item.depth_valid = True
            item.depth_m = 1.0
            item.range_m = 1.1
            item.bbox = RegionOfInterest(
                x_offset=int(round(u - 50.0)),
                y_offset=int(round(v - 80.0)),
                width=100,
                height=80,
                do_rectify=False,
            )
            item.anchor_px = Point32(x=float(u), y=float(v), z=0.0)
            message.detections = [item]
        else:
            with self.lock:
                if self.loss_pose is None:
                    self.loss_pose = (x, y, yaw)
        if self.early_loss_case:
            for identity, u in ((2, CX - 200), (3, CX + 200)):
                item = FodDetection()
                item.class_id = 0
                item.class_name = "Metal"
                item.confidence = .95
                item.track_id = identity
                item.depth_valid = True
                item.depth_m = 2.0
                item.range_m = 2.1
                item.bbox = RegionOfInterest(
                    x_offset=int(u - 30), y_offset=430, width=60, height=80)
                item.anchor_px = Point32(x=u, y=510.0)
                message.detections.append(item)
        self.detection_pub.publish(message)

    def _publish_odometry(self, stamp, x, y, yaw, linear_x, angular_z):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * yaw)
        message.twist.twist.linear.x = linear_x
        message.twist.twist.angular.z = angular_z
        self.odom_pub.publish(message)

    def _enable_controller(self):
        rospy.wait_for_service("/fod_visual_servo/set_enabled", timeout=8.0)
        # Allow every publisher/subscriber and both services to enter the ROS
        # master graph before asking the fail-closed PRECHECK to run.
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.detection_pub.get_num_connections() > 0:
                break
            rospy.sleep(0.05)
        rospy.sleep(0.8)
        # Each test recreates the fake VCU. Seed its healthy initial state so
        # a prior test's aged round-robin replies cannot cause an unrelated
        # PRECHECK/APPROACH timeout before this test injects its intended fault.
        for msg_type in (0x17, 0x18, 0x19, 0x80):
            response = CanBusMessage()
            response.node_type = 0x10
            response.node_seq = 0x00
            response.msg_type = msg_type
            response.payload = [0x10] if msg_type == 0x80 else [0]
            self.canbus_pub.publish(response)
        rospy.sleep(0.1)
        response = rospy.ServiceProxy(
            "/fod_visual_servo/set_enabled", SetBool
        )(True)
        self.assertTrue(response.success, response.message)

    def _wait_for_terminal_state(self, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        state = ""
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self.lock:
                state = self.states[-1] if self.states else ""
            if state in ("COMPLETE", "ABORT"):
                break
            rospy.sleep(0.05)
        if state in ("COMPLETE", "ABORT"):
            rospy.sleep(0.10)
        return state

    def test_complete_recovery_loop(self):
        self._enable_controller()

        self._wait_for_terminal_state(25.0)

        with self.lock:
            states = list(self.states)
            command = self.command
            loss_pose = self.loss_pose
            end_pose = (self.x, self.y)
            minimum_angular = self.minimum_angular_command
            maximum_linear = self.maximum_linear_command
            saw_sync_wait = self.saw_sync_wait
        self.assertTrue(states, "no controller states were published")
        self.assertEqual(states[-1], "COMPLETE", "state trace: %r" % states)
        self.assertIn("APPROACH", states)
        self.assertNotIn("REACQUIRE", states)
        self.assertIn("BLIND_ADVANCE", states)
        self.assertNotIn("STEER_SETTLE", states)
        self.assertIn("BLIND_ADVANCE", states)
        self.assertIn("FINAL_STOP", states)
        self.assertTrue(saw_sync_wait, "delayed-odom synchronization path was not exercised")
        self.assertLess(minimum_angular, -1e-4)
        self.assertLessEqual(maximum_linear, 0.60 + 1e-6)
        self.assertGreaterEqual(maximum_linear, 0.59)
        self.assertAlmostEqual(command.linear.x, 0.0, places=6)
        self.assertAlmostEqual(command.angular.z, 0.0, places=6)
        self.assertIsNotNone(loss_pose)
        forward, lateral = local_displacement(
            loss_pose[0],
            loss_pose[1],
            loss_pose[2],
            end_pose[0],
            end_pose[1],
        )
        self.assertGreaterEqual(forward, 0.795)
        self.assertLessEqual(forward, 0.90)
        self.assertLessEqual(abs(lateral), 0.08)
        self.assertFalse(self.intermediate_zero_commands,
                         "unexpected stop before completing 0.80 m")

    def test_continuous_eighty_cm_with_chassis_inertia(self):
        self._enable_controller()
        terminal = self._wait_for_terminal_state(25.0)
        with self.lock:
            states = list(self.states)
            reason = self.latest_reason
            loss_pose = self.loss_pose
            end_pose = (self.x, self.y)
            intermediate_stops = list(self.intermediate_zero_commands)
            command = self.command
        self.assertEqual(terminal, "COMPLETE", "%r: %s" % (states, reason))
        self.assertNotIn("STEER_SETTLE", states)
        self.assertFalse(intermediate_stops, intermediate_stops)
        self.assertIn("BLIND_ADVANCE", states)
        forward, lateral = local_displacement(*loss_pose, *end_pose)
        self.assertGreaterEqual(forward, 0.795)
        # The command stops at 0.80 m; this model has 120 ms transport delay
        # plus finite deceleration. Check both the trigger and actual rest,
        # rather than calling normal post-command braking a visual abort.
        self.assertGreaterEqual(self.first_zero_forward_m, .795)
        self.assertLessEqual(self.first_zero_forward_m, .835)
        self.assertLessEqual(forward - self.first_zero_forward_m, .065)
        self.assertLessEqual(abs(lateral), 0.08)
        self.assertEqual(command.linear.x, 0.0)

    def test_off_center_target_is_acquired_and_steered_toward(self):
        self._enable_controller()
        terminal = self._wait_for_terminal_state(25.0)

        with self.lock:
            states = list(self.states)
            maximum_angular = self.maximum_angular_command
            maximum_linear = self.maximum_linear_command
        self.assertEqual(terminal, "COMPLETE", "state trace: %r" % states)
        self.assertIn("APPROACH", states)
        self.assertNotIn("REACQUIRE", states)
        self.assertGreater(maximum_linear, 0.0)
        # An image-left target must produce positive ROS yaw while advancing.
        self.assertGreater(maximum_angular, 1e-4)

    def test_early_loss_and_returning_boxes_complete_one_crossing(self):
        self._enable_controller()
        terminal = self._wait_for_terminal_state(25.0)
        with self.lock:
            states = list(self.states)
            reason = self.latest_reason
            forward, lateral = local_displacement(*self.loss_pose, self.x, self.y)
            stopped = list(self.intermediate_zero_commands)
            ids = set(self.selected_target_ids)
            last_visible = self.last_visible_fraction
        self.assertEqual(terminal, "COMPLETE", "%r: %s" % (states, reason))
        self.assertEqual(ids, {1})
        self.assertLess(last_visible, .70)
        self.assertNotIn("EDGE_ARMED", states)
        self.assertNotIn("REACQUIRE", states)
        self.assertEqual(states.count("BLIND_ADVANCE"), 1)
        self.assertFalse(stopped, stopped)
        self.assertGreaterEqual(forward, .795)
        self.assertGreaterEqual(self.first_zero_forward_m, .795)
        self.assertLessEqual(self.first_zero_forward_m, .835)
        self.assertLessEqual(forward - self.first_zero_forward_m, .065)
        self.assertLessEqual(abs(lateral), .08)

    def test_stale_wheel_feedback_cannot_authorize_blind_motion(self):
        self._enable_controller()
        terminal = self._wait_for_terminal_state(18.0)

        with self.lock:
            states = list(self.states)
            command = self.command
            reason = self.latest_reason
        self.assertEqual(terminal, "ABORT", "state trace: %r" % states)
        self.assertIn("BLIND_ADVANCE", states)
        self.assertNotIn("COMPLETE", states)
        self.assertIn("wheel", reason.lower())
        self.assertAlmostEqual(command.linear.x, 0.0, places=6)
        self.assertAlmostEqual(command.angular.z, 0.0, places=6)

    def test_stale_odom_feedback_cannot_authorize_blind_motion(self):
        self._enable_controller()
        terminal = self._wait_for_terminal_state(18.0)

        with self.lock:
            states = list(self.states)
            command = self.command
            reason = self.latest_reason
        self.assertEqual(terminal, "ABORT", "state trace: %r" % states)
        self.assertIn("BLIND_ADVANCE", states)
        self.assertNotIn("COMPLETE", states)
        self.assertIn("odom", reason.lower())
        self.assertAlmostEqual(command.linear.x, 0.0, places=6)
        self.assertAlmostEqual(command.angular.z, 0.0, places=6)

    def test_transient_invalid_wheel_feedback_is_latched(self):
        self._enable_controller()
        terminal = self._wait_for_terminal_state(10.0)

        with self.lock:
            states = list(self.states)
            command = self.command
            reason = self.latest_reason
        self.assertEqual(terminal, "ABORT", "state trace: %r" % states)
        self.assertNotIn("BLIND_ADVANCE", states)
        self.assertIn("invalid wheel", reason.lower())
        self.assertAlmostEqual(command.linear.x, 0.0, places=6)
        self.assertAlmostEqual(command.angular.z, 0.0, places=6)

    def test_transient_chassis_emergency_is_latched(self):
        self._enable_controller()
        terminal = self._wait_for_terminal_state(10.0)

        with self.lock:
            states = list(self.states)
            command = self.command
            reason = self.latest_reason
        self.assertEqual(terminal, "ABORT", "state trace: %r" % states)
        self.assertNotIn("BLIND_ADVANCE", states)
        self.assertIn("chassis emergency", reason.lower())
        self.assertIn("was observed", reason.lower())
        self.assertAlmostEqual(command.linear.x, 0.0, places=6)
        self.assertAlmostEqual(command.angular.z, 0.0, places=6)

    def test_transient_raw_can_fault_is_latched(self):
        self._enable_controller()
        terminal = self._wait_for_terminal_state(10.0)

        with self.lock:
            states = list(self.states)
            command = self.command
            reason = self.latest_reason
        self.assertEqual(terminal, "ABORT", "state trace: %r" % states)
        self.assertNotIn("BLIND_ADVANCE", states)
        self.assertIn("raw can fault", reason.lower())
        self.assertAlmostEqual(command.linear.x, 0.0, places=6)
        self.assertAlmostEqual(command.angular.z, 0.0, places=6)

    def test_transient_steer_center_bias_message_is_latched(self):
        self._enable_controller()
        terminal = self._wait_for_terminal_state(10.0)

        with self.lock:
            states = list(self.states)
            command = self.command
            reason = self.latest_reason
        self.assertEqual(terminal, "ABORT", "state trace: %r" % states)
        self.assertIn("APPROACH", states)
        self.assertNotIn("BLIND_ADVANCE", states)
        self.assertIn("bypass-control message", reason.lower())
        self.assertIn("steer_center_bias", reason)
        self.assertAlmostEqual(command.linear.x, 0.0, places=6)
        self.assertAlmostEqual(command.angular.z, 0.0, places=6)


if __name__ == "__main__":
    rospy.init_node("fake_world", anonymous=False)
    rostest.rosrun(
        "autolabor_fod_control",
        "visual_servo_integration",
        FakeWorldIntegrationTest,
    )
