#!/usr/bin/env python3
"""End-to-end GPS/real visual controller/real mode manager/loopback sweeper tests.

All vehicle motion and sensors are simulated on an isolated rostest ROS master.
No real serial device, camera, GNSS, lidar or sweeper is opened by this test.
"""
import copy
import json
import math
from pathlib import Path
import random
import sys
import time
import unittest

import rospy
import rostest
from actionlib_msgs.msg import GoalID
from autolabor_canbus_driver.msg import CanBusMessage, ChassisStatusInfo
from autolabor_fod_msgs.msg import FodDetection, FodDetectionArray
from geometry_msgs.msg import Point32, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, RegionOfInterest
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool, SetBoolResponse

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_visual_servo_ros import FakeWorldIntegrationTest, MODEL_SHA, WHEELBASE
from autolabor_fod_control.visual_servo import local_displacement
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "SweepDeviceControl"))
from test_sweep_test import DeviceServer

# Every entry runs through actual GPS arming, 5 m sweep, visual handoff and stop.
CASES = [
    ("normal", {}),
    ("three_targets", {"bystanders": True}),
    ("depth_holes", {"bystanders": True, "depth_holes": True}),
    ("invalid_depth_flag", {"invalid_depth": True}),
    ("confidence_class_jitter", {"class_jitter": True}),
    ("region_boundary_jitter", {"region_jitter": True}),
    ("initial_target_low_in_frame", {"initial_q": .97}),
    ("short_dropout_before_region", {"pre_dropout": .16, "initial_q": .30, "loss_at": .55}),
    ("long_dropout_then_same_target", {"pre_dropout": .70, "initial_q": .30, "loss_at": .55}),
    ("boxes_return_after_loss", {"blink": True, "bystanders": True}),
    ("camera_stops_during_crossing", {"camera_off": True}),
    ("bad_frames_during_crossing", {"bad_after_loss": True}),
    ("duplicate_reordered_frames", {"duplicate": True}),
    ("malformed_unrelated_box", {"malformed": True}),
    ("odom_delivery_delay", {"odom_delay": .24}),
    ("odom_sample_gaps", {"odom_gaps": True}),
    ("left_turn_actuator_delay", {"offset": -280, "steer_delay": .25, "loss_at": .15}),
    ("right_turn_actuator_delay", {"offset": 280, "steer_delay": .25, "loss_at": .15}),
    ("slower_braking", {"deceleration": .55, "command_delay": .18}),
    ("stationary_wheel_not_centered", {"wheel_bias": 2.4}),
    ("fragmented_sweep_protocol", {"sweep_fragmented": True}),
    ("gps_fallback_without_fod", {"no_target": True}),
    ("gps_without_recovery_armed", {"not_armed": True}),
    ("real_estop_must_stop", {"fault": "estop"}),
    ("odom_disconnect_must_stop", {"fault": "odom_disconnect"}),
    ("odom_jump_must_stop", {"fault": "odom_jump"}),
    ("chassis_stall_must_stop", {"fault": "stall"}),
    ("detection_gap_060s", {"detection_gap": .60}),
    ("detection_gap_120s", {"detection_gap": 1.20}),
    ("detector_disconnect_120s", {"detection_gap": 1.20, "disconnect_detector": True}),
    ("control_update_pause_045s", {"control_pause": .45}),
    ("control_update_pause_090s", {"control_pause": .90}),
    ("curved_crossing_left", {"wheel_bias": 10.0, "offset": 0.0, "curved": True}),
    ("curved_crossing_right", {"wheel_bias": -10.0, "offset": 0.0, "curved": True}),
]


class FullRecoveryTaskTest(FakeWorldIntegrationTest):
    @classmethod
    def setUpClass(cls):
        cls.sweeper = DeviceServer(port=int(rospy.get_param("~sweep_port", 50194)))
        cls.results = []
        cls.repeat_seed = int(rospy.get_param("~seed", 1))

    @classmethod
    def tearDownClass(cls):
        cls.sweeper.__exit__()
        destination = rospy.get_param("~results_file", "")
        if destination:
            Path(destination).write_text(json.dumps(cls.results, indent=2))

    def setUp(self):
        self.case_name = self._testMethodName.split("_", 2)[2]
        self.scenario = dict(dict(CASES)[self.case_name])
        self.rng = random.Random(self.repeat_seed * 100 + len(self.case_name))
        self.mode_status = {}
        self.visual_status = {}
        self.mode_states = []
        self.visual_start = None
        self.visual_start_travel = None
        self.reference_ready = False
        self.loss_time = None
        self.pause_requests = []
        self.route_paused = False
        self.route_active = True
        self.gps_goal_x = 6.0
        self.started = time.monotonic()
        self.trace = []
        self.premature_sweep = False
        self.sweep_seen_on = False
        self.sweep_on_at_visual_start = False
        self.steering_angle = 0.0
        self.last_odom_publish = 0.0
        self.odom_sample_count = 0
        self.odom_dropped_count = 0
        self.odom_fifo = []
        self.last_message = None
        self.frame_n = 0
        self.detection_gap_frames = 0
        self.detector_unregistered = False
        self.control_pause_started = None
        self.control_pause_until = None
        self.control_pause_injected = False
        self.control_pause_zeros = []
        self.detection_gap_zeros = []
        self.max_blind_heading_deg = 0.0
        self.max_blind_lateral_m = 0.0
        self.sweeper.fragmented = self.scenario.get("sweep_fragmented", False)
        super().setUp()
        self.gps_pub = rospy.Publisher("/cmd_vel_gps", Twist, queue_size=1)
        self.gps_pose_pub = rospy.Publisher("/gps/odom", Odometry, queue_size=1)
        self.route_pub = rospy.Publisher("/gps/long_range/status", String, queue_size=1, latch=True)
        self.mode_sub = rospy.Subscriber("/fod_navigation_mode/status", String, self._mode_cb, queue_size=10)
        self.cancel_sub = rospy.Subscriber("/move_base/cancel", GoalID, lambda m: None, queue_size=2)
        self.pause_service = rospy.Service("/gps/long_range/set_paused", SetBool, self._pause_cb)
        self.mode_proxy = rospy.ServiceProxy("/fod_navigation_mode/set_fod_enabled", SetBool)
        self.actual_linear_x = 0.0
        self.extra_ready = True

    def tearDown(self):
        # Restore the simulated disconnected/estopped hardware before asking
        # the real manager to confirm a stop and reset for the next case.
        self.scenario.pop("fault", None)
        for kind in (0x17, 0x18, 0x19, 0x80):
            self.canbus_pub.publish(CanBusMessage(
                node_type=0x10, node_seq=0, msg_type=kind,
                payload=[0x10] if kind == 0x80 else [0]))
        rospy.sleep(.65)
        try:
            reset = self.mode_proxy(False)
            self.assertTrue(reset.success, reset.message)
        except (rospy.ServiceException, rospy.ROSException):
            pass
        self.timer.shutdown()
        super().tearDown()
        for item in (self.gps_pub, self.gps_pose_pub, self.route_pub, self.mode_sub, self.cancel_sub):
            item.unregister()
        self.pause_service.shutdown("case ended")

    def _mode_cb(self, message):
        with self.lock:
            self.mode_status = json.loads(message.data)
            state = self.mode_status.get("state")
            if not self.mode_states or self.mode_states[-1] != state:
                self.mode_states.append(state)

    def _status_cb(self, message):
        super()._status_cb(message)
        with self.lock:
            self.visual_status = json.loads(message.data)
            self.max_blind_heading_deg = max(self.max_blind_heading_deg,
                abs(self.visual_status.get("blind_heading_change_deg", 0.0)))
            self.max_blind_lateral_m = max(self.max_blind_lateral_m,
                abs(self.visual_status.get("blind_lateral_m", 0.0)))
            if self.visual_status.get("state") == "APPROACH" and self.visual_start is None:
                self.visual_start = time.monotonic()
                self.visual_start_travel = self.travel_m
                self.sweep_on_at_visual_start = self.sweeper.state == 1

    def _command_cb(self, message):
        super()._command_cb(message)
        now = time.monotonic()
        if abs(message.linear.x) > 1e-6:
            return
        with self.lock:
            if (self.control_pause_started is not None
                    and self.control_pause_started <= now < self.control_pause_until):
                self.control_pause_zeros.append(now - self.control_pause_started)
            if (self.visual_start is not None and self.scenario.get("detection_gap")
                    and .20 < now - self.visual_start < .20 + self.scenario["detection_gap"]):
                self.detection_gap_zeros.append(now - self.visual_start)

    def _canbus_service_cb(self, request):
        # Delay an existing blocking dependency, not the command publisher or
        # callbacks. This exercises a real stalled control tick with live odom.
        pause = self.scenario.get("control_pause", 0.0)
        if (pause and not self.control_pause_injected and self.visual_start is not None
                and self.travel_m - self.visual_start_travel > .05):
            self.control_pause_injected = True
            self.control_pause_started = time.monotonic()
            self.control_pause_until = self.control_pause_started + pause
            rospy.sleep(pause)
        return super()._canbus_service_cb(request)

    def _pause_cb(self, request):
        self.route_paused = request.data
        self.pause_requests.append(request.data)
        return SetBoolResponse(success=True, message="simulated GPS route pause/resume")

    def _publish_world(self, event):
        if not getattr(self, "extra_ready", False):
            return
        now = time.monotonic()
        dt = min(.08, max(.001, now - self.last_tick))
        self.last_tick = now
        with self.lock:
            history = list(self.command_history)
            delay = self.scenario.get("command_delay", .10) + self.rng.uniform(0, .025)
            past = [m for t, m in history if t <= now - delay]
            desired = past[-1] if past else Twist()
            target_v = desired.linear.x if abs(desired.linear.x) >= .199 else 0.0
            acceleration = 1.0 if target_v > self.actual_linear_x else self.scenario.get("deceleration", 1.0)
            self.actual_linear_x += max(-acceleration * dt, min(acceleration * dt, target_v - self.actual_linear_x))
            if self.scenario.get("fault") == "stall" and self.loss_time is not None:
                self.actual_linear_x = 0.0
            steer_past = [m for t, m in history if t <= now - self.scenario.get("steer_delay", .08)]
            steer = steer_past[-1] if steer_past else Twist()
            desired_angle = math.atan(steer.angular.z * WHEELBASE / steer.linear.x) if abs(steer.linear.x) > 1e-6 else 0.0
            desired_angle += math.radians(self.scenario.get("wheel_bias", 0.0))
            self.steering_angle += max(-1.2 * dt, min(1.2 * dt, desired_angle - self.steering_angle))
            v = self.actual_linear_x
            omega = v * math.tan(self.steering_angle) / WHEELBASE
            self.yaw += omega * dt
            self.x += v * math.cos(self.yaw) * dt
            self.y += v * math.sin(self.yaw) * dt
            self.travel_m += max(0.0, v) * dt
            x, y, yaw, travel = self.x, self.y, self.yaw, self.travel_m
            if self.sweeper.state:
                self.sweep_seen_on = True
                if self.gps_goal_x - x >= 5.0 + .02:
                    self.premature_sweep = True
            self.trace.append([now - self.started, x, y, v, self.command.linear.x,
                               self.visual_status.get("state", ""), self.sweeper.state])
        stamp = rospy.Time.now()
        after_loss = self.loss_time is not None
        if not (after_loss and self.scenario.get("camera_off")):
            self._publish_camera(stamp)
            self._publish_detection(stamp, travel, x, y, yaw)
        odom_period = .1
        if now - self.last_odom_publish >= odom_period - .003:
            self.last_odom_publish = now
            self.odom_sample_count += 1
            skip = (self.scenario.get("odom_gaps") and after_loss
                    and now - self.loss_time > .35
                    and self.odom_sample_count % 3 == 0)
            if skip:
                self.odom_dropped_count += 1
            else:
                self.odom_fifo.append((now + self.scenario.get("odom_delay", 0), (stamp, x, y, yaw, v, omega)))
        while self.odom_fifo and self.odom_fifo[0][0] <= now:
            _, sample = self.odom_fifo.pop(0)
            if not (after_loss and self.scenario.get("fault") == "odom_disconnect"):
                if after_loss and self.scenario.get("fault") == "odom_jump":
                    sample = (sample[0], sample[1] + 1.0, *sample[2:])
                self._publish_odometry(*sample)
        self.wheel_pub.publish(Float64(data=self.steering_angle))
        status = ChassisStatusInfo()
        status.hard_emergency = bool(after_loss and self.scenario.get("fault") == "estop")
        self.chassis_pub.publish(status)
        self.timeout_pub.publish(Bool(data=False))
        if status.hard_emergency:
            raw = CanBusMessage(node_type=0x10, node_seq=0, msg_type=0x19, payload=[1])
            self.canbus_pub.publish(raw)
        gps = Odometry()
        gps.header.stamp = stamp
        gps.header.frame_id = "camera_init"
        gps.pose.pose.position.x, gps.pose.pose.position.y = x, y
        self.gps_pose_pub.publish(gps)
        reached = x >= self.gps_goal_x - .12 and not self.route_paused
        self.route_active = not reached
        self.route_pub.publish(String(data=json.dumps({
            "active": self.route_active, "state": "COMPLETE" if reached else "ACTIVE_FINAL",
            "paused": self.route_paused, "frame_id": "camera_init", "final_x": self.gps_goal_x,
            "final_y": 0.0, "route_id": "sim/" + self.case_name})))
        gps_cmd = Twist()
        gps_cmd.linear.x = 0 if reached else .60
        self.gps_pub.publish(gps_cmd)

    def _publish_camera(self, stamp):
        m = CameraInfo()
        m.header.stamp, m.header.frame_id = stamp, "zed2_left_camera_optical_frame"
        m.width, m.height = 1280, 720
        m.K = [800., 0., 620., 0., 800., 360., 0., 0., 1.]
        m.P = [800., 0., 620., 0., 0., 800., 360., 0., 0., 0., 1., 0.]
        self.camera_pub.publish(m)

    @staticmethod
    def _box(identity, u, q, distance=1.0):
        item = FodDetection()
        item.class_id, item.class_name, item.confidence = 0, "Metal", .90
        item.track_id, item.depth_valid = identity, True
        item.depth_m, item.range_m = distance, distance + .05
        v = (.25 + .375 * q) * 719
        item.anchor_px = Point32(x=u, y=v)
        item.bbox = RegionOfInterest(x_offset=int(u - 25), y_offset=int(v - 35), width=50, height=35)
        return item

    def _publish_detection(self, stamp, travel, x, y, yaw):
        self.frame_n += 1
        elapsed = 0.0 if self.visual_start is None else time.monotonic() - self.visual_start
        gap = self.scenario.get("detection_gap", 0.0)
        if gap and .20 < elapsed < .20 + gap:
            self.detection_gap_frames += 1
            if self.scenario.get("disconnect_detector") and not self.detector_unregistered:
                self.detection_pub.unregister()
                self.detector_unregistered = True
            return
        if self.detector_unregistered:
            self.detection_pub = rospy.Publisher("/fod/detections", FodDetectionArray, queue_size=5)
            self.detector_unregistered = False
        if (self.scenario.get("disconnect_detector")
                and self.detection_pub.get_num_connections() < 2):
            # Both the manager and controller must reconnect before the first
            # delivered empty frame defines the independent loss-distance
            # reference. Frames published without subscribers are not seen.
            return
        distance = 0.0 if self.visual_start_travel is None else travel - self.visual_start_travel
        initial_q = self.scenario.get("initial_q", .53)
        q = min(.98, initial_q + distance * .8)
        if self.scenario.get("region_jitter"):
            q = .50 + (.015 if self.frame_n % 2 else -.015)
        u = 620.0 + self.scenario.get("offset", 70.0) * max(0, 1 - distance / .60)
        visible = distance < self.scenario.get("loss_at", .35)
        if self.scenario.get("pre_dropout") and .12 < elapsed < .12 + self.scenario["pre_dropout"]:
            visible = False
        if self.scenario.get("blink") and self.loss_time is not None:
            visible = self.frame_n % 3 == 0
        if self.scenario.get("no_target"):
            visible = False
        if visible and self.visual_status.get("target_locked") and .5 <= q <= 1 and 236 <= u <= 1004:
            self.reference_ready = True
        if not visible and self.reference_ready and self.loss_pose is None:
            self.loss_pose = (x, y, yaw)
            self.loss_time = time.monotonic()
        m = FodDetectionArray()
        m.header.stamp, m.header.frame_id = stamp, "zed2_left_camera_optical_frame"
        m.image_width, m.image_height, m.model_sha256, m.model_task = 1280, 720, MODEL_SHA, "detect"
        m.model_name = "simulated-approved-model"
        if visible:
            item = self._box(1, u, q)
            if elapsed > .05 and self.scenario.get("depth_holes"):
                item.depth_valid, item.depth_m, item.range_m = False, 0, 0
            if elapsed > .05 and self.scenario.get("invalid_depth"):
                item.depth_m = float("nan")
            if elapsed > .05 and self.scenario.get("class_jitter"):
                item.class_name = "Plastic" if self.frame_n % 2 else "Metal"
                item.confidence = .12 if self.frame_n % 3 else .65
            m.detections.append(item)
        if self.scenario.get("bystanders"):
            m.detections.extend([self._box(2, 410., .55, 2.0), self._box(3, 830., .70, 3.0)])
            if self.frame_n % 2:
                m.detections.reverse()
        if self.scenario.get("malformed"):
            bad = self._box(4, 420., .6)
            bad.bbox.width = 0
            m.detections.append(bad)
        if (self.visual_status.get("state") in ("BLIND_ADVANCE", "FINAL_STOP")
                and self.scenario.get("bad_after_loss")):
            m.model_sha256 = "bad-post-crossing-camera-data"
        self.detection_pub.publish(m)
        if self.scenario.get("duplicate"):
            self.detection_pub.publish(m)
            if self.last_message is not None:
                self.detection_pub.publish(self.last_message)
        self.last_message = copy.deepcopy(m)

    def run_case(self):
        rospy.wait_for_service("/fod_navigation_mode/set_fod_enabled", timeout=6)
        rospy.sleep(.7)
        # Seed the same VCU state a connected vehicle supplies before enable.
        for kind in (0x17, 0x18, 0x19, 0x80):
            self.canbus_pub.publish(CanBusMessage(node_type=0x10, node_seq=0, msg_type=kind,
                                                payload=[0x10] if kind == 0x80 else [0]))
        if not self.scenario.get("not_armed"):
            response = self.mode_proxy(True)
            self.assertTrue(response.success, response.message)
        deadline = time.monotonic() + 30
        is_gps = self.scenario.get("no_target") or self.scenario.get("not_armed")
        while time.monotonic() < deadline:
            if self.mode_status.get("state") in ("FOD_COMPLETE_STOP", "FOD_ABORTED", "FAULT_STOP"):
                break
            if is_gps and not self.route_active and not self.sweeper.state:
                break
            rospy.sleep(.05)
        rospy.sleep(.75)
        with self.lock:
            state = self.mode_status.get("state", "")
            result = {"case": self.case_name, "seed": self.repeat_seed, "mode": state,
                      "visual": self.visual_status.get("state"), "reason": self.latest_reason,
                      "ids": sorted(self.selected_target_ids), "modes": list(self.mode_states),
                      "visual_states": list(self.states), "sweep_on": self.sweep_seen_on,
                      "sweep_off": self.sweeper.state == 0, "premature_sweep": self.premature_sweep,
                      "first_zero_after_loss_m": self.first_zero_forward_m,
                      "intermediate_zero_commands": list(self.intermediate_zero_commands),
                      "odom_dropped_count": self.odom_dropped_count,
                      "detection_gap_frames": self.detection_gap_frames,
                      "detection_gap_zero_commands": list(self.detection_gap_zeros),
                      "control_pause_injected": self.control_pause_injected,
                      "control_pause_zero_commands": list(self.control_pause_zeros),
                      "max_blind_heading_deg": self.max_blind_heading_deg,
                      "max_blind_lateral_m": self.max_blind_lateral_m}
            if self.loss_pose is not None:
                result["forward_m"], result["lateral_m"] = local_displacement(*self.loss_pose, self.x, self.y)
            self.results.append(result)
            output = rospy.get_param("~results_file", "")
            if output:
                Path(output).write_text(json.dumps(self.results, indent=2))
                Path(output).with_name(Path(output).stem + "_" + self.case_name + ".json").write_text(json.dumps(self.trace))
        self.assertFalse(self.premature_sweep, result)
        if self.scenario.get("fault"):
            self.assertEqual(state, "FOD_ABORTED", result)
            self.assertNotIn("FOD_COMPLETE_STOP", self.mode_states)
            self.assertEqual(self.command.linear.x, 0.0)
            return
        if is_gps:
            self.assertEqual(state, "GPS_ACTIVE", result)
            self.assertNotIn("APPROACH", self.states)
            self.assertFalse(self.route_active)
            self.assertEqual(self.sweep_seen_on, not self.scenario.get("not_armed", False))
            return
        self.assertEqual(state, "FOD_COMPLETE_STOP", result)
        self.assertEqual(self.visual_status.get("state"), "COMPLETE", result)
        self.assertEqual(self.selected_target_ids, {1}, result)
        self.assertTrue(self.sweep_on_at_visual_start, result)
        self.assertTrue(self.sweep_seen_on, result)
        self.assertEqual(self.sweeper.state, 0, result)
        self.assertIn(True, self.pause_requests, result)
        self.assertNotIn("ABORT", self.states, result)
        self.assertEqual(self.states.count("BLIND_ADVANCE"), 1, result)
        self.assertFalse(self.intermediate_zero_commands, result)
        self.assertGreaterEqual(result["forward_m"], .795, result)
        self.assertLessEqual(result["forward_m"], .96, result)
        self.assertGreaterEqual(self.first_zero_forward_m, .795, result)
        # Delayed odometry describes a past pose: at the measured 0.80 m
        # trigger the simulated chassis has already travelled further. Bound
        # both this latency and the subsequent real braking distance above.
        trigger_limit = .90 if self.scenario.get("odom_delay") else .86
        self.assertLessEqual(self.first_zero_forward_m, trigger_limit, result)
        self.assertEqual(self.command.linear.x, 0.0, result)
        self.assertLess(abs(self.actual_linear_x), .01, result)
        if self.scenario.get("odom_gaps"):
            self.assertGreater(self.odom_dropped_count, 0, result)
        if self.scenario.get("pre_dropout", 0) > .3:
            self.assertIn("TRACK_WAIT", self.states, result)
        if self.scenario.get("detection_gap"):
            self.assertGreater(self.detection_gap_frames, 0, result)
            self.assertFalse(self.detection_gap_zeros, result)
        if self.scenario.get("control_pause"):
            self.assertTrue(self.control_pause_injected, result)
            self.assertFalse(self.control_pause_zeros, result)
        if self.scenario.get("curved"):
            self.assertGreater(self.max_blind_heading_deg, 4.0, result)
            self.assertGreater(self.max_blind_lateral_m, .08, result)


# Reuse only the physical-world/ROS fixture, not its standalone test methods.
for name in unittest.defaultTestLoader.getTestCaseNames(FakeWorldIntegrationTest):
    setattr(FullRecoveryTaskTest, name, None)
for index, (name, _) in enumerate(CASES):
    setattr(FullRecoveryTaskTest, "test_%02d_%s" % (index, name), FullRecoveryTaskTest.run_case)

if __name__ == "__main__":
    rospy.init_node("fake_world", anonymous=False)
    selected = rospy.get_param("~case_filter", "")
    if selected:
        allowed = set(selected.split(","))
        for name in unittest.defaultTestLoader.getTestCaseNames(FullRecoveryTaskTest):
            if name.split("_", 2)[2] not in allowed:
                setattr(FullRecoveryTaskTest, name, None)
    rostest.rosrun("autolabor_fod_control", "full_recovery", FullRecoveryTaskTest)
