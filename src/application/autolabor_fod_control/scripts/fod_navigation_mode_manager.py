#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fail-closed GPS/FOD mode manager and the sole chassis command publisher."""

import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time

import rosgraph
import rospy
import tf2_ros
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, CompressedImage, LaserScan, NavSatFix
from autolabor_canbus_driver.msg import ChassisStatusInfo
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from autolabor_fod_msgs.msg import FodDetectionArray
from autolabor_fod_control.visual_servo import PixelDetection, recovery_depth_allowed, validate_detection
from autolabor_fod_control.gps_recovery import (
    FOD_GOAL_RADIUS_M, FodGpsMemory, depth_point_in_world,
    inside_vehicle_footprint, point_matches_recovery_goal,
)
from autolabor_fod_control.recovery_verification import RecoveryVerification, TERMINAL, rear_corridor_clear

try:
    from autolabor_gps_keepout.geometry import KeepoutGeometry
except ImportError:
    KeepoutGeometry = None

from autolabor_fod_control.mode_manager import (
    FOD_SOURCE,
    GPS_SOURCE,
    STOP_SOURCE,
    CommandArbiter,
    final_goal_distance,
    stopped_sample_is_valid,
)


GPS_ACTIVE = "GPS_ACTIVE"
ENTERING_GPS_RECOVERY = "ENTERING_GPS_RECOVERY"
GPS_FOD_RECOVERY = "GPS_FOD_RECOVERY"
ENTERING_FOD = "ENTERING_FOD"
FOD_ACTIVE = "FOD_ACTIVE"
FOD_COMPLETE_STOP = "FOD_COMPLETE_STOP"
FOD_VERIFY = "FOD_VERIFY"
RETURNING_GPS = "RETURNING_GPS"
FOD_ABORTED = "FOD_ABORTED"
FAULT_STOP = "FAULT_STOP"


def _strict_bool_param(name, default):
    value = rospy.get_param(name, default)
    if type(value) is not bool:
        raise ValueError("{} must be a YAML boolean true/false".format(name))
    return value


class FodNavigationModeManager:
    """Serialize controller handoff and forward exactly one velocity source."""

    def __init__(self):
        self.gps_cmd_topic = rospy.get_param("~gps_cmd_topic", "/cmd_vel_gps")
        self.fod_cmd_topic = rospy.get_param("~fod_cmd_topic", "/cmd_vel_fod")
        self.output_cmd_topic = rospy.get_param("~output_cmd_topic", "/cmd_vel")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.visual_state_topic = rospy.get_param(
            "~visual_state_topic", "/fod_visual_servo/state"
        )
        self.visual_enable_service = rospy.get_param(
            "~visual_enable_service", "/fod_visual_servo/set_enabled"
        )
        self.gps_pause_service = rospy.get_param(
            "~gps_pause_service", "/gps/long_range/set_paused"
        )
        self.gps_clear_service = rospy.get_param(
            "~gps_clear_service", "/gps/long_range/clear_route")
        # Indoor LiDAR navigation has no retained WGS84 route. The existing
        # command arbitration, goal cancellation and stopped-vehicle handoff
        # still apply; only the absent GPS route service is skipped.
        self.retain_gps_route = _strict_bool_param("~retain_gps_route", True)
        self.gps_fallback_enabled = self.retain_gps_route and _strict_bool_param(
            "~gps_fallback_enabled", False)
        self.gps_route_status_topic = rospy.get_param(
            "~gps_route_status_topic", "/gps/long_range/status")
        self.gps_pose_topic = rospy.get_param("~gps_pose_topic", "/gps/odom")
        self.recovery_radius_m = float(rospy.get_param("~recovery_radius_m", 10.0))
        self.detections_topic = rospy.get_param("~detections_topic", "/fod/detections")
        self.select_nearest_by_depth = _strict_bool_param(
            "/fod_visual_servo/select_nearest_by_depth", False)
        self.min_confidence = float(rospy.get_param(
            "/fod_visual_servo/min_confidence", 0.30))
        self.allowed_class_names = set(rospy.get_param(
            "/fod_visual_servo/allowed_class_names", ["Metal", "Soft", "Plastic", "Wire", "Tool", "w"]))
        self.sweep_enabled = _strict_bool_param("~sweep_enabled", True)
        self.sweep_auto_stop = _strict_bool_param("~sweep_auto_stop", True)
        self.sweep_script = rospy.get_param(
            "~sweep_script", str(Path(__file__).resolve().parents[4] / "scripts/sweep_test.py"))
        self.sweep_ip = rospy.get_param("~sweep_ip", "192.168.0.197")
        self.sweep_port = int(rospy.get_param("~sweep_port", 50003))
        self.move_base_goal_topic = rospy.get_param(
            "~move_base_goal_topic", "/move_base/goal"
        )
        self.move_base_cancel_topic = rospy.get_param(
            "~move_base_cancel_topic", "/move_base/cancel"
        )
        self.expected_driver_node = rospy.get_param(
            "~expected_driver_node", "/m2_driver"
        )
        self.expected_visual_node = rospy.get_param(
            "~expected_visual_node", "/fod_visual_servo"
        )
        self.expected_gps_limiter_node = rospy.get_param(
            "~expected_gps_limiter_node", "/gps_goal_speed_limiter"
        )
        self.expected_move_base_node = rospy.get_param(
            "~expected_move_base_node", "/move_base"
        )

        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 20.0))
        self.status_rate_hz = float(rospy.get_param("~status_rate_hz", 2.0))
        self.command_timeout_sec = float(
            rospy.get_param("~command_timeout_sec", 0.5)
        )
        self.odom_timeout_sec = float(rospy.get_param("~odom_timeout_sec", 0.6))
        self.stop_linear_speed_mps = float(
            rospy.get_param("~stop_linear_speed_mps", 0.03)
        )
        self.stop_angular_speed_rps = float(
            rospy.get_param("~stop_angular_speed_rps", 0.05)
        )
        self.stop_confirm_sec = float(rospy.get_param("~stop_confirm_sec", 0.5))
        self.transition_timeout_sec = float(
            rospy.get_param("~transition_timeout_sec", 12.0)
        )
        self.service_wait_timeout_sec = float(
            rospy.get_param("~service_wait_timeout_sec", 2.0)
        )
        self.graph_check_rate_hz = float(
            rospy.get_param("~graph_check_rate_hz", 1.0)
        )
        self.require_output_driver = _strict_bool_param(
            "~require_output_driver", True
        )

        self._validate_parameters()

        self.lock = threading.RLock()
        self.transition_lock = threading.Lock()
        self.shutdown_event = threading.Event()
        self.arbiter = CommandArbiter(self.command_timeout_sec)
        self.mode = GPS_ACTIVE
        self.reason = "GPS navigation owns the chassis command path"
        self.command_source = GPS_SOURCE
        self.visual_state = "UNKNOWN"
        self.gps_paused = False
        self.allow_move_base_goals = True
        self.latest_odom_receipt = None
        self.latest_linear_x = None
        self.latest_angular_z = None
        self.verification_enabled = _strict_bool_param('~verification_enabled', True)
        self.verification_roi_top = float(rospy.get_param('/fod_visual_servo/roi_top_fraction', 0.0))
        self.verification_roi_bottom = float(rospy.get_param('/fod_visual_servo/roi_bottom_fraction', 1.0))
        self.verification_this_run = False
        self.verification = RecoveryVerification()
        self.verification_odom = None
        self.verification_scan = None
        self.verification_scan_receipt = None
        self.verification_emergency = False
        self.last_output_reason = "waiting for first GPS command"
        self.fault_worker_started = False
        self.recovery_armed = False
        self.direct_recovery = False
        self.proximity_worker_started = False
        self.gps_route = {}
        self.gps_pose = None
        self.gps_pose_yaw = None
        self.keepout = None
        self.keepout_frame = "camera_init"
        self.gps_pose_receipt = None
        self.gps_pose_stamp = None
        self.sweep_started_for_recovery = False
        self.sweep_start_requested = False
        self.recovery_sweep_ready = False
        self.sweep_status = "not requested"
        self.sweep_cleanup_confirmed = True
        self.sweep_cleanup_pending = False
        self.sweep_cleanup_hydraulic_sent = False
        self.sweep_cleanup_hydraulic_acknowledged = False
        self.sweep_operation_attempts = 0
        self.fod_target_present = False
        self.detection_receipt = None
        self.detection_stamp = None
        self.fod_gps_memory = FodGpsMemory(frames=int(rospy.get_param(
            "/fod_visual_servo/acquire_frames", 6)))
        self.target_loss_grace_sec = float(rospy.get_param(
            "/fod_visual_servo/tracking_loss_grace_sec", .30))
        self.fod_approach_started = False
        self.vehicle_footprint = rospy.get_param(
            "~vehicle_footprint", rospy.get_param(
                "/vehicle_outline/footprint",
                [[1.22,.4],[1.22,-.4],[-.52,-.4],[-.52,.4]]))
        self.fod_gps_point = None
        self.fod_reference_goal = None
        self.fod_gps_reason = "等待 6 米内同一 FOD 连续 {} 帧稳定深度".format(self.fod_gps_memory.frames)
        self.visible_track_ids = set()
        self.camera_info = None
        self.gps_origin = None
        self.recovery_generation = 0
        self.recovery_cancel_requested = threading.Event()
        self.gps_fallback_target = None
        self.gps_fallback_route_id = None
        self.gps_fallback_sent = False
        self.gps_fallback_error = None
        self.gps_fallback_ack = threading.Event()
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(15.0)) if self.gps_fallback_enabled else None
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer) if self.tf_buffer else None

        self.master = rosgraph.Master(rospy.get_name())
        self.master_lock = threading.Lock()
        self.master_pid = self.master.getPid()

        self.cmd_pub = rospy.Publisher(
            self.output_cmd_topic, Twist, queue_size=1
        )
        self.cancel_pub = rospy.Publisher(
            self.move_base_cancel_topic, GoalID, queue_size=10
        )
        self.gps_goal_pub = rospy.Publisher("/gps/goal_fix", NavSatFix, queue_size=1)
        self.camera_info_sub = rospy.Subscriber(
            "/fod_camera/camera_info", CameraInfo, self._camera_info_cb, queue_size=1)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )
        # A latched visual-state message can call _publish_status as soon as
        # the subscriber is registered, before the constructor returns.
        self.verification_pub = rospy.Publisher('~verification/status', String, queue_size=5, latch=True)

        self.gps_cmd_sub = rospy.Subscriber(
            self.gps_cmd_topic, Twist, self._gps_cmd_cb, queue_size=1
        )
        self.fod_cmd_sub = rospy.Subscriber(
            self.fod_cmd_topic, Twist, self._fod_cmd_cb, queue_size=1
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_cb, queue_size=20
        )
        self.visual_state_sub = rospy.Subscriber(
            self.visual_state_topic, String, self._visual_state_cb, queue_size=10
        )
        self.move_base_goal_sub = rospy.Subscriber(
            self.move_base_goal_topic,
            MoveBaseActionGoal,
            self._move_base_goal_cb,
            queue_size=10,
        )
        self.gps_route_sub = rospy.Subscriber(
            self.gps_route_status_topic, String, self._gps_route_cb, queue_size=10)
        self.gps_pose_sub = rospy.Subscriber(
            self.gps_pose_topic, Odometry, self._gps_pose_cb, queue_size=10)
        self.detections_sub = rospy.Subscriber(
            self.detections_topic, FodDetectionArray, self._detections_cb, queue_size=1)
        self.keepout_sub = rospy.Subscriber(
            "/gps/keepout/state", String, self._keepout_cb, queue_size=1)
        self.mode_service = rospy.Service(
            "~set_fod_enabled", SetBool, self._set_fod_enabled_cb
        )
        self.direct_mode_service = rospy.Service(
            "~set_direct_fod_enabled", SetBool, self._set_direct_fod_enabled_cb)
        self.prepare_goal_service = rospy.Service(
            "~prepare_gps_goal", Trigger, self._prepare_gps_goal_cb)
        self.verification_service = rospy.Service(
            '~set_verification_enabled', SetBool, self._set_verification_enabled_cb)
        self.verification_scan_sub = rospy.Subscriber('/scan', LaserScan, self._verification_scan_cb, queue_size=1)
        self.verification_image_sub = rospy.Subscriber('/fod_camera/image_raw/compressed', CompressedImage,
                                                      self._verification_image_cb, queue_size=1, buff_size=4*1024*1024)
        self.verification_chassis_sub = rospy.Subscriber('/m2_driver/chassis_info', ChassisStatusInfo,
                                                        self._verification_chassis_cb, queue_size=1)

        self.command_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz),
            self._publish_command,
        )
        self.status_timer = rospy.Timer(
            rospy.Duration(1.0 / self.status_rate_hz),
            self._publish_status,
        )
        self.graph_timer = rospy.Timer(
            rospy.Duration(1.0 / self.graph_check_rate_hz),
            self._graph_watchdog,
        )
        rospy.on_shutdown(self._shutdown)

        self._publish_status(force=True)
        rospy.logwarn(
            "GPS/FOD mode manager owns %s: GPS=%s, FOD=%s. "
            "FOD motion is disabled until %s/set_fod_enabled true.",
            self.output_cmd_topic,
            self.gps_cmd_topic,
            self.fod_cmd_topic,
            rospy.get_name(),
        )

    def _validate_parameters(self):
        absolute_names = {
            "gps_cmd_topic": self.gps_cmd_topic,
            "fod_cmd_topic": self.fod_cmd_topic,
            "output_cmd_topic": self.output_cmd_topic,
            "odom_topic": self.odom_topic,
            "visual_state_topic": self.visual_state_topic,
            "visual_enable_service": self.visual_enable_service,
            "gps_pause_service": self.gps_pause_service,
            "move_base_goal_topic": self.move_base_goal_topic,
            "move_base_cancel_topic": self.move_base_cancel_topic,
            "gps_route_status_topic": self.gps_route_status_topic,
            "gps_pose_topic": self.gps_pose_topic,
            "detections_topic": self.detections_topic,
            "expected_driver_node": self.expected_driver_node,
            "expected_visual_node": self.expected_visual_node,
            "expected_gps_limiter_node": self.expected_gps_limiter_node,
            "expected_move_base_node": self.expected_move_base_node,
        }
        for name, value in absolute_names.items():
            if not isinstance(value, str) or not value.startswith("/"):
                raise ValueError("{} must be an absolute ROS name".format(name))
        if len(
            {
                self.gps_cmd_topic,
                self.fod_cmd_topic,
                self.output_cmd_topic,
            }
        ) != 3:
            raise ValueError("GPS, FOD, and chassis command topics must be distinct")
        positive = {
            "publish_rate_hz": self.publish_rate_hz,
            "status_rate_hz": self.status_rate_hz,
            "command_timeout_sec": self.command_timeout_sec,
            "odom_timeout_sec": self.odom_timeout_sec,
            "stop_confirm_sec": self.stop_confirm_sec,
            "transition_timeout_sec": self.transition_timeout_sec,
            "service_wait_timeout_sec": self.service_wait_timeout_sec,
            "graph_check_rate_hz": self.graph_check_rate_hz,
            "recovery_radius_m": self.recovery_radius_m,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        nonnegative = {
            "stop_linear_speed_mps": self.stop_linear_speed_mps,
            "stop_angular_speed_rps": self.stop_angular_speed_rps,
        }
        for name, value in nonnegative.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError("{} must be finite and nonnegative".format(name))
        if self.stop_confirm_sec >= self.transition_timeout_sec:
            raise ValueError("stop_confirm_sec must be below transition_timeout_sec")

    @staticmethod
    def _topic_nodes(entries, topic):
        for candidate, nodes in entries:
            if candidate == topic:
                return set(nodes)
        return set()

    def _output_route_error(self):
        try:
            # rosgraph.Master shares an XML-RPC HTTP connection. Timer and
            # handoff workers must not use it concurrently (Request-sent).
            with self.master_lock:
                if self.master.getPid() != self.master_pid:
                    return "ROS master was replaced"
                publishers, subscribers, _services = self.master.getSystemState()
        except Exception as exc:
            return "cannot inspect ROS command graph: {}".format(exc)
        output_publishers = self._topic_nodes(publishers, self.output_cmd_topic)
        other_publishers = output_publishers - {rospy.get_name()}
        if other_publishers:
            return "{} has conflicting publishers: {}".format(
                self.output_cmd_topic,
                ", ".join(sorted(other_publishers)),
            )
        output_subscribers = self._topic_nodes(subscribers, self.output_cmd_topic)
        if (
            self.require_output_driver
            and self.expected_driver_node not in output_subscribers
        ):
            return "{} is not connected to {}; subscribers: {}".format(
                self.output_cmd_topic,
                self.expected_driver_node,
                ", ".join(sorted(output_subscribers)) or "none",
            )
        gps_publishers = self._topic_nodes(publishers, self.gps_cmd_topic)
        allowed_gps_publishers = {
            self.expected_gps_limiter_node,
            self.expected_move_base_node,
        }
        if len(gps_publishers) > 1 or not gps_publishers.issubset(
            allowed_gps_publishers
        ):
            return "{} publisher ownership is invalid: {}".format(
                self.gps_cmd_topic,
                ", ".join(sorted(gps_publishers)) or "none",
            )
        fod_publishers = self._topic_nodes(publishers, self.fod_cmd_topic)
        if len(fod_publishers) > 1 or not fod_publishers.issubset(
            {self.expected_visual_node}
        ):
            return "{} publisher ownership is invalid: {}".format(
                self.fod_cmd_topic,
                ", ".join(sorted(fod_publishers)) or "none",
            )
        return ""

    def _gps_cmd_cb(self, msg):
        with self.lock:
            self.arbiter.update(
                GPS_SOURCE,
                msg.linear.x,
                msg.angular.z,
                time.monotonic(),
            )

    def _fod_cmd_cb(self, msg):
        with self.lock:
            self.arbiter.update(
                FOD_SOURCE,
                msg.linear.x,
                msg.angular.z,
                time.monotonic(),
            )

    def _odom_cb(self, msg):
        linear_x = float(msg.twist.twist.linear.x)
        angular_z = float(msg.twist.twist.angular.z)
        with self.lock:
            p, q = msg.pose.pose.position, msg.pose.pose.orientation
            norm = q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.verification_odom = ((p.x, p.y, yaw, msg.header.frame_id, msg.header.stamp.to_sec())
                                      if .9 < norm < 1.1 and not msg.header.stamp.is_zero() else None)
            self.latest_odom_receipt = time.monotonic()
            if math.isfinite(linear_x) and math.isfinite(angular_z):
                self.latest_linear_x = linear_x
                self.latest_angular_z = angular_z
            else:
                self.latest_linear_x = None
                self.latest_angular_z = None

    def _gps_route_cb(self, msg):
        try:
            route = json.loads(msg.data)
            if not isinstance(route, dict):
                return
        except (ValueError, TypeError):
            return
        finish_gps_recovery = False
        with self.lock:
            # During loss handoff the old route is resumed only while output
            # is STOP. Its late cancel/completion is not the new pickup result.
            if self.mode == ENTERING_GPS_RECOVERY:
                target = self.gps_fallback_target
                if self.gps_fallback_sent:
                    if route.get("state") == "REJECTED_NEW_GOAL":
                        self.gps_fallback_error = route.get("reason", "GPS goal rejected")
                        self.gps_fallback_ack.set()
                    elif (target and (route.get("route_id") != self.gps_fallback_route_id or self.gps_fallback_ack.is_set())
                          and abs(route.get("final_latitude", 999)-target["goal_latitude"]) < 1e-9
                          and abs(route.get("final_longitude", 999)-target["goal_longitude"]) < 1e-9):
                        self.gps_fallback_route_id = route.get("route_id")
                        if not route.get("active") and route.get("state") != "COMPLETE":
                            self.gps_fallback_error = route.get("reason", "GPS route inactive")
                        self.gps_fallback_ack.set()
                if not self.gps_fallback_ack.is_set() or route.get("route_id") == self.gps_fallback_route_id:
                    self.gps_route = route
                return
            if self.mode == GPS_FOD_RECOVERY:
                if route.get("route_id") != self.gps_fallback_route_id:
                    if route.get("active"):
                        # An explicit newer GPS goal supersedes this pickup.
                        self.mode, self.command_source = GPS_ACTIVE, GPS_SOURCE
                        self.reason = "新 GPS 目标已接管，结束本次回收"
                        finish_gps_recovery = True
                    else:
                        return  # Late status belonging to the previous route.
                elif not route.get("active"):
                    if route.get("state") == "COMPLETE":
                        self._recovery_finished_locked("已到达记录 FOD 前方 0.8 米终点；停车、停止清扫并收杆 3 秒")
                    elif route.get("state") == "SUPERSEDED":
                        self.mode, self.command_source = GPS_ACTIVE, GPS_SOURCE
                        self.reason = "手动目标已接管，结束本次回收"
                    else:
                        self.mode, self.command_source = FOD_ABORTED, STOP_SOURCE
                        self.allow_move_base_goals = False
                        self.reason = "GPS 回收未完成：" + str(route.get("reason", route.get("state")))
                        self._publish_zero_locked(self.reason)
                    finish_gps_recovery = True
                self.gps_route = route
                if finish_gps_recovery:
                    threading.Thread(target=self._finish_gps_recovery_worker,
                                     name="fod-gps-sweep-finish", daemon=True).start()
                return
            arrived = (route.get("state") == "COMPLETE"
                       and self.gps_route.get("active", False))
            if (self.gps_fallback_enabled and self.recovery_armed and self.mode == GPS_ACTIVE
                    and route.get("active") and self.gps_route.get("route_id")
                    and route.get("route_id") != self.gps_route.get("route_id")):
                self.fod_gps_memory.reset()
                self.fod_gps_point = None
                self.fod_reference_goal = None
                self.fod_approach_started = False
                self.recovery_generation += 1
            self.gps_route = route
            if (self.recovery_armed and self.mode == GPS_ACTIVE
                    and not route.get("active", False)
                    and (arrived or route.get("state") in ("CANCELED", "ABORTED", "SUPERSEDED", "KEEPOUT_BLOCKED"))):
                self.recovery_armed = False
                self.reason = ("GPS 目标已到达，停止清扫并回收液压杆 3 秒" if arrived else
                               "回收未完成，GPS 任务结束：" + str(route.get("reason", route.get("state"))))
                if arrived and getattr(self, 'verification_this_run', False):
                    self._recovery_finished_locked(self.reason)
                finish_gps_recovery = True
        if finish_gps_recovery:
            threading.Thread(target=self._finish_gps_recovery_worker,
                             name="fod-gps-sweep-finish", daemon=True).start()
        self._maybe_start_recovery()

    def _finish_gps_recovery_worker(self):
        with self.transition_lock:
            with self.lock:
                # A newer operator request may already own this same sweeper.
                if self.mode not in (GPS_ACTIVE, FOD_COMPLETE_STOP, FOD_ABORTED, FOD_VERIFY) or self.recovery_armed:
                    return
            cleaned = self._stop_recovery_sweep()
            self._start_verification_after_finish(cleaned)
            self._publish_status(force=True)

    def _gps_pose_cb(self, msg):
        with self.lock:
            p = msg.pose.pose.position
            self.gps_pose = (p.x, p.y, msg.header.frame_id)
            q = msg.pose.pose.orientation
            self.gps_pose_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.gps_pose_stamp = msg.header.stamp.to_sec()
            self.gps_pose_receipt = time.monotonic()
        self._maybe_start_recovery()

    def _keepout_cb(self, msg):
        if KeepoutGeometry is None or not self.retain_gps_route:
            return
        try:
            state = json.loads(msg.data)
            geometry = KeepoutGeometry.from_state(state)
            frame = state['frame_id']
        except (ValueError, TypeError, KeyError):
            return
        with self.lock:
            self.keepout = geometry
            self.keepout_frame = frame

    def _detections_cb(self, msg):
        if self.gps_fallback_enabled:
            with self.lock:
                if (msg.header.stamp.is_zero() or
                        (self.detection_stamp is not None and msg.header.stamp.to_sec() <= self.detection_stamp)):
                    return
        present = False
        observations = []
        verification_observations = []
        tracking_observations = []
        visible_ids = set()
        for item in msg.detections:
            detection = PixelDetection(
                item.class_id, item.class_name, float(item.confidence),
                item.bbox.x_offset, item.bbox.y_offset, item.bbox.width,
                item.bbox.height, item.anchor_px.x, item.anchor_px.y,
                track_id=item.track_id, depth_valid=item.depth_valid,
                depth_m=item.depth_m, range_m=item.range_m)
            if not recovery_depth_allowed(detection):
                continue
            try:
                validate_detection(detection, msg.image_width, msg.image_height)
            except ValueError:
                continue
            visible_ids.add(detection.track_id)
            if (detection.class_name in self.allowed_class_names
                    and detection.confidence >= self.min_confidence):
                # Reinspect valid detector output from the configured image ROI.
                # ZED currently publishes only lower-half detections.
                # The navigation goal's 0.3 m circle does not limit reinspection.
                # Missing depth may still prove a residual object; it must not count as an empty frame.
                verification_observations.append(detection)
            if detection.class_name in self.allowed_class_names:
                tracking_observations.append(detection)
            if (detection.class_name in self.allowed_class_names
                    and detection.confidence >= self.min_confidence
                    and (not self.select_nearest_by_depth or
                         (detection.depth_valid and detection.track_id > 0))):
                present = True
                observations.append(detection)
        with self.lock:
            if getattr(self, 'verification', None) and self.verification.phase == 'OBSERVING':
                age = rospy.Time.now().to_sec()-msg.header.stamp.to_sec()
                if 0 <= age <= .6:
                    self.verification.detections(msg.header.stamp.to_sec(), time.monotonic(), verification_observations)
            self.fod_target_present = present
            self.detection_receipt = time.monotonic()
            self.detection_stamp = msg.header.stamp.to_sec()
            self.visible_track_ids = visible_ids
        if self._use_gps_fallback():
            self._remember_fod_gps(msg, observations, tracking_observations)
            self._maybe_fallback_to_gps()
        self._maybe_start_recovery()

    def _camera_info_cb(self, msg):
        with self.lock:
            self.camera_info = msg

    def _set_verification_enabled_cb(self, request):
        with self.lock:
            if request.data and (self.mode not in (GPS_ACTIVE, FOD_COMPLETE_STOP, FOD_ABORTED) or self.recovery_armed):
                return SetBoolResponse(success=False, message='请在下一轮回收开始前开启倒退复查')
            self.verification_enabled = bool(request.data)
            if not request.data:
                self.verification_this_run = False
                if self.mode == FOD_VERIFY:
                    self._finish_verification_locked('CANCELED', '操作员停止倒退复查，保持停车')
            # Setting this option never arms recovery or executes an old completion.
            rospy.set_param('~verification_enabled', self.verification_enabled)
        self._publish_status(force=True)
        return SetBoolResponse(success=True, message=(f'下一轮回收结束后倒退 {self.verification.distance_m:g} 米并检查相机'
                                                      if request.data else '回收后倒退复查已关闭'))

    def _recovery_finished_locked(self, reason):
        self.recovery_armed = False
        self.command_source = STOP_SOURCE
        self.allow_move_base_goals = False
        self.reason = reason
        self.mode = FOD_COMPLETE_STOP
        if getattr(self, 'verification_this_run', False) and self.verification.phase == 'IDLE':
            self.verification.prepare()
            self.mode, self.reason = FOD_VERIFY, self.verification.reason
        self._publish_zero_locked('recovery travel completed')

    def _finish_verification_locked(self, phase, reason):
        self.verification.finish(phase, reason)
        self.mode, self.reason = FOD_COMPLETE_STOP, reason
        self.command_source, self.allow_move_base_goals = STOP_SOURCE, False
        self._publish_zero_locked(reason)
        rospy.loginfo('垃圾回收判定 %s: %s', phase, reason)

    def _start_verification_after_finish(self, cleaned):
        # Called under the existing transition lock, only after the normal cleanup.
        with self.lock:
            if self.mode != FOD_VERIFY or self.verification.phase != 'PREPARING':
                return
            generation = self.recovery_generation
            if not cleaned:
                self._finish_verification_locked('INCONCLUSIVE', '清扫停机/收杆指令未确认，不执行倒退复查')
                return
        success, message = self._call_set_bool(self.visual_enable_service, False)
        if success:
            success, message = self._pause_navigation_route(True)
            with self.lock:
                self.gps_paused = success
        self._cancel_all_move_base_goals()
        if success:
            success = self._wait_for_stopped_vehicle()
            message = '未确认车辆停车'
        with self.lock:
            if (self.mode != FOD_VERIFY or self.recovery_generation != generation
                    or self.recovery_cancel_requested.is_set() or self.shutdown_event.is_set()):
                return
            if not success or self.verification_odom is None:
                self._finish_verification_locked('INCONCLUSIVE', '复查准备失败：'+message)
            else:
                self.verification.begin_reverse(self.verification_odom, time.monotonic())
                self.reason = self.verification.reason
        self._publish_status(force=True)

    def _verification_scan_cb(self, msg):
        with self.lock:
            self.verification_scan, self.verification_scan_receipt = msg, time.monotonic()

    def _verification_chassis_cb(self, msg):
        with self.lock:
            self.verification_emergency = any((msg.hard_emergency, msg.soft_emergency,
                                               msg.gamepad_emergency, msg.robot_emergency))

    def _verification_image_cb(self, msg):
        with self.lock:
            current = self.verification
            if current.phase != 'OBSERVING':
                return
        age = rospy.Time.now().to_sec()-msg.header.stamp.to_sec()
        if not 0 <= age <= .6:
            return
        try:
            # Decode only during the two-second demonstration, not during navigation.
            import cv2
            import numpy as np
            image = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_GRAYSCALE)
            band = (image[round(image.shape[0]*self.verification_roi_top):
                          round(image.shape[0]*self.verification_roi_bottom)]
                    if image is not None else None)
            valid = (band is not None and band.size > 0
                     and not (band.std() < 1. and (band.mean() < 2. or band.mean() > 253.)))
        except Exception:
            valid = False
        with self.lock:
            if current is self.verification:
                current.image(msg.header.stamp.to_sec(), time.monotonic(), valid)

    def _verification_command_locked(self, now):
        check = self.verification
        if check.phase == 'PREPARING':
            return 0.
        ros_now = rospy.Time.now().to_sec()
        pose = self.verification_odom
        if (self.latest_odom_receipt is None or now-self.latest_odom_receipt > self.odom_timeout_sec
                or pose is None or not 0 <= ros_now-pose[4] <= self.odom_timeout_sec):
            self._finish_verification_locked('INCONCLUSIVE', '倒退复查里程计断流，保持停车')
            return 0.
        if self.verification_emergency:
            self._finish_verification_locked('INCONCLUSIVE', '底盘报告急停，复查结束且保持停车')
            return 0.
        speed = check.motion(pose, self.latest_linear_x, now, ros_now,
                             self.stop_linear_speed_mps, self.stop_confirm_sec)
        if check.phase in TERMINAL:
            self._finish_verification_locked(check.phase, check.reason)
            return 0.
        if speed:
            scan = self.verification_scan
            if (scan is None or self.verification_scan_receipt is None
                    or now-self.verification_scan_receipt > .6
                    or not 0 <= ros_now-scan.header.stamp.to_sec() <= .6):
                self._finish_verification_locked('INCONCLUSIVE', '倒退复查雷达断流，保持停车')
                return 0.
            clear, reason = rear_corridor_clear(scan, self.vehicle_footprint)
            if not clear:
                self._finish_verification_locked('INCONCLUSIVE', reason)
                return 0.
            geometry = self.keepout
            if geometry is not None and geometry.active:
                p, yaw = self.gps_pose, self.gps_pose_yaw
                if (p is None or yaw is None or p[2] != self.keepout_frame
                        or self.gps_pose_receipt is None or now-self.gps_pose_receipt > self.odom_timeout_sec
                        or not geometry.motion_clear(p[0], p[1], yaw, speed, 0.)):
                    self._finish_verification_locked('INCONCLUSIVE', '倒退复查触及地图禁区或定位无效，保持停车')
                    return 0.
        self.reason = check.reason
        return speed

    def _remember_fod_gps(self, msg, observations, tracking_observations):
        with self.lock:
            if not self._use_gps_fallback():
                return
            if not (self.recovery_armed or self.mode in (ENTERING_FOD, FOD_ACTIVE)):
                return
            if self.mode == GPS_ACTIVE:
                distance = self._recovery_distance_locked()
                if distance is None or distance >= self.recovery_radius_m:
                    return
            camera, pose, origin = self.camera_info, self.gps_pose, self.gps_origin
            generation = self.recovery_generation
            route = dict(self.gps_route)
        try:
            if camera is None or pose is None or origin is None:
                raise ValueError("等待相机内参、GPS 定位和原点")
            if (camera.header.frame_id != msg.header.frame_id or
                    (camera.width, camera.height) != (msg.image_width, msg.image_height)):
                raise ValueError("相机内参和检测坐标系/分辨率不一致")
            if msg.header.stamp.is_zero():
                raise ValueError("检测缺少源时间戳")
            if (not route.get("active") or not route.get("route_id")
                    or str(route.get("frame_id", "")).lstrip("/") != pose[2].lstrip("/")):
                raise ValueError("等待本次有效 GPS 最终目标；继续原导航")
            goal_xy = (float(route["final_x"]), float(route["final_y"]))
            if not all(math.isfinite(v) for v in goal_xy):
                raise ValueError("GPS 最终目标坐标无效；继续原导航")
            transform = self.tf_buffer.lookup_transform(
                pose[2], msg.header.frame_id, msg.header.stamp, rospy.Duration(0.08)).transform
            t, q = transform.translation, transform.rotation
            # Check every valid-depth observation before both acquisition and
            # same-ID visibility refresh. A side object cannot bypass the goal
            # match merely by inheriting a tracker ID or having low confidence.
            eligible = []
            world_points = []
            for d in tracking_observations:
                if not d.depth_valid:
                    # Only an already established target can use pixel-only
                    # observations; these never establish/refine a GPS point.
                    eligible.append(d)
                    continue
                p = depth_point_in_world(d, camera.P, (t.x,t.y,t.z), (q.x,q.y,q.z,q.w))
                if point_matches_recovery_goal(p, goal_xy):
                    eligible.append(d)
                    if d in observations and 0 < d.range_m <= 6.0:
                        world_points.append((d, p))
            # Reject the vehicle's own sweeper before selecting the nearest
            # FOD. Once acquired, a real FOD may enter the chassis envelope.
            acquiring = self.fod_gps_memory.point is None
            if acquiring:
                body = self.tf_buffer.lookup_transform(
                    "base_link", msg.header.frame_id, msg.header.stamp, rospy.Duration(0.08)).transform
                bt, bq = body.translation, body.rotation
                world_points = [(d, p) for d, p in world_points if not inside_vehicle_footprint(
                    depth_point_in_world(d, camera.P, (bt.x,bt.y,bt.z), (bq.x,bq.y,bq.z,bq.w)),
                    self.vehicle_footprint)]
            with self.lock:
                if (not self._use_gps_fallback() or generation != self.recovery_generation
                        or self.gps_route.get("route_id") != route["route_id"]
                        or not (self.recovery_armed or self.mode in (ENTERING_FOD, FOD_ACTIVE))):
                    return
                self.fod_reference_goal = dict(route_id=route["route_id"], frame_id=pose[2],
                                               x=goal_xy[0], y=goal_xy[1])
                if world_points and self.recovery_armed and self.mode == GPS_ACTIVE:
                    # Start the already configured 0.6 m/s pickup speed at
                    # the first eligible nearby FOD, before confirmation and
                    # the stopped-controller handoff consume its field of view.
                    self.fod_approach_started = True
                self.fod_gps_memory.observe(msg.header.stamp.to_sec(), world_points, (t.x,t.y))
                if self.fod_gps_memory.point is not None and not self.fod_gps_memory.frozen:
                    self.fod_gps_memory.match_visible(msg.header.stamp.to_sec(), eligible)
                self.fod_gps_point = self.fod_gps_memory.snapshot(*origin)
                self.fod_gps_reason = ("已记录原 GPS 目标 0.3 米内 FOD；丢失后穿越 0.8 米"
                                      if self.fod_gps_point else
                                      "等待原 GPS 目标 0.3 米内稳定 FOD：{}/{} 帧；继续原导航".format(
                                          len(self.fod_gps_memory.samples), self.fod_gps_memory.frames))
        except (ValueError, TypeError, KeyError, tf2_ros.TransformException) as exc:
            with self.lock:
                self.fod_gps_reason = "FOD GPS 记录：" + str(exc)

    def _prepare_gps_fallback_locked(self, reason="目标丢失"):
        if (not self._use_gps_fallback() or not self.fod_gps_point
                or not self.recovery_sweep_ready or self.recovery_cancel_requested.is_set()):
            return False
        reference = self.fod_reference_goal
        if not reference or not point_matches_recovery_goal(
                (self.fod_gps_point["fod_x"], self.fod_gps_point["fod_y"]),
                (reference["x"], reference["y"])):
            return False
        self.fod_gps_memory.frozen = True
        self.gps_fallback_target = dict(self.fod_gps_point)
        self.gps_fallback_reason = reason
        self.gps_fallback_route_id = self.gps_route.get("route_id")
        self.gps_fallback_sent = False
        self.gps_fallback_error = None
        self.gps_fallback_ack.clear()
        self.recovery_armed = False
        self.mode = ENTERING_GPS_RECOVERY
        self.command_source = STOP_SOURCE
        self.reason = reason + "，切换到记录 FOD 前方 0.8 米 GPS 终点"
        self._publish_zero_locked(self.reason)
        return True

    def _maybe_fallback_to_gps(self):
        with self.lock:
            if self.proximity_worker_started:
                return
            if self.mode not in (GPS_ACTIVE, FOD_ACTIVE):
                return
            if self.mode == GPS_ACTIVE and not self.recovery_armed:
                return
            # A target can vanish during the stopped handoff while it is
            # still inside the existing 1 s grace. Visual then reaches ACQUIRE
            # without a box; use the verified memory after that grace instead
            # of waiting at zero until the whole visual session times out.
            if self.mode == FOD_ACTIVE and self.visual_state not in ("ACQUIRE", "TRACK_WAIT", "GPS_HANDOFF"):
                return
            if not self.fod_gps_point or self.fod_gps_memory.recently_visible(
                    self.detection_stamp, self.target_loss_grace_sec):
                return
            if not self._prepare_gps_fallback_locked():
                return
        threading.Thread(target=self._gps_fallback_worker,
                         name="fod-gps-loss", daemon=True).start()

    def _gps_fallback_worker(self):
        with self.transition_lock:
            self._execute_gps_fallback()

    def _execute_gps_fallback(self):
        """Called with transition_lock held; no old GPS velocity may escape."""
        try:
            with self.lock:
                if self.mode != ENTERING_GPS_RECOVERY or self.recovery_cancel_requested.is_set():
                    return False, "GPS recovery canceled"
            success, message = self._call_set_bool(self.visual_enable_service, False)
            if not success:
                raise ValueError(message)
            if not self._wait_for_stopped_vehicle():
                raise ValueError("GPS 回收切换未确认停车")
            with self.lock:
                if self.mode != ENTERING_GPS_RECOVERY or self.recovery_cancel_requested.is_set():
                    return False, "GPS recovery canceled"
                self.allow_move_base_goals = True
            # The existing pause service resumes the retained route. Keep
            # STOP selected until our distinct replacement route is accepted.
            success, message = self._pause_navigation_route(False)
            if not success:
                raise ValueError(message)
            with self.lock:
                self.gps_paused = False
                if self.recovery_cancel_requested.is_set() or self.mode != ENTERING_GPS_RECOVERY:
                    return False, "GPS recovery canceled"
                target = self.gps_fallback_target
                goal = NavSatFix()
                goal.header.stamp = rospy.Time.now()
                goal.header.frame_id = "fod_gps_recovery"
                goal.latitude, goal.longitude = target["goal_latitude"], target["goal_longitude"]
                self.gps_fallback_sent = True
                self.gps_goal_pub.publish(goal)
            deadline = time.monotonic() + self.service_wait_timeout_sec
            while not self.gps_fallback_ack.wait(0.05):
                if self.recovery_cancel_requested.is_set() or self.shutdown_event.is_set():
                    return False, "GPS recovery canceled"
                if time.monotonic() >= deadline:
                    raise ValueError("FOD GPS 终点未被导航确认")
            with self.lock:
                if self.recovery_cancel_requested.is_set() or self.mode != ENTERING_GPS_RECOVERY:
                    return False, "GPS recovery canceled"
                if self.gps_fallback_error:
                    raise ValueError(self.gps_fallback_error)
                self.arbiter.clear(GPS_SOURCE)
                completed = self.gps_route.get("state") == "COMPLETE"
                self.mode = FOD_COMPLETE_STOP if completed else GPS_FOD_RECOVERY
                self.command_source = STOP_SOURCE if completed else GPS_SOURCE
                self.allow_move_base_goals = not completed
                self.reason = ("已到达记录 FOD 前方 0.8 米 GPS 终点" if completed else
                               self.gps_fallback_reason + "，按记录 GPS 点穿过 FOD 0.8 米；速度 0.6 m/s")
                if completed:
                    self._recovery_finished_locked(self.reason)
            if completed:
                cleaned = self._stop_recovery_sweep()
                self._start_verification_after_finish(cleaned)
            self._publish_status(force=True)
            return True, self.reason
        except ValueError as exc:
            self._set_mode(FOD_ABORTED, "GPS 回收接续失败：" + str(exc), STOP_SOURCE, False)
            success, _message = self._pause_navigation_route(True)
            with self.lock:
                self.gps_paused = success
            self._stop_recovery_sweep()
            return False, str(exc)

    def _use_gps_fallback(self):
        # Per-session selection only; do not change the GPS navigation profile.
        return self.gps_fallback_enabled and not getattr(self, 'direct_recovery', False)

    def _has_fod_locked(self):
        if self._use_gps_fallback():
            return self.fod_gps_point is not None and self.fod_gps_memory.recently_visible(
                self.detection_stamp, self.target_loss_grace_sec)
        if not self.fod_target_present or self.detection_receipt is None:
            return False
        # Presence follows the latest real frame, without an interruption
        # timeout. A subsequent empty frame still clears the target normally.
        return True

    def _recovery_distance_locked(self):
        if self.gps_pose is None or self.gps_pose_receipt is None:
            return None
        age = time.monotonic() - self.gps_pose_receipt
        source_age = rospy.Time.now().to_sec() - self.gps_pose_stamp
        if not (0.0 <= age <= self.odom_timeout_sec
                and 0.0 <= source_age <= self.odom_timeout_sec):
            return None
        return final_goal_distance(self.gps_route, *self.gps_pose)

    def _maybe_start_recovery(self):
        with self.lock:
            if (self.shutdown_event.is_set() or not self.recovery_armed
                    or self.mode != GPS_ACTIVE or self.proximity_worker_started):
                return
            distance = self._recovery_distance_locked()
            if distance is None or distance >= self.recovery_radius_m:
                return
            # Start sweeping and lower the assembly once on entry to the target area, even
            # without a camera/detection stream. GPS keeps command ownership.
            if (self.sweep_start_requested
                    and not (self.recovery_sweep_ready and self._has_fod_locked())):
                return
            self.proximity_worker_started = True
        threading.Thread(target=self._proximity_recovery_worker,
                         name="fod-goal-proximity", daemon=True).start()

    def _proximity_recovery_worker(self):
        try:
            with self.transition_lock:
                with self.lock:
                    # Recheck after acquiring the transition lock: the operator
                    # may have canceled recovery or replaced the GPS target.
                    distance = self._recovery_distance_locked()
                    if (not self.recovery_armed or self.mode != GPS_ACTIVE
                            or self.shutdown_event.is_set() or distance is None
                            or distance >= self.recovery_radius_m):
                        return
                    start_sweep = not self.sweep_start_requested
                    self.sweep_start_requested = True
                if start_sweep:
                    with self.lock:
                        self.sweep_status = "正在联动：液压杆升出 3 秒并启动清扫"
                    self._publish_status(force=True)
                    success, message = self._set_sweep(True)
                    with self.lock:
                        self.recovery_sweep_ready = success
                        self.reason = ("sweeper started and hydraulic extension acknowledged; GPS navigation continues until FOD is visible"
                                       if success else "sweeper start not confirmed; GPS navigation continues: " + message)
                    if not success:
                        rospy.logerr("%s", self.reason)
                        self._stop_recovery_sweep()
                        # Do not pause GPS or retry a failed pulse on every
                        # pose callback. Cancel/re-enable retries the request.
                        return
                with self.lock:
                    interrupted = (not self.recovery_armed or self.mode != GPS_ACTIVE
                                   or self.shutdown_event.is_set())
                if interrupted:
                    self._stop_recovery_sweep()
                    return
                with self.lock:
                    # GPS may have reached/replaced the goal while the TCP
                    # transaction was in flight. Recheck before visual takeover.
                    distance = self._recovery_distance_locked()
                    if (distance is None or distance >= self.recovery_radius_m
                            or not self.recovery_sweep_ready or not self._has_fod_locked()):
                        return
                    self.recovery_armed = False  # One recovery per button press.
                success, message = self._enter_fod_mode()
                if not success:
                    self._stop_recovery_sweep()
                (rospy.loginfo if success else rospy.logerr)("%s", message)
        finally:
            with self.lock:
                self.proximity_worker_started = False
            if self.gps_fallback_enabled:
                self._maybe_fallback_to_gps()
            self._publish_status(force=True)

    def _set_sweep(self, enabled):
        if not self.sweep_enabled:
            return True, "automatic sweeper and hydraulic control disabled"
        with self.lock:
            if enabled and self.recovery_sweep_ready:
                return True, self.sweep_status
            if enabled:
                self.sweep_cleanup_confirmed = False
                self.sweep_cleanup_pending = False
                self.sweep_cleanup_hydraulic_sent = False
                self.sweep_cleanup_hydraulic_acknowledged = False
            rod_sent = False if enabled else getattr(self, "sweep_cleanup_hydraulic_sent", False)
            rod_ack = False if enabled else getattr(self, "sweep_cleanup_hydraulic_acknowledged", False)
        success = False
        message = "清扫联动尚未确认"
        # Retry only the unfinished component. A motor retry always queries
        # the real state first; a rod pulse that may have been sent is never
        # repeated, because the controller has no rod-position feedback.
        for attempt in range(3):
            if enabled and attempt:
                cancel = getattr(self, "recovery_cancel_requested", None)
                shutdown = getattr(self, "shutdown_event", None)
                with self.lock:
                    ended = (getattr(self, "mode", None) == GPS_ACTIVE
                             and not getattr(self, "recovery_armed", True))
                if ended or (cancel and cancel.is_set()) or (shutdown and shutdown.is_set()):
                    message = "任务已结束或取消，不再补发启动；转入机构收尾"
                    break
            action = (("on" if enabled else "off") if rod_sent else
                      ("recovery-start" if enabled else "recovery-stop"))
            command = [sys.executable, self.sweep_script, action,
                       "--json", "--ip", self.sweep_ip, "--port", str(self.sweep_port)]
            if action in ("on", "off"):
                command.append("--verify-stable")
            try:
                result = subprocess.run(command, stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE, text=True, timeout=40.0)
                reply = json.loads(result.stdout)
                if not isinstance(reply, dict):
                    raise ValueError("invalid sweeper response")
                if action.startswith("recovery-"):
                    rod_sent = rod_sent or reply.get("hydraulic_sent") is True
                    rod_ack = rod_ack or (reply.get("hydraulic_sent") is True
                                          and reply.get("acknowledged") is True)
                motor_ok = (reply.get("state") == int(enabled)
                            and reply.get("motor_confirmed", reply.get("success")) is True)
                success = result.returncode == 0 and reply.get("success") is True and rod_ack and motor_ok
                message = str(reply.get("message", "清扫与液压联动未返回操作结果"))
                with self.lock:
                    if enabled and (rod_sent or reply.get("motor_command_sent") is True):
                        self.sweep_started_for_recovery = True
                if success:
                    message = ("清扫电机已持续回读确认为{}；液压杆{} 3 秒指令已应答、计时已结束"
                               "（无杆位置回读）").format("开启" if enabled else "关闭",
                                                        "升出" if enabled else "回收")
                elif rod_sent and not rod_ack:
                    message += "；液压指令可能已执行但应答未确认，不重复杆动作"
                    # Acknowledge uncertainty without suppressing the motor
                    # shutdown. Further motor-only attempts are still allowed.
                    if enabled or motor_ok:
                        break
            except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
                if not isinstance(exc, OSError):
                    # A killed/crashed child may already have moved the rod.
                    # Preserve ownership and never repeat an uncertain pulse.
                    if action.startswith("recovery-"):
                        rod_sent = True
                    with self.lock:
                        if enabled:
                            self.sweep_started_for_recovery = True
                message = "清扫联动通信未确认：{}".format(exc)
                if enabled and rod_sent and not rod_ack:
                    break
            finally:
                with self.lock:
                    if not enabled:
                        self.sweep_cleanup_hydraulic_sent = rod_sent
                        self.sweep_cleanup_hydraulic_acknowledged = rod_ack
                    self.sweep_operation_attempts = attempt + 1
            if success:
                break
            rospy.logwarn("Sweeper %s attempt %d/3 not confirmed: %s", action, attempt + 1, message)
            if attempt < 2:
                time.sleep(0.3)
        with self.lock:
            if enabled:
                self.recovery_sweep_ready = success
            elif success:
                self.sweep_started_for_recovery = False
            self.sweep_status = message
        return success, message

    def _stop_recovery_sweep(self):
        with self.lock:
            owned = self.sweep_started_for_recovery and self.sweep_auto_stop
            self.recovery_sweep_ready = False
            if owned:
                self.sweep_cleanup_pending = True
        if owned:
            # Keep the cleanup obligation until both components are confirmed.
            # Partial results remember the rod pulse so another finish/cancel
            # can retry the motor without issuing another 3-second rod pulse.
            success, message = self._set_sweep(False)
            self.sweep_cleanup_confirmed = success
            self.sweep_cleanup_pending = not success
            if not success:
                rospy.logerr("Recovery sweeper cleanup remains unconfirmed: %s", message)
            return success
        return getattr(self, 'sweep_cleanup_confirmed', True)

    def _sweep_abort_worker(self):
        with self.transition_lock:
            with self.lock:
                if self.mode not in (FOD_ABORTED, FOD_COMPLETE_STOP, FOD_VERIFY):
                    return
            cleaned = self._stop_recovery_sweep()
            self._start_verification_after_finish(cleaned)

    def _resume_waiting_for_target(self):
        with self.lock:
            fallback = self._prepare_gps_fallback_locked()
        if fallback:
            return self._execute_gps_fallback()
        success, message = self._return_to_gps(
            "FOD disappeared before visual handoff", keep_sweep=True)
        if success:
            with self.lock:
                self.recovery_armed = True
                self.reason = "camera recovery armed; no visible FOD, GPS continues with sweeping and hydraulic ready"
            self._publish_status(force=True)
            message = self.reason
        else:
            self._stop_recovery_sweep()
        return success, message

    def _move_base_goal_cb(self, msg):
        with self.lock:
            permitted = self.allow_move_base_goals
        if permitted:
            return
        cancel = GoalID()
        cancel.id = msg.goal_id.id
        cancel.stamp = rospy.Time()
        self.cancel_pub.publish(cancel)
        rospy.logerr(
            "Canceled move_base goal %s because GPS navigation is paused",
            msg.goal_id.id or "<empty>",
        )

    def _visual_state_cb(self, msg):
        state = str(msg.data).strip().upper()
        stop_sweep = False
        continue_gps = False
        with self.lock:
            self.visual_state = state
            if state == "GPS_HANDOFF" and self.mode == FOD_ACTIVE:
                continue_gps = self._prepare_gps_fallback_locked("视觉接续记录位置")
            elif state == "COMPLETE" and self.mode == FOD_ACTIVE:
                self._recovery_finished_locked("one FOD recovery completed; stopped until an explicit operator request")
                stop_sweep = True
            elif state == "ABORT" and self.mode in (ENTERING_FOD, FOD_ACTIVE):
                self.mode = FOD_ABORTED
                self.reason = (
                    "visual controller aborted; GPS remains paused until "
                    "an explicit stop/return command"
                )
                self.command_source = STOP_SOURCE
                self.allow_move_base_goals = False
                self._publish_zero_locked("visual ABORT stop")
                stop_sweep = True
        self._publish_status(force=True)
        if continue_gps:
            threading.Thread(target=self._gps_fallback_worker,
                             name="fod-gps-approach-limit", daemon=True).start()
        if stop_sweep:
            threading.Thread(target=self._sweep_abort_worker,
                             name="fod-sweep-abort", daemon=True).start()

    def _call_set_bool(self, service_name, enabled):
        try:
            rospy.wait_for_service(
                service_name, timeout=self.service_wait_timeout_sec
            )
            response = rospy.ServiceProxy(service_name, SetBool)(enabled)
        except (rospy.ROSException, rospy.ServiceException) as exc:
            return False, "{} call failed: {}".format(service_name, exc)
        return bool(response.success), str(response.message)

    def _fault_stop_worker(self):
        with self.transition_lock:
            self._call_set_bool(self.visual_enable_service, False)
            self._stop_recovery_sweep()
            success, _message = self._pause_navigation_route(True)
            if success:
                with self.lock:
                    self.gps_paused = True

    def _pause_navigation_route(self, paused):
        if self.retain_gps_route:
            return self._call_set_bool(self.gps_pause_service, paused)
        # Local move_base goals are canceled by the existing handoff. There is
        # no stored GPS route to pause/resume or artificial GPS origin to set.
        return True, "Indoor local navigation has no retained GPS route"

    def _graph_watchdog(self, _event=None):
        if self.shutdown_event.is_set():
            return
        graph_error = self._output_route_error()
        if not graph_error:
            return
        start_worker = False
        with self.lock:
            if self.mode == FAULT_STOP and self.reason == graph_error:
                return
            if self.mode == FOD_VERIFY:
                self.verification.finish('INCONCLUSIVE', '控制链故障：'+graph_error)
            self.mode = FAULT_STOP
            self.recovery_armed = False
            self.reason = graph_error
            self.command_source = STOP_SOURCE
            self.allow_move_base_goals = False
            self._publish_zero_locked("command-graph fault stop")
            if not self.fault_worker_started:
                self.fault_worker_started = True
                start_worker = True
        self._cancel_all_move_base_goals()
        self._publish_status(force=True)
        rospy.logfatal("GPS/FOD command graph fault: %s", graph_error)
        if start_worker:
            threading.Thread(
                target=self._fault_stop_worker,
                name="fod-command-graph-fault",
                daemon=True,
            ).start()

    def _set_mode(self, mode, reason, source, allow_move_base_goals):
        with self.lock:
            self.mode = mode
            self.reason = reason
            self.command_source = source
            self.allow_move_base_goals = allow_move_base_goals
            if source == STOP_SOURCE:
                self._publish_zero_locked("mode transition/fault stop")
        self._publish_status(force=True)

    def _cancel_all_move_base_goals(self):
        self.cancel_pub.publish(GoalID())

    def _wait_for_stopped_vehicle(self):
        deadline = time.monotonic() + self.transition_timeout_sec
        stable_since = None
        while time.monotonic() < deadline and not self.shutdown_event.is_set():
            now = time.monotonic()
            with self.lock:
                receipt = self.latest_odom_receipt
                linear_x = self.latest_linear_x
                angular_z = self.latest_angular_z
            age = math.inf if receipt is None else now - receipt
            stopped = (
                linear_x is not None
                and angular_z is not None
                and stopped_sample_is_valid(
                    linear_x,
                    angular_z,
                    age,
                    self.odom_timeout_sec,
                    self.stop_linear_speed_mps,
                    self.stop_angular_speed_rps,
                )
            )
            if stopped:
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= self.stop_confirm_sec:
                    return True
            else:
                stable_since = None
            self.shutdown_event.wait(0.05)
        return False

    def _enter_fod_mode(self):
        graph_error = self._output_route_error()
        if graph_error:
            self._set_mode(FAULT_STOP, graph_error, STOP_SOURCE, False)
            return False, graph_error

        self._set_mode(
            ENTERING_FOD,
            "blocking GPS commands and requesting a retained-route pause",
            STOP_SOURCE,
            False,
        )
        with self.lock:
            self.arbiter.clear(FOD_SOURCE)

        success, message = self._pause_navigation_route(True)
        if not success:
            reason = (
                "GPS pause failed or was uncertain: {}; chassis remains stopped"
            ).format(message)
            self._set_mode(
                FOD_ABORTED,
                reason,
                STOP_SOURCE,
                False,
            )
            return False, reason
        with self.lock:
            self.gps_paused = True
        self._cancel_all_move_base_goals()

        if not self._wait_for_stopped_vehicle():
            reason = (
                "vehicle did not provide fresh stopped odometry within {:.1f}s; "
                "GPS remains paused"
            ).format(self.transition_timeout_sec)
            self._set_mode(FOD_ABORTED, reason, STOP_SOURCE, False)
            return False, reason

        with self.lock:
            if self.recovery_cancel_requested.is_set():
                return False, "recovery canceled before actuator startup"
            target_missing = (self.retain_gps_route and not self.direct_recovery
                              and not self._has_fod_locked())
        if target_missing:
            return self._resume_waiting_for_target()

        # The proximity worker already confirmed both actuators. Reopening
        # the TCP connection here used to stall a stopped visual handoff.
        with self.lock:
            already_ready = self.recovery_sweep_ready
        success, message = ((True, self.sweep_status) if already_ready else self._set_sweep(True))
        if not success:
            reason = "recovery sweeper start / hydraulic extension not confirmed: {}".format(message)
            self._set_mode(FOD_ABORTED, reason, STOP_SOURCE, False)
            self._stop_recovery_sweep()
            return False, reason
        with self.lock:
            still_entering = self.mode == ENTERING_FOD and not self.shutdown_event.is_set()
        if not still_entering:
            self._stop_recovery_sweep()
            return False, "visual handoff interrupted"
        with self.lock:
            target_missing = (self.retain_gps_route and not self.direct_recovery
                              and not self._has_fod_locked())
        if target_missing:
            return self._resume_waiting_for_target()

        # Visual acquisition and the GPS memory must refer to the same FOD,
        # even if another box becomes nearer while the chassis is stopping.
        preferred = self.fod_gps_memory.track_id if self._use_gps_fallback() else 0
        rospy.set_param(self.visual_enable_service.rsplit("/", 1)[0] + "/preferred_track_id", preferred)
        rospy.set_param(self.visual_enable_service.rsplit("/", 1)[0] + "/preferred_target",
                        vars(self.fod_gps_memory.last_detection)
                        if preferred and self.fod_gps_memory.last_detection is not None else {})
        rospy.set_param(self.visual_enable_service.rsplit("/", 1)[0] + "/gps_handoff_on_approach_limit",
                        bool(self._use_gps_fallback() and self.fod_gps_point))
        rospy.set_param(self.visual_enable_service.rsplit("/", 1)[0] + "/recovery_reference_goal",
                        self.fod_reference_goal if self._use_gps_fallback() else {})
        success, message = self._call_set_bool(self.visual_enable_service, True)
        if not success:
            reason = "visual controller enable failed: {}; GPS remains paused".format(
                message
            )
            self._set_mode(FOD_ABORTED, reason, STOP_SOURCE, False)
            self._stop_recovery_sweep()
            return False, reason

        with self.lock:
            if self.mode != ENTERING_FOD or self.visual_state == "ABORT":
                return False, self.reason
            self.mode = FOD_ACTIVE
            self.reason = (
                "独立相机回收：锁定最近有效 FOD，视觉靠近；原 GPS 路线保持暂停"
                if self.direct_recovery else
                "visual controller owns the chassis; GPS route is retained"
            )
            self.command_source = FOD_SOURCE
            self.allow_move_base_goals = False
        self._publish_status(force=True)
        return True, (
            "Single FOD recovery enabled; GPS stays paused after visual COMPLETE "
            "until an explicit operator return command"
        )

    def _return_to_gps(self, reason_prefix, keep_sweep=False):
        with self.lock:
            self.recovery_armed = False
        self._set_mode(
            RETURNING_GPS,
            "{}; disabling visual motion and confirming stop".format(reason_prefix),
            STOP_SOURCE,
            False,
        )

        success, message = self._call_set_bool(self.visual_enable_service, False)
        if not keep_sweep:
            self._stop_recovery_sweep()
        if not success:
            reason = "cannot return to GPS because visual disable failed: {}".format(
                message
            )
            self._set_mode(FOD_ABORTED, reason, STOP_SOURCE, False)
            return False, reason

        if not self._wait_for_stopped_vehicle():
            reason = (
                "cannot return to GPS: fresh stopped odometry was not confirmed "
                "within {:.1f}s"
            ).format(self.transition_timeout_sec)
            self._set_mode(FOD_ABORTED, reason, STOP_SOURCE, False)
            return False, reason

        graph_error = self._output_route_error()
        if graph_error:
            self._set_mode(FAULT_STOP, graph_error, STOP_SOURCE, False)
            return False, graph_error

        with self.lock:
            self.arbiter.clear(GPS_SOURCE)
            # The resumed long-range manager publishes a new action goal from
            # the current position. Permit that one while chassis output stays
            # hard-stopped until the service succeeds.
            self.allow_move_base_goals = True
        success, message = self._pause_navigation_route(False)
        if not success:
            with self.lock:
                self.allow_move_base_goals = False
            self._cancel_all_move_base_goals()
            reason = "GPS resume failed: {}; chassis remains stopped".format(message)
            self._set_mode(FOD_ABORTED, reason, STOP_SOURCE, False)
            return False, reason

        with self.lock:
            self.gps_paused = False
            self.fault_worker_started = False
            self.direct_recovery = False
        self._set_mode(
            GPS_ACTIVE,
            "GPS navigation resumed from the post-recovery vehicle position",
            GPS_SOURCE,
            True,
        )
        return True, "GPS navigation resumed; visual controller is in standby"

    def _prepare_gps_goal_cb(self, _request):
        """An explicit new goal releases completed recovery, without resuming old GPS."""
        with self.transition_lock:
            with self.lock:
                mode = self.mode
            if mode == GPS_ACTIVE:
                return TriggerResponse(success=True, message="GPS already accepts new goals")
            if mode != FOD_COMPLETE_STOP:
                return TriggerResponse(success=False,
                                       message="cannot replace GPS goal from " + mode)
            if self.retain_gps_route:
                try:
                    rospy.wait_for_service(self.gps_clear_service,
                                           timeout=self.service_wait_timeout_sec)
                    response = rospy.ServiceProxy(self.gps_clear_service, Trigger)()
                except (rospy.ROSException, rospy.ServiceException) as exc:
                    return TriggerResponse(success=False, message=str(exc))
                if not response.success:
                    return TriggerResponse(success=False, message=response.message)
            success, message = self._return_to_gps("explicit new GPS goal after recovery")
            return TriggerResponse(success=success, message=message)

    def _set_direct_fod_enabled_cb(self, request):
        """Same servo/actuators/arbiter, explicitly started without a GPS goal."""
        if not request.data:
            with self.lock:
                if not self.direct_recovery:
                    return SetBoolResponse(success=True, message="独立相机回收未开启")
                # Stop output immediately, including during the 3 s rod action.
                self.recovery_cancel_requested.set()
                self.recovery_generation += 1
                self.recovery_armed = False
                if self.mode == FOD_VERIFY:
                    self.verification.finish('CANCELED', '操作员停止独立相机回收')
                self.mode, self.command_source = FOD_ABORTED, STOP_SOURCE
                self.allow_move_base_goals = False
                self.reason = "独立相机回收已停止，正在停清扫并收杆；GPS 保持暂停"
                self._publish_zero_locked(self.reason)
            self._publish_status(force=True)
            with self.transition_lock:
                disabled, message = self._call_set_bool(self.visual_enable_service, False)
                cleaned = self._stop_recovery_sweep()
                paused, pause_message = self._pause_navigation_route(True)
                self._cancel_all_move_base_goals()
                stopped = self._wait_for_stopped_vehicle()
                success = disabled and cleaned and paused and stopped
                with self.lock:
                    self.gps_paused = paused
                    self.reason = ("独立相机回收已停止并收尾，保持停车；不会恢复旧 GPS 路线" if success else
                                   "已禁止回收运动，收尾待确认：{}；{}；{}".format(
                                       message, self.sweep_status, pause_message))
                self._publish_status(force=True)
                return SetBoolResponse(success=success, message=self.reason)

        with self.transition_lock:
            with self.lock:
                if self.direct_recovery and self.mode in (ENTERING_FOD, FOD_ACTIVE, FOD_VERIFY):
                    return SetBoolResponse(success=True, message="独立相机回收正在执行")
                restart = self.direct_recovery and self.mode in (FOD_COMPLETE_STOP, FOD_ABORTED)
                if (self.mode != GPS_ACTIVE and not restart) or self.recovery_armed:
                    return SetBoolResponse(success=False, message="请先结束当前任务；当前状态 " + self.mode)
            # On a second press after completion, reset the SAME controller.
            # Finish any outstanding actuator cleanup before a new cycle.
            if restart:
                if not self._stop_recovery_sweep():
                    return SetBoolResponse(success=False, message=self.sweep_status)
                success, message = self._call_set_bool(self.visual_enable_service, False)
                if not success:
                    return SetBoolResponse(success=False, message=message)
            with self.lock:
                self.direct_recovery = True
                self.recovery_generation += 1
                self.recovery_cancel_requested.clear()
                self.verification_this_run = self.verification_enabled
                self.verification = RecoveryVerification()
                self.fod_gps_memory.reset()
                self.fod_gps_point = None
                self.fod_reference_goal = None
                self.fod_approach_started = False
                self.gps_fallback_target = None
                self.gps_fallback_route_id = None
                self.sweep_start_requested = False
                self.recovery_sweep_ready = False
            success, message = self._enter_fod_mode()
            return SetBoolResponse(success=success, message=message)

    def _set_fod_enabled_cb(self, request):
        if not request.data:
            # Cancel is visible to an in-flight loss handoff before it can
            # submit a GPS target, even while this service waits for the lock.
            self.recovery_cancel_requested.set()
            with self.lock:
                if self.mode == FOD_VERIFY:
                    self._finish_verification_locked('CANCELED', '操作员取消倒退复查')
        with self.transition_lock:
            with self.lock:
                mode = self.mode
            if request.data:
                if mode in (FOD_ACTIVE, ENTERING_GPS_RECOVERY, GPS_FOD_RECOVERY):
                    return SetBoolResponse(
                        success=True,
                        message="FOD recovery is already active",
                    )
                if mode != GPS_ACTIVE:
                    return SetBoolResponse(
                        success=False,
                        message=(
                            "cannot enter FOD mode from {}; call with data:false "
                            "to return/reset first"
                        ).format(mode),
                    )
                if not self.recovery_armed:
                    self.direct_recovery = False
                    self.verification_this_run = self.verification_enabled
                    self.verification = RecoveryVerification()
                    if not self.retain_gps_route:
                        self.recovery_cancel_requested.clear()
                        self.recovery_generation += 1
                if self.retain_gps_route:
                    if self.gps_fallback_enabled:
                        try:
                            origin = (float(rospy.get_param("/gps/origin_lat")),
                                      float(rospy.get_param("/gps/origin_lon")),
                                      float(rospy.get_param("/gps_long_range_goal_manager/yaw_offset", 0.0)) +
                                      math.radians(float(rospy.get_param("/gps_long_range_goal_manager/yaw_offset_deg", 0.0))))
                        except (KeyError, TypeError, ValueError):
                            origin = None
                    with self.lock:
                        if not self.recovery_armed:
                            self.recovery_generation += 1
                            self.recovery_cancel_requested.clear()
                            self.fod_gps_memory.reset()
                            self.fod_gps_point = None
                            self.fod_reference_goal = None
                            self.fod_approach_started = False
                            self.gps_fallback_target = None
                            self.gps_fallback_route_id = None
                            self.gps_origin = origin if self.gps_fallback_enabled else None
                            self.sweep_start_requested = False
                            self.recovery_sweep_ready = False
                        self.recovery_armed = True
                        self.reason = "回收已开启：目标小于 {:.1f} 米升杆 3 秒并启动清扫；只锁定原 GPS 目标 0.3 米内 FOD，丢失后穿越 0.8 米".format(self.recovery_radius_m)
                        if not self.gps_route.get("active", False):
                            self.reason = "回收已开启，等待新的有效 GPS 导航目标；已结束的旧目标不触发回收"
                    self._publish_status(force=True)
                    self._maybe_start_recovery()
                    return SetBoolResponse(success=True, message=self.reason)
                success, message = self._enter_fod_mode()
                return SetBoolResponse(success=success, message=message)

            if mode == GPS_ACTIVE:
                with self.lock:
                    self.recovery_armed = False
                    self.reason = "camera recovery disabled; normal GPS navigation"
                self._stop_recovery_sweep()
                self._publish_status(force=True)
                return SetBoolResponse(
                    success=True,
                    message="pending camera recovery canceled; GPS navigation continues",
                )
            if mode in (ENTERING_GPS_RECOVERY, GPS_FOD_RECOVERY):
                self._cancel_all_move_base_goals()
            success, message = self._return_to_gps(
                "explicit operator request to leave FOD mode"
            )
            return SetBoolResponse(success=success, message=message)

    def _publish_command(self, _event=None):
        now = time.monotonic()
        with self.lock:
            linear_x, angular_z, output_reason = self.arbiter.sample(
                self.command_source, now
            )
            if self.mode == FOD_VERIFY:
                linear_x = self._verification_command_locked(now)
                angular_z = 0.0
                output_reason = self.verification.reason
            if (self.command_source == GPS_SOURCE and
                    (self.mode == GPS_FOD_RECOVERY or
                     (self.recovery_armed and self.gps_fallback_enabled and
                      (self.fod_gps_point or self.fod_approach_started)))):
                # The requested pickup speed also applies to GPS fallback;
                # scale yaw with speed to retain Ackermann curvature.
                scale = min(1.0, 0.6 / max(abs(linear_x), 1e-9))
                linear_x *= scale
                angular_z *= scale
            # Explicit GPS keepout mode also covers visual approach/0.8 m
            # collection, which bypasses the TEB costmaps. No effect when off.
            geometry = getattr(self, "keepout", None)
            if self.command_source == FOD_SOURCE and geometry is not None and geometry.active:
                pose = self.gps_pose
                yaw = getattr(self, "gps_pose_yaw", None)
                if (pose is None or yaw is None or pose[2] != self.keepout_frame
                        or not geometry.motion_clear(pose[0], pose[1], yaw, linear_x, angular_z)):
                    linear_x = angular_z = 0.0
                    output_reason = "GPS 禁入区：视觉回收路径触及车体预留范围"
            self.last_output_reason = output_reason
            command = Twist()
            command.linear.x = linear_x
            command.angular.z = angular_z
            # Publication is serialized with every source change. An old
            # sampled command therefore cannot overtake a transition stop.
            self.cmd_pub.publish(command)

    def _publish_zero_locked(self, reason):
        """Publish a stop while holding the same lock as the periodic output."""
        self.last_output_reason = reason
        self.cmd_pub.publish(Twist())

    def _status_dictionary(self):
        now = time.monotonic()
        with self.lock:
            odom_age = (
                None
                if self.latest_odom_receipt is None
                else max(0.0, now - self.latest_odom_receipt)
            )
            gps_age = self.arbiter.age(GPS_SOURCE, now)
            fod_age = self.arbiter.age(FOD_SOURCE, now)
            return {
                "state": self.mode,
                "reason": self.reason,
                "command_source": "VERIFICATION" if self.mode == FOD_VERIFY and self.verification.phase == 'REVERSING' else self.command_source,
                "output_reason": self.last_output_reason,
                "gps_paused": self.gps_paused,
                "retain_gps_route": self.retain_gps_route,
                "direct_recovery_supported": True,
                "direct_recovery": self.direct_recovery,
                "recovery_armed": self.recovery_armed,
                "recovery_radius_m": self.recovery_radius_m,
                "final_goal_distance_m": self._recovery_distance_locked(),
                "fod_target_available": self._has_fod_locked(),
                "gps_fallback_enabled": self._use_gps_fallback(),
                "fod_gps_point": self.fod_gps_point,
                "fod_goal_radius_m": FOD_GOAL_RADIUS_M,
                "fod_reference_goal": self.fod_reference_goal,
                "fod_gps_reason": self.fod_gps_reason,
                "gps_fallback_route_id": self.gps_fallback_route_id,
                "sweep_status": self.sweep_status,
                "sweep_motor_control": "automatic" if self.sweep_enabled else "disabled",
                "sweep_started_for_recovery": self.sweep_started_for_recovery,
                "sweep_start_requested": self.sweep_start_requested,
                "recovery_sweep_ready": self.recovery_sweep_ready,
                "sweep_cleanup_pending": self.sweep_cleanup_pending,
                "sweep_cleanup_confirmed": self.sweep_cleanup_confirmed,
                "sweep_cleanup_hydraulic_sent": self.sweep_cleanup_hydraulic_sent,
                "sweep_cleanup_hydraulic_acknowledged": self.sweep_cleanup_hydraulic_acknowledged,
                "sweep_operation_attempts": self.sweep_operation_attempts,
                "visual_state": self.visual_state,
                "move_base_goals_allowed": self.allow_move_base_goals,
                "odom_age_sec": None if odom_age is None else round(odom_age, 3),
                "measured_linear_x": self.latest_linear_x,
                "measured_angular_z": self.latest_angular_z,
                "verification_enabled": self.verification_enabled,
                "verification_this_run": self.verification_this_run,
                "verification": self.verification.status(),
                "gps_command_age_sec": (
                    None if gps_age is None else round(max(0.0, gps_age), 3)
                ),
                "fod_command_age_sec": (
                    None if fod_age is None else round(max(0.0, fod_age), 3)
                ),
            }

    def _publish_status(self, _event=None, force=False):
        del force
        try:
            status = self._status_dictionary()
            self.state_pub.publish(String(data=status["state"]))
            self.status_pub.publish(
                String(data=json.dumps(status, ensure_ascii=False, sort_keys=True))
            )
            self.verification_pub.publish(String(data=json.dumps(
                dict(self.verification.status(), enabled=self.verification_enabled,
                     task_generation=self.recovery_generation, source_ros_ns=rospy.Time.now().to_nsec()),
                ensure_ascii=False, sort_keys=True)))
        except (AttributeError, rospy.ROSException):
            if not rospy.is_shutdown():
                raise

    def _shutdown(self):
        if self.shutdown_event.is_set():
            return
        self.shutdown_event.set()
        with self.lock:
            self.recovery_armed = False
            self.command_source = STOP_SOURCE
            self.allow_move_base_goals = False
            self.arbiter.clear()
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            try:
                self.cmd_pub.publish(Twist())
            except rospy.ROSException:
                break
            time.sleep(1.0 / self.publish_rate_hz)
        # Let any in-flight start finish/cancel before the final cleanup, so
        # shutdown cannot miss a pulse whose subprocess has not replied yet.
        with self.transition_lock:
            self._stop_recovery_sweep()


def main():
    rospy.init_node("fod_navigation_mode", anonymous=False)
    try:
        FodNavigationModeManager()
    except (TypeError, ValueError) as exc:
        rospy.logfatal("Invalid GPS/FOD mode-manager configuration: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
