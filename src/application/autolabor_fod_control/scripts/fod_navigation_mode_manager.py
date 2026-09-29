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
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse
from autolabor_fod_msgs.msg import FodDetectionArray
from autolabor_fod_control.visual_servo import PixelDetection, validate_detection

from autolabor_fod_control.mode_manager import (
    FOD_SOURCE,
    GPS_SOURCE,
    STOP_SOURCE,
    CommandArbiter,
    final_goal_distance,
    stopped_sample_is_valid,
)


GPS_ACTIVE = "GPS_ACTIVE"
ENTERING_FOD = "ENTERING_FOD"
FOD_ACTIVE = "FOD_ACTIVE"
FOD_COMPLETE_STOP = "FOD_COMPLETE_STOP"
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
        # Indoor LiDAR navigation has no retained WGS84 route. The existing
        # command arbitration, goal cancellation and stopped-vehicle handoff
        # still apply; only the absent GPS route service is skipped.
        self.retain_gps_route = _strict_bool_param("~retain_gps_route", True)
        self.gps_route_status_topic = rospy.get_param(
            "~gps_route_status_topic", "/gps/long_range/status")
        self.gps_pose_topic = rospy.get_param("~gps_pose_topic", "/gps/odom")
        self.recovery_radius_m = float(rospy.get_param("~recovery_radius_m", 5.0))
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
        self.last_output_reason = "waiting for first GPS command"
        self.fault_worker_started = False
        self.recovery_armed = False
        self.proximity_worker_started = False
        self.gps_route = {}
        self.gps_pose = None
        self.gps_pose_receipt = None
        self.gps_pose_stamp = None
        self.sweep_started_for_recovery = False
        self.sweep_start_requested = False
        self.recovery_sweep_ready = False
        self.sweep_status = "not requested"
        self.fod_target_present = False
        self.detection_receipt = None
        self.detection_stamp = None

        self.master = rosgraph.Master(rospy.get_name())
        self.master_lock = threading.Lock()
        self.master_pid = self.master.getPid()

        self.cmd_pub = rospy.Publisher(
            self.output_cmd_topic, Twist, queue_size=1
        )
        self.cancel_pub = rospy.Publisher(
            self.move_base_cancel_topic, GoalID, queue_size=10
        )
        self.state_pub = rospy.Publisher("~state", String, queue_size=1, latch=True)
        self.status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )

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
        self.mode_service = rospy.Service(
            "~set_fod_enabled", SetBool, self._set_fod_enabled_cb
        )

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
            arrived = (route.get("state") == "COMPLETE"
                       and self.gps_route.get("active", False))
            self.gps_route = route
            if (self.recovery_armed and self.mode == GPS_ACTIVE
                    and not route.get("active", False)
                    and (arrived or route.get("state") in ("CANCELED", "ABORTED", "SUPERSEDED"))):
                self.recovery_armed = False
                self.reason = "GPS target ended; camera recovery finished"
                finish_gps_recovery = True
        if finish_gps_recovery:
            threading.Thread(target=self._finish_gps_recovery_worker,
                             name="fod-gps-sweep-finish", daemon=True).start()
        self._maybe_start_recovery()

    def _finish_gps_recovery_worker(self):
        with self.transition_lock:
            with self.lock:
                # A newer operator request may already own this same sweeper.
                if self.mode != GPS_ACTIVE or self.recovery_armed:
                    return
            self._stop_recovery_sweep()
            self._publish_status(force=True)

    def _gps_pose_cb(self, msg):
        with self.lock:
            p = msg.pose.pose.position
            self.gps_pose = (p.x, p.y, msg.header.frame_id)
            self.gps_pose_stamp = msg.header.stamp.to_sec()
            self.gps_pose_receipt = time.monotonic()
        self._maybe_start_recovery()

    def _detections_cb(self, msg):
        present = False
        for item in msg.detections:
            detection = PixelDetection(
                item.class_id, item.class_name, float(item.confidence),
                item.bbox.x_offset, item.bbox.y_offset, item.bbox.width,
                item.bbox.height, item.anchor_px.x, item.anchor_px.y,
                track_id=item.track_id, depth_valid=item.depth_valid,
                depth_m=item.depth_m, range_m=item.range_m)
            try:
                validate_detection(detection, msg.image_width, msg.image_height)
            except ValueError:
                continue
            if (detection.class_name in self.allowed_class_names
                    and detection.confidence >= self.min_confidence
                    and (not self.select_nearest_by_depth or
                         (detection.depth_valid and detection.track_id > 0))):
                present = True
                break
        with self.lock:
            self.fod_target_present = present
            self.detection_receipt = time.monotonic()
            self.detection_stamp = msg.header.stamp.to_sec()
        self._maybe_start_recovery()

    def _has_fod_locked(self):
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
            # Start the sweeper once on entry to the GPS target area, even
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
                        self.sweep_status = "正在启动清扫装置"
                    self._publish_status(force=True)
                    success, message = self._set_sweep(True)
                    with self.lock:
                        self.recovery_sweep_ready = success
                        self.reason = ("sweeper started; GPS navigation continues until FOD is visible"
                                       if success else "sweeper start failed; GPS navigation continues: " + message)
                    if not success:
                        rospy.logerr("%s", self.reason)
                        # Do not pause GPS or retry a failed toggle on every
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
            self._publish_status(force=True)

    def _set_sweep(self, enabled):
        if not self.sweep_enabled:
            return True, "automatic sweeper control disabled"
        try:
            result = subprocess.run(
                [sys.executable, self.sweep_script, "on" if enabled else "off",
                 "--json", "--ip", self.sweep_ip, "--port", str(self.sweep_port)],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=20.0)
            reply = json.loads(result.stdout)
            if not isinstance(reply, dict):
                raise ValueError("invalid sweeper response")
            success = result.returncode == 0 and reply.get("success") is True
            message = str(reply.get("message", "sweeper returned no status"))
            with self.lock:
                if enabled and reply.get("previous_state") == 0:
                    # Also remember an uncertain toggle so abort cleanup can
                    # query and turn it off without blindly toggling again.
                    self.sweep_started_for_recovery = True
                if not enabled and success:
                    self.sweep_started_for_recovery = False
                self.sweep_status = message
            return success, message
        except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
            with self.lock:
                self.sweep_status = "sweeper command failed: {}".format(exc)
            return False, self.sweep_status

    def _stop_recovery_sweep(self):
        with self.lock:
            owned = self.sweep_started_for_recovery and self.sweep_auto_stop
            self.recovery_sweep_ready = False
        if owned:
            success, message = self._set_sweep(False)
            if not success:
                rospy.logerr("Recovery sweeper stop not confirmed: %s", message)

    def _sweep_abort_worker(self):
        with self.transition_lock:
            with self.lock:
                if self.mode not in (FOD_ABORTED, FOD_COMPLETE_STOP):
                    return
            self._stop_recovery_sweep()

    def _resume_waiting_for_target(self):
        success, message = self._return_to_gps(
            "FOD disappeared before visual handoff", keep_sweep=True)
        if success:
            with self.lock:
                self.recovery_armed = True
                self.reason = "camera recovery armed; no visible FOD, GPS continues with sweeping"
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
        with self.lock:
            self.visual_state = state
            if state == "COMPLETE" and self.mode == FOD_ACTIVE:
                self.mode = FOD_COMPLETE_STOP
                self.reason = "one FOD recovery completed; stopped until an explicit operator request"
                self.recovery_armed = False
                self.command_source = STOP_SOURCE
                self.allow_move_base_goals = False
                self._publish_zero_locked("visual COMPLETE transition stop")
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
            target_missing = self.retain_gps_route and not self._has_fod_locked()
        if target_missing:
            return self._resume_waiting_for_target()

        success, message = self._set_sweep(True)
        if not success:
            reason = "recovery sweeper could not start: {}".format(message)
            self._set_mode(FOD_ABORTED, reason, STOP_SOURCE, False)
            self._stop_recovery_sweep()
            return False, reason
        with self.lock:
            still_entering = self.mode == ENTERING_FOD and not self.shutdown_event.is_set()
        if not still_entering:
            self._stop_recovery_sweep()
            return False, "visual handoff interrupted"
        with self.lock:
            target_missing = self.retain_gps_route and not self._has_fod_locked()
        if target_missing:
            return self._resume_waiting_for_target()

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
        self._set_mode(
            GPS_ACTIVE,
            "GPS navigation resumed from the post-recovery vehicle position",
            GPS_SOURCE,
            True,
        )
        return True, "GPS navigation resumed; visual controller is in standby"

    def _set_fod_enabled_cb(self, request):
        with self.transition_lock:
            with self.lock:
                mode = self.mode
            if request.data:
                if mode == FOD_ACTIVE:
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
                if self.retain_gps_route:
                    with self.lock:
                        if not self.recovery_armed:
                            self.sweep_start_requested = False
                            self.recovery_sweep_ready = False
                        self.recovery_armed = True
                        self.reason = "camera recovery armed; start sweeping inside {:.1f} m of final GPS goal, switch to visual only with FOD".format(self.recovery_radius_m)
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
                "command_source": self.command_source,
                "output_reason": self.last_output_reason,
                "gps_paused": self.gps_paused,
                "retain_gps_route": self.retain_gps_route,
                "recovery_armed": self.recovery_armed,
                "recovery_radius_m": self.recovery_radius_m,
                "final_goal_distance_m": self._recovery_distance_locked(),
                "fod_target_available": self._has_fod_locked(),
                "sweep_status": self.sweep_status,
                "sweep_started_for_recovery": self.sweep_started_for_recovery,
                "sweep_start_requested": self.sweep_start_requested,
                "recovery_sweep_ready": self.recovery_sweep_ready,
                "visual_state": self.visual_state,
                "move_base_goals_allowed": self.allow_move_base_goals,
                "odom_age_sec": None if odom_age is None else round(odom_age, 3),
                "measured_linear_x": self.latest_linear_x,
                "measured_angular_z": self.latest_angular_z,
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
