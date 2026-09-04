#!/usr/bin/env python3
"""Deterministic command/pose fault injection for coverage robustness tests."""

import json
import math
import os
import threading

import rospy
from autolabor_coverage.msg import CoverageStatus, EnforcedPath
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String


def clamp(value, low, high):
    return max(low, min(high, value))


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def positive_cross_track_offset(sweep_yaw, distance):
    """Return the map delta that adds `distance` to `_sweep_geometry` cross."""
    return (
        float(distance) * math.sin(float(sweep_yaw)),
        -float(distance) * math.cos(float(sweep_yaw)),
    )


class NavigationFaultInjector:
    def __init__(self):
        self.lock = threading.RLock()
        self.closing = False
        self.scenario = str(rospy.get_param("~scenario", "none"))
        if self.scenario not in ("none", "sweep_overshoot", "entry_offset"):
            raise ValueError("unsupported navigation fault scenario")
        self.target_region = str(rospy.get_param("~target_region", "A区"))
        self.target_segment = int(rospy.get_param("~target_segment", 2))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.40))
        self.overshoot_trigger_distance = float(rospy.get_param(
            "~overshoot_trigger_distance_m", 0.18
        ))
        self.overshoot_distance = float(rospy.get_param(
            "~overshoot_distance_m", 0.45
        ))
        self.overshoot_hold_speed = float(rospy.get_param(
            "~overshoot_hold_speed_mps", 0.55
        ))
        self.entry_lateral_offset = float(rospy.get_param(
            "~entry_lateral_offset_m", 0.22
        ))
        self.entry_yaw_offset = float(rospy.get_param(
            "~entry_yaw_offset_rad", 0.18
        ))
        self.entry_injection_phase = str(rospy.get_param(
            "~entry_injection_phase", "pre_sweep"
        ))
        self.entry_trigger_distance = float(rospy.get_param(
            "~entry_trigger_distance_m", 0.30
        ))
        self.result_dir = os.path.abspath(rospy.get_param("~result_dir"))
        values = (
            self.command_timeout,
            self.overshoot_trigger_distance,
            self.overshoot_distance,
            self.overshoot_hold_speed,
            abs(self.entry_lateral_offset),
            abs(self.entry_yaw_offset),
            self.entry_trigger_distance,
        )
        if (
            not all(math.isfinite(value) for value in values)
            or self.entry_injection_phase not in ("pre_sweep", "sweep_started")
            or not 0.10 <= self.command_timeout <= 2.0
            or not 0.05 <= self.overshoot_trigger_distance <= 0.50
            or not 0.10 <= self.overshoot_distance <= 0.80
            or not 0.10 <= self.overshoot_hold_speed <= 0.80
            or abs(self.entry_lateral_offset) > 0.60
            or abs(self.entry_yaw_offset) > 0.80
            or not 0.10 <= self.entry_trigger_distance <= 0.60
        ):
            raise ValueError("navigation fault parameters are invalid")

        os.makedirs(self.result_dir, exist_ok=True)
        self.event_file = open(
            os.path.join(self.result_dir, "fault_events.jsonl"),
            "w", encoding="utf-8",
        )
        self.nominal_command = Twist()
        self.nominal_stamp = rospy.Time(0)
        self.pose = None
        self.actual_velocity = 0.0
        self.status = None
        self.sweep = None
        self.transition_target = None
        self.injected = False
        self.phase = "pass_through"
        self.saved_sweep = None
        self.max_overshoot = 0.0
        self.stop_samples = 0

        self.command_pub = rospy.Publisher(
            "/cmd_vel_sim", Twist, queue_size=1
        )
        self.offset_pub = rospy.Publisher(
            "/coverage_gz_sim/inject_pose_offset", Pose2D, queue_size=1
        )
        self.event_pub = rospy.Publisher(
            "/coverage_gz_sim/fault_event", String, queue_size=10, latch=True
        )
        rospy.Subscriber(
            "/cmd_vel_nominal", Twist, self._command_callback, queue_size=1
        )
        rospy.Subscriber(
            "/coverage/status", CoverageStatus,
            self._status_callback, queue_size=5,
        )
        rospy.Subscriber(
            "/coverage/enforced_path", EnforcedPath,
            self._path_callback, queue_size=5,
        )
        rospy.Subscriber(
            "/coverage/hybrid_transition_path", Path,
            self._hybrid_path_callback, queue_size=5,
        )
        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.05), self._update)
        rospy.on_shutdown(self._stop)
        self._event("fault_injector_ready", scenario=self.scenario)

    def _event(self, kind, **payload):
        record = {
            "stamp": rospy.Time.now().to_sec(),
            "kind": kind,
            "scenario": self.scenario,
        }
        record.update(payload)
        rendered = json.dumps(record, ensure_ascii=False, sort_keys=True)
        if not self.closing:
            self.event_file.write(rendered + "\n")
            self.event_file.flush()
            try:
                self.event_pub.publish(String(data=rendered))
            except rospy.ROSException:
                pass
        rospy.logwarn("FAULT %s", rendered)

    def _command_callback(self, message):
        with self.lock:
            if self.closing:
                return
            self.nominal_command = message
            self.nominal_stamp = rospy.Time.now()

    def _status_callback(self, message):
        with self.lock:
            if self.closing:
                return
            self.status = message

    def _path_callback(self, message):
        if (
            not message.active
            or message.planner_mode != EnforcedPath.MODE_ENFORCED_SWEEP
            or len(message.path.poses) < 2
        ):
            return
        first = message.path.poses[0].pose.position
        last = message.path.poses[-1].pose.position
        dx = float(last.x) - float(first.x)
        dy = float(last.y) - float(first.y)
        length = math.hypot(dx, dy)
        if length <= 1.0e-6:
            return
        with self.lock:
            if self.closing:
                return
            self.sweep = {
                "plan_id": str(message.plan_id),
                "segment_index": int(message.segment_index),
                "start_x": float(first.x),
                "start_y": float(first.y),
                "end_x": float(last.x),
                "end_y": float(last.y),
                "length": length,
                "yaw": math.atan2(dy, dx),
            }

    def _hybrid_path_callback(self, message):
        if len(message.poses) < 2:
            return
        final = message.poses[-1].pose
        with self.lock:
            if self.closing:
                return
            self.transition_target = {
                "start_x": float(final.position.x),
                "start_y": float(final.position.y),
                "yaw": quaternion_yaw(final.orientation),
            }

    def _odom_callback(self, message):
        with self.lock:
            if self.closing:
                return
            self.pose = (
                float(message.pose.pose.position.x),
                float(message.pose.pose.position.y),
                quaternion_yaw(message.pose.pose.orientation),
            )
            self.actual_velocity = float(message.twist.twist.linear.x)

    @staticmethod
    def _sweep_geometry(sweep, pose):
        tangent_x = math.cos(sweep["yaw"])
        tangent_y = math.sin(sweep["yaw"])
        relative_x = pose[0] - sweep["start_x"]
        relative_y = pose[1] - sweep["start_y"]
        along = relative_x * tangent_x + relative_y * tangent_y
        cross = relative_x * tangent_y - relative_y * tangent_x
        return along, cross

    def _matches_sweep_target(self):
        return (
            self.status is not None
            and self.sweep is not None
            and self.status.state == "SWEEPING"
            and self.status.current_region_name == self.target_region
            and int(self.status.current_segment) == self.target_segment
        )

    def _matches_entry_approach(self):
        if (
            self.status is None
            or self.transition_target is None
            or self.pose is None
            or self.status.state != "TRANSITING"
            or self.status.current_region_name != self.target_region
            or int(self.status.current_segment) != self.target_segment - 1
            or abs(self.actual_velocity) > 0.08
        ):
            return False
        return math.hypot(
            self.pose[0] - self.transition_target["start_x"],
            self.pose[1] - self.transition_target["start_y"],
        ) <= self.entry_trigger_distance

    def _inject_entry_offset(self, target):
        # `_sweep_geometry` defines positive cross-track as
        # relative_x * tangent_y - relative_y * tangent_x.  Use the matching
        # map-frame normal so a positive requested disturbance adds to the
        # measured pre-existing cross-track error instead of canceling it.
        lateral_x, lateral_y = positive_cross_track_offset(
            target["yaw"], self.entry_lateral_offset
        )
        offset = Pose2D()
        offset.x = lateral_x
        offset.y = lateral_y
        offset.theta = self.entry_yaw_offset
        along, cross = self._sweep_geometry(target, self.pose)
        self.offset_pub.publish(offset)
        self.injected = True
        self._event(
            "entry_offset_injected",
            region=self.target_region,
            segment=self.target_segment,
            pre_x=self.pose[0], pre_y=self.pose[1],
            pre_yaw=self.pose[2], pre_along=along, pre_cross=cross,
            map_dx=offset.x, map_dy=offset.y,
            lateral_offset_m=self.entry_lateral_offset,
            expected_post_cross_m=cross + self.entry_lateral_offset,
            yaw_offset_rad=self.entry_yaw_offset,
            injection_phase=self.entry_injection_phase,
        )

    def _overshoot_command(self):
        along, cross = self._sweep_geometry(self.saved_sweep, self.pose)
        overshoot = along - self.saved_sweep["length"]
        self.max_overshoot = max(self.max_overshoot, overshoot)
        if self.phase == "overshoot_hold":
            if overshoot < self.overshoot_distance:
                heading_error = wrap_angle(
                    self.saved_sweep["yaw"] - self.pose[2]
                )
                command = Twist()
                command.linear.x = self.overshoot_hold_speed
                maximum_omega = self.overshoot_hold_speed / 1.20
                command.angular.z = clamp(
                    1.2 * heading_error, -maximum_omega, maximum_omega
                )
                return command
            self.phase = "overshoot_brake"
            self._event(
                "sweep_overshoot_hold_released",
                region=self.target_region,
                segment=self.target_segment,
                overshoot_m=overshoot,
                cross_m=cross,
                actual_speed_mps=self.actual_velocity,
            )
        command = Twist()
        if abs(self.actual_velocity) <= 0.02:
            self.stop_samples += 1
        else:
            self.stop_samples = 0
        if self.stop_samples >= 5:
            self.phase = "fault_complete"
            self._event(
                "sweep_overshoot_complete",
                region=self.target_region,
                segment=self.target_segment,
                overshoot_m=overshoot,
                maximum_overshoot_m=self.max_overshoot,
                cross_m=cross,
                actual_speed_mps=self.actual_velocity,
            )
        return command

    def _update(self, event):
        now = event.current_real
        with self.lock:
            if self.closing or rospy.is_shutdown():
                return
            command = Twist()
            if (
                self.scenario == "entry_offset"
                and not self.injected
                and (
                    (
                        self.entry_injection_phase == "pre_sweep"
                        and self._matches_entry_approach()
                    )
                    or (
                        self.entry_injection_phase == "sweep_started"
                        and self._matches_sweep_target()
                        and self.pose is not None
                    )
                )
            ):
                target = (
                    self.transition_target
                    if self.entry_injection_phase == "pre_sweep"
                    else self.sweep
                )
                self._inject_entry_offset(target)
            if (
                self.scenario == "sweep_overshoot"
                and not self.injected
                and self._matches_sweep_target()
                and self.pose is not None
            ):
                along, cross = self._sweep_geometry(self.sweep, self.pose)
                remaining = self.sweep["length"] - along
                if remaining <= self.overshoot_trigger_distance:
                    self.injected = True
                    self.phase = "overshoot_hold"
                    self.saved_sweep = dict(self.sweep)
                    self.max_overshoot = max(0.0, -remaining)
                    self._event(
                        "sweep_overshoot_started",
                        region=self.target_region,
                        segment=self.target_segment,
                        remaining_m=remaining,
                        cross_m=cross,
                        nominal_speed_mps=float(
                            self.nominal_command.linear.x
                        ),
                        actual_speed_mps=self.actual_velocity,
                        hold_speed_mps=self.overshoot_hold_speed,
                        requested_overshoot_m=self.overshoot_distance,
                    )
            if self.phase in ("overshoot_hold", "overshoot_brake"):
                command = self._overshoot_command()
            elif (now - self.nominal_stamp).to_sec() <= self.command_timeout:
                command = self.nominal_command
            try:
                self.command_pub.publish(command)
            except rospy.ROSException:
                return

    def _stop(self):
        with self.lock:
            if self.closing:
                return
            self.closing = True
        self.timer.shutdown()
        try:
            self.command_pub.publish(Twist())
        except rospy.ROSException:
            pass
        self.event_file.close()


if __name__ == "__main__":
    rospy.init_node("navigation_fault_injector")
    NavigationFaultInjector()
    rospy.spin()
