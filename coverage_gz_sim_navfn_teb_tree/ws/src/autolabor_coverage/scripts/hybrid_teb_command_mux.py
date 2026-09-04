#!/usr/bin/env python3
"""Fail-closed output mux for TEB-tracked fixed-gear Hybrid actions."""

import math
import threading

import rospy
import tf2_ros
from autolabor_coverage.msg import EnforcedPath
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32


def clamp(value, low, high):
    return max(low, min(high, value))


def quaternion_yaw(quaternion):
    norm = math.sqrt(
        quaternion.x * quaternion.x
        + quaternion.y * quaternion.y
        + quaternion.z * quaternion.z
        + quaternion.w * quaternion.w
    )
    if not math.isfinite(norm) or norm < 1.0e-9:
        raise ValueError("path contains an invalid quaternion")
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def infer_fixed_gear(points):
    """Infer one signed gear and reject mixed/non-tangent path edges."""
    signs = set()
    for first, second in zip(points, points[1:]):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        distance = math.hypot(dx, dy)
        if distance <= 1.0e-6:
            continue
        yaw_delta = math.atan2(
            math.sin(second[2] - first[2]),
            math.cos(second[2] - first[2]),
        )
        tangent_yaw = first[2] + 0.5 * yaw_delta
        projection = dx * math.cos(tangent_yaw) + dy * math.sin(tangent_yaw)
        if abs(projection) < 0.5 * distance:
            raise ValueError("Hybrid edge is not tangent to vehicle heading")
        signs.add(1 if projection > 0.0 else -1)
    if len(signs) != 1:
        raise ValueError("Hybrid action is not one constant-gear path")
    return signs.pop()


def checked_teb_command(message, expected_gear, minimum_turning_radius):
    """Copy a TEB command and apply final gear/curvature safety checks."""
    command = Twist()
    command.linear.x = float(message.linear.x)
    command.linear.y = float(message.linear.y)
    command.linear.z = float(message.linear.z)
    command.angular.x = float(message.angular.x)
    command.angular.y = float(message.angular.y)
    command.angular.z = float(message.angular.z)
    values = (
        command.linear.x, command.linear.y, command.linear.z,
        command.angular.x, command.angular.y, command.angular.z,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("TEB emitted a non-finite command")
    if expected_gear not in (
            EnforcedPath.GEAR_FORWARD, EnforcedPath.GEAR_REVERSE):
        raise ValueError("Hybrid action has no fixed-gear contract")
    if (
        (expected_gear > 0 and command.linear.x < -1.0e-6)
        or (expected_gear < 0 and command.linear.x > 1.0e-6)
    ):
        raise ValueError("TEB command violated the fixed-gear contract")
    maximum_omega = abs(command.linear.x) / minimum_turning_radius
    command.angular.z = clamp(
        command.angular.z, -maximum_omega, maximum_omega
    )
    return command


class HybridTebCommandMux:
    """Forward TEB commands only while all Hybrid safety leases are fresh."""

    def __init__(self):
        self.lock = threading.RLock()
        self.closing = False
        self.enabled = rospy.get_param("~enabled", False)
        self.use_tf_pose = rospy.get_param("~use_tf_pose", True)
        if type(self.enabled) is not bool or type(self.use_tf_pose) is not bool:
            raise ValueError("Hybrid TEB mux boolean parameters are invalid")
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.40))
        self.safety_timeout = float(rospy.get_param("~safety_timeout", 1.50))
        self.minimum_turning_radius = float(rospy.get_param(
            "~minimum_turning_radius", 1.35
        ))
        if not all(math.isfinite(value) and value > 0.0 for value in (
                self.command_timeout, self.safety_timeout,
                self.minimum_turning_radius)):
            raise ValueError("Hybrid TEB mux limits are invalid")

        self.global_frame = str(rospy.get_param(
            "~global_frame", "map"
        )).lstrip("/")
        self.robot_base_frame = str(rospy.get_param(
            "~robot_base_frame", "base_link"
        )).lstrip("/")
        self.command_topic = str(rospy.get_param(
            "~command_topic", "/cmd_vel_unlocalized"
        ))
        self.teb_command_topic = str(rospy.get_param(
            "~teb_command_topic", "/cmd_vel_teb"
        ))
        self.enforced_path_topic = str(rospy.get_param(
            "~enforced_path_topic", "/coverage/enforced_path"
        ))
        self.odom_topic = str(rospy.get_param("~odom_topic", "/odom"))
        self.tracking_error_topic = str(rospy.get_param(
            "~tracking_error_topic", "/coverage/hybrid_tracking_error"
        ))
        self.safety_topic = str(rospy.get_param(
            "~safety_topic",
            "/move_base/CoverageGlobalPlanner/hybrid_path_safe",
        ))
        topics = (
            self.command_topic, self.teb_command_topic,
            self.enforced_path_topic, self.odom_topic,
            self.tracking_error_topic, self.safety_topic,
        )
        if (
            not self.global_frame
            or not self.robot_base_frame
            or not all(topic.startswith("/") for topic in topics)
        ):
            raise ValueError("Hybrid TEB mux frames/topics are invalid")

        self.pose = None
        self.pose_stamp = rospy.Time(0)
        self.teb_command = Twist()
        self.teb_stamp = rospy.Time(0)
        self.path_stamp = rospy.Time(0)
        self.safety_stamp = rospy.Time(0)
        self.path_signature = None
        self.path_points = []
        self.expected_gear = EnforcedPath.GEAR_AUTO
        self.hybrid_active = False
        self.hybrid_path_safe = False
        self.zero_handoff_cycles = 0

        self.tf_buffer = None
        self.tf_listener = None
        if self.use_tf_pose:
            self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.command_pub = rospy.Publisher(
            self.command_topic, Twist, queue_size=1
        )
        self.error_pub = rospy.Publisher(
            self.tracking_error_topic, Float32, queue_size=10
        )
        rospy.Subscriber(
            self.teb_command_topic, Twist, self._teb_callback, queue_size=1
        )
        rospy.Subscriber(
            self.enforced_path_topic, EnforcedPath,
            self._path_callback, queue_size=1
        )
        rospy.Subscriber(
            self.odom_topic, Odometry, self._odom_callback, queue_size=1
        )
        rospy.Subscriber(
            self.safety_topic, Bool, self._safety_callback, queue_size=1
        )
        self.timer = rospy.Timer(rospy.Duration(0.05), self._update)
        rospy.on_shutdown(self._stop)

    @staticmethod
    def _path_points(message):
        return [
            (
                float(pose.pose.position.x),
                float(pose.pose.position.y),
                quaternion_yaw(pose.pose.orientation),
            )
            for pose in message.path.poses
        ]

    def _disarm_hybrid_locked(self, now):
        if self.hybrid_active or self.path_signature is not None:
            self.zero_handoff_cycles = max(self.zero_handoff_cycles, 2)
        self.hybrid_active = False
        self.path_signature = None
        self.path_points = []
        self.expected_gear = EnforcedPath.GEAR_AUTO
        self.hybrid_path_safe = False
        self.safety_stamp = rospy.Time(0)
        self.path_stamp = now

    def _path_callback(self, message):
        now = rospy.Time.now()
        is_hybrid = (
            self.enabled
            and message.planner_mode == EnforcedPath.MODE_HYBRID_TRANSIT
            and len(message.path.poses) >= 2
        )
        with self.lock:
            if self.closing:
                return
            if not is_hybrid:
                self._disarm_hybrid_locked(now)
                return
            try:
                path_frame = str(
                    message.path.header.frame_id or message.header.frame_id
                ).lstrip("/")
                if path_frame != self.global_frame:
                    raise ValueError("Hybrid path uses the wrong frame")
                points = self._path_points(message)
                expected_gear = int(message.expected_gear)
                if infer_fixed_gear(points) != expected_gear:
                    raise ValueError(
                        "Hybrid path direction disagrees with fixed gear"
                    )
                signature = (
                    str(message.plan_id), int(message.segment_index),
                    int(message.path_generation), expected_gear,
                )
                if signature != self.path_signature:
                    self.zero_handoff_cycles = max(
                        self.zero_handoff_cycles, 2
                    )
                    self.hybrid_path_safe = False
                    self.safety_stamp = rospy.Time(0)
                    self.path_signature = signature
                    self.path_points = points
                    self.expected_gear = expected_gear
                    rospy.loginfo(
                        "Hybrid TEB mux armed segment=%d generation=%d "
                        "gear=%s poses=%d",
                        int(message.segment_index),
                        int(message.path_generation),
                        "forward" if expected_gear > 0 else "reverse",
                        len(points),
                    )
                self.hybrid_active = True
                self.path_stamp = now
            except ValueError as error:
                self._disarm_hybrid_locked(now)
                rospy.logerr_throttle(
                    1.0, "Hybrid TEB mux rejected path: %s", error
                )

    def _teb_callback(self, message):
        with self.lock:
            if not self.closing:
                self.teb_command = message
                self.teb_stamp = rospy.Time.now()

    def _odom_callback(self, message):
        if self.use_tf_pose:
            return
        try:
            yaw = quaternion_yaw(message.pose.pose.orientation)
        except ValueError:
            return
        with self.lock:
            if not self.closing:
                self.pose = (
                    float(message.pose.pose.position.x),
                    float(message.pose.pose.position.y), yaw,
                )
                self.pose_stamp = message.header.stamp

    def _safety_callback(self, message):
        with self.lock:
            if not self.closing:
                self.hybrid_path_safe = bool(message.data)
                self.safety_stamp = rospy.Time.now()

    def _tf_pose(self, now):
        if not self.use_tf_pose or self.tf_buffer is None:
            return None, rospy.Time(0)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame, self.robot_base_frame, rospy.Time(0),
                rospy.Duration(0.02),
            )
            translation = transform.transform.translation
            yaw = quaternion_yaw(transform.transform.rotation)
            stamp = transform.header.stamp
            age = (now - stamp).to_sec()
            if (
                stamp == rospy.Time(0)
                or not all(math.isfinite(value) for value in (
                    translation.x, translation.y, yaw, age
                ))
                or age < -0.20
                or age > self.command_timeout
            ):
                return None, stamp
            return (float(translation.x), float(translation.y), yaw), stamp
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, ValueError):
            return None, rospy.Time(0)

    @staticmethod
    def _tracking_error(points, x, y):
        if not points:
            return float("inf")
        return math.sqrt(min(
            (point[0] - x) ** 2 + (point[1] - y) ** 2
            for point in points
        ))

    def _update(self, event):
        now = event.current_real
        with self.lock:
            need_tf = (
                not self.closing and self.use_tf_pose
                and self.hybrid_active and self.zero_handoff_cycles <= 0
            )
        tf_pose, tf_stamp = (
            self._tf_pose(now) if need_tf else (None, rospy.Time(0))
        )
        with self.lock:
            if self.closing or rospy.is_shutdown():
                return
            command = Twist()
            if self.zero_handoff_cycles > 0:
                self.zero_handoff_cycles -= 1
            elif self.hybrid_active:
                pose = tf_pose if self.use_tf_pose else self.pose
                pose_stamp = tf_stamp if self.use_tf_pose else self.pose_stamp
                fresh = (
                    pose is not None and self.hybrid_path_safe
                    and (now - pose_stamp).to_sec() <= self.command_timeout
                    and (now - self.teb_stamp).to_sec() <= self.command_timeout
                    and (now - self.path_stamp).to_sec() <= self.command_timeout
                    and (now - self.safety_stamp).to_sec() <= self.safety_timeout
                )
                if fresh:
                    try:
                        command = checked_teb_command(
                            self.teb_command, self.expected_gear,
                            self.minimum_turning_radius,
                        )
                        self.error_pub.publish(Float32(data=self._tracking_error(
                            self.path_points, pose[0], pose[1]
                        )))
                    except ValueError as error:
                        rospy.logerr_throttle(
                            1.0, "Hybrid TEB mux rejected command: %s", error
                        )
                    except rospy.ROSException:
                        return
                else:
                    rospy.logwarn_throttle(
                        1.0, "Hybrid TEB mux is holding zero for a stale "
                        "TF/TEB/path lease or unsafe planner permit"
                    )
            elif (now - self.teb_stamp).to_sec() <= self.command_timeout:
                command = self.teb_command
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


if __name__ == "__main__":
    rospy.init_node("hybrid_teb_command_mux")
    HybridTebCommandMux()
    rospy.spin()
