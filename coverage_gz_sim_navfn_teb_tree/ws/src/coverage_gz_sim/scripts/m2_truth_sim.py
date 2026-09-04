#!/usr/bin/env python3
"""Perfect-state Ackermann plant with the M2 Twist-to-steering contract.

Gazebo remains the visual/physics world.  This node integrates the commanded
base-frame Twist with a bicycle model, enforces the configured steering/radius
limit, writes the resulting pose to Gazebo, and publishes exact odometry/TF.
No localization or wheel-slip noise is intentionally introduced.
"""

import math
import threading

import rospy
import tf2_ros
from autolabor_canbus_driver.msg import ChassisParameter, ChassisStatusInfo
from autolabor_canbus_driver.srv import (
    ChassisParameterServer,
    ChassisParameterServerResponse,
)
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Pose2D, TransformStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler


def clamp(value, low, high):
    return max(low, min(high, value))


def angular_velocity_after_linear_limit(requested_velocity, limited_velocity,
                                        requested_omega):
    """Match the production M2 driver's curvature-preserving speed clamp."""
    values = (requested_velocity, limited_velocity, requested_omega)
    if not all(math.isfinite(value) for value in values):
        return 0.0
    if abs(requested_velocity) <= 1.0e-9:
        return requested_omega
    return requested_omega * limited_velocity / requested_velocity


def twist_to_front_steering(linear_velocity, angular_velocity, wheelbase,
                            maximum_steer, zero_linear_epsilon=1.0e-4,
                            zero_angular_epsilon=1.0e-2):
    """Python equivalent of m2_twist_steering.h used on the real chassis."""
    values = (
        linear_velocity,
        angular_velocity,
        wheelbase,
        maximum_steer,
        zero_linear_epsilon,
        zero_angular_epsilon,
    )
    if (
        not all(math.isfinite(value) for value in values)
        or wheelbase <= 0.0
        or maximum_steer <= 0.0
        or zero_linear_epsilon < 0.0
        or zero_angular_epsilon < 0.0
    ):
        return 0.0
    if abs(linear_velocity) <= zero_linear_epsilon:
        if angular_velocity > zero_angular_epsilon:
            return maximum_steer
        if angular_velocity < -zero_angular_epsilon:
            return -maximum_steer
        return 0.0
    return clamp(
        math.atan(angular_velocity * wheelbase / linear_velocity),
        -maximum_steer,
        maximum_steer,
    )


class M2TruthPlant:
    def __init__(self):
        self.lock = threading.RLock()
        self.closing = False
        self.model_name = rospy.get_param("~model_name", "m2_sim")
        self.wheelbase = float(rospy.get_param("~wheelbase", 0.65))
        self.minimum_radius = float(rospy.get_param("~minimum_turning_radius", 1.35))
        self.maximum_steer = float(rospy.get_param("~maximum_steering_angle", 0.488692))
        self.maximum_speed = float(rospy.get_param("~maximum_speed", 1.60))
        self.linear_accel = float(rospy.get_param("~linear_accel", 1.00))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.50))
        self.x = float(rospy.get_param("~initial_x", 16.65))
        self.y = float(rospy.get_param("~initial_y", -39.30))
        self.yaw = float(rospy.get_param("~initial_yaw", 0.0))
        self.velocity = 0.0
        self.actual_omega = 0.0
        self.actual_steer = 0.0
        self.command = Twist()
        self.command_wall = 0.0
        self.last_stamp = None

        radius_steer = math.atan(self.wheelbase / self.minimum_radius)
        self.effective_steer_limit = min(self.maximum_steer, radius_steer)
        if self.wheelbase <= 0.0 or self.minimum_radius <= 0.0:
            raise ValueError("wheelbase and minimum turning radius must be positive")

        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.lio_odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=10)
        self.chassis_pub = rospy.Publisher(
            "/m2_driver/chassis_info", ChassisStatusInfo, queue_size=5
        )
        self.wheel_pub = rospy.Publisher(
            "/m2_driver/wheel_angle", Float32, queue_size=10
        )
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.cmd_sub = rospy.Subscriber(
            "/cmd_vel_sim", Twist, self._command_callback, queue_size=1
        )
        self.pose_offset_sub = rospy.Subscriber(
            "/coverage_gz_sim/inject_pose_offset",
            Pose2D,
            self._pose_offset_callback,
            queue_size=1,
        )
        self.parameter_service = rospy.Service(
            "/m2_driver/chassis_parameter",
            ChassisParameterServer,
            self._parameter_callback,
        )
        self.gazebo_set_state = None
        self._connect_gazebo()
        self.timer = rospy.Timer(rospy.Duration(0.02), self._update)
        rospy.on_shutdown(self._stop)

    def _connect_gazebo(self):
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
            self.gazebo_set_state = rospy.ServiceProxy(
                "/gazebo/set_model_state", SetModelState, persistent=True
            )
        except (rospy.ROSException, rospy.ServiceException) as error:
            rospy.logwarn("Gazebo pose service is not ready yet: %s", error)

    def _command_callback(self, message):
        with self.lock:
            if self.closing:
                return
            self.command = message
            self.command_wall = rospy.get_time()

    def _pose_offset_callback(self, message):
        values = (float(message.x), float(message.y), float(message.theta))
        if (
            not all(math.isfinite(value) for value in values)
            or math.hypot(values[0], values[1]) > 1.0
            or abs(values[2]) > 1.0
        ):
            rospy.logerr("Rejected invalid simulation pose offset: %r", values)
            return
        with self.lock:
            if self.closing:
                return
            self.x += values[0]
            self.y += values[1]
            self.yaw = math.atan2(
                math.sin(self.yaw + values[2]),
                math.cos(self.yaw + values[2]),
            )
            rospy.logwarn(
                "Applied simulation pose offset dx=%+.3fm dy=%+.3fm "
                "dyaw=%+.1fdeg -> x=%.3f y=%.3f yaw=%.1fdeg",
                values[0], values[1], math.degrees(values[2]),
                self.x, self.y, math.degrees(self.yaw),
            )

    def _parameter_callback(self, _request):
        parameters = ChassisParameter()
        parameters.max_speed = self.maximum_speed
        parameters.max_steer = self.maximum_steer
        parameters.robot_width = 0.70
        parameters.robot_length = self.wheelbase
        parameters.wheel_radius = 0.16
        return ChassisParameterServerResponse(
            success=True,
            parameters=parameters,
            message="Gazebo M2 truth parameters",
        )

    def _desired_motion(self, stamp):
        if self.command_wall <= 0.0 or stamp.to_sec() - self.command_wall > self.command_timeout:
            return 0.0, 0.0
        requested_velocity = float(self.command.linear.x)
        requested_omega = float(self.command.angular.z)
        if not math.isfinite(requested_velocity) or not math.isfinite(requested_omega):
            return 0.0, 0.0
        desired_velocity = clamp(
            requested_velocity, -self.maximum_speed, self.maximum_speed
        )
        return desired_velocity, angular_velocity_after_linear_limit(
            requested_velocity, desired_velocity, requested_omega
        )

    def _integrate(self, stamp, dt):
        desired_velocity, requested_omega = self._desired_motion(stamp)
        # The real M2 driver converts the requested Twist to a steering angle
        # from the requested v/omega pair before sending the CAN frame.  Using
        # the lagging measured velocity here spuriously saturated steering on
        # every acceleration/deceleration and made short Hybrid transitions
        # diverge in simulation.
        requested_steer = twist_to_front_steering(
            desired_velocity,
            requested_omega,
            self.wheelbase,
            self.effective_steer_limit,
        )
        maximum_delta = self.linear_accel * dt
        self.velocity += clamp(
            desired_velocity - self.velocity, -maximum_delta, maximum_delta
        )
        self.actual_steer = requested_steer
        if abs(self.velocity) <= 1.0e-4:
            self.velocity = 0.0
            self.actual_omega = 0.0
        else:
            self.actual_omega = (
                self.velocity * math.tan(self.actual_steer) / self.wheelbase
            )
        midpoint_yaw = self.yaw + 0.5 * self.actual_omega * dt
        self.x += self.velocity * math.cos(midpoint_yaw) * dt
        self.y += self.velocity * math.sin(midpoint_yaw) * dt
        self.yaw = math.atan2(
            math.sin(self.yaw + self.actual_omega * dt),
            math.cos(self.yaw + self.actual_omega * dt),
        )

    def _publish_truth(self, stamp):
        quaternion = quaternion_from_euler(0.0, 0.0, self.yaw)
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "odom"
        odometry.child_frame_id = "base_link"
        odometry.pose.pose.position.x = self.x
        odometry.pose.pose.position.y = self.y
        odometry.pose.pose.orientation.x = quaternion[0]
        odometry.pose.pose.orientation.y = quaternion[1]
        odometry.pose.pose.orientation.z = quaternion[2]
        odometry.pose.pose.orientation.w = quaternion[3]
        odometry.twist.twist.linear.x = self.velocity
        odometry.twist.twist.angular.z = self.actual_omega
        self.odom_pub.publish(odometry)
        self.lio_odom_pub.publish(odometry)

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = stamp
        map_to_odom.header.frame_id = "map"
        map_to_odom.child_frame_id = "odom"
        map_to_odom.transform.rotation.w = 1.0
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = stamp
        odom_to_base.header.frame_id = "odom"
        odom_to_base.child_frame_id = "base_link"
        odom_to_base.transform.translation.x = self.x
        odom_to_base.transform.translation.y = self.y
        odom_to_base.transform.rotation.x = quaternion[0]
        odom_to_base.transform.rotation.y = quaternion[1]
        odom_to_base.transform.rotation.z = quaternion[2]
        odom_to_base.transform.rotation.w = quaternion[3]
        self.tf_pub.sendTransform([map_to_odom, odom_to_base])

        chassis = ChassisStatusInfo()
        chassis.battery_percent = 100.0
        chassis.remain_sec = 36000.0
        chassis.remain_capacity = 100.0
        chassis.battery_voltage = 48.0
        self.chassis_pub.publish(chassis)
        self.wheel_pub.publish(Float32(data=self.actual_steer))

        if self.gazebo_set_state is not None:
            state = ModelState()
            state.model_name = self.model_name
            state.reference_frame = "world"
            state.pose.position.x = self.x
            state.pose.position.y = self.y
            state.pose.position.z = 0.20
            state.pose.orientation.x = quaternion[0]
            state.pose.orientation.y = quaternion[1]
            state.pose.orientation.z = quaternion[2]
            state.pose.orientation.w = quaternion[3]
            state.twist.linear.x = self.velocity * math.cos(self.yaw)
            state.twist.linear.y = self.velocity * math.sin(self.yaw)
            state.twist.angular.z = self.actual_omega
            try:
                self.gazebo_set_state(state)
            except rospy.ServiceException as error:
                rospy.logwarn_throttle(2.0, "Gazebo pose update failed: %s", error)

    def _update(self, event):
        stamp = event.current_real
        with self.lock:
            if self.closing or rospy.is_shutdown():
                return
            if self.last_stamp is None:
                self.last_stamp = stamp
                try:
                    self._publish_truth(stamp)
                except rospy.ROSException:
                    return
                return
            # A simulated-clock jump can make rospy.Timer drain more than one
            # callback at the same current_real value.  Duplicate TF stamps are
            # neither new truth nor useful controller input.
            if stamp <= self.last_stamp:
                return
            dt = max(0.0, min(0.10, (stamp - self.last_stamp).to_sec()))
            self.last_stamp = stamp
            self._integrate(stamp, dt)
            try:
                self._publish_truth(stamp)
            except rospy.ROSException:
                # Publishers are closed concurrently during roslaunch
                # teardown.  Navigation has already stopped at this point.
                return

    def _stop(self):
        with self.lock:
            if self.closing:
                return
            self.closing = True
        self.timer.shutdown()


if __name__ == "__main__":
    rospy.init_node("m2_truth_sim")
    M2TruthPlant()
    rospy.spin()
