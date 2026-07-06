#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import select
import sys
import termios
import time
import tty

import rosgraph
import rospy
from geometry_msgs.msg import Twist


HELP_TEXT = """
Autolabor keyboard teleop

Movement:
  w/s or up/down       : set forward / backward speed
  a/d or left/right    : pulse left / right steering
  f/g                  : pulse max left / right steering
  p   : 6s SLAM init S-curve, left-forward/right-forward/straight
  x   : stop
  space : emergency stop

Speed:
  q/e : increase / decrease linear speed
  z/c : increase / decrease normal steering command

Other:
  steering release is approximated by timeout in a normal terminal
  h   : show this help
  Ctrl-C : quit
"""


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


LINEAR_KEYS = {"w", "s"}
STEERING_KEYS = {"a", "d", "f", "g"}
MOTION_KEYS = LINEAR_KEYS | STEERING_KEYS | {"p"}
STOP_KEYS = {" ", "x"}


class KeyboardTeleop:
    def __init__(self):
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.key_timeout = float(rospy.get_param("~key_timeout", 0.5))
        self.steering_timeout = float(rospy.get_param("~steering_timeout", self.key_timeout))
        self.key_switch_quiet_time = float(rospy.get_param("~key_switch_quiet_time", 0.25))

        self.linear_step = float(rospy.get_param("~linear_step", 0.05))
        self.angular_step = float(rospy.get_param("~angular_step", 0.1))
        self.linear_speed = float(rospy.get_param("~linear_speed", 0.25))
        self.angular_speed = float(rospy.get_param("~angular_speed", 0.6))
        self.max_linear = float(rospy.get_param("~max_linear", 1.5))
        self.max_angular = float(rospy.get_param("~max_angular", 1.2))
        self.max_turn_angular = float(rospy.get_param("~max_turn_angular", self.max_angular))
        self.auto_stop = bool(rospy.get_param("~auto_stop", False))
        self.s_curve_speed = float(rospy.get_param("~s_curve_speed", 0.7))
        self.s_curve_angular = float(rospy.get_param("~s_curve_angular", self.angular_speed))
        self.s_curve_duration = float(rospy.get_param("~s_curve_duration", 6.0))

        self.linear_axis = 0
        self.angular_command = 0.0
        self.s_curve_active = False
        self.s_curve_start_time = 0.0
        self.last_key_time = 0.0
        self.last_input_time = 0.0
        self.last_steering_key_time = 0.0
        self.last_subscriber_warn = 0.0
        self.active_linear_key = None
        self.active_steering_key = None
        self.superseded_linear_keys = set()
        self.superseded_steering_keys = set()
        self.publisher = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)

        self.settings = None

    def master_subscribers(self):
        try:
            state = rosgraph.Master(rospy.get_name()).getSystemState()
        except Exception as exc:
            return [], str(exc)

        for topic, nodes in state[1]:
            if topic == self.cmd_topic:
                return nodes, None
        return [], None

    def print_help(self):
        rospy.loginfo("\n%s", HELP_TEXT)
        rospy.loginfo(
            "Publishing %s, linear=%.2f m/s, steering=%.2f rad/s, max steering command=%.2f rad/s",
            self.cmd_topic,
            self.linear_speed,
            self.angular_speed,
            self.max_turn_angular,
        )

    def warn_if_no_connection(self):
        if self.publisher.get_num_connections() != 0:
            return

        now = time.time()
        if now - self.last_subscriber_warn <= 2.0:
            return

        subscribers, error = self.master_subscribers()
        if subscribers:
            rospy.logwarn(
                "Master shows subscribers on %s (%s), but this publisher has no TCP connection yet. "
                "If this repeats, check ROS_HOSTNAME/ROS_IP name resolution.",
                self.cmd_topic,
                ", ".join(subscribers),
            )
        elif error:
            rospy.logwarn("Cannot query ROS master for %s subscribers: %s", self.cmd_topic, error)
        else:
            rospy.logwarn(
                "No subscriber on %s. Start the CAN chassis driver first, "
                "for example: roslaunch robot_bringup can.launch port_name:=/dev/ttyUSB0",
                self.cmd_topic,
            )
        self.last_subscriber_warn = now

    def translate_key(self, key):
        if key != "\x1b":
            return key

        ready, _, _ = select.select([sys.stdin], [], [], 0.01)
        if not ready:
            return key
        suffix = sys.stdin.read(2)
        arrow_keys = {
            "[A": "w",
            "[B": "s",
            "[D": "a",
            "[C": "d",
        }
        return arrow_keys.get(suffix, key)

    def read_keys(self, timeout):
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return []

        keys = []
        while ready:
            keys.append(self.translate_key(sys.stdin.read(1)))
            ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        return keys

    def publish(self, linear_axis=None, angular_command=None):
        if linear_axis is None:
            linear_axis = self.linear_axis
        if angular_command is None:
            angular_command = self.angular_command

        self.warn_if_no_connection()
        twist = Twist()
        twist.linear.x = linear_axis * self.linear_speed
        twist.angular.z = angular_command
        self.publisher.publish(twist)

    def stop(self):
        self.s_curve_active = False
        self.linear_axis = 0
        self.angular_command = 0.0
        self.last_steering_key_time = 0.0
        self.active_linear_key = None
        self.active_steering_key = None
        self.publish(0, 0)

    def set_steering(self, angular_command):
        self.s_curve_active = False
        self.angular_command = clamp(angular_command, -self.max_turn_angular, self.max_turn_angular)
        self.last_steering_key_time = time.time()

    def maybe_center_steering(self):
        if not self.last_steering_key_time:
            return
        if time.time() - self.last_steering_key_time <= self.steering_timeout:
            return
        self.angular_command = 0.0
        self.last_steering_key_time = 0.0
        self.active_steering_key = None

    def start_s_curve(self):
        self.linear_axis = 0
        self.angular_command = 0.0
        self.last_steering_key_time = 0.0
        self.active_linear_key = None
        self.active_steering_key = None
        self.s_curve_active = True
        self.s_curve_start_time = time.time()
        rospy.loginfo(
            "Starting 6s SLAM init S-curve: %.2f m/s, angular %.2f rad/s",
            self.s_curve_speed,
            self.s_curve_angular,
        )

    def publish_s_curve(self):
        elapsed = time.time() - self.s_curve_start_time
        if elapsed >= self.s_curve_duration:
            self.stop()
            rospy.loginfo("SLAM init S-curve complete")
            return

        segment_duration = self.s_curve_duration / 3.0
        if elapsed < segment_duration:
            angular = self.s_curve_angular
        elif elapsed < 2.0 * segment_duration:
            angular = -self.s_curve_angular
        else:
            angular = 0.0

        self.warn_if_no_connection()
        twist = Twist()
        twist.linear.x = self.s_curve_speed
        twist.angular.z = angular
        self.publisher.publish(twist)

    def preempt_all_motion(self):
        self.s_curve_active = False
        self.linear_axis = 0
        self.angular_command = 0.0
        self.last_steering_key_time = 0.0
        self.active_linear_key = None
        self.active_steering_key = None
        self.publish(0, 0)

    def set_linear_motion(self, key):
        if key == self.active_linear_key:
            return

        if self.active_linear_key:
            self.superseded_linear_keys.add(self.active_linear_key)

        self.s_curve_active = False
        self.linear_axis = 0
        self.publish(0, self.angular_command)

        if key == "w":
            self.linear_axis = 1
        elif key == "s":
            self.linear_axis = -1
        self.active_linear_key = key

    def set_steering_motion(self, key):
        if key == self.active_steering_key:
            self.last_steering_key_time = time.time()
            return

        if self.active_steering_key:
            self.superseded_steering_keys.add(self.active_steering_key)

        self.s_curve_active = False
        self.angular_command = 0.0
        self.last_steering_key_time = 0.0
        self.publish(self.linear_axis, 0)

        if key == "a":
            self.set_steering(self.angular_speed)
        elif key == "d":
            self.set_steering(-self.angular_speed)
        elif key == "f":
            self.set_steering(self.max_turn_angular)
        elif key == "g":
            self.set_steering(-self.max_turn_angular)
        self.active_steering_key = key

    def handle_motion_key(self, key):
        now = time.time()
        if key == "p":
            if self.active_linear_key:
                self.superseded_linear_keys.add(self.active_linear_key)
            if self.active_steering_key:
                self.superseded_steering_keys.add(self.active_steering_key)
            self.preempt_all_motion()
            self.start_s_curve()
        elif key in LINEAR_KEYS:
            self.set_linear_motion(key)
        elif key in STEERING_KEYS:
            self.set_steering_motion(key)

        self.last_key_time = now

    def handle_key(self, key):
        now = time.time()
        if key == "q":
            self.linear_speed = clamp(self.linear_speed + self.linear_step, 0.0, self.max_linear)
            rospy.loginfo("linear speed: %.2f m/s", self.linear_speed)
        elif key == "e":
            self.linear_speed = clamp(self.linear_speed - self.linear_step, 0.0, self.max_linear)
            rospy.loginfo("linear speed: %.2f m/s", self.linear_speed)
        elif key == "z":
            self.angular_speed = clamp(self.angular_speed + self.angular_step, 0.0, self.max_angular)
            rospy.loginfo("angular speed: %.2f rad/s", self.angular_speed)
        elif key == "c":
            self.angular_speed = clamp(self.angular_speed - self.angular_step, 0.0, self.max_angular)
            rospy.loginfo("angular speed: %.2f rad/s", self.angular_speed)
        elif key == "h":
            self.print_help()
        else:
            return

        self.last_key_time = now

    def handle_keys(self, keys):
        if not keys:
            if time.time() - self.last_input_time > self.key_switch_quiet_time:
                self.superseded_linear_keys.clear()
                self.superseded_steering_keys.clear()
            return

        keys = [key.lower() for key in keys]
        self.last_input_time = time.time()

        if any(key in STOP_KEYS for key in keys):
            self.superseded_linear_keys.update(key for key in keys if key in LINEAR_KEYS)
            self.superseded_steering_keys.update(key for key in keys if key in STEERING_KEYS)
            if self.active_linear_key:
                self.superseded_linear_keys.add(self.active_linear_key)
            if self.active_steering_key:
                self.superseded_steering_keys.add(self.active_steering_key)
            self.stop()
            if " " in keys:
                rospy.logwarn("Emergency stop command sent")
            return

        for key in keys:
            if key not in MOTION_KEYS:
                self.handle_key(key)

        linear_keys = [
            key for key in keys
            if key in LINEAR_KEYS and (key == self.active_linear_key or key not in self.superseded_linear_keys)
        ]
        steering_keys = [
            key for key in keys
            if key in STEERING_KEYS and (key == self.active_steering_key or key not in self.superseded_steering_keys)
        ]

        if "p" in keys:
            self.handle_motion_key("p")
            return
        if linear_keys:
            self.handle_motion_key(linear_keys[-1])
        if steering_keys:
            self.handle_motion_key(steering_keys[-1])

    def run(self):
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard teleop must run in an interactive terminal")

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.print_help()

        rate = rospy.Rate(self.rate_hz)
        try:
            while not rospy.is_shutdown():
                self.handle_keys(self.read_keys(0.0))

                if self.s_curve_active:
                    self.publish_s_curve()
                elif self.auto_stop and self.last_key_time and time.time() - self.last_key_time > self.key_timeout:
                    self.stop()
                    self.last_key_time = 0.0
                else:
                    self.maybe_center_steering()
                    self.publish()

                rate.sleep()
        finally:
            self.stop()
            if self.settings is not None:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main():
    rospy.init_node("autolabor_keyboard_teleop")
    node = KeyboardTeleop()
    node.run()


if __name__ == "__main__":
    main()
