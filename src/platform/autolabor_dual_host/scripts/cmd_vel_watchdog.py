#!/usr/bin/env python3
"""NVIDIA-side final velocity lease and graph-ownership guard."""

import json
import math
import threading
import time

import rosgraph
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


def evaluate_twist_values(values, max_linear_speed, max_angular_speed):
    """Return (accepted, reason) for a six-component Twist tuple."""
    if len(values) != 6 or not all(math.isfinite(float(value)) for value in values):
        return False, "command contains a non-finite value"
    linear_x, linear_y, linear_z, angular_x, angular_y, angular_z = values
    if any(abs(value) > 1.0e-6 for value in (linear_y, linear_z, angular_x, angular_y)):
        return False, "unsupported lateral/vertical Twist component"
    if abs(linear_x) > max_linear_speed + 1.0e-6:
        return False, "linear speed exceeds configured cap"
    if abs(angular_z) > max_angular_speed + 1.0e-6:
        return False, "angular speed exceeds configured cap"
    return True, "command accepted"


class CommandWatchdog:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_safe")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel")
        self.expected_input_publisher = rospy.get_param(
            "~expected_input_publisher", "/fod_navigation_mode"
        )
        self.expected_output_subscriber = rospy.get_param(
            "~expected_output_subscriber", "/m2_driver"
        )
        self.motion_enabled = self._strict_bool("~motion_enabled", False)
        self.command_timeout_sec = float(
            rospy.get_param("~command_timeout_sec", 0.25)
        )
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 50.0))
        self.graph_check_rate_hz = float(
            rospy.get_param("~graph_check_rate_hz", 5.0)
        )
        self.status_rate_hz = float(rospy.get_param("~status_rate_hz", 2.0))
        self.max_linear_speed = float(
            rospy.get_param("~max_linear_speed", 1.70)
        )
        self.max_angular_speed = float(
            rospy.get_param("~max_angular_speed", 0.60)
        )
        self.shutdown_zero_sec = float(
            rospy.get_param("~shutdown_zero_sec", 0.50)
        )
        self._validate_parameters()

        self.lock = threading.RLock()
        self.latest_command = None
        self.latest_receipt = None
        self.latest_command_error = "waiting for first command"
        self.graph_error = "command graph has not been checked"
        self.output_active = False
        self.output_reason = "starting"
        self.last_graph_check = 0.0
        self.last_status_publish = 0.0
        self.master = rosgraph.Master(rospy.get_name())
        self.master_pid = self.master.getPid()

        self.output_pub = rospy.Publisher(
            self.output_topic, Twist, queue_size=1, tcp_nodelay=True
        )
        self.active_pub = rospy.Publisher(
            "~active", Bool, queue_size=1, latch=True
        )
        self.status_pub = rospy.Publisher(
            "~status", String, queue_size=1, latch=True
        )
        self.diagnostics_pub = rospy.Publisher(
            "/diagnostics", DiagnosticArray, queue_size=5
        )
        self.input_sub = rospy.Subscriber(
            self.input_topic,
            Twist,
            self._command_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        self.timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate_hz), self._tick
        )
        rospy.on_shutdown(self._shutdown)
        self._publish_status(force=True)

    @staticmethod
    def _strict_bool(name, default):
        value = rospy.get_param(name, default)
        if type(value) is not bool:
            raise ValueError("{} must be a YAML boolean".format(name))
        return value

    def _validate_parameters(self):
        names = (
            self.input_topic,
            self.output_topic,
            self.expected_input_publisher,
            self.expected_output_subscriber,
        )
        if any(not isinstance(name, str) or not name.startswith("/") for name in names):
            raise ValueError("topics and expected node names must be absolute")
        if self.input_topic == self.output_topic:
            raise ValueError("input and output command topics must differ")
        if not 0.10 <= self.command_timeout_sec <= 0.50:
            raise ValueError("command_timeout_sec must be between 0.10 and 0.50")
        if not 20.0 <= self.publish_rate_hz <= 100.0:
            raise ValueError("publish_rate_hz must be between 20 and 100")
        if self.graph_check_rate_hz <= 0.0 or self.status_rate_hz <= 0.0:
            raise ValueError("graph/status rates must be positive")
        if not 0.0 < self.max_linear_speed <= 1.70:
            raise ValueError("max_linear_speed must be in (0, 1.70]")
        if not 0.0 < self.max_angular_speed <= 1.0:
            raise ValueError("max_angular_speed must be in (0, 1.0]")
        if not 0.25 <= self.shutdown_zero_sec <= 2.0:
            raise ValueError("shutdown_zero_sec must be between 0.25 and 2.0")

    @staticmethod
    def _topic_nodes(entries, topic):
        for candidate, nodes in entries:
            if candidate == topic:
                return set(nodes)
        return set()

    def _command_callback(self, message):
        values = (
            float(message.linear.x),
            float(message.linear.y),
            float(message.linear.z),
            float(message.angular.x),
            float(message.angular.y),
            float(message.angular.z),
        )
        accepted, reason = evaluate_twist_values(
            values, self.max_linear_speed, self.max_angular_speed
        )
        with self.lock:
            self.latest_receipt = time.monotonic()
            self.latest_command_error = "" if accepted else reason
            if accepted:
                command = Twist()
                command.linear.x = values[0]
                command.angular.z = values[5]
                self.latest_command = command
            else:
                self.latest_command = None

    def _inspect_graph(self):
        try:
            if self.master.getPid() != self.master_pid:
                return "ROS master was replaced"
            publishers, subscribers, _services = self.master.getSystemState()
        except Exception as exc:
            return "cannot inspect ROS graph: {}".format(exc)

        input_publishers = self._topic_nodes(publishers, self.input_topic)
        if input_publishers != {self.expected_input_publisher}:
            return "{} publisher must be exactly {}; current: {}".format(
                self.input_topic,
                self.expected_input_publisher,
                ", ".join(sorted(input_publishers)) or "none",
            )
        output_publishers = self._topic_nodes(publishers, self.output_topic)
        conflicts = output_publishers - {rospy.get_name()}
        if conflicts:
            return "{} has conflicting publishers: {}".format(
                self.output_topic, ", ".join(sorted(conflicts))
            )
        output_subscribers = self._topic_nodes(subscribers, self.output_topic)
        if self.expected_output_subscriber not in output_subscribers:
            return "{} is not connected to {}; subscribers: {}".format(
                self.output_topic,
                self.expected_output_subscriber,
                ", ".join(sorted(output_subscribers)) or "none",
            )
        return ""

    def _tick(self, _event=None):
        now = time.monotonic()
        with self.lock:
            if now - self.last_graph_check >= 1.0 / self.graph_check_rate_hz:
                self.graph_error = self._inspect_graph()
                self.last_graph_check = now

            command = self.latest_command
            receipt = self.latest_receipt
            command_error = self.latest_command_error
            graph_error = self.graph_error

        output = Twist()
        active = False
        if not self.motion_enabled:
            reason = "motion authorization is disabled"
        elif graph_error:
            reason = graph_error
        elif command_error:
            reason = command_error
        elif command is None or receipt is None:
            reason = "waiting for a valid command"
        elif now - receipt > self.command_timeout_sec:
            reason = "input command lease expired"
        else:
            output = command
            active = True
            reason = "fresh authorized command"

        self.output_pub.publish(output)
        with self.lock:
            self.output_active = active
            self.output_reason = reason
        self._publish_status()

    def _publish_status(self, force=False):
        now = time.monotonic()
        with self.lock:
            if not force and now - self.last_status_publish < 1.0 / self.status_rate_hz:
                return
            self.last_status_publish = now
            receipt = self.latest_receipt
            payload = {
                "motion_enabled": self.motion_enabled,
                "active": self.output_active,
                "reason": self.output_reason,
                "input_topic": self.input_topic,
                "output_topic": self.output_topic,
                "command_age_sec": (
                    round(max(0.0, now - receipt), 3) if receipt is not None else None
                ),
                "command_timeout_sec": self.command_timeout_sec,
                "max_linear_speed": self.max_linear_speed,
                "max_angular_speed": self.max_angular_speed,
            }
        encoded = json.dumps(payload, sort_keys=True)
        self.active_pub.publish(Bool(data=bool(payload["active"])))
        self.status_pub.publish(String(data=encoded))

        diagnostic = DiagnosticStatus()
        diagnostic.name = "autolabor/nvidia_cmd_vel_watchdog"
        diagnostic.hardware_id = "nvidia-m2-command-path"
        diagnostic.level = (
            DiagnosticStatus.OK
            if payload["active"]
            else DiagnosticStatus.WARN
            if not self.motion_enabled
            else DiagnosticStatus.ERROR
        )
        diagnostic.message = payload["reason"]
        diagnostic.values = [
            KeyValue(key=str(key), value=str(value))
            for key, value in sorted(payload.items())
        ]
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        array.status = [diagnostic]
        self.diagnostics_pub.publish(array)

    def _shutdown(self):
        deadline = time.monotonic() + self.shutdown_zero_sec
        zero = Twist()
        while time.monotonic() < deadline:
            try:
                self.output_pub.publish(zero)
            except Exception:
                break
            time.sleep(0.02)


def main():
    rospy.init_node("nvidia_cmd_vel_watchdog", anonymous=False)
    CommandWatchdog()
    rospy.spin()


if __name__ == "__main__":
    main()
