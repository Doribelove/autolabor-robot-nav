#!/usr/bin/env python3
"""Allow navigation velocity only while known-map localization is healthy."""

import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class LocalizationVelocityGate:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_unlocalized")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel_navigation")
        self.status_topic = rospy.get_param(
            "~status_topic", "/fast_lio/localization_status"
        )
        self.status_timeout = float(rospy.get_param("~status_timeout", 0.5))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.5))
        if self.status_timeout <= 0.0 or self.command_timeout <= 0.0:
            raise ValueError("timeouts must be positive")
        self.lock = threading.Lock()
        self.localized = False
        self.last_status = rospy.Time(0)
        self.last_command = rospy.Time(0)
        self.publisher = rospy.Publisher(self.output_topic, Twist, queue_size=5)
        self.status_subscriber = rospy.Subscriber(
            self.status_topic, String, self.status_callback, queue_size=5
        )
        self.command_subscriber = rospy.Subscriber(
            self.input_topic, Twist, self.command_callback, queue_size=5
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.on_shutdown(self.publish_stop)

    def status_callback(self, message):
        now = rospy.Time.now()
        localized = message.data.startswith("state=LOCALIZED;")
        with self.lock:
            was_localized = self.localized
            self.localized = localized
            self.last_status = now
        if was_localized and not localized:
            self.publish_stop()

    def ready(self, now):
        return (
            self.localized
            and self.last_status != rospy.Time(0)
            and (now - self.last_status).to_sec() <= self.status_timeout
        )

    def command_callback(self, message):
        now = rospy.Time.now()
        with self.lock:
            self.last_command = now
            ready = self.ready(now)
        self.publisher.publish(message if ready else Twist())

    def timer_callback(self, _event):
        now = rospy.Time.now()
        with self.lock:
            ready = self.ready(now)
            command_fresh = (
                self.last_command != rospy.Time(0)
                and (now - self.last_command).to_sec() <= self.command_timeout
            )
        if not ready or not command_fresh:
            self.publisher.publish(Twist())

    def publish_stop(self):
        self.publisher.publish(Twist())


def main():
    rospy.init_node("fast_lio_localization_cmd_vel_gate")
    LocalizationVelocityGate()
    rospy.spin()


if __name__ == "__main__":
    main()
