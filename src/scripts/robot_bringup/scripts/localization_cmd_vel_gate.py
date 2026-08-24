#!/usr/bin/env python3
"""Allow navigation velocity only while known-map localization is healthy."""

import threading

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String


def time_is_fresh(now, stamp, timeout):
    if stamp == rospy.Time(0):
        return False
    age = (now - stamp).to_sec()
    return 0.0 <= age <= timeout


class LocalizationVelocityGate:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/cmd_vel_unlocalized")
        self.output_topic = rospy.get_param("~output_topic", "/cmd_vel_navigation")
        self.status_topic = rospy.get_param(
            "~status_topic", "/fast_lio/localization_status"
        )
        self.cancel_topic = rospy.get_param("~cancel_topic", "/move_base/cancel")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base/goal")
        self.status_timeout = float(rospy.get_param("~status_timeout", 0.5))
        self.command_timeout = float(rospy.get_param("~command_timeout", 0.5))
        if self.status_timeout <= 0.0 or self.command_timeout <= 0.0:
            raise ValueError("timeouts must be positive")
        self.lock = threading.Lock()
        self.localized = False
        self.last_status = rospy.Time(0)
        self.last_command = rospy.Time(0)
        self.gate_open = False
        self.publisher = rospy.Publisher(self.output_topic, Twist, queue_size=5)
        self.cancel_publisher = rospy.Publisher(
            self.cancel_topic, GoalID, queue_size=1
        )
        self.status_subscriber = rospy.Subscriber(
            self.status_topic, String, self.status_callback, queue_size=5
        )
        self.command_subscriber = rospy.Subscriber(
            self.input_topic, Twist, self.command_callback, queue_size=5
        )
        self.goal_subscriber = rospy.Subscriber(
            self.goal_topic, MoveBaseActionGoal, self.goal_callback, queue_size=5
        )
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        rospy.on_shutdown(self.publish_stop)

    def status_callback(self, message):
        now = rospy.Time.now()
        localized = message.data.startswith("state=LOCALIZED;")
        with self.lock:
            self.localized = localized
            self.last_status = now
            ready, should_cancel = self.update_gate_state(now)
        if should_cancel:
            self.cancel_goal_and_stop()
        elif not ready:
            self.publish_stop()

    def ready(self, now):
        return (
            self.localized
            and time_is_fresh(now, self.last_status, self.status_timeout)
        )

    def update_gate_state(self, now):
        ready = self.ready(now)
        should_cancel = self.gate_open and not ready
        self.gate_open = ready
        return ready, should_cancel

    def command_callback(self, message):
        now = rospy.Time.now()
        with self.lock:
            self.last_command = now
            ready, should_cancel = self.update_gate_state(now)
        if should_cancel:
            self.cancel_goal_and_stop()
        else:
            self.publisher.publish(message if ready else Twist())

    def goal_callback(self, _message):
        now = rospy.Time.now()
        with self.lock:
            ready, should_cancel = self.update_gate_state(now)
        if should_cancel or not ready:
            self.cancel_goal_and_stop()

    def timer_callback(self, _event):
        now = rospy.Time.now()
        with self.lock:
            ready, should_cancel = self.update_gate_state(now)
            command_fresh = time_is_fresh(
                now, self.last_command, self.command_timeout
            )
        if should_cancel:
            self.cancel_goal_and_stop()
        elif not ready or not command_fresh:
            self.publisher.publish(Twist())

    def cancel_goal_and_stop(self):
        self.publisher.publish(Twist())
        self.cancel_publisher.publish(GoalID())

    def publish_stop(self):
        self.publisher.publish(Twist())


def main():
    rospy.init_node("fast_lio_localization_cmd_vel_gate")
    LocalizationVelocityGate()
    rospy.spin()


if __name__ == "__main__":
    main()
