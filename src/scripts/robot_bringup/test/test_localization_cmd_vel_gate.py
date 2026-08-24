#!/usr/bin/env python3

import copy
import importlib.util
from pathlib import Path
import threading
import unittest
from unittest import mock

import rospy
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseActionGoal
from std_msgs.msg import String


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "localization_cmd_vel_gate.py"
)
SPEC = importlib.util.spec_from_file_location(
    "localization_cmd_vel_gate", str(SCRIPT)
)
GATE_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(GATE_MODULE)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(copy.deepcopy(message))


def make_gate():
    gate = GATE_MODULE.LocalizationVelocityGate.__new__(
        GATE_MODULE.LocalizationVelocityGate
    )
    gate.lock = threading.Lock()
    gate.localized = False
    gate.last_status = rospy.Time(0)
    gate.last_command = rospy.Time(0)
    gate.gate_open = False
    gate.status_timeout = 0.5
    gate.command_timeout = 0.5
    gate.publisher = FakePublisher()
    gate.cancel_publisher = FakePublisher()
    return gate


class LocalizationVelocityGateTest(unittest.TestCase):
    def test_only_fresh_exact_localized_state_forwards_velocity(self):
        gate = make_gate()
        now = rospy.Time.from_sec(10.0)
        with mock.patch.object(GATE_MODULE.rospy.Time, "now", return_value=now):
            gate.status_callback(String(data="state=LOCALIZED;overlap=0.7"))
            command = Twist()
            command.linear.x = 0.4
            gate.command_callback(command)

        self.assertEqual(0.4, gate.publisher.messages[-1].linear.x)
        self.assertEqual([], gate.cancel_publisher.messages)

    def test_goal_before_localization_is_cancelled_and_velocity_is_zero(self):
        gate = make_gate()
        now = rospy.Time.from_sec(10.0)
        with mock.patch.object(GATE_MODULE.rospy.Time, "now", return_value=now):
            gate.status_callback(String(data="state=WAITING_INITIAL_POSE;"))
            gate.goal_callback(MoveBaseActionGoal())

        self.assertEqual(Twist(), gate.publisher.messages[-1])
        self.assertEqual(1, len(gate.cancel_publisher.messages))

    def test_leaving_localized_cancels_old_goal_immediately(self):
        gate = make_gate()
        localized_time = rospy.Time.from_sec(10.0)
        with mock.patch.object(
            GATE_MODULE.rospy.Time, "now", return_value=localized_time
        ):
            gate.status_callback(String(data="state=LOCALIZED;overlap=0.7"))

        degraded_time = rospy.Time.from_sec(10.1)
        with mock.patch.object(
            GATE_MODULE.rospy.Time, "now", return_value=degraded_time
        ):
            gate.status_callback(String(data="state=DEGRADED;overlap=0.1"))

        self.assertEqual(Twist(), gate.publisher.messages[-1])
        self.assertEqual(1, len(gate.cancel_publisher.messages))

    def test_status_timeout_cancels_once_and_stays_zero(self):
        gate = make_gate()
        gate.localized = True
        gate.gate_open = True
        gate.last_status = rospy.Time.from_sec(10.0)
        gate.last_command = rospy.Time.from_sec(10.4)

        with mock.patch.object(
            GATE_MODULE.rospy.Time,
            "now",
            side_effect=(rospy.Time.from_sec(10.6), rospy.Time.from_sec(10.7)),
        ):
            gate.timer_callback(None)
            gate.timer_callback(None)

        self.assertEqual(2, len(gate.publisher.messages))
        self.assertTrue(all(message == Twist() for message in gate.publisher.messages))
        self.assertEqual(1, len(gate.cancel_publisher.messages))

    def test_clock_rollback_is_not_considered_fresh(self):
        now = rospy.Time.from_sec(10.0)
        future = rospy.Time.from_sec(11.0)
        self.assertFalse(GATE_MODULE.time_is_fresh(now, future, 0.5))

        gate = make_gate()
        gate.localized = True
        gate.gate_open = True
        gate.last_status = future
        gate.last_command = future
        with mock.patch.object(GATE_MODULE.rospy.Time, "now", return_value=now):
            gate.timer_callback(None)
        self.assertEqual(Twist(), gate.publisher.messages[-1])
        self.assertEqual(1, len(gate.cancel_publisher.messages))


if __name__ == "__main__":
    unittest.main()
