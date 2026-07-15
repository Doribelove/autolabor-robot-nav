#!/usr/bin/env python3

from collections import OrderedDict
import copy
import importlib.util
import math
from pathlib import Path
import threading
import unittest
from unittest import mock

import rospy
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, Twist


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "gps_goal_speed_limiter.py"
SPEC = importlib.util.spec_from_file_location("gps_goal_speed_limiter", str(SCRIPT_PATH))
LIMITER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LIMITER)


def goal_id(identifier, stamp_sec):
    msg = GoalID()
    msg.id = identifier
    msg.stamp = rospy.Time.from_sec(stamp_sec)
    return msg


class GoalSpeedCapTest(unittest.TestCase):
    def test_hard_stop_distance_is_independent_input(self):
        self.assertEqual(LIMITER.goal_speed_cap(0.2, 0.4, 0.2, 0.15), 0.0)
        self.assertAlmostEqual(
            LIMITER.goal_speed_cap(1.2, 0.4, 0.2, 0.15),
            math.sqrt(0.8),
        )

    def test_minimum_approach_speed_applies_outside_hard_stop(self):
        self.assertEqual(LIMITER.goal_speed_cap(0.21, 0.4, 0.2, 0.15), 0.15)

    def test_invalid_hard_stop_distance_is_rejected(self):
        with self.assertRaises(ValueError):
            LIMITER.goal_speed_cap(1.0, 0.4, -0.1, 0.15)

    def test_yaw_rate_scales_with_forward_speed_to_preserve_curvature(self):
        self.assertAlmostEqual(
            LIMITER.angular_velocity_at_limited_speed(1.0, 0.4, -0.5),
            -0.2,
        )

    def test_yaw_rate_scaling_rejects_a_speed_increase(self):
        with self.assertRaises(ValueError):
            LIMITER.angular_velocity_at_limited_speed(0.5, 0.6, 0.2)

    def test_hard_stop_must_be_strictly_inside_planner_tolerance(self):
        LIMITER.validate_goal_distances(0.2, 0.3)
        with self.assertRaises(ValueError):
            LIMITER.validate_goal_distances(0.3, 0.3)


class ActionlibCancelMatchingTest(unittest.TestCase):
    class FakePublisher:
        def __init__(self):
            self.messages = []

        def publish(self, msg):
            self.messages.append(copy.deepcopy(msg))

    @staticmethod
    def limiter_with_active_goal():
        limiter = LIMITER.GpsGoalSpeedLimiter.__new__(LIMITER.GpsGoalSpeedLimiter)
        limiter.lock = threading.Lock()
        limiter.active_goal_id = goal_id("active", 10.0)
        limiter.stop_latched = False
        limiter.latest_cmd = Twist()
        limiter.latest_cmd.linear.x = 1.0
        limiter.latest_cmd_time = rospy.Time.from_sec(10.0)
        limiter.last_cancel_stamp = rospy.Time()
        limiter.blocked_goal_ids = OrderedDict()
        limiter.cmd_pub = ActionlibCancelMatchingTest.FakePublisher()
        return limiter

    def test_specific_cancel_matches_only_same_goal_id(self):
        active = goal_id("active", 10.0)
        self.assertTrue(
            LIMITER.cancel_request_matches_goal(goal_id("active", 0.0), active)
        )
        self.assertFalse(
            LIMITER.cancel_request_matches_goal(goal_id("old", 0.0), active)
        )

    def test_zero_id_and_zero_stamp_cancels_active_goal(self):
        self.assertTrue(
            LIMITER.cancel_request_matches_goal(goal_id("", 0.0), goal_id("active", 10.0))
        )

    def test_timestamp_cancel_matches_only_older_goals(self):
        cancel = goal_id("", 10.0)
        self.assertTrue(
            LIMITER.cancel_request_matches_goal(cancel, goal_id("older", 9.0))
        )
        self.assertTrue(
            LIMITER.cancel_request_matches_goal(cancel, goal_id("same", 10.0))
        )
        self.assertFalse(
            LIMITER.cancel_request_matches_goal(cancel, goal_id("newer", 11.0))
        )

    def test_cancel_without_active_goal_does_not_match(self):
        self.assertFalse(
            LIMITER.cancel_request_matches_goal(goal_id("", 0.0), None)
        )

    def test_stale_specific_cancel_does_not_latch_or_publish_stop(self):
        limiter = self.limiter_with_active_goal()
        with mock.patch.object(
            LIMITER.rospy.Time,
            "now",
            return_value=rospy.Time.from_sec(20.0),
        ):
            limiter.cancel_callback(goal_id("old", 0.0))
        self.assertFalse(limiter.stop_latched)
        self.assertEqual(limiter.latest_cmd.linear.x, 1.0)
        self.assertEqual(limiter.cmd_pub.messages, [])

    def test_matching_specific_cancel_latches_and_publishes_full_stop(self):
        limiter = self.limiter_with_active_goal()
        with mock.patch.object(
            LIMITER.rospy.Time,
            "now",
            return_value=rospy.Time.from_sec(20.0),
        ):
            limiter.cancel_callback(goal_id("active", 0.0))
        self.assertTrue(limiter.stop_latched)
        self.assertIsNone(limiter.latest_cmd_time)
        self.assertEqual(len(limiter.cmd_pub.messages), 1)
        self.assertEqual(limiter.cmd_pub.messages[0], Twist())

    def test_zero_stamp_cancel_does_not_block_a_future_goal(self):
        limiter = self.limiter_with_active_goal()
        limiter._remember_cancel_request(goal_id("", 0.0))

        self.assertFalse(limiter._goal_was_stopped(goal_id("future", 20.0)))

    def test_timestamp_cancel_blocks_even_a_zero_stamped_goal(self):
        limiter = self.limiter_with_active_goal()
        limiter._remember_cancel_request(goal_id("", 10.0))

        self.assertTrue(limiter._goal_was_stopped(goal_id("late-delivery", 0.0)))

    def test_specific_cancel_received_before_goal_is_remembered(self):
        limiter = self.limiter_with_active_goal()
        limiter._remember_cancel_request(goal_id("delayed", 0.0))

        self.assertTrue(limiter._goal_was_stopped(goal_id("delayed", 30.0)))
        self.assertFalse(limiter._goal_was_stopped(goal_id("different", 30.0)))


class GoalPoseIdentityTest(unittest.TestCase):
    @staticmethod
    def limiter_without_ros_setup():
        limiter = LIMITER.GpsGoalSpeedLimiter.__new__(LIMITER.GpsGoalSpeedLimiter)
        limiter.lock = threading.Lock()
        limiter.active_goal_id = None
        limiter.current_goal = None
        return limiter

    @staticmethod
    def pose_at(x):
        pose = PoseStamped()
        pose.pose.position.x = x
        return pose

    def test_pose_only_goal_is_a_startup_compatibility_source(self):
        limiter = self.limiter_without_ros_setup()
        limiter.goal_callback(self.pose_at(1.0))
        self.assertEqual(limiter.current_goal.pose.position.x, 1.0)

    def test_delayed_pose_only_goal_cannot_overwrite_action_goal(self):
        limiter = self.limiter_without_ros_setup()
        limiter.active_goal_id = goal_id("new", 20.0)
        limiter.current_goal = self.pose_at(2.0)
        limiter.goal_callback(self.pose_at(1.0))
        self.assertEqual(limiter.current_goal.pose.position.x, 2.0)


if __name__ == "__main__":
    unittest.main()
