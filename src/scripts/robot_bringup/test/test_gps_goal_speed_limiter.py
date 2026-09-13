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
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "gps_goal_speed_limiter.py"
SPEC = importlib.util.spec_from_file_location("gps_goal_speed_limiter", str(SCRIPT_PATH))
LIMITER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(LIMITER)


def goal_id(identifier, stamp_sec):
    msg = GoalID()
    msg.id = identifier
    msg.stamp = rospy.Time.from_sec(stamp_sec)
    return msg


def action_goal(identifier, stamp_sec, x=1.0):
    msg = MoveBaseActionGoal()
    msg.goal_id = goal_id(identifier, stamp_sec)
    msg.goal.target_pose.header.frame_id = "camera_init"
    msg.goal.target_pose.pose.position.x = x
    return msg


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


class GoalSpeedCapTest(unittest.TestCase):
    def test_boolean_parameter_parses_ros_and_shell_forms(self):
        for value in (True, 1, "true", "YES", "on"):
            self.assertTrue(LIMITER.boolean_parameter(value, "test"))
        for value in (False, 0, "false", "NO", "off"):
            self.assertFalse(LIMITER.boolean_parameter(value, "test"))

    def test_boolean_parameter_rejects_ambiguous_value(self):
        with self.assertRaises(ValueError):
            LIMITER.boolean_parameter("sometimes", "test")

    def test_speed_cap_uses_remaining_distance_outside_hard_stop(self):
        self.assertAlmostEqual(
            LIMITER.goal_speed_cap(1.2, 0.4, 0.2, 0.15),
            math.sqrt(0.8),
        )

    def test_speed_cap_is_zero_at_hard_stop_distance(self):
        self.assertEqual(LIMITER.goal_speed_cap(0.2, 0.4, 0.2, 0.15), 0.0)

    def test_minimum_approach_speed_applies_outside_hard_stop(self):
        self.assertEqual(LIMITER.goal_speed_cap(0.21, 0.4, 0.2, 0.15), 0.15)

    def test_hard_stop_must_be_strictly_inside_planner_tolerance(self):
        LIMITER.validate_goal_distances(0.2, 0.3)
        with self.assertRaises(ValueError):
            LIMITER.validate_goal_distances(0.3, 0.3)
        with self.assertRaises(ValueError):
            LIMITER.validate_goal_distances(0.31, 0.3)


class CurvaturePreservingSpeedLimitTest(unittest.TestCase):
    def test_yaw_rate_scales_with_forward_speed(self):
        self.assertAlmostEqual(
            LIMITER.angular_velocity_at_limited_speed(1.0, 0.4, 0.5),
            0.2,
        )

    def test_negative_yaw_rate_keeps_its_direction(self):
        self.assertAlmostEqual(
            LIMITER.angular_velocity_at_limited_speed(0.8, 0.2, -0.6),
            -0.15,
        )

    def test_zero_speed_cap_also_zeroes_yaw_rate(self):
        self.assertEqual(
            LIMITER.angular_velocity_at_limited_speed(0.3, 0.0, 0.7),
            0.0,
        )

    def test_speed_increase_is_rejected(self):
        with self.assertRaises(ValueError):
            LIMITER.angular_velocity_at_limited_speed(0.5, 0.6, 0.2)

    def test_non_positive_original_speed_is_rejected(self):
        with self.assertRaises(ValueError):
            LIMITER.angular_velocity_at_limited_speed(0.0, 0.0, 0.2)

    def test_non_finite_yaw_rate_is_rejected(self):
        with self.assertRaises(ValueError):
            LIMITER.angular_velocity_at_limited_speed(0.5, 0.2, math.inf)


class GoalLimiterOutputTest(unittest.TestCase):
    @classmethod
    def limiter_at_distance(cls, distance, command):
        now = rospy.Time.from_sec(10.0)
        odom = Odometry()
        odom.header.frame_id = "camera_init"
        goal = PoseStamped()
        goal.header.frame_id = "camera_init"
        goal.pose.position.x = distance

        limiter = LIMITER.GpsGoalSpeedLimiter.__new__(LIMITER.GpsGoalSpeedLimiter)
        limiter.lock = threading.Lock()
        limiter.latest_cmd = copy.deepcopy(command)
        limiter.latest_cmd_time = now
        limiter.latest_odom = odom
        limiter.latest_odom_time = now
        limiter.current_goal = goal
        limiter.active_goal_id = goal_id("active", 9.0)
        limiter.active_goal_is_terminal = True
        limiter.stop_latched = False
        limiter.arrival_latched = False
        limiter.near_goal_started_time = None
        limiter.near_goal_closest_distance = None
        limiter.last_cancel_stamp = rospy.Time()
        limiter.blocked_goal_ids = OrderedDict()
        limiter.cmd_timeout = 0.5
        limiter.odom_timeout = 1.0
        limiter.speed_cap_enabled = True
        limiter.comfortable_decel = 0.4
        limiter.hard_stop_distance = 0.2
        limiter.min_approach_speed = 0.15
        limiter.near_goal_commit_distance = 1.0
        limiter.near_goal_timeout = 15.0
        limiter.near_goal_max_regression = 0.5
        limiter.cmd_pub = FakePublisher()
        return limiter, now

    def test_hard_stop_publishes_a_complete_zero_twist(self):
        command = Twist()
        command.linear.x = -0.4
        command.linear.y = 0.2
        command.linear.z = 0.1
        command.angular.x = 0.3
        command.angular.y = -0.2
        command.angular.z = 0.5
        limiter, now = self.limiter_at_distance(0.2, command)

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        self.assertEqual(limiter.cmd_pub.messages, [Twist()])
        self.assertTrue(limiter.stop_latched)
        self.assertTrue(limiter.arrival_latched)

        # A noisy GPS sample just outside the threshold and a new planner
        # command must not restart the already-arrived action.
        limiter.current_goal.pose.position.x = 0.21
        limiter.latest_cmd = copy.deepcopy(command)
        limiter.latest_cmd_time = now
        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)
        self.assertEqual(limiter.cmd_pub.messages[-1], Twist())

    def test_gap_inside_planner_tolerance_is_not_hard_stopped(self):
        command = Twist()
        command.linear.x = 1.0
        command.angular.z = 0.5
        limiter, now = self.limiter_at_distance(0.25, command)

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        output = limiter.cmd_pub.messages[-1]
        self.assertAlmostEqual(output.linear.x, 0.2)
        self.assertAlmostEqual(output.angular.z, 0.1)

    def test_disabled_speed_cap_forwards_terminal_command_unchanged(self):
        command = Twist()
        command.linear.x = 1.0
        command.angular.z = 0.5
        limiter, now = self.limiter_at_distance(0.25, command)
        limiter.speed_cap_enabled = False

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        self.assertEqual(limiter.cmd_pub.messages[-1], command)
        self.assertFalse(limiter.stop_latched)
        self.assertFalse(limiter.arrival_latched)

    def test_disabled_speed_cap_keeps_hard_stop_latch(self):
        command = Twist()
        command.linear.x = 1.0
        limiter, now = self.limiter_at_distance(0.2, command)
        limiter.speed_cap_enabled = False

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        self.assertEqual(limiter.cmd_pub.messages, [Twist()])
        self.assertTrue(limiter.stop_latched)
        self.assertTrue(limiter.arrival_latched)

    def test_arrival_latches_even_when_planner_command_has_expired(self):
        command = Twist()
        command.linear.x = 0.4
        limiter, now = self.limiter_at_distance(0.2, command)
        limiter.latest_cmd_time = None

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        self.assertTrue(limiter.stop_latched)
        self.assertTrue(limiter.arrival_latched)
        self.assertEqual(limiter.cmd_pub.messages, [Twist()])

    def test_missing_odometry_during_active_goal_fails_stopped(self):
        command = Twist()
        command.linear.x = 0.4
        command.angular.z = 0.3
        limiter, now = self.limiter_at_distance(2.0, command)
        limiter.latest_odom = None
        limiter.latest_odom_time = None

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        self.assertEqual(limiter.cmd_pub.messages, [Twist()])

    def test_stale_odometry_during_active_goal_fails_stopped(self):
        command = Twist()
        command.linear.x = 0.4
        limiter, now = self.limiter_at_distance(2.0, command)
        limiter.latest_odom_time = now - rospy.Duration.from_sec(1.1)

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        self.assertEqual(limiter.cmd_pub.messages, [Twist()])

    def test_non_finite_goal_position_fails_stopped(self):
        command = Twist()
        command.linear.x = 0.4
        limiter, now = self.limiter_at_distance(2.0, command)
        limiter.current_goal.pose.position.x = math.nan

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        self.assertEqual(limiter.cmd_pub.messages, [Twist()])

    def test_non_finite_odometry_position_fails_stopped(self):
        command = Twist()
        command.linear.x = 0.4
        limiter, now = self.limiter_at_distance(2.0, command)
        limiter.latest_odom.pose.pose.position.y = math.inf

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        self.assertEqual(limiter.cmd_pub.messages, [Twist()])

    def test_new_goal_releases_arrival_latch_and_resets_near_goal_fence(self):
        command = Twist()
        command.linear.x = 0.4
        limiter, now = self.limiter_at_distance(0.2, command)
        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        limiter.near_goal_started_time = now
        limiter.near_goal_closest_distance = 0.2
        limiter.action_goal_callback(action_goal("next", 20.0, x=5.0))

        self.assertFalse(limiter.stop_latched)
        self.assertFalse(limiter.arrival_latched)
        self.assertIsNone(limiter.near_goal_started_time)
        self.assertIsNone(limiter.near_goal_closest_distance)

    def test_rolling_intermediate_goal_bypasses_terminal_slowdown_and_latches(self):
        command = Twist()
        command.linear.x = 1.0
        command.angular.z = 0.4
        limiter, now = self.limiter_at_distance(0.1, command)
        limiter.active_goal_id = goal_id(
            "gps_long_range/100-1/1/intermediate",
            9.0,
        )
        limiter.active_goal_is_terminal = False

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        self.assertEqual(limiter.cmd_pub.messages[-1], command)
        self.assertFalse(limiter.stop_latched)
        self.assertFalse(limiter.arrival_latched)
        self.assertIsNone(limiter.near_goal_started_time)

    def test_rolling_intermediate_goal_still_fails_stopped_on_stale_odom(self):
        command = Twist()
        command.linear.x = 1.0
        limiter, now = self.limiter_at_distance(10.0, command)
        limiter.active_goal_is_terminal = False
        limiter.latest_odom_time = now - rospy.Duration.from_sec(1.1)

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        self.assertEqual(limiter.cmd_pub.messages, [Twist()])

    def test_moving_away_after_near_goal_commit_latches_stop(self):
        command = Twist()
        command.linear.x = 0.3
        limiter, now = self.limiter_at_distance(0.9, command)
        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        later = rospy.Time.from_sec(10.2)
        limiter.current_goal.pose.position.x = 1.41
        limiter.latest_cmd = copy.deepcopy(command)
        limiter.latest_cmd_time = later
        limiter.latest_odom_time = later
        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=later):
            limiter.timer_callback(None)

        self.assertTrue(limiter.stop_latched)
        self.assertEqual(limiter.cmd_pub.messages[-1], Twist())

    def test_near_goal_timeout_latches_stop_even_after_some_progress(self):
        command = Twist()
        command.linear.x = 0.3
        limiter, now = self.limiter_at_distance(0.9, command)
        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            limiter.timer_callback(None)

        later = rospy.Time.from_sec(25.1)
        limiter.current_goal.pose.position.x = 0.4
        limiter.latest_cmd = copy.deepcopy(command)
        limiter.latest_cmd_time = later
        limiter.latest_odom_time = later
        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=later):
            limiter.timer_callback(None)

        self.assertTrue(limiter.stop_latched)
        self.assertEqual(limiter.cmd_pub.messages[-1], Twist())


class ActionGoalIdentityTest(unittest.TestCase):
    @staticmethod
    def limiter_with_active_goal(identifier="active", stamp_sec=10.0):
        limiter = LIMITER.GpsGoalSpeedLimiter.__new__(LIMITER.GpsGoalSpeedLimiter)
        limiter.lock = threading.Lock()
        limiter.active_goal_id = goal_id(identifier, stamp_sec)
        limiter.active_goal_is_terminal = True
        limiter.current_goal = PoseStamped()
        limiter.current_goal.pose.position.x = 2.0
        limiter.stop_latched = False
        limiter.arrival_latched = False
        limiter.near_goal_started_time = None
        limiter.near_goal_closest_distance = None
        limiter.latest_cmd = Twist()
        limiter.latest_cmd.linear.x = 1.0
        limiter.latest_cmd_time = rospy.Time.from_sec(stamp_sec)
        limiter.last_cancel_stamp = rospy.Time()
        limiter.blocked_goal_ids = OrderedDict()
        limiter.cmd_pub = FakePublisher()
        return limiter

    @staticmethod
    def status_message(identifier, state, stamp_sec=10.0):
        item = GoalStatus()
        item.goal_id = goal_id(identifier, stamp_sec)
        item.status = state
        message = GoalStatusArray()
        message.status_list = [item]
        return message

    def test_specific_cancel_matches_only_the_same_goal_id(self):
        active = goal_id("active", 10.0)
        self.assertTrue(
            LIMITER.cancel_request_matches_goal(goal_id("active", 0.0), active)
        )
        self.assertFalse(
            LIMITER.cancel_request_matches_goal(goal_id("old", 0.0), active)
        )

    def test_timestamp_cancel_matches_only_older_goals(self):
        cancel = goal_id("", 10.0)
        self.assertTrue(
            LIMITER.cancel_request_matches_goal(cancel, goal_id("older", 9.0))
        )
        self.assertFalse(
            LIMITER.cancel_request_matches_goal(cancel, goal_id("newer", 11.0))
        )

    def test_zero_id_and_zero_stamp_cancel_matches_active_goal(self):
        self.assertTrue(
            LIMITER.cancel_request_matches_goal(
                goal_id("", 0.0), goal_id("active", 10.0)
            )
        )

    def test_zero_stamp_cancel_does_not_block_a_future_goal(self):
        limiter = self.limiter_with_active_goal()
        limiter.cancel_callback(goal_id("", 0.0))

        limiter.action_goal_callback(action_goal("future", 20.0))

        self.assertFalse(limiter.stop_latched)

    def test_timestamp_cancel_blocks_a_late_delivered_older_goal(self):
        limiter = self.limiter_with_active_goal("old", 9.0)
        limiter.cancel_callback(goal_id("", 10.0))

        limiter.action_goal_callback(action_goal("late-delivery", 9.5))

        self.assertTrue(limiter.stop_latched)

    def test_old_specific_cancel_does_not_stop_new_active_goal(self):
        limiter = self.limiter_with_active_goal("new", 20.0)

        limiter.cancel_callback(goal_id("old", 0.0))

        self.assertFalse(limiter.stop_latched)
        self.assertEqual(limiter.latest_cmd.linear.x, 1.0)
        self.assertEqual(limiter.cmd_pub.messages, [])

    def test_matching_cancel_latches_and_publishes_full_stop(self):
        limiter = self.limiter_with_active_goal()

        limiter.cancel_callback(goal_id("active", 0.0))

        self.assertTrue(limiter.stop_latched)
        self.assertIsNone(limiter.latest_cmd_time)
        self.assertEqual(limiter.cmd_pub.messages, [Twist()])

    def test_delayed_current_goal_cannot_unlock_or_overwrite_cancelled_goal(self):
        limiter = self.limiter_with_active_goal()
        limiter.cancel_callback(goal_id("active", 0.0))
        delayed_pose = PoseStamped()
        delayed_pose.pose.position.x = 99.0

        limiter.goal_callback(delayed_pose)

        self.assertTrue(limiter.stop_latched)
        self.assertEqual(limiter.current_goal.pose.position.x, 2.0)

    def test_new_identity_bearing_goal_unlocks_after_cancel(self):
        limiter = self.limiter_with_active_goal()
        limiter.cancel_callback(goal_id("active", 0.0))

        limiter.action_goal_callback(action_goal("new", 20.0, x=5.0))

        self.assertFalse(limiter.stop_latched)
        self.assertEqual(limiter.active_goal_id.id, "new")
        self.assertEqual(limiter.current_goal.pose.position.x, 5.0)
        self.assertIsNone(limiter.latest_cmd_time)
        self.assertEqual(limiter.cmd_pub.messages[-1], Twist())

    def test_managed_action_goal_marks_only_intermediate_segments_non_terminal(self):
        limiter = self.limiter_with_active_goal()

        limiter.action_goal_callback(
            action_goal(
                "gps_long_range/100-1/1/intermediate",
                20.0,
            )
        )
        self.assertFalse(limiter.active_goal_is_terminal)

        limiter.action_goal_callback(
            action_goal(
                "gps_long_range/100-1/2/final",
                21.0,
            )
        )
        self.assertTrue(limiter.active_goal_is_terminal)

        limiter.action_goal_callback(action_goal("rviz-goal", 22.0))
        self.assertTrue(limiter.active_goal_is_terminal)

    def test_contiguous_route_segment_keeps_fresh_command_without_zero_pulse(self):
        limiter = self.limiter_with_active_goal(
            "gps_long_range/100-1/1/intermediate",
            20.0,
        )
        limiter.active_goal_is_terminal = False

        limiter.action_goal_callback(
            action_goal(
                "gps_long_range/100-1/2/intermediate",
                21.0,
                x=25.0,
            )
        )

        self.assertFalse(limiter.stop_latched)
        self.assertFalse(limiter.active_goal_is_terminal)
        self.assertEqual(limiter.latest_cmd.linear.x, 1.0)
        self.assertEqual(
            limiter.latest_cmd_time,
            rospy.Time.from_sec(20.0),
        )
        self.assertEqual(limiter.cmd_pub.messages, [])

    def test_contiguous_transition_to_final_goal_is_also_seamless(self):
        limiter = self.limiter_with_active_goal(
            "gps_long_range/100-1/7/intermediate",
            20.0,
        )
        limiter.active_goal_is_terminal = False

        limiter.action_goal_callback(
            action_goal(
                "gps_long_range/100-1/8/final",
                21.0,
                x=200.0,
            )
        )

        self.assertFalse(limiter.stop_latched)
        self.assertTrue(limiter.active_goal_is_terminal)
        self.assertEqual(limiter.latest_cmd.linear.x, 1.0)
        self.assertEqual(limiter.cmd_pub.messages, [])

    def test_new_route_token_still_fences_the_previous_command_with_zero(self):
        limiter = self.limiter_with_active_goal(
            "gps_long_range/100-1/1/intermediate",
            20.0,
        )
        limiter.active_goal_is_terminal = False

        limiter.action_goal_callback(
            action_goal(
                "gps_long_range/101-1/1/intermediate",
                21.0,
                x=15.0,
            )
        )

        self.assertFalse(limiter.stop_latched)
        self.assertIsNone(limiter.latest_cmd_time)
        self.assertEqual(limiter.cmd_pub.messages, [Twist()])

    def test_cancel_received_before_same_goal_keeps_that_goal_stopped(self):
        limiter = self.limiter_with_active_goal()
        limiter.cancel_callback(goal_id("delayed", 0.0))

        limiter.action_goal_callback(action_goal("delayed", 30.0))

        self.assertTrue(limiter.stop_latched)

    def test_terminal_status_stops_only_the_matching_active_goal(self):
        limiter = self.limiter_with_active_goal("new", 20.0)

        limiter.status_callback(self.status_message("old", GoalStatus.SUCCEEDED))
        self.assertFalse(limiter.stop_latched)
        self.assertEqual(limiter.cmd_pub.messages, [])

        limiter.status_callback(self.status_message("new", GoalStatus.ABORTED))
        self.assertTrue(limiter.stop_latched)
        self.assertEqual(limiter.cmd_pub.messages, [Twist()])

    def test_status_binds_empty_action_id_before_terminal_state(self):
        limiter = self.limiter_with_active_goal("", 20.0)

        limiter.status_callback(
            self.status_message("generated-id", GoalStatus.ACTIVE, stamp_sec=20.0)
        )

        self.assertEqual(limiter.active_goal_id.id, "generated-id")
        self.assertFalse(limiter.stop_latched)

        limiter.status_callback(
            self.status_message("generated-id", GoalStatus.SUCCEEDED, stamp_sec=20.0)
        )

        self.assertTrue(limiter.stop_latched)
        self.assertEqual(limiter.cmd_pub.messages, [Twist()])

    def test_empty_id_does_not_bind_to_an_older_active_status(self):
        limiter = self.limiter_with_active_goal("", 20.0)

        limiter.status_callback(
            self.status_message("old-id", GoalStatus.ACTIVE, stamp_sec=10.0)
        )

        self.assertEqual(limiter.active_goal_id.id, "")
        self.assertFalse(limiter.stop_latched)


class CancelPublishOrderingTest(unittest.TestCase):
    class BlockingPublisher(FakePublisher):
        def __init__(self):
            super().__init__()
            self.nonzero_publish_entered = threading.Event()
            self.release_nonzero_publish = threading.Event()

        def publish(self, msg):
            if msg.linear.x != 0.0:
                self.nonzero_publish_entered.set()
                if not self.release_nonzero_publish.wait(2.0):
                    raise RuntimeError("test timed out waiting to release nonzero publish")
            super().publish(msg)

    def test_cancel_stop_is_ordered_after_an_in_progress_timer_command(self):
        command = Twist()
        command.linear.x = 0.3
        limiter, now = GoalLimiterOutputTest.limiter_at_distance(10.0, command)
        publisher = self.BlockingPublisher()
        limiter.cmd_pub = publisher

        with mock.patch.object(LIMITER.rospy.Time, "now", return_value=now):
            timer_thread = threading.Thread(target=limiter.timer_callback, args=(None,))
            cancel_thread = None
            timer_thread.start()
            try:
                self.assertTrue(publisher.nonzero_publish_entered.wait(1.0))
                # The final nonzero publication must still own the limiter lock.
                # Therefore cancel cannot publish zero and then be overtaken by
                # this already-computed command.
                self.assertTrue(limiter.lock.locked())
                cancel_thread = threading.Thread(
                    target=limiter.cancel_callback,
                    args=(goal_id("active", 0.0),),
                )
                cancel_thread.start()
            finally:
                publisher.release_nonzero_publish.set()

            timer_thread.join(2.0)
            if cancel_thread is not None:
                cancel_thread.join(2.0)

        self.assertFalse(timer_thread.is_alive())
        self.assertIsNotNone(cancel_thread)
        self.assertFalse(cancel_thread.is_alive())
        self.assertGreater(publisher.messages[0].linear.x, 0.0)
        self.assertEqual(publisher.messages[-1], Twist())
        self.assertTrue(limiter.stop_latched)


if __name__ == "__main__":
    unittest.main()
