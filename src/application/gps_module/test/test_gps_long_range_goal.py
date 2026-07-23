#!/usr/bin/env python3

from collections import OrderedDict
import copy
import importlib.util
import math
from pathlib import Path
import sys
import threading
import unittest
from unittest import mock

import rospy
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from gps_module.long_range import (  # noqa: E402
    FINAL_GOAL_KIND,
    INTERMEDIATE_GOAL_KIND,
    RollingGoalRoute,
    gps_to_xy,
    is_contiguous_route_goal_transition,
    is_intermediate_route_goal_id,
    make_route_goal_id,
    parse_route_goal_id,
    route_goal_kind,
    validate_latitude_longitude,
    validate_route_distances,
)

MANAGER_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "gps_long_range_goal_manager.py"
)
MANAGER_SPEC = importlib.util.spec_from_file_location(
    "gps_long_range_goal_manager",
    str(MANAGER_PATH),
)
MANAGER = importlib.util.module_from_spec(MANAGER_SPEC)
MANAGER_SPEC.loader.exec_module(MANAGER)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(copy.deepcopy(msg))


class RollingGoalGeometryTest(unittest.TestCase):
    def test_long_route_selects_a_fifteen_metre_intermediate_goal(self):
        route = RollingGoalRoute(200.0, 0.0)

        segment = route.next_segment(0.0, 0.0)

        self.assertFalse(segment.is_final)
        self.assertAlmostEqual(segment.x, 15.0)
        self.assertAlmostEqual(segment.y, 0.0)
        self.assertAlmostEqual(segment.final_distance, 200.0)

    def test_diagonal_subgoal_keeps_the_requested_euclidean_horizon(self):
        route = RollingGoalRoute(200.0, 200.0)

        segment = route.next_segment(10.0, -5.0)

        self.assertAlmostEqual(
            math.hypot(segment.x - 10.0, segment.y + 5.0),
            15.0,
        )

    def test_intermediate_goal_advances_at_five_metres_remaining(self):
        route = RollingGoalRoute(200.0, 0.0)
        route.next_segment(0.0, 0.0)

        self.assertFalse(route.should_advance(9.9, 0.0))
        self.assertTrue(route.should_advance(10.0, 0.0))

        next_segment = route.next_segment(10.0, 0.0)
        self.assertAlmostEqual(next_segment.x, 25.0)
        self.assertEqual(next_segment.index, 2)

    def test_detour_progress_advances_without_entering_five_metre_circle(self):
        route = RollingGoalRoute(200.0, 0.0)
        route.next_segment(0.0, 0.0)

        self.assertGreater(route.distance_to_segment(11.0, 6.0), 5.0)
        self.assertTrue(route.should_advance(11.0, 6.0))

    def test_passing_subgoal_laterally_never_requires_turning_back(self):
        route = RollingGoalRoute(200.0, 0.0)
        route.next_segment(0.0, 0.0)

        self.assertGreater(route.distance_to_segment(16.0, 6.0), 5.0)
        self.assertTrue(route.should_advance(16.0, 6.0))

    def test_final_goal_is_sent_once_inside_the_horizon(self):
        route = RollingGoalRoute(200.0, 0.0)
        route.next_segment(0.0, 0.0)

        final_segment = route.next_segment(185.0, 0.0)
        repeated = route.next_segment(190.0, 0.0)

        self.assertTrue(final_segment.is_final)
        self.assertEqual(final_segment.x, 200.0)
        self.assertIs(repeated, final_segment)
        self.assertEqual(route.segment_index, 2)

    def test_goal_exactly_at_horizon_is_final(self):
        route = RollingGoalRoute(15.0, 0.0)

        segment = route.next_segment(0.0, 0.0)

        self.assertTrue(segment.is_final)
        self.assertEqual(segment.x, 15.0)

    def test_horizon_comparison_tolerates_micrometre_roundoff(self):
        route = RollingGoalRoute(15.0000005, 0.0)

        self.assertTrue(route.final_is_within_horizon(0.0, 0.0))
        self.assertTrue(route.next_segment(0.0, 0.0).is_final)

    def test_zero_distance_goal_is_finite_and_final(self):
        route = RollingGoalRoute(3.0, -2.0)

        segment = route.next_segment(3.0, -2.0)

        self.assertTrue(segment.is_final)
        self.assertEqual((segment.x, segment.y), (3.0, -2.0))

    def test_two_hundred_metre_route_reaches_one_final_segment(self):
        route = RollingGoalRoute(200.0, 0.0)
        current_x = 0.0
        previous_subgoal_x = -math.inf

        for _ in range(30):
            segment = route.next_segment(current_x, 0.0)
            self.assertGreater(segment.x, previous_subgoal_x)
            self.assertLessEqual(segment.x - current_x, 15.0 + 1e-9)
            previous_subgoal_x = segment.x
            if segment.is_final:
                break
            current_x = segment.x - route.advance_distance
            self.assertTrue(route.should_advance(current_x, 0.0))
        else:
            self.fail("rolling route did not produce a final segment")

        self.assertEqual(segment.x, 200.0)
        self.assertTrue(segment.is_final)

    def test_invalid_route_distances_are_rejected(self):
        with self.assertRaises(ValueError):
            validate_route_distances(0.0, 1.0)
        with self.assertRaises(ValueError):
            validate_route_distances(15.0, 15.0)
        with self.assertRaises(ValueError):
            RollingGoalRoute(math.inf, 0.0)


class ManagedGoalIdentityTest(unittest.TestCase):
    def test_goal_id_round_trip_preserves_segment_kind(self):
        intermediate = make_route_goal_id("123456-2", 7, False)
        final = make_route_goal_id("123456-2", 8, True)

        self.assertEqual(
            parse_route_goal_id(intermediate),
            ("123456-2", 7, INTERMEDIATE_GOAL_KIND),
        )
        self.assertEqual(
            route_goal_kind(intermediate),
            INTERMEDIATE_GOAL_KIND,
        )
        self.assertEqual(route_goal_kind(final), FINAL_GOAL_KIND)
        self.assertTrue(is_intermediate_route_goal_id(intermediate))
        self.assertFalse(is_intermediate_route_goal_id(final))

    def test_unmanaged_or_malformed_ids_are_never_intermediate(self):
        malformed = (
            "",
            "rviz-goal",
            "gps_long_range/not-a-token/1/intermediate",
            "gps_long_range/1-1/0/intermediate",
            "gps_long_range/1-1/1/unknown",
            "gps_long_range/1-1/1/intermediate/extra",
        )
        for identifier in malformed:
            with self.subTest(identifier=identifier):
                self.assertIsNone(route_goal_kind(identifier))
                self.assertFalse(is_intermediate_route_goal_id(identifier))

    def test_invalid_route_token_or_index_is_rejected(self):
        with self.assertRaises(ValueError):
            make_route_goal_id("unsafe", 1, False)
        with self.assertRaises(ValueError):
            make_route_goal_id("1-1", 0, False)

    def test_only_sequential_segments_of_one_route_are_contiguous(self):
        first = make_route_goal_id("100-1", 1, False)
        second = make_route_goal_id("100-1", 2, False)
        final = make_route_goal_id("100-1", 3, True)

        self.assertTrue(is_contiguous_route_goal_transition(first, second))
        self.assertTrue(is_contiguous_route_goal_transition(second, final))
        self.assertFalse(is_contiguous_route_goal_transition(final, second))
        self.assertFalse(
            is_contiguous_route_goal_transition(
                first,
                make_route_goal_id("100-2", 2, False),
            )
        )
        self.assertFalse(
            is_contiguous_route_goal_transition(
                first,
                make_route_goal_id("100-1", 3, False),
            )
        )


class ActionCancelSemanticsTest(unittest.TestCase):
    @staticmethod
    def identifier(identifier, stamp):
        message = GoalID()
        message.id = identifier
        message.stamp = rospy.Time.from_sec(stamp)
        return message

    def test_specific_cancel_only_matches_current_identifier(self):
        active_stamp = rospy.Time.from_sec(10.0)
        self.assertTrue(
            MANAGER.cancel_matches_goal(
                self.identifier("current", 0.0),
                "current",
                active_stamp,
            )
        )
        self.assertFalse(
            MANAGER.cancel_matches_goal(
                self.identifier("old", 0.0),
                "current",
                active_stamp,
            )
        )

    def test_empty_cancel_uses_actionlib_all_or_timestamp_semantics(self):
        active_stamp = rospy.Time.from_sec(10.0)
        self.assertTrue(
            MANAGER.cancel_matches_goal(
                self.identifier("", 0.0),
                "current",
                active_stamp,
            )
        )
        self.assertTrue(
            MANAGER.cancel_matches_goal(
                self.identifier("", 11.0),
                "current",
                active_stamp,
            )
        )
        self.assertFalse(
            MANAGER.cancel_matches_goal(
                self.identifier("", 9.0),
                "current",
                active_stamp,
            )
        )


class GpsGoalConversionTest(unittest.TestCase):
    def test_nearby_gps_target_converts_to_finite_local_metres(self):
        x, y = gps_to_xy(
            31.0001,
            121.0001,
            31.0,
            121.0,
        )

        self.assertTrue(math.isfinite(x))
        self.assertTrue(math.isfinite(y))
        self.assertGreater(x, 0.0)
        self.assertGreater(y, 0.0)

    def test_invalid_wgs84_coordinates_are_rejected(self):
        with self.assertRaises(ValueError):
            validate_latitude_longitude(91.0, 120.0)
        with self.assertRaises(ValueError):
            validate_latitude_longitude(31.0, math.nan)


class LongRangeManagerStateTest(unittest.TestCase):
    @staticmethod
    def manager_with_route(final_x=200.0):
        manager = MANAGER.GpsLongRangeGoalManager.__new__(
            MANAGER.GpsLongRangeGoalManager
        )
        manager.frame_id = "camera_init"
        manager.lookahead_distance = 15.0
        manager.advance_distance = 5.0
        manager.odom_timeout = 1.0
        manager.move_base_status_timeout = 2.0
        manager.lock = threading.RLock()
        manager.latest_odom = Odometry()
        manager.latest_odom.header.frame_id = "camera_init"
        manager.latest_odom_receipt = rospy.Time.from_sec(10.0)
        manager.latest_move_base_status_receipt = rospy.Time.from_sec(10.0)
        manager.route = RollingGoalRoute(
            final_x,
            0.0,
            lookahead_distance=15.0,
            advance_distance=5.0,
        )
        manager.route_token = "100-1"
        manager.route_active = True
        manager.paused = False
        manager.route_state = "ACTIVE_INTERMEDIATE"
        manager.route_reason = "test route"
        manager.final_latitude = 31.0
        manager.final_longitude = 121.0
        manager.current_goal_id = None
        manager.current_goal_stamp = None
        manager.current_goal_is_final = False
        manager.advance_requested = False
        manager.owned_goal_ids = OrderedDict()
        manager.action_goal_pub = FakePublisher()
        manager.cancel_pub = FakePublisher()
        manager.final_goal_pub = FakePublisher()
        manager.subgoal_pub = FakePublisher()
        manager.route_status_pub = FakePublisher()
        manager.route_active_pub = FakePublisher()
        return manager

    def test_action_goal_identity_marks_first_segment_intermediate(self):
        manager = self.manager_with_route()
        now = rospy.Time.from_sec(10.0)

        with manager.lock:
            manager._send_next_segment_locked(0.0, 0.0, now)

        action_goal = manager.action_goal_pub.messages[-1]
        self.assertEqual(
            route_goal_kind(action_goal.goal_id.id),
            INTERMEDIATE_GOAL_KIND,
        )
        self.assertAlmostEqual(
            action_goal.goal.target_pose.pose.position.x,
            15.0,
        )
        self.assertFalse(manager.current_goal_is_final)

    def test_timer_replaces_subgoal_at_five_metres_remaining(self):
        manager = self.manager_with_route()
        now = rospy.Time.from_sec(10.0)
        with manager.lock:
            manager._send_next_segment_locked(0.0, 0.0, now)
        first_id = manager.current_goal_id
        manager.latest_odom.pose.pose.position.x = 10.0

        with mock.patch.object(MANAGER.rospy.Time, "now", return_value=now):
            manager.timer_callback(None)

        self.assertNotEqual(manager.current_goal_id, first_id)
        self.assertAlmostEqual(
            manager.action_goal_pub.messages[-1].goal.target_pose.pose.position.x,
            25.0,
        )

    def test_timer_switches_to_one_final_goal_inside_horizon(self):
        manager = self.manager_with_route()
        now = rospy.Time.from_sec(10.0)
        with manager.lock:
            manager._send_next_segment_locked(0.0, 0.0, now)
        manager.latest_odom.pose.pose.position.x = 185.0

        with mock.patch.object(MANAGER.rospy.Time, "now", return_value=now):
            manager.timer_callback(None)
            action_count = len(manager.action_goal_pub.messages)
            status_count = len(manager.route_status_pub.messages)
            manager.timer_callback(None)

        self.assertTrue(manager.current_goal_is_final)
        self.assertEqual(
            route_goal_kind(manager.current_goal_id),
            FINAL_GOAL_KIND,
        )
        self.assertEqual(len(manager.action_goal_pub.messages), action_count)
        self.assertGreater(len(manager.route_status_pub.messages), status_count)

    def test_stale_odometry_aborts_and_cancels_current_segment(self):
        manager = self.manager_with_route()
        now = rospy.Time.from_sec(10.0)
        with manager.lock:
            manager._send_next_segment_locked(0.0, 0.0, now)
        manager.latest_odom_receipt = rospy.Time.from_sec(8.0)

        with mock.patch.object(MANAGER.rospy.Time, "now", return_value=now):
            manager.timer_callback(None)

        self.assertFalse(manager.route_active)
        self.assertEqual(manager.route_state, "ABORTED")
        self.assertEqual(
            manager.cancel_pub.messages[-1].id,
            manager.current_goal_id,
        )

    def test_foreign_action_goal_supersedes_route_without_canceling_it(self):
        manager = self.manager_with_route()
        now = rospy.Time.from_sec(10.0)
        with manager.lock:
            manager._send_next_segment_locked(0.0, 0.0, now)
        foreign = MoveBaseActionGoal()
        foreign.goal_id.id = "rviz-goal"

        manager.action_goal_callback(foreign)

        self.assertFalse(manager.route_active)
        self.assertEqual(manager.route_state, "SUPERSEDED")
        self.assertEqual(manager.cancel_pub.messages, [])

    @staticmethod
    def status_message(identifier, state):
        item = GoalStatus()
        item.goal_id.id = identifier
        item.status = state
        message = GoalStatusArray()
        message.status_list = [item]
        return message

    def test_intermediate_success_requests_next_segment(self):
        manager = self.manager_with_route()
        now = rospy.Time.from_sec(10.0)
        with manager.lock:
            manager._send_next_segment_locked(0.0, 0.0, now)

        with mock.patch.object(MANAGER.rospy.Time, "now", return_value=now):
            manager.move_base_status_callback(
                self.status_message(
                    manager.current_goal_id,
                    GoalStatus.SUCCEEDED,
                )
            )

        self.assertTrue(manager.route_active)
        self.assertTrue(manager.advance_requested)
        self.assertEqual(manager.route_state, "ADVANCING")

    def test_final_success_completes_route_without_another_goal(self):
        manager = self.manager_with_route(final_x=10.0)
        now = rospy.Time.from_sec(10.0)
        with manager.lock:
            manager._send_next_segment_locked(0.0, 0.0, now)
        action_count = len(manager.action_goal_pub.messages)

        with mock.patch.object(MANAGER.rospy.Time, "now", return_value=now):
            manager.move_base_status_callback(
                self.status_message(
                    manager.current_goal_id,
                    GoalStatus.SUCCEEDED,
                )
            )

        self.assertFalse(manager.route_active)
        self.assertEqual(manager.route_state, "COMPLETE")
        self.assertEqual(len(manager.action_goal_pub.messages), action_count)

    def test_matching_action_failure_aborts_route(self):
        manager = self.manager_with_route()
        now = rospy.Time.from_sec(10.0)
        with manager.lock:
            manager._send_next_segment_locked(0.0, 0.0, now)

        with mock.patch.object(MANAGER.rospy.Time, "now", return_value=now):
            manager.move_base_status_callback(
                self.status_message(
                    manager.current_goal_id,
                    GoalStatus.ABORTED,
                )
            )

        self.assertFalse(manager.route_active)
        self.assertEqual(manager.route_state, "ABORTED")

    def test_pause_cancels_segment_but_retains_final_route(self):
        manager = self.manager_with_route()
        now = rospy.Time.from_sec(10.0)
        with manager.lock:
            manager._send_next_segment_locked(0.0, 0.0, now)
        retained_route = manager.route
        current_goal_id = manager.current_goal_id

        response = manager.set_paused_callback(mock.Mock(data=True))

        self.assertTrue(response.success)
        self.assertTrue(manager.paused)
        self.assertTrue(manager.route_active)
        self.assertIs(manager.route, retained_route)
        self.assertEqual(manager.route_state, "PAUSED")
        self.assertEqual(manager.cancel_pub.messages[-1].id, current_goal_id)

    def test_pause_ignores_cancel_and_terminal_status_echoes(self):
        manager = self.manager_with_route()
        now = rospy.Time.from_sec(10.0)
        with manager.lock:
            manager._send_next_segment_locked(0.0, 0.0, now)
        current_goal_id = manager.current_goal_id
        manager.set_paused_callback(mock.Mock(data=True))

        cancel = GoalID()
        cancel.id = current_goal_id
        manager.cancel_callback(cancel)
        with mock.patch.object(MANAGER.rospy.Time, "now", return_value=now):
            manager.move_base_status_callback(
                self.status_message(current_goal_id, GoalStatus.PREEMPTED)
            )

        self.assertTrue(manager.paused)
        self.assertTrue(manager.route_active)
        self.assertEqual(manager.route_state, "PAUSED")

    def test_resume_replans_next_segment_from_post_recovery_position(self):
        manager = self.manager_with_route()
        now = rospy.Time.from_sec(10.0)
        with manager.lock:
            manager._send_next_segment_locked(0.0, 0.0, now)
        manager.set_paused_callback(mock.Mock(data=True))
        manager.latest_odom.pose.pose.position.x = 4.0
        action_count = len(manager.action_goal_pub.messages)

        with mock.patch.object(MANAGER.rospy.Time, "now", return_value=now):
            response = manager.set_paused_callback(mock.Mock(data=False))

        self.assertTrue(response.success)
        self.assertFalse(manager.paused)
        self.assertTrue(manager.route_active)
        self.assertEqual(len(manager.action_goal_pub.messages), action_count + 1)
        resumed_x = (
            manager.action_goal_pub.messages[-1]
            .goal.target_pose.pose.position.x
        )
        self.assertAlmostEqual(resumed_x, 19.0)


if __name__ == "__main__":
    unittest.main()
