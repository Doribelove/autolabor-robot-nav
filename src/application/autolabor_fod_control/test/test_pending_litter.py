#!/usr/bin/env python3

import math
import importlib.util
import json
import pathlib
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest import mock

from autolabor_fod_control.pending_litter import (
    map_target_to_wheel_odom,
    newest_goal_id,
    stand_off_pose,
    matches_cleaning_completion,
)

NODE_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts/pending_litter_cleaner_node.py"
SPEC = importlib.util.spec_from_file_location("pending_litter_node_under_test", NODE_PATH)
NODE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = NODE
SPEC.loader.exec_module(NODE)


class PendingLitterCompletionTest(unittest.TestCase):
    def setUp(self):
        self.proof = dict(sequence=3, object_id="42", target_x=1.0, target_y=2.0,
                          covered=True, minimum_distance_m=0.09, eligible_segments=2,
                          completion_policy="visual_servo", request_id="mission-a", stopped=True,
                          forward_m=0.51, path_m=0.52, acquired_object_id="99", blind_target_m=0.5)

    def test_only_new_exact_target_forward_coverage_is_accepted(self):
        self.assertTrue(matches_cleaning_completion(self.proof, 2, 42, 1.0, 2.0))
        for change in (
            {"sequence": 2}, {"object_id": "43"}, {"covered": False},
            {"eligible_segments": 0}, {"minimum_distance_m": 0.1001},
            {"minimum_distance_m": float("nan")}, {"target_x": 1.2},
            {"minimum_distance_m": None},
        ):
            with self.subTest(change=change):
                self.assertFalse(matches_cleaning_completion(
                    dict(self.proof, **change), 2, 42, 1.0, 2.0))

    def make_node(self):
        node = NODE.PendingLitterCleaner.__new__(NODE.PendingLitterCleaner)
        node.lock = threading.RLock()
        node.active = SimpleNamespace(object_id=42, target_x=1.0, target_y=2.0, request_id="mission-a")
        node.visual_completion_floor = 2
        node.visual_completion_seen = False
        node.mode_state = "GPS_ACTIVE"
        node.mode_state_receipt = 10.0
        node._fresh = lambda receipt: True
        node.visual_timeout_sec = 100.0
        node.goal_accept_timeout_sec = 2.0
        return node

    def test_manual_return_to_navigation_never_marks_cleaned(self):
        node = self.make_node()
        def wait(_mission, predicate, _timeout, _reason):
            with mock.patch.object(NODE.time, "monotonic", return_value=10.0):
                self.assertIsNone(predicate())
            with mock.patch.object(NODE.time, "monotonic", return_value=12.1):
                with self.assertRaisesRegex(RuntimeError, "without selected-target"):
                    predicate()
        node._wait = wait
        node._wait_visual_complete(node.active)

    def test_completed_evidence_survives_controller_disable_status(self):
        node = self.make_node()
        node._visual_status_cb(SimpleNamespace(data=json.dumps({
            "state": "DISABLED", "last_cleaning_completion": self.proof})))
        self.assertTrue(node.visual_completion_seen)
        node._wait = lambda mission, predicate, timeout, reason: self.assertTrue(predicate())
        node._wait_visual_complete(node.active)

    def test_old_or_wrong_target_evidence_does_not_latch(self):
        node = self.make_node()
        for change in ({"sequence": 2}, {"object_id": "43"}):
            node._visual_status_cb(SimpleNamespace(data=json.dumps({
                "last_cleaning_completion": dict(self.proof, **change)})))
            self.assertFalse(node.visual_completion_seen)

    def test_cancel_at_final_commit_cannot_publish_success(self):
        node = self.make_node()
        node.cancel_event = threading.Event()
        node.goal_id = "owned-goal"
        node.stand_off_m = 1.5
        node.min_visual_start_distance_m = 1.0
        node.max_visual_start_distance_m = 2.0
        node.max_visual_start_bearing_deg = 35.0
        node.visual_staged = node.visual_mode_requested = True
        node._map_pose = lambda: (0.0, 0.0, 0.0)
        for method in ("_submit_stand_off_goal", "_set_state", "_stage_visual_target",
                       "_enable_visual_mode", "_wait_visual_complete", "_cancel_exact_goal",
                       "_disable_visual_mode", "_clear_visual_target", "_publish_status", "_wait_stopped"):
            setattr(node, method, mock.Mock())
        node.completed_pub = mock.Mock()
        calls = []
        def still_active(mission):
            calls.append(mission)
            if len(calls) == 2:
                # Cancel lands after the outer predicate sampled true, just
                # before the final ownership lock is acquired.
                node.cancel_event.set()
            return True
        node._still_active = still_active
        node._run_mission(node.active)
        self.assertEqual(node.state, "ABORT")
        node.completed_pub.publish.assert_not_called()
        node._cancel_exact_goal.assert_called_once_with("owned-goal")


class PendingLitterGeometryTest(unittest.TestCase):
    def test_stand_off_goal_faces_target_without_crossing_it(self):
        x, y, yaw, distance = stand_off_pose(0.0, 0.0, 4.0, 3.0, 1.25)
        self.assertAlmostEqual(distance, 5.0)
        self.assertAlmostEqual(x, 3.0)
        self.assertAlmostEqual(y, 2.25)
        self.assertAlmostEqual(yaw, math.atan2(3.0, 4.0))
        self.assertAlmostEqual(math.hypot(4.0 - x, 3.0 - y), 1.25)

    def test_stand_off_refuses_a_target_already_inside_handoff(self):
        with self.assertRaises(ValueError):
            stand_off_pose(0.0, 0.0, 1.0, 0.0, 1.25)

    def test_map_target_is_frozen_in_independent_wheel_odom(self):
        x, y = map_target_to_wheel_odom(
            robot_map_x=10.0,
            robot_map_y=20.0,
            robot_map_yaw=math.pi / 2.0,
            target_map_x=9.5,
            target_map_y=22.0,
            robot_odom_x=1.0,
            robot_odom_y=2.0,
            robot_odom_yaw=math.pi,
        )
        self.assertAlmostEqual(x, -1.0)
        self.assertAlmostEqual(y, 1.5)

    def test_only_one_new_simple_goal_identity_is_accepted(self):
        previous = {"sweeper-simple-a"}
        self.assertEqual(
            newest_goal_id(previous, previous | {"sweeper-simple-b"}),
            "sweeper-simple-b",
        )
        self.assertIsNone(newest_goal_id(previous, previous))
        self.assertIsNone(
            newest_goal_id(
                previous,
                previous | {"sweeper-simple-b", "sweeper-simple-c"},
            )
        )


class PendingLitterIsolationContractTest(unittest.TestCase):
    def test_coordinator_never_publishes_vehicle_velocity(self):
        package = pathlib.Path(__file__).resolve().parents[1]
        source = (package / "scripts/pending_litter_cleaner_node.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("geometry_msgs.msg import Twist", source)
        self.assertNotIn("Publisher(\"/cmd_vel", source)
        self.assertIn('"/move_base_simple/goal"', source)
        self.assertIn('"/fod_navigation_mode/set_fod_enabled"', source)
        self.assertIn('"FOD visual motion authorization is disabled"', source)
        self.assertIn('"retained_reissue_allowed"', source)
        self.assertIn("self._cancel_exact_goal(goal_id)", source)

    def test_main_launch_starts_only_the_isolated_coordinator(self):
        workspace = pathlib.Path(__file__).resolve().parents[4]
        launch = (
            workspace
            / "src/platform/autolabor_dual_host/launch/j6m_fastlio_navigation.launch"
        ).read_text(encoding="utf-8")
        self.assertIn('type="pending_litter_cleaner_node.py"', launch)
        self.assertIn('name="pending_litter_cleaner"', launch)
        self.assertIn('value="/move_base_simple/goal"', launch)


if __name__ == "__main__":
    unittest.main()
