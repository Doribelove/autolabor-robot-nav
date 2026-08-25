#!/usr/bin/env python3

import importlib.util
import ast
import inspect
import math
from pathlib import Path
from types import SimpleNamespace
import threading
import textwrap
import unittest
from unittest import mock

from geometry_msgs.msg import Point32


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "coverage_manager_under_test",
    str(PACKAGE_ROOT / "scripts" / "coverage_manager.py"),
)
COVERAGE_MANAGER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(COVERAGE_MANAGER)


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _MoveBase:
    def __init__(self):
        self.cancel_count = 0
        self.cancel_all_count = 0

    def cancel_goal(self):
        self.cancel_count += 1

    def cancel_all_goals(self):
        self.cancel_all_count += 1


class _SuccessfulMoveBase(_MoveBase):
    def __init__(self):
        super().__init__()
        self.goals = []

    def send_goal(self, goal):
        self.goals.append(goal)

    @staticmethod
    def wait_for_result(_timeout):
        return True

    @staticmethod
    def get_state():
        return COVERAGE_MANAGER.GoalStatus.SUCCEEDED


class CoverageManagerStateMachineTest(unittest.TestCase):
    @staticmethod
    def _seed_lifecycle_state(
        manager, current_segment=3, total_segments=8, blocked_segments=None
    ):
        manager.plan = SimpleNamespace()
        manager.plan_id = "test-plan"
        manager.plan_map_digest = "test-map"
        manager.map_digest = "test-map"
        manager.region = COVERAGE_MANAGER.PolygonStamped()
        manager.region.header.frame_id = "map"
        manager.region.polygon.points.append(Point32(x=1.0, y=2.0, z=0.0))
        manager.current_segment = current_segment
        manager.total_segments = total_segments
        manager.blocked_segments = list(blocked_segments or [])
        manager.traversed_distance = 4.5
        manager.covered_cells = {(1, 2), (2, 2)}
        manager.last_tracked_point = COVERAGE_MANAGER.Point(1.0, 2.0)
        manager.executed_path = COVERAGE_MANAGER.Path()
        manager.executed_path.header.frame_id = "map"
        manager.executed_path.poses.append(COVERAGE_MANAGER.PoseStamped())
        manager.kinematics_verified = True
        manager.kinematics_detail = "verified"
        manager.worker = object()
        manager.plan_pending = False
        manager.plan_token = ""
        manager.start_pending = False
        manager.start_token = ""
        manager.path_pub = _Publisher()
        manager.executed_path_pub = _Publisher()
        manager.region_pub = _Publisher()
        manager.marker_pub = _Publisher()
        manager.active_pub = _Publisher()

    def _assert_plan_and_visualizations_cleared(self, manager):
        self.assertIsNone(manager.plan)
        self.assertEqual("", manager.plan_id)
        self.assertEqual("", manager.plan_map_digest)
        self.assertEqual([], list(manager.region.polygon.points))
        self.assertEqual(set(), manager.covered_cells)
        self.assertIsNone(manager.last_tracked_point)
        self.assertEqual([], list(manager.executed_path.poses))
        self.assertFalse(manager.kinematics_verified)
        self.assertIsNone(manager.worker)

        self.assertTrue(manager.path_pub.messages)
        self.assertEqual([], list(manager.path_pub.messages[-1].poses))
        self.assertTrue(manager.executed_path_pub.messages)
        self.assertEqual([], list(manager.executed_path_pub.messages[-1].poses))
        self.assertTrue(manager.region_pub.messages)
        self.assertEqual(
            [], list(manager.region_pub.messages[-1].polygon.points)
        )
        self.assertTrue(manager.marker_pub.messages)
        marker_array = manager.marker_pub.messages[-1]
        self.assertEqual(1, len(marker_array.markers))
        self.assertEqual(
            COVERAGE_MANAGER.Marker.DELETEALL,
            marker_array.markers[0].action,
        )

    def _manager(self, segments, results, restore=True):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.segment_retry_count = 0
        manager.final_retry_count = 1
        manager.obstacle_wait_sec = 0.0
        manager.cancel_requested = False
        self._seed_lifecycle_state(
            manager, current_segment=0, total_segments=0, blocked_segments=[]
        )
        manager.state = "GOING_TO_START"
        manager.detail = "test task"
        manager.active = True
        manager.manual_pause = False
        manager.manual_pause_reason = ""
        manager.external_pause = False
        manager.avoidance_loss_paused = False
        manager.chassis_fault_paused = False
        manager.move_base = _MoveBase()
        manager.enforced_path_pub = _Publisher()
        manager._segments = lambda _route, _current: segments
        manager._wait_while_paused = lambda: True
        manager._restore_teb = lambda: restore
        manager._set_enforced_path = lambda _path, coverage_active=True: True
        manager._publish_status = lambda: None
        executed = []
        pending_results = list(results)

        def execute(segment, _index):
            executed.append(segment["type"])
            return pending_results.pop(0)

        manager._execute_segment = execute
        return manager, executed

    @staticmethod
    def _swath_segments():
        return [
            {"type": "transit", "swath_index": 0},
            {"type": "sweep", "swath_index": 0},
        ]

    def _run(self, manager):
        with mock.patch.object(
            COVERAGE_MANAGER.rospy, "is_shutdown", return_value=False
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            manager._run_task([], None)

    def test_every_swath_has_an_explicit_transit_dependency(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.path_sample_spacing = 0.1
        manager._path_for_points = lambda points, _yaw: list(points)
        swath = SimpleNamespace(
            start=COVERAGE_MANAGER.Point(0.0, 0.0),
            end=COVERAGE_MANAGER.Point(1.0, 0.0),
            length=1.0,
        )
        segments = manager._segments(
            [swath], COVERAGE_MANAGER.Point(0.0, 0.0)
        )
        self.assertEqual(["transit", "sweep"], [item["type"] for item in segments])
        self.assertEqual([0, 0], [item["swath_index"] for item in segments])

    def test_transit_arms_navfn_not_the_enforced_coverage_path(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.plan_id = "test-plan"
        manager.allow_reverse_transit = True
        manager.reverse_transit_speed = 0.3
        manager.entry_position_tolerance = 0.2
        manager.entry_yaw_tolerance = 0.2
        manager.goal_timeout_base_sec = 20.0
        manager.goal_timeout_per_meter_sec = 20.0
        manager.cancel_requested = False
        manager.manual_pause = False
        manager.external_pause = False
        manager.move_base = _SuccessfulMoveBase()
        manager.enforced_path_pub = _Publisher()
        manager._wait_while_paused = lambda: True
        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(-1.0, 0.0), 0.0
        )
        configured_reverse_speeds = []
        manager._set_teb = lambda speed: configured_reverse_speeds.append(speed) or True
        handoffs = []
        manager._set_enforced_path = (
            lambda path, coverage_active=True:
            handoffs.append((path, coverage_active)) or True
        )
        manager._pause_for_avoidance_loss = lambda: None
        manager._localization_is_fresh = lambda: True
        transit = {
            "type": "transit",
            "swath_index": 0,
            "start": COVERAGE_MANAGER.Point(-1.0, 0.0),
            "end": COVERAGE_MANAGER.Point(1.0, 0.0),
            "yaw": 0.0,
            "length": 2.0,
            "path": None,
        }
        with mock.patch.object(
            COVERAGE_MANAGER.rospy, "is_shutdown", return_value=False
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            result = manager._execute_segment(transit, 0)
        self.assertEqual("succeeded", result)
        self.assertEqual([0.3], configured_reverse_speeds)
        self.assertEqual(1, len(handoffs))
        enforced, coverage_active = handoffs[0]
        self.assertTrue(coverage_active)
        self.assertFalse(enforced.active)
        self.assertEqual([], list(enforced.path.poses))
        self.assertEqual(1, len(manager.move_base.goals))

    def test_sweep_arms_only_its_exact_coverage_path(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.plan_id = "test-plan"
        manager.allow_reverse_transit = True
        manager.reverse_transit_speed = 0.3
        manager.goal_timeout_base_sec = 20.0
        manager.goal_timeout_per_meter_sec = 20.0
        manager.cancel_requested = False
        manager.manual_pause = False
        manager.external_pause = False
        manager.last_tracked_point = COVERAGE_MANAGER.Point(0.0, 0.0)
        manager.move_base = _SuccessfulMoveBase()
        manager.enforced_path_pub = _Publisher()
        manager._wait_while_paused = lambda: True
        configured_reverse_speeds = []
        manager._set_teb = lambda speed: configured_reverse_speeds.append(speed) or True
        handoffs = []
        manager._set_enforced_path = (
            lambda path, coverage_active=True:
            handoffs.append((path, coverage_active)) or True
        )
        manager._pause_for_avoidance_loss = lambda: None
        manager._localization_is_fresh = lambda: True
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        stamp = COVERAGE_MANAGER.rospy.Time(100.0)
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(0.0, 0.0), 0.0, stamp),
            manager._pose(COVERAGE_MANAGER.Point(1.0, 0.0), 0.0, stamp),
        ]
        sweep = {
            "type": "sweep",
            "swath_index": 0,
            "start": COVERAGE_MANAGER.Point(0.0, 0.0),
            "end": COVERAGE_MANAGER.Point(1.0, 0.0),
            "yaw": 0.0,
            "length": 1.0,
            "path": path,
        }
        with mock.patch.object(
            COVERAGE_MANAGER.rospy, "is_shutdown", return_value=False
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=stamp,
        ):
            result = manager._execute_segment(sweep, 1)
        self.assertEqual("succeeded", result)
        self.assertEqual([0.0], configured_reverse_speeds)
        enforced, coverage_active = handoffs[0]
        self.assertTrue(coverage_active)
        self.assertTrue(enforced.active)
        self.assertEqual(2, len(enforced.path.poses))

    def test_start_replan_and_external_checks_do_not_hold_callback_lock(self):
        source = textwrap.dedent(inspect.getsource(
            COVERAGE_MANAGER.CoverageManager._start_service
        ))
        tree = ast.parse(source)
        calls_while_locked = []
        for node in ast.walk(tree):
            if not isinstance(node, (ast.With, ast.AsyncWith)):
                continue
            uses_manager_lock = any(
                isinstance(item.context_expr, ast.Attribute)
                and isinstance(item.context_expr.value, ast.Name)
                and item.context_expr.value.id == "self"
                and item.context_expr.attr == "lock"
                for item in node.items
            )
            if not uses_manager_lock:
                continue
            for child in ast.walk(node):
                if isinstance(child, ast.Call) and isinstance(child.func, ast.Attribute):
                    calls_while_locked.append(child.func.attr)
        for blocking_call in (
            "_planner",
            "plan",
            "_current_pose",
            "_start_external_prechecks",
            "wait_for_server",
            "wait_for_service",
        ):
            self.assertNotIn(blocking_call, calls_while_locked)

    def test_ready_cancel_discards_plan_progress_and_latched_visualizations(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(
            manager,
            current_segment=5,
            total_segments=12,
            blocked_segments=[2, 4],
        )
        manager.active = False
        manager.state = "READY"
        manager.detail = "coverage path is ready"
        manager.cancel_requested = False
        manager.move_base = _MoveBase()
        manager._publish_status = lambda: None

        with mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            response = manager._cancel_service(SimpleNamespace())

        self.assertTrue(response.success)
        self.assertEqual("IDLE", manager.state)
        self.assertEqual(0, manager.current_segment)
        self.assertEqual(0, manager.total_segments)
        self.assertEqual([], manager.blocked_segments)
        self.assertEqual(0.0, manager.traversed_distance)
        self._assert_plan_and_visualizations_cleared(manager)
        self.assertTrue(manager.active_pub.messages)
        self.assertFalse(manager.active_pub.messages[-1].data)
        self.assertEqual(0, manager.move_base.cancel_count)
        self.assertEqual(0, manager.move_base.cancel_all_count)

    def test_canceled_planner_generation_cannot_commit_into_the_next_request(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        manager.plan = None
        manager.plan_id = ""
        manager.plan_map_digest = ""
        manager.active = False
        manager.state = "IDLE"
        manager.detail = "ready"
        manager.cancel_requested = False
        manager.grid = object()
        manager.map_digest = "static-map"
        manager.minimum_turning_radius = 1.35
        manager.move_base = _MoveBase()
        manager._publish_status = lambda: None
        manager._localization_is_fresh = lambda: False
        manager._points_from_region = lambda _region: [
            COVERAGE_MANAGER.Point(0.0, 0.0),
            COVERAGE_MANAGER.Point(4.0, 0.0),
            COVERAGE_MANAGER.Point(4.0, 2.0),
        ]
        manager._planned_path = lambda _route, _current: COVERAGE_MANAGER.Path()
        manager._publish_markers = lambda _route, _region: None

        first_entered = threading.Event()
        second_entered = threading.Event()
        release_first = threading.Event()
        release_second = threading.Event()
        planner_call_lock = threading.Lock()
        planner_calls = [0]

        def make_plan(call_index):
            return SimpleNamespace(
                generation=call_index,
                swaths=[SimpleNamespace(
                    start=COVERAGE_MANAGER.Point(0.0, float(call_index)),
                    end=COVERAGE_MANAGER.Point(3.0, float(call_index)),
                    scan_v=float(call_index),
                    length=3.0,
                )],
                spacing=0.85,
                requested_area=6.0,
                reachable_area=6.0,
                unreachable_area=0.0,
                angle=0.0,
            )

        def plan(_points, _width, _overlap, reachable_seed=None):
            del reachable_seed
            with planner_call_lock:
                planner_calls[0] += 1
                call_index = planner_calls[0]
            entered = first_entered if call_index == 1 else second_entered
            release = release_first if call_index == 1 else release_second
            entered.set()
            self.assertTrue(release.wait(2.0))
            return make_plan(call_index)

        planner = SimpleNamespace(plan=plan)
        manager._planner = lambda _grid: planner
        request = SimpleNamespace(
            region=COVERAGE_MANAGER.PolygonStamped(),
            operation_width_m=1.0,
            overlap_ratio=0.15,
            allow_reverse_transit=True,
        )
        request.region.header.frame_id = "map"
        responses = {}
        first_thread = threading.Thread(
            target=lambda: responses.setdefault("first", manager._plan_service(request))
        )
        second_thread = threading.Thread(
            target=lambda: responses.setdefault("second", manager._plan_service(request))
        )

        with mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            first_thread.start()
            self.assertTrue(first_entered.wait(1.0))
            first_token = manager.plan_token
            self.assertTrue(first_token)
            self.assertTrue(manager._cancel_service(SimpleNamespace()).success)

            second_thread.start()
            self.assertTrue(second_entered.wait(1.0))
            second_token = manager.plan_token
            self.assertTrue(second_token)
            self.assertNotEqual(first_token, second_token)

            release_first.set()
            first_thread.join(2.0)
            self.assertFalse(first_thread.is_alive())
            self.assertFalse(responses["first"].success)
            self.assertIn("canceled or superseded", responses["first"].message)
            self.assertTrue(manager.plan_pending)
            self.assertEqual(second_token, manager.plan_token)

            release_second.set()
            second_thread.join(2.0)

        self.assertFalse(second_thread.is_alive())
        self.assertTrue(responses["second"].success)
        self.assertFalse(manager.plan_pending)
        self.assertEqual("", manager.plan_token)
        self.assertEqual(2, manager.plan.generation)
        self.assertEqual("READY", manager.state)

    def test_cancel_during_preparing_invalidates_token_and_never_starts_worker(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        manager.active = False
        manager.state = "READY"
        manager.detail = "coverage path is ready"
        manager.cancel_requested = False
        manager.max_speed_limit = 1.60
        manager.grid = object()
        manager.operation_width = 1.0
        manager.overlap_ratio = 0.15
        manager.minimum_turning_radius = 1.35
        manager.path_sample_spacing = 0.10
        manager.move_base = _SuccessfulMoveBase()
        manager._publish_status = lambda: None
        manager._start_prechecks_locked = (
            lambda _speed, require_kinematics: True
        )
        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(0.0, 0.0), 0.0
        )
        replanned = SimpleNamespace(swaths=[], spacing=0.85)
        manager._planner = lambda _grid: SimpleNamespace(
            plan=lambda _points, _width, _overlap, reachable_seed=None: replanned
        )

        external_check_entered = threading.Event()
        release_external_check = threading.Event()

        def external_prechecks(_token):
            external_check_entered.set()
            return release_external_check.wait(2.0)

        manager._start_external_prechecks = external_prechecks
        responses = []
        request = SimpleNamespace(plan_id="test-plan", max_speed_mps=0.30)
        start_thread = threading.Thread(
            target=lambda: responses.append(manager._start_service(request))
        )

        with mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            start_thread.start()
            self.assertTrue(external_check_entered.wait(1.0))
            preparing_token = manager.start_token
            self.assertTrue(preparing_token)
            self.assertTrue(manager.start_pending)
            cancel_response = manager._cancel_service(SimpleNamespace())
            release_external_check.set()
            start_thread.join(2.0)

        self.assertFalse(start_thread.is_alive())
        self.assertTrue(cancel_response.success)
        self.assertEqual("", manager.start_token)
        self.assertNotEqual(preparing_token, manager.start_token)
        self.assertFalse(manager.start_pending)
        self.assertFalse(manager.active)
        self.assertEqual(1, len(responses))
        self.assertFalse(responses[0].accepted)
        self.assertIn("canceled or superseded", responses[0].message)
        self.assertEqual([], manager.move_base.goals)
        self.assertEqual(0, manager.move_base.cancel_count)
        self.assertEqual(0, manager.move_base.cancel_all_count)
        self._assert_plan_and_visualizations_cleared(manager)

    def test_every_task_terminal_state_discards_plan_and_latched_visualizations(self):
        one_transit = [{"type": "transit", "swath_index": 0}]
        cases = (
            ("COMPLETED", [], []),
            (
                "COMPLETED_PARTIAL",
                self._swath_segments(),
                ["blocked", "blocked"],
            ),
            ("CANCELED", one_transit, ["canceled"]),
            ("FAILED", one_transit, ["failed"]),
        )
        for expected_state, segments, results in cases:
            with self.subTest(expected_state=expected_state):
                manager, _executed = self._manager(segments, results)
                self._run(manager)
                self.assertEqual(expected_state, manager.state)
                self.assertFalse(manager.active)
                self._assert_plan_and_visualizations_cleared(manager)
                self.assertTrue(manager.active_pub.messages)
                self.assertFalse(manager.active_pub.messages[-1].data)

    def test_tracking_timer_does_not_republish_after_plan_generation_changes(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.active = True
        manager.state = "SWEEPING"
        manager.manual_pause = False
        manager.external_pause = False
        manager.plan_id = "old-plan"
        manager.last_tracked_point = COVERAGE_MANAGER.Point(0.0, 0.0)
        manager.executed_path = COVERAGE_MANAGER.Path()
        manager.executed_path.header.frame_id = "map"
        manager.executed_path_pub = _Publisher()
        manager._localization_is_fresh = lambda: True

        def pose_after_generation_change():
            # Model terminal cleanup followed immediately by a new active
            # generation while the old timer callback is outside the lock for
            # its TF query.
            with manager.lock:
                manager.plan_id = "new-plan"
                manager.executed_path = COVERAGE_MANAGER.Path()
                manager.executed_path.header.frame_id = "map"
                manager.last_tracked_point = None
            return COVERAGE_MANAGER.Point(0.1, 0.0), 0.0

        manager._current_pose = pose_after_generation_change
        manager._tracking_timer(None)

        self.assertEqual("new-plan", manager.plan_id)
        self.assertEqual([], list(manager.executed_path.poses))
        self.assertEqual([], manager.executed_path_pub.messages)

    def test_sweep_is_not_executed_when_its_transit_remains_blocked(self):
        manager, executed = self._manager(
            self._swath_segments(), ["blocked", "blocked"]
        )
        self._run(manager)
        self.assertEqual(["transit", "transit"], executed)
        self.assertEqual("COMPLETED_PARTIAL", manager.state)
        self.assertEqual([0, 1], manager.blocked_segments)

    def test_final_transit_retry_unlocks_its_dependent_sweep(self):
        manager, executed = self._manager(
            self._swath_segments(), ["blocked", "succeeded", "succeeded"]
        )
        self._run(manager)
        self.assertEqual(["transit", "transit", "sweep"], executed)
        self.assertEqual("COMPLETED", manager.state)
        self.assertEqual([], manager.blocked_segments)

    def test_cancel_during_final_retry_is_not_reported_as_partial(self):
        manager, executed = self._manager(
            self._swath_segments(), ["blocked", "canceled"]
        )
        self._run(manager)
        self.assertEqual(["transit", "transit"], executed)
        self.assertEqual("CANCELED", manager.state)

    def test_teb_restore_failure_forces_failed_terminal_state(self):
        manager, _executed = self._manager([], [], restore=False)
        self._run(manager)
        self.assertEqual("FAILED", manager.state)
        self.assertIn("TEB parameters could not be restored", manager.detail)

    @staticmethod
    def _kinematics_manager():
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.minimum_turning_radius = 1.35
        manager.expected_wheelbase = 0.65
        manager.chassis_wheelbase_tolerance = 0.02
        manager.steering_angle_margin = math.radians(2.0)
        manager.kinematics_verified = False
        manager.kinematics_detail = "pending"
        manager.detail = "pending"
        manager.required_steering_angle = 0.0
        manager.chassis_wheelbase = 0.0
        manager.chassis_max_steering_angle = 0.0
        manager.chassis_max_speed = 0.0
        return manager

    @staticmethod
    def _chassis_response(maximum_steering=0.488692):
        return SimpleNamespace(
            success=True,
            message="ready",
            parameters=SimpleNamespace(
                robot_length=0.65,
                max_steer=maximum_steering,
                max_speed=1.63284,
            ),
        )

    def _verify_kinematics(
        self, manager, response, teb_radius=1.35, teb_lookahead=8.0
    ):
        parameters = {
            "/move_base/TebLocalPlannerROS/min_turning_radius": teb_radius,
            "/move_base/TebLocalPlannerROS/wheelbase": 0.65,
            "/move_base/TebLocalPlannerROS/use_proportional_saturation": True,
            "/move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel": False,
            "/move_base/TebLocalPlannerROS/treat_unknown_as_obstacle": True,
            "/move_base/TebLocalPlannerROS/max_global_plan_lookahead_dist": teb_lookahead,
            "/move_base/local_costmap/width": 20.0,
            "/move_base/local_costmap/height": 20.0,
            "/move_base/CoverageGlobalPlanner_navfn/allow_unknown": False,
        }
        proxy = mock.Mock(return_value=response)
        with mock.patch.object(COVERAGE_MANAGER.rospy, "wait_for_service"), \
                mock.patch.object(
                    COVERAGE_MANAGER.rospy, "ServiceProxy", return_value=proxy
                ), mock.patch.object(
                    COVERAGE_MANAGER.rospy,
                    "get_param",
                    side_effect=lambda name: parameters[name],
                ):
            return manager._verify_kinematics_locked()

    def test_live_vcu_and_teb_kinematics_are_verified(self):
        manager = self._kinematics_manager()
        self.assertTrue(self._verify_kinematics(
            manager, self._chassis_response()
        ))
        self.assertTrue(manager.kinematics_verified)
        self.assertAlmostEqual(0.65, manager.chassis_wheelbase)
        self.assertAlmostEqual(0.488692, manager.chassis_max_steering_angle)

    def test_insufficient_live_steering_margin_fails_closed(self):
        manager = self._kinematics_manager()
        self.assertFalse(self._verify_kinematics(
            manager, self._chassis_response(maximum_steering=0.46)
        ))
        self.assertFalse(manager.kinematics_verified)
        self.assertIn("does not retain", manager.detail)

    def test_teb_radius_below_coverage_radius_fails_closed(self):
        manager = self._kinematics_manager()
        self.assertFalse(self._verify_kinematics(
            manager, self._chassis_response(), teb_radius=1.20
        ))
        self.assertFalse(manager.kinematics_verified)
        self.assertIn("below the coverage requirement", manager.detail)

    def test_teb_lookahead_must_fit_inside_rolling_costmap_margin(self):
        manager = self._kinematics_manager()
        self.assertFalse(self._verify_kinematics(
            manager, self._chassis_response(), teb_lookahead=12.0
        ))
        self.assertFalse(manager.kinematics_verified)
        self.assertIn("lookahead", manager.detail)

    @staticmethod
    def _avoidance_manager(now=100.0):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.avoidance_scan_fresh_sec = 0.5
        manager.dual_lidar_fresh_sec = 1.0
        manager.avoidance_scan_future_tolerance_sec = 0.2
        manager.avoidance_scan_frame = "base_link"
        manager.require_dual_lidar = True
        manager.avoidance_scan_valid = True
        manager.avoidance_scan_detail = "valid"
        manager.avoidance_scan_received_wall = now - 0.1
        manager.avoidance_scan_last_valid_wall = now - 0.1
        manager.dual_lidar_active = True
        manager.dual_lidar_received_wall = now - 0.1
        manager.avoidance_loss_paused = False
        manager.chassis_status_fresh_sec = 3.0
        manager.chassis_odom_fresh_sec = 1.0
        manager.chassis_monitor_fault_latch_sec = 3.0
        manager.chassis_status = SimpleNamespace(
            hard_emergency=False,
            soft_emergency=False,
            gamepad_emergency=False,
            robot_emergency=False,
        )
        manager.chassis_status_received_wall = now - 0.1
        manager.chassis_odom_received_wall = now - 0.1
        manager.chassis_monitor = SimpleNamespace(
            tcu_state=0,
            tcu_timeout=0,
            tcu_stuck=0,
            lecu_state=0,
            lecu_timeout=0,
            lecu_stuck=0,
            lecu_brake=0,
            recu_state=0,
            recu_timeout=0,
            recu_stuck=0,
            recu_brake=0,
        )
        manager.chassis_monitor_received_wall = now - 0.1
        manager.chassis_fault_paused = False
        return manager

    @staticmethod
    def _scan(stamp=99.9, frame="base_link"):
        return SimpleNamespace(
            header=SimpleNamespace(
                frame_id=frame,
                stamp=SimpleNamespace(to_sec=lambda: stamp),
            ),
            ranges=[1.0, float("inf")],
            angle_min=-1.0,
            angle_max=1.0,
            angle_increment=0.1,
            range_min=0.1,
            range_max=12.0,
        )

    def test_fresh_scan_and_dual_lidar_are_required_for_coverage(self):
        manager = self._avoidance_manager()
        ready, detail = manager._avoidance_ready_locked(100.0)
        self.assertTrue(ready)
        self.assertIn("MID360 and front/rear LD19", detail)

    def test_mid360_only_fallback_fails_coverage_closed(self):
        manager = self._avoidance_manager()
        manager.dual_lidar_active = False
        ready, detail = manager._avoidance_ready_locked(100.0)
        self.assertFalse(ready)
        self.assertIn("not contributing", detail)

    def test_stale_avoidance_scan_fails_coverage_closed(self):
        manager = self._avoidance_manager()
        manager.avoidance_scan_last_valid_wall = 98.0
        ready, detail = manager._avoidance_ready_locked(100.0)
        self.assertFalse(ready)
        self.assertIn("no recent valid", detail)

    def test_one_rejected_scan_does_not_latch_pause_while_last_valid_is_fresh(self):
        manager = self._avoidance_manager()
        manager.avoidance_scan_valid = False
        manager.avoidance_scan_detail = "/scan timestamp is stale"
        manager.avoidance_scan_received_wall = 100.0
        ready, detail = manager._avoidance_ready_locked(100.0)
        self.assertTrue(ready)
        self.assertIn("last valid sample is still fresh", detail)

    def test_continuing_rejected_scans_fail_after_last_valid_timeout(self):
        manager = self._avoidance_manager()
        manager.avoidance_scan_valid = False
        manager.avoidance_scan_detail = "/scan geometry or ranges are invalid"
        manager.avoidance_scan_received_wall = 100.6
        ready, detail = manager._avoidance_ready_locked(100.6)
        self.assertFalse(ready)
        self.assertIn("latest rejected", detail)

    def test_scan_frame_and_message_timestamp_are_checked(self):
        manager = self._avoidance_manager()
        valid, detail = manager._validate_avoidance_scan(
            self._scan(frame="laser"), ros_now=100.0
        )
        self.assertFalse(valid)
        self.assertIn("expected base_link", detail)
        valid, detail = manager._validate_avoidance_scan(
            self._scan(stamp=98.0), ros_now=100.0
        )
        self.assertFalse(valid)
        self.assertIn("timestamp is stale", detail)
        valid, detail = manager._validate_avoidance_scan(
            self._scan(stamp=100.3), ros_now=100.0
        )
        self.assertFalse(valid)
        self.assertIn("future", detail)
        valid, _detail = manager._validate_avoidance_scan(
            self._scan(), ros_now=100.0
        )
        self.assertTrue(valid)

    def test_one_obstacle_sensing_loss_cancels_only_once(self):
        manager = self._avoidance_manager()
        manager.lock = threading.RLock()
        manager.active = True
        manager.cancel_requested = False
        manager.manual_pause = False
        manager.manual_pause_reason = ""
        manager.state = "SWEEPING"
        manager.detail = "running"
        manager.move_base = _MoveBase()
        manager.avoidance_scan_last_valid_wall = 0.0
        manager._pause_for_avoidance_loss()
        manager._pause_for_avoidance_loss()
        self.assertEqual(1, manager.move_base.cancel_count)
        self.assertTrue(manager.manual_pause)
        self.assertTrue(manager.avoidance_loss_paused)
        self.assertEqual("PAUSED", manager.state)

    def test_fresh_vcu_status_without_faults_allows_coverage(self):
        manager = self._avoidance_manager()
        ready, detail = manager._chassis_ready_locked(100.0)
        self.assertTrue(ready)
        self.assertIn("no fault", detail)

    def test_stale_m2_feedback_odometry_fails_chassis_execution_closed(self):
        manager = self._avoidance_manager()
        manager.chassis_odom_received_wall = 98.0
        ready, detail = manager._chassis_ready_locked(100.0)
        self.assertFalse(ready)
        self.assertIn("feedback odometry", detail)

    def test_gamepad_emergency_fails_chassis_execution_closed(self):
        manager = self._avoidance_manager()
        manager.chassis_status.gamepad_emergency = True
        ready, detail = manager._chassis_ready_locked(100.0)
        self.assertFalse(ready)
        self.assertIn("gamepad/remote emergency", detail)

    def test_controller_emergency_and_stale_status_are_distinguished(self):
        manager = self._avoidance_manager()
        manager.chassis_monitor.tcu_state = 1
        ready, detail = manager._chassis_ready_locked(100.0)
        self.assertFalse(ready)
        self.assertIn("TCU emergency", detail)
        manager.chassis_monitor.tcu_state = 0
        manager.chassis_monitor_received_wall = 95.0
        ready, detail = manager._chassis_ready_locked(100.0)
        self.assertTrue(ready)
        self.assertIn("no recent controller fault frame", detail)
        manager.chassis_status_received_wall = 95.0
        ready, detail = manager._chassis_ready_locked(100.0)
        self.assertFalse(ready)
        self.assertIn("absent or stale", detail)

    def test_chassis_fault_cancels_active_goal_only_once(self):
        manager = self._avoidance_manager()
        manager.lock = threading.RLock()
        manager.active = True
        manager.cancel_requested = False
        manager.manual_pause = False
        manager.manual_pause_reason = ""
        manager.state = "TRANSITING"
        manager.detail = "running"
        manager.move_base = _MoveBase()
        manager.chassis_status.gamepad_emergency = True
        with mock.patch.object(
            COVERAGE_MANAGER.time, "monotonic", return_value=100.0
        ):
            manager._pause_for_chassis_fault()
            manager._pause_for_chassis_fault()
        self.assertEqual(1, manager.move_base.cancel_count)
        self.assertTrue(manager.manual_pause)
        self.assertTrue(manager.chassis_fault_paused)
        self.assertEqual("PAUSED", manager.state)

    def test_manual_resume_requires_recovered_obstacle_sensing(self):
        manager = self._avoidance_manager()
        manager.lock = threading.RLock()
        manager.active = True
        manager.manual_pause = True
        manager.manual_pause_reason = "obstacle sensing lost"
        manager.external_pause = False
        manager.avoidance_loss_paused = True
        manager.chassis_fault_paused = False
        manager._localization_is_fresh = lambda: True
        manager._publish_status = lambda: None
        manager.move_base = _MoveBase()
        with mock.patch.object(
            COVERAGE_MANAGER.time, "monotonic", return_value=100.0
        ):
            response = manager._pause_service(SimpleNamespace(data=False))
        self.assertTrue(response.success)
        self.assertFalse(manager.manual_pause)
        self.assertFalse(manager.avoidance_loss_paused)
        manager.dual_lidar_active = False
        manager.manual_pause = True
        manager.avoidance_loss_paused = True
        with mock.patch.object(
            COVERAGE_MANAGER.time, "monotonic", return_value=100.0
        ):
            response = manager._pause_service(SimpleNamespace(data=False))
        self.assertFalse(response.success)
        self.assertTrue(manager.manual_pause)
        self.assertTrue(manager.avoidance_loss_paused)

    def test_manual_resume_requires_recovered_chassis_execution(self):
        manager = self._avoidance_manager()
        manager.lock = threading.RLock()
        manager.active = True
        manager.manual_pause = True
        manager.manual_pause_reason = "chassis execution lost"
        manager.external_pause = False
        manager.chassis_fault_paused = True
        manager._localization_is_fresh = lambda: True
        manager._publish_status = lambda: None
        manager.move_base = _MoveBase()
        manager.chassis_status.gamepad_emergency = True
        with mock.patch.object(
            COVERAGE_MANAGER.time, "monotonic", return_value=100.0
        ):
            response = manager._pause_service(SimpleNamespace(data=False))
        self.assertFalse(response.success)
        self.assertTrue(manager.manual_pause)
        self.assertTrue(manager.chassis_fault_paused)
        manager.chassis_status.gamepad_emergency = False
        with mock.patch.object(
            COVERAGE_MANAGER.time, "monotonic", return_value=100.0
        ):
            response = manager._pause_service(SimpleNamespace(data=False))
        self.assertTrue(response.success)
        self.assertFalse(manager.manual_pause)
        self.assertFalse(manager.chassis_fault_paused)

    def test_pause_reason_remains_visible_while_worker_waits(self):
        manager = self._avoidance_manager()
        manager.lock = threading.RLock()
        manager.cancel_requested = False
        manager.manual_pause = True
        manager.manual_pause_reason = "localization lost; manual resume is required"
        manager.external_pause = False
        manager.state = "TRANSITING"
        manager.detail = "executing transit segment 1 of 6"
        with mock.patch.object(
            COVERAGE_MANAGER.rospy,
            "is_shutdown",
            side_effect=[False, True],
        ), mock.patch.object(COVERAGE_MANAGER.time, "sleep"):
            self.assertFalse(manager._wait_while_paused())
        self.assertEqual("PAUSED", manager.state)
        self.assertEqual(manager.manual_pause_reason, manager.detail)


if __name__ == "__main__":
    unittest.main()
