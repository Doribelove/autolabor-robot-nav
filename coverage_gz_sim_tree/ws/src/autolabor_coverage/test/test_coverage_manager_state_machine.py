#!/usr/bin/env python3

import importlib.util
import ast
import copy
import inspect
import math
from pathlib import Path
from types import SimpleNamespace
import threading
import tempfile
import textwrap
import unittest
from unittest import mock

from geometry_msgs.msg import Point32
import yaml


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


class _GoalHandle:
    def __init__(self, client, on_cancel=None):
        self.client = client
        self.on_cancel = on_cancel

    def cancel(self):
        if self.on_cancel is not None:
            return self.on_cancel()
        return self.client.cancel_goal()


class _MoveBase:
    def __init__(self):
        self.cancel_count = 0
        self.cancel_all_count = 0
        self.gh = None
        self.state = COVERAGE_MANAGER.GoalStatus.PREEMPTED
        self.wait_result = True

    def cancel_goal(self):
        self.cancel_count += 1

    def cancel_all_goals(self):
        self.cancel_all_count += 1

    def wait_for_result(self, _timeout):
        return self.wait_result

    def get_state(self):
        return self.state


class _SuccessfulMoveBase(_MoveBase):
    def __init__(self):
        super().__init__()
        self.goals = []

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        del active_cb, feedback_cb
        self.gh = _GoalHandle(self)
        self.done_cb = done_cb
        self.goals.append(goal)

    @staticmethod
    def wait_for_result(_timeout):
        return True

    def get_state(self):
        return COVERAGE_MANAGER.GoalStatus.SUCCEEDED


class _PendingUntilCanceledMoveBase(_SuccessfulMoveBase):
    """Remain ACTIVE until the manager exactly cancels the tracked goal."""

    def wait_for_result(self, _timeout):
        return self.cancel_count > 0

    def get_state(self):
        return (
            COVERAGE_MANAGER.GoalStatus.PREEMPTED
            if self.cancel_count > 0
            else COVERAGE_MANAGER.GoalStatus.ACTIVE
        )


class CoverageManagerStateMachineTest(unittest.TestCase):
    def test_status_publish_uses_transition_angular_accel_rps2(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.status_pub = _Publisher()
        manager.plan = None
        manager.state = "IDLE"
        manager.plan_id = ""
        manager.grid = None
        manager._localization_is_fresh = lambda: False
        manager.active = False
        manager.manual_pause = False
        manager.external_pause = False
        manager.current_segment = 0
        manager.total_segments = 0
        manager.blocked_segments = []
        manager.operation_width = 1.0
        manager.overlap_ratio = 0.15
        manager.minimum_turning_radius = 1.35
        manager.task_time_parameters = COVERAGE_MANAGER.CoverageTimeParameters()
        manager.watchdog_motion_enabled = True
        manager.watchdog_max_linear_speed = 0.8
        manager.watchdog_max_angular_speed = 0.6
        transition_parameters = COVERAGE_MANAGER.CoverageTimeParameters(
            angular_accel_rps2=0.37
        )
        manager._transit_time_parameters = lambda _parameters: transition_parameters
        manager.hierarchical_hybrid_on_demand = True
        manager.direct_hybrid_to_final_goal = True
        manager.entry_navfn_single_topology = True
        manager.hybrid_execute_unsplit_cusps = False
        manager.hybrid_transit_lookahead_distance = 0.3
        manager.required_steering_angle = 0.45
        manager.chassis_wheelbase = 0.65
        manager.chassis_max_steering_angle = 0.5
        manager.chassis_max_speed = 1.6
        manager.kinematics_verified = True
        manager.kinematics_detail = "verified"
        manager._chassis_ready_locked = lambda: (True, "ready")
        manager._avoidance_ready_locked = lambda: (True, "ready")
        manager.map_digest = ""
        manager.batch_id = ""
        manager.batch_active = False
        manager.batch_cancel_requested = False
        manager.batch_current_index = 0
        manager.batch_total_regions = 0
        manager.batch_completed_regions = 0
        manager.batch_partial_regions = 0
        manager.batch_skipped_regions = 0
        manager.current_region_id = ""
        manager.current_region_name = ""
        manager.last_region_id = ""
        manager.last_region_name = ""
        manager.last_region_state = ""
        manager.detail = "waiting"

        with mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(1.0),
        ):
            manager._publish_status()

        self.assertEqual(1, len(manager.status_pub.messages))
        self.assertAlmostEqual(
            0.37,
            manager.status_pub.messages[0].transition_angular_accel_rps2,
        )

    @staticmethod
    def _planning_defaults_manager(config_file):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.active = False
        manager.batch_active = False
        manager.plan_pending = False
        manager.start_pending = False
        manager.plan = None
        manager.original_teb = None
        manager.navigation_profile_update_pending = False
        manager.max_speed_limit = 1.60
        manager.max_reverse_speed_limit = 0.80
        manager.max_angular_speed_limit = 1.00
        manager.linear_accel_limit = 2.00
        manager.angular_accel_limit = 1.00
        manager.operation_width = 1.00
        manager.overlap_ratio = 0.15
        manager.config_file = str(config_file)
        manager.factory_defaults_file = str(
            PACKAGE_ROOT / "config" / "coverage_factory_defaults.yaml"
        )
        manager.default_time_parameters = COVERAGE_MANAGER.CoverageTimeParameters()
        manager._apply_time_parameters_locked(manager.default_time_parameters)
        manager._publish_status = lambda: None
        return manager

    def test_planning_defaults_yaml_update_preserves_unrelated_content(self):
        source_path = PACKAGE_ROOT / "config" / "coverage.yaml"
        source = source_path.read_text(encoding="utf-8")
        parameters = SimpleNamespace(
            operation_width_m=1.35,
            overlap_ratio=0.20,
            allow_reverse=False,
            max_forward_speed_mps=0.90,
            max_reverse_speed_mps=0.25,
            max_angular_speed_rps=0.55,
            linear_accel_mps2=0.80,
            angular_accel_rps2=0.45,
            direction_change_penalty_sec=1.20,
            segment_handoff_penalty_sec=0.70,
            transit_replan_period_sec=2.00,
        )
        rewritten = COVERAGE_MANAGER._rewrite_flat_yaml_values(
            source,
            COVERAGE_MANAGER._planning_values_to_yaml(parameters),
        )
        self.assertIn("operation_width_m: 1.35", rewritten)
        self.assertIn("allow_reverse_transit: false", rewritten)
        self.assertIn("default_transit_replan_period_sec: 2.00", rewritten)
        self.assertEqual(
            next(line for line in source.splitlines()
                 if line.startswith("minimum_turning_radius_m:")),
            next(line for line in rewritten.splitlines()
                 if line.startswith("minimum_turning_radius_m:")),
        )
        self.assertEqual(source.count("#"), rewritten.count("#"))

    def test_planning_defaults_yaml_commit_is_atomic_and_parseable(self):
        parameters = SimpleNamespace(
            operation_width_m=1.10,
            overlap_ratio=0.10,
            allow_reverse=True,
            max_forward_speed_mps=0.70,
            max_reverse_speed_mps=0.20,
            max_angular_speed_rps=0.50,
            linear_accel_mps2=0.90,
            angular_accel_rps2=0.40,
            direction_change_penalty_sec=1.00,
            segment_handoff_penalty_sec=0.50,
            transit_replan_period_sec=1.50,
        )
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "coverage.yaml"
            target.write_text(
                (PACKAGE_ROOT / "config" / "coverage.yaml").read_text(
                    encoding="utf-8"
                ),
                encoding="utf-8",
            )
            staged, resolved = COVERAGE_MANAGER._stage_atomic_yaml_update(
                str(target), parameters
            )
            self.assertEqual(str(target), resolved)
            self.assertEqual(0.70, yaml.safe_load(
                Path(staged).read_text(encoding="utf-8")
            )["default_max_speed_mps"])
            COVERAGE_MANAGER._commit_staged_yaml(staged, resolved)
            self.assertEqual(1.10, yaml.safe_load(
                target.read_text(encoding="utf-8")
            )["operation_width_m"])

    def test_planning_defaults_service_commits_teb_runtime_and_yaml_together(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "coverage.yaml"
            target.write_text(
                (PACKAGE_ROOT / "config" / "coverage.yaml").read_text(
                    encoding="utf-8"
                ),
                encoding="utf-8",
            )
            manager = self._planning_defaults_manager(target)
            request = COVERAGE_MANAGER.SetCoveragePlanningDefaultsRequest()
            request.parameters = manager._planning_defaults_message(
                1.25,
                0.20,
                COVERAGE_MANAGER.CoverageTimeParameters(
                    max_forward_speed_mps=0.90,
                    max_reverse_speed_mps=0.25,
                    max_angular_speed_rps=0.55,
                    linear_accel_mps2=0.90,
                    angular_accel_rps2=0.45,
                    allow_reverse=True,
                    direction_change_penalty_sec=1.20,
                    segment_handoff_penalty_sec=0.70,
                    transit_replan_period_sec=2.00,
                ),
            )
            previous_teb = manager._teb_navigation_profile(
                manager.default_time_parameters
            )
            client = mock.Mock()
            client.get_configuration.return_value = dict(previous_teb)
            client.update_configuration.side_effect = lambda update: dict(update)
            with mock.patch(
                "dynamic_reconfigure.client.Client", return_value=client
            ), mock.patch.object(COVERAGE_MANAGER.rospy, "set_param"):
                response = manager._set_planning_defaults_service(request)
            self.assertTrue(response.success, response.message)
            self.assertEqual(1.25, manager.operation_width)
            self.assertEqual(0.90, manager.default_max_speed)
            self.assertEqual(1.25, yaml.safe_load(
                target.read_text(encoding="utf-8")
            )["operation_width_m"])
            self.assertFalse(manager.navigation_profile_update_pending)
            client.update_configuration.assert_called_once()

    def test_planning_defaults_yaml_failure_rolls_teb_back_without_runtime_commit(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "coverage.yaml"
            original = (PACKAGE_ROOT / "config" / "coverage.yaml").read_text(
                encoding="utf-8"
            )
            target.write_text(original, encoding="utf-8")
            manager = self._planning_defaults_manager(target)
            request = COVERAGE_MANAGER.SetCoveragePlanningDefaultsRequest()
            request.parameters = manager._planning_defaults_message(
                1.40,
                0.25,
                COVERAGE_MANAGER.CoverageTimeParameters(
                    max_forward_speed_mps=0.75,
                    max_reverse_speed_mps=0.20,
                    max_angular_speed_rps=0.50,
                    linear_accel_mps2=0.80,
                    angular_accel_rps2=0.40,
                    allow_reverse=False,
                    direction_change_penalty_sec=1.00,
                    segment_handoff_penalty_sec=0.50,
                    transit_replan_period_sec=1.50,
                ),
            )
            previous_teb = manager._teb_navigation_profile(
                manager.default_time_parameters
            )
            client = mock.Mock()
            client.get_configuration.return_value = dict(previous_teb)
            client.update_configuration.side_effect = lambda update: dict(update)
            with mock.patch(
                "dynamic_reconfigure.client.Client", return_value=client
            ), mock.patch.object(
                COVERAGE_MANAGER,
                "_commit_staged_yaml",
                side_effect=OSError("simulated fsync/rename failure"),
            ):
                response = manager._set_planning_defaults_service(request)
            self.assertFalse(response.success)
            self.assertIn("simulated fsync/rename failure", response.message)
            self.assertEqual(original, target.read_text(encoding="utf-8"))
            self.assertEqual(1.00, manager.operation_width)
            self.assertEqual(2, client.update_configuration.call_count)
            self.assertEqual(
                previous_teb,
                client.update_configuration.call_args_list[-1].args[0],
            )
            self.assertFalse(manager.navigation_profile_update_pending)

    @staticmethod
    def _seed_move_base_goal_tracking(manager):
        manager.move_base_terminal_timeout_sec = 2.0
        manager.move_base_goal_generation = 0
        manager.move_base_goal_pending = False
        manager.move_base_goal_handle = None
        manager.move_base_goal_terminal_state = COVERAGE_MANAGER.GoalStatus.LOST

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
        CoverageManagerStateMachineTest._seed_move_base_goal_tracking(manager)
        manager.plan_pending = False
        manager.plan_token = ""
        manager.start_pending = False
        manager.start_token = ""
        manager.navigation_owner_service_name = (
            "/navigation_pause/set_coverage_owner"
        )
        manager.navigation_owner_token = ""
        manager.navigation_owner_claimed = False
        manager.navigation_owner_releasing = False
        manager.retained_cleanup_lock = threading.Lock()
        manager._set_navigation_owner = lambda _claim, _token: (True, "ok")
        manager.batch_id = ""
        manager.batch_token = ""
        manager.batch_start_request_id = ""
        manager.batch_request_records = {}
        manager.batch_map_digest = ""
        manager.batch_active = False
        manager.batch_cancel_requested = False
        manager.batch_skip_requested = False
        manager.batch_abort_detail = ""
        manager.batch_phase = "IDLE"
        manager.batch_region_token = ""
        manager.batch_region_outcome = ""
        manager.batch_current_is_last = False
        manager.batch_regions = []
        manager.batch_current_index = 0
        manager.batch_total_regions = 0
        manager.batch_completed_regions = 0
        manager.batch_partial_regions = 0
        manager.batch_skipped_regions = 0
        manager.current_region_id = ""
        manager.current_region_name = ""
        manager.last_region_id = ""
        manager.last_region_name = ""
        manager.last_region_state = ""
        manager.batch_worker = None
        manager.batch_wake_event = threading.Event()
        manager.path_pub = _Publisher()
        manager.hybrid_transition_path_pub = _Publisher()
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
        self.assertTrue(manager.hybrid_transition_path_pub.messages)
        self.assertEqual(
            [], list(manager.hybrid_transition_path_pub.messages[-1].poses)
        )
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
        manager.navigation_owner_token = "coverage-{}".format("1" * 32)
        manager.navigation_owner_claimed = True
        manager.manual_pause = False
        manager.manual_pause_reason = ""
        manager.external_pause = False
        manager.avoidance_loss_paused = False
        manager.chassis_fault_paused = False
        manager.move_base = _MoveBase()
        manager.enforced_path_pub = _Publisher()
        manager._segments = lambda _route, _current: segments
        # A connector that exhausts retries now pauses instead of skipping its
        # dependent swath.  Tests without an operator resume model cancel that
        # pause deterministically.
        manager._wait_while_paused = lambda: False
        manager._restore_teb = lambda: restore
        manager._set_enforced_path = lambda _path, coverage_active=True: True
        manager._publish_status = lambda: None
        # These lifecycle tests are not pose-geometry tests. Model an already
        # accepted connector hand-off; dedicated tests below exercise the real
        # pre-sweep acquisition gate.
        manager._prepare_sweep_entry = lambda _segment, _index: "ready"
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

    @staticmethod
    def _batch_region(region_id):
        region = COVERAGE_MANAGER.PolygonStamped()
        region.header.frame_id = "map"
        region.polygon.points = [
            Point32(x=0.0, y=0.0, z=0.0),
            Point32(x=2.0, y=0.0, z=0.0),
            Point32(x=2.0, y=2.0, z=0.0),
        ]
        return SimpleNamespace(
            id=region_id,
            name="Region {}".format(region_id),
            region=region,
        )

    def _batch_manager(self, outcomes):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(
            manager, current_segment=0, total_segments=0, blocked_segments=[]
        )
        manager.batch_id = "test-batch"
        manager.batch_token = "test-batch-token"
        manager.batch_map_digest = "test-map"
        manager.batch_active = True
        manager.batch_cancel_requested = False
        manager.batch_skip_requested = False
        manager.batch_abort_detail = ""
        manager.batch_phase = "PLANNING"
        manager.batch_region_token = "region-token"
        manager.batch_region_outcome = ""
        manager.batch_current_is_last = False
        manager.batch_regions = [
            self._batch_region("one"), self._batch_region("two")
        ]
        manager.batch_current_index = 1
        manager.batch_total_regions = 2
        manager.batch_completed_regions = 0
        manager.batch_partial_regions = 0
        manager.batch_skipped_regions = 0
        manager.current_region_id = "one"
        manager.current_region_name = "Region one"
        manager.last_region_id = ""
        manager.last_region_name = ""
        manager.last_region_state = ""
        manager.batch_worker = object()
        manager.batch_wake_event = threading.Event()
        manager.active = False
        manager.navigation_owner_token = "coverage-{}".format("2" * 32)
        manager.navigation_owner_claimed = True
        manager.cancel_requested = False
        manager.manual_pause = False
        manager.manual_pause_reason = ""
        manager.external_pause = False
        manager.avoidance_loss_paused = False
        manager.chassis_fault_paused = False
        manager.state = "PLANNING"
        manager.detail = "test batch"
        manager.move_base = _MoveBase()
        manager.enforced_path_pub = _Publisher()
        manager._set_enforced_path = (
            lambda _path, coverage_active=True: True
        )
        manager._restore_teb = lambda: True
        manager._publish_status = lambda: None
        manager.active_pub.publish(COVERAGE_MANAGER.Bool(data=True))

        prepared = []
        executed = []
        pending_outcomes = list(outcomes)

        def prepare(_token, item):
            prepared.append(item.id)
            manager.plan = SimpleNamespace()
            manager.plan_id = "plan-{}".format(item.id)
            manager.plan_map_digest = manager.batch_map_digest
            manager.region = copy.deepcopy(item.region)
            return "READY", [item.id], COVERAGE_MANAGER.Point(0.0, 0.0)

        def run(route, _current, batch_context=False):
            self.assertTrue(batch_context)
            manager.active = True
            executed.append(route[0])
            outcome = pending_outcomes.pop(0)
            manager.active = False
            return outcome

        manager._prepare_batch_region = prepare
        manager._run_task = run
        return manager, prepared, executed

    @staticmethod
    def _run_batch_sync(manager):
        with mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            manager._run_batch("test-batch-token")

    def _run(self, manager):
        with mock.patch.object(
            COVERAGE_MANAGER.rospy, "is_shutdown", return_value=False
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            manager._run_task([], None)

    def test_plan_requires_the_exact_current_map_digest(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        manager.plan = None
        manager.plan_id = ""
        manager.active = False
        manager.batch_active = False
        manager.grid = object()
        manager.map_digest = "actual-map"
        for supplied, expected in (
            ("", "required"),
            ("stale-map", "does not match"),
        ):
            with self.subTest(supplied=supplied):
                request = SimpleNamespace(map_digest=supplied)
                response = manager._plan_service(request)
                self.assertFalse(response.success)
                self.assertIn(expected, response.message)
                self.assertEqual("actual-map", response.map_digest)

    def test_start_batch_freezes_regions_and_latches_public_ownership(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        manager.active = False
        manager.batch_active = False
        manager.batch_worker = None
        manager.grid = object()
        manager.map_digest = "actual-map"
        manager.max_speed_limit = 1.60
        manager.batch_wake_event = threading.Event()
        manager._publish_status = lambda: None
        item = self._batch_region("one")
        request = SimpleNamespace(
            client_request_id="coverage-batch-{}".format("a" * 32),
            regions=[item],
            operation_width_m=1.0,
            overlap_ratio=0.15,
            allow_reverse_transit=True,
            max_speed_mps=0.3,
            map_digest="actual-map",
        )
        events = []

        def set_owner(claim, owner_token):
            events.append(("owner", claim, owner_token, manager.lock._is_owned()))
            return True, "ok"

        manager._set_navigation_owner = set_owner
        manager.active_pub.publish = (
            lambda message: events.append(("active", message.data))
        )
        fake_worker = SimpleNamespace(
            start=mock.Mock(side_effect=lambda: events.append(("worker",)))
        )
        with mock.patch.object(
            COVERAGE_MANAGER.threading, "Thread", return_value=fake_worker
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            response = manager._start_batch_service(request)
        self.assertTrue(response.accepted)
        self.assertEqual(request.client_request_id, response.batch_id)
        self.assertTrue(manager.batch_active)
        self.assertFalse(manager.active)
        self.assertEqual(1, manager.batch_total_regions)
        self.assertEqual("owner", events[0][0])
        self.assertTrue(events[0][1])
        self.assertRegex(events[0][2], r"^coverage-[0-9a-f]{32}$")
        self.assertFalse(events[0][3])
        self.assertEqual(events[0][2], manager.navigation_owner_token)
        self.assertEqual([("active", True), ("worker",)], events[1:])
        fake_worker.start.assert_called_once_with()

        replay = manager._start_batch_service(request)
        self.assertTrue(replay.accepted)
        self.assertEqual(request.client_request_id, replay.batch_id)
        self.assertIn("already accepted", replay.message)
        fake_worker.start.assert_called_once_with()
        self.assertEqual(1, len([
            event for event in events if event[0] == "owner"
        ]))

        changed_request = copy.deepcopy(request)
        changed_request.max_speed_mps = 0.4
        changed_replay = manager._start_batch_service(changed_request)
        self.assertFalse(changed_replay.accepted)
        self.assertIn("different payload", changed_replay.message)
        fake_worker.start.assert_called_once_with()
        item.name = "mutated after service return"
        self.assertEqual("Region one", manager.batch_regions[0].name)

    def test_cancel_during_batch_claim_compensates_the_uncommitted_token(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        manager.active = False
        manager.batch_active = False
        manager.batch_worker = None
        manager.grid = object()
        manager.map_digest = "actual-map"
        manager.max_speed_limit = 1.60
        manager.batch_wake_event = threading.Event()
        manager.cancel_requested = False
        manager.move_base = _MoveBase()
        manager._set_enforced_path = (
            lambda _path, coverage_active=True: True
        )
        manager._restore_teb = lambda: True
        manager._publish_status = lambda: None
        request = SimpleNamespace(
            client_request_id="coverage-batch-{}".format("b" * 32),
            regions=[self._batch_region("one")],
            operation_width_m=1.0,
            overlap_ratio=0.15,
            allow_reverse_transit=True,
            max_speed_mps=0.3,
            map_digest="actual-map",
        )
        claim_entered = threading.Event()
        release_claim = threading.Event()
        owner_calls = []
        claim_calls = []

        def set_owner(claim, owner_token):
            owner_calls.append((claim, owner_token, manager.lock._is_owned()))
            if claim:
                claim_calls.append(owner_token)
                if len(claim_calls) == 1:
                    claim_entered.set()
                    self.assertTrue(release_claim.wait(2.0))
                    manager._navigation_owner_last_outcome = (
                        True, owner_token, "RETAINED_NOT_READY"
                    )
                    return False, "old goal is still canceling"
                manager._navigation_owner_last_outcome = (
                    True, owner_token, "READY"
                )
            return True, "ok"

        manager._set_navigation_owner = set_owner
        responses = []
        start_thread = threading.Thread(
            target=lambda: responses.append(
                manager._start_batch_service(request)
            )
        )
        with mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            start_thread.start()
            self.assertTrue(claim_entered.wait(1.0))
            cancel_response = manager._cancel_service(SimpleNamespace())
            self.assertEqual(request.client_request_id, manager.batch_id)
            self.assertTrue(manager.start_pending)
            self.assertEqual(
                "CANCEL_PENDING_BEFORE_START",
                manager.batch_request_records[request.client_request_id]["state"],
            )
            release_claim.set()
            start_thread.join(2.0)
        self.assertFalse(start_thread.is_alive())
        self.assertFalse(cancel_response.success)
        self.assertIn("in-flight owner claim", cancel_response.message)
        self.assertEqual(1, len(responses))
        self.assertFalse(responses[0].accepted)
        self.assertIn("canceled before", responses[0].message)
        self.assertEqual(3, len(owner_calls))
        self.assertTrue(owner_calls[0][0])
        self.assertTrue(owner_calls[1][0])
        self.assertFalse(owner_calls[2][0])
        self.assertEqual(1, len({call[1] for call in owner_calls}))
        self.assertEqual([False, False, False], [
            call[2] for call in owner_calls
        ])
        self.assertFalse(manager.active)
        self.assertFalse(manager.batch_active)
        self.assertEqual("", manager.batch_id)
        self.assertIsNone(manager.batch_worker)
        self.assertEqual("READY", manager.state)
        settled = manager._cancel_batch_service(SimpleNamespace(
            batch_id=request.client_request_id
        ))
        self.assertTrue(settled.success)
        self.assertTrue(settled.not_started)
        self.assertFalse(settled.cancellation_requested)

    def test_rejected_batch_owner_claim_clears_public_pending_identity(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        manager.active = False
        manager.batch_active = False
        manager.batch_worker = None
        manager.grid = object()
        manager.map_digest = "actual-map"
        manager.max_speed_limit = 1.60
        manager._publish_status = lambda: None
        manager._resolve_navigation_owner_claim = (
            lambda _owner_token, _context:
            ("REJECTED", "bridge has a different owner")
        )
        request = SimpleNamespace(
            client_request_id="coverage-batch-{}".format("6" * 32),
            regions=[self._batch_region("one")],
            operation_width_m=1.0,
            overlap_ratio=0.15,
            allow_reverse_transit=True,
            max_speed_mps=0.3,
            map_digest="actual-map",
        )

        response = manager._start_batch_service(request)

        self.assertFalse(response.accepted)
        self.assertEqual(request.client_request_id, response.batch_id)
        self.assertEqual("", manager.batch_id)
        self.assertFalse(manager.start_pending)
        self.assertFalse(manager.active)
        self.assertEqual("READY", manager.state)
        self.assertEqual(
            "REJECTED",
            manager.batch_request_records[request.client_request_id]["state"],
        )
        replay = manager._start_batch_service(request)
        self.assertFalse(replay.accepted)
        self.assertEqual(request.client_request_id, replay.batch_id)
        self.assertIn("different owner", replay.message)
        self.assertEqual("", manager.batch_id)

    def test_claim_response_lost_after_bridge_commit_reconciles_same_token(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        manager.active = False
        manager.batch_active = False
        manager.batch_worker = None
        manager.grid = object()
        manager.map_digest = "actual-map"
        manager.max_speed_limit = 1.60
        manager.batch_wake_event = threading.Event()
        manager.move_base = _MoveBase()
        manager._publish_status = lambda: None
        request = SimpleNamespace(
            client_request_id="coverage-batch-{}".format("8" * 32),
            regions=[self._batch_region("one")],
            operation_width_m=1.0,
            overlap_ratio=0.15,
            allow_reverse_transit=True,
            max_speed_mps=0.3,
            map_digest="actual-map",
        )
        bridge = {"owner": ""}
        owner_calls = []

        def set_owner(claim, owner_token):
            owner_calls.append((claim, owner_token, manager.lock._is_owned()))
            self.assertTrue(claim)
            if len(owner_calls) == 1:
                # Model bridge commit followed by loss of the service response.
                bridge["owner"] = owner_token
                manager._navigation_owner_last_outcome = (
                    True, owner_token, "UNKNOWN"
                )
                return False, "claim response lost after bridge commit"
            self.assertEqual(owner_token, bridge["owner"])
            if len(owner_calls) == 2:
                manager._navigation_owner_last_outcome = (
                    True, owner_token, "RETAINED_NOT_READY"
                )
                return False, "old goal has no trusted terminal status"
            manager._navigation_owner_last_outcome = (
                True, owner_token, "READY"
            )
            return True, "same-token claim reconciled"

        manager._set_navigation_owner = set_owner
        manager._lifecycle_wait = lambda _timeout: None
        fake_worker = SimpleNamespace(start=mock.Mock())
        with mock.patch.object(
            COVERAGE_MANAGER.threading, "Thread", return_value=fake_worker
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            response = manager._start_batch_service(request)
        self.assertTrue(response.accepted)
        self.assertEqual(request.client_request_id, response.batch_id)
        self.assertEqual(manager.navigation_owner_token, bridge["owner"])
        self.assertEqual(3, len(owner_calls))
        self.assertTrue(owner_calls[0][0])
        self.assertTrue(owner_calls[1][0])
        self.assertTrue(owner_calls[2][0])
        self.assertEqual(1, len({call[1] for call in owner_calls}))
        self.assertEqual([False, False, False], [
            call[2] for call in owner_calls
        ])
        self.assertFalse(manager.active)
        self.assertFalse(manager.start_pending)
        self.assertEqual(
            "ACTIVE",
            manager.batch_request_records[request.client_request_id]["state"],
        )
        fake_worker.start.assert_called_once_with()

    def test_cancel_before_start_tombstone_blocks_late_request(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        manager.active = False
        manager.batch_active = False
        manager.batch_worker = None
        manager.grid = object()
        manager.map_digest = "actual-map"
        manager.max_speed_limit = 1.60
        manager.batch_wake_event = threading.Event()
        manager._publish_status = lambda: None
        owner_calls = []
        manager._set_navigation_owner = (
            lambda claim, token:
            owner_calls.append((claim, token)) or (True, "ok")
        )
        request_id = "coverage-batch-{}".format("c" * 32)
        canceled = manager._cancel_batch_service(SimpleNamespace(
            batch_id=request_id
        ))
        self.assertTrue(canceled.success)
        self.assertTrue(canceled.not_started)
        request = SimpleNamespace(
            client_request_id=request_id,
            regions=[self._batch_region("one")],
            operation_width_m=1.0,
            overlap_ratio=0.15,
            allow_reverse_transit=True,
            max_speed_mps=0.3,
            map_digest="actual-map",
        )
        response = manager._start_batch_service(request)
        self.assertFalse(response.accepted)
        self.assertEqual(request_id, response.batch_id)
        self.assertIn("canceled before", response.message)
        self.assertEqual([], owner_calls)
        self.assertFalse(manager.batch_active)

    def test_inflight_claim_cancel_keeps_id_until_failed_release_is_retried(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        manager.active = False
        manager.batch_active = False
        manager.batch_worker = None
        manager.grid = object()
        manager.map_digest = "actual-map"
        manager.max_speed_limit = 1.60
        manager.batch_wake_event = threading.Event()
        manager.cancel_requested = False
        manager.move_base = _MoveBase()
        manager._set_enforced_path = (
            lambda _path, coverage_active=True: True
        )
        manager._restore_teb = lambda: True
        manager._publish_status = lambda: None
        request = SimpleNamespace(
            client_request_id="coverage-batch-{}".format("9" * 32),
            regions=[self._batch_region("one")],
            operation_width_m=1.0,
            overlap_ratio=0.15,
            allow_reverse_transit=True,
            max_speed_mps=0.3,
            map_digest="actual-map",
        )
        claim_entered = threading.Event()
        release_claim = threading.Event()
        release_calls = []

        def set_owner(claim, owner_token):
            if claim:
                claim_entered.set()
                self.assertTrue(release_claim.wait(2.0))
                return True, "claimed"
            release_calls.append(owner_token)
            if len(release_calls) == 1:
                return False, "release response lost"
            return True, "released"

        manager._set_navigation_owner = set_owner
        responses = []
        start_thread = threading.Thread(
            target=lambda: responses.append(
                manager._start_batch_service(request)
            )
        )
        with mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            start_thread.start()
            self.assertTrue(claim_entered.wait(1.0))
            pending = manager._cancel_batch_service(SimpleNamespace(
                batch_id=request.client_request_id
            ))
            self.assertFalse(pending.success)
            self.assertFalse(pending.not_started)
            self.assertTrue(pending.cancellation_requested)
            release_claim.set()
            start_thread.join(2.0)

        self.assertFalse(start_thread.is_alive())
        self.assertEqual(1, len(responses))
        self.assertFalse(responses[0].accepted)
        self.assertEqual(request.client_request_id, manager.batch_id)
        self.assertTrue(manager.active)
        self.assertEqual(
            "FAILED_RETAINED",
            manager.batch_request_records[request.client_request_id]["state"],
        )
        settled = manager._cancel_batch_service(SimpleNamespace(
            batch_id=request.client_request_id
        ))
        self.assertTrue(settled.success)
        self.assertTrue(settled.not_started)
        self.assertFalse(settled.cancellation_requested)
        self.assertFalse(manager.active)
        self.assertEqual("", manager.batch_id)
        self.assertEqual(
            "TERMINAL",
            manager.batch_request_records[request.client_request_id]["state"],
        )
        self.assertEqual(2, len(release_calls))

    def test_foreign_batch_cancel_never_changes_current_batch(self):
        manager, _prepared, _executed = self._batch_manager([])
        current_id = "coverage-batch-{}".format("d" * 32)
        foreign_id = "coverage-batch-{}".format("e" * 32)
        manager.batch_id = current_id
        manager.batch_request_records[current_id] = {
            "fingerprint": "payload",
            "state": "ACTIVE",
            "accepted": True,
            "message": "accepted",
        }
        response = manager._cancel_batch_service(SimpleNamespace(
            batch_id=foreign_id
        ))
        self.assertTrue(response.success)
        self.assertTrue(response.not_started)
        self.assertFalse(response.cancellation_requested)
        self.assertFalse(manager.batch_cancel_requested)
        self.assertEqual(current_id, manager.batch_id)
        self.assertEqual(0, manager.move_base.cancel_count)
        self.assertEqual(0, manager.move_base.cancel_all_count)

    def test_exact_cancel_retries_failed_retained_precommit_owner_release(self):
        manager, _executed = self._manager([], [])
        request_id = "coverage-batch-{}".format("f" * 32)
        owner_token = manager.navigation_owner_token
        manager.batch_id = request_id
        manager.batch_active = False
        manager.batch_phase = "FINALIZING"
        manager.navigation_owner_releasing = True
        manager.batch_request_records[request_id] = {
            "fingerprint": "payload",
            "state": "FAILED_RETAINED",
            "accepted": False,
            "message": "owner release failed",
            "owner_token": owner_token,
        }
        releases = mock.Mock(side_effect=[
            (False, "bridge timeout"),
            (True, "released"),
        ])
        manager._set_navigation_owner = releases

        first = manager._cancel_batch_service(SimpleNamespace(
            batch_id=request_id
        ))
        self.assertFalse(first.success)
        self.assertTrue(first.cancellation_requested)
        self.assertTrue(first.not_started)
        self.assertTrue(manager.active)
        self.assertEqual(owner_token, manager.navigation_owner_token)
        self.assertEqual(
            "FAILED_RETAINED",
            manager.batch_request_records[request_id]["state"],
        )

        second = manager._cancel_batch_service(SimpleNamespace(
            batch_id=request_id
        ))
        self.assertTrue(second.success)
        self.assertFalse(second.cancellation_requested)
        self.assertTrue(second.not_started)
        self.assertFalse(manager.active)
        self.assertEqual("", manager.batch_id)
        self.assertEqual("", manager.navigation_owner_token)
        self.assertEqual(
            "TERMINAL", manager.batch_request_records[request_id]["state"]
        )
        self.assertEqual([
            mock.call(False, owner_token), mock.call(False, owner_token)
        ], releases.call_args_list)
        self.assertEqual(0, manager.move_base.cancel_all_count)

    def test_retained_cleanup_is_single_owner_and_blocks_new_batch_identity(self):
        manager, _executed = self._manager([], [])
        batch_a = "coverage-batch-{}".format("a" * 32)
        batch_b = "coverage-batch-{}".format("b" * 32)
        owner_a = manager.navigation_owner_token
        manager.batch_id = batch_a
        manager.batch_active = False
        manager.batch_phase = "FINALIZING"
        manager.navigation_owner_releasing = True
        manager.batch_request_records[batch_a] = {
            "fingerprint": "payload-a",
            "state": "FAILED_RETAINED",
            "accepted": False,
            "message": "owner retained",
            "owner_token": owner_a,
        }
        manager.max_speed_limit = 1.60
        manager.grid = object()

        cancel_entered = threading.Event()
        allow_cancel = threading.Event()
        cancel_calls = []

        def cancel_a():
            cancel_calls.append("A")
            cancel_entered.set()
            self.assertTrue(allow_cancel.wait(2.0))
            manager.move_base.cancel_count += 1

        handle_a = _GoalHandle(manager.move_base, on_cancel=cancel_a)
        manager.move_base.gh = handle_a
        manager.move_base_goal_generation = 1
        manager.move_base_goal_pending = True
        manager.move_base_goal_handle = handle_a
        manager.move_base_goal_terminal_state = COVERAGE_MANAGER.GoalStatus.ACTIVE
        manager.move_base.state = COVERAGE_MANAGER.GoalStatus.PREEMPTED

        planner_calls = []
        teb_calls = []
        manager._set_enforced_path = (
            lambda path, coverage_active=True:
            planner_calls.append((path.plan_id, coverage_active)) or True
        )
        manager._restore_teb = lambda: teb_calls.append("restore") or True

        responses = {}

        def cancel_exact(name):
            responses[name] = manager._cancel_batch_service(SimpleNamespace(
                batch_id=batch_a
            ))

        first = threading.Thread(target=cancel_exact, args=("first",))
        second = threading.Thread(target=cancel_exact, args=("second",))
        first.start()
        self.assertTrue(cancel_entered.wait(1.0))
        second.start()
        second.join(1.0)
        self.assertFalse(second.is_alive())
        self.assertFalse(responses["second"].success)
        self.assertIn("already in progress", responses["second"].message)

        item = self._batch_region("new")
        new_request = SimpleNamespace(
            client_request_id=batch_b,
            regions=[item],
            operation_width_m=1.0,
            overlap_ratio=0.15,
            allow_reverse_transit=True,
            max_speed_mps=0.3,
            map_digest="test-map",
        )
        blocked_start = manager._start_batch_service(new_request)
        self.assertFalse(blocked_start.accepted)
        self.assertIn("cleanup is still in progress", blocked_start.message)
        self.assertNotIn(batch_b, manager.batch_request_records)

        allow_cancel.set()
        first.join(2.0)
        self.assertFalse(first.is_alive())
        self.assertTrue(responses["first"].success)
        self.assertEqual(["A"], cancel_calls)
        self.assertEqual(1, manager.move_base.cancel_count)
        self.assertEqual([("test-plan", False)], planner_calls)
        self.assertEqual(["restore"], teb_calls)
        self.assertEqual("", manager.batch_id)
        self.assertEqual("TERMINAL", manager.batch_request_records[batch_a]["state"])

        # Model a later B goal.  No delayed A cleanup callback remains that can
        # cancel its handle or disarm its planner/TEB settings.
        handle_b = _GoalHandle(manager.move_base)
        manager.move_base.gh = handle_b
        manager.move_base_goal_generation = 2
        manager.move_base_goal_pending = True
        manager.move_base_goal_handle = handle_b
        self.assertEqual(["A"], cancel_calls)
        self.assertEqual(1, manager.move_base.cancel_count)
        self.assertEqual(1, len(planner_calls))
        self.assertEqual(1, len(teb_calls))

    def test_stale_expected_goal_identity_cannot_cancel_new_generation(self):
        manager, _executed = self._manager([], [])
        handle_a = _GoalHandle(manager.move_base)
        handle_b = _GoalHandle(manager.move_base)
        manager.move_base.gh = handle_b
        manager.move_base_goal_generation = 2
        manager.move_base_goal_pending = True
        manager.move_base_goal_handle = handle_b
        manager.navigation_owner_releasing = False
        manager.state = "SWEEPING"
        manager.detail = "new batch B"

        success, detail = (
            manager._request_exact_move_base_cancel_or_retain_owner(
                "stale A cancellation",
                expected_generation=1,
                expected_handle=handle_a,
            )
        )

        self.assertFalse(success)
        self.assertIn("generation changed", detail)
        self.assertEqual(0, manager.move_base.cancel_count)
        self.assertIs(handle_b, manager.move_base_goal_handle)
        self.assertFalse(manager.navigation_owner_releasing)
        self.assertEqual("SWEEPING", manager.state)
        self.assertEqual("new batch B", manager.detail)

    def test_batch_request_size_and_identity_limits_are_fail_closed(self):
        item = self._batch_region("one")
        self.assertIn(
            "100 regions",
            COVERAGE_MANAGER.CoverageManager._validate_batch_regions(
                [item] * 101
            ),
        )
        item.name = ""
        self.assertIn(
            "empty name",
            COVERAGE_MANAGER.CoverageManager._validate_batch_regions([item]),
        )
        item = self._batch_region("one")
        item.region.polygon.points = [Point32()] * 4097
        self.assertIn(
            "too many vertices",
            COVERAGE_MANAGER.CoverageManager._validate_batch_regions([item]),
        )

    def test_batch_partial_region_continues_and_public_active_never_drops_between(self):
        manager, prepared, executed = self._batch_manager(
            ["COMPLETED_PARTIAL", "COMPLETED"]
        )
        gap_observations = []
        original_prepare = manager._prepare_batch_region

        def observe_gap(token, item):
            gap_observations.append((
                manager.active,
                manager.active_pub.messages[-1].data,
            ))
            return original_prepare(token, item)

        manager._prepare_batch_region = observe_gap
        self._run_batch_sync(manager)
        self.assertEqual(["one", "two"], prepared)
        self.assertEqual(["one", "two"], executed)
        self.assertEqual([(False, True), (False, True)], gap_observations)
        self.assertEqual("COMPLETED_PARTIAL", manager.state)
        self.assertEqual(1, manager.batch_partial_regions)
        self.assertEqual(1, manager.batch_completed_regions)
        self.assertEqual([True, False], [
            message.data for message in manager.active_pub.messages
        ])
        self.assertFalse(manager.batch_active)
        self.assertEqual("", manager.navigation_owner_token)
        self.assertFalse(manager.navigation_owner_claimed)

    def test_batch_failed_region_stops_without_preparing_the_next(self):
        manager, prepared, executed = self._batch_manager(["FAILED"])
        self._run_batch_sync(manager)
        self.assertEqual(["one"], prepared)
        self.assertEqual(["one"], executed)
        self.assertEqual("FAILED", manager.state)
        self.assertEqual("one", manager.last_region_id)
        self.assertEqual("FAILED", manager.last_region_state)
        self.assertEqual([True, False], [
            message.data for message in manager.active_pub.messages
        ])

    def test_batch_release_failure_preserves_owner_and_public_latch(self):
        manager, prepared, executed = self._batch_manager([
            "COMPLETED", "COMPLETED"
        ])
        owner_token = manager.navigation_owner_token
        releases = []

        def reject_release(claim, token):
            releases.append((claim, token, manager.lock._is_owned()))
            return False, "bridge timeout"

        manager._set_navigation_owner = reject_release
        self._run_batch_sync(manager)
        self.assertEqual(["one", "two"], prepared)
        self.assertEqual(["one", "two"], executed)
        self.assertEqual([(False, owner_token, False)], releases)
        self.assertEqual("FAILED", manager.state)
        self.assertEqual("FINALIZING", manager.batch_phase)
        self.assertTrue(manager.batch_active)
        self.assertTrue(manager.active)
        self.assertTrue(manager.navigation_owner_releasing)
        self.assertEqual(owner_token, manager.navigation_owner_token)
        self.assertEqual([True], [
            message.data for message in manager.active_pub.messages
        ])

    def test_batch_cleanup_failure_never_calls_owner_release(self):
        for failure in ("planner", "teb"):
            with self.subTest(failure=failure):
                manager, _prepared, _executed = self._batch_manager([
                    "COMPLETED", "COMPLETED"
                ])
                if failure == "planner":
                    manager._set_enforced_path = (
                        lambda _path, coverage_active=True: False
                    )
                else:
                    manager._restore_teb = lambda: False
                owner_calls = []
                manager._set_navigation_owner = (
                    lambda claim, token:
                    owner_calls.append((claim, token)) or (True, "released")
                )
                self._run_batch_sync(manager)
                self.assertEqual([], owner_calls)
                self.assertEqual("FAILED", manager.state)
                self.assertTrue(manager.batch_active)
                self.assertTrue(manager.active)
                self.assertTrue(manager.navigation_owner_releasing)
                self.assertEqual([True], [
                    message.data for message in manager.active_pub.messages
                ])

    def test_skip_current_cancels_only_that_region_and_continues(self):
        manager, prepared, executed = self._batch_manager(["COMPLETED"])
        original_prepare = manager._prepare_batch_region

        def skip_first(token, item):
            if item.id == "one":
                prepared.append(item.id)
                response = manager._skip_current_service(SimpleNamespace())
                self.assertTrue(response.success)
                return "CANCELED", None, None
            return original_prepare(token, item)

        manager._prepare_batch_region = skip_first
        self._run_batch_sync(manager)
        self.assertEqual(["one", "two"], prepared)
        self.assertEqual(["two"], executed)
        self.assertEqual(1, manager.batch_skipped_regions)
        self.assertEqual(1, manager.batch_completed_regions)
        self.assertEqual("COMPLETED_PARTIAL", manager.state)
        self.assertFalse(manager.batch_cancel_requested)

    def test_cancel_service_cancels_the_whole_batch_without_advancing(self):
        manager, prepared, executed = self._batch_manager([])

        def cancel_first(_token, item):
            prepared.append(item.id)
            response = manager._cancel_service(SimpleNamespace())
            self.assertTrue(response.success)
            return "CANCELED", None, None

        manager._prepare_batch_region = cancel_first
        self._run_batch_sync(manager)
        self.assertEqual(["one"], prepared)
        self.assertEqual([], executed)
        self.assertEqual("CANCELED", manager.state)
        self.assertFalse(manager.batch_active)
        self.assertFalse(manager.batch_cancel_requested)
        self.assertEqual(0, manager.batch_completed_regions)
        self.assertEqual([True, False], [
            message.data for message in manager.active_pub.messages
        ])

    def test_map_change_requests_immediate_whole_batch_cancel(self):
        manager, _prepared, _executed = self._batch_manager([])
        manager.map_digest = "old-map"
        message = COVERAGE_MANAGER.OccupancyGrid()
        message.header.frame_id = "map"
        message.info.width = 1
        message.info.height = 1
        message.info.resolution = 0.1
        message.info.origin.orientation.w = 1.0
        message.data = [0]
        with mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            manager._map_callback(message)
        self.assertNotEqual("old-map", manager.map_digest)
        self.assertTrue(manager.batch_cancel_requested)
        self.assertTrue(manager.cancel_requested)
        self.assertTrue(manager.batch_wake_event.is_set())
        self.assertEqual(0, manager.move_base.cancel_all_count)
        self.assertEqual(0, manager.move_base.cancel_count)
        # Map callback must not release the public ownership latch before the
        # batch worker's one terminal finalizer.
        self.assertEqual([True], [
            item.data for item in manager.active_pub.messages
        ])

    def test_cancel_skip_and_internal_cancel_never_use_wildcard_action_cancel(self):
        operations = (
            ("cancel service", lambda manager: manager._cancel_service(
                SimpleNamespace()
            )),
            ("skip service", lambda manager: manager._skip_current_service(
                SimpleNamespace()
            )),
            ("internal cancel", lambda manager: manager._request_cancel(
                "test safety cancellation"
            )),
        )
        for name, operation in operations:
            with self.subTest(name=name):
                manager, _prepared, _executed = self._batch_manager([])
                manager.move_base.gh = _GoalHandle(manager.move_base)
                manager.move_base_goal_generation = 1
                manager.move_base_goal_pending = True
                manager.move_base_goal_handle = manager.move_base.gh
                result = operation(manager)
                if hasattr(result, "success"):
                    self.assertTrue(result.success)
                else:
                    self.assertTrue(result[0])
                self.assertEqual(1, manager.move_base.cancel_count)
                self.assertEqual(0, manager.move_base.cancel_all_count)

    def test_unproven_goal_handle_retains_owner_without_wildcard_cancel(self):
        manager, _prepared, _executed = self._batch_manager([])
        manager.move_base.gh = _GoalHandle(manager.move_base)
        manager.move_base_goal_generation = 1
        manager.move_base_goal_pending = True
        manager.move_base_goal_handle = None
        owner_token = manager.navigation_owner_token
        response = manager._cancel_service(SimpleNamespace())
        self.assertFalse(response.success)
        self.assertIn("handle is unavailable", response.message)
        self.assertTrue(manager.batch_active)
        self.assertTrue(manager.navigation_owner_releasing)
        self.assertEqual(owner_token, manager.navigation_owner_token)
        self.assertEqual(0, manager.move_base.cancel_count)
        self.assertEqual(0, manager.move_base.cancel_all_count)

    def test_batch_gap_rejects_external_single_plan_and_start(self):
        manager, _prepared, _executed = self._batch_manager([])
        manager.active = False
        manager.max_speed_limit = 1.60
        plan_response = manager._plan_service(SimpleNamespace(map_digest="test-map"))
        start_response = manager._start_service(SimpleNamespace(
            plan_id="test-plan", max_speed_mps=0.3
        ))
        self.assertFalse(plan_response.success)
        self.assertIn("batch", plan_response.message)
        self.assertFalse(start_response.accepted)
        self.assertIn("batch", start_response.message)

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
        self.assertEqual(["entry", "sweep"], [item["type"] for item in segments])
        self.assertEqual([0, 0], [item["swath_index"] for item in segments])

    def test_online_hybrid_mode_keeps_each_connector_unsplit_and_uncached(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.online_hybrid_without_precompute = True
        manager.path_sample_spacing = 0.1
        manager._path_for_points = lambda points, _yaw: list(points)
        swaths = [
            SimpleNamespace(
                start=COVERAGE_MANAGER.Point(1.0, 0.0),
                end=COVERAGE_MANAGER.Point(2.0, 0.0),
                length=1.0,
            ),
            SimpleNamespace(
                start=COVERAGE_MANAGER.Point(2.0, 1.0),
                end=COVERAGE_MANAGER.Point(1.0, 1.0),
                length=1.0,
            ),
        ]

        segments = manager._segments(
            swaths, COVERAGE_MANAGER.Point(0.0, 0.0),
            transition_paths=[object(), object()],
        )

        transits = [item for item in segments if item["type"] == "transit"]
        self.assertEqual(2, len(transits))
        self.assertTrue(all(item["path"] is None for item in transits))
        self.assertTrue(
            all(item["transition_path"] is None for item in transits)
        )
        self.assertEqual([1, 1], [
            item["transition_part_count"] for item in transits
        ])
        self.assertEqual([0, 0], [item["gear"] for item in transits])

    def test_hierarchical_mode_uses_navfn_teb_for_every_first_swath_entry(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.online_hybrid_without_precompute = False
        manager.hierarchical_hybrid_on_demand = True
        manager.batch_active = True
        manager.path_sample_spacing = 0.1
        manager._path_for_points = lambda points, _yaw: list(points)
        swaths = [
            SimpleNamespace(
                start=COVERAGE_MANAGER.Point(4.0, 1.0),
                end=COVERAGE_MANAGER.Point(4.0, 3.0),
                length=2.0,
            ),
            SimpleNamespace(
                start=COVERAGE_MANAGER.Point(5.0, 3.0),
                end=COVERAGE_MANAGER.Point(5.0, 1.0),
                length=2.0,
            ),
        ]

        for batch_index in (1, 2):
            manager.batch_current_index = batch_index
            segments = manager._segments(
                swaths, COVERAGE_MANAGER.Point(0.0, 0.0),
                transition_paths=[None, None],
            )

            self.assertEqual(
                ["entry", "sweep", "rolling_transit", "sweep"],
                [item["type"] for item in segments],
            )
            entry = segments[0]
            self.assertIsNone(entry["path"])
            self.assertNotIn("ordinary_topology_entry", entry)
            self.assertNotIn("force_rolling_topology", entry)
            self.assertNotIn("transition_goal", entry)
            self.assertNotIn("transition_goal_yaw", entry)
            self.assertIsNone(segments[2]["path"])
            self.assertEqual(5.0, segments[2]["transition_goal"].x)
            self.assertAlmostEqual(
                math.pi / 2.0,
                abs(segments[2]["transition_goal_yaw"]),
            )

    def test_online_route_selection_skips_static_connector_graph_search(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.online_hybrid_without_precompute = True
        manager.minimum_turning_radius = 1.2
        manager.time_search_beam_width = 128
        plan = SimpleNamespace(
            swaths=[],
            score=-1.0,
            estimated_total_time_sec=-1.0,
            estimated_sweep_time_sec=-1.0,
            estimated_transit_time_sec=-1.0,
            estimated_reverse_transitions=-1,
            alternative_plans=[object()],
        )
        planner = mock.Mock()
        planner.plan.return_value = plan
        parameters = COVERAGE_MANAGER.CoverageTimeParameters()

        selected = manager._plan_region_route(
            planner,
            [COVERAGE_MANAGER.Point(0.0, 0.0)],
            1.0,
            0.15,
            COVERAGE_MANAGER.Point(0.0, 0.0),
            COVERAGE_MANAGER.Point(0.0, 0.0),
            0.0,
            parameters,
        )

        self.assertIs(plan, selected)
        self.assertIsNone(planner.plan.call_args.kwargs["time_parameters"])
        self.assertEqual([], selected.alternative_plans)
        self.assertEqual(0.0, selected.estimated_total_time_sec)

    def test_hierarchical_route_selection_skips_static_connector_beam(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.online_hybrid_without_precompute = False
        manager.hierarchical_hybrid_on_demand = True
        manager.minimum_turning_radius = 1.2
        manager.time_search_beam_width = 128
        plan = SimpleNamespace(
            swaths=[], score=-1.0,
            estimated_total_time_sec=-1.0,
            estimated_sweep_time_sec=-1.0,
            estimated_transit_time_sec=-1.0,
            estimated_reverse_transitions=-1,
            alternative_plans=[object()],
        )
        planner = mock.Mock()
        planner.plan.return_value = plan

        selected = manager._plan_region_route(
            planner,
            [COVERAGE_MANAGER.Point(0.0, 0.0)],
            1.0, 0.15,
            COVERAGE_MANAGER.Point(0.0, 0.0),
            COVERAGE_MANAGER.Point(0.0, 0.0),
            0.0,
            COVERAGE_MANAGER.CoverageTimeParameters(),
        )

        self.assertIsNone(planner.plan.call_args.kwargs["time_parameters"])
        self.assertEqual([], selected.alternative_plans)
        self.assertEqual(0.0, selected.estimated_total_time_sec)

    def test_direct_hybrid_route_preserves_adjacent_boustrophedon_rows(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.online_hybrid_without_precompute = False
        manager.hierarchical_hybrid_on_demand = True
        manager.direct_hybrid_to_final_goal = True
        manager.minimum_turning_radius = 1.2
        manager.time_search_beam_width = 64
        original = SimpleNamespace(
            start=COVERAGE_MANAGER.Point(0.0, 0.0),
            end=COVERAGE_MANAGER.Point(4.0, 0.0),
            length=4.0,
        )
        oriented = SimpleNamespace(
            start=COVERAGE_MANAGER.Point(4.0, 0.0),
            end=COVERAGE_MANAGER.Point(0.0, 0.0),
            length=4.0,
        )
        plan = SimpleNamespace(
            swaths=[original], spacing=0.85, score=-1.0,
            estimated_total_time_sec=-1.0,
            estimated_sweep_time_sec=-1.0,
            estimated_transit_time_sec=-1.0,
            estimated_reverse_transitions=-1,
            alternative_plans=[object()],
        )
        planner = mock.Mock()
        planner.plan.return_value = plan
        estimate = SimpleNamespace(
            total_time_sec=7.0,
            sweep_time_sec=5.0,
            transit_time_sec=2.0,
            reverse_transitions=0,
        )

        with mock.patch.object(
            COVERAGE_MANAGER,
            "order_adjacent_boustrophedon",
            return_value=([oriented], estimate),
        ) as reorder:
            selected = manager._plan_region_route(
                planner,
                [COVERAGE_MANAGER.Point(0.0, 0.0)],
                1.0,
                0.15,
                COVERAGE_MANAGER.Point(0.0, 0.0),
                COVERAGE_MANAGER.Point(0.0, 0.0),
                0.0,
                COVERAGE_MANAGER.CoverageTimeParameters(),
            )

        self.assertIsNone(planner.plan.call_args.kwargs["time_parameters"])
        self.assertEqual([oriented], selected.swaths)
        self.assertEqual(7.0, selected.estimated_total_time_sec)
        self.assertTrue(reorder.call_args.kwargs["return_estimate"])

    def test_hybrid_transition_is_split_at_each_signed_gear_cusp(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(0.0, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(1.0, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(0.5, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(0.0, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
        ]

        parts = manager._split_hybrid_transition(path)

        self.assertEqual([1, -1], [gear for _path, gear in parts])
        self.assertEqual([2, 3], [len(part.poses) for part, _gear in parts])
        self.assertAlmostEqual(
            parts[0][0].poses[-1].pose.position.x,
            parts[1][0].poses[0].pose.position.x,
        )
        self.assertAlmostEqual(1.0, manager._hybrid_path_length(parts[0][0]))
        self.assertAlmostEqual(1.0, manager._hybrid_path_length(parts[1][0]))

    def test_direct_connector_splits_every_cusp_into_fixed_gear_actions(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.hybrid_execute_unsplit_cusps = False
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(0.0, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(1.0, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(0.5, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
        ]

        segments = manager._hybrid_transition_segments(
            path,
            2,
            COVERAGE_MANAGER.Point(0.0, 0.0),
            COVERAGE_MANAGER.Point(0.5, 0.0),
            0.0,
        )

        self.assertEqual(2, len(segments))
        self.assertEqual([1, -1], [item["gear"] for item in segments])
        self.assertEqual([0, 1], [
            item["transition_part"] for item in segments
        ])
        self.assertTrue(all(
            item["transition_part_count"] == 2 for item in segments
        ))
        self.assertEqual(
            [True, False],
            [item["check_join_after_completion"] for item in segments],
        )

    @staticmethod
    def _seed_transition_completion_parameters(manager):
        manager.transition_completion_start_gate = 0.60
        manager.transition_completion_cross_track_tolerance = 0.40
        manager.transition_completion_pass_margin = 0.02
        manager.transition_completion_min_progress_ratio = 0.85
        manager.transition_completion_max_sample_step = 0.50
        manager.transition_completion_max_overshoot = 0.60
        manager.transition_tracking_deviation_tolerance = 0.35
        manager.transition_tracking_heading_tolerance = 0.40
        manager.transition_tracking_deviation_confirmation_samples = 3
        manager.transition_completion_confirmation_samples = 2
        manager.entry_yaw_tolerance = 0.436332
        manager.hybrid_cusp_position_tolerance = 0.12
        manager.hybrid_cusp_yaw_tolerance = 0.15

    def test_transition_projection_uses_yaw_at_nearby_cusp_edges(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(0.0, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(1.0, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(1.0, 0.1), math.pi,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(0.0, 0.1), math.pi,
                          COVERAGE_MANAGER.rospy.Time(0)),
        ]
        segment = {
            "type": "transit",
            "path": path,
            "end": COVERAGE_MANAGER.Point(0.0, 0.1),
            "yaw": math.pi,
        }

        geometry = manager._transition_completion_geometry(
            segment, COVERAGE_MANAGER.Point(0.5, 0.06), 0.0
        )

        # The later edge is two centimetres closer but points the opposite
        # way. The live body yaw must keep projection on the true first edge.
        self.assertAlmostEqual(0.50, geometry["progress"], places=6)
        self.assertAlmostEqual(0.06, geometry["distance_to_path"], places=6)
        self.assertAlmostEqual(0.0, geometry["path_heading_error"], places=6)

    def test_uturn_transition_arms_from_local_path_not_terminal_yaw(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        self._seed_transition_completion_parameters(manager)
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(0.0, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(1.0, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(1.0, 1.0), math.pi / 2.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(0.0, 1.0), math.pi,
                          COVERAGE_MANAGER.rospy.Time(0)),
        ]
        segment = {
            "type": "transit",
            "path": path,
            "transition_part": 0,
            "transition_part_count": 1,
        }
        tracker = manager._new_transition_completion_tracker()

        complete, geometry = manager._observe_transition_completion(
            segment, tracker, COVERAGE_MANAGER.Point(0.0, 0.0), 0.0
        )

        self.assertFalse(complete)
        self.assertTrue(tracker["armed"])
        self.assertGreater(geometry["heading_error"], 3.0)
        self.assertLess(geometry["path_heading_error"], 1.0e-6)

    def test_persistent_hybrid_path_deviation_requests_immediate_replan(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        self._seed_transition_completion_parameters(manager)
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(x, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0))
            for x in (0.0, 0.5, 1.0)
        ]
        segment = {
            "type": "transit",
            "path": path,
            "transition_part": 0,
            "transition_part_count": 1,
        }
        tracker = manager._new_transition_completion_tracker()
        manager._observe_transition_completion(
            segment, tracker, COVERAGE_MANAGER.Point(0.5, 0.0), 0.0
        )

        for sample in range(3):
            complete, geometry = manager._observe_transition_completion(
                segment,
                tracker,
                COVERAGE_MANAGER.Point(0.5, 0.38),
                0.48,
            )
            self.assertFalse(complete)
            self.assertEqual(sample == 2, geometry["requires_path_replan"])

        self.assertFalse(geometry["invalidated"])
        self.assertAlmostEqual(0.38, geometry["distance_to_path"])
        self.assertAlmostEqual(0.48, geometry["path_heading_error"])

    def test_hybrid_path_deviation_counter_resets_inside_tracking_corridor(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        self._seed_transition_completion_parameters(manager)
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(x, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0))
            for x in (0.0, 0.5, 1.0)
        ]
        segment = {
            "type": "transit",
            "path": path,
            "transition_part": 0,
            "transition_part_count": 1,
        }
        tracker = manager._new_transition_completion_tracker()
        manager._observe_transition_completion(
            segment, tracker, COVERAGE_MANAGER.Point(0.5, 0.0), 0.0
        )
        for _ in range(2):
            manager._observe_transition_completion(
                segment, tracker, COVERAGE_MANAGER.Point(0.5, 0.36), 0.0
            )
        _, geometry = manager._observe_transition_completion(
            segment, tracker, COVERAGE_MANAGER.Point(0.55, 0.10), 0.10
        )

        self.assertEqual(0, geometry["path_deviation_samples"])
        self.assertFalse(geometry["requires_path_replan"])

    def test_cusp_completion_requires_progress_inside_position_tolerance(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        self._seed_transition_completion_parameters(manager)
        manager.transition_completion_min_progress_ratio = 0.99
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(x, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0))
            for x in (0.0, 0.5, 1.0)
        ]
        segment = {
            "type": "transit",
            "path": path,
            "transition_part": 0,
            "transition_part_count": 2,
        }
        tracker = manager._new_transition_completion_tracker()
        complete = False
        geometry = None
        for x in (0.0, 0.5, 0.89, 0.90):
            complete, geometry = manager._observe_transition_completion(
                segment, tracker, COVERAGE_MANAGER.Point(x, 0.0), 0.0
            )
        self.assertFalse(complete)
        self.assertTrue(geometry["within_goal_pose"])
        self.assertLess(geometry["progress_ratio"], 0.99)
        self.assertLess(geometry["terminal_along"], 0.0)
        for x in (0.99, 0.99):
            complete, geometry = manager._observe_transition_completion(
                segment, tracker, COVERAGE_MANAGER.Point(x, 0.0), 0.0
            )
        self.assertTrue(complete)
        self.assertGreaterEqual(geometry["progress_ratio"], 0.99)

    def test_entry_goal_region_part_uses_relaxed_outer_handoff_tolerance(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        self._seed_transition_completion_parameters(manager)
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(x, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0))
            for x in (0.0, 0.6)
        ]
        segment = {
            "type": "transit",
            "path": path,
            "transition_part": 1,
            "transition_part_count": 2,
            "entry_goal_region": True,
            "entry_handoff_goal": COVERAGE_MANAGER.Point(0.75, 0.0),
            "entry_handoff_yaw": 0.20,
        }
        tracker = manager._new_transition_completion_tracker()
        complete = False
        for x, yaw in ((0.0, 0.0), (0.30, 0.20), (0.30, 0.20)):
            complete, _ = manager._observe_transition_completion(
                segment, tracker, COVERAGE_MANAGER.Point(x, 0.0), yaw
            )
        self.assertFalse(complete)
        for x in (0.40, 0.40):
            complete, geometry = manager._observe_transition_completion(
                segment, tracker, COVERAGE_MANAGER.Point(x, 0.0), 0.20
            )
        self.assertTrue(complete)
        self.assertTrue(geometry["within_goal_pose"])
        self.assertLess(geometry["handoff_position_error"], 0.40)
        self.assertGreater(geometry["heading_error"], 0.15)

    def test_forward_cusp_completion_uses_directed_progress_not_point_radius(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        self._seed_transition_completion_parameters(manager)
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(x, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0))
            for x in (0.0, 0.4, 0.8, 1.0)
        ]
        segment = {
            "type": "transit",
            "path": path,
            "transition_part": 0,
            "transition_part_count": 2,
        }
        tracker = manager._new_transition_completion_tracker()
        complete = False
        geometry = None
        for x in (0.0, 0.4, 0.8, 0.98, 1.03, 1.05):
            complete, geometry = manager._observe_transition_completion(
                segment, tracker, COVERAGE_MANAGER.Point(x, 0.0), 0.0
            )
        self.assertTrue(complete)
        self.assertGreater(geometry["terminal_along"], 0.02)
        self.assertGreater(geometry["progress_ratio"], 0.85)
        self.assertLess(geometry["terminal_cross"], 1.0e-6)

    def test_reverse_cusp_completion_follows_path_order_with_vehicle_yaw(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        self._seed_transition_completion_parameters(manager)
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(x, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0))
            for x in (1.0, 0.8, 0.6, 0.5)
        ]
        segment = {
            "type": "transit",
            "path": path,
            "transition_part": 0,
            "transition_part_count": 2,
        }
        tracker = manager._new_transition_completion_tracker()
        complete = False
        geometry = None
        for x in (1.0, 0.8, 0.6, 0.52, 0.47, 0.45):
            complete, geometry = manager._observe_transition_completion(
                segment, tracker, COVERAGE_MANAGER.Point(x, 0.0), 0.0
            )
        self.assertTrue(complete)
        self.assertGreater(geometry["terminal_along"], 0.02)
        self.assertLess(geometry["heading_error"], 1.0e-6)

    def test_badly_overshot_cusp_requests_complete_transition_replan(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        self._seed_transition_completion_parameters(manager)
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(x, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0))
            for x in (0.0, 0.5, 1.0)
        ]
        segment = {
            "type": "transit",
            "path": path,
            "transition_part": 0,
            "transition_part_count": 2,
        }
        tracker = manager._new_transition_completion_tracker()
        geometry = None
        for x, yaw in (
            (0.0, 0.0), (0.4, 0.0), (0.8, 1.0),
            (1.2, 1.0), (1.4, 1.0), (1.62, 1.0),
        ):
            _, geometry = manager._observe_transition_completion(
                segment, tracker, COVERAGE_MANAGER.Point(x, 0.0), yaw
            )
        self.assertTrue(geometry["requires_replan"])
        self.assertFalse(geometry["invalidated"])

    def test_complete_transition_recovery_replans_to_swath_entry_with_both_gears(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.plan_id = "test-plan"
        manager.active = True
        manager.cancel_requested = False
        manager.hybrid_path_generation = 0
        manager.allow_reverse_transit = True
        manager.hybrid_precompute_service_name = "/hybrid-test"
        manager.hybrid_precompute_timeout_sec = 60.0
        manager.hybrid_recovery_timeout_sec = 3.0
        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(0.0, 0.0), 0.0
        )
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(
                COVERAGE_MANAGER.Point(0.0, 0.0), 0.0,
                COVERAGE_MANAGER.rospy.Time(0),
            ),
            manager._pose(
                COVERAGE_MANAGER.Point(1.0, 0.0), 0.0,
                COVERAGE_MANAGER.rospy.Time(0),
            ),
            manager._pose(
                COVERAGE_MANAGER.Point(0.5, 0.0), 0.0,
                COVERAGE_MANAGER.rospy.Time(0),
            ),
        ]
        result = COVERAGE_MANAGER.HybridTransitionResult()
        result.outcome = result.OUTCOME_SUCCESS
        result.path = path
        result.planned_goal = path.poses[-1]
        result.expansions = 12
        result.estimated_time_sec = 2.5
        result.reverse_distance_m = 0.5
        result.direction_changes = 1
        captured = {}

        def service(**kwargs):
            captured.update(kwargs)
            return SimpleNamespace(
                success=True, results=[result], message="ok"
            )

        manager.hybrid_precompute_client = service
        segment = {
            "type": "transit",
            "swath_index": 1,
            "transition_goal": COVERAGE_MANAGER.Point(0.5, 0.0),
            "transition_goal_yaw": 0.0,
        }
        with mock.patch.object(
            COVERAGE_MANAGER.rospy, "wait_for_service"
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            replanned, detail = manager._replan_remaining_hybrid_transition(
                segment
            )
        self.assertEqual("complete remaining Hybrid transition replanned", detail)
        self.assertEqual([1, -1], [item["gear"] for item in replanned])
        self.assertTrue(captured["transit_profile"].allow_reverse)
        self.assertEqual(3.0, captured["total_timeout_sec"])
        self.assertEqual(3, len(replanned[0]["transition_path"].poses))

    def test_cusp_suffix_rebase_accepts_same_gear_radius_join(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.minimum_turning_radius = 1.35
        manager.hybrid_cusp_join_max_skip = 0.30
        manager.hybrid_cusp_join_chord_tolerance = 0.10
        radius = 1.35
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        for distance in (0.0, 0.1, 0.2, 0.3):
            angle = distance / radius
            path.poses.append(manager._pose(
                COVERAGE_MANAGER.Point(
                    radius * math.sin(angle),
                    radius * (1.0 - math.cos(angle)),
                ),
                angle,
                COVERAGE_MANAGER.rospy.Time(0),
            ))
        segment = {"type": "transit", "path": path, "gear": 1}

        joined, detail = manager._rebase_joinable_hybrid_part(
            segment, COVERAGE_MANAGER.Point(0.0, 0.0), 0.0
        )

        self.assertTrue(joined, detail)
        self.assertEqual(4, len(segment["path"].poses))
        self.assertAlmostEqual(0.0, segment["start"].x)
        self.assertAlmostEqual(0.30, segment["length"], places=3)
        self.assertIn("radius 1.35m", detail)

    def test_cusp_suffix_rebase_rejects_subminimum_radius_join(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.minimum_turning_radius = 1.35
        manager.hybrid_cusp_join_max_skip = 0.30
        manager.hybrid_cusp_join_chord_tolerance = 0.10
        radius = 1.35
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        for distance in (0.0, 0.1, 0.2, 0.3):
            angle = distance / radius
            path.poses.append(manager._pose(
                COVERAGE_MANAGER.Point(
                    radius * math.sin(angle),
                    radius * (1.0 - math.cos(angle)),
                ),
                angle,
                COVERAGE_MANAGER.rospy.Time(0),
            ))
        segment = {"type": "transit", "path": path, "gear": 1}

        joined, detail = manager._rebase_joinable_hybrid_part(
            segment, COVERAGE_MANAGER.Point(0.0, 0.0), -0.18
        )

        self.assertFalse(joined)
        self.assertIn("below 1.35m", detail)

    def test_hybrid_transition_rejects_an_in_place_terminal_append(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(0.0, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(1.0, 0.0), 0.0,
                          COVERAGE_MANAGER.rospy.Time(0)),
            manager._pose(COVERAGE_MANAGER.Point(1.0, 0.0), 0.5,
                          COVERAGE_MANAGER.rospy.Time(0)),
        ]
        with self.assertRaisesRegex(ValueError, "in-place rotation"):
            manager._split_hybrid_transition(path)

    def test_hybrid_precompute_includes_region_entry_and_rescores_routes(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.cancel_requested = False
        manager.minimum_turning_radius = 1.35
        manager.hybrid_precompute_service_name = (
            "/move_base/CoverageGlobalPlanner/precompute_transitions"
        )
        manager.hybrid_precompute_timeout_sec = 60.0
        manager.hybrid_precompute_retry_sec = 5.0
        manager._transit_profile = lambda: COVERAGE_MANAGER.TransitProfile()
        manager._lifecycle_wait = lambda _timeout: None

        first = SimpleNamespace(
            start=COVERAGE_MANAGER.Point(0.0, 0.0),
            end=COVERAGE_MANAGER.Point(3.0, 0.0),
            length=3.0,
        )
        second = SimpleNamespace(
            start=COVERAGE_MANAGER.Point(3.0, 1.0),
            end=COVERAGE_MANAGER.Point(0.0, 1.0),
            length=3.0,
        )

        def candidate(index):
            return SimpleNamespace(
                swaths=[copy.deepcopy(first), copy.deepcopy(second)],
                unreachable_area=0.0,
                estimated_total_time_sec=20.0 + index,
                estimated_sweep_time_sec=8.0,
                estimated_transit_time_sec=12.0 + index,
                estimated_reverse_transitions=0,
                score=20.0 + index,
                angle=0.1 * index,
                alternative_plans=[],
            )

        candidates = [candidate(0), candidate(1)]
        plan = candidates[0]
        plan.alternative_plans = candidates
        requests_seen = []

        def precompute(**request):
            results = []
            for transition in request["transitions"]:
                requests_seen.append((
                    int(transition.candidate_index),
                    int(transition.transition_index),
                ))
                result = COVERAGE_MANAGER.HybridTransitionResult()
                result.candidate_index = transition.candidate_index
                result.transition_index = transition.transition_index
                result.outcome = (
                    COVERAGE_MANAGER.HybridTransitionResult.OUTCOME_SUCCESS
                )
                result.path.header.frame_id = "map"
                result.path.poses = [transition.start, transition.goal]
                # Candidate 1 is slightly worse in the cheap proxy, but its
                # actual curvature/collision-checked connector is much faster.
                result.estimated_time_sec = (
                    40.0 if transition.candidate_index == 0 else 1.0
                )
                result.expansions = (
                    40000 if transition.candidate_index == 0 else 40
                )
                results.append(result)
            return SimpleNamespace(
                success=True, results=results, message="ok"
            )

        manager.hybrid_precompute_client = precompute
        planner = SimpleNamespace(
            connector_distance=lambda start, end: math.hypot(
                end.x - start.x, end.y - start.y
            )
        )
        parameters = COVERAGE_MANAGER.CoverageTimeParameters(
            max_forward_speed_mps=0.8,
            max_reverse_speed_mps=0.3,
            max_angular_speed_rps=0.6,
            linear_accel_mps2=1.0,
            angular_accel_rps2=0.5,
            allow_reverse=True,
            direction_change_penalty_sec=1.0,
            segment_handoff_penalty_sec=0.5,
        )
        stamp = COVERAGE_MANAGER.rospy.Time(100.0)
        with mock.patch.object(
            COVERAGE_MANAGER.rospy, "wait_for_service"
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy, "is_shutdown", return_value=False
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy.Time, "now", return_value=stamp
        ):
            selected = manager._precompute_hybrid_candidates(
                plan,
                COVERAGE_MANAGER.Point(-2.0, 0.0),
                0.0,
                parameters,
                planner,
                "test-plan",
            )

        # Transition zero is the vehicle-to-region entry and is planned with
        # the same Ackermann Hybrid model as each row-to-row connector.
        self.assertEqual([(0, 0), (0, 1), (1, 0), (1, 1)], requests_seen)
        self.assertIs(selected, candidates[1])
        self.assertEqual(2, len(selected.transition_paths[0].poses))
        self.assertEqual(2, len(selected.transition_paths[1].poses))
        self.assertEqual(80, selected.hybrid_precompute_expansions)

    def test_transit_arms_hybrid_planner_for_close_heading_mismatch(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_move_base_goal_tracking(manager)
        manager.plan_id = "test-plan"
        manager.allow_reverse_transit = True
        manager.reverse_transit_speed = 0.3
        manager.entry_position_tolerance = 0.2
        manager.entry_yaw_tolerance = 0.2
        manager.goal_timeout_base_sec = 20.0
        manager.goal_timeout_per_meter_sec = 20.0
        manager.cancel_requested = False
        manager.active = True
        manager.navigation_owner_token = "coverage-{}".format("3" * 32)
        manager.navigation_owner_claimed = True
        manager.navigation_owner_releasing = False
        submission_events = []
        manager._set_navigation_owner = (
            lambda claim, token:
            submission_events.append(("owner", claim, token)) or (True, "ok")
        )
        manager.manual_pause = False
        manager.external_pause = False
        manager.move_base = _SuccessfulMoveBase()
        send_goal = manager.move_base.send_goal
        manager.move_base.send_goal = (
            lambda goal, **kwargs:
            submission_events.append(("goal",)) or send_goal(goal, **kwargs)
        )
        manager.enforced_path_pub = _Publisher()
        manager._wait_while_paused = lambda: True
        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(1.0, 0.0), 1.0
        )
        configured_reverse_speeds = []
        manager._set_teb = (
            lambda speed, straight_tracking=False:
            configured_reverse_speeds.append((speed, straight_tracking)) or True
        )
        handoffs = []
        manager._set_enforced_path = (
            lambda path, coverage_active=True:
            handoffs.append((path, coverage_active)) or True
        )
        manager._pause_for_avoidance_loss = lambda: None
        manager._localization_is_fresh = lambda: True
        cached_path = COVERAGE_MANAGER.Path()
        cached_path.header.frame_id = "map"
        stamp = COVERAGE_MANAGER.rospy.Time(100.0)
        cached_path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(-1.0, 0.0), 0.0, stamp),
            manager._pose(COVERAGE_MANAGER.Point(1.0, 0.0), 0.0, stamp),
        ]
        transit = {
            "type": "transit",
            "swath_index": 0,
            "start": COVERAGE_MANAGER.Point(-1.0, 0.0),
            "end": COVERAGE_MANAGER.Point(1.0, 0.0),
            "yaw": 0.0,
            "length": 2.0,
            "path": cached_path,
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
        self.assertEqual([(0.3, False)], configured_reverse_speeds)
        self.assertEqual(1, len(handoffs))
        enforced, coverage_active = handoffs[0]
        self.assertTrue(coverage_active)
        self.assertFalse(enforced.active)
        self.assertEqual(COVERAGE_MANAGER.EnforcedPath.MODE_HYBRID_TRANSIT,
                         enforced.planner_mode)
        self.assertEqual(2, len(enforced.path.poses))
        self.assertEqual(1, len(manager.move_base.goals))
        self.assertEqual([
            ("owner", True, manager.navigation_owner_token),
            ("goal",),
        ], submission_events)

    def test_first_entry_arms_navfn_teb_with_reverse_immediately(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_move_base_goal_tracking(manager)
        manager.plan_id = "test-plan"
        manager.allow_reverse_transit = True
        manager.reverse_transit_speed = 0.3
        manager.entry_position_tolerance = 0.40
        manager.entry_yaw_tolerance = 0.436332
        manager.goal_timeout_base_sec = 20.0
        manager.goal_timeout_per_meter_sec = 20.0
        manager.cancel_requested = False
        manager.active = True
        manager.navigation_owner_token = "coverage-{}".format("b" * 32)
        manager.navigation_owner_claimed = True
        manager.navigation_owner_releasing = False
        manager.manual_pause = False
        manager.external_pause = False
        manager.move_base = _SuccessfulMoveBase()
        manager.enforced_path_pub = _Publisher()
        manager._wait_while_paused = lambda: True
        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(-1.0, 0.0), math.pi
        )
        configured_reverse_speeds = []
        manager._set_teb = (
            lambda speed, straight_tracking=False:
            configured_reverse_speeds.append((speed, straight_tracking)) or True
        )
        handoffs = []
        manager._set_enforced_path = (
            lambda path, coverage_active=True:
            handoffs.append((path, coverage_active)) or True
        )
        manager._set_navigation_owner = lambda _claim, _token: (True, "ok")
        manager._pause_for_avoidance_loss = lambda: None
        manager._localization_is_fresh = lambda: True
        entry = {
            "type": "entry",
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
            result = manager._execute_segment(entry, 0)

        self.assertEqual("succeeded", result)
        self.assertEqual([(0.3, False)], configured_reverse_speeds)
        self.assertEqual(1, len(handoffs))
        self.assertEqual(COVERAGE_MANAGER.EnforcedPath.MODE_COVERAGE_NAVFN,
                         handoffs[0][0].planner_mode)
        self.assertEqual(1, len(manager.move_base.goals))

    def test_transit_accepts_a_moderate_ackermann_entry_error(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_move_base_goal_tracking(manager)
        manager.plan_id = "test-plan"
        manager.cancel_requested = False
        manager.active = True
        manager.navigation_owner_token = "coverage-{}".format("a" * 32)
        manager.navigation_owner_releasing = False
        manager.entry_position_tolerance = 0.30
        manager.entry_yaw_tolerance = 0.40
        manager._wait_while_paused = lambda: True
        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(0.28, 0.0), 0.35
        )
        manager._set_teb = mock.Mock(return_value=True)
        transit = {
            "type": "transit",
            "swath_index": 0,
            "start": COVERAGE_MANAGER.Point(1.0, 0.0),
            "end": COVERAGE_MANAGER.Point(0.0, 0.0),
            "yaw": 0.0,
            "length": 1.0,
            "path": None,
        }

        result = manager._execute_segment(transit, 0)

        self.assertEqual("succeeded", result)
        manager._set_teb.assert_not_called()

    def test_sweep_arms_only_its_exact_coverage_path(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_move_base_goal_tracking(manager)
        manager.plan_id = "test-plan"
        manager.allow_reverse_transit = True
        manager.reverse_transit_speed = 0.3
        manager.goal_timeout_base_sec = 20.0
        manager.goal_timeout_per_meter_sec = 20.0
        manager.cancel_requested = False
        manager.active = True
        manager.navigation_owner_token = "coverage-{}".format("4" * 32)
        manager.navigation_owner_claimed = True
        manager.navigation_owner_releasing = False
        manager._set_navigation_owner = lambda _claim, _token: (True, "ok")
        manager.manual_pause = False
        manager.external_pause = False
        manager.last_tracked_point = COVERAGE_MANAGER.Point(0.0, 0.0)
        manager.move_base = _SuccessfulMoveBase()
        manager.enforced_path_pub = _Publisher()
        manager._wait_while_paused = lambda: True
        configured_reverse_speeds = []
        manager._set_teb = (
            lambda speed, straight_tracking=False:
            configured_reverse_speeds.append((speed, straight_tracking)) or True
        )
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
        self.assertEqual([(0.0, True)], configured_reverse_speeds)
        enforced, coverage_active = handoffs[0]
        self.assertTrue(coverage_active)
        self.assertTrue(enforced.active)
        self.assertEqual(2, len(enforced.path.poses))

    def test_sweep_completion_requires_entry_history_exit_plane_and_stop(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.entry_yaw_tolerance = 0.40
        manager.sweep_completion_start_gate = 0.45
        manager.sweep_completion_cross_track_tolerance = 0.30
        manager.sweep_completion_heading_tolerance = 0.35
        manager.sweep_completion_pass_margin = 0.02
        manager.sweep_completion_min_progress_ratio = 0.90
        manager.sweep_completion_max_sample_step = 0.50
        manager.sweep_completion_confirmation_samples = 2
        manager.sweep_completion_max_linear_speed = 0.08
        segment = {
            "start": COVERAGE_MANAGER.Point(0.0, 0.0),
            "end": COVERAGE_MANAGER.Point(4.0, 0.0),
        }

        tracker = manager._new_sweep_completion_tracker()
        for index in range(11):
            completed, _ = manager._observe_sweep_completion(
                segment,
                tracker,
                COVERAGE_MANAGER.Point(0.4 * index, 0.0),
                0.0,
                0.4,
            )
            self.assertFalse(completed)
        # Passing the exit while still moving is not a line completion.
        completed, geometry = manager._observe_sweep_completion(
            segment, tracker, COVERAGE_MANAGER.Point(4.04, 0.0), 0.0, 0.20
        )
        self.assertFalse(completed)
        self.assertLess(geometry["exit_dot"], 0.0)
        # Two fresh stationary samples beyond the directed exit plane confirm
        # the stop without requiring the pose to reverse into a point circle.
        completed, _ = manager._observe_sweep_completion(
            segment, tracker, COVERAGE_MANAGER.Point(4.04, 0.0), 0.0, 0.0
        )
        self.assertFalse(completed)
        completed, geometry = manager._observe_sweep_completion(
            segment, tracker, COVERAGE_MANAGER.Point(4.04, 0.0), 0.0, 0.0
        )
        self.assertTrue(completed)
        self.assertGreaterEqual(geometry["progress_ratio"], 1.0)

    def test_sweep_entry_recovery_requires_persistent_early_large_error(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.sweep_entry_recovery_enabled = True
        manager.sweep_entry_recovery_progress_limit = 0.75
        manager.sweep_entry_recovery_confirmation_samples = 3
        manager.sweep_completion_start_gate = 0.45
        manager.entry_yaw_tolerance = 0.40
        segment = {
            "start": COVERAGE_MANAGER.Point(0.0, 0.0),
            "end": COVERAGE_MANAGER.Point(4.0, 0.0),
        }
        tracker = manager._new_sweep_completion_tracker()

        # A small, capturable entry error remains owned by straight tracking.
        for _ in range(4):
            recover, geometry = manager._observe_sweep_entry_recovery(
                segment, tracker, COVERAGE_MANAGER.Point(0.0, 0.40), 0.20
            )
            self.assertFalse(recover)
            self.assertFalse(geometry["outside_entry_acquisition"])

        # A deterministic 0.55 m lateral disturbance must persist for three
        # samples before the local action is canceled and replanned.
        for sample in range(3):
            recover, geometry = manager._observe_sweep_entry_recovery(
                segment, tracker, COVERAGE_MANAGER.Point(0.0, 0.55), 0.48
            )
            self.assertEqual(sample == 2, recover)
        self.assertAlmostEqual(0.55, geometry["start_distance"])

        # Normal on-line progress may be farther than the acquisition radius
        # from the first point and still must not be treated as a deviation.
        recover, _ = manager._observe_sweep_entry_recovery(
            segment, tracker, COVERAGE_MANAGER.Point(0.65, 0.0), 0.0
        )
        self.assertFalse(recover)
        self.assertEqual(0, tracker["entry_deviation_samples"])

        # Once meaningful line progress has begun, even a later cross-track
        # error is owned by normal sweep retry/completion logic.
        recover, _ = manager._observe_sweep_entry_recovery(
            segment, tracker, COVERAGE_MANAGER.Point(1.0, 0.55), 0.48
        )
        self.assertFalse(recover)

    def test_disturbed_sweep_entry_recovery_targets_unchanged_swath_start(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.hierarchical_hybrid_on_demand = True
        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(0.2, 0.5), 0.4
        )
        captured = []
        manager._execute_rolling_hybrid_transition = (
            lambda segment, index: captured.append((segment, index))
            or "succeeded"
        )
        sweep = {
            "type": "sweep",
            "swath_index": 2,
            "start": COVERAGE_MANAGER.Point(0.0, 0.0),
            "end": COVERAGE_MANAGER.Point(4.0, 0.0),
            "yaw": 0.0,
            "length": 4.0,
        }

        self.assertEqual(
            "succeeded", manager._execute_sweep_entry_recovery(sweep, 5)
        )
        recovery, index = captured[0]
        self.assertEqual("rolling_transit", recovery["type"])
        self.assertEqual(5, index)
        self.assertAlmostEqual(0.0, recovery["transition_goal"].x)
        self.assertAlmostEqual(0.0, recovery["transition_goal"].y)
        self.assertAlmostEqual(0.0, recovery["transition_goal_yaw"])
        self.assertTrue(recovery["entry_recovery"])
        self.assertEqual("TRANSITING", manager.state)

    def test_rolling_transition_replans_full_goal_after_measured_cusp(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.entry_position_tolerance = 0.30
        manager.entry_yaw_tolerance = 0.40
        manager.transition_completion_cross_track_tolerance = 0.30
        manager.hybrid_rolling_max_chunks = 4
        manager.hybrid_no_progress_distance = 0.10
        manager.direct_hybrid_to_final_goal = True
        segment = {
            "type": "rolling_transit",
            "swath_index": 2,
            "transition_goal": COVERAGE_MANAGER.Point(10.0, 0.0),
            "transition_goal_yaw": 0.0,
        }
        first_part = {"name": "old-first"}
        stale_suffix = {"name": "old-stale-suffix"}
        live_replan = {"name": "live-full-goal"}
        manager._current_pose = mock.Mock(side_effect=[
            (COVERAGE_MANAGER.Point(0.0, 0.0), 0.0),
            (COVERAGE_MANAGER.Point(1.0, 0.0), 0.1),
            (COVERAGE_MANAGER.Point(1.0, 0.0), 0.1),
        ])
        manager._plan_next_rolling_hybrid_chunk = mock.Mock(side_effect=[
            ([first_part, stale_suffix], True, "planned"),
            ([live_replan], True, "planned"),
        ])
        manager._publish_hybrid_transition_path = mock.Mock()
        manager._execute_segment = mock.Mock(return_value="succeeded")
        manager._rebase_joinable_hybrid_part = mock.Mock(
            return_value=(False, "radius below limit")
        )

        result = manager._execute_rolling_hybrid_transition(segment, 5)

        self.assertEqual("succeeded", result)
        self.assertEqual(2, manager._plan_next_rolling_hybrid_chunk.call_count)
        self.assertEqual(
            [mock.call(first_part, 5), mock.call(live_replan, 5)],
            manager._execute_segment.call_args_list,
        )
        self.assertNotIn(
            mock.call(stale_suffix, 5),
            manager._execute_segment.call_args_list,
        )

    def test_rolling_transition_keeps_joinable_cached_cusp_suffix(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.entry_position_tolerance = 0.30
        manager.entry_yaw_tolerance = 0.40
        manager.transition_completion_cross_track_tolerance = 0.30
        manager.hybrid_rolling_max_chunks = 4
        manager.hybrid_no_progress_distance = 0.10
        manager.direct_hybrid_to_final_goal = True
        segment = {
            "type": "rolling_transit",
            "swath_index": 2,
            "transition_goal": COVERAGE_MANAGER.Point(10.0, 0.0),
            "transition_goal_yaw": 0.0,
        }
        first_part = {"name": "first"}
        cached_suffix = {"name": "checked-suffix"}
        manager._current_pose = mock.Mock(side_effect=[
            (COVERAGE_MANAGER.Point(0.0, 0.0), 0.0),
            (COVERAGE_MANAGER.Point(1.0, 0.0), 0.1),
        ])
        manager._plan_next_rolling_hybrid_chunk = mock.Mock(
            return_value=([first_part, cached_suffix], True, "planned")
        )
        manager._publish_hybrid_transition_path = mock.Mock()
        manager._execute_segment = mock.Mock(return_value="succeeded")
        manager._rebase_joinable_hybrid_part = mock.Mock(
            return_value=(True, "valid tangent bridge")
        )

        result = manager._execute_rolling_hybrid_transition(segment, 5)

        self.assertEqual("succeeded", result)
        self.assertEqual(1, manager._plan_next_rolling_hybrid_chunk.call_count)
        self.assertEqual(
            [mock.call(first_part, 5), mock.call(cached_suffix, 5)],
            manager._execute_segment.call_args_list,
        )

    def test_rolling_transition_accepts_final_entry_after_nonfinal_part(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.entry_position_tolerance = 0.40
        manager.entry_yaw_tolerance = 0.436332
        # The final entry basin is deliberately wider than an intermediate
        # cusp's directed cross-track tolerance.
        manager.transition_completion_cross_track_tolerance = 0.30
        manager.hybrid_rolling_max_chunks = 4
        manager.direct_hybrid_to_final_goal = True
        segment = {
            "type": "rolling_transit",
            "swath_index": 2,
            "transition_goal": COVERAGE_MANAGER.Point(1.0, 0.0),
            "transition_goal_yaw": 0.0,
        }
        first_part = {"name": "first"}
        unnecessary_suffix = {"name": "unnecessary-reverse"}
        manager._current_pose = mock.Mock(side_effect=[
            (COVERAGE_MANAGER.Point(0.0, 0.0), 0.0),
            (COVERAGE_MANAGER.Point(0.70, 0.0), math.radians(15.0)),
        ])
        manager._plan_next_rolling_hybrid_chunk = mock.Mock(
            return_value=([first_part, unnecessary_suffix], True, "planned")
        )
        manager._publish_hybrid_transition_path = mock.Mock()
        manager._execute_segment = mock.Mock(return_value="succeeded")
        manager._wait_for_transition_stop = mock.Mock(return_value=True)
        manager._rebase_joinable_hybrid_part = mock.Mock()

        result = manager._execute_rolling_hybrid_transition(segment, 5)

        self.assertEqual("succeeded", result)
        manager._execute_segment.assert_called_once_with(first_part, 5)
        manager._wait_for_transition_stop.assert_called_once_with()
        manager._rebase_joinable_hybrid_part.assert_not_called()

    def test_sweep_handoff_gate_never_enters_sweeping_outside_tolerance(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.entry_position_tolerance = 0.40
        manager.entry_yaw_tolerance = 0.436332
        manager.sweep_completion_cross_track_tolerance = 0.30
        sweep = {
            "type": "sweep",
            "start": COVERAGE_MANAGER.Point(0.0, 0.0),
            "end": COVERAGE_MANAGER.Point(4.0, 0.0),
            "yaw": 0.0,
        }

        manager.state = "TRANSITING"
        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(0.05, 0.42), 0.48
        )
        self.assertEqual("entry-recovery", manager._prepare_sweep_entry(
            sweep, 1
        ))
        self.assertEqual("TRANSITING", manager.state)
        self.assertEqual(2, manager.current_segment)

        # Entry uses the widened 0.40 m basin without widening the 0.30 m
        # sweep-completion/cusp cross-track contract.
        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(0.0, 0.35), math.radians(15.0)
        )
        self.assertEqual("ready", manager._prepare_sweep_entry(sweep, 1))

        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(0.05, 0.12), 0.20
        )
        self.assertEqual("ready", manager._prepare_sweep_entry(sweep, 1))
        self.assertEqual("TRANSITING", manager.state)

    def test_entry_recovery_planning_cost_favors_short_reverse_alignment(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.allow_reverse_transit = True
        manager.task_max_speed = 0.80
        manager.reverse_transit_speed = 0.30
        manager.task_max_angular_speed = 0.60
        manager.task_linear_accel = 1.00
        manager.task_angular_accel = 0.50
        manager.direction_change_penalty = 1.00
        manager.entry_position_tolerance = 0.30
        manager.entry_yaw_tolerance = 0.40
        manager.transit_replan_period = 1.00
        manager.sweep_entry_recovery_reverse_cost_speed = 0.80
        manager.sweep_entry_recovery_forward_cost_speed = 0.20
        manager.sweep_entry_recovery_direction_change_penalty = 0.15
        manager.sweep_entry_recovery_goal_position_tolerance = 0.30
        manager.sweep_entry_recovery_goal_yaw_tolerance = 0.349066

        normal = manager._transit_profile()
        recovery = manager._entry_recovery_planning_profile()
        self.assertEqual(0.30, normal.max_reverse_speed_mps)
        self.assertEqual(0.20, recovery.max_forward_speed_mps)
        self.assertEqual(0.80, recovery.max_reverse_speed_mps)
        self.assertEqual(0.15, recovery.direction_change_penalty_sec)
        self.assertEqual(0.30, recovery.goal_position_tolerance_m)
        self.assertEqual(0.349066, recovery.goal_yaw_tolerance_rad)
        # Cost shaping must not alter the physical reverse command limit.
        self.assertEqual(0.30, manager.reverse_transit_speed)

    def test_sweep_completion_rejects_false_exit_plane_crossings(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.entry_yaw_tolerance = 0.40
        segment = {
            "start": COVERAGE_MANAGER.Point(0.0, 0.0),
            "end": COVERAGE_MANAGER.Point(4.0, 0.0),
        }

        # A task that first appears beyond the exit has no evidence that this
        # particular swath was traversed from its entrance.
        tracker = manager._new_sweep_completion_tracker()
        for _ in range(3):
            completed, _ = manager._observe_sweep_completion(
                segment,
                tracker,
                COVERAGE_MANAGER.Point(4.10, 0.0),
                0.0,
                0.0,
            )
            self.assertFalse(completed)
        self.assertFalse(tracker["armed"])

        # One discontinuous localization sample cannot manufacture progress.
        tracker = manager._new_sweep_completion_tracker()
        manager._observe_sweep_completion(
            segment, tracker, COVERAGE_MANAGER.Point(0.0, 0.0), 0.0, 0.0
        )
        completed, geometry = manager._observe_sweep_completion(
            segment, tracker, COVERAGE_MANAGER.Point(1.0, 0.0), 0.0, 0.0
        )
        self.assertFalse(completed)
        self.assertTrue(geometry["invalidated"])
        completed, _ = manager._observe_sweep_completion(
            segment, tracker, COVERAGE_MANAGER.Point(4.10, 0.0), 0.0, 0.0
        )
        self.assertFalse(completed)

        # Traversing the line correctly but ending beside it or with the wrong
        # heading also cannot satisfy the directed-line completion gate.
        for point, yaw in (
            (COVERAGE_MANAGER.Point(4.05, 0.41), 0.0),
            (COVERAGE_MANAGER.Point(4.05, 0.0), math.pi / 2.0),
        ):
            tracker = manager._new_sweep_completion_tracker()
            for index in range(11):
                manager._observe_sweep_completion(
                    segment,
                    tracker,
                    COVERAGE_MANAGER.Point(0.4 * index, 0.0),
                    0.0,
                    0.0,
                )
            for _ in range(3):
                completed, _ = manager._observe_sweep_completion(
                    segment, tracker, point, yaw, 0.0
                )
                self.assertFalse(completed)

    def test_execute_sweep_accepts_stationary_directed_exit_crossing(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_move_base_goal_tracking(manager)
        manager.plan_id = "test-plan"
        manager.allow_reverse_transit = True
        manager.reverse_transit_speed = 0.3
        manager.entry_yaw_tolerance = 0.40
        manager.goal_timeout_base_sec = 20.0
        manager.goal_timeout_per_meter_sec = 20.0
        manager.cancel_requested = False
        manager.active = True
        manager.navigation_owner_token = "coverage-{}".format("e" * 32)
        manager.navigation_owner_claimed = True
        manager.navigation_owner_releasing = False
        manager.manual_pause = False
        manager.external_pause = False
        manager.last_tracked_point = None
        manager.sweep_completion_start_gate = 0.45
        manager.sweep_completion_cross_track_tolerance = 0.30
        manager.sweep_completion_heading_tolerance = 0.35
        manager.sweep_completion_pass_margin = 0.02
        manager.sweep_completion_min_progress_ratio = 0.90
        manager.sweep_completion_max_sample_step = 0.50
        manager.sweep_completion_confirmation_samples = 2
        manager.sweep_completion_poll_period = 0.05
        manager.sweep_completion_max_linear_speed = 0.08
        manager.move_base = _PendingUntilCanceledMoveBase()
        manager.enforced_path_pub = _Publisher()
        manager._wait_while_paused = lambda: True
        manager._set_teb = lambda _speed, straight_tracking=False: True
        manager._set_enforced_path = (
            lambda _path, coverage_active=True: True
        )
        manager._set_navigation_owner = lambda _claim, _token: (True, "ok")
        manager._pause_for_avoidance_loss = lambda: None
        manager._localization_is_fresh = lambda: True
        manager._current_chassis_linear_speed = lambda: 0.0
        poses = [
            (COVERAGE_MANAGER.Point(0.4 * index, 0.0), 0.0)
            for index in range(11)
        ] + [
            (COVERAGE_MANAGER.Point(4.04, 0.0), 0.0),
            (COVERAGE_MANAGER.Point(4.04, 0.0), 0.0),
        ]
        pose_index = [0]

        def current_pose():
            index = min(pose_index[0], len(poses) - 1)
            pose_index[0] += 1
            return poses[index]

        manager._current_pose = current_pose
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
        stamp = COVERAGE_MANAGER.rospy.Time(100.0)
        path.poses = [
            manager._pose(COVERAGE_MANAGER.Point(0.0, 0.0), 0.0, stamp),
            manager._pose(COVERAGE_MANAGER.Point(4.0, 0.0), 0.0, stamp),
        ]
        sweep = {
            "type": "sweep",
            "swath_index": 0,
            "start": COVERAGE_MANAGER.Point(0.0, 0.0),
            "end": COVERAGE_MANAGER.Point(4.0, 0.0),
            "yaw": 0.0,
            "length": 4.0,
            "path": path,
        }
        with mock.patch.object(
            COVERAGE_MANAGER.rospy, "is_shutdown", return_value=False
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy.Time, "now", return_value=stamp
        ):
            result = manager._execute_segment(sweep, 1)

        self.assertEqual("succeeded", result)
        self.assertEqual(1, manager.move_base.cancel_count)
        self.assertFalse(manager.move_base_goal_pending)
        self.assertEqual(
            COVERAGE_MANAGER.GoalStatus.PREEMPTED,
            manager.move_base_goal_terminal_state,
        )

    def test_late_segment_claim_is_compensated_after_finalizing_starts(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_move_base_goal_tracking(manager)
        manager.plan_id = "test-plan"
        manager.allow_reverse_transit = True
        manager.reverse_transit_speed = 0.3
        manager.goal_timeout_base_sec = 20.0
        manager.goal_timeout_per_meter_sec = 20.0
        manager.cancel_requested = False
        manager.active = True
        manager.navigation_owner_token = "coverage-{}".format("5" * 32)
        manager.navigation_owner_claimed = True
        manager.navigation_owner_releasing = False
        manager.manual_pause = False
        manager.external_pause = False
        manager.last_tracked_point = None
        manager.move_base = _SuccessfulMoveBase()
        manager.enforced_path_pub = _Publisher()
        manager._wait_while_paused = lambda: True
        manager._set_teb = lambda _speed, straight_tracking=False: True
        manager._set_enforced_path = (
            lambda _path, coverage_active=True: True
        )
        owner_calls = []

        def owner_call(claim, token):
            owner_calls.append((claim, token, manager.lock._is_owned()))
            if claim:
                # Model a finalizer releasing while the cross-node claim was
                # in flight, followed by this delayed claim succeeding.
                with manager.lock:
                    manager.active = False
                    manager.navigation_owner_releasing = True
            return True, "ok"

        manager._set_navigation_owner = owner_call
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
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
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            result = manager._execute_segment(sweep, 0)
        self.assertEqual("canceled", result)
        self.assertEqual([
            (True, manager.navigation_owner_token, False),
            (False, manager.navigation_owner_token, False),
        ], owner_calls)
        self.assertEqual([], manager.move_base.goals)

    def test_skip_during_segment_claim_keeps_the_batch_owner_token(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_move_base_goal_tracking(manager)
        manager.plan_id = "test-plan"
        manager.allow_reverse_transit = True
        manager.reverse_transit_speed = 0.3
        manager.goal_timeout_base_sec = 20.0
        manager.goal_timeout_per_meter_sec = 20.0
        manager.cancel_requested = False
        manager.active = True
        manager.batch_active = True
        manager.batch_cancel_requested = False
        manager.batch_skip_requested = False
        manager.batch_phase = "EXECUTING"
        manager.batch_current_index = 1
        manager.batch_wake_event = threading.Event()
        manager.plan_pending = False
        manager.plan_token = ""
        manager.start_pending = False
        manager.start_token = ""
        manager.detail = "executing"
        manager.navigation_owner_token = "coverage-{}".format("6" * 32)
        manager.navigation_owner_claimed = True
        manager.navigation_owner_releasing = False
        manager.manual_pause = False
        manager.external_pause = False
        manager.last_tracked_point = None
        manager.move_base = _SuccessfulMoveBase()
        manager.enforced_path_pub = _Publisher()
        manager._publish_status = lambda: None
        manager._wait_while_paused = lambda: True
        manager._set_teb = lambda _speed, straight_tracking=False: True
        manager._set_enforced_path = (
            lambda _path, coverage_active=True: True
        )
        owner_calls = []

        def owner_call(claim, token):
            owner_calls.append((claim, token, manager.lock._is_owned()))
            if claim:
                response = manager._skip_current_service(SimpleNamespace())
                self.assertTrue(response.success)
            return True, "ok"

        manager._set_navigation_owner = owner_call
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
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
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            result = manager._execute_segment(sweep, 0)
        self.assertEqual("canceled", result)
        self.assertEqual([
            (True, manager.navigation_owner_token, False),
        ], owner_calls)
        self.assertTrue(manager.batch_skip_requested)
        self.assertEqual("coverage-{}".format("6" * 32),
                         manager.navigation_owner_token)
        self.assertEqual([], manager.move_base.goals)

    def test_previous_unconfirmed_goal_prevents_next_segment_submission(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_move_base_goal_tracking(manager)
        manager.cancel_requested = False
        manager.active = True
        manager.navigation_owner_releasing = False
        manager.move_base_goal_pending = True
        manager._wait_while_paused = lambda: True
        result = manager._execute_segment({}, 1)
        self.assertEqual("failed", result)
        self.assertIn("not safely terminal", manager.detail)

    def test_execute_segment_treats_lost_as_failed_not_retryable_blocked(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_move_base_goal_tracking(manager)
        manager.plan_id = "test-plan"
        manager.allow_reverse_transit = True
        manager.reverse_transit_speed = 0.3
        manager.goal_timeout_base_sec = 20.0
        manager.goal_timeout_per_meter_sec = 20.0
        manager.cancel_requested = False
        manager.active = True
        manager.navigation_owner_token = "coverage-{}".format("9" * 32)
        manager.navigation_owner_claimed = True
        manager.navigation_owner_releasing = False
        manager.manual_pause = False
        manager.external_pause = False
        manager.last_tracked_point = None
        manager.move_base = _SuccessfulMoveBase()
        manager.move_base.get_state = lambda: COVERAGE_MANAGER.GoalStatus.LOST
        manager.enforced_path_pub = _Publisher()
        manager._wait_while_paused = lambda: True
        manager._set_teb = lambda _speed, straight_tracking=False: True
        manager._set_enforced_path = (
            lambda _path, coverage_active=True: True
        )
        manager._set_navigation_owner = (
            lambda _claim, _token: (True, "ok")
        )
        manager._pause_for_avoidance_loss = lambda: None
        manager._localization_is_fresh = lambda: True
        path = COVERAGE_MANAGER.Path()
        path.header.frame_id = "map"
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
            COVERAGE_MANAGER.rospy,
            "is_shutdown",
            return_value=False,
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            result = manager._execute_segment(sweep, 0)
        self.assertEqual("failed", result)
        self.assertTrue(manager.move_base_goal_pending)
        self.assertEqual(COVERAGE_MANAGER.GoalStatus.LOST,
                         manager.move_base_goal_terminal_state)
        self.assertEqual(1, len(manager.move_base.goals))

    def test_pause_can_retry_only_after_exact_goal_preempt_is_confirmed(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_move_base_goal_tracking(manager)
        manager.detail = "executing"
        manager.move_base = _MoveBase()
        manager.move_base.gh = _GoalHandle(manager.move_base)
        manager.move_base.state = COVERAGE_MANAGER.GoalStatus.PREEMPTED
        manager.move_base_goal_generation = 1
        manager.move_base_goal_pending = True
        manager.move_base_goal_handle = manager.move_base.gh
        outcome = manager._cancel_segment_goal(1, "paused")
        self.assertEqual("paused", outcome)
        self.assertEqual(1, manager.move_base.cancel_count)
        self.assertFalse(manager.move_base_goal_pending)
        self.assertEqual(COVERAGE_MANAGER.GoalStatus.PREEMPTED,
                         manager.move_base_goal_terminal_state)

    def test_sweep_teb_profile_is_strict_and_transit_restores_baseline(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.original_teb = None
        manager.task_max_speed = 0.8
        manager.entry_position_tolerance = 0.30
        manager.entry_yaw_tolerance = 0.40
        manager.sweep_viapoint_separation = 0.3
        manager.sweep_weight_viapoint = 50.0
        manager.sweep_weight_viapoint_lateral = 200.0
        manager.sweep_weight_viapoint_heading = 100.0
        manager.sweep_weight_kinematics_forward_drive = 1000.0
        manager.sweep_selection_viapoint_cost_scale = 5.0
        manager.sweep_viapoints_all_candidates = True
        manager.hybrid_transit_viapoint_separation = 0.3
        manager.hybrid_transit_lookahead_distance = 3.0
        manager.hybrid_cusp_position_tolerance = 0.12
        manager.hybrid_cusp_yaw_tolerance = 0.15
        manager.hybrid_transit_min_obstacle_dist = 0.20
        manager.hybrid_transit_inflation_dist = 0.45
        manager.hybrid_transit_weight_viapoint = 8.0
        manager.hybrid_transit_weight_viapoint_lateral = 40.0
        manager.hybrid_transit_weight_viapoint_heading = 20.0
        manager.hybrid_transit_include_costmap_obstacles = False
        manager.hybrid_transit_weight_kinematics_forward_drive = 5.0
        manager.hybrid_transit_selection_viapoint_cost_scale = 1.0
        manager.hybrid_transit_viapoints_all_candidates = False
        manager.hybrid_transit_inner_iterations = 3
        manager.hybrid_transit_outer_iterations = 2
        baseline = {
            "motion_direction_mode": 0,
            "max_vel_x": 0.8,
            "max_vel_x_backwards": 0.3,
            "allow_init_with_backwards_motion": False,
            "xy_goal_tolerance": 0.5,
            "yaw_goal_tolerance": 0.3,
            "global_plan_viapoint_sep": 0.8,
            "weight_viapoint": 8.0,
            "weight_viapoint_lateral": 0.0,
            "weight_viapoint_heading": 0.0,
            "weight_kinematics_forward_drive": 100.0,
            "selection_viapoint_cost_scale": 1.0,
            "viapoints_all_candidates": False,
            "global_plan_overwrite_orientation": True,
            "via_points_ordered": False,
            "max_number_classes": 3,
            "max_global_plan_lookahead_dist": 8.0,
            "min_obstacle_dist": 0.30,
            "inflation_dist": 0.60,
            "include_costmap_obstacles": True,
            "no_inner_iterations": 10,
            "no_outer_iterations": 6,
        }

        client = mock.Mock()
        client.get_configuration.return_value = dict(baseline)
        updates = []
        client.update_configuration.side_effect = (
            lambda configuration: updates.append(dict(configuration))
        )
        with mock.patch(
            "dynamic_reconfigure.client.Client", return_value=client
        ):
            self.assertTrue(manager._set_teb(0.0, straight_tracking=True))
            sweep = updates[-1]
            self.assertEqual(0.0, sweep["max_vel_x_backwards"])
            self.assertEqual(1, sweep["motion_direction_mode"])
            self.assertFalse(sweep["allow_init_with_backwards_motion"])
            self.assertEqual(0.20, sweep["xy_goal_tolerance"])
            self.assertEqual(0.20, sweep["yaw_goal_tolerance"])
            self.assertEqual(0.3, sweep["global_plan_viapoint_sep"])
            self.assertEqual(50.0, sweep["weight_viapoint"])
            self.assertEqual(200.0, sweep["weight_viapoint_lateral"])
            self.assertEqual(100.0, sweep["weight_viapoint_heading"])
            self.assertEqual(1000.0, sweep["weight_kinematics_forward_drive"])
            self.assertEqual(5.0, sweep["selection_viapoint_cost_scale"])
            self.assertTrue(sweep["viapoints_all_candidates"])
            self.assertEqual(1, sweep["max_number_classes"])
            self.assertFalse(sweep["include_costmap_obstacles"])

            manager._teb_hybrid_tracking_requested = False
            manager.entry_navfn_single_topology = True
            self.assertTrue(manager._set_teb(
                0.3, straight_tracking=False, hybrid_tracking=False
            ))
            entry = updates[-1]
            self.assertEqual(1, entry["max_number_classes"])
            self.assertEqual(0, entry["motion_direction_mode"])
            self.assertTrue(entry["include_costmap_obstacles"])
            self.assertEqual(3, entry["no_inner_iterations"])
            self.assertEqual(2, entry["no_outer_iterations"])
            self.assertEqual(3.0, entry["max_global_plan_lookahead_dist"])
            self.assertTrue(entry["global_plan_overwrite_orientation"])
            self.assertTrue(entry["via_points_ordered"])
            self.assertEqual(0.3, entry["global_plan_viapoint_sep"])
            self.assertEqual(8.0, entry["weight_viapoint"])
            self.assertEqual(1.0, entry["selection_viapoint_cost_scale"])
            self.assertFalse(entry["viapoints_all_candidates"])

            manager._teb_hybrid_tracking_requested = True
            self.assertTrue(manager._set_teb(0.3, straight_tracking=False))
            transit = updates[-1]
            self.assertEqual(0.3, transit["max_vel_x_backwards"])
            self.assertEqual(1, transit["motion_direction_mode"])
            self.assertTrue(transit["allow_init_with_backwards_motion"])
            self.assertEqual(0.30, transit["xy_goal_tolerance"])
            self.assertEqual(0.40, transit["yaw_goal_tolerance"])
            self.assertFalse(transit["global_plan_overwrite_orientation"])
            self.assertTrue(transit["via_points_ordered"])
            self.assertEqual(0.3, transit["global_plan_viapoint_sep"])
            self.assertEqual(3.0, transit["max_global_plan_lookahead_dist"])
            self.assertEqual(0.20, transit["min_obstacle_dist"])
            self.assertEqual(0.45, transit["inflation_dist"])
            self.assertEqual(8.0, transit["weight_viapoint"])
            self.assertEqual(40.0, transit["weight_viapoint_lateral"])
            self.assertEqual(20.0, transit["weight_viapoint_heading"])
            self.assertEqual(
                5.0, transit["weight_kinematics_forward_drive"]
            )
            self.assertEqual(
                1.0, transit["selection_viapoint_cost_scale"]
            )
            self.assertFalse(transit["viapoints_all_candidates"])
            self.assertEqual(1, transit["max_number_classes"])
            self.assertFalse(transit["include_costmap_obstacles"])
            self.assertEqual(3, transit["no_inner_iterations"])
            self.assertEqual(2, transit["no_outer_iterations"])
            manager._teb_hybrid_tracking_requested = True
            manager._teb_hybrid_expected_reverse = True
            manager._teb_hybrid_cusp_goal = True
            self.assertTrue(manager._set_teb(0.3, straight_tracking=False))
            reverse_transit = updates[-1]
            self.assertTrue(
                reverse_transit["allow_init_with_backwards_motion"]
            )
            self.assertEqual(
                5.0, reverse_transit["weight_kinematics_forward_drive"]
            )
            self.assertEqual(-1, reverse_transit["motion_direction_mode"])
            self.assertEqual(
                3.0, reverse_transit["max_global_plan_lookahead_dist"]
            )
            self.assertEqual(0.12, reverse_transit["xy_goal_tolerance"])
            self.assertEqual(0.15, reverse_transit["yaw_goal_tolerance"])

            self.assertTrue(manager._restore_teb())
            self.assertEqual(baseline, updates[-1])
            self.assertIsNone(manager.original_teb)

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

    def test_navigation_owner_service_is_never_called_with_manager_lock_held(self):
        methods = (
            COVERAGE_MANAGER.CoverageManager._start_service,
            COVERAGE_MANAGER.CoverageManager._start_batch_service,
            COVERAGE_MANAGER.CoverageManager._execute_segment,
            COVERAGE_MANAGER.CoverageManager._run_task,
            COVERAGE_MANAGER.CoverageManager._finalize_batch,
            COVERAGE_MANAGER.CoverageManager._abort_committed_start,
        )
        for method in methods:
            source = textwrap.dedent(inspect.getsource(method))
            tree = ast.parse(source)
            with self.subTest(method=method.__name__):
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
                    calls = [
                        child.func.attr
                        for child in ast.walk(node)
                        if isinstance(child, ast.Call)
                        and isinstance(child.func, ast.Attribute)
                    ]
                    self.assertNotIn("_set_navigation_owner", calls)

    def test_navigation_owner_response_contract_is_fail_closed(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.navigation_owner_service_name = (
            "/navigation_pause/set_coverage_owner"
        )
        owner_token = "coverage-{}".format("a" * 32)
        calls = []

        def client(**request):
            calls.append((request, manager.lock._is_owned()))
            return SimpleNamespace(
                success=True,
                claimed=request["claim"],
                current_owner_token=(owner_token if request["claim"] else ""),
                message="ok",
            )

        manager.navigation_owner_client = client
        with mock.patch.object(COVERAGE_MANAGER.rospy, "wait_for_service"):
            self.assertEqual(
                (True, "ok"),
                manager._set_navigation_owner(True, owner_token),
            )
            self.assertEqual(
                (True, owner_token, "READY"),
                manager._navigation_owner_last_outcome,
            )
            self.assertEqual(
                (True, "ok"),
                manager._set_navigation_owner(False, owner_token),
            )
            self.assertEqual(
                (False, owner_token, "RELEASED"),
                manager._navigation_owner_last_outcome,
            )
            manager.navigation_owner_client = lambda **_request: SimpleNamespace(
                success=False,
                claimed=True,
                current_owner_token=owner_token,
                message="retained until old goal is terminal",
            )
            success, detail = manager._set_navigation_owner(True, owner_token)
            self.assertFalse(success)
            self.assertIn("retained", detail)
            self.assertEqual(
                (True, owner_token, "RETAINED_NOT_READY"),
                manager._navigation_owner_last_outcome,
            )
            manager.navigation_owner_client = lambda **_request: SimpleNamespace(
                success=True,
                claimed=False,
                current_owner_token="",
                message="inconsistent",
            )
            success, detail = manager._set_navigation_owner(True, owner_token)
            self.assertFalse(success)
            self.assertIn("inconsistent", detail)
            manager.navigation_owner_client = mock.Mock(
                side_effect=RuntimeError("response lost")
            )
            success, detail = manager._set_navigation_owner(True, owner_token)
        self.assertFalse(success)
        self.assertIn("response lost", detail)
        self.assertEqual(
            (True, owner_token, "UNKNOWN"),
            manager._navigation_owner_last_outcome,
        )
        self.assertEqual([False, False], [owned for _request, owned in calls])
        self.assertEqual([True, False], [
            request["claim"] for request, _owned in calls
        ])

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

        def plan(_points, _width, _overlap, reachable_seed=None, **_kwargs):
            del reachable_seed, _kwargs
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
            map_digest="static-map",
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
            plan=lambda _points, _width, _overlap, reachable_seed=None,
                        **_kwargs: replanned
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
        self.assertFalse(cancel_response.success)
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
        self.assertIsNotNone(manager.plan)
        self.assertEqual("test-plan", manager.plan_id)
        self.assertEqual("READY", manager.state)

    def test_cancel_during_single_owner_claim_compensates_exact_token(self):
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
        manager._start_external_prechecks = lambda _token: True
        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(0.0, 0.0), 0.0
        )
        replanned = SimpleNamespace(swaths=[], spacing=0.85)
        manager._planner = lambda _grid: SimpleNamespace(
            plan=lambda _points, _width, _overlap, reachable_seed=None,
                        **_kwargs: replanned
        )
        claim_entered = threading.Event()
        release_claim = threading.Event()
        owner_calls = []

        def set_owner(claim, owner_token):
            owner_calls.append((claim, owner_token, manager.lock._is_owned()))
            if claim:
                claim_entered.set()
                self.assertTrue(release_claim.wait(2.0))
            return True, "ok"

        manager._set_navigation_owner = set_owner
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
            self.assertTrue(claim_entered.wait(1.0))
            cancel_response = manager._cancel_service(SimpleNamespace())
            release_claim.set()
            start_thread.join(2.0)
        self.assertFalse(start_thread.is_alive())
        self.assertFalse(cancel_response.success)
        self.assertEqual(1, len(responses))
        self.assertFalse(responses[0].accepted)
        self.assertIn("canceled or superseded", responses[0].message)
        self.assertEqual(2, len(owner_calls))
        self.assertTrue(owner_calls[0][0])
        self.assertFalse(owner_calls[1][0])
        self.assertEqual(owner_calls[0][1], owner_calls[1][1])
        self.assertEqual([False, False], [call[2] for call in owner_calls])
        self.assertRegex(owner_calls[0][1], r"^coverage-[0-9a-f]{32}$")
        self.assertFalse(manager.active)
        self.assertEqual([], manager.move_base.goals)

    def test_single_retained_failure_clears_old_batch_identity_and_can_finalize(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        old_batch_id = "coverage-batch-{}".format("7" * 32)
        manager.batch_id = old_batch_id
        manager.batch_total_regions = 3
        manager.batch_completed_regions = 3
        manager.last_region_id = "old-region"
        manager.last_region_name = "Old region"
        manager.last_region_state = "COMPLETED"
        manager.batch_request_records[old_batch_id] = {
            "fingerprint": "old-payload",
            "state": "TERMINAL",
            "accepted": True,
            "message": "old batch completed",
        }
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
        manager._start_external_prechecks = lambda _token: True
        manager._current_pose = lambda: (
            COVERAGE_MANAGER.Point(0.0, 0.0), 0.0
        )
        replanned = SimpleNamespace(swaths=[], spacing=0.85)
        manager._planner = lambda _grid: SimpleNamespace(
            plan=lambda _points, _width, _overlap, reachable_seed=None,
                        **_kwargs: replanned
        )
        manager._resolve_navigation_owner_claim = (
            lambda _owner_token, _context:
            ("UNKNOWN", "claim response was lost after commit")
        )
        cleanup_events = []
        manager._set_enforced_path = (
            lambda path, coverage_active=True:
            cleanup_events.append((
                "planner", path.active, coverage_active,
                manager.lock._is_owned(),
            )) or True
        )
        manager._restore_teb = (
            lambda: cleanup_events.append((
                "teb", manager.lock._is_owned()
            )) or True
        )
        manager._set_navigation_owner = (
            lambda claim, owner_token:
            cleanup_events.append((
                "owner", claim, owner_token, manager.lock._is_owned()
            )) or (True, "released")
        )

        with mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            response = manager._start_service(SimpleNamespace(
                plan_id="test-plan", max_speed_mps=0.30
            ))

        self.assertFalse(response.accepted)
        self.assertTrue(manager.active)
        self.assertEqual("FAILED", manager.state)
        self.assertEqual("", manager.batch_id)
        self.assertEqual(0, manager.batch_total_regions)
        self.assertEqual("", manager.last_region_id)
        self.assertIn(old_batch_id, manager.batch_request_records)
        retained_owner_token = manager.navigation_owner_token
        self.assertRegex(retained_owner_token, r"^coverage-[0-9a-f]{32}$")

        with mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            canceled = manager._cancel_service(SimpleNamespace())

        self.assertTrue(canceled.success)
        self.assertFalse(manager.active)
        self.assertEqual("CANCELED", manager.state)
        self.assertEqual("", manager.navigation_owner_token)
        self.assertFalse(manager.navigation_owner_claimed)
        self.assertFalse(manager.navigation_owner_releasing)
        self.assertEqual([
            ("planner", False, False, False),
            ("teb", False),
            ("owner", False, retained_owner_token, False),
        ], cleanup_events)
        self.assertEqual(0, manager.move_base.cancel_count)
        self.assertEqual(0, manager.move_base.cancel_all_count)

    def test_worker_start_failure_cleans_planner_and_teb_before_release(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        owner_token = "coverage-{}".format("7" * 32)
        manager.navigation_owner_token = owner_token
        manager.navigation_owner_claimed = True
        manager.active = True
        manager.cancel_requested = False
        manager.state = "GOING_TO_START"
        manager.detail = "accepted"
        manager._publish_status = lambda: None
        events = []
        manager._set_enforced_path = (
            lambda path, coverage_active=True:
            events.append(("planner", path.active, coverage_active)) or True
        )
        manager._restore_teb = (
            lambda: events.append(("teb",)) or True
        )
        manager._set_navigation_owner = (
            lambda claim, token:
            events.append(("owner", claim, token)) or (True, "released")
        )
        with mock.patch.object(
            COVERAGE_MANAGER.rospy.Time,
            "now",
            return_value=COVERAGE_MANAGER.rospy.Time(100.0),
        ):
            response = manager._abort_committed_start(
                owner_token, "worker failed"
            )
        self.assertFalse(response.accepted)
        self.assertEqual([
            ("planner", False, False),
            ("teb",),
            ("owner", False, owner_token),
        ], events)
        self.assertFalse(manager.active)
        self.assertEqual("", manager.navigation_owner_token)
        self.assertEqual([False], [
            message.data for message in manager.active_pub.messages
        ])

    def test_worker_start_cleanup_failure_retains_owner_without_release(self):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        self._seed_lifecycle_state(manager)
        owner_token = "coverage-{}".format("8" * 32)
        manager.navigation_owner_token = owner_token
        manager.navigation_owner_claimed = True
        manager.active = True
        manager.cancel_requested = False
        manager.state = "GOING_TO_START"
        manager.detail = "accepted"
        manager._publish_status = lambda: None
        manager._set_enforced_path = (
            lambda _path, coverage_active=True: False
        )
        manager._restore_teb = lambda: True
        owner_calls = []
        manager._set_navigation_owner = (
            lambda claim, token:
            owner_calls.append((claim, token)) or (True, "released")
        )
        response = manager._abort_committed_start(
            owner_token, "worker failed"
        )
        self.assertFalse(response.accepted)
        self.assertEqual([], owner_calls)
        self.assertTrue(manager.active)
        self.assertTrue(manager.navigation_owner_releasing)
        self.assertEqual(owner_token, manager.navigation_owner_token)
        self.assertEqual([True], [
            message.data for message in manager.active_pub.messages
        ])

    def test_every_task_terminal_state_discards_plan_and_latched_visualizations(self):
        one_transit = [{"type": "transit", "swath_index": 0}]
        cases = (
            ("COMPLETED", [], []),
            (
                "COMPLETED_PARTIAL",
                [{"type": "sweep", "swath_index": 0}],
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

    def test_standalone_release_happens_before_active_is_cleared(self):
        manager, _executed = self._manager([], [])
        owner_token = manager.navigation_owner_token
        observations = []

        def release(claim, token):
            observations.append((
                claim,
                token,
                manager.active,
                manager.navigation_owner_token,
                [message.data for message in manager.active_pub.messages],
                manager.lock._is_owned(),
            ))
            return True, "released"

        manager._set_navigation_owner = release
        self._run(manager)
        self.assertEqual([(
            False, owner_token, True, owner_token, [], False,
        )], observations)
        self.assertFalse(manager.active)
        self.assertEqual("", manager.navigation_owner_token)
        self.assertFalse(manager.navigation_owner_claimed)
        self.assertFalse(manager.navigation_owner_releasing)
        self.assertEqual([False], [
            message.data for message in manager.active_pub.messages
        ])

    def test_standalone_release_failure_keeps_every_gate_closed(self):
        manager, _executed = self._manager([], [])
        owner_token = manager.navigation_owner_token
        manager._set_navigation_owner = (
            lambda _claim, _token: (False, "bridge timeout")
        )
        self._run(manager)
        self.assertEqual("FAILED", manager.state)
        self.assertIn("navigation ownership could not be released", manager.detail)
        self.assertTrue(manager.active)
        self.assertTrue(manager.cancel_requested)
        self.assertEqual(owner_token, manager.navigation_owner_token)
        self.assertTrue(manager.navigation_owner_claimed)
        self.assertTrue(manager.navigation_owner_releasing)
        self.assertEqual([], [
            message.data for message in manager.active_pub.messages
        ])
        self.assertIsNotNone(manager.plan)

    def test_goal_terminal_confirmation_precedes_all_cleanup_and_release(self):
        manager, _executed = self._manager([], [])
        events = []

        class TerminalMoveBase(_MoveBase):
            def cancel_goal(inner_self):
                events.append("cancel")
                super(TerminalMoveBase, inner_self).cancel_goal()

            def wait_for_result(inner_self, _timeout):
                events.append("wait")
                return True

            def get_state(inner_self):
                events.append("terminal")
                return COVERAGE_MANAGER.GoalStatus.PREEMPTED

        move_base = TerminalMoveBase()
        move_base.gh = _GoalHandle(move_base)
        manager.move_base = move_base
        manager.move_base_goal_generation = 1
        manager.move_base_goal_pending = True
        manager.move_base_goal_handle = move_base.gh
        manager.move_base_goal_terminal_state = COVERAGE_MANAGER.GoalStatus.ACTIVE
        manager._set_enforced_path = (
            lambda _path, coverage_active=True:
            events.append("planner") or True
        )
        manager._restore_teb = lambda: events.append("teb") or True
        manager._set_navigation_owner = (
            lambda _claim, _token: events.append("owner") or (True, "ok")
        )
        self._run(manager)
        self.assertEqual([
            "cancel", "wait", "terminal", "planner", "teb", "owner"
        ], events)
        self.assertFalse(manager.move_base_goal_pending)
        self.assertFalse(manager.active)
        self.assertFalse(manager.active_pub.messages[-1].data)

    def test_lost_goal_blocks_planner_teb_owner_and_active_release(self):
        manager, _executed = self._manager([], [])
        move_base = _MoveBase()
        move_base.gh = _GoalHandle(move_base)
        move_base.state = COVERAGE_MANAGER.GoalStatus.LOST
        move_base.wait_result = True
        manager.move_base = move_base
        manager.move_base_terminal_timeout_sec = 0.01
        manager.move_base_goal_generation = 1
        manager.move_base_goal_pending = True
        manager.move_base_goal_handle = move_base.gh
        manager.move_base_goal_terminal_state = COVERAGE_MANAGER.GoalStatus.ACTIVE
        cleanup_calls = []
        manager._set_enforced_path = (
            lambda _path, coverage_active=True:
            cleanup_calls.append("planner") or True
        )
        manager._restore_teb = (
            lambda: cleanup_calls.append("teb") or True
        )
        manager._set_navigation_owner = (
            lambda _claim, _token:
            cleanup_calls.append("owner") or (True, "ok")
        )
        confirmed, lost_detail = manager._confirm_move_base_goal_terminal(
            cancel=True, expected_generation=1
        )
        self.assertFalse(confirmed)
        self.assertIn("LOST", lost_detail)
        manager._wait_for_move_base_goal_terminal = (
            lambda: (False, lost_detail)
        )
        self._run(manager)
        self.assertEqual([], cleanup_calls)
        self.assertGreaterEqual(move_base.cancel_count, 1)
        self.assertTrue(manager.move_base_goal_pending)
        self.assertEqual("FAILED", manager.state)
        self.assertIn("LOST", manager.detail)
        self.assertTrue(manager.active)
        self.assertTrue(manager.navigation_owner_releasing)
        self.assertEqual([], manager.active_pub.messages)

    def test_batch_lost_goal_cannot_enter_an_inter_region_gap(self):
        manager, _prepared, _executed = self._batch_manager([
            "COMPLETED", "COMPLETED"
        ])
        move_base = _MoveBase()
        move_base.gh = _GoalHandle(move_base)
        move_base.state = COVERAGE_MANAGER.GoalStatus.LOST
        manager.move_base = move_base
        manager.move_base_terminal_timeout_sec = 0.01
        manager.move_base_goal_generation = 1
        manager.move_base_goal_pending = True
        manager.move_base_goal_handle = move_base.gh
        manager.move_base_goal_terminal_state = COVERAGE_MANAGER.GoalStatus.ACTIVE
        cleanup_calls = []
        manager._set_enforced_path = (
            lambda _path, coverage_active=True:
            cleanup_calls.append("planner") or True
        )
        manager._restore_teb = (
            lambda: cleanup_calls.append("teb") or True
        )
        manager._set_navigation_owner = (
            lambda _claim, _token:
            cleanup_calls.append("owner") or (True, "ok")
        )
        confirmed, lost_detail = manager._confirm_move_base_goal_terminal(
            cancel=True, expected_generation=1
        )
        self.assertFalse(confirmed)
        self.assertIn("LOST", lost_detail)
        manager._wait_for_move_base_goal_terminal = (
            lambda: (False, lost_detail)
        )
        manager._finalize_batch(
            "test-batch-token", "CANCELED", "operator canceled"
        )
        self.assertEqual([], cleanup_calls)
        self.assertGreaterEqual(move_base.cancel_count, 1)
        self.assertTrue(manager.move_base_goal_pending)
        self.assertEqual("FAILED", manager.state)
        self.assertIn("LOST", manager.detail)
        self.assertEqual("FINALIZING", manager.batch_phase)
        self.assertTrue(manager.batch_active)
        self.assertTrue(manager.active)
        self.assertEqual([True], [
            message.data for message in manager.active_pub.messages
        ])

    def test_goal_handle_change_is_never_misassociated_as_terminal(self):
        manager, _executed = self._manager([], [])
        expected_handle = object()
        manager.move_base.gh = _GoalHandle(manager.move_base)
        manager.move_base_goal_generation = 4
        manager.move_base_goal_pending = True
        manager.move_base_goal_handle = expected_handle
        confirmed, detail = manager._confirm_move_base_goal_terminal(
            cancel=True, expected_generation=4
        )
        self.assertFalse(confirmed)
        self.assertIn("different goal handle", detail)
        self.assertEqual(0, manager.move_base.cancel_count)
        self.assertTrue(manager.move_base_goal_pending)

    def test_status_timer_reasserts_cancel_for_retained_uncertain_goal(self):
        manager, _executed = self._manager([], [])
        manager.navigation_owner_releasing = True
        manager.move_base_goal_pending = True
        manager.move_base.gh = _GoalHandle(manager.move_base)
        manager.move_base_goal_handle = manager.move_base.gh
        manager.original_teb = None
        manager._pause_for_avoidance_loss = lambda: None
        manager._pause_for_chassis_fault = lambda: None
        manager._status_timer(None)
        self.assertEqual(1, manager.move_base.cancel_count)

    def test_finalizer_persists_until_a_later_trusted_terminal_state(self):
        manager, _executed = self._manager([], [])
        attempts = mock.Mock(side_effect=[
            (False, "terminal wait timed out"),
            (False, "move_base reported LOST"),
            (True, "move_base reached PREEMPTED"),
        ])
        manager._confirm_move_base_goal_terminal = attempts
        waits = []
        manager._lifecycle_wait = lambda timeout: waits.append(timeout)
        with mock.patch.object(
            COVERAGE_MANAGER.rospy, "is_shutdown", return_value=False
        ), mock.patch.object(
            COVERAGE_MANAGER.rospy, "logerr_throttle"
        ):
            confirmed, detail = manager._wait_for_move_base_goal_terminal()
        self.assertTrue(confirmed)
        self.assertIn("PREEMPTED", detail)
        self.assertEqual(3, attempts.call_count)
        self.assertEqual([0.1, 0.1], waits)
        self.assertEqual("FINALIZING", manager.state)
        self.assertIn("LOST", manager.detail)

    def test_cancel_during_finalizing_idempotently_reasserts_exact_goal(self):
        manager, _prepared, _executed = self._batch_manager([
            "COMPLETED", "COMPLETED"
        ])
        manager.navigation_owner_releasing = True
        manager.move_base_goal_generation = 1
        manager.move_base_goal_pending = True
        manager.move_base.gh = _GoalHandle(manager.move_base)
        manager.move_base_goal_handle = manager.move_base.gh
        response = manager._cancel_service(SimpleNamespace())
        self.assertTrue(response.success)
        self.assertIn("reasserted", response.message)
        self.assertEqual(1, manager.move_base.cancel_count)
        self.assertTrue(manager.batch_active)
        self.assertEqual("test-batch-token", manager.batch_token)

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
        self.assertEqual(["transit"], executed)
        self.assertEqual("CANCELED", manager.state)
        self.assertEqual([0], manager.blocked_segments)

    def test_failed_entry_alignment_pauses_same_sweep_without_advancing(self):
        sweep = {"type": "sweep", "swath_index": 0}
        later_transition = {"type": "rolling_transit", "swath_index": 1}
        manager, executed = self._manager(
            [sweep, later_transition], []
        )
        manager._prepare_sweep_entry = lambda _segment, _index: (
            "entry-recovery"
        )
        manager._execute_sweep_entry_recovery = (
            lambda _segment, _index: "blocked"
        )
        pause_reasons = []
        manager._wait_while_paused = lambda: (
            pause_reasons.append(manager.manual_pause_reason) or False
        )

        self._run(manager)

        self.assertEqual([], executed)
        self.assertEqual("CANCELED", manager.state)
        self.assertEqual([0], manager.blocked_segments)
        self.assertEqual(1, len(pause_reasons))
        self.assertIn("sweep-entry Hybrid alignment", pause_reasons[0])

    def test_final_transit_retry_unlocks_its_dependent_sweep(self):
        manager, executed = self._manager(
            self._swath_segments(), ["blocked", "succeeded", "succeeded"]
        )
        manager.segment_retry_count = 1
        self._run(manager)
        self.assertEqual(["transit", "transit", "sweep"], executed)
        self.assertEqual("COMPLETED", manager.state)
        self.assertEqual([], manager.blocked_segments)

    def test_blocked_gear_part_is_replaced_by_complete_transition_replan(self):
        goal = COVERAGE_MANAGER.Point(2.0, 0.0)
        old = {
            "type": "transit",
            "swath_index": 1,
            "transition_goal": goal,
            "transition_goal_yaw": 0.0,
            "transition_path": COVERAGE_MANAGER.Path(),
        }
        sweep = {"type": "sweep", "swath_index": 1}
        first = {
            "type": "transit",
            "swath_index": 1,
            "transition_goal": goal,
            "transition_goal_yaw": 0.0,
            "transition_path": COVERAGE_MANAGER.Path(),
        }
        second = dict(first)
        manager, executed = self._manager(
            [old, sweep],
            ["blocked", "succeeded", "succeeded", "succeeded"],
        )
        manager.segment_retry_count = 1
        replans = []
        manager._replan_remaining_hybrid_transition = lambda segment: (
            replans.append(segment) or [first, second],
            "replanned",
        )

        self._run(manager)

        self.assertEqual(1, len(replans))
        self.assertIs(old, replans[0])
        self.assertEqual(
            ["transit", "transit", "transit", "sweep"], executed
        )
        self.assertEqual("COMPLETED", manager.state)
        self.assertEqual(3, manager.total_segments)

    def test_cancel_during_final_retry_is_not_reported_as_partial(self):
        manager, executed = self._manager(
            self._swath_segments(), ["blocked", "canceled"]
        )
        manager.segment_retry_count = 1
        self._run(manager)
        self.assertEqual(["transit", "transit"], executed)
        self.assertEqual("CANCELED", manager.state)

    def test_teb_restore_failure_forces_failed_terminal_state(self):
        manager, _executed = self._manager([], [], restore=False)
        owner_calls = []
        manager._set_navigation_owner = (
            lambda claim, token:
            owner_calls.append((claim, token)) or (True, "released")
        )
        self._run(manager)
        self.assertEqual("FAILED", manager.state)
        self.assertIn("TEB parameters could not be restored", manager.detail)
        self.assertEqual([], owner_calls)
        self.assertTrue(manager.active)
        self.assertTrue(manager.navigation_owner_releasing)
        self.assertEqual([], manager.active_pub.messages)

    def test_planner_cleanup_failure_retains_owner_without_release_call(self):
        manager, _executed = self._manager([], [])
        manager._set_enforced_path = (
            lambda _path, coverage_active=True: False
        )
        owner_calls = []
        manager._set_navigation_owner = (
            lambda claim, token:
            owner_calls.append((claim, token)) or (True, "released")
        )
        self._run(manager)
        self.assertEqual("FAILED", manager.state)
        self.assertIn("planner ownership could not be restored", manager.detail)
        self.assertEqual([], owner_calls)
        self.assertTrue(manager.active)
        self.assertTrue(manager.navigation_owner_releasing)
        self.assertEqual([], manager.active_pub.messages)

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
        self, manager, response, teb_radius=1.35, teb_lookahead=8.0,
        local_costmap_width=20.0, local_costmap_height=20.0,
        hybrid_radius=1.35
    ):
        parameters = {
            "/move_base/TebLocalPlannerROS/min_turning_radius": teb_radius,
            "/move_base/TebLocalPlannerROS/wheelbase": 0.65,
            "/move_base/TebLocalPlannerROS/use_proportional_saturation": True,
            "/move_base/TebLocalPlannerROS/cmd_angle_instead_rotvel": False,
            "/move_base/TebLocalPlannerROS/treat_unknown_as_obstacle": True,
            "/move_base/TebLocalPlannerROS/max_global_plan_lookahead_dist": teb_lookahead,
            "/move_base/local_costmap/width": local_costmap_width,
            "/move_base/local_costmap/height": local_costmap_height,
            "/move_base/CoverageGlobalPlanner/hybrid_minimum_turning_radius": (
                hybrid_radius
            ),
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

    def test_hybrid_radius_must_match_coverage_model(self):
        manager = self._kinematics_manager()
        self.assertFalse(self._verify_kinematics(
            manager, self._chassis_response(), hybrid_radius=1.20
        ))
        self.assertFalse(manager.kinematics_verified)
        self.assertIn("Hybrid A*", manager.detail)

    def test_teb_lookahead_must_fit_inside_rolling_costmap_margin(self):
        manager = self._kinematics_manager()
        self.assertFalse(self._verify_kinematics(
            manager, self._chassis_response(), teb_lookahead=8.6
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
