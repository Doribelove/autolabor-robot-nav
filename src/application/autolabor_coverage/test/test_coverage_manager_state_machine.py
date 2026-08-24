#!/usr/bin/env python3

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace
import threading
import unittest
from unittest import mock


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

    def cancel_goal(self):
        self.cancel_count += 1


class CoverageManagerStateMachineTest(unittest.TestCase):
    def _manager(self, segments, results, restore=True):
        manager = COVERAGE_MANAGER.CoverageManager.__new__(
            COVERAGE_MANAGER.CoverageManager
        )
        manager.lock = threading.RLock()
        manager.segment_retry_count = 0
        manager.final_retry_count = 1
        manager.obstacle_wait_sec = 0.0
        manager.cancel_requested = False
        manager.current_segment = 0
        manager.total_segments = 0
        manager.blocked_segments = []
        manager.state = "GOING_TO_START"
        manager.detail = "test task"
        manager.active = True
        manager.manual_pause = False
        manager.manual_pause_reason = ""
        manager.external_pause = False
        manager.plan_id = "test-plan"
        manager.move_base = _MoveBase()
        manager.enforced_path_pub = _Publisher()
        manager.active_pub = _Publisher()
        manager._segments = lambda _route, _current: segments
        manager._wait_while_paused = lambda: True
        manager._restore_teb = lambda: restore
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

    def test_manual_resume_requires_recovered_obstacle_sensing(self):
        manager = self._avoidance_manager()
        manager.lock = threading.RLock()
        manager.active = True
        manager.manual_pause = True
        manager.manual_pause_reason = "obstacle sensing lost"
        manager.external_pause = False
        manager.avoidance_loss_paused = True
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
