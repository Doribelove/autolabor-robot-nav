#!/usr/bin/env python3

import math
from concurrent.futures import ThreadPoolExecutor
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
import unittest
from types import SimpleNamespace
from unittest import mock


PACKAGE_SRC = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(PACKAGE_SRC))

from autolabor_fod_control.mode_manager import (  # noqa: E402
    FOD_SOURCE,
    GPS_SOURCE,
    STOP_SOURCE,
    CommandArbiter,
    final_goal_distance,
    stopped_sample_is_valid,
)


class FinalGoalDistanceTest(unittest.TestCase):
    def setUp(self):
        self.route = dict(active=True, paused=False, state="ACTIVE_INTERMEDIATE",
                          frame_id="camera_init", final_x=13.0, final_y=24.0,
                          subgoal_x=10.1, subgoal_y=20.0)

    def test_uses_final_target_not_nearby_rolling_subgoal(self):
        self.assertEqual(final_goal_distance(self.route, 10, 20, "/camera_init"), 5.0)
        self.assertGreater(final_goal_distance(self.route, 9.99, 20, "camera_init"), 5.0)

    def test_replaced_final_target_is_used(self):
        self.route.update(final_x=100, final_y=20)
        self.assertEqual(final_goal_distance(self.route, 10, 20, "camera_init"), 90)

    def test_invalid_or_inactive_routes_cannot_trigger(self):
        for change in ({"paused": True}, {"active": False, "state": "CANCELED"},
                       {"active": False, "state": "ABORTED"}, {"final_x": math.nan},
                       {"final_y": None}, {"frame_id": "odom"}):
            with self.subTest(change=change):
                self.assertIsNone(final_goal_distance(dict(self.route, **change), 10, 20, "camera_init"))
        self.assertIsNone(final_goal_distance({}, 10, 20, "camera_init"))
        self.assertIsNone(final_goal_distance(self.route, math.inf, 20, "camera_init"))

    def test_arrived_final_target_can_still_be_recovered(self):
        self.route.update(active=False, state="COMPLETE")
        self.assertEqual(final_goal_distance(self.route, 13, 24, "camera_init"), 0)


class CommandArbiterTest(unittest.TestCase):
    def test_only_selected_fresh_source_can_move(self):
        arbiter = CommandArbiter(0.5)
        arbiter.update(GPS_SOURCE, 1.2, 0.2, 10.0)
        arbiter.update(FOD_SOURCE, 0.2, -0.05, 10.0)

        self.assertEqual(
            arbiter.sample(GPS_SOURCE, 10.1)[:2],
            (1.2, 0.2),
        )
        self.assertEqual(
            arbiter.sample(FOD_SOURCE, 10.1)[:2],
            (0.2, -0.05),
        )

    def test_transition_stop_cannot_forward_a_cached_command(self):
        arbiter = CommandArbiter(0.5)
        arbiter.update(GPS_SOURCE, 2.7, 0.4, 10.0)

        self.assertEqual(arbiter.sample(STOP_SOURCE, 10.01)[:2], (0.0, 0.0))

    def test_stale_or_nonfinite_command_fails_to_zero(self):
        arbiter = CommandArbiter(0.5)
        arbiter.update(GPS_SOURCE, 1.0, 0.1, 10.0)
        self.assertEqual(arbiter.sample(GPS_SOURCE, 10.51)[:2], (0.0, 0.0))

        arbiter.update(FOD_SOURCE, math.nan, 0.0, 11.0)
        self.assertEqual(arbiter.sample(FOD_SOURCE, 11.1)[:2], (0.0, 0.0))

    def test_clearing_gps_prevents_pre_pause_command_reuse(self):
        arbiter = CommandArbiter(0.5)
        arbiter.update(GPS_SOURCE, 1.0, 0.0, 10.0)
        arbiter.clear(GPS_SOURCE)

        self.assertEqual(arbiter.sample(GPS_SOURCE, 10.1)[:2], (0.0, 0.0))


class StopConfirmationInputTest(unittest.TestCase):
    def test_fresh_low_speed_odometry_is_stopped(self):
        self.assertTrue(
            stopped_sample_is_valid(0.02, 0.04, 0.1, 0.6, 0.03, 0.05)
        )

    def test_stale_fast_or_nonfinite_odometry_is_not_stopped(self):
        self.assertFalse(
            stopped_sample_is_valid(0.0, 0.0, 0.61, 0.6, 0.03, 0.05)
        )
        self.assertFalse(
            stopped_sample_is_valid(0.031, 0.0, 0.1, 0.6, 0.03, 0.05)
        )
        self.assertFalse(
            stopped_sample_is_valid(0.0, math.nan, 0.1, 0.6, 0.03, 0.05)
        )


class ConcurrentGraphQueryTest(unittest.TestCase):
    def test_first_nearby_fod_uses_pickup_speed_before_gps_memory_confirmation(self):
        path=Path(__file__).resolve().parents[1]/"scripts/fod_navigation_mode_manager.py"
        spec=importlib.util.spec_from_file_location("near_fod_speed_test",path)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        node=module.FodNavigationModeManager.__new__(module.FodNavigationModeManager)
        node.lock=threading.RLock();node.arbiter=CommandArbiter(.5)
        node.command_source=GPS_SOURCE;node.mode=module.GPS_ACTIVE
        node.recovery_armed=True;node.gps_fallback_enabled=True
        node.fod_gps_point=None;node.fod_approach_started=True;node.keepout=None
        node.cmd_pub=mock.Mock()
        node.arbiter.update(GPS_SOURCE,1.6,.32,time.monotonic())
        node._publish_command()
        command=node.cmd_pub.publish.call_args.args[0]
        self.assertAlmostEqual(command.linear.x,.6)
        self.assertAlmostEqual(command.angular.z,.12)
        node.recovery_armed=False
        node._publish_command()
        command=node.cmd_pub.publish.call_args.args[0]
        self.assertAlmostEqual(command.linear.x,1.6)

    def test_delayed_completion_worker_cannot_stop_a_new_recovery(self):
        path=Path(__file__).resolve().parents[1]/"scripts/fod_navigation_mode_manager.py"
        spec=importlib.util.spec_from_file_location("single_completion_test",path)
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        node=module.FodNavigationModeManager.__new__(module.FodNavigationModeManager)
        node.lock=threading.RLock()
        node.transition_lock=threading.Lock()
        node._stop_recovery_sweep=mock.Mock()
        for mode in (module.GPS_ACTIVE,module.FOD_ACTIVE,module.ENTERING_FOD):
            node.mode=mode
            node._sweep_abort_worker()
        node._stop_recovery_sweep.assert_not_called()
        node.mode=module.FOD_COMPLETE_STOP
        node._sweep_abort_worker()
        node._stop_recovery_sweep.assert_called_once()

    def test_timer_and_handoff_queries_do_not_share_a_busy_connection(self):
        path = Path(__file__).resolve().parents[1] / "scripts/fod_navigation_mode_manager.py"
        spec = importlib.util.spec_from_file_location("mode_manager_node_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)

        class SingleConnectionMaster:
            def __init__(self):
                self.busy = threading.Lock()

            def call(self, result):
                if not self.busy.acquire(blocking=False):
                    raise RuntimeError("Request-sent")
                try:
                    time.sleep(0.005)
                    return result
                finally:
                    self.busy.release()

            def getPid(self):
                return self.call(123)

            def getSystemState(self):
                return self.call(([], [], []))

        node = module.FodNavigationModeManager.__new__(module.FodNavigationModeManager)
        node.master = SingleConnectionMaster()
        node.master_pid = 123
        node.master_lock = threading.Lock()
        node.output_cmd_topic = "/test/cmd_vel"
        node.require_output_driver = False
        node.gps_cmd_topic = "/test/gps_cmd"
        node.fod_cmd_topic = "/test/fod_cmd"
        node.expected_gps_limiter_node = "/fake_gps"
        node.expected_move_base_node = "/fake_move_base"
        node.expected_visual_node = "/fake_visual"
        with mock.patch.object(module.rospy, "get_name", return_value="/mode_test"):
            with ThreadPoolExecutor(max_workers=4) as pool:
                results = list(pool.map(lambda _: node._output_route_error(), range(12)))
        self.assertEqual(results, [""] * 12)


class RecoveryEquipmentLifecycleTest(unittest.TestCase):
    def setUp(self):
        path = Path(__file__).resolve().parents[1] / "scripts/fod_navigation_mode_manager.py"
        spec = importlib.util.spec_from_file_location("recovery_equipment_test", path)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.node = self.module.FodNavigationModeManager.__new__(self.module.FodNavigationModeManager)
        self.node.lock = threading.RLock()
        self.node.sweep_enabled = True
        self.node.sweep_auto_stop = True
        self.node.recovery_sweep_ready = False
        self.node.sweep_started_for_recovery = False
        self.node.sweep_script = "/mock/sweep_test.py"
        self.node.sweep_ip = "127.0.0.1"
        self.node.sweep_port = 50193

    @staticmethod
    def reply(success=True, sent=True, acknowledged=True):
        return SimpleNamespace(returncode=0 if success else 2, stdout=json.dumps(dict(
            success=success, previous_state=1, hydraulic_sent=sent,
            acknowledged=acknowledged, message="mock result")))

    def test_proximity_handoff_completion_and_cancel_use_one_pulse_each(self):
        with mock.patch.object(self.module.subprocess, "run", return_value=self.reply()) as run:
            self.assertTrue(self.node._set_sweep(True)[0])
            # Preexisting manual ON does not exempt this cycle from cleanup.
            self.assertTrue(self.node.sweep_started_for_recovery)
            self.assertTrue(self.node._set_sweep(True)[0])
            self.node._stop_recovery_sweep()
            self.node._stop_recovery_sweep()
            self.assertEqual([c.args[0][2] for c in run.call_args_list],
                             ["recovery-start", "on", "recovery-stop"])
            self.assertFalse(self.node.sweep_started_for_recovery)
            self.assertFalse(self.node.recovery_sweep_ready)

    def test_uncertain_start_is_cleaned_up_without_repeating_failed_retraction(self):
        with mock.patch.object(self.module.subprocess, "run",
                               return_value=self.reply(False, True, False)) as run, \
                mock.patch.object(self.module.rospy, "logerr"):
            self.assertFalse(self.node._set_sweep(True)[0])
            self.node._stop_recovery_sweep()
            self.node._stop_recovery_sweep()
            self.assertEqual([c.args[0][2] for c in run.call_args_list],
                             ["recovery-start", "recovery-stop"])

    def test_no_hydraulic_command_sent_requires_no_cleanup_motion(self):
        with mock.patch.object(self.module.subprocess, "run",
                               return_value=self.reply(False, False, False)) as run:
            self.assertFalse(self.node._set_sweep(True)[0])
            self.node._stop_recovery_sweep()
            self.assertEqual(run.call_count, 1)

    def test_child_timeout_keeps_one_cleanup_for_possibly_sent_pulse(self):
        timeout = self.module.subprocess.TimeoutExpired(["sweep_test.py"], 20)
        with mock.patch.object(self.module.subprocess, "run",
                               side_effect=[timeout, self.reply()]) as run:
            self.assertFalse(self.node._set_sweep(True)[0])
            self.node._stop_recovery_sweep()
            self.node._stop_recovery_sweep()
            self.assertEqual(run.call_count, 2)


if __name__ == "__main__":
    unittest.main()
