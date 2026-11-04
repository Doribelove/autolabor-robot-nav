#!/usr/bin/env python3
"""Direct-camera lifecycle regressions; no ROS master, hardware or publishers."""
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace as Obj
import unittest
from unittest import mock

from autolabor_fod_control.gps_recovery import FodGpsMemory
from autolabor_fod_control.mode_manager import CommandArbiter
from autolabor_fod_control.recovery_verification import RecoveryVerification

spec = importlib.util.spec_from_file_location('direct_mode', Path(__file__).resolve().parents[1] /
                                            'scripts/fod_navigation_mode_manager.py')
MODE = importlib.util.module_from_spec(spec)
spec.loader.exec_module(MODE)


class DirectRecoveryTest(unittest.TestCase):
    def setUp(self):
        n = self.n = MODE.FodNavigationModeManager.__new__(MODE.FodNavigationModeManager)
        n.lock, n.transition_lock = threading.RLock(), threading.Lock()
        n.shutdown_event, n.recovery_cancel_requested = threading.Event(), threading.Event()
        n.mode, n.command_source, n.visual_state = MODE.GPS_ACTIVE, MODE.GPS_SOURCE, 'DISABLED'
        n.retain_gps_route, n.gps_fallback_enabled = True, True
        n.direct_recovery, n.recovery_armed = False, False
        n.recovery_generation, n.recovery_radius_m = 0, 10.
        n.verification_enabled, n.verification_this_run = True, False
        n.verification = RecoveryVerification()
        n.arbiter = CommandArbiter(.5)
        n.fod_gps_memory = FodGpsMemory(frames=3)
        n.fod_gps_point, n.fod_reference_goal = {'stale': True}, {'x': 100., 'y': 200.}
        n.gps_route = dict(active=False)
        n.recovery_sweep_ready = False
        n.sweep_status = 'mock actuators'
        n.visual_enable_service = '/fod_visual_servo/set_enabled'
        n.gps_clear_service = '/gps/long_range/clear'
        n.service_wait_timeout_sec = .1
        n._publish_status, n._publish_zero_locked = mock.Mock(), mock.Mock()
        n._output_route_error = mock.Mock(return_value='')
        n._wait_for_stopped_vehicle = mock.Mock(return_value=True)
        n._pause_navigation_route = mock.Mock(return_value=(True, 'paused'))
        n._cancel_all_move_base_goals = mock.Mock()
        n._call_set_bool = mock.Mock(return_value=(True, 'servo configured'))
        n._set_sweep = mock.Mock(return_value=(True, 'sweep on'))
        n._stop_recovery_sweep = mock.Mock(return_value=True)
        n._maybe_start_recovery = mock.Mock()
        self.params = {}
        self.patch = mock.patch.object(MODE.rospy, 'set_param', side_effect=self.params.__setitem__)
        self.patch.start()
        self.addCleanup(self.patch.stop)

    def start(self):
        r = self.n._set_direct_fod_enabled_cb(Obj(data=True))
        self.assertTrue(r.success, r.message)

    def test_direct_starts_without_gps_goal_and_erases_stale_target_filters(self):
        self.start()
        n = self.n
        self.assertEqual(n.mode, MODE.FOD_ACTIVE)
        self.assertTrue(n.direct_recovery)
        self.assertTrue(n.gps_fallback_enabled)  # GPS configuration is not rewritten.
        self.assertFalse(n._use_gps_fallback())
        self.assertFalse(n.recovery_armed)
        self.assertIsNone(n.fod_gps_point)
        self.assertEqual(self.params['/fod_visual_servo/preferred_track_id'], 0)
        self.assertEqual(self.params['/fod_visual_servo/recovery_reference_goal'], {})
        self.assertFalse(self.params['/fod_visual_servo/gps_handoff_on_approach_limit'])
        n._pause_navigation_route.assert_called_once_with(True)
        n._set_sweep.assert_called_once_with(True)
        self.assertTrue(n.verification_this_run)
        self.assertEqual(n.verification.distance_m, 1.5)

    def test_double_start_is_idempotent_and_next_cycle_resets_same_servo(self):
        self.start(); self.start()
        self.n._set_sweep.assert_called_once_with(True)
        self.n.mode = MODE.FOD_COMPLETE_STOP
        self.start()
        self.assertEqual(self.n._set_sweep.call_count, 2)
        self.assertEqual(self.n._call_set_bool.call_args_list[-2:], [
            mock.call(self.n.visual_enable_service, False), mock.call(self.n.visual_enable_service, True)])
        self.assertNotIn(mock.call(False), self.n._pause_navigation_route.call_args_list)

    def test_stop_preempts_inflight_rod_action_and_never_enables_servo(self):
        entered, release = threading.Event(), threading.Event()
        def sweep(_enabled):
            entered.set(); release.wait(3)
            return True, 'ack'
        self.n._set_sweep.side_effect = sweep
        started, stopped = [], []
        a = threading.Thread(target=lambda: started.append(self.n._set_direct_fod_enabled_cb(Obj(data=True))))
        a.start()
        self.assertTrue(entered.wait(1))
        b = threading.Thread(target=lambda: stopped.append(self.n._set_direct_fod_enabled_cb(Obj(data=False))))
        b.start()
        self.assertTrue(self.n.recovery_cancel_requested.wait(1))
        self.assertEqual(self.n.command_source, MODE.STOP_SOURCE)
        release.set(); a.join(3); b.join(3)
        self.assertFalse(a.is_alive() or b.is_alive())
        self.assertFalse(started[0].success)
        self.assertTrue(stopped[0].success)
        self.assertNotIn(mock.call(self.n.visual_enable_service, True), self.n._call_set_bool.call_args_list)
        self.assertNotIn(mock.call(False), self.n._pause_navigation_route.call_args_list)
        self.assertEqual(self.n.mode, MODE.FOD_ABORTED)

    def test_direct_stop_does_not_touch_plain_gps_navigation(self):
        self.assertTrue(self.n._set_direct_fod_enabled_cb(Obj(data=False)).success)
        self.n._pause_navigation_route.assert_not_called()
        self.n._call_set_bool.assert_not_called()
        self.n._publish_zero_locked.assert_not_called()
        self.assertEqual(self.n.mode, MODE.GPS_ACTIVE)

    def test_restore_next_cycle_verification_after_aborted_series_never_moves(self):
        self.n.mode = MODE.FOD_ABORTED
        self.n.verification_enabled = False
        response = self.n._set_verification_enabled_cb(Obj(data=True))
        self.assertTrue(response.success)
        self.assertTrue(self.n.verification_enabled)
        self.assertEqual(self.n.mode, MODE.FOD_ABORTED)
        self.n._call_set_bool.assert_not_called()
        self.n._set_sweep.assert_not_called()

    def test_verification_cannot_be_reenabled_mid_recovery(self):
        self.n.mode = MODE.FOD_ACTIVE
        self.n.verification_enabled = False
        self.assertFalse(self.n._set_verification_enabled_cb(Obj(data=True)).success)
        self.assertFalse(self.n.verification_enabled)

    def test_cancellation_during_reverse_does_not_resume_old_route(self):
        self.start()
        self.n._recovery_finished_locked('travel finished')
        self.assertEqual(self.n.mode, MODE.FOD_VERIFY)
        self.assertTrue(self.n._set_direct_fod_enabled_cb(Obj(data=False)).success)
        self.assertEqual(self.n.verification.phase, 'CANCELED')
        self.assertEqual(self.n.command_source, MODE.STOP_SOURCE)
        self.assertNotIn(mock.call(False), self.n._pause_navigation_route.call_args_list)

    def test_direct_never_uses_even_an_old_gps_memory_point(self):
        self.start()
        self.n.fod_gps_point = {'fod_x': 100., 'fod_y': 200.}
        self.n.fod_reference_goal = {'x': 100., 'y': 200.}
        self.n.recovery_sweep_ready = True
        self.assertFalse(self.n._prepare_gps_fallback_locked())
        self.n._remember_fod_gps(None, [], [])  # No GPS/TF dependencies are consulted.
        self.assertEqual(self.n.mode, MODE.FOD_ACTIVE)

    def test_existing_gps_start_still_only_arms_proximity_recovery(self):
        with mock.patch.object(MODE.rospy, 'get_param', return_value=0.):
            self.assertTrue(self.n._set_fod_enabled_cb(Obj(data=True)).success)
        self.assertTrue(self.n.recovery_armed)
        self.assertFalse(self.n.direct_recovery)
        self.assertTrue(self.n._use_gps_fallback())
        self.assertEqual(self.n.mode, MODE.GPS_ACTIVE)
        self.n._set_sweep.assert_not_called()
        self.n._call_set_bool.assert_not_called()

    def test_new_gps_goal_clears_old_route_before_resuming_after_direct(self):
        self.start(); self.n.mode = MODE.FOD_COMPLETE_STOP
        order = []
        def pause(enabled):
            order.append(('pause', enabled)); return True, 'ok'
        self.n._pause_navigation_route.side_effect = pause
        def clear():
            order.append(('clear',)); return Obj(success=True, message='cleared')
        with mock.patch.object(MODE.rospy, 'wait_for_service'), \
                mock.patch.object(MODE.rospy, 'ServiceProxy', return_value=clear):
            self.assertTrue(self.n._prepare_gps_goal_cb(None).success)
        self.assertEqual(order, [('clear',), ('pause', False)])
        self.assertFalse(self.n.direct_recovery)
        self.assertTrue(self.n._use_gps_fallback())

    def test_reinspection_image_health_uses_lower_half_only(self):
        import cv2
        import numpy as np
        import rospy
        self.n.verification = Obj(phase='OBSERVING', image=mock.Mock())
        self.n.verification_roi_top, self.n.verification_roi_bottom = .5, 1.
        pattern = np.tile(np.arange(100, dtype=np.uint8), (50, 1))
        for lower_visible in (False, True):
            pixels = np.zeros((100, 100), dtype=np.uint8)
            pixels[50:] = pattern if lower_visible else 0
            pixels[:50] = 0 if lower_visible else pattern
            msg = Obj(header=Obj(stamp=rospy.Time.from_sec(10.)),
                      data=cv2.imencode('.png', pixels)[1].tobytes())
            with mock.patch.object(MODE.rospy.Time, 'now', return_value=rospy.Time.from_sec(10.1)):
                self.n._verification_image_cb(msg)
            self.assertEqual(self.n.verification.image.call_args[0][2], lower_visible)


if __name__ == '__main__':
    unittest.main()
