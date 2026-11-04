#!/usr/bin/env python3
"""Reverse distance and temporal evidence tests; no ROS master or actuators."""
import math
import json
import unittest
import importlib.util
from pathlib import Path
import threading
from types import SimpleNamespace as Obj
from unittest import mock
from autolabor_fod_control.recovery_verification import RecoveryVerification, rear_corridor_clear
from autolabor_fod_control.visual_servo import PixelDetection


def box(x=500, identity=1):
    return PixelDetection(0, 'Metal', .8, x, 300, 30, 30, x+15, 330, track_id=identity)


class VerificationTest(unittest.TestCase):
    def observing(self):
        c = RecoveryVerification()
        c.prepare(); c.begin_reverse((0., 0., 0., 'odom', 10.), 10.)
        for i in range(1, 151):
            v = c.motion((-i*.01, 0., 0., 'odom', 10+i*.05), -.2, 10+i*.05, 10+i*.05)
            if i == 100:
                self.assertEqual(c.phase, 'REVERSING')
                self.assertLess(v, 0.)  # One metre is no longer the endpoint.
        self.assertEqual(c.phase, 'STOPPING')
        self.assertEqual(v, 0)
        for i in range(12):
            t = 17.6+i*.05
            c.motion((-1.5, 0., 0., 'odom', t), 0., t, t)
        self.assertEqual(c.phase, 'OBSERVING')
        return c

    def feed_window(self, c, boxes=None, image_valid=True):
        for i in range(1, 46):
            t = c.observe_started+i*.05
            c.image(t, t, image_valid)
            c.detections(t, t, boxes(i) if boxes else [])
            c.motion((-1.5, 0., 0., 'odom', t), 0., t, t)
        return c.phase

    def test_distance_uses_odom_and_stops_before_observation(self):
        c = self.observing()
        self.assertAlmostEqual(c.progress_m, 1.5)
        self.assertEqual(c.status()['reverse_distance_m'], 1.5)
        self.assertEqual(c.frame_count, 0)

    def test_empty_fresh_camera_passes_after_full_window(self):
        self.assertEqual(self.feed_window(self.observing()), 'PASSED')

    def test_same_fod_at_point_three_seconds_fails_even_if_later_disappears(self):
        c = self.observing()
        self.assertEqual(self.feed_window(c, lambda i: [box()] if 2 <= i <= 8 else []), 'FOD_REMAINS')
        self.assertGreaterEqual(c.peak_stable_sec+1e-8, .3)

    def test_short_flicker_does_not_fail(self):
        self.assertEqual(self.feed_window(self.observing(), lambda i: [box()] if 2 <= i <= 6 else []), 'PASSED')

    def test_unrelated_boxes_cannot_accumulate_stability(self):
        self.assertEqual(self.feed_window(self.observing(), lambda i: [box(100 if i%2 else 600, i)]), 'PASSED')

    def test_id_change_with_same_box_still_detects_residual(self):
        self.assertEqual(self.feed_window(self.observing(), lambda i: [box(identity=i)]), 'FOD_REMAINS')

    def test_detection_stall_is_not_empty_scene(self):
        c=self.observing(); start=c.observe_started
        for i in range(1, 80):
            t=start+i*.05
            c.image(t,t)
            c.detections(start+.05,t,[])
            c.motion((-1.5,0.,0.,'odom',t),0.,t,t)
        self.assertEqual(c.phase,'INCONCLUSIVE')

    def test_missing_images_is_inconclusive(self):
        c=self.observing(); start=c.observe_started
        for i in range(1,80):
            t=start+i*.05
            c.detections(t,t,[]); c.motion((-1.5,0.,0.,'odom',t),0.,t,t)
        self.assertEqual(c.phase,'INCONCLUSIVE')

    def test_black_camera_is_inconclusive(self):
        self.assertEqual(self.feed_window(self.observing(), image_valid=False),'INCONCLUSIVE')

    def test_gap_during_observation_cannot_produce_pass(self):
        c=self.observing(); start=c.observe_started
        for i in range(1,80):
            t=start+i*.05
            if not 10<=i<=20: c.image(t,t); c.detections(t,t,[])
            c.motion((-1.5,0.,0.,'odom',t),0.,t,t)
        self.assertEqual(c.phase,'INCONCLUSIVE')

    def test_reverse_jump_does_not_fake_distance(self):
        c=RecoveryVerification();c.begin_reverse((0,0,0,'odom',10),10)
        self.assertEqual(c.motion((-1,0,0,'odom',10.05),-.25,10.05,10.05),0)
        self.assertEqual(c.phase,'INCONCLUSIVE')

    def test_motion_during_observation_is_not_pass(self):
        c=self.observing();t=c.observe_started+.1
        c.motion((-1.5,0,0,'odom',t),.1,t,t)
        self.assertEqual(c.phase,'INCONCLUSIVE')

    def test_no_reverse_progress_times_out_without_success(self):
        c=RecoveryVerification();c.begin_reverse((0,0,0,'odom',10),10)
        c.motion((0,0,0,'odom',26),0,26,26)
        self.assertEqual(c.phase,'INCONCLUSIVE')

    def test_rear_obstacle_and_invalid_scan(self):
        scan=Obj(header=Obj(frame_id='base_link'),ranges=[math.inf]*361,
                 angle_min=-math.pi,angle_increment=math.pi/180,range_min=.2,range_max=8.1)
        body=[[-.8,-.4],[.8,.4]]
        self.assertTrue(rear_corridor_clear(scan,body)[0])
        scan.ranges[0]=1.
        self.assertFalse(rear_corridor_clear(scan,body)[0])
        scan.ranges=[math.nan]*361
        self.assertFalse(rear_corridor_clear(scan,body)[0])


class CompletionHookTest(unittest.TestCase):
    def test_stable_residual_in_former_top_or_bottom_mask_is_not_passed(self):
        from autolabor_fod_msgs.msg import FodDetection, FodDetectionArray
        for y in (30, 650):
            with self.subTest(y=y):
                n = self.node()
                n.verification = VerificationTest().observing()
                n.gps_fallback_enabled = False
                n.allowed_class_names, n.min_confidence = {'Metal'}, .12
                n.select_nearest_by_depth = True
                n._maybe_start_recovery = mock.Mock()
                start = n.verification.observe_started
                for i in range(1, 46):
                    t = start+i*.05
                    msg = FodDetectionArray()
                    msg.header.stamp = self.module.rospy.Time.from_sec(t)
                    msg.image_width, msg.image_height = 1280, 720
                    item = FodDetection()
                    item.class_name, item.confidence, item.track_id = 'Metal', .8, 1
                    item.bbox.x_offset, item.bbox.y_offset = 600, y
                    item.bbox.width, item.bbox.height = 30, 30
                    item.anchor_px.x, item.anchor_px.y = 615, y+30
                    # Missing depth must still count as residual-image evidence.
                    item.depth_valid = False
                    msg.detections = [item]
                    with mock.patch.object(self.module.rospy.Time, 'now', return_value=msg.header.stamp), \
                            mock.patch.object(self.module.time, 'monotonic', return_value=t):
                        n._detections_cb(msg)
                    n.verification.image(t, t)
                    n.verification.motion((-1.5, 0., 0., 'odom', t), 0., t, t)
                self.assertEqual(n.verification.phase, 'FOD_REMAINS')

    @classmethod
    def setUpClass(cls):
        spec=importlib.util.spec_from_file_location('verification_mode_test',Path(__file__).resolve().parents[1]/'scripts/fod_navigation_mode_manager.py')
        cls.module=importlib.util.module_from_spec(spec);spec.loader.exec_module(cls.module)

    def node(self, enabled=True):
        n=self.module.FodNavigationModeManager.__new__(self.module.FodNavigationModeManager)
        n.lock=threading.RLock();n.verification=RecoveryVerification()
        n.verification_this_run=enabled;n.verification_enabled=enabled;n.recovery_armed=False
        n.recovery_generation=1
        n._publish_zero_locked=mock.Mock();n._publish_status=mock.Mock()
        return n

    def test_default_startup_accepts_latched_state_without_starting_recovery(self):
        def subscribe(topic, message_type, callback, **kwargs):
            if topic == '/fod_visual_servo/state':
                callback(Obj(data='DISABLED'))
            return mock.Mock()
        rospy=self.module.rospy
        with mock.patch.object(rospy,'get_param',side_effect=lambda name,default=None:default), \
                mock.patch.object(rospy,'get_name',return_value='/mode_test'), \
                mock.patch.object(self.module.rosgraph,'Master'), \
                mock.patch.object(rospy,'Publisher') as publisher, \
                mock.patch.object(rospy,'Subscriber',side_effect=subscribe), \
                mock.patch.object(rospy,'Service'), mock.patch.object(rospy,'Timer'), \
                mock.patch.object(rospy,'on_shutdown'), \
                mock.patch.object(rospy.Time,'now',return_value=rospy.Time.from_sec(10.)):
            n=self.module.FodNavigationModeManager()
        self.assertTrue(n.verification_enabled)
        self.assertFalse(n.verification_this_run)
        self.assertFalse(n.recovery_armed)
        self.assertEqual(n.verification.phase,'IDLE')
        self.assertEqual(n.visual_state,'DISABLED')
        self.assertEqual(n.mode,'GPS_ACTIVE')

    def test_feature_off_keeps_original_completion(self):
        n=self.node(False);n._recovery_finished_locked('normal')
        self.assertEqual(n.mode,'FOD_COMPLETE_STOP');self.assertEqual(n.verification.phase,'IDLE')

    def test_visual_completion_prepares_one_check_and_waits_for_cleanup(self):
        n=self.node();n.mode='FOD_ACTIVE'
        with mock.patch.object(self.module.threading,'Thread') as worker:
            n._visual_state_cb(Obj(data='COMPLETE'))
            self.assertEqual(n.mode,'FOD_VERIFY')
            self.assertEqual(n.verification.phase,'PREPARING')
            self.assertIsNone(n.verification.start_pose)
            n._visual_state_cb(Obj(data='COMPLETE'))
            worker.assert_called_once()
        n._publish_zero_locked.assert_called_once()

    def test_visual_abort_only_cleans_up_without_reversing(self):
        n=self.node();n.mode='FOD_ACTIVE'
        with mock.patch.object(self.module.threading,'Thread') as worker:
            n._visual_state_cb(Obj(data='ABORT'))
            worker.assert_called_once()
        self.assertEqual(n.mode,'FOD_ABORTED')
        self.assertEqual(n.verification.phase,'IDLE')

    def test_plain_gps_arrival_does_not_reverse(self):
        # Even a retained verification flag from the previous pickup cannot
        # turn a plain, unarmed GPS route into a new reverse manoeuvre.
        n=self.node();n.mode='GPS_ACTIVE'
        n.gps_fallback_enabled=False;n.gps_route={'active':True}
        n._maybe_start_recovery=mock.Mock()
        with mock.patch.object(self.module.threading,'Thread') as worker:
            n._gps_route_cb(Obj(data=json.dumps({'active':False,'state':'COMPLETE'})))
            worker.assert_not_called()
        self.assertEqual(n.mode,'GPS_ACTIVE')
        self.assertEqual(n.verification.phase,'IDLE')
        n._publish_zero_locked.assert_not_called()

    def test_canceled_or_failed_gps_recovery_does_not_reverse(self):
        for state in ('CANCELED','ABORTED','SUPERSEDED','KEEPOUT_BLOCKED'):
            with self.subTest(state=state):
                n=self.node();n.mode='GPS_ACTIVE';n.recovery_armed=True
                n.gps_fallback_enabled=False;n.gps_route={'active':True}
                n._maybe_start_recovery=mock.Mock()
                with mock.patch.object(self.module.threading,'Thread') as worker:
                    n._gps_route_cb(Obj(data=json.dumps({'active':False,'state':state})))
                    worker.assert_called_once()
                self.assertFalse(n.recovery_armed)
                self.assertEqual(n.verification.phase,'IDLE')

    def test_old_completion_and_option_enable_do_not_reverse(self):
        n=self.node();n.mode='FOD_COMPLETE_STOP'
        with mock.patch.object(self.module.rospy,'set_param'):
            r=n._set_verification_enabled_cb(Obj(data=True))
        self.assertTrue(r.success);self.assertEqual(n.verification.phase,'IDLE')
        n._publish_zero_locked.assert_not_called()

    def test_failed_cleanup_is_not_forgotten_on_second_call(self):
        n=self.node();n.sweep_started_for_recovery=True;n.sweep_auto_stop=True
        n._set_sweep=mock.Mock(return_value=(False,'missing ACK'))
        with mock.patch.object(self.module.rospy,'logerr'):
            self.assertFalse(n._stop_recovery_sweep())
            self.assertFalse(n._stop_recovery_sweep())
        self.assertEqual(n._set_sweep.call_args_list, [mock.call(False), mock.call(False)])
        self.assertTrue(n.sweep_started_for_recovery)
        self.assertTrue(n.sweep_cleanup_pending)
        n._recovery_finished_locked('done');n._start_verification_after_finish(False)
        self.assertEqual(n.verification.phase,'INCONCLUSIVE')

    def test_cancel_check_stops_without_resuming_gps(self):
        n=self.node();n._recovery_finished_locked('done')
        n._pause_navigation_route=mock.Mock()
        with mock.patch.object(self.module.rospy,'set_param'):
            r=n._set_verification_enabled_cb(Obj(data=False))
        self.assertTrue(r.success);self.assertEqual(n.verification.phase,'CANCELED')
        self.assertEqual(n.mode,'FOD_COMPLETE_STOP');self.assertFalse(n.verification_this_run)
        n._pause_navigation_route.assert_not_called()


if __name__ == '__main__':
    unittest.main()
