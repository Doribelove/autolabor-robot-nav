#!/usr/bin/env python3
import math
import contextlib
import io
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as Obj
import unittest
from unittest import mock
from camera_line_recovery_trial import CycleEvidence, line_progress, main


class LineSequenceTest(unittest.TestCase):
    def complete_mode(self):
        return dict(state='FOD_COMPLETE_STOP', sweep_cleanup_pending=False,
                    sweep_started_for_recovery=False, sweep_cleanup_confirmed=True,
                    measured_linear_x=0.0)

    def ready(self):
        c = CycleEvidence(10.)
        c.mode(dict(state='ENTERING_FOD'), 11.)
        c.visual(dict(state='APPROACH', target_locked=True, target_track_id=3), 12.)
        c.visual(dict(state='COMPLETE'), 13.)
        return c

    def test_projection_follows_start_heading_not_world_x(self):
        f, l = line_progress((3, 5, math.pi/2), (2.9, 17., 0))
        self.assertAlmostEqual(f, 12.)
        self.assertAlmostEqual(l, .1)

    def test_old_latched_completion_cannot_count_as_a_new_target(self):
        c = CycleEvidence(10.)
        c.mode(dict(state='FOD_ACTIVE'), 9.)
        c.visual(dict(state='COMPLETE'), 11.)
        self.assertFalse(c.finished(self.complete_mode(), 12.))

    def test_crossing_then_stopped_cleanup_is_required(self):
        c = self.ready()
        self.assertTrue(c.finished(self.complete_mode(), 14.))
        for bad in [dict(sweep_cleanup_pending=True), dict(sweep_cleanup_confirmed=False),
                    dict(sweep_started_for_recovery=True), dict(measured_linear_x=.1),
                    dict(state='FOD_ABORTED')]:
            self.assertFalse(c.finished(dict(self.complete_mode(), **bad), 14.))

    def test_next_cycle_does_not_inherit_completed_or_target_identity(self):
        old = self.ready()
        self.assertEqual(old.target_ids, {3})
        new = CycleEvidence(20.)
        new.mode(dict(state='ENTERING_FOD'), 21.)
        self.assertFalse(new.finished(self.complete_mode(), 22.))
        self.assertFalse(new.target_ids)

    def run_coordinator(self, abort_second=False, exceed_line=False):
        # Exercise the actual executable without a ROS master or hardware.
        from nav_msgs.msg import Odometry
        callbacks, calls, count = {}, [], [0]
        initial = dict(self.complete_mode(), verification_enabled=True)
        def send(topic, value):
            callbacks[topic](Obj(data=json.dumps(value)))
        def subscribe(topic, kind, callback, **kwargs):
            callbacks[topic] = callback
            return Obj(unregister=lambda: None)
        def wait(topic, kind, **kwargs):
            if topic == '/odom':
                value = Odometry(); value.pose.pose.orientation.w = 1.
                return value
            return Obj(data=json.dumps(initial if topic.endswith('/status') and 'fod_' in topic else {'active':False}))
        def service(name, kind):
            def call(enabled):
                calls.append((name, enabled))
                if 'set_direct' in name and enabled:
                    count[0] += 1
                    send('/fod_navigation_mode/status', dict(state='ENTERING_FOD'))
                    send('/fod_visual_servo/status', dict(state='APPROACH', target_locked=True, target_track_id=count[0]))
                    pose = Odometry(); pose.pose.pose.orientation.w = 1.
                    pose.pose.pose.position.x = 11.9 if exceed_line and count[0] == 2 else 3.*count[0]
                    callbacks['/odom'](pose)
                    send('/fod_visual_servo/status', dict(state='COMPLETE'))
                    terminal = dict(initial)
                    if abort_second and count[0] == 2:
                        terminal.update(state='FOD_ABORTED', reason='operator stopped')
                    send('/fod_navigation_mode/status', terminal)
                return Obj(success=True, message='simulated service')
            return call
        ros = Obj(init_node=lambda *a,**k:None, Subscriber=subscribe, wait_for_message=wait,
                  ServiceProxy=service, is_shutdown=lambda:False,
                  Time=Obj(now=lambda:Obj(to_nsec=lambda:123000000000)))
        with tempfile.TemporaryDirectory() as directory:
            with mock.patch.dict(sys.modules, {'rospy':ros}), mock.patch('signal.signal'), \
                 mock.patch.object(sys, 'argv', ['line_trial','--execute','--audit-dir',directory]), \
                 contextlib.redirect_stdout(io.StringIO()):
                main()
            result = json.loads((Path(directory)/'sequence_result.json').read_text())
        return calls, result

    def test_three_cycles_stop_and_restore_single_target_reverse_default(self):
        calls, result = self.run_coordinator()
        self.assertEqual(result['result'], 'three_crossings_completed_and_stopped')
        self.assertEqual(len(result['completed']), 3)
        self.assertEqual([c[1] for c in calls if 'set_direct' in c[0]], [True]*3)
        self.assertEqual([c[1] for c in calls if 'set_verification' in c[0]], [False,True])

    def test_manual_abort_does_not_start_the_third_target(self):
        calls, result = self.run_coordinator(abort_second=True)
        self.assertEqual(result['result'], 'stopped_incomplete')
        self.assertEqual(len(result['completed']), 1)
        self.assertEqual([c[1] for c in calls if 'set_direct' in c[0]], [True,True,False])
        self.assertEqual(calls[-1][1], True)  # Restore verification, never resume GPS.

    def test_line_limit_stops_this_trial_instead_of_following_a_far_fod(self):
        calls, result = self.run_coordinator(exceed_line=True)
        self.assertEqual(result['result'], 'stopped_incomplete')
        self.assertEqual(len(result['completed']), 1)
        self.assertEqual([c[1] for c in calls if 'set_direct' in c[0]], [True,True,False])


if __name__ == '__main__':
    unittest.main()
