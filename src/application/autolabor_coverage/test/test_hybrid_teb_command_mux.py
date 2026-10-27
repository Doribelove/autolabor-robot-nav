#!/usr/bin/env python3

import importlib.util
import math
import os
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

from geometry_msgs.msg import Twist


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "hybrid_teb_command_mux.py",
)
SPEC = importlib.util.spec_from_file_location("hybrid_teb_command_mux", SCRIPT)
MUX = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MUX)


class HybridTebCommandMuxTest(unittest.TestCase):
    def _ready_mux(self):
        mux = MUX.HybridTebCommandMux.__new__(MUX.HybridTebCommandMux)
        mux.lock = threading.RLock()
        mux.closing = False
        mux.enabled = True
        mux.use_tf_pose = False
        mux.global_frame = 'map'
        mux.hybrid_active = False
        mux.path_signature = None
        mux.pose = (0.0, 0.0, 0.0)
        mux.pose_stamp = MUX.rospy.Time(100)
        mux.command_timeout = .4
        mux.safety_timeout = 1.5
        mux.minimum_turning_radius = 1.35
        mux.command_pub = mock.Mock()
        mux.error_pub = mock.Mock()
        mux.teb_command = Twist()
        mux.teb_command.linear.x = -.3
        mux.teb_stamp = MUX.rospy.Time(100)
        path = MUX.EnforcedPath()
        path.header.frame_id = path.path.header.frame_id = 'map'
        path.planner_mode = MUX.EnforcedPath.MODE_HYBRID_TRANSIT
        path.expected_gear = MUX.EnforcedPath.GEAR_FORWARD
        path.plan_id = 'same-task'
        path.path_generation = 2
        from geometry_msgs.msg import PoseStamped
        for x in [0., 1.]:
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.
            path.path.poses.append(pose)
        return mux, path

    def test_new_reference_discards_old_command_and_releases_first_valid_tick(self):
        mux, path = self._ready_mux()
        with mock.patch.object(MUX.rospy.Time, 'now', return_value=MUX.rospy.Time(100)), \
                mock.patch.object(MUX.rospy, 'is_shutdown', return_value=False):
            mux._path_callback(path)
            mux._safety_callback(SimpleNamespace(data=True))
            event = SimpleNamespace(current_real=MUX.rospy.Time(100))
            mux._update(event)
            self.assertEqual(0., mux.command_pub.publish.call_args[0][0].linear.x)
            fresh = Twist()
            fresh.linear.x = .3
            mux._teb_callback(fresh)
            mux._update(event)
            self.assertEqual(.3, mux.command_pub.publish.call_args[0][0].linear.x)
            mux.hybrid_path_safe = False
            mux._update(event)
            self.assertEqual(0., mux.command_pub.publish.call_args[0][0].linear.x)

    def test_leaving_hybrid_does_not_replay_previous_reverse_command(self):
        mux, path = self._ready_mux()
        mux.hybrid_active = True
        mux.path_signature = ('old', 0, 1, -1)
        path.planner_mode = MUX.EnforcedPath.MODE_ENFORCED_SWEEP
        with mock.patch.object(MUX.rospy.Time, 'now', return_value=MUX.rospy.Time(100)), \
                mock.patch.object(MUX.rospy, 'is_shutdown', return_value=False):
            mux._path_callback(path)
            event = SimpleNamespace(current_real=MUX.rospy.Time(100))
            mux._update(event)
            self.assertEqual(0., mux.command_pub.publish.call_args[0][0].linear.x)
            fresh = Twist()
            fresh.linear.x = .4
            mux._teb_callback(fresh)
            mux._update(event)
            self.assertEqual(.4, mux.command_pub.publish.call_args[0][0].linear.x)

    def test_fixed_gear_inference_rejects_mixed_path(self):
        self.assertEqual(1, MUX.infer_fixed_gear([
            (0.0, 0.0, 0.0), (1.0, 0.0, 0.0)
        ]))
        self.assertEqual(-1, MUX.infer_fixed_gear([
            (0.0, 0.0, 0.0), (-1.0, 0.0, 0.0)
        ]))
        with self.assertRaisesRegex(ValueError, "constant-gear"):
            MUX.infer_fixed_gear([
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0),
                (0.5, 0.0, 0.0),
            ])

    def test_teb_command_must_match_action_gear(self):
        command = Twist()
        command.linear.x = -0.3
        command.angular.z = 0.1
        checked = MUX.checked_teb_command(
            command, MUX.EnforcedPath.GEAR_REVERSE, 1.35
        )
        self.assertAlmostEqual(-0.3, checked.linear.x)
        with self.assertRaisesRegex(ValueError, "fixed-gear"):
            MUX.checked_teb_command(
                command, MUX.EnforcedPath.GEAR_FORWARD, 1.35
            )

    def test_teb_command_obeys_final_ackermann_curvature_bound(self):
        command = Twist()
        command.linear.x = 0.27
        command.angular.z = 0.60
        checked = MUX.checked_teb_command(
            command, MUX.EnforcedPath.GEAR_FORWARD, 1.35
        )
        self.assertTrue(math.isfinite(checked.angular.z))
        self.assertAlmostEqual(0.20, checked.angular.z)


if __name__ == "__main__":
    unittest.main()
