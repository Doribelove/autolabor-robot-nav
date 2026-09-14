#!/usr/bin/env python3
"""New completion contract, using the real node methods without ROS motion."""
from collections import deque
from types import SimpleNamespace
import math
import threading
import unittest
from unittest import mock

import test_visual_servo as fixtures
FOD_NODE = fixtures.FOD_NODE
from autolabor_fod_control.visual_servo import BlindProgress
from autolabor_fod_control.visual_session import VisualSessionReceipt
from sweeper_mcp.visual_completion import visual_completed


class SimpleVisualCompletionTest(unittest.TestCase):
    def node(self):
        node = fixtures.ApprovedBottomGateTest.configured_node()
        node.sensor_lock = threading.Lock()
        node.latest_odom = SimpleNamespace(sequence=10, x=5.0, y=2.0, yaw=0.0,
                                         linear_x=0.2, angular_z=0.0,
                                         receipt_monotonic=100.0, stamp_sec=200.0)
        node.odom_queue = deque()
        node.staged_target = None
        node.coverage_progress = None
        node._set_motion = mock.Mock()
        node._hard_stop = mock.Mock()
        node.visual_session = VisualSessionReceipt()
        node.visual_session.begin()
        node.last_cleaning_completion = {}
        return node

    def test_baseline_is_current_odometry_after_tenth_miss(self):
        node = self.node()
        node.odom_queue.append(SimpleNamespace(x=4.7, y=2.0))
        node._begin_forward_pass()
        progress = node.blind_tracker.update(5.01, 2.0, 0.0, .05)
        self.assertAlmostEqual(progress.forward_m, .01)
        self.assertEqual(node.last_processed_odom_sequence, 10)
        self.assertFalse(node.odom_queue)

    def test_reaching_point_three_requests_completion_without_stop_or_wheel_gate(self):
        for distance in (.299, .30, .31):
            with self.subTest(distance=distance):
                node = self.node()
                node._begin_forward_pass()
                progress = BlindProgress(distance, distance, .10, math.radians(8))
                node._update_blind_odometry = mock.Mock(return_value=progress)
                node._complete_forward_pass = mock.Mock()
                node._blind_advance_tick()
                self.assertEqual(node._complete_forward_pass.called, distance >= .30)
                if distance < .30:
                    node._set_motion.assert_called_once_with(.20, 0.0)

    def test_odom_fault_cannot_be_converted_to_success(self):
        node = self.node()
        node._begin_forward_pass()
        node._update_blind_odometry = mock.Mock(
            side_effect=FOD_NODE.ControllerAbort("invalid odometry"))
        node._complete_forward_pass = mock.Mock()
        with self.assertRaises(FOD_NODE.ControllerAbort):
            node._blind_advance_tick()
        node._complete_forward_pass.assert_not_called()

    def test_stop_metadata_does_not_claim_stopped_until_new_physical_feedback(self):
        node = self.node()
        node.completed_odom_sequence = 10
        node.visual_session.complete(stopped=False)
        session_id = node.visual_session.session_id
        result = dict(ai_completion_protocol=1, visual_session_id=session_id,
                      visual_result="COMPLETE", visual_state="DISABLED",
                      visual_active=False, state="RELATIVE_NAV_ACTIVE",
                      completion=node.visual_session.last_completion)
        with mock.patch.object(FOD_NODE.time, "monotonic", return_value=100.1), \
                mock.patch.object(FOD_NODE.rospy.Time, "now") as now:
            now.return_value.to_sec.return_value = 200.1
            node._confirm_completion_stopped()
            self.assertFalse(visual_completed(result, session_id, 0))
            node.latest_odom.sequence = 11
            node._confirm_completion_stopped()  # still moving
            self.assertFalse(visual_completed(result, session_id, 0))
            node.latest_odom.linear_x = 0.0
            node.visual_session.disable()
            node._confirm_completion_stopped()
            self.assertTrue(visual_completed(result, session_id, 0))
            self.assertEqual(node.visual_session.sequence, 1)

    def test_stale_stop_feedback_never_releases_ai(self):
        node = self.node()
        node.completed_odom_sequence = 9
        node.visual_session.complete(stopped=False)
        node.latest_odom.linear_x = 0.0
        with mock.patch.object(FOD_NODE.time, "monotonic", return_value=105.0):
            node._confirm_completion_stopped()
        self.assertFalse(node.visual_session.last_completion["stopped"])

    def test_cancelled_session_does_not_promote_an_old_receipt(self):
        node = self.node()
        node.visual_session.complete(stopped=False)
        old_id = node.visual_session.session_id
        node.visual_session.begin()
        node.visual_session.disable()
        node._confirm_completion_stopped()
        self.assertEqual(node.visual_session.result, "CANCELLED")
        self.assertEqual(node.visual_session.last_completion["session_id"], old_id)
        self.assertFalse(node.visual_session.last_completion["stopped"])


if __name__ == "__main__":
    unittest.main()
