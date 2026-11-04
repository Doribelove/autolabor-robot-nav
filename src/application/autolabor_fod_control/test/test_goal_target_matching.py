#!/usr/bin/env python3
"""Original-goal matching regressions; no ROS master or vehicle commands."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import threading
import unittest
from types import SimpleNamespace
from unittest import mock

import rospy
from autolabor_fod_msgs.msg import FodDetection, FodDetectionArray
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String

from autolabor_fod_control.gps_recovery import FodGpsMemory, point_matches_recovery_goal
from autolabor_fod_control.visual_servo import PixelDetection


def load_node(name, filename):
    spec = importlib.util.spec_from_file_location(
        name, Path(__file__).resolve().parents[1] / 'scripts' / filename)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


MODE = load_node('goal_matching_mode', 'fod_navigation_mode_manager.py')
VISUAL = load_node('goal_matching_visual', 'fod_visual_servo_node.py')


def observation(identity=1, x=4., y=0., valid=True):
    d = FodDetection()
    d.track_id, d.class_name, d.class_id, d.confidence = identity, 'Metal', 0, .9
    u, v = 635 - 523*y/x, 366 + 523*.45/x
    d.bbox.x_offset, d.bbox.y_offset = round(u)-10, round(v)-10
    d.bbox.width, d.bbox.height = 20, 20
    d.anchor_px.x, d.anchor_px.y = u, v+10
    d.depth_valid = valid
    d.depth_m = x if valid else 0.
    d.range_m = math.sqrt(x*x+y*y+.45**2) if valid else 0.
    return d


class GoalMatchingTest(unittest.TestCase):
    def setUp(self):
        n = self.node = MODE.FodNavigationModeManager.__new__(MODE.FodNavigationModeManager)
        n.lock = threading.RLock()
        n.gps_fallback_enabled, n.recovery_armed = True, True
        n.mode = MODE.GPS_ACTIVE
        n.detection_stamp = None
        n.allowed_class_names, n.min_confidence = {'Metal', 'Plastic'}, .12
        n.select_nearest_by_depth = True
        n.fod_gps_memory = FodGpsMemory(frames=3)
        n.fod_gps_point, n.fod_reference_goal = None, None
        n.fod_approach_started = False
        n.gps_pose, n.gps_origin = (0., 0., 'camera_init'), (30.3, 104.3, 0.)
        n.gps_route = dict(active=True, paused=False, route_id='original',
                           frame_id='camera_init', final_x=4.2, final_y=0.)
        n.recovery_generation, n.recovery_radius_m = 0, 10.
        n.vehicle_footprint = [[.8,.4],[.8,-.4],[-.8,-.4],[-.8,.4]]
        n._recovery_distance_locked = mock.Mock(return_value=4.2)
        n._maybe_start_recovery = mock.Mock()
        n._maybe_fallback_to_gps = mock.Mock()
        n._publish_zero_locked = mock.Mock()
        n.camera_info = CameraInfo()
        n.camera_info.header.frame_id = 'optical'
        n.camera_info.width, n.camera_info.height = 1280, 720
        n.camera_info.P = [523.,0.,635.,0.,0.,523.,366.,0.,0.,0.,1.,0.]
        tf = TransformStamped()
        tf.transform.translation.z = .45
        q = tf.transform.rotation
        q.x, q.y, q.z, q.w = -.5, .5, -.5, .5
        n.tf_buffer = mock.Mock()
        n.tf_buffer.lookup_transform.return_value = tf
        n.recovery_sweep_ready = True
        n.recovery_cancel_requested = threading.Event()
        n.gps_fallback_ack = threading.Event()

    def feed(self, detections, stamp):
        msg = FodDetectionArray()
        msg.header.stamp, msg.header.frame_id = rospy.Time.from_sec(stamp), 'optical'
        msg.image_width, msg.image_height = 1280, 720
        msg.detections = detections
        self.node._detections_cb(msg)

    def lock(self):
        for i in range(3):
            self.feed([observation()], 1+i*.04)
        self.assertIsNotNone(self.node.fod_gps_point)

    def test_radius_is_2d_inclusive_and_uses_recorded_pt5_false_positive(self):
        self.assertTrue(point_matches_recovery_goal((4.3, 0., 8.), (4., 0.)))
        self.assertFalse(point_matches_recovery_goal((4.30001, 0., 0.), (4., 0.)))
        self.assertFalse(point_matches_recovery_goal((4.25, .25, 0.), (4., 0.)))
        self.assertFalse(point_matches_recovery_goal((math.nan, 0.), (4., 0.)))
        # Exact world coordinates from pt5 attempt 9, 2026-10-17 10:40:19.
        self.assertFalse(point_matches_recovery_goal(
            (42.66105072483355, 38.41302735503278),
            (42.847302776709846, 43.315861020721336)))

    def test_closer_side_target_never_locks_slows_gps_or_becomes_fallback(self):
        for i in range(12):
            self.feed([observation(231, 2., -1.)], 1+i*.04)
        self.assertIsNone(self.node.fod_gps_point)
        self.assertFalse(self.node.fod_approach_started)
        self.assertFalse(self.node._has_fod_locked())
        self.assertFalse(self.node._prepare_gps_fallback_locked())
        self.assertTrue(self.node.gps_route['active'])
        self.assertEqual(self.node.mode, MODE.GPS_ACTIVE)

    def test_near_goal_target_wins_even_with_closer_side_target(self):
        for i in range(3):
            self.feed([observation(231, 2., -1.), observation()], 1+i*.04)
        self.assertEqual(self.node.fod_gps_point['track_id'], 1)
        self.assertTrue(self.node.fod_approach_started)
        self.assertAlmostEqual(self.node.fod_gps_point['fod_x'], 4.)

    def test_unknown_depth_never_acquires_but_keeps_verified_target_visible(self):
        for i in range(3):
            self.feed([observation(valid=False)], .1+i*.04)
        self.assertIsNone(self.node.fod_gps_point)
        self.lock()
        old = dict(self.node.fod_gps_point)
        self.feed([observation(valid=False)], 1.2)
        self.assertAlmostEqual(self.node.fod_gps_memory.last_seen_stamp, 1.2)
        self.assertEqual(self.node.fod_gps_point, old)

    def test_same_id_outside_goal_cannot_refresh_visibility_or_refine_memory(self):
        self.lock()
        old, seen = dict(self.node.fod_gps_point), self.node.fod_gps_memory.last_seen_stamp
        for i in range(4):
            self.feed([observation(x=3.7)], 2+i*.04)
        self.assertEqual(self.node.fod_gps_point, old)
        self.assertEqual(self.node.fod_gps_memory.last_seen_stamp, seen)

    def test_crossing_endpoint_is_not_clipped_to_the_object_radius(self):
        self.lock()
        self.feed([], 3.)
        self.assertTrue(self.node._prepare_gps_fallback_locked())
        self.assertAlmostEqual(self.node.gps_fallback_target['goal_x'], 4.8)
        self.assertEqual(self.node.fod_reference_goal['x'], 4.2)

    def test_paused_route_keeps_original_reference_for_visual_tracking(self):
        self.lock()
        self.node.mode, self.node.recovery_armed = MODE.FOD_ACTIVE, False
        self.node.gps_route['paused'] = True
        self.feed([observation()], 1.2)
        self.assertEqual(self.node.fod_reference_goal['x'], 4.2)
        self.assertAlmostEqual(self.node.fod_gps_memory.last_seen_stamp, 1.2)

    def test_loss_during_visual_acquisition_uses_verified_memory_after_grace(self):
        self.lock()
        self.node.mode, self.node.recovery_armed = MODE.FOD_ACTIVE, False
        self.node.visual_state = 'ACQUIRE'
        self.node.target_loss_grace_sec = 1.
        self.node.proximity_worker_started = False
        self.node.detection_stamp = 1.5
        with mock.patch.object(MODE.threading, 'Thread') as worker:
            MODE.FodNavigationModeManager._maybe_fallback_to_gps(self.node)
            worker.assert_not_called()
            self.node.detection_stamp = 2.2
            MODE.FodNavigationModeManager._maybe_fallback_to_gps(self.node)
            self.assertEqual(self.node.mode, MODE.ENTERING_GPS_RECOVERY)
            self.assertAlmostEqual(self.node.gps_fallback_target['goal_x'], 4.8)
            worker.return_value.start.assert_called_once()

    def test_replacement_goal_clears_previous_fod_and_reference(self):
        self.lock()
        route = dict(self.node.gps_route, route_id='next', final_x=8.)
        self.node._gps_route_cb(String(data=json.dumps(route)))
        self.assertIsNone(self.node.fod_gps_point)
        self.assertIsNone(self.node.fod_reference_goal)
        for i in range(3):
            self.feed([observation()], 2+i*.04)
        self.assertIsNone(self.node.fod_gps_point)
        self.assertFalse(self.node.fod_approach_started)

    def test_frame_mismatch_or_missing_final_goal_leaves_gps_in_control(self):
        for changes in ({'frame_id':'other'}, {'final_x':None}, {'active':False}):
            with self.subTest(changes=changes):
                self.setUp()
                self.node.gps_route.update(changes)
                for i in range(3):
                    self.feed([observation()], 1+i*.04)
                self.assertIsNone(self.node.fod_gps_point)
                self.assertFalse(self.node.fod_approach_started)
                self.assertEqual(self.node.mode, MODE.GPS_ACTIVE)

    def test_old_detection_cannot_commit_after_route_replacement(self):
        tf = self.node.tf_buffer.lookup_transform.return_value
        def replace_route(*args):
            self.node.recovery_generation += 1
            self.node.gps_route = dict(self.node.gps_route, route_id='next', final_x=8.)
            return tf
        self.node.tf_buffer.lookup_transform.side_effect = replace_route
        self.feed([observation()], 1.)
        self.assertIsNone(self.node.fod_reference_goal)
        self.assertIsNone(self.node.fod_gps_point)

    def test_visual_tracking_rejects_same_id_at_wrong_location(self):
        node = VISUAL.FodVisualServoNode.__new__(VISUAL.FodVisualServoNode)
        node.sensor_lock = threading.Lock()
        node.recovery_reference_goal = dict(frame_id='camera_init', x=4.2, y=0.)
        node.latest_camera = SimpleNamespace(frame_id='optical', projection=self.node.camera_info.P)
        node.steering_tf_buffer = self.node.tf_buffer
        observations = []
        for msg in (observation(), observation(x=3.7), observation(valid=False)):
            observations.append(PixelDetection(msg.class_id, msg.class_name, msg.confidence,
                msg.bbox.x_offset,msg.bbox.y_offset,msg.bbox.width,msg.bbox.height,
                msg.anchor_px.x,msg.anchor_px.y,track_id=msg.track_id,
                depth_valid=msg.depth_valid,depth_m=msg.depth_m,range_m=msg.range_m))
        frame = VISUAL.DetectionFrame(1, 1., 1., 1280, 720, tuple(observations), tuple(observations))
        filtered = node._filter_goal_observations(frame)
        self.assertEqual(filtered.observations, (observations[0], observations[2]))
        self.assertEqual(filtered.candidates, filtered.observations)
        node.recovery_reference_goal = None
        self.assertIs(node._filter_goal_observations(frame), frame)


if __name__ == '__main__':
    unittest.main()
