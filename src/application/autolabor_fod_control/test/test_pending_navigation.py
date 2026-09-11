#!/usr/bin/env python3
"""Offline regression tests: no init_node, ROS service, or real command."""
import copy
import importlib.util
import math
import pathlib
import sys
import threading
import time
import unittest
from types import SimpleNamespace as NS
from unittest import mock

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid
from autolabor_fod_control.pending_litter import (
    approach_candidates, footprint_is_free, goal_subscriber_error,
    matches_visual_completion, target_geometry,
)

PATH = pathlib.Path(__file__).resolve().parents[1]/'scripts/pending_litter_cleaner_node.py'
SPEC = importlib.util.spec_from_file_location('pending_navigation_tests_node', PATH)
NODE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = NODE
SPEC.loader.exec_module(NODE)


class GeometryTests(unittest.TestCase):
    def test_all_candidates_face_target_inside_range_including_near_behind(self):
        for robot in ((0, 0, 0), (0, 0, math.pi), (1, 0, 0)):
            poses = list(approach_candidates(robot, 1, 0))
            self.assertEqual(len(poses), 8)
            for pose in poses:
                distance, bearing = target_geometry(pose, 1, 0)
                self.assertAlmostEqual(distance, 1.5)
                self.assertAlmostEqual(bearing, 0)

    def test_bad_geometry_rejected(self):
        for radius in (0, 0.9, 2.1, math.nan):
            with self.assertRaises(ValueError):
                list(approach_candidates((0,0,0), 3, 4, radius))

    def make_grid(self):
        grid = OccupancyGrid()
        grid.info.width = grid.info.height = 40
        grid.info.resolution = .1
        grid.info.origin.orientation.w = 1.
        grid.data = [0]*1600
        return grid

    def test_entire_footprint_not_only_center_is_checked(self):
        grid = self.make_grid()
        shape = [(-.5,-.3), (.5,-.3), (.5,.3), (-.5,.3)]
        self.assertTrue(footprint_is_free(grid, (2,2,0), shape))
        for cost in (-1, 99, 100):
            grid.data[22*40+24] = cost
            self.assertFalse(footprint_is_free(grid, (2,2,0), shape))
        grid.data[22*40+24] = 0
        self.assertFalse(footprint_is_free(grid, (.1,.1,0), shape))

    def test_footprint_rotation_and_interior(self):
        grid = self.make_grid()
        shape = [(-.5,-.1), (.5,-.1), (.5,.1), (-.5,.1)]
        grid.data[24*40+20] = 100
        self.assertTrue(footprint_is_free(grid, (2,2,0), shape))
        self.assertFalse(footprint_is_free(grid, (2,2,math.pi/2), shape))
        grid.data[24*40+20] = 0
        grid.data[20*40+20] = 100
        self.assertFalse(footprint_is_free(grid, (2,2,0), shape))

    def test_bad_grid_rejected(self):
        grid = self.make_grid()
        grid.data = []
        self.assertFalse(footprint_is_free(grid, (2,2,0), [(-1,-1),(1,-1),(0,1)]))


class OwnershipTests(unittest.TestCase):
    def node(self):
        node = NODE.PendingLitterCleaner.__new__(NODE.PendingLitterCleaner)
        node.lock = threading.RLock()
        node.expected_bridge_node = '/navigation_pause'
        node.simple_goal_topic = '/move_base_simple/goal'
        node.allowed_goal_observers = ['/autolabor_operator_gui', '/rviz']
        node.expected_pose = PoseStamped()
        node.expected_pose.header.seq = 412
        node.expected_pose.header.frame_id = 'map'
        node.expected_pose.pose.position.x = 5
        node.accepted_goal_ids = set()
        return node

    def test_qt_display_is_not_a_second_controller(self):
        self.assertEqual(goal_subscriber_error(['/navigation_pause','/autolabor_operator_gui'],
                                              '/navigation_pause', ['/autolabor_operator_gui']), '')

    def test_unknown_subscriber_or_missing_bridge_rejected(self):
        for nodes in ([], ['/autolabor_operator_gui'], ['/navigation_pause','/unknown']):
            self.assertTrue(goal_subscriber_error(nodes, '/navigation_pause', ['/autolabor_operator_gui']))

    def test_graph_failure_rejects(self):
        node = self.node()
        node.master = mock.Mock()
        node.master.getSystemState.side_effect = OSError('offline')
        self.assertTrue(node._goal_route_error())

    def test_only_exact_pose_token_from_bridge_can_be_adopted(self):
        node = self.node()
        message = MoveBaseActionGoal()
        message._connection_header = {'callerid':'/navigation_pause'}
        message.goal.target_pose = copy.deepcopy(node.expected_pose)
        message.goal_id.id = 'goal-a'
        node._action_goal_cb(message)
        self.assertEqual(node.accepted_goal_ids, {'goal-a'})
        for kind in ('caller', 'sequence', 'position', 'frame'):
            changed = copy.deepcopy(message)
            changed.goal_id.id = kind
            if kind == 'caller': changed._connection_header = {'callerid':'/other'}
            if kind == 'sequence': changed.goal.target_pose.header.seq += 1
            if kind == 'position': changed.goal.target_pose.pose.position.x += 1
            if kind == 'frame': changed.goal.target_pose.header.frame_id = 'odom'
            node._action_goal_cb(changed)
        self.assertEqual(node.accepted_goal_ids, {'goal-a'})

    def test_foreign_navigation_cannot_be_preempted(self):
        node = self.node()
        node.navigation_status_receipt = time.monotonic()
        node.status_timeout_sec = 1.
        node.navigation_status = {'active_ordinary_goal_ids':['manual']}
        with self.assertRaises(RuntimeError): node._assert_navigation_available('own')
        node.navigation_status = {'retained_reissue_allowed':True}
        with self.assertRaises(RuntimeError): node._assert_navigation_available()


class MissionTests(unittest.TestCase):
    def make_node(self, pose):
        node = NODE.PendingLitterCleaner.__new__(NODE.PendingLitterCleaner)
        node.lock = threading.RLock()
        node.cancel_event = threading.Event()
        node.active = NS(object_id=42, target_x=1., target_y=0., request_id='a'*32)
        node.goal_id = ''
        node.stand_off_m = 1.5
        node.min_visual_start_distance_m = 1.
        node.max_visual_start_distance_m = 2.
        node.max_visual_start_bearing_deg = 35.
        node.visual_mode_requested = node.visual_staged = False
        node._map_pose = lambda: pose
        for method in ('_submit_stand_off_goal','_set_state','_wait_stopped','_stage_visual_target',
                       '_enable_visual_mode','_wait_visual_complete','_publish_status',
                       '_cancel_exact_goal','_disable_visual_mode','_clear_visual_target'):
            setattr(node, method, mock.Mock())
        node.completed_pub = mock.Mock()
        return node

    def test_near_behind_target_navigates_instead_of_aborting(self):
        node = self.make_node((0,0,math.pi))
        node._run_mission(node.active)
        node._submit_stand_off_goal.assert_called_once()
        self.assertEqual(node.state, 'COMPLETE')

    def test_near_front_target_still_waits_for_stop(self):
        node = self.make_node((0,0,0))
        order = []
        node._wait_stopped.side_effect = lambda _: order.append('stop')
        node._stage_visual_target.side_effect = lambda _: order.append('stage')
        node._enable_visual_mode.side_effect = lambda _: order.append('enable')
        node._run_mission(node.active)
        node._submit_stand_off_goal.assert_not_called()
        self.assertEqual(order, ['stop','stage','enable'])

    def test_too_close_target_also_requires_repositioning(self):
        node = self.make_node((.5,0,0))
        node._run_mission(node.active)
        node._submit_stand_off_goal.assert_called_once()

    def test_navigation_failure_never_enables_or_disables_other_visual_task(self):
        node = self.make_node((-5,0,0))
        node._submit_stand_off_goal.side_effect = RuntimeError('no path')
        node._run_mission(node.active)
        self.assertEqual(node.state, 'ABORT')
        node._enable_visual_mode.assert_not_called()
        node._disable_visual_mode.assert_not_called()
        node.completed_pub.publish.assert_not_called()

    def test_missing_stop_evidence_never_enables_visual(self):
        node = self.make_node((0,0,0))
        node._wait_stopped.side_effect = RuntimeError('no stop feedback')
        node._run_mission(node.active)
        node._enable_visual_mode.assert_not_called()
        self.assertEqual(node.state, 'ABORT')


class CompletionTests(unittest.TestCase):
    def test_visual_completion_is_bound_to_new_session_and_original_list_identity(self):
        proof = dict(sequence=3, object_id='42', acquired_object_id='77', request_id='token',
                     completion_policy='visual_servo', stopped=True, target_x=1., target_y=2.,
                     forward_m=.51, path_m=.52, blind_target_m=.5)
        self.assertTrue(matches_visual_completion(proof, 2, 42, 1., 2., 'token'))
        for change in ({'sequence':2}, {'request_id':'old'}, {'object_id':'77'}, {'stopped':False},
                       {'forward_m':.2}, {'path_m':.56}, {'target_x':math.nan},
                       {'completion_policy':''}, {'acquired_object_id':'0'}):
            self.assertFalse(matches_visual_completion(dict(proof, **change), 2, 42, 1., 2., 'token'))
        self.assertTrue(matches_visual_completion(dict(proof, blind_target_m=.3, forward_m=.31, path_m=.32),
                                                 2, 42, 1., 2., 'token'))


if __name__ == '__main__': unittest.main()
