#!/usr/bin/env python3
"""Real coordinator + ownership bridge, synthetic navigator/visual backend.

Never run against a vehicle: the supplied wrapper isolates network and devices.
The named UI observer models the production RViz goal display subscription.
"""
import copy
import json
import math
import socket
import threading
import time
import unittest

import actionlib
import rospy
import rostest
import tf2_ros
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from nav_msgs.srv import GetPlan, GetPlanResponse
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse, Trigger
from autolabor_fod_msgs.srv import (
    SetFodCleaningTarget, SetFodCleaningTargetResponse,
    StartPendingLitterCleaning, StartPendingLitterCleaningRequest,
)


class PendingNavigationIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert [name for _,name in socket.if_nameindex()] == ['lo']
        cls.lock = threading.RLock()
        cls.pose = (0.,0.,0.)
        cls.mode = 'GPS_ACTIVE'
        cls.visual_state = 'DISABLED'
        cls.proof = {}
        cls.staged = None
        cls.pending_status = {}
        cls.nav_status = {}
        cls.goals = []
        cls.cancels = []
        cls.plans = []
        cls.mode_calls = []
        cls.hold_goal = False
        cls.block_plan = False
        cls.freeze_stop = False
        cls.visual_started = None
        cls.state_pub = rospy.Publisher('/fod_navigation_mode/state', String, queue_size=5)
        cls.visual_pub = rospy.Publisher('/fod_visual_servo/status', String, queue_size=5)
        cls.grid_pub = rospy.Publisher('/move_base/global_costmap/costmap', OccupancyGrid, queue_size=1)
        cls.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        cls.tf = tf2_ros.TransformBroadcaster()
        cls.observer = rospy.Subscriber('/move_base_simple/goal', PoseStamped, lambda _:None)
        cls.cancel_sub = rospy.Subscriber('/move_base/cancel', GoalID, lambda m: cls.cancels.append(m.id))
        cls.status_sub = rospy.Subscriber('/pending_litter_cleaner/status', String,
                                          lambda m: setattr(cls, 'pending_status', json.loads(m.data)))
        cls.nav_sub = rospy.Subscriber('/navigation_pause/status', String,
                                      lambda m: setattr(cls, 'nav_status', json.loads(m.data)))
        cls.plan_srv = rospy.Service('/move_base/make_plan', GetPlan, cls.make_plan)
        cls.target_srv = rospy.Service('/fod_visual_servo/set_cleaning_target', SetFodCleaningTarget, cls.stage)
        cls.mode_srv = rospy.Service('/fod_navigation_mode/set_fod_enabled', SetBool, cls.set_mode)
        cls.server = actionlib.SimpleActionServer('/move_base', MoveBaseAction, cls.execute, auto_start=False)
        cls.server.start()
        cls.timer = rospy.Timer(rospy.Duration(.05), cls.tick)
        rospy.wait_for_service('/pending_litter_cleaner/start_cleaning', timeout=10)
        cls.wait(lambda: cls.nav_status.get('action_status_received'), 'bridge ready')
        rospy.sleep(.6)

    @staticmethod
    def wait(predicate, reason, timeout=10):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if predicate(): return
            rospy.sleep(.03)
        raise AssertionError('timeout: '+reason)

    @classmethod
    def make_plan(cls, request):
        cls.plans.append(copy.deepcopy(request))
        path = Path()
        path.header.frame_id = 'map'
        if not cls.block_plan:
            path.poses = [request.start, request.goal]
        return GetPlanResponse(path)

    @classmethod
    def execute(cls, goal):
        cls.goals.append(copy.deepcopy(goal))
        started = time.monotonic()
        while not rospy.is_shutdown():
            if cls.server.is_preempt_requested():
                cls.server.set_preempted(MoveBaseResult())
                return
            if not cls.hold_goal and time.monotonic()-started > .4:
                p, q = goal.target_pose.pose.position, goal.target_pose.pose.orientation
                cls.pose = (p.x, p.y, math.atan2(2*q.w*q.z,1-2*q.z*q.z))
                cls.server.set_succeeded(MoveBaseResult())
                return
            rospy.sleep(.02)

    @classmethod
    def stage(cls, request):
        cls.staged = None if request.clear else copy.deepcopy(request)
        return SetFodCleaningTargetResponse(True, 'staged')

    @classmethod
    def set_mode(cls, request):
        cls.mode_calls.append(request.data)
        cls.mode = 'FOD_ACTIVE' if request.data else 'GPS_ACTIVE'
        cls.visual_state = 'ACQUIRE' if request.data else 'DISABLED'
        cls.visual_started = time.monotonic() if request.data else None
        return SetBoolResponse(True, 'synthetic handoff')

    @classmethod
    def tick(cls, _event):
        stamp = rospy.Time.now()
        x,y,yaw = cls.pose
        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp, 'map', 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = x,y
        transform.transform.rotation.z, transform.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        cls.tf.sendTransform(transform)
        grid = OccupancyGrid()
        grid.header.stamp, grid.header.frame_id = stamp, 'map'
        grid.info.width = grid.info.height = 100
        grid.info.resolution = .2
        grid.info.origin.position.x = grid.info.origin.position.y = -10
        grid.info.origin.orientation.w = 1
        grid.data = [0]*10000
        cls.grid_pub.publish(grid)
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, 'odom', 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x,y
        odom.pose.pose.orientation.w = 1
        if not cls.freeze_stop: cls.odom_pub.publish(odom)
        if cls.visual_started is not None and time.monotonic()-cls.visual_started > .6:
            request = cls.staged
            cls.proof = dict(sequence=cls.proof.get('sequence',0)+1, request_id=request.request_id,
                completion_policy=request.completion_policy, object_id=str(request.object_id),
                acquired_object_id='77', target_x=request.target_position.x, target_y=request.target_position.y,
                forward_m=.51, path_m=.52, blind_target_m=.5, stopped=True)
            cls.mode, cls.visual_state, cls.visual_started = 'GPS_ACTIVE', 'DISABLED', None
        cls.state_pub.publish(String(data=cls.mode))
        cls.visual_pub.publish(String(data=json.dumps(dict(state=cls.visual_state, allow_motion=True,
            external_estop_override=False, pending_target_protocol=2, last_cleaning_completion=cls.proof))))

    def setUp(self):
        cls = type(self)
        cls.wait(lambda: not cls.pending_status.get('active'), 'previous task finished')
        cls.hold_goal = cls.block_plan = cls.freeze_stop = False
        cls.pose = (0.,0.,0.)
        cls.mode, cls.visual_state = 'GPS_ACTIVE', 'DISABLED'
        cls.goals.clear(); cls.cancels.clear(); cls.plans.clear(); cls.mode_calls.clear()
        rospy.sleep(.6)

    def start(self, x, y=0.):
        request = StartPendingLitterCleaningRequest()
        request.object_id = 42
        request.target_position.x, request.target_position.y = x,y
        request.target_frame = 'map'
        request.observed_at = rospy.Time.now()
        request.label = 'a'
        result = rospy.ServiceProxy('/pending_litter_cleaner/start_cleaning', StartPendingLitterCleaning)(request)
        self.assertTrue(result.success, result.message)
        return result

    def terminal(self, state):
        cls = type(self)
        cls.wait(lambda: cls.pending_status.get('state') == state and not cls.pending_status.get('active'), state, 15)

    def test_01_far_goal_accepted_with_ui_observer_and_visual_completion(self):
        self.start(5)
        self.terminal('COMPLETE')
        cls = type(self)
        self.assertEqual(len(cls.goals), 1)
        self.assertAlmostEqual(cls.goals[0].target_pose.pose.position.x, 3.5)
        self.assertEqual(cls.goals[0].target_pose.header.frame_id, 'map')
        self.assertEqual(cls.staged.completion_policy, 'visual_servo')
        self.assertEqual(cls.mode_calls, [True])
        self.assertFalse(cls.nav_status.get('retained_reissue_allowed'))

    def test_02_near_behind_still_issues_target_facing_navigation(self):
        type(self).pose = (0,0,math.pi)
        rospy.sleep(.2)
        self.start(1)
        self.terminal('COMPLETE')
        self.assertEqual(len(type(self).goals), 1)
        self.assertAlmostEqual(type(self).goals[0].target_pose.pose.position.x, -.5)

    def test_03_front_near_can_handoff_without_goal(self):
        self.start(1.5)
        self.terminal('COMPLETE')
        self.assertFalse(type(self).goals)
        self.assertEqual(type(self).mode_calls, [True])

    def test_04_no_path_does_not_start_visual_or_mark_cleaned(self):
        type(self).block_plan = True
        self.start(5)
        self.terminal('ABORT')
        self.assertFalse(type(self).goals)
        self.assertFalse(type(self).mode_calls)

    def test_05_cancel_retires_exact_goal_only(self):
        cls = type(self)
        cls.hold_goal = True
        self.start(5)
        cls.wait(lambda: cls.pending_status.get('goal_id'), 'accepted goal')
        goal_id = cls.pending_status['goal_id']
        result = rospy.ServiceProxy('/pending_litter_cleaner/cancel', Trigger)()
        self.assertTrue(result.success)
        self.terminal('ABORT')
        cls.wait(lambda: goal_id in cls.cancels, 'exact cancel')
        self.assertNotIn('', cls.cancels)
        self.assertFalse(cls.mode_calls)
        cls.wait(lambda: not cls.nav_status.get('active_ordinary_goal_ids'), 'bridge retired')

    def test_06_no_fresh_stop_feedback_prevents_visual(self):
        type(self).freeze_stop = True
        self.start(1.5)
        self.terminal('ABORT')
        self.assertFalse(type(self).mode_calls)

    def test_07_manual_goal_replacement_is_not_canceled_by_pending_cleanup(self):
        cls = type(self)
        cls.hold_goal = True
        self.start(5)
        cls.wait(lambda: cls.pending_status.get('goal_id'), 'pending goal')
        own_id = cls.pending_status['goal_id']
        manual = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        cls.wait(lambda: manual.get_num_connections() >= 2, 'manual publisher')
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = 7.
        pose.pose.orientation.w = 1.
        manual.publish(pose)
        cls.wait(lambda: cls.nav_status.get('retained_goal_id') not in ('', own_id), 'manual accepted')
        manual_id = cls.nav_status['retained_goal_id']
        self.terminal('ABORT')
        self.assertNotIn(manual_id, cls.cancels)
        self.assertFalse(cls.mode_calls)
        self.assertIn(manual_id, cls.nav_status.get('active_ordinary_goal_ids', []))
        cancel = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        cls.wait(lambda: cancel.get_num_connections() >= 1, 'test cleanup publisher')
        cancel.publish(GoalID(id=manual_id))
        cls.wait(lambda: not cls.nav_status.get('active_ordinary_goal_ids'), 'test manual cleanup')
        manual.unregister()
        cancel.unregister()


if __name__ == '__main__':
    rospy.init_node('autolabor_operator_gui')
    rostest.rosrun('autolabor_fod_control', 'pending_navigation_integration', PendingNavigationIntegration)
