#!/usr/bin/env python3
"""Real cleaner + real bridge + synthetic perception/planner/action server.

Run only with scripts/test_bev_cleaning.sh (network and devices isolated).
"""
import copy
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import socket
import threading
import time
import unittest

import actionlib
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import TransformStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from nav_msgs.msg import OccupancyGrid, Path
from nav_msgs.srv import GetPlan, GetPlanResponse
import rospy
import rostest
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros


class BevCleaningIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert [name for _,name in socket.if_nameindex()] == ['lo'], 'Use isolated test wrapper'
        cls.lock = threading.RLock()
        cls.pose = (0.,0.,0.)
        cls.objects = []
        cls.status, cls.nav = {}, {}
        cls.goals, cls.cancels = [], []
        cls.sequence = 0
        cls.observation_pose, cls.observation_stamp = cls.pose, time.time()
        cls.hold = False
        cls.reject = False
        cls.stale = False
        cls.hide_on_approach = False
        cls.hide_other_on_approach = False
        cls.prove_motion = True
        cls.blocked = set()
        cls.plan_frame = 'map'
        cls.plan_pose_frame = 'map'
        cls.tf = tf2_ros.TransformBroadcaster()
        cls.grid_pub = rospy.Publisher('/move_base/global_costmap/costmap', OccupancyGrid, queue_size=1)
        cls.status_sub = rospy.Subscriber('/bev_cleaner/status', String,
                                          lambda msg: setattr(cls,'status',json.loads(msg.data)))
        cls.nav_sub = rospy.Subscriber('/navigation_pause/status', String,
                                      lambda msg: setattr(cls,'nav',json.loads(msg.data)))
        cls.cancel_sub = rospy.Subscriber('/move_base/cancel', GoalID,
                                         lambda msg: cls.cancels.append(msg.id))
        cls.plan_srv = rospy.Service('/move_base/make_plan', GetPlan, cls.plan)
        rospy.set_param('/move_base/global_costmap/footprint', [[-.2,-.15],[.2,-.15],[.2,.15],[-.2,.15]])
        cls.server = actionlib.SimpleActionServer('/move_base', MoveBaseAction, cls.execute, auto_start=False)
        cls.server.start()
        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_):
                pass
            def do_GET(self):
                with cls.lock:
                    cls.sequence += 1
                    x,y,yaw = cls.observation_pose
                    points = []
                    for tx,ty in cls.objects:
                        dx,dy=tx-x,ty-y
                        points.append(dict(x=math.cos(yaw)*dx+math.sin(yaw)*dy,
                                           y=-math.sin(yaw)*dx+math.cos(yaw)*dy,
                                           label='paper', confidence=.9))
                    data=json.dumps(dict(schema='conbine.bev.cleaning.observations.v1',
                        source_id=dict(session='isolated-test',sequence=cls.sequence),
                        config_sha256='synthetic-only', ground_frame='camera_rig_ground_estimate',
                        configured_cameras=['front','left','right','rear'],camera_count=4,
                        observed_epoch_s=cls.observation_stamp-(10 if cls.stale else 0), ground_objects=points)).encode()
                self.send_response(200)
                self.send_header('Content-Length',str(len(data)))
                self.end_headers()
                self.wfile.write(data)
        cls.http = ThreadingHTTPServer(('127.0.0.1',18984),Handler)
        threading.Thread(target=cls.http.serve_forever,daemon=True).start()
        cls.timer=rospy.Timer(rospy.Duration(.03),cls.tick)
        rospy.wait_for_service('/bev_cleaner/start',timeout=10)
        cls.wait(lambda: cls.nav.get('action_status_received'),'bridge action status')
        rospy.sleep(.5)

    @classmethod
    def tearDownClass(cls):
        cls.http.shutdown()
        cls.http.server_close()
        cls.timer.shutdown()

    @staticmethod
    def wait(predicate,reason,timeout=15):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if predicate(): return
            rospy.sleep(.03)
        raise AssertionError('timeout: '+reason+'; status='+str(BevCleaningIntegration.status))

    @classmethod
    def tick(cls,_event):
        with cls.lock:
            x,y,yaw=cls.pose
            cls.observation_pose, cls.observation_stamp = cls.pose, rospy.Time.now().to_sec()
        t=TransformStamped()
        t.header.stamp,t.header.frame_id,t.child_frame_id=rospy.Time.from_sec(cls.observation_stamp),'map','base_link'
        t.transform.translation.x,t.transform.translation.y=x,y
        t.transform.rotation.z,t.transform.rotation.w=math.sin(yaw/2),math.cos(yaw/2)
        cls.tf.sendTransform(t)
        grid=OccupancyGrid()
        grid.header.stamp,grid.header.frame_id=rospy.Time.now(),'map'
        grid.info.resolution,grid.info.width,grid.info.height=.1,160,160
        grid.info.origin.position.x=grid.info.origin.position.y=-8.
        grid.info.origin.orientation.w=1.
        grid.data=[0]*25600
        for cx,cy in cls.blocked:
            grid.data[cy*160+cx]=100
        cls.grid_pub.publish(grid)

    @classmethod
    def plan(cls,request):
        plan=Path()
        plan.header.frame_id=cls.plan_frame
        a,b=request.start.pose.position,request.goal.pose.position
        n=max(2,math.ceil(math.hypot(b.x-a.x,b.y-a.y)/.05))
        for i in range(n+1):
            p=copy.deepcopy(request.goal)
            p.header.frame_id=cls.plan_pose_frame
            p.pose.position.x=a.x+(b.x-a.x)*i/n
            p.pose.position.y=a.y+(b.y-a.y)*i/n
            plan.poses.append(p)
        return GetPlanResponse(plan)

    @classmethod
    def execute(cls,goal):
        cls.goals.append(copy.deepcopy(goal))
        if cls.hide_on_approach:
            cls.objects=[]
            cls.hide_on_approach=False
        if cls.hide_other_on_approach:
            cls.objects=cls.objects[:1]
            cls.hide_other_on_approach=False
        p,q=goal.target_pose.pose.position,goal.target_pose.pose.orientation
        desired=(p.x,p.y,math.atan2(2*q.w*q.z,1-2*q.z*q.z))
        started=time.monotonic()
        while not rospy.is_shutdown():
            if cls.server.is_preempt_requested():
                cls.server.set_preempted(MoveBaseResult())
                return
            if cls.reject:
                cls.server.set_aborted(MoveBaseResult())
                return
            if not cls.hold:
                if cls.prove_motion:
                    with cls.lock:
                        x,y,yaw=cls.pose
                        dx,dy=desired[0]-x,desired[1]-y
                        distance=math.hypot(dx,dy)
                        step=min(.025,distance)
                        cls.pose=(x+dx*step/max(distance,1e-10),
                                  y+dy*step/max(distance,1e-10),desired[2])
                    if distance < .005:
                        cls.server.set_succeeded(MoveBaseResult())
                        return
                elif time.monotonic()-started>.2:
                    cls.server.set_succeeded(MoveBaseResult())
                    return
            rospy.sleep(.03)

    def setUp(self):
        cls=type(self)
        cls.wait(lambda: not cls.status.get('active'),'previous task idle')
        cls.wait(lambda: not cls.nav.get('active_ordinary_goal_ids'),'previous action released')
        cls.hold=cls.reject=cls.stale=False
        cls.hide_on_approach=False
        cls.hide_other_on_approach=False
        cls.prove_motion=True
        cls.plan_frame=cls.plan_pose_frame='map'
        cls.pose=(0.,0.,0.)
        cls.objects=[]
        cls.goals.clear();cls.cancels.clear();cls.blocked.clear()
        rospy.sleep(2.2)  # Expire the previous run's currently visible points.

    def start(self,objects):
        cls=type(self)
        cls.objects=list(objects)
        cls.wait(lambda: cls.status.get('visible_count',0)>0,'fresh localized BEV points')
        previous=cls.status.get('round_id')
        result=rospy.ServiceProxy('/bev_cleaner/start',Trigger)()
        self.assertTrue(result.success,result.message)
        cls.wait(lambda: cls.status.get('round_id') != previous,'new cleaning round status')

    def terminal(self,state='COMPLETE',timeout=30):
        cls=type(self)
        cls.wait(lambda: not cls.status.get('active') and cls.status.get('state')==state,
                 state,timeout)

    def test_01_current_targets_are_cleaned_in_order_with_real_motion_proof(self):
        cls=type(self)
        self.start([(2.,0.),(3.8,0.)])
        self.terminal()
        self.assertEqual(2,cls.status['passed_count'])
        self.assertEqual(4,len(cls.goals))
        self.assertLess(cls.goals[0].target_pose.pose.position.x,cls.goals[2].target_pose.pose.position.x)
        self.assertFalse(cls.status['pickup_verified'])
        self.assertEqual(4,cls.status['camera_count'])
        self.assertIn('rear',cls.status['camera_names'])

    def test_02_navigation_success_without_motion_is_not_cleaning(self):
        cls=type(self)
        cls.prove_motion=False
        self.start([(2.,0.)])
        self.terminal('PARTIAL')
        self.assertEqual(0,cls.status['passed_count'])
        self.assertEqual(1,cls.status['skipped_count'])
        self.assertEqual(1,cls.status['pending_count'])
        self.assertIn('实际刷子轨迹未通过', next(iter(cls.status['skipped_reasons'].values())))

    def test_03_stop_cancels_only_the_exact_current_goal(self):
        cls=type(self)
        cls.hold=True
        self.start([(2.,0.)])
        cls.wait(lambda: bool(cls.status.get('goal_id')),'owned goal')
        owned=cls.status['goal_id']
        result=rospy.ServiceProxy('/bev_cleaner/cancel',Trigger)()
        self.assertTrue(result.success)
        self.terminal('CANCELED')
        self.assertIn(owned,cls.cancels)
        self.assertNotIn('',cls.cancels)

    def test_04_navigation_failure_does_not_claim_any_cleaning(self):
        cls=type(self)
        cls.reject=True
        self.start([(2.,0.)])
        self.terminal('FAILED')
        self.assertEqual(0,cls.status['passed_count'])

    def test_05_missing_installation_extrinsic_keeps_preview_without_a_navigation_goal(self):
        cls=type(self)
        cls.objects=[(2.,0.)]
        rospy.wait_for_service('/bev_unconfigured/start',timeout=5)
        rospy.sleep(.4)
        result=rospy.ServiceProxy('/bev_unconfigured/start',Trigger)()
        self.assertFalse(result.success)
        self.assertIn('外参',result.message)
        self.assertEqual([],cls.goals)

    def test_06_stale_bev_cannot_start_from_an_old_point(self):
        cls=type(self)
        cls.stale=True
        cls.objects=[(2.,0.)]
        rospy.sleep(.5)
        result=rospy.ServiceProxy('/bev_cleaner/start',Trigger)()
        self.assertFalse(result.success)
        self.assertEqual([],cls.goals)

    def test_07_sweep_blocked_by_costmap_is_skipped_without_a_goal(self):
        cls=type(self)
        cls.blocked={(x,y) for x in range(96,105) for y in range(76,85)}
        self.start([(2.,0.)])
        self.terminal('PARTIAL')
        self.assertEqual(0,cls.status['passed_count'])
        self.assertEqual(1,cls.status['skipped_count'])
        self.assertEqual([],cls.goals)

    def test_08_lost_detection_keeps_recorded_location_for_actual_sweep(self):
        cls=type(self)
        cls.hide_on_approach=True
        self.start([(2.,0.)])
        self.terminal()
        self.assertEqual(1,cls.status['passed_count'])
        self.assertEqual(0,cls.status['pending_count'])
        self.assertEqual(2,len(cls.goals))

    def test_09_new_litter_in_view_is_planned_after_the_current_pass(self):
        cls=type(self)
        self.start([(2.,0.)])
        cls.wait(lambda: cls.status.get('state')=='SWEEPING','first cleaning pass')
        cls.objects.append((3.8,0.))
        self.terminal()
        self.assertEqual(2,cls.status['passed_count'])
        self.assertEqual(4,len(cls.goals))

    def test_10_move_base_empty_path_header_with_map_poses_is_usable(self):
        cls=type(self)
        cls.plan_frame=''
        self.start([(2.,0.)])
        self.terminal()
        self.assertEqual(1,cls.status['passed_count'])
        self.assertEqual(2,len(cls.goals))

    def test_11_empty_path_header_does_not_assume_pose_frame(self):
        cls=type(self)
        cls.plan_frame=''
        for frame in ('', 'odom'):
            cls.plan_pose_frame=frame
            self.start([(2.,0.)])
            self.terminal('PARTIAL')
            self.assertEqual(0,cls.status['passed_count'])
            self.assertEqual([],cls.goals)

    def test_12_second_target_lost_during_first_approach_is_still_cleaned(self):
        cls=type(self)
        cls.hold=True
        cls.hide_other_on_approach=True
        self.start([(2.,0.),(3.8,0.)])
        cls.wait(lambda: len(cls.goals)==1, 'first approach started')
        # Other target disappears for longer than the original 2-second TTL.
        cls.wait(lambda: cls.status.get('remembered_count')==1, 'second target remembered', timeout=5)
        self.assertEqual(2,cls.status['pending_count'])
        self.assertEqual(1,cls.status['visible_count'])
        cls.hold=False
        self.terminal()
        self.assertEqual(2,cls.status['passed_count'])
        self.assertEqual(0,cls.status['pending_count'])
        self.assertEqual(0,cls.status['skipped_count'])
        self.assertEqual(4,len(cls.goals))
        self.assertGreater(cls.goals[2].target_pose.pose.position.x, 2.)


if __name__=='__main__':
    rospy.init_node('test_bev_cleaning')
    rostest.rosrun('autolabor_fod_control','bev_cleaning',BevCleaningIntegration)
