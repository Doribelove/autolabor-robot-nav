#!/usr/bin/env python3
"""Actual GPS manager + limiter + mode + visual nodes on an isolated master.

Only chassis, sensors, TCP sweeper and move_base's physical planner are fake.
Actionlib cancellation/history are real, including late old-goal status.
"""
import json
import math
from pathlib import Path
import sys
import time
import unittest

import actionlib
import rospy
import rostest
import tf2_ros
from actionlib_msgs.msg import GoalID
from autolabor_canbus_driver.msg import CanBusMessage, ChassisStatusInfo
from autolabor_fod_msgs.msg import FodDetection, FodDetectionArray
from geometry_msgs.msg import Point32, PoseStamped, TransformStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseResult
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, NavSatFix, RegionOfInterest
from std_msgs.msg import Bool, Float64, String
from std_srvs.srv import SetBool
from autolabor_fod_control.gps_recovery import local_to_gps

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_visual_servo_ros import FakeWorldIntegrationTest, MODEL_SHA, WHEELBASE
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / 'SweepDeviceControl'))
from test_sweep_test import DeviceServer


class GpsLossRecoveryTest(FakeWorldIntegrationTest):
    def setUp(self):
        self.ready = False
        self.case = rospy.get_param('~case')
        self.mode, self.visual, self.route = {}, {}, {}
        self.mode_states, self.route_states, self.goals, self.fallbacks = [], [], [], []
        self.trace = []
        self.cancel_ids = []
        self.loss_x = None
        self.approach_x = None
        self.current_goal = None
        self.nearest_x = 4.0
        self.nearest_y = float(rospy.get_param("~fod_offset_y", 1.0)) if self.case == "fuzzy_offset" else 0.0
        self.approach_time = None
        self.min_fod_distance = float("inf")
        self.sweep_first_x = None
        self.loss_injected = False
        self.late_cancel_sent = False
        self.started = time.monotonic()
        self.sweeper = DeviceServer(port=int(rospy.get_param('~sweep_port')))
        super().setUp()
        self.x = -5.2
        self.tf_pub = tf2_ros.TransformBroadcaster()
        self.gps_pose_pub = rospy.Publisher('/gps/odom', Odometry, queue_size=10)
        self.navigation_pub = rospy.Publisher('/cmd_vel_navigation', Twist, queue_size=1)
        self.goal_fix_pub = rospy.Publisher('/gps/goal_fix', NavSatFix, queue_size=1)
        self.current_goal_pub = rospy.Publisher('/move_base/current_goal', PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.extra_subs = [
            rospy.Subscriber('/fod_navigation_mode/status', String, self._mode_cb, queue_size=30),
            rospy.Subscriber('/gps/long_range/status', String, self._route_cb, queue_size=30),
            rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, lambda m: self.goals.append(m), queue_size=30),
            rospy.Subscriber('/move_base/cancel', GoalID, lambda m: self.cancel_ids.append(m.id), queue_size=30),
            rospy.Subscriber('/gps/goal_fix', NavSatFix, self._goal_fix_cb, queue_size=10)]
        self.action = actionlib.SimpleActionServer('/move_base', MoveBaseAction, auto_start=False)
        self.action.register_goal_callback(self._accept_goal)
        self.action.register_preempt_callback(self._preempt)
        self.action.start()
        self.ready = True

    def tearDown(self):
        try:
            rospy.ServiceProxy('/fod_navigation_mode/set_fod_enabled', SetBool)(False)
        finally:
            self.ready = False
            super().tearDown()
            for sub in self.extra_subs:
                sub.unregister()
            self.sweeper.__exit__()

    def _accept_goal(self):
        self.current_goal = self.action.accept_new_goal().target_pose
        self.current_goal_pub.publish(self.current_goal)

    def _preempt(self):
        if self.action.is_active():
            self.action.set_preempted(MoveBaseResult(), 'actual actionlib preemption')

    def _mode_cb(self, msg):
        self.mode = json.loads(msg.data)
        state = self.mode['state']
        if not self.mode_states or state != self.mode_states[-1]:
            self.mode_states.append(state)

    def _route_cb(self, msg):
        self.route = json.loads(msg.data)
        self.route_states.append(self.route.get('state'))

    def _goal_fix_cb(self, msg):
        if msg.header.frame_id == 'fod_gps_recovery':
            self.fallbacks.append(msg)

    def _status_cb(self, msg):
        super()._status_cb(msg)
        self.visual = json.loads(msg.data)
        if self.visual.get('state') == 'APPROACH' and self.approach_x is None:
            self.approach_x = self.x
            self.approach_time = time.monotonic()

    def _publish_world(self, _event):
        if not self.ready:
            return
        now = time.monotonic()
        dt = min(.1, max(.001, now-self.last_tick))
        self.last_tick = now
        with self.lock:
            delayed = [m for t,m in self.command_history if t <= now-.10]
            cmd = delayed[-1] if delayed else Twist()
            v = cmd.linear.x if abs(cmd.linear.x) >= .19 else 0.
            self.actual_linear_x += max(-2*dt, min(2*dt, v-self.actual_linear_x))
            v = self.actual_linear_x
            omega = cmd.angular.z * v/cmd.linear.x if abs(cmd.linear.x) > 1e-6 else 0.
            self.yaw += omega*dt
            self.x += v*math.cos(self.yaw)*dt
            self.y += v*math.sin(self.yaw)*dt
            self.travel_m += max(0., v)*dt
            x,y,yaw = self.x,self.y,self.yaw
        stamp = rospy.Time.now()
        odom = Odometry()
        odom.header.stamp, odom.header.frame_id = stamp, 'camera_init'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x,y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = v,omega
        self.gps_pose_pub.publish(odom)
        self._publish_odometry(stamp,x,y,yaw,v,omega)
        # Known camera at 0.45 m height, level optical axes; dynamic timestamped TF.
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id = stamp, 'camera_init'
        tf.child_frame_id = 'zed2_left_camera_optical_frame'
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = x,y,.45
        # q(yaw) * optical_to_body(-.5,.5,-.5,.5)
        s,c = math.sin(yaw/2),math.cos(yaw/2)
        q=tf.transform.rotation
        q.x,q.y,q.z,q.w = -.5*c-.5*s, .5*c-.5*s, -.5*c+.5*s, .5*c+.5*s
        self.tf_pub.sendTransform(tf)
        base = TransformStamped()
        base.header.stamp, base.header.frame_id = stamp, 'camera_init'
        base.child_frame_id = 'base_link'
        base.transform.translation.x, base.transform.translation.y = x,y
        base.transform.rotation.z, base.transform.rotation.w = math.sin(yaw/2), math.cos(yaw/2)
        self.tf_pub.sendTransform(base)
        camera=CameraInfo();camera.header.stamp=stamp;camera.header.frame_id=tf.child_frame_id
        camera.width,camera.height=1280,720
        camera.K=[523.,0.,635.,0.,523.,366.,0.,0.,1.]
        camera.P=[523.,0.,635.,0.,0.,523.,366.,0.,0.,0.,1.,0.]
        self.camera_pub.publish(camera)
        self._detections(stamp,x,y,yaw)
        self.wheel_pub.publish(Float64(data=math.atan(omega*WHEELBASE/v) if abs(v)>1e-6 else 0.))
        self.chassis_pub.publish(ChassisStatusInfo())
        self.timeout_pub.publish(Bool(data=False))
        navigation=Twist()
        if self.action.is_active() and self.current_goal:
            p=self.current_goal.pose.position
            distance=math.hypot(p.x-x,p.y-y)
            if distance <= .28:
                self.action.set_succeeded(MoveBaseResult(), 'simulated planner arrival within 0.3 m tolerance')
            else:
                error=math.atan2(p.y-y,p.x-x)-yaw
                error=math.atan2(math.sin(error),math.cos(error))
                navigation.linear.x=1.6
                navigation.angular.z=max(-.35,min(.35,error))*1.6
        self.navigation_pub.publish(navigation)
        if self.sweeper.state and self.sweep_first_x is None:
            self.sweep_first_x=x
        if self.mode.get('state') == 'GPS_FOD_RECOVERY' and not self.late_cancel_sent:
            old=[i for i in self.cancel_ids if i]
            if old:
                self.cancel_pub.publish(GoalID(id=old[0]))
                self.late_cancel_sent=True
        self.min_fod_distance=min(self.min_fod_distance,math.hypot(x-self.nearest_x,y-self.nearest_y))
        self.trace.append([now-self.started,x,y,v,self.mode.get('state'),self.visual.get('state')])

    def _detections(self, stamp,x,y,yaw):
        msg=FodDetectionArray();msg.header.stamp=stamp;msg.header.frame_id='zed2_left_camera_optical_frame'
        msg.image_width,msg.image_height=1280,720
        msg.model_name,msg.model_sha256,msg.model_task='simulated-approved-model',MODEL_SHA,'detect'
        visible=True
        if self.case == 'lost_during_handoff':
            visible='ENTERING_FOD' not in self.mode_states
        elif self.approach_x is not None and self.case != 'approach_limit':
            travel=x-self.approach_x
            if self.case == 'fuzzy_offset':
                elapsed=time.monotonic()-self.approach_time
                visible=not (1.0<elapsed<1.4) and x<self.nearest_x-1.5
            elif self.case == 'relaxed_tracking':
                visible=travel < 1.0 and not (.18 < travel < .26)
            else:
                visible=travel < .25
        if self.case == 'no_fod':
            visible=False
        if (not visible and self.mode.get('fod_gps_point') and not self.loss_injected
                and (self.case != 'relaxed_tracking' or
                     (self.approach_x is not None and x-self.approach_x >= 1.0))):
            self.loss_injected=True;self.loss_x=x
        if self.case == 'returning_boxes' and self.loss_injected:
            visible=int(stamp.to_sec()*10)%3==0
        objects=[(1,self.nearest_x,self.nearest_y)] if visible else []
        if self.case in ('fuzzy_offset','three_targets','returning_boxes','invalid_depth','self_reflection','relaxed_tracking'):
            objects += [(2,5.3,1.2),(3,5.8,-1.1)]
        if self.case == 'self_reflection':
            # Fixed on the moving chassis, closer and more confident than FOD.
            objects += [(99,x+.5*math.cos(yaw),y+.5*math.sin(yaw))]
        for identity,wx,wy in objects:
            dx,dy=wx-x,wy-y
            z=math.cos(yaw)*dx+math.sin(yaw)*dy
            right=math.sin(yaw)*dx-math.cos(yaw)*dy
            if z <= .4:
                continue
            u,v=635+523*right/z,366+523*(.04 if identity==99 else .45)/z
            if not (25<u<1255 and 20<v<700):
                continue
            d=FodDetection();d.class_id,d.class_name,d.confidence=0,'Metal',.9
            if self.case == 'relaxed_tracking' and identity==1:
                d.confidence=.13
                if self.approach_x is not None and x-self.approach_x > .26:
                    identity=77
                    d.class_id,d.class_name=2,'Plastic'
            d.track_id,d.depth_valid=identity,True
            d.depth_m,d.range_m=z,math.sqrt(dx*dx+dy*dy+.45**2)
            d.bbox=RegionOfInterest(x_offset=int(round(u))-20,y_offset=int(round(v))-10,width=40,height=20)
            d.anchor_px=Point32(x=u,y=v+10)
            if self.case == 'invalid_depth' and identity==1 and self.approach_x is not None:
                d.depth_valid=False;d.depth_m=0;d.range_m=0
            msg.detections.append(d)
        self.detection_pub.publish(msg)

    def test_loss_finishes_one_gps_crossing(self):
        rospy.wait_for_service('/fod_navigation_mode/set_fod_enabled', timeout=10)
        rospy.sleep(1.5)
        for kind in (0x17,0x18,0x19,0x80):
            self.canbus_pub.publish(CanBusMessage(node_type=0x10,node_seq=0,msg_type=kind,payload=[0x10] if kind==0x80 else [0]))
        original=NavSatFix();original.header.stamp=rospy.Time.now()
        original.latitude,original.longitude=local_to_gps(5,0,30.3007,104.3898)
        self.goal_fix_pub.publish(original)
        rospy.sleep(.3)
        response=rospy.ServiceProxy('/fod_navigation_mode/set_fod_enabled',SetBool)(True)
        self.assertTrue(response.success,response.message)
        deadline=time.monotonic()+55
        while time.monotonic()<deadline:
            state=self.mode.get('state')
            if state in ('FOD_COMPLETE_STOP','FOD_ABORTED','FAULT_STOP'):
                break
            if self.case=='no_fod' and self.route.get('state')=='COMPLETE' and self.sweeper.state==0:
                break
            rospy.sleep(.05)
        rospy.sleep(3.7)
        result=dict(case=self.case,mode=self.mode,route=self.route,modes=self.mode_states,
                    visual_states=self.states,selected_ids=sorted(self.selected_target_ids),
                    fallback_count=len(self.fallbacks),stop_x=self.x,loss_x=self.loss_x,
                    sweep_first_x=self.sweep_first_x,sweep_state=self.sweeper.state,
                    min_fod_distance=self.min_fod_distance,stop_y=self.y,
                    hydraulic=[list(h[:2]) for h in self.sweeper.hydraulic],
                    goal_ids=[m.goal_id.id for m in self.goals],late_cancel_sent=self.late_cancel_sent)
        path=Path(rospy.get_param('~results_file'));path.write_text(json.dumps(result,ensure_ascii=False,indent=2))
        path.with_suffix('.trace.json').write_text(json.dumps(self.trace))
        self.assertIsNotNone(self.sweep_first_x,result)
        self.assertGreater(self.sweep_first_x,-5.01,result)
        self.assertEqual(self.sweeper.state,0,result)
        self.assertEqual([h[:2] for h in self.sweeper.hydraulic],[('extend',3),('retract',3)],result)
        self.assertEqual(len(set(result['goal_ids'])),len(result['goal_ids']),result)
        self.assertNotIn('ABORTED',self.route_states,result)
        if self.case=='no_fod':
            self.assertEqual(len(self.fallbacks),0,result)
            self.assertNotIn('FOD_ACTIVE',self.mode_states,result)
            return
        self.assertEqual(self.mode.get('state'),'FOD_COMPLETE_STOP',result)
        self.assertEqual(len(self.fallbacks),1,result)
        point=self.mode['fod_gps_point']
        self.assertEqual(point['track_id'],77 if self.case=='relaxed_tracking' else 1,result)
        self.assertAlmostEqual(point['fod_x'],4.,delta=.025)
        if self.case == 'fuzzy_offset':
            self.assertLess(self.min_fod_distance,.22,result)
            self.assertNotIn('TRACK_WAIT',self.states[:self.states.index('GPS_HANDOFF')] if 'GPS_HANDOFF' in self.states else self.states,result)
        else:
            self.assertAlmostEqual(point['goal_x'],4.8,delta=.025)
        self.assertGreater(self.x,4.5,result)  # Existing GPS arrival tolerance is 0.3 m.
        self.assertLess(self.x,5.05,result)
        self.assertAlmostEqual(self.command.linear.x,0.)
        self.assertNotIn('FOD_ABORTED',self.mode_states,result)
        self.assertNotIn('ABORT',self.states,result)
        if self.case!='lost_during_handoff':
            self.assertIn('APPROACH',self.states,result)
            self.assertEqual(self.selected_target_ids,{1,77} if self.case=='relaxed_tracking' else {1},result)
        if self.case=='relaxed_tracking':
            self.assertNotIn('TRACK_WAIT',self.states,result)
            self.assertGreater(self.loss_x-self.approach_x,.95,result)
        self.assertTrue(self.late_cancel_sent,result)


for name in unittest.defaultTestLoader.getTestCaseNames(FakeWorldIntegrationTest):
    setattr(GpsLossRecoveryTest,name,None)

if __name__=='__main__':
    rospy.init_node('fake_world')
    rostest.rosrun('autolabor_fod_control','gps_loss_recovery',GpsLossRecoveryTest)
