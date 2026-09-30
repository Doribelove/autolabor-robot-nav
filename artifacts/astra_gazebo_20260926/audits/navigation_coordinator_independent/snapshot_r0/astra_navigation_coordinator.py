#!/usr/bin/env python3
"""Only navigation-output owner; no Gazebo lifecycle/service calls."""
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from urllib.parse import urlparse


def main():
    import rospy
    import rosgraph
    import actionlib
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from std_srvs.srv import Trigger,TriggerResponse
    from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal
    rospy.init_node('astra_navigation_coordinator')
    master_uri=urlparse(os.environ.get('ROS_MASTER_URI',''))
    if master_uri.scheme!='http' or master_uri.hostname not in ('127.0.0.1','localhost','::1') or master_uri.username or master_uri.password:raise ValueError('loopback simulation master required')
    if rospy.get_param('/use_sim_time',False)is not True or rospy.get_param('/m2_gazebo/simulation_only',False)is not True:raise ValueError('isolated simulation required')
    workspace=Path(os.environ['THESIS_WS']).resolve();sys.path.insert(0,str(workspace/'scripts/astra_gazebo'))
    from navigation_coordinator import NavigationCoordinator,CoordinatorConfig,PERIOD_NS,SCHEMA
    from common_route import load_bundle
    from route_world_adapter import RouteFrame
    directory=Path(rospy.get_param('~bundle'));expected=rospy.get_param('~bundle_sha256')
    if hashlib.sha256((directory/'common_route.json').read_bytes()).hexdigest()!=expected:raise ValueError('common bundle mismatch')
    grid,route,data=load_bundle(directory)
    config=CoordinatorConfig(str(rospy.get_param('~run_id')),rospy.get_param('~seed'),str(rospy.get_param('~method')),
        tuple(data['start_xy_yaw']),tuple(data['goal_xy_yaw']),tuple(data['map_from_odom_xy_yaw']))
    core=NavigationCoordinator(config);map_from_odom=RouteFrame(*data['map_from_odom_xy_yaw']);lock=threading.RLock()
    command_pub=rospy.Publisher('/cmd_vel_navigation',Twist,queue_size=1000)
    identity_pub=rospy.Publisher('/astra/navigation/identity',String,queue_size=1,latch=True)
    status_pub=rospy.Publisher('/astra/navigation/status',String,queue_size=100,latch=True)
    gate_pub=rospy.Publisher('/astra/navigation/gate',String,queue_size=1,latch=True)
    trace_pub=rospy.Publisher('/astra/navigation/command_trace',String,queue_size=1000)
    method_status_pub=None;baseline=config.method in ('actual_teb_common_route','dwa_common_route')
    client=actionlib.SimpleActionClient('/move_base',MoveBaseAction) if baseline else None
    baseline_server_ready=threading.Event();gate_to_dispatch=[]
    if baseline:
        method_status_pub=rospy.Publisher('/astra/method/status',String,queue_size=100,latch=True)
        def await_action_server():
            try:
                if client.wait_for_server():baseline_server_ready.set()
            except Exception as exc:
                with lock:core.fault('baseline_action_server:'+repr(exc))
        threading.Thread(target=await_action_server,daemon=True).start()
    def emit(pub,row):pub.publish(String(data=json.dumps(row,separators=(',',':'),allow_nan=False)))
    last_emitted_phase=[None]
    def publish_output(row):
        message=Twist();message.linear.x=row['v'];message.angular.z=row['w'];command_pub.publish(message);emit(trace_pub,row);last_emitted_phase[0]=row['phase']
    def callback_guard(function):
        def callback(message):
            with lock:
                try:function(message)
                except Exception as exc:core.fault(type(exc).__name__+':'+str(exc))
        return callback
    def on_odometry(message):
        stamp=message.header.stamp.to_nsec()
        if stamp==0 and core.phase=='bootstrap':return
        if message.header.frame_id.lstrip('/')!='odom' or message.child_frame_id.lstrip('/')!='base_link':raise ValueError('actual odom references changed')
        pose=message.pose.pose;q=pose.orientation
        values=(q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in values)-1)>1e-5:raise ValueError('invalid odom quaternion')
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        twist=message.twist.twist
        event=core.observe_odom(stamp,map_from_odom.pose_to_world((pose.position.x,pose.position.y,yaw)),(twist.linear.x,twist.linear.y,twist.angular.z))
        if event is not None:emit(gate_pub,event);gate_to_dispatch.append(event)
    def on_method_command(message):
        excluded=(message.linear.y,message.linear.z,message.angular.x,message.angular.y)
        if not all(math.isfinite(v) for v in excluded) or any(abs(v)>1e-12 for v in excluded):raise ValueError('nonplanar or nonfinite method command')
        core.method_command(message.linear.x,message.angular.z,rospy.Time.now().to_nsec(),time.monotonic_ns())
    def on_method_status(message):core.method_status(json.loads(message.data))
    subscribers=[rospy.Subscriber('/odom',Odometry,callback_guard(on_odometry),queue_size=1000),
                 rospy.Subscriber('/astra/method_cmd',Twist,callback_guard(on_method_command),queue_size=1000),
                 rospy.Subscriber('/astra/method/status',String,callback_guard(on_method_status),queue_size=1000)]
    def publishers_valid():
        publishers=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
        if publishers.get('/cmd_vel_navigation')!=[rospy.get_name()]:raise ValueError('coordinator is not sole navigation command publisher')
        expected_node='/move_base' if baseline else ('/astra_frozen_r2_method' if config.method=='frozen_r2_v060_shared_activation' else '/astra_route_runtime')
        if publishers.get('/astra/method_cmd')!=[expected_node]:raise ValueError('private method publisher identity differs')
    def arm(_request):
        with lock:
            try:publishers_valid();result=core.arm(rospy.Time.now().to_nsec());return TriggerResponse(True,json.dumps(result))
            except Exception as exc:return TriggerResponse(False,str(exc))
    def stop(_request):
        with lock:
            core.request_stop()
            core.command=None;publish_output(dict(core.output(rospy.Time.now().to_nsec()),v=0.,w=0.,reason='external_stop'))
            if client is not None:client.cancel_all_goals()
            return TriggerResponse(True,'zero; coordinator remains alive')
    services=[rospy.Service('/astra/navigation/arm',Trigger,arm),rospy.Service('/astra/navigation/stop',Trigger,stop)]
    emit(identity_pub,{'schema':SCHEMA,'run_id':config.run_id,'seed':config.seed,'method':config.method,'bundle_sha256':expected,
        'sole_output':'/cmd_vel_navigation','method_input':'/astra/method_cmd','gate_time_authority':'first qualifying actual odom header x>=-4 in route frame',
        'method_twist_generation_source_time_known':False,'moving_method_pre_gate_forbidden':True,'gazebo_lifecycle_owned':False})
    next_tick=None;last_zero_source=None;last_status_wall=0.;last_progress_wall=time.monotonic();last_source=None;last_graph_wall=0.;baseline_goal_sent=False
    try:
        while not rospy.is_shutdown():
            wall_now=time.monotonic()
            with lock:
                source_now=rospy.Time.now().to_nsec()
                if last_source is None or source_now>last_source:last_progress_wall=wall_now
                if last_source is not None and source_now<last_source:core.fault('publisher_clock_regression')
                last_source=source_now
                if source_now>0 and wall_now-last_progress_wall>5:core.fault('publisher_clock_wall_stall')
                if baseline and baseline_server_ready.is_set() and core.gate is None:
                    row={'run_id':config.run_id,'seed':config.seed,'method':config.method,'state':'ready','ready':True}
                    core.method_status(row)
                    if wall_now-last_status_wall>=.2:emit(method_status_pub,row)
                if gate_to_dispatch and baseline and not baseline_goal_sent:
                    gate_event=gate_to_dispatch.pop(0);baseline_goal_sent=True
                    goal_message=MoveBaseGoal();goal_message.target_pose.header.frame_id='map';goal_message.target_pose.header.stamp=rospy.Time.now()
                    goal_message.target_pose.pose.position.x,goal_message.target_pose.pose.position.y=config.goal_pose[:2]
                    goal_message.target_pose.pose.orientation.z=math.sin(config.goal_pose[2]/2);goal_message.target_pose.pose.orientation.w=math.cos(config.goal_pose[2]/2)
                    def baseline_active_callback():
                        with lock:
                            row={'run_id':config.run_id,'seed':config.seed,'method':config.method,'state':'active','ready':True,'gate_source_ns':gate_event['source_ns'],
                                 'activation_ack_receipt_clock_ns':rospy.Time.now().to_nsec(),'ack_semantics':'move_base action ACTIVE callback; not command generation timestamp'}
                            core.method_status(row);emit(method_status_pub,row)
                    client.send_goal(goal_message,active_cb=baseline_active_callback)
                if source_now==0:
                    # One clock0 prime after relay+recorder; no paused queue fill.
                    if last_zero_source is None and command_pub.get_num_connections()>=2:publish_output(core.output(0));last_zero_source=0
                elif next_tick is None or source_now>=next_tick:
                    publish_output(core.output(source_now));next_tick=source_now+PERIOD_NS
                if core.phase in ('fault','complete','timeout','stopped') and core.phase!=last_emitted_phase[0]:publish_output(core.output(source_now))
                if wall_now-last_status_wall>=.2:
                    fresh=core.latest_odom is not None and 0<=source_now-core.latest_odom['stamp_ns']<=config.maximum_source_age_ns
                    emit(status_pub,{'schema':SCHEMA,'run_id':config.run_id,'seed':config.seed,'method':config.method,'phase':core.phase,'fault':core.fault_reason,
                        'ready_to_arm':bool(source_now>0 and source_now<5_000_000_000 and core.ready and fresh and not core.armed and core.phase=='bootstrap'),
                        'armed':core.armed,'terminal_source_ns':core.terminal_source_ns,'terminal_reason':core.terminal_reason,'terminal_time_authority':core.terminal_time_authority,'gate_source_ns':None if core.gate is None else core.gate['source_ns'],'publisher_visible_clock_ns':source_now})
                    last_status_wall=wall_now
            if wall_now-last_graph_wall>=1. and core.armed:
                try:publishers_valid()
                except Exception as exc:
                    with lock:core.fault('publisher_roster:'+str(exc))
                last_graph_wall=wall_now
            time.sleep(.002)
    finally:
        # Do not rebind any callback function name in terminal cleanup.
        final_zero=Twist();command_pub.publish(final_zero)
        if client is not None:client.cancel_all_goals()

if __name__=='__main__':main()
