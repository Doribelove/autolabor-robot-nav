#!/usr/bin/env python3
"""N06 analytic shared-gate route node with a 1.2 m lookahead factor.

Map and route are verified against the provider's actual latched messages.
Only LaserScan, timestamp TF, Odometry and actual JointState feed the policy.
No Gazebo model/link/actor/oracle subscription and no runtime parameter writes.
"""
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import threading
import time
from collections import deque
from urllib.parse import urlparse
import xml.etree.ElementTree as ET


def main():
    import rospy
    import tf2_ros
    import numpy as np
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import LaserScan,JointState
    from nav_msgs.msg import Odometry,OccupancyGrid,Path as RosPath
    from std_msgs.msg import String
    from std_srvs.srv import Trigger,TriggerResponse
    rospy.init_node('astra_route_runtime')
    uri=urlparse(os.environ.get('ROS_MASTER_URI',''))
    if uri.scheme!='http' or uri.hostname not in ('127.0.0.1','localhost','::1') or uri.username or uri.password:raise RuntimeError('loopback simulation master required')
    if rospy.get_param('/use_sim_time',False)is not True or rospy.get_param('/m2_gazebo/simulation_only',False)is not True:raise RuntimeError('isolated simulation markers required')
    workspace=Path(os.environ['THESIS_WS']).resolve();sys.path.insert(0,str(workspace/'scripts/astra_gazebo'))
    from common_route import load_bundle
    from route_world_adapter import RouteFrame
    from route_sensor_buffer import RouteSensorBuffer
    from route_candidate_core import RouteAwareRuntime,RouteConfig,OBSERVATION_SCHEMA,ACTION_NAMES,prepublish_guard,physical_backend
    run_id=str(rospy.get_param('~run_id'));seed=rospy.get_param('~seed');method=rospy.get_param('~method')
    if not run_id or type(seed)is not int or seed<=0 or method!='astra_analytic_lookahead_1p2_n06':raise ValueError('N06 lookahead identity and positive seed required')
    bundle=Path(rospy.get_param('~bundle')).resolve();bundle_sha=str(rospy.get_param('~bundle_sha256'))
    if hashlib.sha256((bundle/'common_route.json').read_bytes()).hexdigest()!=bundle_sha:raise ValueError('bundle SHA mismatch')
    grid,route,route_data=load_bundle(bundle);map_from_odom=RouteFrame(*route_data['map_from_odom_xy_yaw'])
    urdf=rospy.get_param('/robot_description');backend=physical_backend(urdf)
    if backend['seed']!=seed:raise ValueError('actual physical backend seed differs from run identity')
    config=RouteConfig(lookahead_m=1.2,maximum_episode_duration_s=float(rospy.get_param('~maximum_episode_duration_s',22.)));core=RouteAwareRuntime(grid,route,urdf,config)
    actor_artifact_sha=None
    if method=='astra_pct_route_v1':
        metadata_path=Path(rospy.get_param('~checkpoint_metadata')).resolve()
        metadata_bytes=metadata_path.read_bytes()
        if hashlib.sha256(metadata_bytes).hexdigest()!=rospy.get_param('~checkpoint_metadata_sha256'):raise ValueError('checkpoint metadata SHA mismatch')
        metadata=json.loads(metadata_bytes);checkpoint=Path(metadata['checkpoint_path']).resolve();checkpoint_bytes=checkpoint.read_bytes()
        actor_artifact_sha=hashlib.sha256(checkpoint_bytes).hexdigest()
        if actor_artifact_sha!=metadata['checkpoint_sha256']:raise ValueError('new actor artifact SHA mismatch')
        sys.path.insert(0,str(workspace/'src/application/v5_e2e_scope_rl/src'))
        from route_actor_adapter import load_route_actor
        choose=load_route_actor(checkpoint_bytes,core,metadata)
        core=RouteAwareRuntime(grid,route,urdf,config,choose,metadata)
    buffer=RouteSensorBuffer();lock=threading.RLock();pending=deque(maxlen=16);seen_scan_ns=None
    state={'external_gate':None,'activation_wait_reason':None,'phase':'bootstrap','fault':None,'map_ready':False,'route_ready':False,'start_ns':None,'started_once':False,'selected_action':0,'next_decision_ns':None,'next_control_ns':None,'bootstrap_zero_stamp_discarded':{'scan':0,'odom':0,'joint':0}}
    tf_buffer=tf2_ros.Buffer(cache_time=rospy.Duration(10));listener=tf2_ros.TransformListener(tf_buffer)
    command_pub=rospy.Publisher('/cmd_vel_navigation',Twist,queue_size=10)
    identity_pub=rospy.Publisher('/astra/route_runtime/identity',String,queue_size=1,latch=True)
    status_pub=rospy.Publisher('/astra/route_runtime/status',String,queue_size=1,latch=True)
    trace_pub=rospy.Publisher('/astra/route_runtime/trace',String,queue_size=1000)
    method_status_pub=rospy.Publisher('/astra/method/status',String,queue_size=100,latch=True)
    candidate_pub=rospy.Publisher('/astra/route_runtime/candidates',String,queue_size=100)
    def emit(pub,data):pub.publish(String(data=json.dumps(data,separators=(',',':'),allow_nan=False)))
    def publish(v=0.,w=0.):
        message=Twist();message.linear.x=v;message.angular.z=w;command_pub.publish(message)
    def quaternion(q):
        vals=(q.x,q.y,q.z,q.w)
        if not all(math.isfinite(v) for v in vals) or abs(sum(v*v for v in vals)-1)>1e-5:raise ValueError('non-normalized sensor quaternion')
        return (math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y)),math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x)))),math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    def fault(reason):state['fault']=str(reason);state['phase']='fault';publish()
    def protected(function):
        def callback(message):
            with lock:
                try:function(message)
                except Exception as exc:fault(type(exc).__name__+':'+str(exc))
        return callback
    def bootstrap_zero(message,kind):
        if message.header.stamp.to_nsec()==0 and state['phase']=='bootstrap' and not state['started_once']:
            state['bootstrap_zero_stamp_discarded'][kind]+=1;return True
        return False
    def scan(message):
        nonlocal seen_scan_ns
        if bootstrap_zero(message,'scan'):return
        stamp=message.header.stamp.to_nsec()
        if message.header.frame_id.lstrip('/')!='laser_link' or stamp<=0 or (seen_scan_ns is not None and stamp<=seen_scan_ns):raise ValueError('invalid scan source identity/order')
        if len(pending)==pending.maxlen:raise ValueError('scan TF queue overflow')
        pending.append(message);seen_scan_ns=stamp
    def odom(message):
        if bootstrap_zero(message,'odom'):return
        if message.header.frame_id.lstrip('/')!='odom' or message.child_frame_id.lstrip('/')!='base_link':raise ValueError('actual odometry reference mismatch')
        p=message.pose.pose;roll,pitch,yaw=quaternion(p.orientation);twist=message.twist.twist
        buffer.odom(message.header.stamp.to_nsec(),map_from_odom.pose_to_world((p.position.x,p.position.y,yaw)),(twist.linear.x,twist.linear.y,twist.angular.z),(roll,pitch))
    def joint(message):
        if bootstrap_zero(message,'joint'):return
        buffer.joint(message.header.stamp.to_nsec(),message.name,message.position)
    def map_message(message):
        info=message.info
        if message.header.frame_id!='map' or info.width!=grid.width or info.height!=grid.height or abs(info.resolution-grid.resolution)>1e-7 or math.dist((info.origin.position.x,info.origin.position.y),grid.origin[:2])>1e-9 or max(abs(v) for v in quaternion(info.origin.orientation))>1e-9 or abs(info.origin.position.z)>1e-9 or not np.array_equal(np.asarray(message.data,dtype=np.int8).reshape(grid.height,grid.width),grid.blocking_occupancy):raise ValueError('actual shared map bytes/geometry differ from bundle')
        state['map_ready']=True
    def path_message(message):
        points=[[pose.pose.position.x,pose.pose.position.y] for pose in message.poses]
        if message.header.frame_id!='map' or any(p.header.frame_id!='map' for p in message.poses) or points!=route_data['points_xy']:raise ValueError('actual shared global route differs from bundle')
        for index,pose in enumerate(message.poses):
            roll,pitch,yaw=quaternion(pose.pose.orientation);expected=route.goal_yaw if index==len(message.poses)-1 else route.sample(route.arcs[index])[2]
            if abs(roll)+abs(pitch)>1e-9 or abs(math.atan2(math.sin(yaw-expected),math.cos(yaw-expected)))>1e-9 or abs(pose.pose.position.z)>1e-9:raise ValueError('shared global route orientation/reference differs')
        state['route_ready']=True
    def drain(now):
        while pending:
            message=pending[0];stamp=message.header.stamp.to_nsec()
            if stamp>now:break
            if now-stamp>buffer.MAX_AGE_NS:raise ValueError('scan expired while waiting for its timestamp TF')
            try:transform=tf_buffer.lookup_transform('map','laser_link',message.header.stamp,rospy.Duration(0))
            except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):break
            if transform.header.stamp.to_nsec()!=stamp:raise ValueError('TF returned latest/different scan epoch')
            t=transform.transform;p=t.translation;q=t.rotation;angles=quaternion(q)
            buffer.scan(stamp,stamp,message.ranges,message.angle_min,message.angle_increment,message.range_min,message.range_max,(p.x,p.y,angles[2]),now,(q.x,q.y,q.z,q.w));pending.popleft()
    subscribers=[rospy.Subscriber(topic,kind,protected(callback),queue_size=size) for topic,kind,callback,size in (
        ('/scan',LaserScan,scan,16),('/odom',Odometry,odom,100),('/joint_states',JointState,joint,100),('/map',OccupancyGrid,map_message,1),('/astra/common_route/path',RosPath,path_message,1))]
    def health(now):
        failures=buffer.health(now)
        if not state['map_ready']:failures.append('shared_map_not_verified')
        if not state['route_ready']:failures.append('shared_route_not_verified')
        if state['fault']:failures.append(state['fault'])
        return failures
    def start(_request):
        with lock:
            if state['external_gate'] is None:return TriggerResponse(success=False,message='shared gate absent')
            if state['started_once']:return TriggerResponse(success=False,message='start already consumed')
            now=rospy.Time.now().to_nsec()
            try:
                drain(now);failures=health(now)
                if failures:return TriggerResponse(success=False,message=json.dumps(failures))
                import rosgraph
                publishers=dict(rosgraph.Master(rospy.get_name()).getSystemState()[0])
                if publishers.get(command_pub.resolved_name)!=[rospy.get_name()]:raise ValueError('navigation publisher is not uniquely this node')
                state.update(started_once=True,start_ns=state['external_gate']['source_ns'],phase='running',next_control_ns=now,next_decision_ns=now)
                return TriggerResponse(success=True,message=json.dumps({'run_id':run_id,'seed':seed,'start_ns':state['external_gate']['source_ns'],'activation_publisher_clock_ns':now,'semantics_sha256':core.semantics_sha256}))
            except Exception as exc:return TriggerResponse(success=False,message=str(exc))
    def stop(_request):
        with lock:state['phase']='stopped';publish();return TriggerResponse(success=True,message='zero command; node remains alive')
    services=[rospy.Service('/astra/route_runtime/start',Trigger,start),rospy.Service('/astra/route_runtime/stop',Trigger,stop)]
    def on_shared_gate(message):
        with lock:
            try:
                gate_payload=json.loads(message.data)
                if gate_payload.get('schema')!='astra_common_navigation_v1' or (gate_payload.get('run_id'),gate_payload.get('seed'),gate_payload.get('method'))!=(run_id,seed,method):raise ValueError('shared gate identity mismatch')
                if type(gate_payload.get('source_ns'))is not int or gate_payload['source_ns']<=0:raise ValueError('invalid shared gate source stamp')
                if state['external_gate'] is not None and state['external_gate']!=gate_payload:raise ValueError('shared gate changed')
                state['external_gate']=gate_payload
            except Exception as exc:fault('shared_gate:'+str(exc))
    subscribers.append(rospy.Subscriber('/astra/navigation/gate',String,on_shared_gate,queue_size=1))
    def emit_method_status(current_ns):
        try:ready=current_ns>0 and not health(current_ns)
        except Exception:ready=False
        phase='active' if state['phase']=='running' else state['phase']
        emit(method_status_pub,{'run_id':run_id,'seed':seed,'method':method,'state':phase,'ready':ready,'reason':state['fault'],
            'gate_source_ns':None if state['external_gate'] is None else state['external_gate']['source_ns'],
            'activation_ack_semantics':'local runtime began after shared gate; headerless commands retain receipt-only timing'})
    emit(identity_pub,{'run_id':run_id,'seed':seed,'method':method,'observation_schema':OBSERVATION_SCHEMA,'candidate_semantics_sha256':core.semantics_sha256,'core_identity':core.identity,
        'execution_protocol':'astra_common_navigation_v1','standalone_parent_sha256':'23f35c1d9092954ab1d8f75713e8a2c74adc1272380c85662ece6b65e9e608c6','bundle_sha256':bundle_sha,'path_points_sha256':route_data['path_points_sha256'],'actual_urdf_sha256':core.footprint.urdf_sha256,'checkpoint_sha256':actor_artifact_sha,
        'actor_inference_protocol':'SensorActor architecture only; deterministic choose_actions advantage_gate_margin0.10; proposed denotes its post-gate policy output',
        'actor_adapter_sha256':hashlib.sha256((workspace/'scripts/astra_gazebo/route_actor_adapter.py').read_bytes()).hexdigest(),
        'physical_safety_certified':False,'oracle_input':False,'frozen_R2':False,'history_policy':'six real at-or-before200ms integer buckets; latest scan separately used by shield'})
    last_wall=time.monotonic();last_progress=last_wall;last_ros=0;last_status=0.
    try:
        while not rospy.is_shutdown():
            wall=time.monotonic()
            with lock:
                now=rospy.Time.now().to_nsec()
                try:
                    if now>0:drain(now)
                    if state['phase']=='bootstrap' and state['external_gate'] is not None and now>=state['external_gate']['source_ns']:
                        activation_answer=start(None);state['activation_wait_reason']=None if activation_answer.success else activation_answer.message
                        if activation_answer.success:emit_method_status(now)
                    if now>last_ros:last_progress=wall
                    if now<last_ros:fault('source_clock_regression')
                    if state['phase']=='running' and wall-last_progress>10:fault('source_clock_stall')
                    last_ros=now
                    if state['phase']!='running':
                        if wall-last_wall>=.05:publish();last_wall=wall
                    elif now>=state['next_control_ns']:
                        started_wall=time.perf_counter_ns();failures=health(now);built=None;requested=None;reason=None;selection_reason=None;command_reason=None;inference_ms=0.
                        state['next_control_ns']=now+config.control_period_ns
                        if failures:reason='input_health:'+','.join(failures);v=w=0.;executed=0;clearance=None
                        elif now-state['start_ns']>=round(config.maximum_episode_duration_s*1e9):state['phase']='timeout';reason='episode_time_limit';v=w=0.;executed=0;clearance=None
                        elif core.goal_reached(buffer.state):state['phase']='complete';reason='base_goal_reached';v=w=0.;executed=0;clearance=None
                        else:
                            if now>=state['next_decision_ns']:
                                history,buckets,stamps=buffer.model_history()
                                built=core.build(buffer.state,buffer.fresh,history,(now-state['start_ns'])/1e9,now)
                                selected,requested,selection_reason,inference_ms=core.select(built);reason=selection_reason;state['selected_action']=selected
                                state['next_decision_ns']=now+config.decision_period_ns
                            v,w,executed,command_reason,clearance=core.command(buffer.state,buffer.fresh,state['selected_action'],now)
                            reason=command_reason or reason
                        end_ns=rospy.Time.now().to_nsec();elapsed_ms=(time.perf_counter_ns()-started_wall)/1e6
                        if end_ns<=0 or health(end_ns):v=w=0.;executed=0;reason='prepublish_input_health_drop'
                        v,w,executed,reason,clearance=prepublish_guard((v,w,executed,reason,clearance),end_ns,None if buffer.state is None else buffer.state['stamp_ns'],None if buffer.fresh is None else buffer.fresh.stamp_ns,elapsed_ms,config)
                        publish(v,w);core.previous_action=executed;core.stop_count=core.stop_count+1 if executed==0 else 0
                        emit(trace_pub,{'stamp_ns':end_ns,'run_id':run_id,'phase':state['phase'],'state_source_ns':None if buffer.state is None else buffer.state['stamp_ns'],
                            'fresh_scan_ns':None if buffer.fresh is None else buffer.fresh.stamp_ns,'proposed':requested,'chosen':state['selected_action'],'executed':executed,'v':v,'w':w,
                            'stop_reason':reason,'selection_reason':selection_reason,'command_stop_reason':command_reason,'input_failures':failures,
                            'inference_deadline_dropped':selection_reason=='inference_deadline_drop','transaction_deadline_dropped':elapsed_ms>config.transaction_deadline_ms,'backup_prediction_clearance_m':clearance,'inference_ms':inference_ms,'transaction_ms':elapsed_ms,'physical_safety_certified':False})
                        if built is not None:
                            emit(candidate_pub,{'stamp_ns':now,'mask':built['mask'],'backup_clearances':built['backup_clearances'],'actions':ACTION_NAMES,'model_bucket_ns':buckets,'model_scan_ns':stamps,
                                'prediction':built['prediction_audit'],'candidate_features':built['observation']['candidates'].tolist(),'previews':[[[r['base'][0],r['base'][1],r['base'][2],r['vx'],r['steer']] for r in preview] for preview in built['previews']]})
                except Exception as exc:fault(type(exc).__name__+':'+str(exc))
                if wall-last_status>=.2:
                    emit(status_pub,dict(state,run_id=run_id,seed=seed));emit_method_status(rospy.Time.now().to_nsec());last_status=wall
            time.sleep(.002)
    finally:publish()


if __name__=='__main__':main()
