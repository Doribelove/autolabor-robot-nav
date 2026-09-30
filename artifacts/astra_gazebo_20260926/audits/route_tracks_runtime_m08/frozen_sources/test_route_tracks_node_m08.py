"""Coordinated route node: real ROS messages, fully fake transport, no ROS."""
import pytest
import time
import hashlib
import io
import math
import sys
import xml.etree.ElementTree as ET
from test_route_runtime_core import SCRIPTS,STAGE,URDF
from common_route import load_bundle

@pytest.mark.parametrize('mode',['nominal','complete_then_fault','wrong_tf','source_stall','late_transaction','source_expires_before_publish'])
def test_actual_new_node_callbacks_and_shared_terminal(monkeypatch,mode):
    # Actual generated ROS messages serialize. Transport, clock, TF and services
    # are offline stubs; no ROS master, simulator or process is launched.
    import importlib.util
    import json
    import types
    monkeypatch.syspath_prepend('/opt/ros/noetic/lib/python3/dist-packages')
    import genpy
    from nav_msgs.msg import OccupancyGrid,Path as RosPath,Odometry
    from geometry_msgs.msg import PoseStamped,TransformStamped
    from sensor_msgs.msg import LaserScan,JointState
    from std_msgs.msg import String
    directory=next((STAGE/'audits/common_route_components/samples').glob('*width_change*'))
    grid,route,data=load_bundle(directory);tree=ET.fromstring(URDF)
    plugin=next(p for p in tree.findall('.//plugin') if p.get('filename')=='libastra_physical_ackermann_p02_plugin.so');seed=int(plugin.findtext('dynamicsSeed'))
    params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'/robot_description':URDF,'~run_id':'offline_transport_fixture','~seed':seed,'~method':'astra_analytic_route_tracks_m08','~bundle':str(directory),'~bundle_sha256':hashlib.sha256((directory/'common_route.json').read_bytes()).hexdigest()}
    publishers={};subscribers={};services={};clock={'ns':1_000_000_000,'ticks':0,'wall':0.};race={'armed':False,'fired':False};started=[];fake=types.ModuleType('rospy')
    class Publisher:
        def __init__(self,topic,kind,**kwargs):self.messages=[];self.resolved_name='/astra/method_cmd' if topic=='/cmd_vel_navigation' else topic;publishers[topic]=self
        def publish(self,message):
            stream=io.BytesIO();message.serialize(stream);assert stream.getvalue();self.messages.append(message)
    class Buffer:
        def __init__(self,**kwargs):pass
        def lookup_transform(self,target,source,stamp,duration):
            result=TransformStamped();result.header.stamp=stamp;result.header.frame_id=target;result.child_frame_id=source
            if mode=='wrong_tf' and clock['ticks']>=28:result.header.stamp=genpy.Time(1)
            result.transform.translation.x,result.transform.translation.y=data['start_xy_yaw'][:2];result.transform.translation.z=.6;result.transform.rotation.w=1.;return result
    def stamp():return genpy.Time(clock['ns']//1_000_000_000,clock['ns']%1_000_000_000)
    def advance(_duration):
        clock['ticks']+=1;clock['wall']+=.05
        if mode=='source_stall' and clock['ticks']>=29:clock['wall']+=6.;return
        clock['ns']+=50_000_000;t=stamp()
        odom=Odometry();odom.header.stamp=t;odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.pose.pose.orientation.w=1.
        # At route start, source odom reference is the provider's declared SE2.
        from route_world_adapter import RouteFrame
        base=RouteFrame(*data['map_from_odom_xy_yaw']).pose_to_route(data['start_xy_yaw']);odom.pose.pose.position.x,odom.pose.pose.position.y=base[:2]
        odom.pose.pose.orientation.z=math.sin(base[2]/2);odom.pose.pose.orientation.w=math.cos(base[2]/2)
        joints=JointState();joints.header.stamp=t;joints.name=['front_left_steer_joint','front_right_steer_joint'];joints.position=[0,0]
        scan=LaserScan();scan.header.stamp=t;scan.header.frame_id='laser_link';scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/719;scan.range_min=.1;scan.range_max=30.;scan.ranges=[math.inf]*720
        if clock['ticks']==1:
            for topic,empty in (('/odom',Odometry()),('/joint_states',JointState()),('/scan',LaserScan())):subscribers[topic](empty)
        subscribers['/odom'](odom);subscribers['/joint_states'](joints);subscribers['/scan'](scan)
        if clock['ticks']==1:
            mapping=OccupancyGrid();mapping.header.frame_id='map';mapping.info.width=grid.width;mapping.info.height=grid.height;mapping.info.resolution=grid.resolution
            mapping.info.origin.position.x,mapping.info.origin.position.y=grid.origin[:2];mapping.info.origin.orientation.w=1.;mapping.data=grid.blocking_occupancy.ravel().tolist();subscribers['/map'](mapping)
            path=RosPath();path.header.frame_id='map'
            for index,xy in enumerate(route.points):
                pose=PoseStamped();pose.header.frame_id='map';pose.pose.position.x,pose.pose.position.y=xy;yaw=route.goal_yaw if index==len(route.points)-1 else route.sample(route.arcs[index])[2]
                pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);path.poses.append(pose)
            subscribers['/astra/common_route/path'](path)
            started.append(services['/astra/route_tracks_m08/start'](None))
        if clock['ticks']==24:
            subscribers['/astra/navigation/gate'](String(data=json.dumps({'schema':'astra_common_navigation_v1','run_id':'offline_transport_fixture','seed':seed,'method':'astra_analytic_route_tracks_m08','source_ns':clock['ns'],'deadline_ns':clock['ns']+22_000_000_000})))
            race['armed']=True
        if mode=='complete_then_fault' and clock['ticks']==29:
            subscribers['/astra/navigation/status'](String(data=json.dumps(dict(schema='astra_common_navigation_v1',run_id='offline_transport_fixture',seed=seed,method='astra_analytic_route_tracks_m08',phase='complete',terminal_source_ns=clock['ns'],terminal_reason='base_goal_reached',terminal_time_authority='actual_odom_header'))))
        if mode=='complete_then_fault' and clock['ticks']==30:
            bad=LaserScan();bad.header.stamp=stamp();bad.header.frame_id='bad_frame';subscribers['/scan'](bad)
        if clock['ticks']==25:started.append(services['/astra/route_tracks_m08/start'](None))
    fake.init_node=lambda _:None;fake.get_name=lambda:'/astra_route_tracks_runtime_m08';fake.get_param=lambda name,default=None:params.get(name,default)
    fake.Publisher=Publisher;fake.Subscriber=lambda topic,kind,callback,**kwargs:subscribers.setdefault(topic,callback);fake.Service=lambda name,kind,callback:services.setdefault(name,callback)
    fake.Time=types.SimpleNamespace(now=stamp);fake.Duration=genpy.Duration;fake.is_shutdown=lambda:clock['ticks']>=33
    tf=types.ModuleType('tf2_ros');tf.Buffer=Buffer;tf.TransformListener=lambda _:None;tf.LookupException=tf.ConnectivityException=tf.ExtrapolationException=LookupError
    rosgraph=types.ModuleType('rosgraph');rosgraph.Master=lambda _:types.SimpleNamespace(getSystemState=lambda:([('/astra/method_cmd',['/astra_route_tracks_runtime_m08']),('/cmd_vel_navigation',['/astra_navigation_coordinator'])],[],[]))
    monkeypatch.setitem(sys.modules,'rospy',fake);monkeypatch.setitem(sys.modules,'tf2_ros',tf);monkeypatch.setitem(sys.modules,'rosgraph',rosgraph)
    monkeypatch.setenv('ROS_MASTER_URI','http://127.0.0.1:14820');monkeypatch.setenv('THESIS_WS',str(SCRIPTS.parents[1]))
    node_path=SCRIPTS.parents[1]/'src/simulation/m2_gazebo/scripts/astra_route_tracks_runtime_m08.py';spec=importlib.util.spec_from_file_location('runtime_offline_transport',node_path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    original_rlock=module.threading.RLock
    class StartRaceLock:
        def __init__(self):self.actual=original_rlock()
        def __enter__(self):
            if race['armed']:
                race['armed']=False;race['fired']=True;clock['ns']+=2_000_000
                # Simulate a service callback winning the lock before main.
                # A main-loop timestamp sampled before lock acquisition would
                # now be stale and falsely trip the sensor clock guard.
                started.append(services['/astra/route_tracks_m08/start'](None))
            return self.actual.__enter__()
        def __exit__(self,*args):return self.actual.__exit__(*args)
    module.threading=types.SimpleNamespace(RLock=StartRaceLock)
    module.time=types.SimpleNamespace(sleep=advance,monotonic=lambda:clock['wall'],perf_counter_ns=time.perf_counter_ns)
    import route_tracks_sensor_io_m08 as bridge
    original=bridge.SharedLifecycle;instances=[]
    def capture(*args):
        instance=original(*args);instances.append(instance);return instance
    monkeypatch.setattr(bridge,'SharedLifecycle',capture)
    import route_candidate_core_m08 as runtime
    original_transaction=runtime.RouteTracksRuntime.transaction
    def altered_transaction(self,*args,**kwargs):
        if mode=='late_transaction':time.sleep(.052)
        result=original_transaction(self,*args,**kwargs)
        if mode=='source_expires_before_publish':clock['ns']+=200_000_001
        return result
    monkeypatch.setattr(runtime.RouteTracksRuntime,'transaction',altered_transaction);module.main()
    assert race['fired']
    assert [result.success for result in started]==[False,True,False]
    traces=[json.loads(message.data) for message in publishers['/astra/route_tracks_m08/trace'].messages];assert len(traces)>=2
    assert all(row['state_source_ns']==row['fresh_scan_ns'] for row in traces)
    if mode!='source_expires_before_publish':assert all(0<=row['stamp_ns']-row['state_source_ns']<=200_000_000 for row in traces)
    if mode!='source_expires_before_publish':assert traces[0]['stamp_ns']-traces[0]['state_source_ns']==2_000_000
    assert all(row['physical_safety_certified']is False for row in traces)
    identity=json.loads(publishers['/astra/route_tracks_m08/identity'].messages[0].data)
    assert identity['oracle_input']is False and identity['method']=='astra_analytic_route_tracks_m08'
    candidates=[json.loads(message.data) for message in publishers['/astra/route_tracks_m08/candidates'].messages];assert candidates
    assert all(len(set(row['model_scan_ns']))==6 and len(row['mask'])==8 for row in candidates)
    assert set(subscribers)=={'/scan','/odom','/joint_states','/map','/astra/common_route/path','/astra/navigation/gate','/astra/navigation/status'}
    assert all(row['phase']!='fault' for row in traces)
    assert identity['execution_protocol']=='astra_common_navigation_v1'
    state=instances[0].state
    if mode=='complete_then_fault':
        assert state['phase']=='complete' and state['shared_terminal']['terminal_reason']=='base_goal_reached'
        assert state['late_fault_count']>=1
    elif mode=='wrong_tf':assert state['phase']=='fault' and 'different scan epoch' in state['fault']
    elif mode=='source_stall':assert state['phase']=='fault' and state['fault']=='source_clock_stall'
    else:assert state['phase']=='running' and state['fault'] is None
    if mode=='late_transaction':assert all(row['transaction_deadline_dropped'] and row['v']==row['w']==0 and row['executed']==0 for row in traces)
    if mode=='source_expires_before_publish':
        assert all(row['v']==row['w']==0 and row['executed']==0 for row in traces)
        assert any(row['stop_reason']=='prepublish_input_health_drop' for row in traces)
    assert '/cmd_vel_navigation' not in publishers and '/astra/method_cmd' in publishers
    assert publishers['/astra/method_cmd'].messages[-1].linear.x==0
    # Exercise exact captured callbacks after cleanup; no callback name shadowing.
    callbacks_before=dict(subscribers)
    subscribers['/astra/navigation/status'](String(data=json.dumps(dict(schema='astra_common_navigation_v1',run_id='offline_transport_fixture',seed=seed,method='astra_analytic_route_tracks_m08',phase='complete',terminal_source_ns=clock['ns'],terminal_reason='base_goal_reached',terminal_time_authority='actual_odom_header'))))
    assert all(callbacks_before[k] is v for k,v in subscribers.items())
