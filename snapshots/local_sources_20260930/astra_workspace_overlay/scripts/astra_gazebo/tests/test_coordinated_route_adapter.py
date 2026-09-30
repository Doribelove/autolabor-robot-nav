"""Coordinated route node: real ROS messages, fully fake transport, no ROS."""
import hashlib
import io
import math
import sys
import xml.etree.ElementTree as ET
from test_route_runtime_core import SCRIPTS,STAGE,URDF
from common_route import load_bundle

def test_coordinated_remap_and_start_callback_lock_race(monkeypatch):
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
    params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'/robot_description':URDF,'~run_id':'offline_transport_fixture','~seed':seed,'~method':'astra_analytic_route_v1','~bundle':str(directory),'~bundle_sha256':hashlib.sha256((directory/'common_route.json').read_bytes()).hexdigest()}
    publishers={};subscribers={};services={};clock={'ns':1_000_000_000,'ticks':0};race={'armed':False,'fired':False};started=[];fake=types.ModuleType('rospy')
    class Publisher:
        def __init__(self,topic,kind,**kwargs):self.messages=[];self.resolved_name='/astra/method_cmd' if topic=='/cmd_vel_navigation' else topic;publishers[topic]=self
        def publish(self,message):
            stream=io.BytesIO();message.serialize(stream);assert stream.getvalue();self.messages.append(message)
    class Buffer:
        def __init__(self,**kwargs):pass
        def lookup_transform(self,target,source,stamp,duration):
            result=TransformStamped();result.header.stamp=stamp;result.header.frame_id=target;result.child_frame_id=source
            result.transform.translation.x,result.transform.translation.y=data['start_xy_yaw'][:2];result.transform.translation.z=.6;result.transform.rotation.w=1.;return result
    def stamp():return genpy.Time(clock['ns']//1_000_000_000,clock['ns']%1_000_000_000)
    def advance(_duration):
        clock['ticks']+=1;clock['ns']+=50_000_000;t=stamp()
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
            started.append(services['/astra/route_runtime/start'](None))
        if clock['ticks']==24:
            subscribers['/astra/navigation/gate'](String(data=json.dumps({'schema':'astra_common_navigation_v1','run_id':'offline_transport_fixture','seed':seed,'method':'astra_analytic_route_v1','source_ns':clock['ns']})))
            race['armed']=True
        if clock['ticks']==25:started.append(services['/astra/route_runtime/start'](None))
    fake.init_node=lambda _:None;fake.get_name=lambda:'/astra_route_runtime';fake.get_param=lambda name,default=None:params.get(name,default)
    fake.Publisher=Publisher;fake.Subscriber=lambda topic,kind,callback,**kwargs:subscribers.setdefault(topic,callback);fake.Service=lambda name,kind,callback:services.setdefault(name,callback)
    fake.Time=types.SimpleNamespace(now=stamp);fake.Duration=genpy.Duration;fake.is_shutdown=lambda:clock['ticks']>=33
    tf=types.ModuleType('tf2_ros');tf.Buffer=Buffer;tf.TransformListener=lambda _:None;tf.LookupException=tf.ConnectivityException=tf.ExtrapolationException=LookupError
    rosgraph=types.ModuleType('rosgraph');rosgraph.Master=lambda _:types.SimpleNamespace(getSystemState=lambda:([('/astra/method_cmd',['/astra_route_runtime']),('/cmd_vel_navigation',['/astra_navigation_coordinator'])],[],[]))
    monkeypatch.setitem(sys.modules,'rospy',fake);monkeypatch.setitem(sys.modules,'tf2_ros',tf);monkeypatch.setitem(sys.modules,'rosgraph',rosgraph)
    monkeypatch.setenv('ROS_MASTER_URI','http://127.0.0.1:14820');monkeypatch.setenv('THESIS_WS',str(SCRIPTS.parents[1]))
    node_path=SCRIPTS.parents[1]/'src/simulation/m2_gazebo/scripts/astra_route_coordinated_runtime.py';spec=importlib.util.spec_from_file_location('runtime_offline_transport',node_path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    original_rlock=module.threading.RLock
    class StartRaceLock:
        def __init__(self):self.actual=original_rlock()
        def __enter__(self):
            if race['armed']:
                race['armed']=False;race['fired']=True;clock['ns']+=2_000_000
                # Simulate a service callback winning the lock before main.
                # A main-loop timestamp sampled before lock acquisition would
                # now be stale and falsely trip the sensor clock guard.
                started.append(services['/astra/route_runtime/start'](None))
            return self.actual.__enter__()
        def __exit__(self,*args):return self.actual.__exit__(*args)
    monkeypatch.setattr(module.threading,'RLock',StartRaceLock)
    monkeypatch.setattr(module.time,'sleep',advance);module.main()
    assert race['fired']
    assert [result.success for result in started]==[False,True,False]
    traces=[json.loads(message.data) for message in publishers['/astra/route_runtime/trace'].messages];assert len(traces)>=5
    assert all(row['state_source_ns']==row['fresh_scan_ns'] and 0<=row['stamp_ns']-row['state_source_ns']<=200_000_000 for row in traces)
    assert traces[0]['stamp_ns']-traces[0]['state_source_ns']==2_000_000
    assert all(row['physical_safety_certified']is False for row in traces)
    identity=json.loads(publishers['/astra/route_runtime/identity'].messages[0].data)
    assert identity['oracle_input']is False and identity['method']=='astra_analytic_route_v1'
    candidates=[json.loads(message.data) for message in publishers['/astra/route_runtime/candidates'].messages];assert candidates
    assert all(len(set(row['model_scan_ns']))==6 and len(row['mask'])==8 for row in candidates)
    assert set(subscribers)=={'/scan','/odom','/joint_states','/map','/astra/common_route/path','/astra/navigation/gate'}
    assert all(row['phase']!='fault' for row in traces)
    assert identity['execution_protocol']=='astra_common_navigation_v1'
    # Keep callbacks alive after finally, exercising actual captured function
    # objects instead of only parsing source names.
    subscribers['/astra/navigation/gate'](String(data=json.dumps({'schema':'astra_common_navigation_v1','run_id':'offline_transport_fixture','seed':seed,'method':'astra_analytic_route_v1','source_ns':2_200_000_000})))
