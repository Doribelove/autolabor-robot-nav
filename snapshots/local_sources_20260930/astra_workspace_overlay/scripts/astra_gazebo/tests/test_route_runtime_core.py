"""Offline sensor/geometry/reference/transaction tests; no ROS is started."""
import hashlib
import io
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
import pytest
from shapely.geometry import Polygon,Point,box
from shapely.ops import unary_union

SCRIPTS=Path(__file__).resolve().parents[1];sys.path.insert(0,str(SCRIPTS))
from test_common_route import make_grid
from common_route import CommonRoute,load_bundle
from geometry import Pose3
from route_sensor_buffer import RouteSensorBuffer,ScanFrame
from route_candidate_core import RouteAwareRuntime,RouteConfig,FullFootprintEnvelope,actual_center_steering,occupied_union,prepublish_guard
from route_actor_adapter import FEATURE_SHAPES,load_route_actor,package_route_actor,validate_metadata
STAGE=SCRIPTS.parents[2];URDF=(STAGE/'audits/m12_p02_robot_description.xml').read_text()


def actual(stamp=2_000_000_000,pose=(3.,4.,0.),twist=(0.,0.,0.),steer=(0.,0.)):
    return {'stamp_ns':stamp,'pose_map':pose,'body_twist':twist,'roll_pitch':(0.,0.),'joints':{'front_left_steer_joint':steer[0],'front_right_steer_joint':steer[1]}}


def scan_frame(stamp=2_000_000_000,points=()):
    return ScanFrame(stamp,np.full(72,8.,np.float32),np.asarray(points,dtype=float).reshape(-1,2),(3,4,0))


def insert_scan(buffer,stamp,now=None,ranges=None,**kwargs):
    return buffer.scan(stamp,stamp,np.full(720,np.inf) if ranges is None else ranges,-math.pi,2*math.pi/719,.1,30.,(0.,0.,0.),stamp if now is None else now,**kwargs)


@pytest.fixture(scope='module')
def core():
    grid,_,_=make_grid(size=140)
    return RouteAwareRuntime(grid,CommonRoute([(2,7),(12,7)]),URDF)


def test_fresh_scan_is_not_decimated_with_six_integer_model_buckets():
    b=RouteSensorBuffer();epoch=1_000_000_001
    stamps=[epoch+round(i*1e9/15) for i in range(17)]
    for stamp in stamps:insert_scan(b,stamp)
    values,buckets,sources=b.model_history()
    assert b.fresh_count==17 and b.history_count==6 and b.fresh.stamp_ns==stamps[-1]
    assert buckets==[epoch+i*200_000_000 for i in range(6)]
    assert sources==[max(s for s in stamps if s<=bucket) for bucket in buckets]
    assert len(set(sources))==6 and values.shape==(6,72)
    insert_scan(b,epoch+1_199_999_999)
    assert b.history_count==6 and b.fresh_count==18
    insert_scan(b,epoch+1_200_000_000)
    assert b.history_count==7


def test_no_history_padding_and_missing_bucket_resets_warmup():
    b=RouteSensorBuffer();insert_scan(b,1_000_000_000)
    with pytest.raises(ValueError,match='fabricated'):b.model_history()
    for i in range(1,6):insert_scan(b,1_000_000_000+i*200_000_000)
    assert len(b.history)==6
    insert_scan(b,3_000_000_000)
    assert len(b.history)<6
    with pytest.raises(ValueError):b.model_history()


@pytest.mark.parametrize('case',['stale','future','tf','zero','nan','negative_inf','below_min','replay','clock_rollback','partial_scan','bad_quaternion'])
def test_sensor_invalidity_fails_closed(case):
    b=RouteSensorBuffer();insert_scan(b,1_000_000_000)
    ranges=np.full(720,np.inf);args=dict(stamp_ns=1_100_000_000,tf_stamp_ns=1_100_000_000,ranges=ranges,angle_min=-math.pi,angle_increment=2*math.pi/719,range_min=.1,range_max=30.,laser_pose_map=(0,0,0),now_ns=1_100_000_000)
    if case=='stale':args['now_ns']+=200_000_001
    if case=='future':args['stamp_ns']+=1;args['tf_stamp_ns']+=1
    if case=='tf':args['tf_stamp_ns']+=1
    if case=='zero':args['stamp_ns']=0
    if case=='nan':ranges[44]=math.nan
    if case=='negative_inf':ranges[44]=-math.inf
    if case=='below_min':ranges[44]=.09
    if case=='replay':args['stamp_ns']=args['tf_stamp_ns']=1_000_000_000
    if case=='clock_rollback':args['now_ns']=999_999_999
    if case=='partial_scan':args['angle_min']=-1.
    if case=='bad_quaternion':args['laser_quaternion_xyzw']=(0,0,0,2)
    with pytest.raises((ValueError,RuntimeError)):b.scan(**args)
    if case=='clock_rollback':
        with pytest.raises(RuntimeError):b.health(2_000_000_000)


def test_full_quaternion_scan_projection_keeps_roll_pitch_geometry():
    b=RouteSensorBuffer();values=np.full(720,np.inf);values[111]=2.7;values[507]=4.
    pose=Pose3.from_xyz_rpy((2,3,.6),(.2,-.1,1.1));frame=b.scan(1_000_000_000,1_000_000_000,values,-math.pi,2*math.pi/719,.1,30.,(2,3,1.1),1_000_000_000,pose.xyzw)
    expected=[pose.transform((values[i]*math.cos(-math.pi+i*2*math.pi/719),values[i]*math.sin(-math.pi+i*2*math.pi/719),0))[:2] for i in (111,507)]
    assert frame.points_map==pytest.approx(np.asarray(expected))


def test_exact_actual_state_join_never_uses_nearest_joint_or_target():
    b=RouteSensorBuffer();names=['front_left_steer_joint','front_right_steer_joint']
    b.odom(100,(1,2,.3),(1,.2,.4));b.joint(101,names,[.2,.19]);assert b.state is None
    b.joint(100,names,[.3,.25]);assert b.state['stamp_ns']==100 and b.state['body_twist']==(1,.2,.4)
    b.odom(101,(1,2,.3),(1,.3,.4));assert b.state['joints'][names[0]]==.2
    with pytest.raises(ValueError):b.joint(101,names,[.2,.19])


def test_front_and_rear_twists_and_measured_steering_are_distinct(core):
    state=core.state(actual(pose=(3,4,math.pi/2),twist=(1,.2,.4),steer=(.2,.19)))
    assert state['rear']==pytest.approx((3,3.675,math.pi/2))
    assert state['vy']==pytest.approx(.07) and state['w']==.4
    assert state['steer']==pytest.approx(actual_center_steering(.2,.19)[0])
    assert state['steer']!=pytest.approx(math.atan(.65*.4))
    next_state=core.step(state,2,.05,hold=True)
    assert next_state['vy']==.07 and math.dist(next_state['rear'][:2],next_state['base'][:2])==pytest.approx(.325)


def test_envelope_covers_all_nine_actual_collision_parts_with_steering_and_tilt(core):
    envelope=core.footprint;assert envelope.component_count==9
    rng=np.random.default_rng(171)
    for _ in range(80):
        yaw=rng.uniform(-math.pi,math.pi);roll=rng.uniform(-.025,.025);pitch=rng.uniform(-.025,.025)
        pose=Pose3.from_xyz_rpy((4,7,.01),(roll,pitch,yaw))
        joints={name:float(rng.uniform(-.7446,.7446) if 'steer' in name else rng.uniform(-100,100)) for name,j in envelope.model.joints.items() if j.kind!='fixed'}
        shape=envelope.at((4,7,yaw))
        for part in envelope.model.project(pose,joints):
            assert shape.buffer(1e-9).covers(Polygon(part.polygon_xy)),part.link_name


def test_occupied_union_is_exact_cell_union_not_simplified_free_space():
    grid,_,_=make_grid(size=30,obstacles=[(5,14,8,11),(19,22,10,25)])
    exact=unary_union([box(c*.1,r*.1,(c+1)*.1,(r+1)*.1) for r,c in np.argwhere(grid.pixels!=254)])
    assert occupied_union(grid).symmetric_difference(exact).area<1e-12


def test_fast_projection_agrees_shared_polyline_and_uses_y_progress():
    grid,_,_=make_grid(size=140);runtime=RouteAwareRuntime(grid,CommonRoute([(2,2),(3,2),(3,11)]),URDF)
    for point in np.random.default_rng(4).uniform(1,12,(100,2)):
        expected=runtime.route.project(point);observed=runtime.project(point)
        for key in ('arc_m','lateral_m','heading_rad','distance_m','segment'):assert observed[key]==pytest.approx(expected[key])
    assert runtime.project((3,9))['arc_m']==pytest.approx(8.)


def test_full_footprint_rejects_center_clear_fixture_and_stop_is_not_always_allowed():
    grid,_,_=make_grid(size=100,obstacles=[(46,50,52,56)])
    runtime=RouteAwareRuntime(grid,CommonRoute([(2,4),(8,4)]),URDF)
    state=actual(pose=(5,4,0));fresh=scan_frame()
    assert grid.clearance((5,4))>.33
    built=runtime.build(state,fresh,np.full((6,72),8.),0,2_000_000_000)
    assert not any(built['mask']) and built['backup_clearances'][0]<0
    assert runtime.command(state,fresh,0,2_000_000_000)[3]=='backup_revalidation_failed'


def test_committed_action_rechecked_with_new_scan_before_next_decision(core):
    state=actual(pose=(3,7,0));clear=scan_frame();assert core.command(state,clear,3,2_000_000_000)[2]==3
    fresh=scan_frame(2_050_000_000,[(3.8,7)])
    updated=actual(2_050_000_000,pose=(3,7,0))
    command=core.command(updated,fresh,3,2_050_000_000)
    assert command[:3]==(0.,0.,0) and command[3]=='backup_revalidation_failed'


def test_stop_does_not_command_in_place_ackermann_rotation(core):
    state=actual(pose=(3,7,0),steer=(.3,.23));v,w,executed,reason,_=core.command(state,scan_frame(),0,2_000_000_000)
    assert (v,w,executed)==(0.,0.,0) and reason=='backup_stop_requested'


def test_route_nominal_prediction_obeys_frozen_reference_bounds_and_base_goal(core):
    state=core.state(actual(pose=(3,7,0)))
    trajectory=core.rollout(state,6)
    assert max(abs(row['steer']) for row in trajectory)<=.4964+1e-12
    for a,b in zip(trajectory,trajectory[1:]):
        dt=b['t']-a['t'];assert -.9-1e-10<=(b['vx']-a['vx'])/dt<=.6+1e-10
        assert abs(b['steer']-a['steer'])/dt<=.75+1e-10
    assert not core.goal_reached(actual(pose=(11.9,7,.4)))
    assert core.goal_reached(actual(pose=(11.9,7,.1)))
    assert not core.goal_reached(actual(pose=(11.9,7.3,0)))


@pytest.mark.parametrize('case',['tilt','speed','nonfinite','steer'])
def test_actual_state_outside_support_is_rejected(core,case):
    row=actual()
    if case=='tilt':row['roll_pitch']=(.03,.03)
    if case=='speed':row['body_twist']=(3,0,0)
    if case=='nonfinite':row['body_twist']=(0,math.nan,0)
    if case=='steer':row['joints']['front_left_steer_joint']=.75
    with pytest.raises(ValueError):core.state(row)


def test_checkpoint_semantics_ignore_only_seed_and_preserve_physical_gain(core):
    tree=ET.fromstring(URDF);plugin=next(p for p in tree.findall('.//plugin') if p.get('filename')=='libastra_physical_ackermann_p02_plugin.so')
    plugin.find('dynamicsSeed').text='260999999'
    seeded=RouteAwareRuntime(core.grid,core.route,ET.tostring(tree,encoding='unicode'))
    assert seeded.semantics_sha256==core.semantics_sha256
    plugin.find('steeringKd').text='6'
    gained=RouteAwareRuntime(core.grid,core.route,ET.tostring(tree,encoding='unicode'))
    assert gained.semantics_sha256!=core.semantics_sha256


def test_new_artifact_format_rejects_old_weights_and_roundtrips_new_interface(core,monkeypatch):
    monkeypatch.syspath_prepend(str(SCRIPTS.parents[1]/'src/application/v5_e2e_scope_rl/src'))
    import torch
    from v5_e2e_scope_rl.asymmetric_ppo import SensorActor
    torch.manual_seed(93);actor=SensorActor();metadata={'observation_schema':'astra_route_observation_v1','candidate_semantics_sha256':core.semantics_sha256,'same_generator_predictor_verifier_used_in_training':True,'feature_shapes':FEATURE_SHAPES}
    old=io.BytesIO();torch.save(actor.actor_checkpoint(),old)
    with pytest.raises(ValueError,match='old or mismatched'):load_route_actor(old.getvalue(),core,metadata)
    artifact=package_route_actor(actor,core,{'training_run_id':'synthetic_unit_fixture_no_training','candidate_semantics_sha256':core.semantics_sha256})
    buffer=io.BytesIO();torch.save(artifact,buffer);choose=load_route_actor(buffer.getvalue(),core,metadata)
    built=core.build(actual(pose=(3,7,0)),scan_frame(),np.full((6,72),8.),0,2_000_000_000)
    action=choose(built['observation']);assert 0<=action<8 and built['mask'][action]
    for field,value in [('observation_schema','legacy'),('candidate_semantics_sha256','0'*64),('feature_shapes',{})]:
        with pytest.raises(ValueError):validate_metadata(dict(metadata,**{field:value}),core)


@pytest.mark.parametrize('case',['masked','error','late'])
def test_inference_transaction_fallback(core,case):
    previous=core.actor
    built={'observation':{'action_mask':np.asarray([False,True,True,False,False,False,False,False]),'analytic_prior':np.arange(8.)}}
    core.actor=(lambda _:0) if case=='masked' else ((lambda _:1) if case=='late' else (lambda _:1/0))
    times=iter([10,30_000_011] if case=='late' else [10,11])
    chosen,proposed,reason,latency=core.select(built,lambda:next(times));core.actor=previous
    assert chosen==2 and reason=={'masked':'masked_action_drop','error':'inference_error:ZeroDivisionError','late':'inference_deadline_drop'}[case]
    if case=='error':assert proposed is None


@pytest.mark.parametrize('case',['stale_state','stale_scan','future','late','nan','valid'])
def test_prepublish_gate_drops_late_or_stale_proposal(core,case):
    now=2_000_000_000;odom=scan=now;elapsed=30.;command=(.4,.1,2,None,.7)
    if case=='stale_state':odom-=200_000_001
    if case=='stale_scan':scan-=200_000_001
    if case=='future':scan+=1
    if case=='late':elapsed=50.0001
    if case=='nan':command=(.4,math.nan,2,None,.7)
    result=prepublish_guard(command,now,odom,scan,elapsed,core.config)
    assert (result==command) is (case=='valid')
    if case!='valid':assert result[:3]==(0.,0.,0)


def test_new_node_has_sensor_only_dependencies_and_no_ros_run_in_this_test():
    node=(SCRIPTS.parents[1]/'src/simulation/m2_gazebo/scripts/astra_route_runtime.py').read_text()
    compile(node,'astra_route_runtime.py','exec')
    for forbidden in ('ModelStates','LinkStates','GetModelState','/astra/world_models','/astra/world_links','/gazebo/model_states','set_param','dynamic_reconfigure'):
        assert forbidden not in node
    assert "'/cmd_vel_navigation'" in node and 'prepublish_guard' in node


def test_standalone_node_real_ros_messages_with_fake_transport_and_start(monkeypatch):
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
    directory=next((STAGE/'audits/common_route_components/samples').glob('*width_change*'))
    grid,route,data=load_bundle(directory);tree=ET.fromstring(URDF)
    plugin=next(p for p in tree.findall('.//plugin') if p.get('filename')=='libastra_physical_ackermann_p02_plugin.so');seed=int(plugin.findtext('dynamicsSeed'))
    params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'/robot_description':URDF,'~run_id':'offline_transport_fixture','~seed':seed,'~method':'astra_analytic_route_v1','~bundle':str(directory),'~bundle_sha256':hashlib.sha256((directory/'common_route.json').read_bytes()).hexdigest()}
    publishers={};subscribers={};services={};clock={'ns':1_000_000_000,'ticks':0};started=[];fake=types.ModuleType('rospy')
    class Publisher:
        def __init__(self,topic,kind,**kwargs):self.messages=[];publishers[topic]=self
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
        if clock['ticks']==24:started.append(services['/astra/route_runtime/start'](None))
        if clock['ticks']==25:started.append(services['/astra/route_runtime/start'](None))
    fake.init_node=lambda _:None;fake.get_name=lambda:'/astra_route_runtime';fake.get_param=lambda name,default=None:params.get(name,default)
    fake.Publisher=Publisher;fake.Subscriber=lambda topic,kind,callback,**kwargs:subscribers.setdefault(topic,callback);fake.Service=lambda name,kind,callback:services.setdefault(name,callback)
    fake.Time=types.SimpleNamespace(now=stamp);fake.Duration=genpy.Duration;fake.is_shutdown=lambda:clock['ticks']>=33
    tf=types.ModuleType('tf2_ros');tf.Buffer=Buffer;tf.TransformListener=lambda _:None;tf.LookupException=tf.ConnectivityException=tf.ExtrapolationException=LookupError
    rosgraph=types.ModuleType('rosgraph');rosgraph.Master=lambda _:types.SimpleNamespace(getSystemState=lambda:([('/cmd_vel_navigation',['/astra_route_runtime'])],[],[]))
    monkeypatch.setitem(sys.modules,'rospy',fake);monkeypatch.setitem(sys.modules,'tf2_ros',tf);monkeypatch.setitem(sys.modules,'rosgraph',rosgraph)
    monkeypatch.setenv('ROS_MASTER_URI','http://127.0.0.1:14820');monkeypatch.setenv('THESIS_WS',str(SCRIPTS.parents[1]))
    node_path=SCRIPTS.parents[1]/'src/simulation/m2_gazebo/scripts/astra_route_runtime.py';spec=importlib.util.spec_from_file_location('runtime_offline_transport',node_path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module.time,'sleep',advance);module.main()
    assert [result.success for result in started]==[False,True,False]
    traces=[json.loads(message.data) for message in publishers['/astra/route_runtime/trace'].messages];assert len(traces)>=5
    assert all(row['state_source_ns']==row['fresh_scan_ns']==row['stamp_ns'] for row in traces)
    assert all(row['physical_safety_certified']is False for row in traces)
    identity=json.loads(publishers['/astra/route_runtime/identity'].messages[0].data)
    assert identity['oracle_input']is False and identity['method']=='astra_analytic_route_v1'
    candidates=[json.loads(message.data) for message in publishers['/astra/route_runtime/candidates'].messages];assert candidates
    assert all(len(set(row['model_scan_ns']))==6 and len(row['mask'])==8 for row in candidates)
    assert set(subscribers)=={'/scan','/odom','/joint_states','/map','/astra/common_route/path'}


def test_nominal_detour_does_not_stop_just_outside_terminal_base_goal():
    # Regression for a real shared static-input guide, using only the nominal
    # plant model. This is not physical navigation evidence.
    directory=next((STAGE/'audits/route_runtime_components_v2/new_guide_radius_1m').glob('*static_avoidance*'))
    grid,route,data=load_bundle(directory);runtime=RouteAwareRuntime(grid,route,URDF)
    state=runtime.state(actual(pose=tuple(data['start_xy_yaw'])));chosen=0
    for tick in range(440):
        now=2_000_000_000+tick*50_000_000;k=math.tan(state['steer'])/.65
        row=actual(now,state['base'],(state['vx'],state['vy']+.325*state['w'],state['w']),(math.atan(.65*k/(1-.3*k)),math.atan(.65*k/(1+.3*k))))
        if runtime.goal_reached(row):break
        fresh=scan_frame(now)
        if tick%5==0:chosen=runtime.select(runtime.build(row,fresh,np.full((6,72),8.),tick*.05,now))[0]
        executed=runtime.command(row,fresh,chosen,now)[2]
        state=runtime.step(runtime.state(row),executed,.05);runtime.previous_action=executed
    assert runtime.goal_reached(row)
    assert tick*.05<22.


def test_real_p05_interface_identity_is_explicit_and_not_qualification(core):
    xml=(STAGE/'audits/m12_p05_driver_ingress_implementation/p05_expanded.xml').read_text()
    runtime=RouteAwareRuntime(core.grid,core.route,xml)
    assert runtime.backend['library_filename']=='libastra_physical_ackermann_p05_plugin.so'
    assert 'pending_external_stage_result_review' in runtime.backend['qualification']
    assert runtime.footprint.component_count==9 and runtime.footprint.sha256==core.footprint.sha256
    assert runtime.semantics_sha256!=core.semantics_sha256
    assert runtime.identity['physical_safety_certified']is False
    changed=xml.replace('libastra_physical_ackermann_p05_plugin.so','libm2_ackermann_plugin.so')
    with pytest.raises(ValueError,match='explicitly accepted'):RouteAwareRuntime(core.grid,core.route,changed)


def test_mass_friction_and_duplicate_parameters_cannot_silently_reuse_semantics(core):
    for tag in ('mass','mu1'):
        tree=ET.fromstring(URDF);element=tree.find('.//'+tag);assert element is not None
        if tag=='mass':element.set('value',str(float(element.get('value'))+.1))
        else:element.text=str(float(element.text)+.1)
        runtime=RouteAwareRuntime(core.grid,core.route,ET.tostring(tree,encoding='unicode'))
        assert runtime.semantics_sha256!=core.semantics_sha256
    tree=ET.fromstring(URDF);plugin=next(p for p in tree.findall('.//plugin') if p.get('filename')=='libastra_physical_ackermann_p02_plugin.so')
    ET.SubElement(plugin,'steeringKd').text='99'
    with pytest.raises(ValueError,match='duplicate'):RouteAwareRuntime(core.grid,core.route,ET.tostring(tree,encoding='unicode'))
