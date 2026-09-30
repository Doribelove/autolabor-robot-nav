"""Offline actual-code regression tests. No ROS master, launch or run allocation."""
import ast
import copy
import hashlib
import importlib.util
import inspect
import json
import math
from pathlib import Path
import sys
import threading
import types

import numpy as np
import pytest

SCRIPTS = Path(__file__).resolve().parents[1]
W = SCRIPTS.parents[1]
S = W.parent
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
from frozen_r2_sensor_lifecycle import (METHOD, PROTOCOL, FROZEN_CHECKPOINT_SHA256,
    LifecycleFault, SharedTerminal, SensorIngress, SharedLifecycle, StreamingTrace,
    authenticate_frozen, verify_static_union)
from common_route import load_bundle
import genpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

NODE = W/'src/simulation/m2_gazebo/scripts/astra_frozen_r2_sensor_lifecycle.py'
SCENE = S/'scenes/navigation_n01_clear_r03'
CHECKPOINT = W/'artifacts/gazebo_pct_teb_dwa_benchmark/development/training/pct_benchmark_r1_seed9701/actor_final.pt'

def stamp(ns):
    return genpy.Time(ns//1_000_000_000,ns%1_000_000_000)

def odom(ns, x=1.4):
    msg=Odometry();msg.header.stamp=stamp(ns);msg.header.frame_id='odom';msg.child_frame_id='base_link'
    msg.pose.pose.orientation.w=1.;msg.pose.pose.position.x=x;msg.twist.twist.linear.x=.8
    return msg

def scan(ns):
    msg=LaserScan();msg.header.stamp=stamp(ns);msg.header.frame_id='laser_link'
    msg.angle_min=-math.pi;msg.angle_max=math.pi;msg.angle_increment=2*math.pi/719
    msg.range_min=.1;msg.range_max=30.;msg.ranges=[math.inf]*720
    return msg

def gate():
    return dict(schema=PROTOCOL,run_id='offline_r2_lifecycle',seed=260926026,method=METHOD,
        source_ns=7_000_000_000,activation_source_ns=5_020_000_000,
        deadline_source_ns=29_000_000_000,pose_map=[-4.,0.,0.],body_twist=[.8,0.,0.],goal_pose=[5.,0.,0.])

def terminal(phase='complete'):
    return dict(schema=PROTOCOL,run_id='offline_r2_lifecycle',seed=260926026,method=METHOD,
        phase=phase,terminal_source_ns=28_980_000_000 if phase=='complete' else 29_000_000_000,
        terminal_time_authority='first_actual_odom_header_meeting_base_goal' if phase=='complete' else 'declared_gate_actual_odom_source_plus22s')

def test_authenticate_original_sources_and_checkpoint():
    assert len(authenticate_frozen(W))==4
    assert hashlib.sha256(CHECKPOINT.read_bytes()).hexdigest()==FROZEN_CHECKPOINT_SHA256

@pytest.mark.parametrize('kind', ['odom','scan'])
def test_future_two_ms_waits_then_accepts_without_stamp_substitution(kind):
    core=SensorIngress(); message=(odom if kind=='odom' else scan)(1_002_000_000); seen=[]
    core.enqueue(kind,message,1_000_000_000)
    core.drain(kind,1_000_000_000,lambda m:seen.append(m))
    assert not seen
    core.drain(kind,1_002_000_000,lambda m:seen.append(m))
    assert seen==[message] and message.header.stamp.to_nsec()==1_002_000_000

@pytest.mark.parametrize('kind',['odom','scan'])
@pytest.mark.parametrize('offset',[-200_000_001,200_000_001])
def test_source_age_or_lead_beyond_bound_rejected(kind,offset):
    with pytest.raises(LifecycleFault):
        SensorIngress().enqueue(kind,(odom if kind=='odom' else scan)(1_000_000_000+offset),1_000_000_000)

@pytest.mark.parametrize('kind',['odom','scan'])
@pytest.mark.parametrize('next_stamp',[1_000_000_000,999_999_999,0])
def test_duplicate_regression_zero_after_real_packet_rejected(kind,next_stamp):
    core=SensorIngress();make=odom if kind=='odom' else scan
    core.enqueue(kind,make(1_000_000_000),1_000_000_000)
    with pytest.raises(LifecycleFault):core.enqueue(kind,make(next_stamp),1_000_000_000)

@pytest.mark.parametrize('kind',['odom','scan'])
def test_bootstrap_zero_discard_is_not_history(kind):
    core=SensorIngress();make=odom if kind=='odom' else scan
    assert core.enqueue(kind,make(0),0) is False
    assert core.zero_discarded[kind]==1 and not core.history_ready() and not core.queues[kind]
    with pytest.raises(LifecycleFault):core.enqueue(kind,make(0),0,active=True)

def test_pending_expiry_clock_regression_and_overflow():
    core=SensorIngress();core.enqueue('scan',scan(1_000_000_000),1_000_000_000)
    core.drain('scan',1_100_000_000,lambda m:False)
    with pytest.raises(LifecycleFault,match='expired'):core.drain('scan',1_200_000_001,lambda m:False)
    with pytest.raises(LifecycleFault,match='regressed'):core.clock(1)
    core=SensorIngress()
    for i in range(100):core.enqueue('odom',odom(1_000_000_000+i),1_000_000_000)
    with pytest.raises(LifecycleFault,match='overflow'):core.enqueue('odom',odom(1_000_000_100),1_000_000_000)

@pytest.mark.parametrize('field', ['frame','child','position_nan','quaternion','twist_inf'])
def test_invalid_odometry_rejected(field):
    m=odom(1_000_000_000)
    if field=='frame':m.header.frame_id='world'
    elif field=='child':m.child_frame_id='rear_axle'
    elif field=='position_nan':m.pose.pose.position.y=math.nan
    elif field=='quaternion':m.pose.pose.orientation.w=.8
    else:m.twist.twist.linear.y=math.inf
    with pytest.raises(LifecycleFault):SensorIngress().enqueue('odom',m,1_000_000_000)

@pytest.mark.parametrize('field',['frame','nan','negative','negative_inf','zero_increment','empty'])
def test_invalid_scan_rejected(field):
    m=scan(1_000_000_000)
    if field=='frame':m.header.frame_id='base_link'
    elif field=='nan':m.ranges[0]=math.nan
    elif field=='negative':m.ranges[0]=-1
    elif field=='negative_inf':m.ranges[0]=-math.inf
    elif field=='zero_increment':m.angle_increment=0
    else:m.ranges=[]
    with pytest.raises(LifecycleFault):SensorIngress().enqueue('scan',m,1_000_000_000)

def test_real_history_count_and_trace_stream_before_return():
    core=SensorIngress()
    for i in range(5):core.appended(1_000_000_000+i*220_000_000)
    assert not core.history_ready()
    core.appended(2_100_000_000);assert core.history_ready()
    events=[];trace=StreamingTrace('decision',events.append);trace.append({'value':math.inf})
    assert events[0]['legacy_row']['value'] is None and events[0]['actor_truth_available'] is False

@pytest.mark.parametrize('mutation', ['seed_bool','method','schema','deadline','deadline_float','activation','activation_late','goal','pose','twist'])
def test_gate_identity_and_protocol_rejection(mutation):
    c=SharedLifecycle('offline_r2_lifecycle',260926026);g=gate()
    if mutation=='seed_bool':g['seed']=True
    elif mutation=='method':g['method']='frozen_r2_v060_shared_activation'
    elif mutation=='schema':g['schema']='other'
    elif mutation=='deadline':g['deadline_source_ns']+=2_000_000
    elif mutation=='deadline_float':g['deadline_source_ns']=float(g['deadline_source_ns'])
    elif mutation=='activation':g['activation_source_ns']=4_999_999_999
    elif mutation=='activation_late':g['activation_source_ns']=5_200_000_001
    elif mutation=='goal':g['goal_pose'][0]=6.
    elif mutation=='pose':g['pose_map'][0]+=.05
    else:g['body_twist'][0]=1.2
    with pytest.raises(LifecycleFault):c.on_gate(g)

def test_terminal_first_write_immutable_and_bad_terminal_rejected():
    c=SharedLifecycle('offline_r2_lifecycle',260926026);c.on_gate(gate());c.on_status(terminal())
    c.fault('late paused clock');c.on_status(terminal('timeout'))
    assert c.terminal['phase']=='complete' and c.fault_reason is None and len(c.late_faults)==2
    c.check()
    c=SharedLifecycle('offline_r2_lifecycle',260926026)
    with pytest.raises(LifecycleFault):c.on_status(terminal())
    c.on_gate(gate());bad=terminal('timeout');bad['terminal_source_ns']-=1
    with pytest.raises(LifecycleFault):c.on_status(bad)

def test_static_union_input_exact_and_mutations_fail():
    import yaml
    grid,_,_=load_bundle(SCENE/'common_bundle');scene=yaml.safe_load((SCENE/'scene_manifest.yaml').read_text())
    assert verify_static_union(scene,grid)['cell_union_mismatch']==0
    for change in ('drop','duplicate','fraction','circle'):
        altered=copy.deepcopy(scene)
        if change=='drop':altered['static_boxes'].pop()
        elif change=='duplicate':altered['static_boxes'].append(altered['static_boxes'][0])
        elif change=='fraction':altered['static_boxes'][0]['x']+=.01
        else:altered['static_circles']=[dict(x=0,y=0,radius=1)]
        with pytest.raises(LifecycleFault):verify_static_union(altered,grid)

@pytest.fixture
def actual_episode(monkeypatch,tmp_path):
    for package in ('v5_e2e_scope_rl','v3_scope_planner'):
        monkeypatch.syspath_prepend(str(W/'src/application'/package/'src'))
    import rospy, tf2_ros
    clock={'ns':1_000_000_000,'tf_fail':False,'tf_offset':0}
    subscribers=[];publishers=[];events=[]
    params={'~method':'pct_scope_rl','~scene_id':'navigation_n01_clear','~seed':260926026,
        '~report_path':str(tmp_path/'unused_report.json'),'~scene_manifest':str(SCENE/'scene_manifest.yaml'),
        '~actor_checkpoint_path':str(CHECKPOINT),'~actor_checkpoint_sha256':FROZEN_CHECKPOINT_SHA256}
    class Pub:
        def __init__(self,topic,*a,**k):self.resolved_name=topic;self.messages=[];publishers.append(self)
        def publish(self,m):self.messages.append(m)
    class Buffer:
        def __init__(self,*a,**k):pass
        def lookup_transform(self,target,source,t,*a):
            if clock['tf_fail']:raise LookupError('offline TF unavailable')
            ns=(t.to_nsec() or clock['ns'])+clock['tf_offset']
            m=TransformStamped();m.header.stamp=stamp(ns);m.header.frame_id=target;m.child_frame_id=source
            m.transform.rotation.w=1.;m.transform.translation.x=1.4
            return m
    monkeypatch.setattr(rospy,'get_param',lambda name,default=None:params.get(name,default))
    monkeypatch.setattr(rospy.Time,'now',staticmethod(lambda:stamp(clock['ns'])))
    monkeypatch.setattr(rospy,'Publisher',Pub)
    monkeypatch.setattr(rospy,'Subscriber',lambda topic,kind,cb,**kw:subscribers.append((topic,cb)))
    monkeypatch.setattr(rospy,'logwarn_throttle',lambda *a:None)
    monkeypatch.setattr(tf2_ros,'Buffer',Buffer);monkeypatch.setattr(tf2_ros,'TransformListener',lambda b:None)
    monkeypatch.setenv('THESIS_WS',str(W))
    spec=importlib.util.spec_from_file_location('offline_frozen_r2_lifecycle_node',NODE)
    node=importlib.util.module_from_spec(spec);spec.loader.exec_module(node)
    r2,bindings=node.load_frozen(W)
    lifecycle=SharedLifecycle('offline_r2_lifecycle',260926026);ingress=SensorIngress()
    lock=threading.RLock();inputs={'ready':True}
    cls=node.make_episode_class(r2,lambda kind,row:events.append((kind,row)),lifecycle,ingress,lock,lambda:inputs['ready'])
    episode=cls()
    return types.SimpleNamespace(node=node,r2=r2,episode=episode,lifecycle=lifecycle,ingress=ingress,
        clock=clock,events=events,subscribers=subscribers,publishers=publishers,params=params,inputs=inputs)

def feed(f,ns):
    f.clock['ns']=ns;f.episode._odom(odom(ns));f.episode._scan(scan(ns));f.episode.drain()

def test_actual_constructor_only_sensor_subscriptions_and_exact_algorithm_function(actual_episode):
    f=actual_episode
    assert [x[0] for x in f.subscribers]==['/odom','/scan']
    assert not hasattr(f.episode,'model_states') and not hasattr(f.episode,'model_index')
    assert 'actors' not in f.episode.scene
    for name in ('execute_sensor','input_health','policy_pose'):
        assert getattr(type(f.episode),name) is getattr(f.r2.r1.BenchmarkEpisode,name)
    assert f.episode.candidate_target_speeds_mps[4:6]==[.6,.6]

def test_actor_runtime_constructor_ast_is_exact_frozen_block():
    original=ast.parse((W/'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py').read_text())
    adapted=ast.parse(NODE.read_text())
    def block(tree):
        init=next(n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef) and n.name=='__init__' and any(isinstance(x,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='actor' for t in x.targets) for x in n.body))
        start=next(i for i,n in enumerate(init.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='actor' for t in n.targets))
        end=next(i for i,n in enumerate(init.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Attribute) and t.attr=='runtime' for t in n.targets))
        return [ast.dump(n,include_attributes=False) for n in init.body[start:end+1]]
    assert block(original)==block(adapted)

def test_six_successful_actual_history_updates_required(actual_episode):
    f=actual_episode;f.lifecycle.on_status(dict(schema=PROTOCOL,run_id='offline_r2_lifecycle',seed=260926026,method=METHOD,phase='bootstrap'))
    assert f.episode.runtime.scan_history.shape==(6,72)
    for i in range(5):feed(f,1_000_000_000+i*220_000_000)
    assert not f.episode.ready()
    feed(f,2_100_000_000);assert f.episode.ready()
    assert len(f.ingress.accepted_scan_ns)==6
    f.inputs['ready']=False;assert not f.episode.ready()

def test_raw_scan_count_does_not_replace_history_count(actual_episode):
    f=actual_episode
    for i in range(10):feed(f,1_000_000_000+i*20_000_000)
    assert len(f.episode.scan_stamps)==10 and len(f.ingress.accepted_scan_ns)==1

def test_tf_failure_pending_and_exact_stamp_guard(actual_episode):
    f=actual_episode;f.clock['tf_fail']=True
    feed(f,1_000_000_000)
    assert not f.ingress.accepted_scan_ns and len(f.ingress.queues['scan'])==1
    f.clock['tf_fail']=False;f.clock['ns']=1_002_000_000;f.episode.drain()
    assert list(f.ingress.accepted_scan_ns)==[1_000_000_000]
    f.clock['tf_offset']=1;feed(f,1_220_000_000)
    assert len(f.ingress.accepted_scan_ns)==1
    f.clock['ns']=1_420_000_001
    with pytest.raises(LifecycleFault,match='expired'):f.episode.drain()

def test_sensor_pose_and_shared_terminal_exclusively_control_stop(actual_episode):
    f=actual_episode;feed(f,7_000_000_000);f.lifecycle.on_gate(gate());f.lifecycle.active=True
    f.episode.model_states='deliberate hostile unused oracle object'
    pose=f.episode.robot_world_pose();assert pose==dict(x_m=-4.,y_m=0.,yaw_rad=0.)
    assert f.episode.goal_state(dict(x_m=5.,y_m=0.,yaw_rad=0.))[0] is False
    f.lifecycle.on_status(terminal());assert f.episode.goal_state(pose)[0] is True
    with pytest.raises(SharedTerminal):f.episode.publish_command(0,0)
    assert f.episode.actor_world_poses() is None and f.episode.conservative_clearance(pose) is None
    f.lifecycle.fault('late data');f.episode.status('fault',False)
    assert f.events[-1][1]['state']=='complete'

@pytest.mark.parametrize('phase',['timeout','stopped','fault'])
def test_non_goal_terminals_never_become_goal_true(actual_episode,phase):
    f=actual_episode;feed(f,7_000_000_000);f.lifecycle.on_gate(gate());f.lifecycle.on_status(terminal(phase))
    with pytest.raises(LifecycleFault if phase=='fault' else SharedTerminal):f.episode.goal_state(f.episode.robot_world_pose())

def test_post_gate_command_values_preserved_and_pregate_rejected(actual_episode):
    f=actual_episode;feed(f,7_000_000_000)
    with pytest.raises(LifecycleFault,match='before gate'):f.episode.publish_command(.6,.12)
    assert f.publishers[0].messages[-1].linear.x==0
    f.lifecycle.on_gate(gate());f.lifecycle.active=True;f.episode.publish_command(.6,.12)
    assert (f.publishers[0].messages[-1].linear.x,f.publishers[0].messages[-1].angular.z)==(.6,.12)
    assert f.events[-1][1]['publication_source_ns']==7_000_000_000

def test_actual_legacy_observations_candidates_masks_and_commands_equal(actual_episode):
    f=actual_episode
    # Instantiate the immutable original with fake transport, then feed the
    # same valid source sequence through its exact parent callbacks.
    original=f.r2.R2BenchmarkEpisode()
    for i in range(8):
        ns=1_000_000_000+i*220_000_000;feed(f,ns)
        f.r2.r1.BenchmarkEpisode._odom(original,odom(ns))
        f.r2.r1.BenchmarkEpisode._scan(original,scan(ns))
    assert np.array_equal(f.episode.runtime.scan_history,original.runtime.scan_history)
    assert f.episode.last_runtime_scan_stamp_s==original.last_runtime_scan_stamp_s
    pose=(-4.,0.,0.)
    left=f.episode.runtime.build(pose,.8,0.,0.);right=original.runtime.build(pose,.8,0.,0.)
    def same(a,b):
        if isinstance(a,dict):return set(a)==set(b) and all(same(a[k],b[k]) for k in a)
        if isinstance(a,(tuple,list)):return len(a)==len(b) and all(same(x,y) for x,y in zip(a,b))
        if isinstance(a,np.ndarray):return np.array_equal(a,b,equal_nan=True)
        return a==b
    assert same(left,right)
    health=f.r2.r1.RuntimeInputHealth(0.,0.,0.)
    suggestions=[runtime.suggest(pose,.8,0.,0.,health) for runtime in (f.episode.runtime,original.runtime)]
    # Wall latency is measured independently, never forced equal. All learned
    # ranks, observations, previews, masks and fallback semantics must match.
    for suggestion in suggestions:suggestion.pop('latency_ms')
    assert same(*suggestions)
    for action in range(8):
        validations=[runtime.revalidate({'suggested_action':action},pose,.8,0.,0.,health) for runtime in (f.episode.runtime,original.runtime)]
        assert same(*validations)
        executed=validations[0]['validated_action']
        assert same(f.episode.runtime.commit_executed(executed),original.runtime.commit_executed(executed))
        assert same(f.episode.runtime.terminal_command(action,pose,.6,.12),original.runtime.terminal_command(action,pose,.6,.12))
    assert same(f.episode.runtime.candidate_safety_diagnostics(pose,.8,0.),original.runtime.candidate_safety_diagnostics(pose,.8,0.))

def test_actual_inherited_control_loop_streams_and_unwinds_shared_terminal(actual_episode,monkeypatch):
    f=actual_episode
    for i in range(6):feed(f,1_000_000_000+i*220_000_000)
    feed(f,7_000_000_000);f.lifecycle.on_gate(gate());f.lifecycle.active=True
    calls=[]
    class Rate:
        def __init__(self,hz):assert hz==20
        def sleep(self):
            calls.append(True);feed(f,f.clock['ns']+50_000_000)
            if len(calls)==3:
                row=terminal();row['terminal_source_ns']=f.clock['ns'];f.lifecycle.on_status(row)
    monkeypatch.setattr(f.r2.r1.rospy,'Rate',Rate)
    monkeypatch.setattr(f.r2.r1.rospy,'is_shutdown',lambda:False)
    monkeypatch.setattr(f.r2.r1.rospy,'sleep',lambda _:pytest.fail('inherited tail must not be required'))
    with pytest.raises(SharedTerminal) as ended:f.episode.execute_sensor()
    assert ended.value.row['phase']=='complete' and len(calls)==3
    traces=[row for kind,row in f.events if kind=='trace']
    assert {row['kind'] for row in traces}=={'control','decision'}
    assert all(row['legacy_row']['actor_poses'] is None for row in traces)
    assert all(row['legacy_row']['conservative_clearance_m'] is None for row in traces)
    assert f.publishers[0].messages[-1].linear.x==0

def test_health_retains_point_two_second_failure_semantics(actual_episode):
    f=actual_episode;feed(f,1_000_000_000)
    f.clock['ns']=1_200_000_001
    failures=f.episode.input_health(f.clock['ns']/1e9).failures(.20)
    assert {'scan_stale','odometry_stale'} <= set(failures)

def test_node_has_no_service_or_truth_transport_calls():
    source=NODE.read_text();tree=ast.parse(source)
    attrs=[n.func.attr for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)]
    assert not set(attrs)&{'Service','ServiceProxy','wait_for_service','execute_baseline','run'}
    literals={n.value for n in ast.walk(tree) if isinstance(n,ast.Constant) and isinstance(n.value,str)}
    forbidden={'/gazebo/model_states','/gazebo/link_states','/m2_gazebo/contacts','/astra/world_models','/astra/world_links','/astra/physics_contacts'}
    assert not literals&forbidden

@pytest.mark.parametrize('mutation',[None,'missing_map','missing_path','map_cell','path_point','path_yaw'])
def test_full_node_actual_map_path_receipt_callbacks_and_terminal_report(actual_episode,monkeypatch,mutation):
    """Execute actual main closure with real ROS messages and fake transport.

    Disable the separately exercised algorithm worker so this isolates main's
    map/path/status callback closures and pause-independent report orchestration.
    """
    from nav_msgs.msg import OccupancyGrid,Path as RosPath
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    f=actual_episode;rospy=f.r2.r1.rospy
    grid,route,data=load_bundle(SCENE/'common_bundle')
    f.params.update({'~execution_method':METHOD,'~run_id':'offline_r2_lifecycle',
        '~bundle':str(SCENE/'common_bundle'),
        '~bundle_sha256':hashlib.sha256((SCENE/'common_bundle/common_route.json').read_bytes()).hexdigest(),
        '/use_sim_time':True,'/m2_gazebo/simulation_only':True})
    callbacks={};ticks=[]
    monkeypatch.setenv('ROS_MASTER_URI','http://127.0.0.1:14820')
    monkeypatch.setattr(rospy,'init_node',lambda name:None)
    monkeypatch.setattr(rospy,'is_shutdown',lambda:len(ticks)>=3)
    monkeypatch.setattr(rospy,'Subscriber',lambda topic,kind,cb,**kw:callbacks.setdefault(topic,cb))
    class Thread:
        def __init__(self,**kw):self.target=kw['target']
        def start(self):pass
    monkeypatch.setattr(f.node.threading,'Thread',Thread)
    def step(_seconds):
        ticks.append(True);f.clock['ns']+=20_000_000
        if len(ticks)==1:
            mapping=OccupancyGrid();mapping.header.frame_id='map';mapping.info.width=grid.width;mapping.info.height=grid.height
            mapping.info.resolution=grid.resolution;mapping.info.origin.position.x,mapping.info.origin.position.y=grid.origin[:2]
            mapping.info.origin.orientation.w=1.;mapping.data=grid.blocking_occupancy.ravel().tolist()
            path=RosPath();path.header.frame_id='map'
            for index,xy in enumerate(route.points):
                pose=PoseStamped();pose.header.frame_id='map';pose.pose.position.x,pose.pose.position.y=xy
                yaw=route.goal_yaw if index==len(route.points)-1 else route.sample(route.arcs[index])[2]
                pose.pose.orientation.z=math.sin(yaw/2);pose.pose.orientation.w=math.cos(yaw/2);path.poses.append(pose)
            if mutation=='map_cell':mapping.data[0]=0
            if mutation=='path_point':path.poses[1].pose.position.y+=.01
            if mutation=='path_yaw':path.poses[1].pose.orientation.z=math.sin(.1);path.poses[1].pose.orientation.w=math.cos(.1)
            if mutation!='missing_map':callbacks['/map'](mapping)
            if mutation!='missing_path':callbacks['/astra/common_route/path'](path)
        if len(ticks)==2:
            row=terminal('stopped');row['terminal_source_ns']=f.clock['ns']
            callbacks['/astra/navigation/status'](String(data=json.dumps(row)))
    monkeypatch.setattr(f.node.time,'sleep',step)
    f.node.main()
    assert set(callbacks)=={'/odom','/scan','/map','/astra/common_route/path','/astra/navigation/gate','/astra/navigation/status'}
    pubs={p.resolved_name:p for p in f.publishers}
    receipts=pubs['/astra/frozen_r2_sensor_lifecycle/input_receipt'].messages
    assert bool(receipts)==(mutation is None)
    if receipts:
        row=json.loads(receipts[-1].data)
        assert row['points_xy']==data['points_xy'] and row['route_use']=='audit_only'
        assert row['full_polyline_consumed'] is False
    reports=[json.loads(m.data) for m in pubs['/astra/frozen_r2_sensor_lifecycle/report'].messages]
    assert len(reports)==1 and reports[0]['scientific_inclusion'] is None and reports[0]['contact_state_count'] is None
    assert (reports[0]['lifecycle_fault'] is not None)==(mutation in ('map_cell','path_point','path_yaw'))
    assert Path(f.params['~report_path']).exists()
