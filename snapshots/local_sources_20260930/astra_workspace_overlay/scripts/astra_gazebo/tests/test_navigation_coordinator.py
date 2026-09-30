import math
from pathlib import Path
import sys
import pytest
SCRIPTS=Path(__file__).resolve().parents[1];sys.path.insert(0,str(SCRIPTS))
from navigation_coordinator import CoordinatorConfig,NavigationCoordinator,SCHEMA


def ready():
    c=NavigationCoordinator(CoordinatorConfig('offline_fixture',42,'astra_analytic_route_v1'))
    c.observe_odom(1_000_000_000,(-5.4,0,0),(0,0,0))
    c.method_status({'run_id':'offline_fixture','seed':42,'method':'astra_analytic_route_v1','state':'ready','ready':True})
    c.arm(1_000_000_000);return c


def gated():
    c=ready();c.observe_odom(5_000_000_000,(-5.4,0,0),(0,0,0));assert c.output(5_000_000_000)['v']==.8
    c.observe_odom(7_000_000_000,(-4.01,0,0),(.8,0,0));gate=c.observe_odom(7_020_000_000,(-3.994,0,0),(.8,0,0))
    assert gate['source_ns']==7_020_000_000;return c


def acknowledge(c):c.method_status(dict(run_id='offline_fixture',seed=42,method='astra_analytic_route_v1',state='active',ready=True,gate_source_ns=c.gate['source_ns']))


def test_bootstrap_exact5s_requires_arm_and_actual_source_epoch():
    c=ready();c.observe_odom(4_980_000_000,(-5.4,0,0),(0,0,0));assert c.output(5_000_000_000)['v']==0
    c.observe_odom(5_000_000_000,(-5.4,0,0),(0,0,0));assert c.output(5_002_000_000)['v']==.8
    assert c.activation_source_ns==5_000_000_000
    other=NavigationCoordinator(c.config);assert other.output(5_000_000_000)['phase']=='fault'


def test_gate_is_first_actual_odom_crossing_not_20hz_control_poll():
    c=gated();later=c.output(7_050_000_000)
    assert later['gate_source_ns']==7_020_000_000 and later['phase']=='activating' and later['v']==0
    assert c.observe_odom(7_060_000_000,(-3.962,0,0),(.8,0,0))is None
    assert c.gate['source_ns']==7_020_000_000


def test_queued_pre_ack_commands_are_not_executed_and_receipt_not_source():
    c=gated();c.method_command(.9,.1,7_020_000_000,100);acknowledge(c)
    assert c.output(7_020_000_000)['v']==0
    c.method_command(.6,.04,7_040_000_000,200);row=c.output(7_040_000_000)
    assert (row['v'],row['w'])==(.6,.04)
    assert row['selected_method_receipt']['generation_source_ns']is None
    assert row['method_command_generation_stamp_known']is False


def test_moving_method_before_gate_faults_even_during_common_warmup():
    c=ready();c.method_command(.01,0,1_100_000_000,100)
    assert c.output(1_100_000_000)['phase']=='fault'


@pytest.mark.parametrize('change',[(.031,0,0,.8),(.001,.031,0,.8),(.001,0,.031,.8),(.001,0,0,.759)])
def test_bad_crossing_state_cannot_be_delayed_until_a_better_outcome(change):
    c=ready();c.observe_odom(5_000_000_000,(-5.4,0,0),(0,0,0));c.output(5_000_000_000)
    x,y,yaw,v=change;c.observe_odom(7_000_000_000,(-4+x,y,yaw),(v,0,0));assert c.phase=='fault'
    assert c.gate is None


def test_common_goal_and22s_timeout_override_late_method_commands():
    c=gated();acknowledge(c);c.observe_odom(29_020_000_000,(0,0,0),(.8,0,0))
    c.method_command(1.2,0,29_020_000_000,100)
    assert c.output(29_020_000_000)['phase']=='timeout' and c.output(29_020_000_000)['v']==0
    c=gated();acknowledge(c);c.observe_odom(8_000_000_000,(4.8,.1,.2),(.8,0,0));assert c.phase=='complete'


def test_clock_state_replay_and_wrong_ack_fail_closed():
    c=gated();c.observe_odom(7_020_000_000,(-3.99,0,0),(.8,0,0));assert c.phase=='fault'
    c=gated();c.output(7_020_000_000);assert c.output(7_019_999_999)['phase']=='fault'
    c=gated();c.method_status(dict(run_id='offline_fixture',seed=42,method='astra_analytic_route_v1',state='active',ready=True,gate_source_ns=1));assert c.phase=='fault'


def test_stale_method_receipt_zero_and_ack_timeout_are_separate():
    c=gated();acknowledge(c);c.method_command(.8,0,7_030_000_000,100)
    c.observe_odom(7_180_000_001,(-3.9,0,0),(.8,0,0));row=c.output(7_180_000_001)
    assert row['v']==0 and row['reason']=='stale_or_future_method_receipt_zero'
    c=gated();c.observe_odom(8_020_000_001,(-3.9,0,0),(.1,0,0));assert c.output(8_020_000_001)['reason']=='method_activation_ack_timeout'


def test_r2_route_limit_is_explicit_instead_of_relabelled_fullroute():
    with pytest.raises(ValueError,match='legacy x route'):
        CoordinatorConfig('fixture',42,'frozen_r2_v060_shared_activation',goal_pose=(5,1,0))


@pytest.mark.parametrize('method',['astra_analytic_route_v1','actual_teb_common_route','dwa_common_route'])
@pytest.mark.parametrize('source_lead_ns',[0,2_000_000,200_000_001])
@pytest.mark.parametrize('late_fault_and_pause',[False,True])
def test_coordinator_ros_callbacks_survive_terminal_and_have_single_authority(monkeypatch,method,source_lead_ns,late_fault_and_pause):
    import importlib.util,json,types,io,hashlib
    monkeypatch.syspath_prepend('/opt/ros/noetic/lib/python3/dist-packages')
    import genpy
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
    from std_msgs.msg import String
    stage=SCRIPTS.parents[2];directory=stage/'scenes/navigation_n01_clear_r03/common_bundle'
    params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'~run_id':'offline_fixture','~seed':42,'~method':method,'~bundle':str(directory),'~bundle_sha256':hashlib.sha256((directory/'common_route.json').read_bytes()).hexdigest()}
    baseline=method!='astra_analytic_route_v1';goals=[];cancelled=[]
    publishers={};subscribers={};services={};clock={'ns':0,'ticks':0,'gate_tick':None};arm_results=[]
    fake=types.ModuleType('rospy')
    def timestamp():return genpy.Time(clock['ns']//10**9,clock['ns']%10**9)
    def status(state,**extra):return String(data=json.dumps(dict(run_id='offline_fixture',seed=42,method=method,state=state,ready=True,**extra)))
    class Publisher:
        def __init__(self,topic,kind,**kwargs):self.messages=[];self.resolved_name=topic;publishers[topic]=self
        def get_num_connections(self):return 2
        def publish(self,message):
            stream=io.BytesIO();message.serialize(stream);assert stream.getvalue();self.messages.append(message)
            if self.resolved_name=='/astra/navigation/gate':
                gate_payload=json.loads(message.data);clock['gate_tick']=clock['ticks'];clock['gate_ns']=gate_payload['source_ns']
                if not baseline:subscribers['/astra/method/status'](status('active',gate_source_ns=clock['gate_ns']))
    def odom_message(world_x,velocity):
        msg=Odometry();msg.header.frame_id='odom';msg.child_frame_id='base_link';msg.header.stamp=timestamp()+genpy.Duration(0,source_lead_ns)
        msg.pose.pose.position.x=world_x+5.4;msg.pose.pose.orientation.w=1.;msg.twist.twist.linear.x=velocity;return msg
    def advance(_):
        clock['ticks']+=1
        if late_fault_and_pause and clock['gate_tick']is not None and clock['ticks']>=clock['gate_tick']+9:return
        clock['ns']=4_800_000_000+(clock['ticks']-1)*20_000_000
        age=max(0.,(clock['ns']-5_000_000_000)/1e9);position=-5.4+.8*age;speed=0. if age==0 else .8
        if clock['gate_tick']is not None and clock['ticks']>=clock['gate_tick']+6:position=4.8;speed=.4
        subscribers['/odom'](odom_message(position,speed))
        if clock['ticks']==1:subscribers['/astra/method/status'](status('ready'))
        if clock['ticks']==(2 if source_lead_ns==2_000_000 else 1):arm_results.append(services['/astra/navigation/arm'](None))
        if clock['gate_tick']is not None:
            command=Twist();command.linear.x=.6;subscribers['/astra/method_cmd'](command)
            if late_fault_and_pause and clock['ticks']==clock['gate_tick']+8:subscribers['/astra/method/status'](status('fault',reason='late injected method fault'))
    fake.init_node=lambda _:None;fake.get_name=lambda:'/astra_navigation_coordinator';fake.get_param=lambda n,d=None:params.get(n,d)
    fake.Publisher=Publisher;fake.Subscriber=lambda topic,kind,cb,**kwargs:subscribers.setdefault(topic,cb);fake.Service=lambda name,kind,cb:services.setdefault(name,cb)
    fake.Time=types.SimpleNamespace(now=timestamp);fake.is_shutdown=lambda:(source_lead_ns>200_000_000 and clock['ticks']>=3) or (clock['gate_tick']is not None and clock['ticks']>=clock['gate_tick']+12)
    graph=types.ModuleType('rosgraph');graph.Master=lambda _:types.SimpleNamespace(getSystemState=lambda:([('/cmd_vel_navigation',['/astra_navigation_coordinator']),('/astra/method_cmd',['/move_base' if baseline else '/astra_route_runtime'])],[],[]))
    actionlib=types.ModuleType('actionlib')
    class ActionClient:
        def __init__(self,name,kind):assert name=='/move_base'
        def wait_for_server(self):return True
        def send_goal(self,goal,active_cb):
            assert clock['gate_tick']is not None and clock['ns']>=clock['gate_ns']
            assert goal.target_pose.header.frame_id=='map' and goal.target_pose.pose.position.x==5.
            goals.append(goal);active_cb()
        def cancel_all_goals(self):cancelled.append(clock['ns'])
    actionlib.SimpleActionClient=ActionClient
    monkeypatch.setitem(sys.modules,'rospy',fake);monkeypatch.setitem(sys.modules,'rosgraph',graph);monkeypatch.setitem(sys.modules,'actionlib',actionlib)
    monkeypatch.setenv('ROS_MASTER_URI','http://127.0.0.1:14820');monkeypatch.setenv('THESIS_WS',str(SCRIPTS.parents[1]))
    path=SCRIPTS.parents[1]/'src/simulation/m2_gazebo/scripts/astra_navigation_coordinator.py';spec=importlib.util.spec_from_file_location('coordinator_transport_fixture',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    class ImmediateThread:
        def __init__(self,target,daemon):self.target=target
        def start(self):self.target()
    monkeypatch.setattr(module.threading,'Thread',ImmediateThread)
    monkeypatch.setattr(module.time,'sleep',advance);monkeypatch.setattr(module.time,'monotonic',lambda:100+clock['ticks']*.01+(6 if late_fault_and_pause and clock['gate_tick']is not None and clock['ticks']>=clock['gate_tick']+10 else 0));monkeypatch.setattr(module.time,'monotonic_ns',lambda:int((100+clock['ticks']*.01)*1e9))
    module.main()
    if source_lead_ns>200_000_000:
        assert [x.success for x in arm_results]==[False] and not publishers['/astra/navigation/gate'].messages
        traces=[json.loads(x.data) for x in publishers['/astra/navigation/command_trace'].messages]
        assert all(row['v']==0 for row in traces)
        assert any('future transport bound'in str(row['reason']) for row in traces)
        return
    assert [x.success for x in arm_results]==[True]
    assert len(publishers['/astra/navigation/gate'].messages)==1
    traces=[json.loads(x.data) for x in publishers['/astra/navigation/command_trace'].messages]
    assert any(x['v']==.8 and x['phase']=='warmup' for x in traces)
    assert any(x['v']==.6 and x['phase']=='active' for x in traces)
    terminal=next(x for x in traces if x['phase']=='complete');assert terminal['terminal_time_authority']=='first_actual_odom_header_meeting_base_goal'
    assert all(row['phase']=='complete' for row in traces[traces.index(terminal):])
    if late_fault_and_pause:
        latest_status=json.loads(publishers['/astra/navigation/status'].messages[-1].data)
        assert latest_status['phase']=='complete' and latest_status['late_fault_count']>=2
        assert any(row['reason']=='publisher_clock_wall_stall' for row in latest_status['late_faults'])
    if source_lead_ns:
        assert all(row['actual_odom_source_ns']is None or row['actual_odom_source_ns']<=row['publisher_visible_clock_ns'] for row in traces)
    assert all(x['v']==0 for x in traces if x['publisher_visible_clock_ns']>=terminal['terminal_source_ns'])
    assert set(publishers)&{'/cmd_vel_navigation','/cmd_vel','/astra/method_cmd'}=={'/cmd_vel_navigation'}
    assert len(goals)==int(baseline)
    if baseline:assert cancelled
    # Delayed callbacks remain callable after finally; no response variable may
    # shadow any callback function in this scope (P06 terminal-race regression).
    clock['ns']+=20_000_000;subscribers['/odom'](odom_message(4.81,0))
    subscribers['/astra/method/status'](status('active',gate_source_ns=clock['gate_ns']))
    subscribers['/astra/method_cmd'](Twist())
    assert not any('TypeError' in str(x) for x in traces)


def test_launch_lifecycle_and_geometry_configuration_are_explicit():
    import xml.etree.ElementTree as ET,json,yaml
    from shapely.geometry import Polygon
    folder=SCRIPTS.parents[1]/'src/simulation/m2_gazebo'
    root=ET.parse(folder/'launch/astra_common_navigation_smoke.launch').getroot()
    assert not any('spawn_model' in n.get('type','') for n in root.iter('node'))
    assert not any('spawn_m2' in n.get('file','') for n in root.iter('include'))
    assert any(n.get('type')=='$(arg collector_script)' for n in root.iter('node'))
    config=yaml.safe_load((folder/'config/astra_navigation_common_footprint.yaml').read_bytes())
    raw=Polygon(config['TebLocalPlannerROS']['footprint_model']['vertices'])
    for key in ('global_costmap','local_costmap'):
        assert config[key]['footprint_padding']==0
        assert Polygon(config[key]['footprint']).covers(raw.buffer(.33,resolution=4096))
    assert config['TebLocalPlannerROS']['min_obstacle_dist']==.33


def test_frozen_r2_wrapper_only_overrides_execution_readiness_and_warmup():
    import ast
    path=SCRIPTS.parents[1]/'src/simulation/m2_gazebo/scripts/astra_frozen_r2_coordinated.py'
    tree=ast.parse(path.read_text());klass=next(n for n in ast.walk(tree) if isinstance(n,ast.ClassDef) and n.name=='SharedExecutionR2')
    assert {n.name for n in klass.body if isinstance(n,ast.FunctionDef)}=={'wait_ready','warmup'}
    assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='publish_command' for method in klass.body for n in ast.walk(method))
    assert 'ServiceProxy' not in path.read_text() and 'unpause()' not in path.read_text()


def test_frozen_r2_gate_wrapper_transport_and_raster_identity(monkeypatch):
    import importlib.util,json,types,io,yaml
    monkeypatch.syspath_prepend('/opt/ros/noetic/lib/python3/dist-packages')
    import genpy
    from std_msgs.msg import String
    stage=SCRIPTS.parents[2];manifest=stage/'scenes/navigation_n01_clear_r03/scene_manifest.yaml'
    params={'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'~run_id':'fixture_r2','~seed':42,'~method':'pct_scope_rl','~candidate_variant':'r2_candidate_v060'}
    publishers={};subscribers={};calls=[];fake=types.ModuleType('rospy')
    gate={'schema':'astra_common_navigation_v1','run_id':'fixture_r2','seed':42,'method':'frozen_r2_v060_shared_activation','source_ns':7_000_000_000,
          'activation_source_ns':5_000_000_000,'pose_map':[-3.994,0,0],'body_twist':[.8,0,0]}
    class Publisher:
        def __init__(self,topic,kind,**kwargs):self.topic=topic;self.messages=[];publishers[topic]=self
        def publish(self,message):
            stream=io.BytesIO();message.serialize(stream);self.messages.append(message)
            if self.topic=='/astra/method/status' and json.loads(message.data)['state']=='ready':subscribers['/astra/navigation/gate'](String(data=json.dumps(gate)))
    fake.init_node=lambda _:None;fake.get_param=lambda key,default=None:params.get(key,default);fake.Time=types.SimpleNamespace(now=lambda:genpy.Time(7,0))
    fake.Publisher=Publisher;fake.Subscriber=lambda topic,kind,callback,**kwargs:subscribers.setdefault(topic,callback)
    fake.is_shutdown=lambda:False;fake.spin=lambda:calls.append('spin')
    monkeypatch.setitem(sys.modules,'rospy',fake);monkeypatch.setenv('ROS_MASTER_URI','http://127.0.0.1:14820');monkeypatch.setenv('THESIS_WS',str(SCRIPTS.parents[1]))
    class FrozenAlgorithmFixture:
        def __init__(self):
            self.odom=self.scan=self.model_states=object();self.ego_model='m2';self.model_index={'m2':0};self.actor_models=[]
            self.odom_received_s=self.scan_received_s=7.;self.scan_stamps=list(range(6))
            self.spawn_world_x=-5.4;self.policy_start_world_x=-4.;self.goal_world_x=5.;self.actor_checkpoint_sha256='fixture_only'
            self.scene_manifest_path=str(manifest);self.scene=yaml.safe_load(manifest.read_bytes())
        def execute_sensor(self):
            assert json.loads(publishers['/astra/method/status'].messages[-1].data)['state']=='active'
            calls.append('unchanged_execute_sensor');return {'goal_reached':True}
        def run(self):
            self.wait_ready();warmup=self.warmup();assert warmup['shared_actual_odom_gate_source_ns']==gate['source_ns']
            return {'result':self.execute_sensor()}
        def publish_command(self,*args):raise AssertionError('wrapper must not move before gate')
    path=SCRIPTS.parents[1]/'src/simulation/m2_gazebo/scripts/astra_frozen_r2_coordinated.py'
    spec=importlib.util.spec_from_file_location('frozen_r2_transport_fixture',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    real_r2=SCRIPTS.parents[1]/'src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_r2_episode.py'
    monkeypatch.setattr(module,'load_frozen_class',lambda workspace:(FrozenAlgorithmFixture,real_r2));module.main()
    assert calls==['unchanged_execute_sensor','spin']
    identity=json.loads(publishers['/astra/frozen_r2_execution_identity'].messages[0].data)
    assert identity['static_prior_box_count']==43 and identity['shared_route_consumed']is False
    assert identity['static_prior_consumer_semantics_equal']is False
    assert json.loads(publishers['/astra/method/status'].messages[-1].data)['internal_report_goal_reached']is True
    subscribers['/astra/navigation/gate'](String(data=json.dumps(gate)))


@pytest.mark.parametrize('terminal_phase',['complete','timeout','stopped','fault'])
def test_first_terminal_is_immutable_with_late_fault_diagnostics(terminal_phase):
    c=gated();acknowledge(c)
    if terminal_phase=='complete':c.observe_odom(8_000_000_000,(4.8,0,0),(.8,0,0))
    elif terminal_phase=='timeout':c.observe_odom(29_020_000_000,(0,0,0),(.8,0,0))
    elif terminal_phase=='stopped':c.request_stop()
    else:c.fault('first active fault')
    original=(c.phase,c.terminal_source_ns,c.terminal_reason,c.terminal_time_authority,c.fault_reason)
    c.fault('late method failure');c.fault('publisher_clock_wall_stall');c.request_stop()
    assert (c.phase,c.terminal_source_ns,c.terminal_reason,c.terminal_time_authority,c.fault_reason)==original
    assert c.late_fault_count==2 and len(c.late_faults)==2
    assert c.output(29_040_000_000)['v']==0


@pytest.mark.parametrize('clock_first',[False,True])
def test_deadline_clock_and_earlier_goal_odom_order_have_same_result(clock_first):
    c=gated();acknowledge(c);c.observe_odom(28_980_000_000,(4.7,0,0),(.8,0,0))
    if clock_first:
        row=c.output(29_020_000_000)
        assert row['phase']=='active' and row['v']==0 and row['reason']=='deadline_zero_waiting_ordered_actual_odometry'
    c.observe_odom(29_000_000_000,(4.8,0,0),(.8,0,0))
    if not clock_first:c.output(29_020_000_000)
    assert c.phase=='complete' and c.terminal_source_ns==29_000_000_000


def test_deadline_requires_ordered_actual_odom_and_stale_is_still_failure():
    c=gated();acknowledge(c);c.observe_odom(29_000_000_000,(4.7,0,0),(.8,0,0))
    assert c.output(29_020_000_000)['phase']=='active'
    c.observe_odom(29_020_000_000,(4.7,0,0),(.8,0,0))
    assert c.phase=='timeout' and c.terminal_source_ns==29_020_000_000
    c=gated();acknowledge(c);c.observe_odom(29_000_000_000,(4.7,0,0),(.8,0,0))
    assert c.output(29_200_000_001)['phase']=='fault'
    assert c.fault_reason=='actual_odometry_stale_or_future'


def test_active_fault_immediately_revokes_method_output():
    c=gated();acknowledge(c);c.method_command(.8,0,7_030_000_000,100)
    assert c.output(7_030_000_000)['v']==.8
    c.fault('active sensor failure');row=c.output(7_030_000_000)
    assert row['phase']=='fault' and row['v']==row['w']==0
