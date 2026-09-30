"""New navigation lifecycle and failure-inclusive assessment boundaries, no ROS."""
import copy
import json
import importlib.util
from pathlib import Path
import sys
import types

import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import navigation_evidence_p04 as n


def contract(method='astra_analytic_route_v1'):
    scene=n.STAGE/'scenes/navigation_n01_clear_r03'
    names=['/rosout','/gazebo','/fixture_bag','/astra_navigation_collector',
      '/m2_robot_state_publisher','/navigation_command_relay','/astra_common_route_provider','/astra_navigation_coordinator']
    names.append('/move_base' if method in ('actual_teb_common_route','dwa_common_route') else
      '/astra_frozen_r2_method' if method=='frozen_r2_v060_shared_activation' else '/astra_route_runtime')
    return n.make_contract(stage='astra_navigation_n01',run_id='astra_navigation_n01_'+method,seed=260926026,method=method,
      scene_id='navigation_n01_clear',world=scene/'world.sdf',robot_description=n.STAGE/'audits/navigation_evidence_samples/robot_description_n01_kd6.xml',
      scene_manifest=scene/'scene_manifest.yaml',shared_bundle=scene/'common_bundle',spawn_world_xyz_yaw=[-5.4,0.,0.,0.],
      steering_kd=6.,allowed_nodes=names,required_nodes=names)


def coordinator(value,**changes):
    result={'schema':'astra_common_navigation_v1',**{k:value[k] for k in ('run_id','seed','method')}}
    result.update(changes);return result


def terminal(value,phase='complete',stamp=8_000_000_000,gate=7_000_000_000):
    return coordinator(value,phase=phase,terminal_source_ns=stamp,gate_source_ns=gate,terminal_reason=phase,
      terminal_time_authority=n.TERMINAL_AUTHORITIES[phase])


@pytest.mark.parametrize('method',n.METHODS)
def test_actual_world_map_urdf_contract_all_method_identities(method):
    value=contract(method)
    assert value['execution_authorized']is False
    assert value['steering_kd']==6. and value['camera_topic'] not in value['record_topics']
    assert '/astra/method_cmd' not in value['required_before_unpause']
    assert '/astra/method_cmd' in value['required_before_arm']
    assert n.validate_robot_description(n.read_bound(value['robot_description']).decode(),value)['collision_components']==9


@pytest.mark.parametrize('key,bad',[('seed',True),('seed',260926026.),('attempt',True),('activation_source_ns',4_000_000_000),
 ('policy_cap_ns',23_000_000_000),('execution_authorized',True),('training_authorized',True),('steering_kd',5.),
 ('wall_cap_s',float('nan')),('drain_wall_s',6.),('spawn_world_xyz_yaw',[-5.4,0.,.02,0.]),('extra',1)])
def test_closed_contract_rejects_changed_identity_physics_timing_or_authority(key,bad):
    value=contract();value[key]=bad
    with pytest.raises(ValueError):n.validate_contract(value)


@pytest.mark.parametrize('mutation',['bad_model','no_core_topic','missing_node','camera_in_bag','urdf_hash','unhashable_node','wrong_bundle_spawn','wrong_scene'])
def test_actual_binding_roster_and_recording_contract_fail_closed(mutation,tmp_path):
    value=contract()
    if mutation=='bad_model':value['expected_models'].append('phantom')
    elif mutation=='no_core_topic':value['required_before_unpause'].remove('/astra/p03/end_links')
    elif mutation=='missing_node':value['required_nodes'].remove('/astra_navigation_collector')
    elif mutation=='camera_in_bag':value['record_topics'].append(value['camera_topic'])
    elif mutation=='urdf_hash':value['robot_description']['sha256']='0'*64
    elif mutation=='unhashable_node':value['allowed_nodes'].append([])
    elif mutation=='wrong_bundle_spawn':value['spawn_world_xyz_yaw'][0]-=1
    else:value['scene_id']='other_scene'
    with pytest.raises(ValueError):n.validate_contract(value)


def test_gate_records_actual_header_not_local_receipt():
    value=contract();gate=coordinator(value,source_ns=7_020_000_000,deadline_source_ns=29_020_000_000,
      deadline_ns=29_020_000_000,source='actual_odom_header_first_crossing',pose_map=[-3.99,0.,0.],body_twist=[.8,0.,0.],
      goal_pose=[5.,0.,0.],activation_source_ns=4_998_000_000,publisher_visible_clock_ns=7_018_000_000)
    assert n.gate_record(gate,value)['source_ns']==7_020_000_000
    for key,bad in [('source','clock_receipt'),('source_ns',7_020_000_001),('deadline_ns',29_040_000_000),('body_twist',[float('nan'),0.,0.]),('seed',True)]:
        changed=copy.deepcopy(gate);changed[key]=bad
        with pytest.raises(ValueError):n.gate_record(changed,value)


def test_terminal_timeout_uses_declared_deadline_and_fault_preserves_authority():
    value=contract()
    row=terminal(value,'timeout',29_000_000_000)
    assert n.terminal_record(row,value)==row
    row['terminal_source_ns']+=n.STEP_NS
    with pytest.raises(ValueError):n.terminal_record(row,value)
    row=terminal(value,'fault',0,gate=None)
    assert n.terminal_record(row,value)['terminal_source_ns']==0
    row['terminal_time_authority']='fault_generation_time'
    with pytest.raises(ValueError):n.terminal_record(row,value)


class Clock:
    def __init__(self):self.now=0.
    def __call__(self):return self.now
    def sleep(self,seconds):self.now+=seconds


def test_readiness_late_success_cannot_extend_hard_wall_deadline():
    clock=Clock()
    def slow():clock.now=1.1;return True
    with pytest.raises(TimeoutError):n.wait_condition(slow,1.,'late',clock=clock,sleep=clock.sleep)


def test_transient_readiness_error_retries_but_permanent_failure_does_not():
    clock=Clock();count=[0]
    def transient():
        count[0]+=1
        if count[0]==1:raise ConnectionError('not advertised yet')
        return 'ready'
    assert n.wait_condition(transient,1.,'transient',clock=clock,sleep=clock.sleep)=='ready'
    def fatal():raise n.LifecycleFault('source activation missed')
    with pytest.raises(n.LifecycleFault):n.wait_condition(fatal,90.,'permanent',clock=clock,sleep=clock.sleep)
    assert clock.now==.02


@pytest.mark.parametrize('source,status',[(5_000_000_000,{'ready_to_arm':True}),
 (2_000_000_000,{'ready_to_arm':True,'phase':'fault'}),(6_000_000_000,None)])
def test_no_wait_past_fixed_activation_or_terminal_fault(source,status):
    with pytest.raises(n.LifecycleFault):n.readiness_guard(source,status,contract())


def test_lazy_method_topics_wait_until_positive_clock_phase():
    value=contract()
    early=[[i,'/publisher','i','TCPROS',topic,True] for i,topic in enumerate(value['required_before_unpause'])]
    assert n.recorder_connections(early,value['required_before_unpause'])['pass']
    assert not n.recorder_connections(early,value['required_before_arm'])['pass']
    assert not n.readiness_guard(2_000_000,None,value)


def samples(start,end,x=5.,yaw=0.):
    import math
    return [(stamp,(x,0.,0.,0.,0.,math.sin(yaw/2),math.cos(yaw/2))) for stamp in range(start,end+1,n.STEP_NS)]


def test_actual_base_arrival_and_timeout_failure_inclusive_are_separate():
    value=contract();start=7_000_000_000;end=start+4_000_000
    rows=samples(start,end,x=4.7);rows[-1]=samples(end,end,x=4.75,yaw=.25)[0]
    result=n.navigation_outcome(rows,[5.,0.,0.],start,end,terminal(value,stamp=end),value)
    assert result['raw_geometric_arrival_source_ns']==end and result['goal_geometry_contract_pass']
    assert result['raw_geometric_arrival_time_s']==.004
    end=start+22_000_000_000
    result=n.navigation_outcome(samples(start,end,x=4.7),[5.,0.,0.],start,end,terminal(value,'timeout',end),value)
    assert result['task_phase']=='timeout' and result['goal_geometry_contract_pass']
    assert result['raw_geometric_arrival_source_ns']is None and result['failure_restricted_time_s']==22.


def test_missing_or_false_completion_and_wrong_orientation_not_valid_goal():
    value=contract();start=7_000_000_000;end=start+4_000_000
    rows=samples(start,end,yaw=.251)
    result=n.navigation_outcome(rows,[5.,0.,0.],start,end,terminal(value,stamp=end),value)
    assert result['raw_geometric_arrival_source_ns']is None and not result['goal_geometry_contract_pass']
    rows=samples(start,end);del rows[1]
    assert not n.navigation_outcome(rows,[5.,0.,0.],start,end,terminal(value,stamp=end),value)['goal_geometry_contract_pass']
    rows=samples(start,end);rows.insert(1,rows[0])
    with pytest.raises(ValueError):n.navigation_outcome(rows,[5.,0.,0.],start,end,terminal(value,stamp=end),value)


def test_fixed_navigation_service_gateway_forbids_pose_writes_and_duplicate_stop():
    start,end=7_000_000_000,8_000_000_000
    rows=[dict(name=name,phase=phase,source_ns=stamp,completed=True,error=None) for name,phase,stamp in (
      ('/gazebo/spawn_urdf_model','bootstrap',0),('/gazebo/unpause_physics','bootstrap',0),
      ('/astra/navigation/arm','warmup',2_000_000_000),('/astra/navigation/stop','terminal',end),('/gazebo/pause_physics','terminal',end))]
    assert n.validate_services(rows,start,end)['pass']
    for bad in (dict(rows[0],name='/gazebo/set_model_state'),dict(rows[-2]),dict(rows[-1],completed=False)):
        assert not n.validate_services(rows+[bad],start,end)['pass']


@pytest.mark.parametrize('method,local',[('actual_teb_common_route','teb_local_planner/TebLocalPlannerROS'),
 ('dwa_common_route','dwa_local_planner/DWAPlannerROS')])
def test_actual_baseline_configured_classes_exact(method,local):
    params={'base_local_planner':local,'base_global_planner':'astra_common_route/FixedRouteGlobalPlanner'}
    assert n.planner_parameters(params,method)==params
    for changes in ({'base_global_planner':'navfn/NavfnROS'},{'base_local_planner':'other'}, {'extra':True}):
        with pytest.raises(ValueError):n.planner_parameters(dict(params,**changes),method)
    assert n.planner_parameters({},'astra_analytic_route_v1')=={}


def test_real_collector_main_bad_runtime_backend_fails_before_spawn_with_durable_null_window(tmp_path,monkeypatch):
    """Execute actual node main against fake ROS; no live services or sockets."""
    value=contract();path=tmp_path/'contract.json';path.write_text(json.dumps(value))
    root=tmp_path/'runs'/value['run_id'];root.parent.mkdir()
    urdf=n.read_bound(value['robot_description']).decode().replace('libastra_physical_ackermann_p05_plugin.so','libastra_physical_ackermann_p02_plugin.so')
    params={'~output':str(root),'~contract_path':str(path),'~contract_sha256':n.sha(path),
      '/use_sim_time':True,'/m2_gazebo/simulation_only':True,'/robot_description':urdf}
    calls=[]
    rospy=types.ModuleType('rospy');rospy.init_node=lambda *a,**k:None
    rospy.get_param=lambda name,default=None:params.get(name,default)
    rospy.get_name=lambda:'/astra_navigation_collector'
    rospy.Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_nsec=lambda:0))
    rospy.Subscriber=lambda *a,**k:object()
    rospy.is_shutdown=lambda:False
    def service(name,kind):
        def invoke(*args):calls.append(name);return types.SimpleNamespace(success=True,message='fake cleanup success')
        return invoke
    rospy.ServiceProxy=service
    monkeypatch.setitem(sys.modules,'rospy',rospy)
    message_names={'gazebo_msgs.msg':['WorldState','ContactsState'],'gazebo_msgs.srv':['SpawnModel','GetWorldProperties','GetPhysicsProperties'],
      'geometry_msgs.msg':['Pose'],'diagnostic_msgs.msg':['DiagnosticArray'],'sensor_msgs.msg':['Image'],
      'std_msgs.msg':['String'],'std_srvs.srv':['Empty','Trigger']}
    for module,names in message_names.items():
        package=module.split('.')[0]
        if package not in sys.modules:monkeypatch.setitem(sys.modules,package,types.ModuleType(package))
        fake=types.ModuleType(module)
        for name in names:setattr(fake,name,type(name,(),{}))
        monkeypatch.setitem(sys.modules,module,fake)
    monkeypatch.setenv('ROS_MASTER_URI','http://127.0.0.1:14820')
    monkeypatch.setattr(n,'STAGE',tmp_path)
    entry=n.WORKSPACE/'src/simulation/m2_gazebo/scripts/astra_navigation_collector_p04.py'
    spec=importlib.util.spec_from_file_location('navigation_collector_under_test',entry)
    node=importlib.util.module_from_spec(spec);spec.loader.exec_module(node)
    assert node.main()==3
    assert calls==['/astra/navigation/stop','/gazebo/pause_physics']
    result=json.loads((root/'collector_report.json').read_text())
    assert result['pass']is False and result['source_window_ns']==[None,None]
    assert result['descriptive_overall_sim_seconds_per_wall_second']is None
    assert not (root/'ready_for_maps.json').exists()
    raw=json.loads((root/'collector_raw.json').read_text())
    assert 'P05' in raw['errors'][0]
