"""Offline Q02 contract, failure and command-node integration fixtures; no ROS."""
import copy
import importlib.util
import json
import math
from pathlib import Path
import sys
import types
import xml.etree.ElementTree as ET
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import passive_sweep_q02 as q
import run_passive_sweep_q02 as runner

def value(actor=False):return q.contract(q.FIXTURES[int(actor)][0],4.,True)

def commands():
    schedule=q.CommandSchedule(value());start=schedule.start(1_000_000_000)
    events=[]
    for slot in range(200):
        event=schedule.poll(start+slot*q.PERIOD_NS)
        events.append(dict(event,run_id=value()['run_id']))
    return schedule,start,events

def test_exact_schedule_200_slots_and_terminal_zero():
    schedule,start,events=commands()
    assert q.command_audit(events,value(),start)['pass']
    assert [events[i]['phase'] for i in (0,19,20,59,60,99,100,139,140,159,160,199)]==[
       'settle','settle','straight','straight','left','left','right','right','straight_exit','straight_exit','stop','stop']
    assert events[60]['w']==pytest.approx(.3*math.tan(.1)/.65)
    assert events[100]['w']==-events[60]['w']
    assert schedule.poll(start+q.DURATION_NS)['v']==0.
    assert schedule.state=='complete'
    assert schedule.poll(start+q.DURATION_NS+1) is None

@pytest.mark.parametrize('bad',[True,1.,-1,2**63,None])
def test_integer_source_contract(bad):
    with pytest.raises(ValueError):q.integer_ns(bad)

@pytest.mark.parametrize('key,new',[('seed',260926015.),('attempt',True),('actor',0),('camera',1),
  ('duration_ns',10_000_000_000.),('retry_allowed',True),('extra',False)])
def test_closed_contract_rejects_coercions_and_changes(key,new):
    data=value();data[key]=new
    with pytest.raises(ValueError):q.validate_contract(data)

def test_unresolved_kd_has_no_execution_readiness():
    data=q.contract(q.FIXTURES[0][0])
    assert q.validate_contract(data,False)==data
    with pytest.raises(ValueError):q.validate_contract(data)
    for bad in (True,0,4.1,float('nan')):
        with pytest.raises(ValueError):q.contract(q.FIXTURES[0][0],bad)

def test_start_one_shot_positive_and_aligned():
    obj=q.CommandSchedule(value())
    with pytest.raises(ValueError):obj.start(0)
    start=obj.start(1_002_000_000)
    assert start==1_120_000_000 and start%20_000_000==0
    with pytest.raises(ValueError):obj.start(2_000_000_000)
    obj=q.CommandSchedule(value());obj.state='fault'
    with pytest.raises(ValueError):obj.start(1)

@pytest.mark.parametrize('offset',[20_000_001,50_000_000,500_000_000])
def test_no_late_or_missed_replay(offset):
    obj=q.CommandSchedule(value());start=obj.start(1)
    with pytest.raises(ValueError):obj.poll(start+offset)
    assert obj.state=='fault'
    assert obj.poll(start+offset+1) is None

def test_clock_reset_fault_and_no_reactivation():
    obj=q.CommandSchedule(value());start=obj.start(1000);obj.poll(start)
    with pytest.raises(ValueError):obj.poll(start-1)
    assert obj.state=='fault'
    with pytest.raises(ValueError):obj.start(start+1)

@pytest.mark.parametrize('offset',[20_000_001,50_000_000,-1])
def test_timestamp_rechecked_immediately_before_publish(offset):
    obj=q.CommandSchedule(value());start=obj.start(1);event=obj.poll(start)
    with pytest.raises(ValueError):obj.prepare_send(event,start+offset)
    assert obj.state=='fault'

def test_late_terminal_publish_is_forced_zero():
    obj,start,events=commands()
    last=dict(events[-1],v=.3,w=.01,steering=.1)
    terminal=obj.prepare_send(last,start+q.DURATION_NS)
    assert terminal['phase']=='terminal_zero' and terminal['v']==terminal['w']==0

@pytest.mark.parametrize('mutation',['duplicate','missing','out_of_order','wrong_phase','wrong_v','wrong_w','wrong_stamp','wrong_identity'])
def test_command_audit_failure_inclusive(mutation):
    _,start,rows=commands()
    if mutation=='duplicate':rows.insert(8,copy.deepcopy(rows[7]))
    if mutation=='missing':del rows[7]
    if mutation=='out_of_order':rows[7:9]=rows[7:9][::-1]
    if mutation=='wrong_phase':rows[70]['phase']='straight'
    if mutation=='wrong_v':rows[70]['v']=.2
    if mutation=='wrong_w':rows[70]['w']=0.
    if mutation=='wrong_stamp':rows[70]['actual_ns']+=1
    if mutation=='wrong_identity':rows[70]['run_id']='other'
    assert not q.command_audit(rows,value(),start)['pass']

@pytest.mark.parametrize('mutation',['missing','duplicate','order','nan','negative','off_grid'])
def test_complete_dense_source_coverage(mutation):
    rows=list(range(1_000_000_000,1_010_000_001,q.STEP_NS))
    assert q.source_coverage(rows,rows[0],rows[-1])['pass']
    if mutation=='missing':del rows[2]
    elif mutation=='duplicate':rows.insert(2,rows[2])
    elif mutation=='order':rows[2:4]=rows[2:4][::-1]
    elif mutation=='nan':rows.append(float('nan'))
    elif mutation=='negative':rows.append(-1)
    else:rows[2]+=1
    assert not q.source_coverage(rows,1_000_000_000,1_010_000_000)['pass']

def service_rows():
    return [{'name':name,'phase':phase,'source_ns':stamp,'completed':True,'error':None} for name,phase,stamp in (
      ('/gazebo/spawn_urdf_model','bootstrap',0),('/gazebo/unpause_physics','bootstrap',0),
      ('/astra_q02/start','warmup',900_000_000),('/gazebo/pause_physics','terminal',11_000_000_000))]

@pytest.mark.parametrize('mutation',['reset','set_model','pause_early','duplicate','unknown_completion','missing'])
def test_service_gateway_trace_rejects_mutations(mutation):
    rows=service_rows();assert q.validate_services(rows,1_000_000_000,11_000_000_000)['pass']
    if mutation=='reset':rows.append(dict(rows[0],name='/gazebo/reset_world'))
    elif mutation=='set_model':rows.append(dict(rows[0],name='/gazebo/set_model_state'))
    elif mutation=='pause_early':rows[-1]['source_ns']-=1
    elif mutation=='duplicate':rows.append(copy.deepcopy(rows[-1]))
    elif mutation=='unknown_completion':rows[0]['completed']=False
    else:rows.pop()
    assert not q.validate_services(rows,1_000_000_000,11_000_000_000)['pass']

def test_recorder_barrier_requires_every_connected_input():
    rows=[[i,'/gazebo','i','TCPROS',topic,True] for i,topic in enumerate(q.RECORD_TOPICS)]
    assert q.recorder_connections(rows,q.RECORD_TOPICS)['pass']
    rows[-1][-1]=False
    assert not q.recorder_connections(rows,q.RECORD_TOPICS)['pass']
    rows[-1][-1]=True;rows[-1][2]='o'
    assert not q.recorder_connections(rows,q.RECORD_TOPICS)['pass']

@pytest.mark.parametrize('actor',[False,True])
def test_actual_world_geometry_and_camera_source_contract(actor):
    from evaluate_episode import WorldCollisions
    world=q.world_sdf(value(actor));data=WorldCollisions(world)
    assert not data.errors and len(data.primitives)==2+int(actor)
    tree=ET.fromstring(world)
    box=tree.find("world/model[@name='rotated_box']")
    assert float(box.findtext('pose').split()[-1])==.6
    assert box.findtext('link/collision/geometry/box/size')=='1.5 .5 1.2'
    cylinder=tree.find("world/model[@name='cylinder']/link/collision/geometry/cylinder")
    assert float(cylinder.findtext('radius'))==.45 and float(cylinder.findtext('length'))==1.2
    assert tree.findtext("world/model[@name='q02_camera']/link/sensor/camera/image/width")=='2560'
    assert tree.findtext("world/model[@name='q02_camera']/link/sensor/camera/image/height")=='1440'
    assert all('camera/image_raw' not in name for name in q.RECORD_TOPICS)
    if actor:
        assert data.actor_speed_bounds['q02_actor']>=.6
        assert tree.findtext("world/model[@name='q02_actor']/plugin/gate_route_x")=='-10'


def test_launch_has_exact_seed_and_only_gateway_spawns():
    data=value();root=q.STAGE/'runs'/data['run_id'];tree=ET.fromstring(runner.render_launch(root,data))
    assert tree.find("include/arg[@name='extra_gazebo_args']").get('value')=='--seed 260926015'
    nodes=tree.findall('node');assert not any(n.get('type')=='spawn_model' for n in nodes)
    expected={n.lstrip('/') for n in q.REQUIRED_NODES-{'/gazebo','/rosout'}}
    assert {n.get('name') for n in nodes}==expected
    assert '/m2_driver/command_ingress' in tree.find("node[@name='fixture_bag']").get('args')
    assert 'camera/image_raw' not in tree.find("node[@name='fixture_bag']").get('args')
    for name in ('passive_collector','passive_command'):
        item=tree.find("node[@name='%s']"%name)
        assert item.get('required')=='true'
        assert item.find("param[@name='contract_sha256']").get('value')==__import__('hashlib').sha256((json.dumps(data,indent=2)+'\n').encode()).hexdigest()


def test_hash_binding_mutation_and_existing_prepare_rejected(tmp_path,monkeypatch):
    path=tmp_path/'file';path.write_text('one');binding=q.binding(path);path.write_text('two')
    with pytest.raises(ValueError):q.read_bound(binding)
    with pytest.raises(FileExistsError):runner.prepare(tmp_path,4.)
    missing=tmp_path/'release.json';missing.write_text(json.dumps({'stage':q.NAME,'execution_authorized':False}))
    with pytest.raises(ValueError):runner.verify_release(missing,q.sha(missing))


def test_descriptive_rtf_no_causal_overhead_claim():
    item=q.source_rtf([(10,100),(20,140),(30,180)],10,30)
    assert item['ratio']==.25 and 'no observer-off' in item['interpretation']
    assert q.source_rtf([],None,None) is None


@pytest.mark.parametrize('wrong_backend',[False,True])
def test_command_node_paused_bootstrap_and_complete_clock_schedule(tmp_path,monkeypatch,wrong_backend):
    """Execute real node main against fake ROS: 30wall s paused, then 10sim s."""
    data=value();path=tmp_path/'contract.json';path.write_text(json.dumps(data))
    parameters={'/robot_description':(q.STAGE/'audits/passive_sweep_q02_samples/passive_sweep_q02_static.nominal_kd4.urdf.xml').read_text(),'/use_sim_time':True,'/m2_gazebo/simulation_only':True,'~contract_path':str(path),'~contract_sha256':q.sha(path)}
    fake=types.SimpleNamespace(clock=0,wall=0.,iterations=0,started=False,start=None,service=None,outputs={})
    class Message:
        def __init__(self,data=None):self.data=data
    class Response:
        def __init__(self,success,message):self.success=success;self.message=message
    class Twist:
        def __init__(self):self.linear=types.SimpleNamespace(x=0.);self.angular=types.SimpleNamespace(z=0.)
    class Publisher:
        def __init__(self,topic,*args,**kwargs):self.topic=topic;fake.outputs[topic]=[]
        def publish(self,message):fake.outputs[self.topic].append((fake.clock,message))
        def get_num_connections(self):return 2 if fake.iterations>=100 else 0
    def sleep(seconds):
        fake.wall+=seconds;fake.iterations+=1
        if fake.iterations>15000:fake.clock+=2_000_000
        if fake.clock>=1_000_000_000 and not fake.started:
            fake.started=True;answer=fake.service(None);assert answer.success
            fake.start=json.loads(answer.message)['start_ns']
            assert not fake.service(None).success
    fake_rospy=types.ModuleType('rospy')
    fake_rospy.init_node=lambda *a,**k:None
    fake_rospy.get_param=lambda name,default=None:parameters.get(name,default)
    fake_rospy.Publisher=Publisher
    fake_rospy.Service=lambda name,kind,fn:setattr(fake,'service',fn)
    fake_rospy.Time=types.SimpleNamespace(now=lambda:types.SimpleNamespace(to_nsec=lambda:fake.clock))
    fake_rospy.is_shutdown=lambda:fake.start is not None and fake.clock>=fake.start+q.DURATION_NS+60_000_000
    monkeypatch.setitem(sys.modules,'rospy',fake_rospy)
    for name,members in [('geometry_msgs.msg',{'Twist':Twist}),('std_msgs.msg',{'String':Message,'Int64':Message}),
      ('std_srvs.srv',{'Trigger':object,'TriggerResponse':Response})]:
        package,child=name.split('.');monkeypatch.setitem(sys.modules,package,types.ModuleType(package))
        module=types.ModuleType(name);module.__dict__.update(members);monkeypatch.setitem(sys.modules,name,module)
    monkeypatch.setenv('ROS_MASTER_URI','http://127.0.0.1:14820')
    entry=q.WORKSPACE/'src/simulation/m2_gazebo/scripts/astra_passive_command_q02.py'
    spec=importlib.util.spec_from_file_location('q02_node_under_test',entry);node=importlib.util.module_from_spec(spec);spec.loader.exec_module(node)
    monkeypatch.setattr(node,'time',types.SimpleNamespace(monotonic=lambda:fake.wall,monotonic_ns=lambda:round(fake.wall*1e9),sleep=sleep))
    if wrong_backend:
        parameters['/robot_description']=parameters['/robot_description'].replace('libastra_physical_ackermann_p05_plugin.so','libastra_physical_ackermann_p02_plugin.so')
        with pytest.raises(ValueError):node.main()
        assert fake.outputs=={}
        return
    assert node.main()==0
    emitted=[json.loads(msg.data) for _,msg in fake.outputs['/astra_q02/command_event']]
    assert sum(row['actual_ns']==0 for row in emitted)==1
    assert q.command_audit(emitted,data,fake.start)['pass']
    assert json.loads(fake.outputs['/astra_q02/command_status'][-1][1].data)['state']=='complete'
    assert all(msg.linear.x==0 and msg.angular.z==0 for stamp,msg in fake.outputs['/cmd_vel_navigation'] if stamp>=fake.start+q.DURATION_NS)


def test_real_p05_urdf_full_compound_nominal_geometry():
    """Use saved real xacro output; this is offline geometry, not a live trajectory."""
    from geometry import RobotCollisionModel,Pose3,compound_clearance
    from evaluate_episode import WorldCollisions,sdf_pose
    path=q.STAGE/'audits/passive_sweep_q02_samples/passive_sweep_q02_static.nominal_kd4.urdf.xml'
    assert path.is_file()
    tree=ET.fromstring(path.read_text())
    assert tree.find(".//plugin[@filename='libastra_physical_ackermann_p05_plugin.so']") is not None
    robot=RobotCollisionModel(path.read_text())
    joints={name:0. for name,j in robot.joints.items() if j.kind!='fixed'}
    shapes=robot.project(root_world_pose=Pose3(),joint_positions=joints)
    assert len(shapes)==9
    wheels=[p for p in shapes if '_wheel_link' in p.link_name]
    assert len(wheels)==4 and max(abs(p.z_min_m) for p in wheels)<1e-10
    world=WorldCollisions(q.world_sdf(value(True)))
    models={name:sdf_pose(element.find('pose')) for name,element in world.models.items()}
    # Far actor path endpoints and middle remain separated from this nominal robot box.
    for x,y,yaw in [(0,0,0),(2.2,.5,.3),(2.2,-.5,-.3)]:
        shapes=robot.project(root_world_pose=Pose3.from_xyz_rpy((x,y,0),(0,0,yaw)),joint_positions=joints)
        for actor_y in (-2.,0.,2.):
            models['q02_actor']=Pose3.from_xyz_rpy((5,actor_y,0),(0,0,math.pi/2))
            result=compound_clearance(shapes,world.project(models,{}))
            assert result['minimum_conservative_projected_clearance_m']>.7
            assert not result['potential_3d_intersection_pairs']


@pytest.fixture
def synthetic_release(tmp_path,monkeypatch):
    """Synthetic release for parser tests only, never an actual authorization artifact."""
    monkeypatch.setattr(q,'STAGE',tmp_path)
    source=tmp_path/'resource';source.write_text('offline synthetic binary/source')
    monkeypatch.setattr(q,'required_resources',lambda camera=True:[source])
    monkeypatch.setattr(q,'SOURCE_FILES',())
    monkeypatch.setattr(q,'render_urdf',lambda value:'<robot name="synthetic"/>\n')
    monkeypatch.setattr(q,'validate_robot_description',lambda xml,value:None)
    prepared=runner.prepare(tmp_path/'prepared',4.,True)
    descriptor=tmp_path/'prepared/prepared.json'
    native=tmp_path/'native';native.write_text('{}')
    decision=tmp_path/'decision.json';decision.write_text(json.dumps({'resolved_steering_kd':4.,'evidence_bindings':[q.binding(native)]}))
    review={'schema':'astra_passive_q02_source_review_v1','status':'independently_reviewed','resources':[],
      'robot_pose_velocity_writes':[],'actor_updates':'reviewed_bounded_Begin_updates',
      'engine_module_bindings':[dict(q.binding(source),role=role) for role in set(q.ELF_ROLES.values())],
      'native_engine_report':q.binding(native),'native_engine_records':q.binding(native),'upstream_source_manifest':q.binding(native),
      'engine_source_bindings':[q.binding(source)],
      'gazebo_callback_order_review':'BeforePhysicsUpdate then native integration then dirty-pose propagation then WorldUpdateEnd'}
    records=tmp_path/'native_records.jsonl';records.write_text('{}\n'+('{'+'"finite_rotation_mode":0}\n')*300)
    manifest=tmp_path/'upstream_manifest.json';manifest.write_text(json.dumps([q.binding(source)]))
    native.write_text(json.dumps({'why_end_twist_is_not_a_bound':{'native_library':q.binding(source)},
      'native_probe_artifacts':[q.binding(records)],'upstream_source_manifest':q.binding(manifest)}))
    review.update(native_engine_report=q.binding(native),native_engine_records=q.binding(records),upstream_source_manifest=q.binding(manifest))
    # The synthetic decision binds the final native report bytes.
    decision.write_text(json.dumps({'resolved_steering_kd':4.,'evidence_bindings':[q.binding(native)]}))
    review_path=tmp_path/'review.json';review_path.write_text(json.dumps(review))
    release={'stage':q.NAME,'status':'root_reviewed_release','execution_authorized':True,'maximum_episodes':2,
      'prepared':q.binding(descriptor),'physical_parameter_decision':q.binding(decision),'source_review':q.binding(review_path),
      'resources':[q.binding(source)]}
    path=tmp_path/'release.json'
    def write():path.write_text(json.dumps(release));return path,q.sha(path)
    return release,prepared,review,write


def test_exact_release_parser_offline(synthetic_release):
    release,prepared,review,write=synthetic_release
    result=runner.verify_release(*write())
    assert len(result[1]['units'])==2

@pytest.mark.parametrize('change',['budget_bool','auth_false','prepared_changed','binary_changed','missing_binding',
  'review_absent','review_duplicate_role','review_missing_role','callback_absent','kd_changed','world_changed','urdf_changed','launch_changed'])
def test_release_mutation_fails_before_any_execution(synthetic_release,change):
    release,prepared,review,write=synthetic_release
    if change=='budget_bool':release['maximum_episodes']=True
    elif change=='auth_false':release['execution_authorized']=False
    elif change=='prepared_changed':Path(release['prepared']['path']).write_text('{}')
    elif change=='binary_changed':Path(release['resources'][0]['path']).write_text('different bytes')
    elif change=='missing_binding':release['resources']=[]
    elif change=='kd_changed':
        path=Path(release['physical_parameter_decision']['path']);item=json.loads(path.read_text());item['resolved_steering_kd']=6.;path.write_text(json.dumps(item));release['physical_parameter_decision']=q.binding(path)
    elif change in ('world_changed','urdf_changed','launch_changed'):
        index={'world_changed':1,'urdf_changed':2,'launch_changed':3}[change]
        path=Path(prepared['units'][0]['files'][index]['path']);path.write_text('<tampered/>')
        prepared['units'][0]['files'][index]=q.binding(path)
        path=Path(release['prepared']['path']);path.write_text(json.dumps(prepared));release['prepared']=q.binding(path)
    else:
        if change=='review_absent':review['status']='pending'
        elif change=='review_duplicate_role':review['engine_module_bindings'].append(review['engine_module_bindings'][0])
        elif change=='review_missing_role':review['engine_module_bindings'].pop()
        elif change=='callback_absent':review.pop('gazebo_callback_order_review')
        path=Path(release['source_review']['path']);path.write_text(json.dumps(review));release['source_review']=q.binding(path)
    with pytest.raises(ValueError):runner.verify_release(*write())


@pytest.mark.parametrize('failure',['p02_substitute','duplicate_driver','seed','kd','nan','joint','extra_plugin'])
def test_actual_p05_startup_validator_mutations(failure):
    path=q.STAGE/'audits/passive_sweep_q02_samples/passive_sweep_q02_static.nominal_kd4.urdf.xml'
    xml=path.read_text();assert q.validate_robot_description(xml,value())['collision_count']==9
    tree=ET.fromstring(xml);driver=tree.find(".//plugin[@filename='libastra_physical_ackermann_p05_plugin.so']")
    if failure=='p02_substitute':driver.set('filename','libastra_physical_ackermann_p02_plugin.so')
    elif failure=='duplicate_driver':tree.append(copy.deepcopy(driver))
    elif failure=='seed':driver.find('dynamicsSeed').text='260926016'
    elif failure=='kd':driver.find('steeringKd').text='6.0'
    elif failure=='nan':driver.find('commandTimeout').text='nan'
    elif failure=='joint':tree.remove(tree.find('joint'))
    else:ET.SubElement(tree,'plugin',filename='libunknown_pose_writer.so')
    with pytest.raises(ValueError):q.validate_robot_description(ET.tostring(tree,encoding='unicode'),value())


def test_regression_after_fresher_send_timestamp_is_detected():
    obj=q.CommandSchedule(value());start=obj.start(1)
    event=obj.poll(start);obj.prepare_send(event,start+2_000_000)
    assert obj.last_ns==start+2_000_000
    with pytest.raises(ValueError):obj.poll(start+1_000_000)
    assert obj.state=='fault'
