"""P03 offline fixtures. Synthetic audit facts never constitute a run release."""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import dense_sweep_p03 as p03
from geometry import Pose3

def fixture(actor=False):
    other='actor' if actor else 'wall'
    plugin='<plugin name="actor" filename="libastra_reactive_actor_plugin.so"><base_speed>.8</base_speed><initial_speed>.8</initial_speed><ego_model>autolabor_m2</ego_model><policy>cautious</policy><start>4 0</start><end>4 3</end><conflict>4 1</conflict><route_origin>0 0</route_origin><route_yaw>0</route_yaw><gate_route_x>0</gate_route_x><gate_delay>0</gate_delay></plugin>' if actor else ''
    world='<sdf version="1.6"><world name="test"><physics type="ode"><max_step_size>.002</max_step_size></physics><model name="%s"><static>true</static><pose>4 0 0 0 0 0</pose><link name="body"><collision name="box"><geometry><box><size>.72 .72 1</size></box></geometry></collision></link>%s</model></world></sdf>'%(other,plugin)
    robot=p03.ROBOT+'::base_link';obstacle=other+'::body'
    def body(name,isrobot):
        return {'name':name,'mass_kg':2.,'com_local_m':[.1,0,0] if isrobot else [0,0,0],
            'native_ode_mass_kg':2.,'native_ode_mass_center':[0,0,0],'has_ode_body':isrobot,'finite_rotation_mode':0,'kinematic':False,
            'collisions':[{'name':name+'::box','kind':'box','size_xyz_m':[1.,.4,.4] if isrobot else [.72,.72,1.],
                'relative_pose':{'xyz':[0,0,0],'xyzw':[0,0,0,1]}}]}
    meta={'schema':'astra_physics_runtime_p03_v1','revision':1,'stamp_ns':1000000000,'phase':'BeforePhysicsUpdate','epoch':0,
        'valid_numeric':True,'passive_observer':True,'planner_input_authorized':False,
        'config':{'physics_type':'ode','solver_type':'quick','max_step_s':.002,
            'models':[{'name':p03.ROBOT,'static':False,'links':[body(robot,True)]},
                      {'name':other,'static':True,'links':[body(obstacle,False)]}]}}
    streams=[[] for _ in range(4)];statuses=[]
    for i in range(11):
        stamp=1000000000+i*2000000
        for k,rows in enumerate(streams):
            x=(i-(1 if k<2 else 0))*.002;y=i*.001 if actor else 0.
            rows.append({'stamp_ns':stamp,'frame_id':'world','names':[p03.ROBOT,other] if k%2==0 else [robot,obstacle],
                'poses':[(x,0,0,0,0,0,1),(4,y,0,0,0,0,1)],'twists':[(999.,)*6,(0.,)*6]})
        statuses.append({'schema':'astra_physics_step_p03_v1','stamp_ns':stamp,'step_index':i,'epoch':0,'valid':True,
            'pre_end_same_source_stamp':True,'same_roster':True,'pre_revision':1,'end_revision':1,'config_stable_within_step':True,
            'pre_enabled':[{'name':robot,'enabled':True,'native_com_world_m':[(i-1)*.002+.1,0,0],'native_quaternion_xyzw':[0,0,0,1]}],
            'end_enabled':[{'name':robot,'enabled':True,'native_com_world_m':[i*.002+.1,0,0],'native_quaternion_xyzw':[0,0,0,1]}]})
    return dict(pre_model_rows=streams[0],pre_link_rows=streams[1],end_model_rows=streams[2],end_link_rows=streams[3],
        statuses=statuses,metadata_records=[meta],world_xml=world,start_ns=1000000000,end_ns=1020000000,
        closure_audit={'pass':True,'pose_writer_review':{'actor_updates':'reviewed_bounded_Begin_updates'},'bindings':[{'role':'world','sha256':hashlib.sha256(world.encode()).hexdigest()},{'role':'actor_sources'},{'role':'actor_binaries'}]})

def test_exact_pre_end_budget_ignores_large_measured_twist_and_pays_pads():
    report,rows,endpoints=p03.assess_records(**fixture())
    assert report['integrity_pass'],report['issues']
    assert report['qualification']=='qualified_within_declared_Gazebo_engineering_scope'
    assert report['formal_safety'] is False
    assert len(rows)==10 and len(endpoints)==2
    assert rows[0]['robot_point_budget_m']==pytest.approx(.00202)
    assert rows[0]['relative_clearance_variation_budget_m']==pytest.approx(.00203)
    assert report['minimum_engineering_conservative_swept_xy_m']==pytest.approx(4-.02-.5-.36-.33-1e-5-.0203/2)

@pytest.mark.parametrize('failure',['rotation_mode','kinematic','disabled','pre_gap','missing_pre','missing_status','duplicate_status',
    'out_of_order','reset_epoch','step_index','metadata_revision','missing_metadata','future_metadata','bad_quaternion',
    'nan_twist','pose_domain','configuration','world_binding','closure','solver','unknown_dynamic','link_only_move'])
def test_failed_obligations_do_not_qualify(failure):
    data=fixture();meta=data['metadata_records'][0];status=data['statuses'][5];link=meta['config']['models'][0]['links'][0]
    if failure=='rotation_mode':link['finite_rotation_mode']=1
    elif failure=='kinematic':link['kinematic']=True
    elif failure=='disabled':status['pre_enabled'][0]['enabled']=False
    elif failure=='pre_gap':data['pre_link_rows'][5]['poses'][0]=(.03,0,0,0,0,0,1)
    elif failure=='missing_pre':del data['pre_link_rows'][5]
    elif failure=='missing_status':del data['statuses'][5]
    elif failure=='duplicate_status':data['statuses'].append(copy.deepcopy(status))
    elif failure=='out_of_order':data['end_link_rows'][3:5]=data['end_link_rows'][3:5][::-1]
    elif failure=='reset_epoch':status['epoch']=1
    elif failure=='step_index':status['step_index']=1
    elif failure=='metadata_revision':status['end_revision']=2
    elif failure=='missing_metadata':data['metadata_records']=[]
    elif failure=='future_metadata':meta['stamp_ns']+=1
    elif failure=='bad_quaternion':data['pre_link_rows'][5]['poses'][0]=(.008,0,0,0,0,0,1.00000005)
    elif failure=='nan_twist':data['end_link_rows'][5]['twists'][0]=(float('nan'),)*6
    elif failure=='pose_domain':data['end_link_rows'][5]['poses'][0]=(1001,0,0,0,0,0,1)
    elif failure=='configuration':status['config_stable_within_step']=False
    elif failure=='world_binding':data['world_xml']=data['world_xml'].replace('.72 .72','1 1')
    elif failure=='closure':data['closure_audit']['pass']=False
    elif failure=='solver':meta['config']['solver_type']='world'
    elif failure=='unknown_dynamic':data['world_xml']=data['world_xml'].replace('<static>true','<static>false')
    elif failure=='link_only_move':data['end_link_rows'][5]['poses'][1]=(4.01,0,0,0,0,0,1)
    report,_,_=p03.assess_records(**data)
    assert not report['integrity_pass'] and report['issues']
    assert report['qualification']=='pending_or_invalid_evidence'
    assert report['clearance_status']!='pass_within_model_scope'

def test_actor_zero_twist_begin_jump_bounded_and_illegal_during_solver_rejected():
    data=fixture(True);report,rows,_=p03.assess_records(**data)
    assert report['integrity_pass'],report['issues']
    assert rows[0]['actor_Begin_variation_budget_m']>=1.35*.002
    data['end_link_rows'][5]['poses'][1]=(4,.008,0,0,0,0,1)
    assert not p03.assess_records(**data)[0]['integrity_pass']

def test_side_component_changes_collision_minimum_and_runtime_com_radius():
    data=fixture();body=data['metadata_records'][0]['config']['models'][0]['links'][0]
    body['collisions'].append({'name':p03.ROBOT+'::base_link::side','kind':'box','size_xyz_m':[.8,.8,.4],
        'relative_pose':{'xyz':[3.1,0,0],'xyzw':[0,0,0,1]}})
    report,rows,endpoints=p03.assess_records(**data)
    assert report['integrity_pass'],report['issues']
    assert endpoints[0]['robot_components']==2
    assert endpoints[0]['physical_xy_m']==pytest.approx(.14)
    assert report['clearance_status']=='fail_or_unavailable'
    assert rows[0]['bodies'][p03.ROBOT+'::base_link']['shape_radius_from_body_com_m']>3.

def test_tolerated_phase_shift_is_explicitly_paid():
    data=fixture()
    for i in range(1,11):
        old=data['pre_link_rows'][i]['poses'][0];data['pre_link_rows'][i]['poses'][0]=(old[0]+5e-8,)+old[1:]
    report,rows,_=p03.assess_records(**data)
    assert report['integrity_pass'],report['issues']
    assert rows[0]['bodies'][p03.ROBOT+'::base_link']['End_to_Pre_engineering_variation_m']==pytest.approx(1e-5+5e-8)

def test_empty_obstacles_is_not_positive_clearance_pass():
    data=fixture();data['world_xml']='<sdf version="1.6"><world name="empty"><physics type="ode"><max_step_size>.002</max_step_size></physics></world></sdf>'
    data['closure_audit']['bindings'][0]['sha256']=hashlib.sha256(data['world_xml'].encode()).hexdigest()
    data['metadata_records'][0]['config']['models'].pop()
    for key in ('pre_model_rows','pre_link_rows','end_model_rows','end_link_rows'):
        for row in data[key]:
            for field in ('names','poses','twists'):row[field].pop()
    report,_,_=p03.assess_records(**data)
    assert report['integrity_pass'] and report['clearance_status']=='not_applicable_no_nonground_obstacle'

def closure_fixture(tmp_path):
    run='synthetic_unit_fixture_not_release';bindings={}
    def file(role,value):
        path=tmp_path/(role+'.json');path.write_text(json.dumps(value))
        record={'path':str(path),'sha256':p03.sha(path)};bindings[role]=[record];return record
    for role in ('launch','world','robot_description','driver_source','driver_binary','evidence_source','evidence_binary'):file(role,{'role':role})
    bindings['actor_sources']=[];bindings['actor_binaries']=[]
    common={'pass':True,'run_id':run,'start_ns':1,'end_ns':3000000000}
    file('node_roster_audit',dict(common,loopback_master_isolated=True,unexpected_nodes=[]))
    file('service_call_audit',dict(common,forbidden_calls=[],scope='authorized_launch_and_nodes'))
    file('pose_writer_review',dict(common,robot_pose_velocity_writes=[],actor_updates='none'))
    file('engine_binary_maps',dict(common,loaded_modules=[dict(bindings['driver_binary'][0],roles=['ode_integrator','gazebo_physics']),bindings['evidence_binary'][0]],implementation_review={'source_bindings':bindings['driver_source'],
        'ode_normal_euler_update_verified':True,'gazebo_pre_end_callback_order_verified':True,
        'loaded_module_source_correspondence_reviewed':True}))
    document={'schema':'astra_p03_run_closure_v1','scope':p03.SCOPE,'numeric_profile':p03.PROFILE,'run_id':run,'bindings':bindings}
    path=tmp_path/'closure.json';path.write_text(json.dumps(document));return path,document

def test_closure_hashes_audits_and_loaded_driver_membership(tmp_path):
    path,doc=closure_fixture(tmp_path)
    assert p03.authenticate_closure(path,1000000000,1020000000)['pass']
    Path(doc['bindings']['driver_source'][0]['path']).write_text('changed')
    assert not p03.authenticate_closure(path,1000000000,1020000000)['pass']

@pytest.mark.parametrize('role',['node_roster_audit','service_call_audit','pose_writer_review','engine_binary_maps'])
def test_missing_review_fails_closed_without_index_error(tmp_path,role):
    path,doc=closure_fixture(tmp_path);doc['bindings'][role]=[];path.write_text(json.dumps(doc))
    assert not p03.authenticate_closure(path,1000000000,1020000000)['pass']

def test_no_library_name_whitelist_can_supply_engine_implementation_review(tmp_path):
    path,doc=closure_fixture(tmp_path);record=doc['bindings']['engine_binary_maps'][0];p=Path(record['path'])
    value=json.loads(p.read_text());value['implementation_review']={};p.write_text(json.dumps(value));record['sha256']=p03.sha(p);path.write_text(json.dumps(doc))
    assert not p03.authenticate_closure(path,1000000000,1020000000)['pass']

def test_high_precision_budget_numeric_envelope_and_rotating_com():
    from decimal import Decimal as D, getcontext
    import numpy as np
    from scipy.spatial.transform import Rotation
    getcontext().prec=70;rng=np.random.RandomState(260926013);worst=0.
    def vec(xs):return [D(float(x)) for x in xs]
    def norm(xs):return sum(x*x for x in xs).sqrt()
    def rotate(q,v):
        q=[x/norm(q) for x in q];x,y,z,w=q
        matrix=[[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
        return [sum(a*b for a,b in zip(row,v)) for row in matrix]
    for _ in range(1000):
        q0=Rotation.random(random_state=rng);axis=rng.normal(size=3);axis/=np.linalg.norm(axis)
        theta=10**rng.uniform(-12,math.log10(.249));q1=Rotation.from_rotvec(axis*theta)*q0
        com=rng.normal(size=3);com=com/np.linalg.norm(com)*rng.uniform(0,9.99)
        center=rng.uniform(-985,985,3);dc=rng.normal(size=3);dc=dc/np.linalg.norm(dc)*rng.uniform(0,.249)
        first=tuple(center-q0.apply(com))+tuple(q0.as_quat()*(1+rng.uniform(-.9e-8,.9e-8)))
        second=tuple(center+dc-q1.apply(com))+tuple(q1.as_quat()*(1+rng.uniform(-.9e-8,.9e-8)))
        radius=rng.uniform(0,10);actual=p03.point_budget(first,second,com,radius)
        a,b=vec(first),vec(second);qa,qb=a[3:],b[3:];qa=[x/norm(qa) for x in qa];qb=[x/norm(qb) for x in qb]
        dot=sum(x*y for x,y in zip(qa,qb))
        if dot<0:qb=[-x for x in qb];dot=-dot
        tangent=norm([y-dot*x for x,y in zip(qa,qb)])/dot
        ca=[x+y for x,y in zip(a[:3],rotate(qa,vec(com)))];cb=[x+y for x,y in zip(b[:3],rotate(qb,vec(com)))]
        reference=norm([y-x for x,y in zip(ca,cb)])+2*D(float(radius))*tangent
        error=float(abs(reference-D(actual['point_phase_budget_m'])));worst=max(worst,error)
        assert D(actual['engineering_point_budget_m'])>=reference
    print("Decimal70 maximum budget discrepancy m:",worst)
    assert worst<1e-9

@pytest.mark.parametrize('kind',['rotation','translation','radius','com'])
def test_out_of_numeric_domain_rejected(kind):
    first=(0,0,0,0,0,0,1);second=(0,0,0,0,0,0,1);com=(0,0,0);radius=1.
    if kind=='rotation':q=Pose3.from_xyz_rpy(rpy=(0,0,.251));second=tuple(q.xyz)+tuple(q.xyzw)
    elif kind=='translation':second=(.251,0,0,0,0,0,1)
    elif kind=='radius':radius=10.01
    else:com=(10.01,0,0)
    with pytest.raises(ValueError):p03.point_budget(first,second,com,radius)


def test_independent_scipy_shapely_endpoint_pad_in_large_coordinate_domain():
    import itertools
    import numpy as np
    from scipy.spatial.transform import Rotation
    from shapely.geometry import MultiPoint
    from geometry import project_box, compound_clearance
    rng=np.random.RandomState(260926014);worst=0.
    for _ in range(200):
        center=rng.uniform(-950,950,3);dims=rng.uniform(.02,10,3);rotation=Rotation.random(random_state=rng)
        points=rotation.apply(np.array(list(itertools.product((-1,1),repeat=3)))*dims/2)+center
        second=points+np.array([15,5,0])
        reference=MultiPoint(points[:,:2]).convex_hull.distance(MultiPoint(second[:,:2]).convex_hull)
        box=project_box('a',dims,Pose3(center,rotation.as_quat()))
        other=project_box('b',dims,Pose3(center+np.array([15,5,0]),rotation.as_quat()))
        actual=compound_clearance([box],[other],0.)['minimum_physical_projected_clearance_m']
        worst=max(worst,abs(actual-reference))
        assert actual-p03.PROFILE['endpoint_clearance_pad_m']<=reference
    print('SciPy/Shapely maximum endpoint discrepancy m:',worst)
    assert worst<1e-9

@pytest.mark.parametrize('part',['mass','native_com','native_rotation','collision_size','collision_missing'])
def test_native_ode_and_saved_collision_correspondence(part):
    data=fixture()
    if part=='mass':data['metadata_records'][0]['config']['models'][0]['links'][0]['native_ode_mass_kg']=3.
    elif part=='native_com':data['statuses'][5]['end_enabled'][0]['native_com_world_m'][0]+=.001
    elif part=='native_rotation':data['statuses'][5]['end_enabled'][0]['native_quaternion_xyzw']=[0,0,.1,math.sqrt(.99)]
    elif part=='collision_size':data['metadata_records'][0]['config']['models'][1]['links'][0]['collisions'][0]['size_xyz_m'][0]=.8
    else:data['metadata_records'][0]['config']['models'][1]['links'][0]['collisions']=[]
    assert not p03.assess_records(**data)[0]['integrity_pass']

def test_offline_rosbag_cli_uses_source_stamps_and_captures_outputs(tmp_path,monkeypatch):
    import rosbag
    from genpy import Time
    from gazebo_msgs.msg import WorldState
    from geometry_msgs.msg import Pose, Twist
    from std_msgs.msg import String
    data=fixture();closure,document=closure_fixture(tmp_path)
    world=Path(document['bindings']['world'][0]['path']);world.write_text(data['world_xml'])
    urdf=Path(document['bindings']['robot_description'][0]['path'])
    urdf.write_text('<robot name="test"><link name="base_link"><inertial><origin xyz=".1 0 0"/><mass value="2"/><inertia ixx=".1" iyy=".1" izz=".1" ixy="0" ixz="0" iyz="0"/></inertial><collision name="box"><geometry><box size="1 .4 .4"/></geometry></collision></link></robot>')
    for role,path in (('world',world),('robot_description',urdf)):document['bindings'][role][0]['sha256']=p03.sha(path)
    closure.write_text(json.dumps(document));bagfile=tmp_path/'synthetic.bag'
    messages=[]
    for key,topic in (('pre_model_rows','pre_models'),('pre_link_rows','pre_links'),('end_model_rows','end_models'),('end_link_rows','end_links')):
        for row in data[key]:
            message=WorldState();message.header.stamp=Time(row['stamp_ns']//1000000000,row['stamp_ns']%1000000000)
            message.header.frame_id=row['frame_id'];message.name=row['names']
            for p,t in zip(row['poses'],row['twists']):
                pose=Pose();pose.position.x,pose.position.y,pose.position.z=p[:3]
                pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=p[3:];message.pose.append(pose)
                twist=Twist();twist.linear.x,twist.linear.y,twist.linear.z=t[:3]
                twist.angular.x,twist.angular.y,twist.angular.z=t[3:];message.twist.append(twist)
            messages.append((row['stamp_ns'],topic,message))
    for key,topic in (('statuses','step_status'),('metadata_records','runtime_metadata')):
        for row in data[key]:messages.append((row['stamp_ns'],topic,String(json.dumps(row))))
    with rosbag.Bag(str(bagfile),'w') as bag:
        for stamp,topic,message in sorted(messages,key=lambda x:x[0]):
            # Deliberately offset receipt by 75ms: evaluator must use source.
            receipt=stamp+75000000
            bag.write('/astra/p03/'+topic,message,Time(receipt//1000000000,receipt%1000000000))
    output=tmp_path/'evaluation'
    monkeypatch.setattr(sys,'argv',['dense_sweep_p03','--bag',str(bagfile),'--world',str(world),'--urdf',str(urdf),
        '--closure',str(closure),'--output',str(output),'--start-ns',str(data['start_ns']),'--end-ns',str(data['end_ns'])])
    assert p03.main()==0
    report=json.loads((output/'report.json').read_text())
    assert report['integrity_pass'] and report['dense_solver_intervals']==10
    assert report['source_window_ns']==[1000000000,1020000000]
    assert len((output/'intervals.jsonl').read_text().splitlines())==10
    assert len((output/'endpoints.csv').read_text().splitlines())==3
    with pytest.raises(FileExistsError):p03.main()

def test_separate_compound_body_maxima_for_join_and_solver_phases():
    data=fixture();second=p03.ROBOT+'::second'
    body=copy.deepcopy(data['metadata_records'][0]['config']['models'][0]['links'][0]);body['name']=second
    body['collisions'][0]['name']=second+'::box';data['metadata_records'][0]['config']['models'][0]['links'].append(body)
    for i in range(11):
        for key,ispre in (('pre_link_rows',True),('end_link_rows',False)):
            row=data[key][i];x=(i-1)*.001+5e-8 if ispre else i*.001
            row['names'].append(second);row['poses'].append((x,1,0,0,0,0,1));row['twists'].append((0.,)*6)
            label='pre_enabled' if ispre else 'end_enabled'
            data['statuses'][i][label].append({'name':second,'enabled':True,'native_com_world_m':[x+.1,1,0],'native_quaternion_xyzw':[0,0,0,1]})
    report,rows,_=p03.assess_records(**data)
    assert report['integrity_pass'],report['issues']
    assert rows[0]['robot_point_budget_m']==pytest.approx(.00201+1e-5+5e-8)
