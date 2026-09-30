#!/usr/bin/env python3
"""Independent synthetic assessor/fault fixtures, NOT Gazebo or physical evidence.

Read source and Q02 effective parameter metadata only. Never initialize ROS.
Every fixture is a separate in-memory copy; historical raw bytes remain read-only.
"""
import os,ast,copy,hashlib,importlib.util,json,math,pathlib,sys,traceback,yaml,types
W=pathlib.Path(__file__).resolve().parents[3];S=W.parent;P=W/'src/simulation/m2_gazebo/scripts';OUT=S/'audits/m12_p02_probe_independent_review.json'
def sha(path):return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()
SNAPSHOTS={}
def module(name,path):
 payload=path.read_bytes();SNAPSHOTS[str(path)]={'sha256':hashlib.sha256(payload).hexdigest(),'source_text':payload.decode()}
 m=types.ModuleType(name);m.__file__=str(path);exec(compile(payload,str(path),'exec'),vars(m));return m
old=module('old_q02',P/'astra_actuation_probe_q02.py');PROBE_PATH=pathlib.Path(os.environ.get('P02_PROBE_TEST_SOURCE',str(P/'astra_physical_actuation_probe_p02.py')));probe=module('physical_p02',PROBE_PATH);contract=yaml.safe_load((S/'contracts/m12_physics_p02.yaml').read_text());oldcontract=yaml.safe_load((S/'contracts/actuation_q02.yaml').read_text())
# Reuse only the analytically generated fixture body, not recorded motion data.
func=next(n for n in ast.parse((P/'astra_actuation_probe_q02.py').read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='offline_fixtures');i=next(i for i,n in enumerate(func.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='baseline' for t in n.targets));func.body=func.body[:i]+[ast.Return(value=ast.Name(id='raw',ctx=ast.Load()))];func.name='construct_original_synthetic';tree=ast.fix_missing_locations(ast.Module(body=[func],type_ignores=[]));ns=dict(vars(old));exec(compile(tree,'synthetic_fixture_source','exec'),ns)
effective=copy.deepcopy(json.loads((S/'runs/actuation_q02/probe_raw.json').read_text())['effective_configuration']);base=ns['construct_original_synthetic'](contract,effective)
effective['plugin']=copy.deepcopy(contract['effective_plugin_parameters'])
base['synthetic_only']=True;base['metadata']['all_body_states_every_physics_step']=True;base['targets']=[];base['dense_models']=[];base['dense_links']=[]
for od,j in zip(base['odom'],base['joints']):
 od['vy']=0.;j['velocities']={k:0. for k in j['positions']}
 center,_=probe.pair_center(j['positions']['front_left_steer_joint'],j['positions']['front_right_steer_joint'],.65,.6)
 base['targets'].append({'t':j['t'],'ns':j['ns'],'positions':{'speed':od['v'],'center_steering':center,'front_left_steering':j['positions']['front_left_steer_joint'],'front_right_steering':j['positions']['front_right_steer_joint']},'velocities':{k:0. for k in ('speed','center_steering','front_left_steering','front_right_steering')},'names':['speed','center_steering','front_left_steering','front_right_steering'],'position_count':4,'velocity_count':4,'frame':'actuator_reference_units_named'})
# Provide explicit per-link twists and all four wheel links, analytically derived.
for model,links,odom,joint in zip(base['models'],base['links'],base['odom'],base['joints']):
 yaw=old.yaw(odom['p']);v=odom['v'];w=odom['w'];tw=[v*math.cos(yaw),v*math.sin(yaw),0.,0.,0.,w]
 model['twists']=[[0.]*6,tw.copy()];links['twists']=[tw.copy() for _ in links['names']]
 for axle in ('front','rear'):
  for side in ('left','right'):
   name='autolabor_m2::'+axle+'_'+side+'_wheel_link';pose=copy.deepcopy(links['poses'][1 if side=='left' else 2] if axle=='front' else odom['p']);links['names'].append(name);links['poses'].append(pose);links['twists'].append(tw.copy())
   joint['positions'][axle+'_'+side+'_wheel_joint']=0.;joint['velocities'][axle+'_'+side+'_wheel_joint']=v/.15
# Dense analytic interpolation at exact integer source nanoseconds. Low-rate
# samples are copied exactly at coincident stamps. This is a toy, no solver.
for key,out in [('models','dense_models'),('links','dense_links')]:
 coarse=base[key]
 for step in range(int(contract['total_schedule_s']/.002)+1):
  at=step*2000000;idx=min(step//10,len(coarse)-1);a=coarse[idx];b=coarse[min(idx+1,len(coarse)-1)];u=(step%10)/10.;r=copy.deepcopy(a);r['ns']=at;r['t']=at*1e-9
  if u and idx<len(coarse)-1:
   for ai,bi,po in zip(a['poses'],b['poses'],r['poses']):
    for k in range(3):po[k]=ai[k]*(1-u)+bi[k]*u
    ya=old.yaw(ai);yb=old.yaw(bi);h=ya+old.angle_diff(yb,ya)*u;po[3:]=[0.,0.,math.sin(h/2),math.cos(h/2)]
  base[out].append(r)
# Single valid normal support contact; all non-wheel ground variants must fail.
base['contacts'][1000]['pairs']=[{'a':'autolabor_m2::front_left_wheel_link::collision','b':'ground_plane::link::collision','depths':[0.0001],'positions':[[0,0,0]],'normals':[[0,0,1]],'source_info':'synthetic_only'}]
for i,r in enumerate(base['targets']):
 if i:
  for key,value in r['positions'].items():r['velocities'][key]=(value-base['targets'][i-1]['positions'][key])/.02
for r in base['odom']:r['frame']='odom';r['child_frame']='base_link'
for r in base['joints']:
 r['names']=list(r['positions']);r['position_count']=len(r['positions']);r['velocity_count']=len(r['velocities']);r['frame']='';r['efforts']={}
print('TEST_LOADED_PROBE_SHA256 '+SNAPSHOTS[str(PROBE_PATH)]['sha256'],flush=True)
params=contract['effective_plugin_parameters']
base['physical_metadata']={'backend':'astra_physical_ackermann_v1','effort_only':True,'calibrated':False}
for key,param in [('actual_mass_kg','expectedMass'),('wheelbase_m','wheelbase'),('track_m','track'),('wheel_radius_m','wheelRadius'),('wheel_drive_torque_limit_nm','wheelDriveTorqueLimit'),('wheel_brake_torque_limit_nm','wheelBrakeTorqueLimit'),('steering_torque_limit_nm','steeringTorqueLimit')]:base['physical_metadata'][key]=params[param]
base['backend_fault']=[{'t':r['t'],'value':False} for r in base['joints']]
base['commanded_efforts']=[{'ns':r['ns'],'t':r['t'],'frame':'commanded_torque_not_reaction','names':list(r['positions']),'effort':[0. for _ in r['positions']]} for r in base['joints']]
# P02 synthetic identity / integer source schedule / recorder handshake.
base['run_id']=contract['run_id'];base['seed']=contract['seed'];base['contract_sha256']=sha(S/'contracts/m12_physics_p02.yaml')
OFFSET_NS=1100000000;OFFSET_S=1.1
base['schedule_start_ns']=OFFSET_NS;base['schedule_start_s']=OFFSET_S
for key in ('models','links','dense_models','dense_links','odom','joints','targets','contacts','commanded_efforts'):
 for row in base[key]:row['ns']+=OFFSET_NS;row['t']=row['ns']/1e9
for key in ('backend_fault','commands_sent','commands_observed'):
 for row in base[key]:row['t']+=OFFSET_S
for row in base['commands_sent']:
 row['ns']=round(row['t']*1e9);row['scheduled_ns']=row['ns'];row['lateness_ns']=0;row['missed_slots']=0;row['wall_monotonic_s']=row['t']
base['commands_sent'].insert(0,{'ns':1050000000,'t':1.05,'phase':'bootstrap_zero','v':0.,'w':0.,'scheduled_ns':1050000000,'lateness_ns':0,'missed_slots':0,'wall_monotonic_s':1.05})
base['commands_sent'].append({'ns':37100000000,'t':37.1,'phase':'terminal_zero','v':0.,'w':0.,'scheduled_ns':37100000000,'lateness_ns':0,'missed_slots':0,'wall_monotonic_s':37.1})
base['backend_status']=[]
for row in base['joints']:
 timeout=row['ns']>=33600000000
 values={'backend_fault':'false','command_timeout':'true' if timeout else 'false','brake':'false','emergency':'false'}
 base['backend_status'].append({'ns':row['ns'],'t':row['t'],'frame':'','statuses':[{'name':'astra_physical_ackermann','hardware_id':'simulation-only','level':1 if timeout else 0,'message':'command_timeout' if timeout else 'ok','values':values,'value_count':4}]})
base['source_status']=[]
for state,nsvalue in [('bootstrap',900000000),('running',1000000000),('complete',37100000000)]:
 values={'state':state,'run_id':base['run_id'],'seed':str(base['seed']),'contract_sha256':base['contract_sha256'],'reason':'','start_ns':'' if state=='bootstrap' else str(base['schedule_start_ns'])}
 base['source_status'].append({'ns':nsvalue,'t':nsvalue/1e9,'frame':'','statuses':[{'name':'astra_actuation_command_source','hardware_id':'simulation-only','level':0,'message':state,'values':values,'value_count':6}]})
base['command_source_start_response']={'run_id':base['run_id'],'seed':base['seed'],'contract_sha256':base['contract_sha256'],'start_ns':base['schedule_start_ns']}
base['schedule_start_records']=[{'data':base['schedule_start_ns'],'receipt_t':1.}]
base['recording_topics']=list(contract['required_topics'])
base['recorder_connection_barrier']={'node':'/fixture_bag','uri':'http://127.0.0.1:43210/','required':sorted(base['recording_topics']),'connected':base['recording_topics'][:],'missing':[],'wall_monotonic_s':1.}
base['warmup_start_ns']=0;base['warmup_end_ns']=1000000000

CASES=[]
def case(name,mutate,expect_pass=False,gate=None,expected_gate=None,claim=None):
 raw=copy.deepcopy(base);mutate(raw)
 try:
  r=probe.assess(raw,contract);actual=r['pass'];gate_actual=r.get('checks',{}).get(gate) if gate else None
  ok=(actual==expect_pass) if gate is None else gate_actual==expected_gate
  CASES.append({'name':name,'expected_pass':expect_pass if gate is None else None,'expected_gate':{gate:expected_gate} if gate else None,'actual_pass':actual,'actual_gate':gate_actual,'fixture_pass':ok,'false_pass':actual and expect_pass is False and gate is None,'failed_checks':[k for k,v in r.get('checks',{}).items() if not v],'metrics_excerpt':{k:v for k,v in r.get('metrics',{}).items() if k in ('internal_target_acceleration_mps2','internal_target_center_slew_radps','measured_odom_acceleration_mps2','robot_nonwheel_ground_contact_pair_count')},'claim':claim})
 except BaseException as exc:CASES.append({'name':name,'fixture_pass':not expect_pass and gate is None,'actual_pass':False,'exception':type(exc).__name__+': '+str(exc),'traceback':traceback.format_exc(),'claim':claim})
def set_at(key,index,field,value):return lambda r:r[key][index].__setitem__(field,value)
def contact_pair(robot,other,reverse=False):
 pair={'a':robot,'b':other,'depths':[.001],'positions':[[0,0,0]],'normals':[[0,0,1]],'source_info':'synthetic_only'}
 if reverse:pair['a'],pair['b']=pair['b'],pair['a']
 return lambda r:r['contacts'][1000].__setitem__('pairs',[pair])
case('valid_analytic_baseline',lambda r:None,True)
case('normal_wheel_ground_reverse_order',contact_pair('autolabor_m2::rear_right_wheel_link::collision','ground_plane::link::collision',True),True)
case('base_ground_rejected',contact_pair('autolabor_m2::base_link::collision','ground_plane::link::collision'))
case('steer_ground_rejected_reverse',contact_pair('autolabor_m2::front_left_steer_link::collision','ground_plane::link::collision',True))
case('wheel_obstacle_rejected',contact_pair('autolabor_m2::front_left_wheel_link::collision','obstacle::link::collision'))
case('robot_self_collision_rejected',contact_pair('autolabor_m2::front_left_wheel_link::collision','autolabor_m2::base_link::collision'))
case('dense_missing_step_rejected',lambda r:r['dense_models'].pop(333))
case('dense_duplicate_stamp_rejected',lambda r:r['dense_models'].insert(333,copy.deepcopy(r['dense_models'][332])))
case('dense_cross_topic_stamp_offset_rejected',lambda r:[p.__setitem__('ns',p['ns']+1000000) for p in r['dense_links']])
case('dense_body_payload_empty_must_fail',lambda r:[p.update(names=[],poses=[],twists=[]) for k in ('dense_models','dense_links') for p in r[k]],claim='An every-step timestamp shell is not body truth.')
case('dense_same_stamp_body_mismatch_must_fail',lambda r:r['dense_models'][200].__setitem__('poses',[[0,0,0,0,0,0,1],[30,30,0,0,0,0,1]]),claim='Dense and low-rate source-stamp-identical poses must agree.')
case('dense_ros_seconds_disagree_with_ns_must_fail',lambda r:[p.__setitem__('t',p['t']+.001) for k in ('dense_models','dense_links') for p in r[k]],claim='Header seconds/nanoseconds encode the same source stamp.')
case('dense_named_pose_length_mismatch_must_fail',lambda r:r['dense_links'][333].__setitem__('poses',[]))
case('actual_joint_values_nonsense_must_fail',lambda r:[p['positions'].__setitem__('front_left_steer_joint',99.) for p in r['joints']],claim='Actual joint_states contents must be independently coherent with actual link state.')
case('target_left_right_missing_must_fail',lambda r:[p.__setitem__('positions',{k:v for k,v in p['positions'].items() if k in ('speed','center_steering')}) for p in r['targets']],claim='Target schema requires all four named fields.')
case('target_left_right_geometry_wrong_must_fail',lambda r:[p['positions'].update(front_left_steering=20.,front_right_steering=20.) for p in r['targets']])
case('target_speed_above_effective_max_must_fail',lambda r:[p['positions'].__setitem__('speed',p['positions']['speed']+10.) for p in r['targets']])
case('target_acceleration_violation_rejected',lambda r:r['targets'][200]['positions'].__setitem__('speed',10.))
case('target_slew_violation_rejected',lambda r:r['targets'][200]['positions'].__setitem__('center_steering',.4))
def coherent_odom_velocity(r,index=200,v=None,vy=None):
 o=r['odom'][index]
 if v is not None:o['v']=v
 if vy is not None:o['vy']=vy
 h=old.yaw(o['p']);world_v=[o['v']*math.cos(h)-o['vy']*math.sin(h),o['v']*math.sin(h)+o['vy']*math.cos(h),0.]
 for key in ('models','links','dense_models','dense_links'):
  packet=next(p for p in r[key] if p['ns']==o['ns'])
  name=contract['model_name'] if key.endswith('models') else contract['model_name']+'::base_link'
  packet['twists'][packet['names'].index(name)][:3]=world_v
case('actual_derivative_not_used_for_internal_gate',lambda r:coherent_odom_velocity(r,v=10.),gate='internal_accel_limit',expected_gate=True,claim='Coherent actual body velocity may exceed target acceleration; internal target gate must remain separate.')
case('nonzero_lateral_actual_twist_rotates_correctly',lambda r:coherent_odom_velocity(r,index=350,vy=.2),True)
case('actual_twist_inconsistent_with_truth_must_fail',lambda r:r['odom'][200].__setitem__('v',10.),claim='Independent target gate should pass; odom velocity truth consistency must reject the false measurement.')
case('target_nan_rejected',lambda r:r['targets'][200]['positions'].__setitem__('speed',float('nan')))
case('actual_slew_overlimit_cannot_pass',lambda r:r['links'][350]['poses'][1].__setitem__(5,r['links'][350]['poses'][1][5]+.03))
case('dense_nonfinite_rejected',lambda r:r['dense_links'][200]['poses'][0].__setitem__(0,float('nan')))
case('command_loss_extra_publication_rejected',lambda r:r['commands_observed'].append({'t':34.,'v':1.2,'w':0.}))
case('actual_vy_truth_mismatch_rejected',lambda r:r['odom'][200].__setitem__('vy',.3))
case('backend_fault_late_true_rejected',lambda r:r['backend_fault'][-1].__setitem__('value',True))
case('legacy_Bool_missing_is_not_source_gate',lambda r:r.__setitem__('backend_fault',[]),True)
case('legacy_Bool_initial_only_is_not_source_gate',lambda r:r.__setitem__('backend_fault',r['backend_fault'][:1]),True)
case('physical_metadata_wrong_backend_rejected',lambda r:r['physical_metadata'].__setitem__('backend','kinematic_pose_setter'))
case('physical_metadata_wrong_mass_rejected',lambda r:r['physical_metadata'].__setitem__('actual_mass_kg',100.))
case('commanded_torque_overlimit_rejected',lambda r:r['commanded_efforts'][200]['effort'].__setitem__(0,999.))
case('commanded_effort_wrong_shape_rejected',lambda r:r['commanded_efforts'][200].__setitem__('effort',[0.]))
case('commanded_effort_dropped_step_rejected',lambda r:r['commanded_efforts'].pop(200))
case('target_instantaneous_accel_rate_rejected',lambda r:r['targets'][200]['velocities'].__setitem__('speed',2.))
case('target_instantaneous_steer_rate_rejected',lambda r:r['targets'][200]['velocities'].__setitem__('center_steering',2.))
case('target_duplicate_names_rejected',lambda r:r['targets'][200]['names'].__setitem__(1,'speed'))
def coherent_actual_slew_failure(r):
 i=350;model=contract['model_name'];p=next(x for x in r['dense_links'] if x['ns']==r['links'][i]['ns']);b=r['links'][i]['poses'][0];heading=old.yaw(b);c=r['targets'][i]['positions']['center_steering']+.06;k=math.tan(c)/params['wheelbase'];angles=[math.atan2(params['wheelbase']*k,1-sign*params['track']*k/2) for sign in (1,-1)]
 for name,joint,angle in zip(contract['front_link_names'],contract['front_joint_names'],angles):
  q=[0.,0.,math.sin((heading+angle)/2),math.cos((heading+angle)/2)]
  for packet in (r['links'][i],p):packet['poses'][packet['names'].index(model+'::'+name)][3:]=q[:]
  r['joints'][i]['positions'][joint]=angle
case('unit_quaternion_coherent_actual_slew_rejected',coherent_actual_slew_failure,gate='actual_center_slew_with_declared_sampling_tolerance',expected_gate=False)

# New source protocol faults. Each is a distinct fresh toy, not recorded physics.
case('backend_status_missing_rejected',lambda r:r.__setitem__('backend_status',[]))
case('backend_status_dropped_step_rejected',lambda r:r['backend_status'].pop(200))
case('backend_status_wrong_level_rejected',lambda r:r['backend_status'][200]['statuses'][0].__setitem__('level',2))
def shift_backend(r):
 for row in r['backend_status']:row['ns']+=1000000;row['t']=row['ns']/1e9
case('backend_status_shifted_source_ns_rejected',shift_backend)
case('backend_status_duplicate_key_count_rejected',lambda r:r['backend_status'][200]['statuses'][0].__setitem__('value_count',5))
case('source_status_no_complete_rejected',lambda r:r['source_status'].pop())
case('source_status_only_early_complete_rejected',lambda r:r.__setitem__('source_status',[dict(r['source_status'][-1],ns=1000000000,t=1.)]))
case('source_status_wrong_name_rejected',lambda r:r['source_status'][-1]['statuses'][0].__setitem__('name','other'))
case('source_status_wrong_hardware_rejected',lambda r:r['source_status'][-1]['statuses'][0].__setitem__('hardware_id','vehicle'))
case('source_status_wrong_start_rejected',lambda r:r['source_status'][-1]['statuses'][0]['values'].__setitem__('start_ns','999'))
case('source_status_duplicate_key_count_rejected',lambda r:r['source_status'][-1]['statuses'][0].__setitem__('value_count',7))
case('source_status_wrong_order_rejected',lambda r:r['source_status'].reverse())
case('source_status_fault_rejected',lambda r:r['source_status'][1]['statuses'][0]['values'].__setitem__('state','fault'))
case('source_status_negative_level_rejected',lambda r:r['source_status'][-1]['statuses'][0].__setitem__('level',-1))
case('recorder_claims_empty_connected_rejected',lambda r:r['recorder_connection_barrier'].__setitem__('connected',[]))
case('recorder_wrong_required_roster_rejected',lambda r:r['recorder_connection_barrier'].update(required=['/bogus'],connected=['/bogus'],missing=[]))
case('recorder_wrong_node_rejected',lambda r:r['recorder_connection_barrier'].__setitem__('node','/wrong'))
case('recorder_remote_uri_rejected',lambda r:r['recorder_connection_barrier'].__setitem__('uri','http://robot.lan:11311/'))
case('recorder_missing_topic_rejected',lambda r:r['recorder_connection_barrier'].__setitem__('missing',[contract['required_topics'][0]]))
case('warmup_short_rejected',lambda r:r.__setitem__('warmup_end_ns',900000000))
case('event_wrong_command_value_rejected',lambda r:r['commands_sent'][100].__setitem__('v',.8))
case('event_wrong_phase_rejected',lambda r:r['commands_sent'][100].__setitem__('phase','turn_negative'))
case('event_ns_t_inconsistent_rejected',lambda r:r['commands_sent'][100].__setitem__('t',r['commands_sent'][100]['t']+.001))
case('event_duplicate_slot_rejected',lambda r:r['commands_sent'].insert(100,copy.deepcopy(r['commands_sent'][100])))
case('event_loss_boundary_zero_rejected',lambda r:r['commands_sent'].insert(-1,{'ns':33100000000,'t':33.1,'phase':'fault_stop_zero','v':0.,'w':0.,'scheduled_ns':33100000000,'lateness_ns':0,'missed_slots':0}))
case('event_no_terminal_rejected',lambda r:r['commands_sent'].pop())
case('event_terminal_early_rejected',lambda r:r['commands_sent'][-1].update(ns=37099999999,t=37.099999999,scheduled_ns=37099999999))
case('schedule_marker_wrong_rejected',lambda r:r['schedule_start_records'][0].__setitem__('data',2))
recorder_fixture={'only_incoming_connected_TCPROS_counts':probe.recorder_connections([[1,'publisher','o','TCPROS','/a',True],[2,'publisher','i','UDPROS','/a',True],[3,'publisher','i','TCPROS','/a',False],[4,'publisher','i','TCPROS','/b',True]],['/a','/b'])=={'required':['/a','/b'],'connected':['/b'],'missing':['/a']}}

resources=[P/'astra_physical_actuation_probe_p02.py',S/'contracts/m12_physics_p02.yaml',P.parent/'src/astra_dense_evidence_plugin.cpp',W/'scripts/astra_gazebo/run_m12_physics_p02.py',P/'astra_actuation_probe_q02.py',S/'runs/actuation_q02/probe_raw.json']
rotation_checks={}
for name,angle,axis,world_v,expected in [('roll90',math.pi/2,0,[0.,1.,0.],[0.,0.,-1.]),('pitch90',math.pi/2,1,[0.,0.,1.],[-1.,0.,0.]),('yaw90',math.pi/2,2,[0.,1.,0.],[1.,0.,0.])]:
 q=[0.,0.,0.,math.cos(angle/2)];q[axis]=math.sin(angle/2);actual=probe.world_vector_in_body(world_v,q);rotation_checks[name]=max(abs(a-b) for a,b in zip(actual,expected))<1e-12
report={'recorder_connection_fixtures':recorder_fixture,'world_vector_inverse_rotation_fixtures':rotation_checks,'status':'independent_synthetic_assessor_audit_pass' if all(x['fixture_pass'] for x in CASES) else 'independent_synthetic_assessor_false_pass_defects','synthetic_only':True,'no_simulation_started':True,'cases':CASES,'case_count':len(CASES),'passing_fixture_count':sum(c['fixture_pass'] for c in CASES),'source_resources':[{'path':str(p),'sha256':sha(p)} for p in resources],'loaded_source_snapshots':SNAPSHOTS,'loaded_source_changed_during_test':{p:sha(p)!=v['sha256'] for p,v in SNAPSHOTS.items()},'test_script_path':str(pathlib.Path(__file__).resolve()),'test_script_sha256':sha(__file__),'commands':[('P02_PROBE_TEST_SOURCE='+str(PROBE_PATH)+' ' if os.environ.get('P02_PROBE_TEST_SOURCE') else '')+'workspace/.venv/bin/python workspace/scripts/astra_gazebo/tests/test_physical_actuation_probe_p02.py'],'working_directory':str(S),'prior_tool_errors':[{'command':'cat workspace/src/simulation/m2_gazebo/test/test_astra_actuation_probe.py','error':'No such file or directory; used independent new test file as assigned.'},{'command':'sed include/m2_gazebo/astra_physical_actuation.h','error':'No such file; rg resolved actual astra_physical_control.h; no source changed.'}],'all_motion_phases_unchanged':contract['phase_schedule']==oldcontract['phase_schedule'],'frozen_original_gates_unchanged':contract['gates']==oldcontract['gates']}
if OUT.exists():
 i=1
 while OUT.with_name(OUT.stem+'.prior'+str(i)+'.json').exists():i+=1
 OUT.with_name(OUT.stem+'.prior'+str(i)+'.json').write_bytes(OUT.read_bytes())
OUT.write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'status':report['status'],'count':len(CASES),'passed':report['passing_fixture_count'],'false_passes':[x['name'] for x in CASES if x.get('false_pass')],'unexpected_exceptions':[{'name':x['name'],'exception':x.get('exception')} for x in CASES if 'exception' in x],'baseline':CASES[0],'source_probe_sha256':SNAPSHOTS[str(PROBE_PATH)]['sha256']},indent=2))
if __name__=='__main__':sys.exit(0 if all(x['fixture_pass'] for x in CASES) else 1)
