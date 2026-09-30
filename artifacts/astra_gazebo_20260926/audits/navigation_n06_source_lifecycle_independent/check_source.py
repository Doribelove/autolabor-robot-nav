"""N06 actual frozen source/lifecycle checks. No launch execution or authority."""
import ast,copy,hashlib,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo';G=W/'src/simulation/m2_gazebo'
sys.path.insert(0,str(P))
import prepare_navigation_n06_static as prep
import navigation_evidence_n06 as ev
import navigation_coordinator_n06 as nc
import run_passive_sweep_q05 as passive
checks=[];observations=[]
def bind(p):
 p=Path(p).resolve();b=p.read_bytes();return {'path':str(p),'sha256':hashlib.sha256(b).hexdigest(),'bytes':len(b)}
def bound(row):assert bind(row['path'])['sha256']==row['sha256'];return Path(row['path'])
def add(name,**kw):checks.append(dict(name=name,passed=True,**kw))
def fail(fn):
 try:fn()
 except (ValueError,KeyError,TypeError):return
 raise AssertionError('fault input accepted')
clones=json.loads((S/'audits/navigation_n06_preparation/identity_clone_derivations.json').read_text())
for row in clones:
 assert bind(row['parent'])['sha256']==row['parent_sha256'] and bind(row['child'])['sha256']==row['child_sha256']
 text=Path(row['parent']).read_text()
 for a,b in row['replacements']:assert text.count(a)==1;text=text.replace(a,b)
 assert text==Path(row['child']).read_text()
add('four_entire_frozen_identity_only_derivations_recomputed',count=len(clones))
factor=prep.treatment_equivalence();assert factor['configuration_delta']=={'lookahead_m':[.7,1.2]}
add('actual_treatment_control_only_three_text_substitutions_and_one_config_factor',delta=factor['configuration_delta'])

prepared=json.loads((prep.PREPARED/'prepared.json').read_text());schedule=json.loads((prep.PREPARED/'schedule.json').read_text())
assert schedule['schedule']==prep.schedule() and len(prepared['units'])==8
pair={}
for unit,row in zip(prepared['units'],prep.schedule()):
 assert all(type(unit[k])is type(v) and unit[k]==v for k,v in row.items())
 value=json.loads(bound(unit['contract']).read_text());ev.validate_contract(value)
 assert value['stage']==prep.NAME and not value['execution_authorized'] and value['attempt']==1
 assert bound(unit['launch']).read_text()==prep.render_launch(value,Path(unit['contract']['path']),S/'runs'/unit['run_id'])
 for k in ('world','robot_description','gate_expectation'):bound(unit[k])
 world=ET.fromstring(bound(unit['world']).read_text());urdf=ET.fromstring(bound(unit['robot_description']).read_text())
 assert not ev.event_actor.world_actors(ET.tostring(world,encoding='unicode'))
 assert {x.get('filename') for x in world.findall('world/plugin')}=={'libastra_dense_evidence_plugin.so','libastra_physics_metadata_p03_plugin.so'}
 plugins={x.get('filename') for x in urdf.findall('.//plugin')}
 assert plugins=={'libastra_physical_ackermann_p05_plugin.so','libgazebo_ros_bumper.so','libgazebo_ros_imu_sensor.so','libgazebo_ros_laser.so'}
 assert value['allowed_nodes']==value['required_nodes']==prep.nodes() and value['record_topics']==prep.topics()
 assert len(value['record_topics'])==len(set(value['record_topics']))==37
 gate=json.loads(Path(unit['gate_expectation']['path']).read_text());assert gate['lookahead_m']==row['lookahead_m'] and gate['timeout_ns']==22_000_000_000 and gate['activation_ns']==5_000_000_000
 pair.setdefault(row['block_index'],[]).append(unit)
 observations.append({'run_id':row['run_id'],'method':row['method'],'seed':row['seed'],'contract':bind(unit['contract']['path']),'world':unit['world'],'urdf':unit['robot_description'],'topics':len(value['record_topics']),'actor_count':0})
for rows in pair.values():assert rows[0]['seed']==rows[1]['seed'] and rows[0]['world']['sha256']==rows[1]['world']['sha256'] and rows[0]['robot_description']['sha256']==rows[1]['robot_description']['sha256']
add('all_eight_actual_contract_rendering_rosters_frames_and_paired_physics',units=8,pairs=4)

launch=ET.fromstring(prep.LAUNCH.read_text());nodes=launch.findall('.//node')
collector=[n for n in nodes if n.get('name')=='astra_navigation_collector'];assert len(collector)==1 and collector[0].get('required')=='true'
assert {g.get('if') for g in launch.findall('group')}=={"$(eval arg('method') == 'astra_analytic_route_v1')","$(eval arg('method') == 'astra_analytic_lookahead_1p2_n06')"}
for group in launch.findall('group'):
 method=group.find('node');assert [(r.get('from'),r.get('to')) for r in method.findall('remap')]==[('/cmd_vel_navigation','/astra/method_cmd')]
 assert method.get('launch-prefix')=='$(env THESIS_WS)/.venv/bin/python'
assert not launch.findall('.//env')
add('actual_launch_private_remaps_required_collector_and_no_environment_overrides')

collector_tree=ast.parse((G/'scripts/astra_navigation_collector_n06.py').read_text())
gateway_calls=[]
for node in ast.walk(collector_tree):
 if isinstance(node,ast.Call) and isinstance(node.func,ast.Name) and node.func.id=='gateway':
  assert isinstance(node.args[0],ast.Constant);gateway_calls.append((node.lineno,node.args[0].value))
assert set(x[1] for x in gateway_calls)==set(ev.SERVICE_PHASES)
assert all('reset' not in name and 'delete' not in name and 'set_model' not in name for _,name in gateway_calls)
assert sum(name=='/gazebo/spawn_urdf_model' for _,name in gateway_calls)==1
assert sum(name=='/gazebo/unpause_physics' for _,name in gateway_calls)==1
for name in ('astra_navigation_coordinator_n06.py','astra_route_coordinated_runtime.py','astra_route_lookahead_runtime_n06.py','astra_common_route_provider.py'):
 tree=ast.parse((G/'scripts'/name).read_text())
 assert not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='ServiceProxy' for n in ast.walk(tree))
add('actual_collector_only_fixed_service_gateway_and_single_spawn_unpause',call_sites=gateway_calls)

coord_tree=ast.parse((G/'scripts/astra_navigation_coordinator_n06.py').read_text())
private_tree=ast.parse((G/'scripts/astra_route_lookahead_runtime_n06.py').read_text())
literal_topics=[]
for node in ast.walk(private_tree):
 if isinstance(node,ast.Constant) and isinstance(node.value,str) and node.value.startswith('/'):
  literal_topics.append(node.value)
assert not any(x in literal_topics for x in ('/gazebo/model_states','/gazebo/link_states','/astra/p03/end_models','/astra/p03/end_links'))
assert all(x in literal_topics for x in ('/scan','/odom','/joint_states','/map','/astra/common_route/path'))
add('actual_planner_sensor_map_route_inputs_no_truth_topics',topics=sorted(set(literal_topics)))

# Source-shaped coordinator timing tests; no transport or robot motion is claimed.
for method in nc.METHODS:
 for order in ('clock_first','odom_first'):
  c=nc.NavigationCoordinator(nc.CoordinatorConfig('n06_synthetic',263003001,method))
  c.method_status(dict(run_id='n06_synthetic',seed=263003001,method=method,state='ready',ready=True))
  c.observe_odom(1_000_000_000,(-5.4,0.,0.),(0.,0.,0.));c.arm(1_000_000_000)
  c.observe_odom(5_000_000_000,(-5.4,0.,0.),(0.,0.,0.));c.output(5_000_000_000)
  gate=c.observe_odom(7_000_000_000,(-4.,0.,0.),(.8,0.,0.));assert gate
  c.method_status(dict(run_id='n06_synthetic',seed=263003001,method=method,state='active',ready=True,gate_source_ns=gate['source_ns']))
  c.observe_odom(28_980_000_000,(4.,0.,0.),(.8,0.,0.))
  if order=='clock_first':out=c.output(29_000_000_000);assert out['v']==out['w']==0 and c.phase=='active'
  c.observe_odom(28_999_999_999,(5.,0.,0.),(.8,0.,0.));assert c.phase=='complete'
  c.output(29_000_000_000);c.fault('later_paused_clock_stall');assert c.phase=='complete' and c.terminal_source_ns==28_999_999_999 and c.late_fault_count==1
add('both_N06_methods_ordered_actual_goal_before_deadline_and_first_terminal_immutable',cases=4)

value=json.loads(Path(prepared['units'][0]['contract']['path']).read_text())
for change in ('wrong_method','missing_essential_topic','bad_frame_spawn','noninteger_seed','camera_in_dense_bag'):
 bad=copy.deepcopy(value)
 if change=='wrong_method':bad['method']='astra_analytic_preview_eligible_n05'
 elif change=='missing_essential_topic':bad['record_topics'].remove('/odom')
 elif change=='bad_frame_spawn':bad['spawn_world_xyz_yaw'][0]+=1
 elif change=='noninteger_seed':bad['seed']=str(bad['seed'])
 else:bad['record_topics'].append(bad['camera_topic'])
 fail(lambda:ev.validate_contract(bad))
add('actual_N06_contract_method_source_frame_type_and_recorder_rejections',cases=5)

parent_review=json.loads((S/'audits/navigation_n05_root_release/source_review.json').read_text())
native={k:copy.deepcopy(parent_review[k]) for k in ('native_engine_report','native_engine_records','upstream_source_manifest','engine_source_bindings','gazebo_callback_order_review')}
native['engine_module_bindings']=[r for r in parent_review['engine_module_bindings'] if r['role']!='actor_binaries']
assert len(native['engine_module_bindings'])==14 and len({r['role'] for r in native['engine_module_bindings']})==14
for row in native['engine_module_bindings']+native['engine_source_bindings']+[native[k] for k in ('native_engine_report','native_engine_records','upstream_source_manifest')]:bound(row)
facts=passive.native_review_facts(native);assert facts['native_reference_pass'] and facts['upstream_source_review_pass']
assert set(ev.ELF_ROLES.values())-{'actor_binaries'}=={r['role'] for r in native['engine_module_bindings']}
for name in ('astra_physical_ackermann_p05_plugin.cpp','astra_dense_evidence_plugin.cpp','astra_physics_metadata_p03_plugin.cpp'):
 text=(G/'src'/name).read_text()
 assert not any(s in text for s in ('SetWorldPose(', 'SetWorldLinearVel(', 'SetWorldAngularVel(', 'SetLinearVel(', 'SetAngularVel(', 'SetPosition(', 'SetVelocity('))
 if name.startswith('astra_physical_ackermann'):assert 'SetForce(' in text and 'ConnectWorldUpdateBegin' in text
 else:assert 'SetForce(' not in text and 'ConnectWorldUpdateEnd' in text
add('inherited_fourteen_native_roles_rehashed_actual_native_facts_and_writer_review',facts=facts,actor_role_inactive=True)

out={'schema':'n06_independent_source_lifecycle_checks_v1','checks':checks,'actual_units':observations,'native_reference':native,'factor':factor,'final_roster_pending':True,'scope':'source/static protocol checks; native proof inherited and rehashed, no rebuild or simulator; existing 8 launch expansions inherited rather than rerun','execution_authorized':False,'actual_release_created':False,'seed_registered':False,'ROS_started':False}
(A/'source_checks.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({'groups':len(checks),'output':bind(A/'source_checks.json')},indent=2))
