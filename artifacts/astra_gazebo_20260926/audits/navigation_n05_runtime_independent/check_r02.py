"""Independent r02 source/actual prepared-input closure; no ROS/build/witness regeneration."""
from pathlib import Path
import ast,copy,csv,difflib,hashlib,json,math,sys,xml.etree.ElementTree as ET
import yaml
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';A=Path(__file__).resolve().parent;P=W/'scripts/astra_gazebo';sys.path.insert(0,str(P))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def bind(p):return {'path':str(Path(p).resolve()),'sha256':sha(p)}
def funcs(p):return {n.name:n for n in ast.parse(Path(p).read_text()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
def segment(p,name):return ast.get_source_segment(Path(p).read_text(),funcs(p)[name])
checks={};detail={};resources={}
def check(name,condition):
 checks[name]=bool(condition)
 if not condition:raise AssertionError(name)
def verify(row):
 p=Path(row['path']);check('binding:'+str(p),sha(p)==row['sha256']);resources[str(p)]=bind(p)
snap_path=S/'audits/navigation_n05_runtime_derivation/source_snapshot_r02.json';snap=json.loads(snap_path.read_bytes());old=json.loads((A/'source_snapshot_r01.json').read_bytes())
check('r02_snapshot_exact_requested_SHA',sha(snap_path)=='db2362b8f9e5e0fe9827f063df7c5ff37db2f0c666edc8388843067af2e6d828')
for row in snap['resources']:
 verify(row);dest=A/'snapshot_r02'/Path(row['path']).relative_to(W);dest.parent.mkdir(parents=True,exist_ok=True)
 if dest.exists():assert dest.read_bytes()==Path(row['path']).read_bytes()
 else:dest.write_bytes(Path(row['path']).read_bytes())
(A/'source_snapshot_r02.json').write_bytes(snap_path.read_bytes())
current={x['path']:x['sha256'] for x in snap['resources']};changed=[Path(x['path']).name for x in old['resources'] if current.get(x['path'])!=x['sha256']]
check('r01_only_runner_and_its_mock_target_changed',changed==['run_navigation_n05.py','test_navigation_n05_pipeline.py'])
old_runner=A/'snapshot_r01/scripts/astra_gazebo/run_navigation_n05.py';runner=P/'run_navigation_n05.py';before=old_runner.read_text();after=runner.read_text()
expected=before.replace('import run_smoke_a02 as common','import run_smoke_a02 as common\nimport navigation_process_cleanup_n05 as cleanup').replace("'release_validation_session_n05.py')","'release_validation_session_n05.py', 'navigation_process_cleanup_n05.py')").replace('common.stop_group(proc, root)','cleanup.stop_group(proc, root)')
check('runner_three_exact_repair_changes',expected==after)
(A/'runner_r01_to_r02.diff').write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='r01',tofile='r02')))
helper=P/'navigation_process_cleanup_n05.py';original=P/'run_smoke_a02.py';a=segment(original,'stop_group');b=segment(helper,'stop_group')
check('stop_group_byte_exact_except_port_pair_and_report_key',b==a.replace('14820','14830').replace('14821','14831'))
check('cleanup_imports_original_ownership_atomic_utc', 'from run_smoke_a02 import owned_processes, atomic, utc' in helper.read_text())
(A/'stop_group_port_only.diff').write_text(''.join(difflib.unified_diff(a.splitlines(True),b.splitlines(True),fromfile='run_smoke_a02.stop_group',tofile='N05.stop_group')))
# Previous independent AST/loop/control/launch and schedule checks carry forward only under exact source equality.
prior=json.loads((A/'checks_r01.json').read_bytes())
check('r01_single_retained_failure_is_port_integration',[k for k,v in prior['checks'].items() if not v]==['N05_actual_terminal_ports_verified'])
check('all_r01_other10_checks_retained_with_source_equivalence',sum(prior['checks'].values())==10 and changed==['run_navigation_n05.py','test_navigation_n05_pipeline.py'])
# Parse current actual prepared scenes; regenerate ONLY small metadata/XML, no map/witness build.
import prepare_navigation_n05_scenes as new
import prepare_navigation_n04_scenes as oldscene
import event_actor_contract_n05 as event
import run_navigation_n05 as manager
prep_path=S/'scenes/navigation_n05_development_r01/preparation_report.json';prep=json.loads(prep_path.read_bytes());resources[str(prep_path)]=bind(prep_path)
for row in prep['files']:verify(row)
for path,digest in prep['parent_bindings'].items():verify({'path':path,'sha256':digest})
verify(prep['design_binding']);check('actual_prepared_generator_matches',sha(Path(new.__file__))==prep['generator_sha256'])
for name in ['write_json','put','collision_signature','ReferenceRobot','reduced_clearance','witness','scene_qc','static_input']:
 check('unchanged_scene_AST:'+name,ast.dump(funcs(oldscene.__file__)[name],include_attributes=False)==ast.dump(funcs(new.__file__)[name],include_attributes=False))
rows=manager.input_blocks();parents=list(csv.DictReader((S/'scenes/development_a02_live_v2/diagnostic_blocks.csv').open()));schedule=manager.design()['schedule'];summaries=[]
check('actual_input_roster_12_and_exact24',len(rows)==12 and len(schedule)==24)
for index,(row,parentrow) in enumerate(zip(rows,parents)):
 seed=int(row['seed']);d=Path(row['manifest_path']).parent;parent=yaml.safe_load(Path(parentrow['manifest_path']).read_bytes());raw=Path(parentrow['world_path']).read_text();derived,world=new.derive(parent,raw,seed)
 actual_world=Path(row['world_path']).read_text();scene=yaml.safe_load(Path(row['manifest_path']).read_bytes())
 check('prepared_world_exact_derive:%02d'%index,world==actual_world)
 check('parent_saved_bytes_preserved:%02d'%index,(d/'parent/world.sdf').read_bytes()==Path(parentrow['world_path']).read_bytes() and (d/'parent/scene_manifest.yaml').read_bytes()==Path(parentrow['manifest_path']).read_bytes())
 check('actual_collision_signature_preserved:%02d'%index,new.collision_signature(actual_world)==new.collision_signature(raw))
 for key in ('family','layout_id','parent_geometry_id','canonical_geometry_sha256','scene_id','seed','replicate','actors','route','actor_activation','camera','paired_method_roster','derivation_design_sha256'):
  check('prepared_manifest_semantic:%02d:%s'%(index,key),scene[key]==derived[key])
 oldd=S/'scenes/navigation_n04_development_r01'/d.relative_to(S/'scenes/navigation_n05_development_r01')
 for name in ('map.pgm','map.yaml','common_route.json'):
  check('unchanged_N04_common_bundle:%02d:%s'%(index,name),(d/'common_bundle'/name).read_bytes()==(oldd/'common_bundle'/name).read_bytes())
 route=json.loads((d/'common_bundle/common_route.json').read_bytes());points=route['points_xy'];length=sum(math.hypot(b[0]-a[0],b[1]-a[1]) for a,b in zip(points,points[1:]))
 check('route_length_recomputed:%02d'%index,abs(length-prep['input_rows'][index]['route_length_m'])<1e-12)
 static=ET.parse(d/'map_export/static_projection_input.sdf').getroot();actors={a['model_name'] for a in scene['actors']};check('map_projection_excludes_actors:%02d'%index,not actors.intersection({m.get('name') for m in static.findall('world/model')}))
 for identity in schedule[2*index:2*index+2]:
  bound=new.bind_gate_identity(actual_world,identity['run_id'],seed,identity['method']);parsed=event.world_actors(bound)
  check('exact_method_actor_identity:'+identity['run_id'],set(parsed)==actors and all(a['run_id']==identity['run_id'] and a['seed']==seed and a['method']==identity['method'] and a['max_gate_transport_delay_ns']==50_000_000 for a in parsed.values()))
  check('per_run_binding_geometry_unchanged:'+identity['run_id'],new.collision_signature(bound)==new.collision_signature(actual_world))
 summaries.append({'index':index,'seed':seed,'family':row['family'],'actors':len(actors),'guide_length_m_recomputed':length,'reported_static_witness_pass':prep['input_rows'][index]['static_witness_pass']})
check('reported_occlusion_negative_retained',prep['all_static_witnesses_pass'] is False and [r['family'] for r in summaries if not r['reported_static_witness_pass']]==['occluded_emergence']*2)
check('reference_URDF_SDF_equal_previously_reviewed_N04',all((S/'scenes/navigation_n05_development_r01/reference_robot'/n).read_bytes()==(S/'scenes/navigation_n04_development_r01/reference_robot'/n).read_bytes() for n in ['robot_description.xml','converted_robot.sdf']))
for p in [snap_path,original,helper,runner,Path(new.__file__),Path(oldscene.__file__),P/'event_actor_contract_n05.py',S/'audits/navigation_n05_components_independent/review.json',S/'audits/release_validation_session_n05_independent/review.json',S/'audits/navigation_n05_scene_derivation/implementation_report.json']:
 resources[str(p)]=bind(p)
# Collapse verbose per-file/per-scene positives without discarding their machine evidence.
report={'status':'independent_source_and_prepared_input_checks_pass','checks':checks,'check_count':len(checks),'all_checks_pass':all(checks.values()),'r02_resources_rehashed':len(snap['resources']),'prepared_files_rehashed':len(prep['files']),'parent_bindings_rehashed':len(prep['parent_bindings']),'input_summaries':summaries,'resources':list(resources.values()),'scene_geometry_witness_recomputed':False,'source_and_metadata_only':True,'ROS_started':False,'native_build_run':False,'seed_allocated':False,'release_verified':False,'r01_failure_preserved':bind(A/'checks_r01.json')}
(A/'checks_r02.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('checks','resources')},indent=2))
