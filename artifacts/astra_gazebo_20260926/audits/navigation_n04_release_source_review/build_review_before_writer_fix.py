"""Seal independent Python review; authority stays with parent release/registry."""
import ast,copy,json,hashlib,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import run_navigation_n04 as manager
import navigation_evidence_n04 as evidence
import run_passive_sweep_q05 as passive

def load(p):return json.loads(Path(p).read_bytes())
def bind(p):p=Path(p).resolve();return {'path':str(p),'sha256':evidence.sha(p)}
def write(name,v):(D/name).write_text(json.dumps(v,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
def functions(path):return {n.name:ast.dump(n) for n in ast.parse(Path(path).read_text()).body if isinstance(n,(ast.FunctionDef,ast.ClassDef))}
old_path=S/'audits/navigation_n03_release_source_review/source_review.json';old=load(old_path)
freeze_path=S/'audits/navigation_n04_root_source_freeze_r02.json';freeze=load(freeze_path)
for x in freeze['files']:evidence.read_bound(x)
focused=load(D/'focused_review.json');assert focused['status']=='pass' and len(focused['prepared_rows'])==48
for x in focused['source_files']:evidence.read_bound(x)
status=load(D/'status_boundary_review.json');evidence.read_bound(status['source']);assert len(status['cases'])==10
prepared_path=S/'contracts/navigation_n04_prepared_r01/prepared.json';prepared=load(prepared_path)
preflight_path=S/'audits/navigation_n04_root_launch_preflight_r01/report.json';preflight=load(preflight_path)
assert len(prepared['units'])==len(preflight['runs'])==48
# The preflight precedes only source roster/auditor corrections. Its launch
# generator remains AST-identical, and actual prepared output is revalidated.
snapshot=load(S/'audits/navigation_n04_root_source_r01_snapshot/manifest.json')
old_manager=next(x['snapshot'] for x in snapshot['entries'] if x['original']['path']==str(Path(manager.__file__).resolve()))
assert functions(old_manager)['render_launch']==functions(manager.__file__)['render_launch']
# No dense numeric/profile/reference changes. Replayed old bag is evidence of
# numeric inheritance, not live N04 event actor qualification.
f0=functions(W/'scripts/astra_gazebo/dense_sweep_p04.py');f1=functions(W/'scripts/astra_gazebo/dense_sweep_n04.py')
assert set(f0)==set(f1) and all(f0[k]==f1[k] for k in f0 if k!='main')
replay_path=S/'audits/navigation_n04_dense_inherited_replay_check_r02.json';replay=load(replay_path)
assert replay['pass_'] and replay['full_physical_intervals']==4790 and all(x['byte_identical'] for x in replay['checks'])
old_event=next(x['snapshot'] for x in snapshot['entries'] if x['original']['path'].endswith('/event_actor_contract_n04.py'))
assert functions(old_event)['plugin_contract']==functions(W/'scripts/astra_gazebo/event_actor_contract_n04.py')['plugin_contract']
for name in ('intervals.jsonl','endpoints.csv'):
 before=S/'runs/astra_navigation_n03_dwa_common_route/sweep_evaluation'/name;after=S/'audits/navigation_n04_dense_inherited_replay_r02'/name
 assert before.read_bytes()==after.read_bytes()
actor_path=S/'audits/navigation_n04_actor_independent/review.json';actor=load(actor_path)
assert actor['pass'] is True and actor['independent_review'] is True
assert actor['reviewed_library_sha256']=='8ca7406469a1eefe34a8be837d003c6345e5e04dcf81eed3476085c68e6c0fdf'
for x in actor['resources']:evidence.read_bound(x)
scene_path=S/'audits/navigation_n04_scenes_independent/review.json';scene=load(scene_path)
assert scene.get('pass') is True or scene.get('status','').endswith('_pass'),scene.get('status')
engine=copy.deepcopy(old['engine_module_bindings']);assert not any(x['role']=='actor_binaries' for x in engine)
engine.append(dict(bind(W/'devel/lib/libastra_event_actor_n04_plugin.so'),role='actor_binaries'))
assert engine[-1]['sha256']==actor['reviewed_library_sha256']
assert set(evidence.ELF_ROLES.values())<=set(x['role'] for x in engine)
assert len({x['role'] for x in engine})==len(engine)
facts=passive.native_review_facts(old);assert facts['native_reference_pass'] and facts['upstream_source_review_pass']
for item in engine+old['engine_source_bindings']:evidence.read_bound(item)
audit={'schema':'astra_navigation_n04_independent_incremental_source_audit_v1','status':'pass','blocking_findings':[],
 'reviewed_root_source_freeze':bind(freeze_path),'prepared':bind(prepared_path),'actual_launch_preflight':bind(preflight_path),
 'focused_actual48_and_callback_checks':bind(D/'focused_review.json'),'new_status_boundary_regressions':bind(D/'status_boundary_review.json'),
 'author44_event_tests_independently_rerun':bind(D/'event_contract_tests.log'),
 'findings_fixed_before_release':['N04 closure required_sources previously named N02 helper/collector','missing/future/changed first-active timestamp accepted','three top-level scene-generator imports absent from required runtime source roster'],
 'preserved_negative_reproduction':bind(D/'status_counterexamples_r01.json'),'original_sources_preserved':bind(S/'audits/navigation_n04_root_source_r01_snapshot/manifest.json'),
 'old_dense_nonmain_AST_identical':True,'old4790_intervals_and480_endpoints_byte_identical':True,
 'inherited_dense_replay':bind(replay_path),'replay_final_guard_revision_note':'Replay event source precedes only audit_status guard corrections; parser plugin_contract and all dense nonmain functions unchanged. Not represented as a fresh N04 runtime.',
 'actual_launch_generator_AST_unchanged_after_preflight':True,'runner_revision_equivalence':bind(D/'runner_r02_launch_equivalence.json'),'engine_role_count':len(engine),
 'actor_cross_review':bind(actor_path),'scene_cross_review':bind(scene_path),
 'division':'environment_audit independently reviews root Python; historical_audit independently reviews environment_audit C++/scenes. Root independently qualified inherited coordinator/launch.',
 'no_ROS_or_execution_or_registry_write':True}
write('audit.json',audit)
paths=set(manager.required_resources())
for unit in prepared['units']:paths.update(Path(x['path']) for x in unit['files']+unit['conversion_resources'])
paths.update(Path(x['path']) for x in engine+old['engine_source_bindings']+actor['resources'])
for key in ('native_engine_report','native_engine_records','upstream_source_manifest'):paths.add(Path(old[key]['path']))
# Include reviewer-declared evidence without silently accepting current hashes
# for stale bindings. Both independent components own their final manifests.
for key in ('resources','source_resources'):
 for item in scene.get(key,[]):
  if isinstance(item,dict) and 'path' in item and 'sha256' in item:
   evidence.read_bound(item);paths.add(Path(item['path']))
paths.update([old_path,freeze_path,prepared_path,preflight_path,replay_path,actor_path,scene_path,
 S/'audits/navigation_n02_coordinator_root_independent.json',S/'contracts/passive_q05_navigation_evidence_qualification_r1.json'])
paths.update(Path(x['path']) for x in freeze['files'])
paths.update(Path(row['parameters']['path']) for row in preflight['runs'])
paths.update(p for p in (S/'audits/navigation_n04_root_source_r01_snapshot').iterdir() if p.is_file())
paths.update(p for p in D.iterdir() if p.is_file() and p.name not in ('source_review.json','release_required_resources.json','build_review.log'))
resources=[bind(p) for p in sorted({p.resolve() for p in paths})]
review={k:old[k] for k in ('schema','navigation_lifecycle_authority','robot_pose_velocity_writes','actor_updates',
 'native_engine_report','native_engine_records','upstream_source_manifest','engine_source_bindings','gazebo_callback_order_review','qualified_passive_evidence_prerequisite')}
review.update(stage=manager.NAME,status='independently_reviewed',engine_module_bindings=engine,qualified_passive_evidence_prerequisite=bind(S/'contracts/passive_q05_navigation_evidence_qualification_r1.json'),
 reviewer='/root/environment_audit independent review of root-owned six new Python files, actual prepared48 and expanded parameter output. C++/scene authorship is separate and qualified only by /root/historical_audit independent bound reviews.',
 resources=resources,source_audit=bind(D/'audit.json'),prepared=bind(prepared_path),actual_launch_preflight=bind(preflight_path),
 inherited_source_review=bind(old_path),actor_independent_review=bind(actor_path),scene_independent_review=bind(scene_path),
 scope='48 fresh development units over six existing parent geometries and two repeats, four methods in counterbalanced order; no held-out or final performance claim.',
 writer_review={**old['writer_review'],'sensor_method':'Frozen R2 sensor lifecycle r1 uses no Gazebo truth; common route audit-only, original model/runtime/weights unchanged.',
 'actor':'Only reviewed event actor Begin writes its own static model pose/zero twist; callback queues strict episode-bound coordinator event, 50ms max source-to-Begin, first consuming step dt0. Native robot driver remains force-only.',
 'collector':'Sole fixed phase-checked Gazebo spawn/unpause/pause service gateway; actual registered command/gate/status graph, full planner params bounded RPC, durable error+stop/pause on actor faults; exact paused dense watermark and ELF closure remain mandatory.'},
 runtime_obligations=['Root must bind this review, all listed resources and each prepared fixture.launch/world/URDF/conversion before new registry reservation.',
 'Measure actual event transport/activation/status latching and actor pose Begin update; offline source review does not qualify live actor timing.',
 'Capture actual Gazebo pre/final PID/start/maps including new actor SHA8ca74064 and selected engine roles; baseline move_base PID/plugin classes/loaded ELF must match.',
 'Retain all actual algorithm failures at Tcap22; integrity failure retires remaining units, no retries or seed replacements.',
 'Keep whole-contact/dense 2ms coverage, converted model1e-7 checks, total drain5s and native images as declared.'],
 preserved_limitations=['Six reused parent geometries, not48 independent scenes. Parent actor parameter stream retained; new simulation noise seeds only.',
 'R2 sees exact blocked raster rectangles but does not consume shared full polyline; all current headings are identity and arbitrary SE2 R2 is unsupported.',
 'All new actor event qualification remains pending actual N04 evidence; 50ms cap fixed before measurement.',
 'Static geometry witnesses exclude dynamic actors and actual control tracking; original occlusion sin4 insufficiency retained alongside new quintic static witness.',
 'Engineering sweep is a declared Gazebo algorithm extension with discrete actor Begin jumps, not formal or real continuous physical safety.',
 'Selected mapped ELF/source/native facts cover trusted authorized nodes/services, not adversarial process proof or anonymous graphics memory equality.'],
 execution_authorized=False,seed_allocation_authorized=False,review_does_not_grant_authority=True,
 pending_before_execution=['Root exact release/verification and fresh48 registry rows; source review itself grants no authority.'])
write('source_review.json',review)
write('release_required_resources.json',{'schema':'astra_navigation_review_resources_v1','stage':manager.NAME,'resources':resources+[bind(D/'source_review.json')],'execution_authorized':False})
print(json.dumps({'review':bind(D/'source_review.json'),'resources':len(resources),'active_runtime_paths':len(manager.required_resources()),'engine_roles':len(engine),'no_blocking_findings':True},indent=2))
