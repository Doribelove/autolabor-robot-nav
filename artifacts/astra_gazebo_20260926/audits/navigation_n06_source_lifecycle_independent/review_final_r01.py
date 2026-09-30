"""Independent final N06 source review. All release-shaped inputs stay in memory."""
import copy,hashlib,json,os,stat,sys,time
from pathlib import Path
from unittest.mock import patch
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo';sys.path.insert(0,str(P))
import navigation_n06_integrity_r05 as q
import navigation_n06_environment_r05 as environment
import run_passive_sweep_q05 as passive
SUFFIXES={'.py','.launch','.xacro','.cpp','.h','.yaml','.xml'}
checks=[]
def bind(p):
 p=Path(p).resolve();h=hashlib.sha256()
 with p.open('rb') as f:
  for chunk in iter(lambda:f.read(4*1024*1024),b''):h.update(chunk)
 return {'path':str(p),'sha256':h.hexdigest()}
def read(row):
 assert type(row)is dict and {'path','sha256'}<=set(row)
 p=Path(row['path']);assert p.is_absolute() and str(p.resolve())==str(p) and p.is_file() and not p.is_symlink()
 assert bind(p)=={k:row[k] for k in ('path','sha256')},str(p)
 return json.loads(p.read_bytes())
def write(name,value):
 path=A/name
 with path.open('x') as f:json.dump(value,f,ensure_ascii=False,indent=2,sort_keys=True);f.write('\n')
 return bind(path)
def add(name,**kw):checks.append(dict(name=name,passed=True,**kw))
roster_binding={'path':str(S/'audits/navigation_n06_candidate_roster_r05_r02/resources.json'),'sha256':'232b9352ea34474c5179ffdee3304a651d0eba96031b2cb823277f0f264855c9'}
runtime_binding={'path':str(S/'audits/navigation_n06_runtime_reviewed_r01/closure.json'),'sha256':'48986889780c681a87cacc20cf53d22e01e967096b02354732e43fc8a3c50c36'}
qualification_binding={'path':str(S/'audits/navigation_n06_candidate_roster_preparation_r02/qualification_canonical.json'),'sha256':'d4f462ee14492bf671716571108047af9122eda98463b9d6def2f25488b712b7'}
roster=read(roster_binding);runtime=read(runtime_binding);qualification=read(qualification_binding)
assert roster['stage']==q.NAME and roster['execution_authorized'] is False and roster['canonical_release_created'] is False
resources=roster['resources'];by_path={r['path']:r for r in resources}
assert len(resources)==len(by_path)==roster['resource_count']==5656
total_bytes=0
for row in resources:
 p=Path(row['path']);assert p.is_absolute() and str(p.resolve())==str(p) and not p.is_symlink() and p.is_file()
 assert bind(p)==row,row['path'];total_bytes+=p.stat().st_size
source_rows=[r for r in resources if Path(r['path']).suffix in SUFFIXES]
assert source_rows==roster['source_config_roster'] and len(source_rows)==roster['source_config_count']==2456
assert not {str(S/name) for name in ('seed_registry.csv','run_manifest.csv','EXPERIMENTS.csv')} & set(by_path)
add('5656_exact_canonical_unique_resources_and_2456_source_config_paths_rehashed',resource_count=5656,source_count=2456,total_file_bytes=total_bytes)
baseline=Path(roster['registry_baseline']['path']);assert bind(baseline)==roster['registry_baseline'] and baseline.read_bytes()==(S/'seed_registry.csv').read_bytes()
rows=q.prepared_api.schedule();q.reserved_registry_bytes(baseline.read_bytes(),'f'*64,rows)
assert not any((S/'runs'/row['run_id']).exists() for row in rows) and not (S/'queue/navigation_n06_ledger.json').exists()
add('immutable_registry_snapshot_fresh_and_eight_proposed_identities_unallocated_at_review')

author=read(runtime['author_closure']);disk_review=read(runtime['independent_review_report'])
assert runtime['independent_review_report']['sha256']=='9154b5d8a145d747b5b15d15505a9c693ad14833637c05a1424ff9cb1e5660e7'
assert disk_review['status']=='independent_reviewed_disk_dependency_closure_pass' and disk_review['disk_dependency_closure_qualified'] is True
assert disk_review['author_closure']==runtime['author_closure'] and disk_review['independent_resources']==runtime['independent_review_manifest']
allowed={'status','independent_review','author_closure','independent_review_report','independent_review_manifest','author_report','author_manifest','disk_dependency_closure_qualified','runtime_ready','reviewed_envelope_limits','future_live_check_boundary'}
changed={k for k in runtime if k not in author or runtime[k]!=author[k]}
assert changed==allowed and set(author)<=set(runtime)
assert runtime['resources']==author['resources'] and len(runtime['resources'])==5217
assert runtime['execution_environment']==author['execution_environment'] and environment.validate_spec(runtime['execution_environment'])
assert runtime['status']=='reviewed_disk_dependency_closure' and runtime['runtime_ready'] is False and runtime['execution_authorized'] is False and runtime['live_mapped_elf_observed'] is False
assert all(by_path.get(r['path'])==r for r in runtime['resources'])
for key in ('author_closure','independent_review_report','independent_review_manifest','author_report','author_manifest','parent_closure','prior_independent_negative_review','export_input_supplement'):
 if key in runtime:assert by_path.get(runtime[key]['path'])==runtime[key];read(runtime[key])
assert runtime['reviewed_envelope_limits']==disk_review['limitations']
boundary=runtime['future_live_check_boundary']
assert boundary['gazeborc_immutable_absence_guarantee']is False and boundary['current_manager_dedicated_gazeborc_absence_gate']is False and boundary['native_SystemPaths_plugin_executed']is False
observed_absent=not Path(boundary['gazeborc_path']).exists();assert observed_absent
add('reviewed_envelope_exact_disk_pass_lineage_no_runtime_semantics_change',original_resources=5217,changed_envelope_fields=sorted(changed),gazeborc_current_absence_only=True,dedicated_live_absence_gate=False)

original_binding={'path':str(S/'contracts/passive_q05_navigation_evidence_qualification.json'),'sha256':'deb76315ff660eba1b9ffa9a2e73dcc20d09713f0c6b13def8d8ee9f65d65fba'}
original=read(original_binding);restored=copy.deepcopy(qualification)
old=original['evidence_bindings'][0];new=qualification['evidence_bindings'][0]
assert old['path']=='artifacts/astra_gazebo_20260926/audits/passive_q05_qualify.py' and new['path']==str((S.parents[1]/old['path']).resolve()) and new['sha256']==old['sha256']
restored['evidence_bindings'][0]['path']=old['path'];assert q.same_typed(restored,original)
assert len(original['evidence_bindings'])==len(qualification['evidence_bindings'])==30
for row in qualification['evidence_bindings']:assert by_path.get(row['path'])==row and bind(row['path'])==row
add('historical_Q05_only_one_path_canonicalized_no_qualification_or_evidence_hash_change',evidence_bindings=30,json_pointer='/evidence_bindings/0/path')

source=read(bind(A/'source_checks.json'));assert len(source['checks'])==9 and all(x['passed'] for x in source['checks'])
native=source['native_reference'];assert passive.native_review_facts(native)['native_reference_pass'] and passive.native_review_facts(native)['upstream_source_review_pass']
for row in native['engine_module_bindings']+native['engine_source_bindings']+[native[k] for k in ('native_engine_report','native_engine_records','upstream_source_manifest')]:
 assert by_path.get(row['path'])=={k:row[k] for k in ('path','sha256')}
assert len(native['engine_module_bindings'])==14 and all(r['role']!='actor_binaries' for r in native['engine_module_bindings'])
assert all(any(z['path']==r['path'] and z['sha256']==r['sha256'] for z in roster['inherited_native_fields_for_independent_revalidation']['engine_module_bindings']) for r in native['engine_module_bindings'])
add('14_active_nonactor_native_roles_and_source_reference_facts_bound_static_actor_exclusion',actor_library_in_roster_but_not_active_native_role=True)
manager_review=read(bind(S/'audits/navigation_n06_manager_independent/r05/review.json'))
assert manager_review['manager_component_review_pass']is True and manager_review['reviewed_manager_sha256']==bind(P/'run_navigation_n06_manager_r05.py')['sha256']
for path in q.required_manager_paths():assert by_path.get(str(path.resolve()))==bind(path)
add('r05_manager_and_first_import_boundary_exact_frozen_sources_in_roster')

scope=read(bind(S/'audits/navigation_n06_source_lifecycle_scope/required_source_bindings.json'))
required_scope=[]
for group in ('direct_runtime_and_preparation_python','recursive_static_local_import_superset','launch_robot_environment_build_context','custom_native_sources','inherited_native_reference_candidates_excluding_actor','prepared_unit_inputs'):
 for row in scope[group]:
  assert by_path.get(row['path'])=={k:row[k] for k in ('path','sha256')},(group,row['path']);required_scope.append(row['path'])
add('prior_scope_actual_active_sources_closed_by_final_roster_without_stale_r03_status',unique_scope_paths=len(set(required_scope)))

global_observation=[]
for name in ('seed_registry.csv','run_manifest.csv','EXPERIMENTS.csv'):
 p=S/name;st=p.stat();global_observation.append(dict(bind(p),inode=st.st_ino,mtime_ns=st.st_mtime_ns))
evidence={'schema':'astra_n06_final_source_lifecycle_independent_evidence_v1','stage':q.NAME,'checks':checks,'static_source_checks':bind(A/'source_checks.json'),'candidate_roster':roster_binding,'reviewed_runtime':runtime_binding,'disk_independent_review':runtime['independent_review_report'],'manager_independent_review':bind(S/'audits/navigation_n06_manager_independent/r05/review.json'),'qualification_projection':{'original':original_binding,'projected':qualification_binding,'only_changed_pointer':'/evidence_bindings/0/path'},'observed_global_snapshots':global_observation,'gazeborc_current_absence_only':{'path':boundary['gazeborc_path'],'currently_absent':observed_absent,'live_verified':False,'immutable_absence_guarantee':False,'dedicated_manager_gate':False},'no_live_runtime_or_physics_qualification':True}
evidence_binding=write('final_evidence_r01.json',evidence)

api_resources=dict(by_path)
extras=[Path(__file__).resolve(),A/'check_source.py',A/'source_checks.json',A/'check_source_attempt01.log',A/'final_evidence_r01.json',Path(roster_binding['path'])]
for p in extras:api_resources[str(p)]=bind(p)
review=dict(native,schema='astra_navigation_source_review_v1',stage=q.NAME,status='independently_reviewed',review_revision='N06_r05_manager_r02_disk_final_source_r01',reviewer='historical_audit independent source/lifecycle and final candidate roster review',
 navigation_lifecycle_authority='collector_only_spawn_unpause_pause_fixed_gateway',robot_pose_velocity_writes=[],actor_updates='none_in_exact_eight_static_worlds',resources=sorted(api_resources.values(),key=lambda x:x['path']),
 candidate_roster=roster_binding,candidate_resource_count=5656,candidate_source_config_count=2456,independent_source_evidence=evidence_binding,
 manager_independent_review=bind(S/'audits/navigation_n06_manager_independent/r05/review.json'),runtime_independent_review=runtime['independent_review_report'],reviewed_runtime=runtime_binding,
 preparation_independent_review=bind(S/'audits/navigation_n06_preparation_independent/review.json'),conversion_independent_review=bind(S/'audits/navigation_n06_conversion_independent/review.json'),
 inherited_native_source_review_reference_only=bind(S/'audits/navigation_n05_root_release/source_review.json'),
 scope='Exact eight proposed N06 static development launches; active method factor only 0.7 to 1.2 m lookahead. Source/config path+SHA coverage includes all frozen candidate disk Python/config dependencies. Review of installed dependencies relies on independently inspected loader/import lineage; not every third-party source line is re-audited.',
 retained_limits=['No N06 release, reservation, runtime process, closed-loop result or safety/performance promotion is created by this source review.','GAZEBORC and unused-export filename absence are current disk observations only; no immutable absence or dedicated live manager absence gate.','Restricted outer clean entry, exact environment and controlled local sole-writer workflow are required; no malicious same-account/OS proof.','Future runs still require actual pre/final mapped-ELF, converted-model correspondence, source coverage/contact/geometry and terminal assessment.','Static-only exact N06 population: old N05 actor native library may remain in transitive roster but is not an active role or new dynamic qualification.','Method command generation stamps remain unknown for headerless Twist; source measurements, local publication and receipt clocks are distinct.','Native Euler integration and graphics runtime exception inherit versioned source/binary/reference evidence, not a source-to-binary reproducibility or anonymous code-byte theorem.','Physical and task failures are retained; supplementary images cannot suppress them. Malformed/incomplete interrupted assessment remains unresolved rather than promoted.','Four new static development parent pairs only; inherited negative witness and previous negative studies remain unchanged.'],
 source_review_does_not_grant_authority=True,execution_authorized=False,seed_allocation_authorized=False,training_authorized=False,formal_result=False)
source_path=A/'source_review.json';source_bytes=q.json_bytes(review);source_binding={'path':str(source_path),'sha256':q.digest(source_bytes)}
# Read-only actual verifier integration. Two synthetic documents exist only in
# memory: the release-shaped header and this not-yet-written final review.
memory_release_path=Path('/tmp/astra_n06_source_review_memory_only_never_written.json');assert not memory_release_path.exists()
release={'schema':'astra_navigation_n06_exact_release_v1','stage':q.NAME,'status':'root_reviewed_release','execution_authorized':True,'training_authorized':False,'real_vehicle_authorized':False,'maximum_episodes':8,'retry_allowed':False,'resume_allowed':False,
 'prepared':bind(q.prepared_api.PREPARED/'prepared.json'),'schedule':bind(q.prepared_api.PREPARED/'schedule.json'),'preparation_manifest':bind(S/'audits/navigation_n06_preparation/source_evidence_manifest.json'),'source_review':source_binding,'d04_review':bind(S/'audits/development_geometry_d04_r05_independent/review.json'),'passive_geometry_qualification':qualification_binding,'conversion_bundle':bind(S/'contracts/navigation_n06_conversion_r01/bundle.json'),'runtime_closure':runtime_binding,'registry_baseline':roster['registry_baseline'],'predecessor_ledger':bind(S/'queue/navigation_n05_ledger.json'),'resources':review['resources']+[source_binding]}
memory_bytes=q.json_bytes(release);memory_sha=q.digest(memory_bytes);original_read=q.read_bound
def memory_reader(item):
 if item=={'path':str(memory_release_path),'sha256':memory_sha}:return memory_bytes
 if item==source_binding:return source_bytes
 return original_read(item)
started=time.monotonic()
with patch.object(q,'read_bound',side_effect=memory_reader):verified=q.verify_release(memory_release_path,memory_sha)
assert len(verified['rows'])==8 and verified['runtime']['status']=='reviewed_disk_dependency_closure'
assert verified['review']==review and verified['registry_baseline']==baseline.read_bytes()
assert not memory_release_path.exists() and not source_path.exists()
for row in global_observation:
 p=Path(row['path']);st=p.stat();assert bind(p)['sha256']==row['sha256'] and st.st_ino==row['inode'] and st.st_mtime_ns==row['mtime_ns']
with source_path.open('xb') as f:f.write(source_bytes)
assert bind(source_path)==source_binding
interface=write('interface_verification_r01.json',{'schema':'astra_n06_source_review_actual_verifier_memory_fixture_v1','passed':True,'verifier':bind(P/'navigation_n06_integrity_r05.py'),'elapsed_wall_s':time.monotonic()-started,'actual_bound_resources_read':len(release['resources']),'source_review':source_binding,'all_eight_real_contracts_and_conversions_verified':True,'native_and_runtime_stubs_used':False,'only_injected_reads':['unwritten in-memory release-shaped header','same exact source_review bytes before final file write'],'synthetic_authority_flag_is_fixture_only':True,'release_file_written':False,'runtime_spawned':False,'seed_registered':False,'global_CSV_bytes_inode_mtime_unchanged':True})
summary={'schema':'astra_n06_final_source_lifecycle_independent_review_v1','status':'independent_exact_N06_source_lifecycle_and_candidate_roster_review_pass_no_authority','independent_review':True,'source_review':source_binding,'candidate_roster':roster_binding,'candidate_resources_rehashed':5656,'candidate_source_configs_covered':2456,'source_review_resources':len(review['resources']),'active_nonactor_native_roles':14,'final_evidence':evidence_binding,'source_checks':bind(A/'source_checks.json'),'interface_verification':interface,'runtime_scope':'reviewed disk only; no actual N06 mapped-ELF result','execution_authorized':False,'release_created':False,'seeds_registered':0,'ROS_or_Gazebo_started':False,'real_CSV_written':False,'old_bytes_modified':False}
report=write('review_r01.json',summary)
print(json.dumps({'review':report,'source_review':source_binding,'source_resources':len(review['resources']),'status':summary['status']},indent=2))
