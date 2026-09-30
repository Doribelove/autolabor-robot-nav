#!/usr/bin/env python3
"""Freeze only the N06 author implementation/evidence; no execution authority."""
import ast,difflib,hashlib,json,os,re,shutil
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo'
def bound(p):
 p=Path(p).resolve();b=p.read_bytes();return {'path':str(p),'sha256':hashlib.sha256(b).hexdigest(),'bytes':len(b)}
def short(p):return {k:v for k,v in bound(p).items() if k!='bytes'}
def write(p,v):
 with p.open('x') as f:json.dump(v,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
if (A/'implementation_report.json').exists():raise RuntimeError('author freeze already exists; never overwrite')
source_names=['run_navigation_n06_manager.py','navigation_n06_integrity.py','navigation_n06_process.py','navigation_n06_admin_plan.py','tests/test_navigation_n06_manager.py']
owned=[P/name for name in source_names]
log=(A/'author_tests_final.log').read_text();match=re.search(r'Ran (\d+) tests in ([0-9.]+)s\n\nOK',log)
if not match or int(match[1])!=48:raise RuntimeError('final 48 tests not passed')
readonly=json.loads((A/'readonly_dependency_authority_audit.json').read_text())
for key in ('preparation_changed','local_imports_outside_release_required_roster','observed_workspace_imports_outside_roster','actual_registry_matches','execution_artifacts_already_present'):
 if readonly[key]:raise RuntimeError('readonly failed: '+key)
snap=A/'frozen_sources';snap.mkdir()
for p in owned:
 target=snap/p.name;target.write_bytes(p.read_bytes())
 old=A/'draft_r00'/p.name
 if old.exists():(A/(p.name+'.draft_r00_to_final.diff')).write_text(''.join(difflib.unified_diff(old.read_text().splitlines(True),p.read_text().splitlines(True),fromfile=str(old),tofile=str(p))))
prep_path=S/'audits/navigation_n06_preparation/source_evidence_manifest.json';prep=json.loads(prep_path.read_text())
refs=[prep_path,S/'audits/navigation_n06_preparation/implementation_report.json',S/'audits/navigation_n06_preparation_independent/review.json',
 S/'audits/development_geometry_d04_r05_independent/review.json',S/'audits/navigation_n05_terminal_independent/review.json',
 S/'audits/navigation_n05_admin_projection_independent/r01/review.json',S/'audits/navigation_n05_admin_projection_independent/r03/review.json']
readonly_paths={str(p.resolve()) for p in refs}
readonly_paths.update(item['path'] for group in prep.values() if isinstance(group,list) for item in group)
readonly_paths.update(item['path'] for item in readonly['manager_required_resources'])
readonly_paths.update(readonly['observed_workspace_python_imports'])
readonly_paths.add(str(Path('/usr/bin/gzserver').resolve()))
readonly_paths-=set(str(p.resolve()) for p in owned)
for group in prep.values():
 if isinstance(group,list):
  for item in group:
   if short(item['path'])['sha256']!=item['sha256']:raise RuntimeError('frozen preparation changed')
observations={'schema':'astra_navigation_n06_author_global_observation_v1','table_bindings':readonly['original_csv_observations'],
 'unchanged_since_readonly_audit':all(short(item['path'])['sha256']==item['sha256'] for item in readonly['original_csv_observations']),
 'no_csv_writes_performed':True,'proposed_seed_registry_matches':readonly['actual_registry_matches'],
 'gzserver_alias':{'path':'/usr/bin/gzserver','link_text':os.readlink('/usr/bin/gzserver'),'canonical_target':short(Path('/usr/bin/gzserver').resolve())}}
if not observations['unchanged_since_readonly_audit']:raise RuntimeError('global table changed concurrently; preserve and re-observe')
write(A/'final_authority_observation.json',observations)
tests=ast.parse(owned[-1].read_text());test_names=[node.name for node in ast.walk(tests) if isinstance(node,ast.FunctionDef) and node.name.startswith('test_')]
negative={
 'frozen_attempt03_fixture_failure':{'log':short(A/'author_tests_attempt03.log'),'source':short(A/'test_fixture_attempt03.py'),
  'failed_tests':['test_closure_failure_retained_and_no_assessment','test_integrity_valid_timeout_is_task_result'],
  'reason':'test-only fake verified-data dictionary omitted review; real verify_release returns review; corrected fixture and retained original'},
 'readonly_import_gap':{'log':short(A/'readonly_audit_attempt01.log'),'source':short(A/'inspect_readonly_attempt01.py'),
  'reason':'observed virtualenv startup modules absent from initial mandatory support roster; final requires those modules, pyvenv.cfg and three .pth files'},
 'repaired_draft_findings':['supervisor spawn failure needs retirement with no invented child wait status',
  'valid negative terminal worker return 3 must preserve original terminal ledger/reason',
  'cleanup vanished/zombie race must be benign; alive unverifiable must fail',
  'offline assessor child needs same nonce ownership plus separate final/supervisor cleanup records',
  'administrative targets must be exact canonical stage paths, not only matching basenames',
  'resolved Python binary alone cannot identify virtualenv; require prefix and disabled user site',
  'gzserver symlink alias cannot satisfy canonical-file basename test; require actual resolved target binding'],
 'test_names':test_names,
 'fault_coverage':['duplicate/nonfinite JSON, closed schema, strict bool/int and authority',
  'all real frozen prepared resource bindings plus synthetic native/other-runtime stand-ins; exact IDs, order, type and predecessor changes',
  'missing/duplicate/drifting resources, permanent session invalidation, canonical gzserver actual symlink',
  'whole registry baseline+exact8 reservation, foreign seed/run, no global writes',
  'real disposable process starttick/nonce/pidfd cleanup, assessment environment restoration, vanished and uncertain identity',
  'fake subprocess spawn/nonzero/cleanup/closure faults, valid task timeout, reportNone',
  'durable event/pointer crash, broken chain, worker missing receipt, no-allocation default, system-prefix alias rejection',
  'three CSV partial replacement and before-progress crashes, deterministic roll-forward, byte/inode/mtime idempotence, foreign third version',
  'wrong-root targets, incomplete review, mutated snapshot/receipt, ledger totals/outcome mismatch, history/duplicate preservation']}
write(A/'retained_failures_and_test_scope.json',negative)
manifest={'schema':'astra_navigation_n06_manager_author_source_evidence_v1','stage':'astra_navigation_n06_lookahead_static_pilot',
 'owned_sources':[bound(p) for p in owned], 'read_only_dependencies':[bound(p) for p in sorted(readonly_paths)],
 'author_evidence':[bound(p) for p in sorted(A.rglob('*')) if p.is_file()],
 'execution_authorized':False,'scope':'Author source/evidence freeze only; future exact source/native/runtime/conversion release remains absent.'}
write(A/'source_evidence_manifest.json',manifest)
report={'schema':'astra_navigation_n06_manager_author_report_v1','stage':'astra_navigation_n06_lookahead_static_pilot',
 'status':'offline_one_shot_manager_author_component_pass_independent_review_pending','independent_review':False,
 'frozen_output_root':str(A),'source_evidence_manifest':short(A/'source_evidence_manifest.json'),
 'sources':[short(p) for p in owned], 'api':short(A/'API.md'),
 'validation':{'tests_passed':int(match[1]),'wall_seconds':float(match[2]),'log':short(A/'author_tests_final.log'),
  'tests_use_temporary_synthetic_authority_only':True,'native_runtime_standins_never_executed':True,
  'actual_gzserver_binary_read_only_not_launched':True,'disposable_python_process_tests':True,
  'prepared_resources_reverified':len({i['path'] for g in prep.values() if isinstance(g,list) for i in g}),
  'recursive_local_python_modules':len(readonly['local_import_resources']),'observed_workspace_python_modules':len(readonly['observed_workspace_python_imports']),
  'local_and_observed_imports_all_in_required_roster':True,'readonly_audit':short(A/'readonly_dependency_authority_audit.json')},
 'prior_independent_reviews':{'prepared':short(refs[2]),'d04':short(refs[3])},
 'authority':{'execution_authorized':False,'training_authorized':False,'allocated_budget':0,'seeds_registered':0,'release_created':False,'ros_gazebo_started':False,'global_csv_writes':False,'frozen_prepared_inputs_changed':False},
 'retained_failures':short(A/'retained_failures_and_test_scope.json'),
 'future_interfaces':{'source_review':str(S/'audits/navigation_n06_release_review_r01/source_review.json'),
  'runtime_closure':str(S/'audits/navigation_n06_runtime_closure_r01/closure.json'),
  'conversion_bundle':str(S/'contracts/navigation_n06_conversion_r01/bundle.json'),
  'registry_baseline':str(S/'contracts/navigation_n06_registry_baseline_r01.csv'),
  'release':str(S/'contracts/navigation_n06_manager_release_r01.json')},
 'guarantees_within_scope':{'exact_release_only':True,'one_shot_no_retry_resume':True,'durable_consume_before_launch':True,'infra_stop_retires_tail':True,
  'task_failure_not_conflated_with_integrity':True,'per_pid_ancestry_nonce_starttick_and_pidfd':True,'cleanup_races_rechecked':True,
  'launch_and_assessment_child_cleanup':True,'host_exit_unknown':True,'three_csv_joint_atomicity_claimed':False,
  'administration_separate_review_required':True,'before_after_rollforward_idempotence':True},
 'limitations':['No real release/full native-runtime-conversion prerequisite package was created or qualified.',
  'Transitive runtime roster completeness requires independent review; required root paths/hashes do not prove complete loader inventory.',
  'No ROS/Gazebo or live physical/process-drain qualification; later collector mapped ELF and all frozen evidence gates remain mandatory.',
  'Same-user hostile mutation, immutable process memory, kernel/filesystem disaster recovery and outer-host exit are outside proven scope.',
  'Unobserved live process or uncertain ownership stays an integrity failure; disk/supervisor failure may leave closure pending.',
  'Three original CSV paths publish sequentially, require sole writer and deterministic reviewed roll-forward; no joint atomic visibility.',
  'N05 outer143 mechanism remains unknown; new process design does not retrospectively explain it.']}
write(A/'implementation_report.json',report)
checks=[]
for group in ('owned_sources','read_only_dependencies','author_evidence'):
 for item in manifest[group]:
  if short(item['path'])['sha256']!=item['sha256']:raise RuntimeError('postwrite source changed: '+item['path'])
  checks.append(item['path'])
write(A/'freeze_receipt.json',{'schema':'astra_navigation_n06_author_freeze_receipt_v1','report':short(A/'implementation_report.json'),
 'manifest':short(A/'source_evidence_manifest.json'),'postwrite_resources_checked':len(checks),'postwrite_unique_resources':len(set(checks)),
 'authority':False,'all_bound_bytes_match':True})
print(json.dumps({'report':short(A/'implementation_report.json'),'manifest':short(A/'source_evidence_manifest.json'),'unique_resources':len(set(checks)),'tests':int(match[1])},indent=2))
