"""N06 r02 independent focused tests, temporary files only. Never execute release."""
import copy,hashlib,json,sys
from pathlib import Path
from unittest.mock import patch
A=Path(__file__).resolve().parent;S=A.parents[2];P=S/'workspace/scripts/astra_gazebo';sys.path[:0]=[str(P),str(P/'tests')]
import test_navigation_n06_manager_r02 as author
import navigation_n06_integrity_r02 as q,navigation_n06_admin_plan_r02 as admin
import dense_sweep_n05 as dense
checks=[];gaps=[]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def b(p):return {'path':str(Path(p).resolve()),'sha256':sha(p)}
for version,expected,count in [('r01','50a5af06ff0be1ddd7d94ae17af3b108c41a213a855be51a7d4656cf9b3f8c8b',352),('r02','76bd9a6256f927668eb46a5a6e366c6ab3735f4c4fe190480402c37d21dc0415',733)]:
 path=S/'audits'/('navigation_n06_manager_'+version)/'source_evidence_manifest.json';assert sha(path)==expected
 value=json.loads(path.read_text());roster={x['path']:x for group in ('owned_sources','read_only_dependencies','author_evidence') for x in value[group]};assert len(roster)==count
 assert all(sha(x['path'])==x['sha256'] for x in roster.values());checks.append({'case':version+'_whole_roster_unchanged','resources':count,'pass':True})
for name,old,new in [('manager','run_navigation_n06_manager.py','run_navigation_n06_manager_r02.py'),('process','navigation_n06_process.py','navigation_n06_process_r02.py')]:
 original=(P/old).read_text().replace('navigation_n06_integrity','navigation_n06_integrity_r02').replace('navigation_n06_process','navigation_n06_process_r02')
 assert original==(P/new).read_text();checks.append({'case':name+'_only_dependency_rename','pass':True})
# Resource drift before/after a partial replacement, after last table, and late progress.
for where in ('before_commit','seed_registry.csv','run_manifest.csv','EXPERIMENTS.csv','final_progress'):
 f=author.AdministrationTests();f.setUp()
 try:
  plan,review=f.plan();victim=f.stage/'receipt1.json';original_bytes=victim.read_bytes();original=q.publish;changed=[False]
  if where=='before_commit':victim.write_bytes(b'{}\n');changed[0]=True
  def publish(path,*args,**kw):
   result=original(path,*args,**kw)
   target=(Path(path)==f.stage/where and kw.get('replace')) or (where=='final_progress' and Path(path).name=='progress.json' and (f.stage/'EXPERIMENTS.csv').read_bytes()!=f.before['EXPERIMENTS.csv'])
   if target and not changed[0]:victim.write_bytes(b'{}\n');changed[0]=True
   return result
  rejected=False
  with patch.object(q,'publish',side_effect=publish):
   try:admin.apply_reviewed_plan(plan,review)
   except ValueError:rejected=True
  assert rejected and changed[0] and not (f.stage/'transaction/completion.json').exists()
  if where=='seed_registry.csv':assert (f.stage/'run_manifest.csv').read_bytes()==f.before['run_manifest.csv']
  victim.write_bytes(original_bytes);admin.apply_reviewed_plan(plan,review)
  snapshot={str(p):(p.read_bytes(),p.stat().st_ino,p.stat().st_mtime_ns) for p in f.stage.rglob('*') if p.is_file()};admin.apply_reviewed_plan(plan,review)
  assert snapshot=={str(p):(p.read_bytes(),p.stat().st_ino,p.stat().st_mtime_ns) for p in f.stage.rglob('*') if p.is_file()}
  checks.append({'case':'bound_input_drift_'+where,'pass':True,'completion_absent_until_exact_restore':True,'recovery_and_noop':True})
 finally:f.tearDown()
# Original wrong stage/error/missing report semantic failures are now rejected.
for key,bad in [('stage','foreign'),('error','nonempty failure'),('assessment',None),('integrity_pass',1),('seed',True),('roslaunch_exit_code',True),('task_outcome','unknown')]:
 f=author.AdministrationTests();f.setUp()
 try:
  receipts=copy.deepcopy(f.receipts);receipts[f.rows[0]['run_id']][key]=bad
  try:admin.projected_tables(f.before,f.state,receipts,f.sha);rejected=False
  except (ValueError,KeyError):rejected=True
  assert rejected;checks.append({'case':'receipt_'+key+'_reject','pass':True})
 finally:f.tearDown()
# Remaining closed-receipt bypasses within the same previously reported boundary.
f=author.AdministrationTests();f.setUp()
try:
 receipts=copy.deepcopy(f.receipts);first=receipts[f.rows[0]['run_id']]
 first['administrative_metadata']={'run_id':'historical_duplicate','receipt_path':'/tmp/unbound_receipt.json','report_path':'/tmp/unbound_report.json','bindings_sha256':'forged'}
 projected=admin.projected_tables(f.before,f.state,receipts,f.sha)
 head,actual=admin.records(projected['run_manifest.csv'],admin.FIELDS['run_manifest.csv'])
 added=[r for r,_ in actual][-8:]
 assert added[0]['run_id']=='historical_duplicate' and added[0]['receipt_path']=='/tmp/unbound_receipt.json'
 gaps.append({'id':'receipt_administrative_identity_and_path_override','accepted':True,'projected_first_run_id':added[0]['run_id'],'expected_first_run_id':f.rows[0]['run_id'],'unbound_receipt_path':added[0]['receipt_path'],'unbound_report_path_in_experiments':b'/tmp/unbound_report.json' in projected['EXPERIMENTS.csv'],'mechanism':'arbitrary administrative_metadata overwrites run_id and paths without validation'})
 first=copy.deepcopy(f.receipts[f.rows[0]['run_id']]);dependencies=q.validate_terminal_receipt(first,f.rows[0],f.sha)
 report=q.json_bound(first['assessment'])
 assert 'source_bindings' not in report and 'sweep_report' not in report
 gaps.append({'id':'assessment_not_canonical_or_source_complete','accepted':True,'assessment_outside_run_root':not Path(first['assessment']['path']).is_relative_to(f.stage) if hasattr(Path,'.is_relative_to') else not str(first['assessment']['path']).startswith(str(f.stage)+'/'),'arbitrary_integrity_check_names':list(report['integrity_checks']),'missing_source_bindings_and_sweep_report':True,'dependency_count':len(dependencies)})
finally:f.tearDown()
# Stop the author's explicit converter stub: three-field synthetic conversion now fails.
f=author.SyntheticClosureTests();f.setUp()
try:
 f.converter.stop();path,digest=f.document()
 try:q.verify_release(path,digest);rejected=False
 except ValueError as err:rejected='converter provenance failed' in str(err)
 assert rejected;checks.append({'case':'r01_three_key_fake_conversion_now_rejected','pass':True,'native_facts_only_stubbed':True})
 # Retain native/runtime structural stand-ins, replace ONLY conversions with all8 actual saved N06 conversions.
 actual=json.loads((S/'contracts/navigation_n06_conversion_r01/bundle.json').read_text());f.documents['conversion_bundle']={k:actual[k] for k in ('schema','stage','units')}
 for unit in actual['units']:
  for item in unit['conversion_resources']:f.add(item['path'])
 path,digest=f.document();out=q.verify_release(path,digest);assert len(out['rows'])==8
 checks.append({'case':'all_eight_actual_saved_N06_conversions_accepted','pass':True,'native_and_other_runtime_standins':True,'no_execution_release_qualification':True})
finally:f.tearDown()
# Record real global indices only as read-only observations, not immutable dependencies.
globals=[b(S/name) for name in ('seed_registry.csv','run_manifest.csv','EXPERIMENTS.csv')]
result={'scope':'focused offline r02 review; temporary filesystem only','checks':checks,'remaining_gaps':gaps,'new_failure_count':len(gaps),'global_csv_observations':globals,'ROS_started':False,'release_executed':False,'seed_registered':False}
(A/'focused_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'passed_focused_groups':len(checks),'remaining_gaps':gaps},indent=2))
