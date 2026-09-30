"""R03 focused repair regression; temp FS and immutable source reads only."""
import ast,copy,hashlib,json,sys
from pathlib import Path
from unittest.mock import patch
A=Path(__file__).resolve().parent;S=A.parents[2];P=S/'workspace/scripts/astra_gazebo'
sys.path[:0]=[str(P),str(P/'tests')]
import test_navigation_n06_manager_r03 as t
q=t.q;admin=t.admin;checks=[];mutations=[]
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def add(name,**kw):checks.append(dict(case=name,pass_=True,**kw))
def reject(fn,name):
 try:fn()
 except (ValueError,KeyError,TypeError) as exc:mutations.append({'case':name,'rejected':True,'exception':type(exc).__name__+': '+str(exc)});return
 raise AssertionError('accepted '+name)
for version,count,expected in [('r01',352,'50a5af06ff0be1ddd7d94ae17af3b108c41a213a855be51a7d4656cf9b3f8c8b'),('r02',733,'76bd9a6256f927668eb46a5a6e366c6ab3735f4c4fe190480402c37d21dc0415'),('r03',764,'cb48b726ab31680eb2ee61c2721f5e2ccfa169c49f33ed16ac82409096b3fa87')]:
 p=S/('audits/navigation_n06_manager_'+version)/'source_evidence_manifest.json';assert sha(p)==expected
 m=json.loads(p.read_text());rows={r['path']:r for group in ('owned_sources','read_only_dependencies','author_evidence') for r in m[group]}
 assert len(rows)==count and all(sha(r['path'])==r['sha256'] for r in rows.values());add(version+'_whole_roster_unchanged',resources=count)
for stem in ('run_navigation_n06_manager','navigation_n06_process'):
 old=(P/(stem+'_r02.py')).read_text().replace('navigation_n06_integrity_r02','navigation_n06_integrity_r03').replace('navigation_n06_process_r02','navigation_n06_process_r03')
 assert old==(P/(stem+'_r03.py')).read_text();add(stem+'_only_dependency_rename')

# Exact schema is derived from actual frozen assessor syntax, not test constants.
tree=ast.parse((P/'navigation_evidence_n06.py').read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='assess')
node=next(n for n in ast.walk(fn) if isinstance(n,ast.Assign) and any(isinstance(k,ast.Name) and k.id=='checks' for k in n.targets))
keys={ast.literal_eval(k) for k in node.value.keys};assert keys==q.ASSESSMENT_CHECKS and len(keys)==9
node=next(n for n in ast.walk(fn) if isinstance(n,ast.Assign) and any(isinstance(k,ast.Name) and k.id=='source_files' for k in n.targets))
sources=ast.literal_eval(node.value.generators[0].iter);assert set(sources)==set(q.ASSESSMENT_SOURCES) and len(sources)==6
tree=ast.parse((P/'dense_sweep_n05.py').read_text());fn=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='main')
node=next(n for n in ast.walk(fn) if isinstance(n,ast.Assign) and any(isinstance(k,ast.Name) and k.id=='sources' for k in n.targets))
assert len(node.value.elts)==11
add('actual_assessor_9_checks_6_sources_dense_11_sources_match',conditional_extra_R2_check_inactive_in_N06=True)

f=t.ReceiptSemanticsTests();f.setUp()
try:
 row=q.prepared_api.schedule()[0];receipt=t.receipt(row);root=q.STAGE/'runs'/row['run_id'];base=q.json_bound(receipt['assessment']);sweep=q.json_bound(base['sweep_report'])
 originals={p:p.read_bytes() for p in root.rglob('*') if p.is_file()}
 def restore():
  for p,data in originals.items():p.write_bytes(data)
 def save_report(report,r):
  p=root/'assessment.json';p.write_bytes(q.json_bytes(report));r['assessment']=q.binding(p);return r
 def validate(r):return q.validate_terminal_receipt(r,row,'1'*64)
 deps=validate(receipt);assert len(deps)==20
 assert {str(root/name) for name in q.ASSESSMENT_SOURCES}|{str(root/'process.json'),str(root/'assessment.json'),str(root/'sweep_evaluation/report.json')} <= {r['path'] for r in deps}
 add('canonical_positive_complete_source_consistency_fixture',dependency_occurrences=len(deps),synthetic_not_physics=True)
 for key in q.METADATA_FIELDS:
  r=copy.deepcopy(receipt);del r['administrative_metadata'][key];reject(lambda:validate(r),'missing_metadata_'+key)
 for key,value in [('run_id','historical_duplicate'),('receipt_path','/tmp/not_receipt'),('report_path','/tmp/not_report'),('raw_path','/tmp/not_raw'),('heartbeat_path','/tmp/not_heartbeat'),('cwd','/tmp'),('bindings_sha256','9'*64),('code_sha256','9'*64),('config_sha256','9'*64),('model_sha256','9'*64),('exit_code',True),('pid',True),('process_group',True),('command_json','["/bin/false"]')]:
  r=copy.deepcopy(receipt);r['administrative_metadata'][key]=value;reject(lambda:validate(r),'metadata_'+key)
 add('closed_metadata_and_r02_identity_paths_override_rejected',cases=29)
 p=q.STAGE/'foreign_assessment.json';p.write_bytes(q.json_bytes(base));r=copy.deepcopy(receipt);r['assessment']=q.binding(p);reject(lambda:validate(r),'noncanonical_assessment')
 for key in q.ASSESSMENT_CHECKS:
  r=copy.deepcopy(receipt);report=copy.deepcopy(base);del report['integrity_checks'][key];save_report(report,r);reject(lambda:validate(r),'missing_check_'+key)
 report=copy.deepcopy(base);report['integrity_checks']={'synthetic_check':True};r=save_report(report,copy.deepcopy(receipt));reject(lambda:validate(r),'r02_minimal_arbitrary_check')
 for key in q.ASSESSMENT_SOURCES:
  report=copy.deepcopy(base);del report['source_bindings'][key];r=save_report(report,copy.deepcopy(receipt));reject(lambda:validate(r),'missing_source_'+key)
 for key in ('source_bindings','sweep_report'):
  report=copy.deepcopy(base);del report[key];r=save_report(report,copy.deepcopy(receipt));reject(lambda:validate(r),'missing_'+key)
 add('canonical_full_assessment_r02_minimal_bypass_rejected',cases=19)
 for index,source in enumerate(sweep['source_bindings']):
  value=copy.deepcopy(sweep);del value['source_bindings'][index];p=root/'sweep_evaluation/report.json';p.write_bytes(q.json_bytes(value))
  report=copy.deepcopy(base);report['sweep_report']=q.binding(p);r=save_report(report,copy.deepcopy(receipt));reject(lambda:validate(r),'missing_sweep_'+Path(source['path']).name)
 restore();add('every_one_of_11_sweep_sources_required',cases=11)
 for name in ('world.sdf','robot_description_runtime.xml'):
  restore();p=root/name;p.write_bytes(b'synthetic changed frozen model')
  report=copy.deepcopy(base);report['source_bindings'][name]=q.binding(p)
  value=copy.deepcopy(sweep)
  for index,item in enumerate(value['source_bindings']):
   if item['path']==str(p):value['source_bindings'][index]=q.binding(p)
  sp=root/'sweep_evaluation/report.json';sp.write_bytes(q.json_bytes(value));report['sweep_report']=q.binding(sp)
  r=save_report(report,copy.deepcopy(receipt));reject(lambda:validate(r),'rebound_model_'+name)
 restore();add('world_URDF_rebinding_cannot_change_prepared_model')
 for name,key,bad in [('collector_report.json','pass',False),('collector_raw.json','seed',True),('run_closure.json','run_id','foreign')]:
  restore();p=root/name;value=json.loads(p.read_text());value[key]=bad;p.write_bytes(q.json_bytes(value))
  report=copy.deepcopy(base);report['source_bindings'][name]=q.binding(p)
  if name=='run_closure.json':
   value=copy.deepcopy(sweep)
   for index,item in enumerate(value['source_bindings']):
    if item['path']==str(p):value['source_bindings'][index]=q.binding(p)
   sp=root/'sweep_evaluation/report.json';sp.write_bytes(q.json_bytes(value));report['sweep_report']=q.binding(sp)
  r=save_report(report,copy.deepcopy(receipt));reject(lambda:validate(r),'collector_or_closure_'+name)
 restore();add('collector_and_closure_actual_identity_join_rejected')
 report=copy.deepcopy(base);report['capture_integrity_pass']=False;report['integrity_checks']['passive_source_geometry_complete']=False
 r=save_report(report,copy.deepcopy(receipt));r['integrity_pass']=False;assert validate(r)
 restore();r=copy.deepcopy(receipt);r.update(integrity_pass=False,error='late cleanup failure');assert validate(r)
 add('negative_assessment_and_later_error_are_not_promoted')
 # Separate empty run for prelaunch failure; prior successful artifacts absent.
 r=t.receipt(q.prepared_api.schedule()[1],False);assert q.validate_terminal_receipt(r,q.prepared_api.schedule()[1],'1'*64)==[]
 add('no_assessment_nonempty_error_before_launch_remains_accepted')
finally:f.tearDown()

# The original r02 projection override now fails before appended rows can exist.
f=t.AdministrationTests();f.setUp()
try:
 receipts=copy.deepcopy(f.receipts);receipts[f.rows[0]['run_id']]['administrative_metadata']['run_id']='historical_duplicate'
 reject(lambda:admin.projected_tables(f.before,f.state,receipts,f.sha),'projection_identity_override')
 result=admin.projected_tables(f.before,f.state,f.receipts,f.sha)
 _,rows=admin.records(result['run_manifest.csv'],admin.FIELDS['run_manifest.csv']);assert [r['run_id'] for r,_ in rows][-8:]==[r['run_id'] for r in f.rows]
 add('actual_projected_rows_keep_all_eight_schedule_identities')
 plan,review=f.plan();victim=f.stage/'runs'/f.rows[0]['run_id']/'process.json';original=victim.read_bytes();publish=q.publish;changed=[False]
 def inject(path,*args,**kw):
  result=publish(path,*args,**kw)
  if Path(path)==f.stage/'seed_registry.csv' and kw.get('replace') and not changed[0]:victim.write_bytes(b'{}');changed[0]=True
  return result
 with patch.object(q,'publish',side_effect=inject):reject(lambda:admin.apply_reviewed_plan(plan,review),'new_process_dependency_drift_after_first_replace')
 assert changed[0] and (f.stage/'run_manifest.csv').read_bytes()==f.before['run_manifest.csv'] and not (f.stage/'transaction/completion.json').exists()
 victim.write_bytes(original);admin.apply_reviewed_plan(plan,review)
 before={str(p):(p.read_bytes(),p.stat().st_ino,p.stat().st_mtime_ns) for p in f.stage.rglob('*') if p.is_file()};admin.apply_reviewed_plan(plan,review)
 assert before=={str(p):(p.read_bytes(),p.stat().st_ino,p.stat().st_mtime_ns) for p in f.stage.rglob('*') if p.is_file()}
 add('new_dependency_drift_recovery_and_completion_noop')
finally:f.tearDown()

result={'schema':'astra_n06_r03_independent_focused_checks_v1','checks':checks,'rejected_mutations':mutations,
 'new_blocker':json.loads((A/'late_assessment_counterexample.json').read_text()),
 'no_ROS':True,'no_execution_release':True,'no_real_CSV_writes':True,'no_seed_registration':True}
(A/'focused_result.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({'passed_groups':len(checks),'rejected_mutations':len(mutations),'new_blocker':result['new_blocker']['classification']},indent=2))
