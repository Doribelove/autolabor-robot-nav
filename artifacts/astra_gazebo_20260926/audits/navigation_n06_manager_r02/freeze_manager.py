#!/usr/bin/env python3
import ast,difflib,hashlib,json,re,sys
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];P=S/'workspace/scripts/astra_gazebo';sys.path.insert(0,str(P))
import navigation_n06_integrity_r02 as q
import dense_sweep_n05 as dense

def b(p):p=Path(p).resolve();data=p.read_bytes();return {'path':str(p),'sha256':hashlib.sha256(data).hexdigest()}
def write(p,v):
 with p.open('x') as f:json.dump(v,f,indent=2,sort_keys=True,allow_nan=False);f.write('\n')
if (A/'implementation_report.json').exists():raise RuntimeError('frozen already')
r01=S/'audits/navigation_n06_manager_r01';old=json.loads((r01/'source_evidence_manifest.json').read_text())
olditems={i['path']:i for key in ('owned_sources','read_only_dependencies','author_evidence') for i in old[key]}
if any(b(p)['sha256']!=i['sha256'] for p,i in olditems.items()):raise RuntimeError('r01 frozen bytes changed')
log=(A/'author_tests_final.log').read_text();m=re.search(r'Ran (\d+) tests in ([0-9.]+)s\n\nOK',log)
if not m or int(m[1])!=56:raise RuntimeError('56 tests must pass')
names=['run_navigation_n06_manager','navigation_n06_integrity','navigation_n06_process','navigation_n06_admin_plan']
owned=[P/(name+'_r02.py') for name in names]+[P/'tests/test_navigation_n06_manager_r02.py']
snap=A/'frozen_sources';snap.mkdir();diffs=[]
for source in owned:
 (snap/source.name).write_bytes(source.read_bytes())
 prior=source.with_name(source.name.replace('_r02',''))
 before=prior.read_text();after=source.read_text();out=A/(source.name+'.diff');out.write_text(''.join(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile=str(prior),tofile=str(source))))
 diffs.append(b(out))
# The two orchestration modules have only renamed dependency imports.
for name in ('run_navigation_n06_manager','navigation_n06_process'):
 expected=(P/(name+'.py')).read_text()
 for original in names:expected=expected.replace(original,original+'_r02')
 if expected!=(P/(name+'_r02.py')).read_text():raise RuntimeError('unexpected orchestration/cleanup behavior change')
conversion=S/'contracts/passive_sweep_q05_prepared_r02/passive_sweep_q05_static/conversion/manifest.json'
c=json.loads(conversion.read_text());auth=dense.authenticate_conversion(conversion,c['input_urdf']['path'],c['converted_sdf']['path'],{},diagnostic=True)
if auth['pass']is not True or auth['runtime_converter_correspondence_qualified']is not False:raise RuntimeError('retained conversion diagnostic failed')
write(A/'retained_conversion_fixture_check.json',{'manifest':b(conversion),'result':auth,'no_process_spawned':True,'new_N06_evidence':False})
resources={p:b(p) for p in olditems};resources.update({i['path']:i for i in dense.conversion_resources(conversion)})
refs=[r01/'implementation_report.json',r01/'source_evidence_manifest.json',r01/'freeze_receipt.json',
 S/'audits/navigation_n06_manager_independent/r01/review.json',S/'audits/navigation_n06_manager_independent/initial_counterexamples.json']
resources.update({str(p.resolve()):b(p) for p in refs})
rows=q.prepared_api.schedule();reg=q.csv_rows((S/'seed_registry.csv').read_bytes(),q.REGISTRY_FIELDS)
if any(r['run_id'] in {u['run_id'] for u in rows} or r['seed'] in {str(u['seed']) for u in rows} for r in reg):raise RuntimeError('proposed identities unexpectedly registered')
if any((S/'runs'/row['run_id']).exists() for row in rows) or (S/'queue/navigation_n06_ledger.json').exists():raise RuntimeError('unexpected execution allocation')
write(A/'authority_observation.json',{'execution_authorized':False,'allocated_budget':0,'seed_rows':0,'ros_gazebo_started':False,'global_csv_writes':False,
 'observed_csv_bindings':[b(S/name) for name in ('seed_registry.csv','run_manifest.csv','EXPERIMENTS.csv')],
 'r01_unique_resources_unchanged':len(olditems)})
manifest={'schema':'astra_navigation_n06_manager_r02_source_evidence_v1','owned_sources':[b(p) for p in owned],
 'read_only_dependencies':list(resources.values()),'author_evidence':[b(p) for p in sorted(A.rglob('*')) if p.is_file()],
 'execution_authorized':False,'scope':'New author repair only; r01 retained negative and all real authority absent.'}
write(A/'source_evidence_manifest.json',manifest)
report={'schema':'astra_navigation_n06_manager_r02_author_report_v1','stage':q.NAME,
 'status':'offline_three_blocker_repair_author_pass_independent_review_pending','independent_review':False,
 'source_evidence_manifest':b(A/'source_evidence_manifest.json'),'sources':[b(p) for p in owned],'api':b(A/'API.md'),
 'tests':{'passed':int(m[1]),'wall_seconds':float(m[2]),'log':b(A/'author_tests_final.log'),
 'structural_positive_native_and_converter_auth_explicit_standins':True,'actual_fake_converter_negative_before_allocation':True,
 'real_saved_Q05_converter_positive_diagnostic_only':b(A/'retained_conversion_fixture_check.json')},
 'repaired_blockers':[
 {'id':'administrative_input_drift','repair':'rehash plan/review/resources before and after each table, before completion and existing-completion return; bind assessments/sources','regressions':['receipt changed after first replacement: following table untouched, no completion','assessment changed after last replacement: no completion']},
 {'id':'receipt_semantics','repair':'shared strict identity/error/exit/assessment/task/integrity validation for ledger close and projection','regressions':['foreign stage','true integrity with nonempty error','true integrity with absent assessment','nonzero exit','assessment identity and task mismatch','negative assessment and later-error forms retained']},
 {'id':'conversion_provenance','repair':'call actual frozen diagnostic authenticate_conversion before allocation','regressions':['three-field fake denied with no allocation','all eight calls diagnostic in structural fixture','saved historical complete conversion accepted without subprocess/live qualification']}],
 'r01_independent_negative_review':b(refs[3]),'r01_resources_unchanged':len(olditems),'exact_diffs':diffs,
 'unchanged':'Process cleanup and manager control behavior byte-equivalent after dependency-module renaming; no frozen runtime/physical/prepared edits.',
 'authority':{'execution_authorized':False,'release_created':False,'allocated_budget':0,'seeds_registered':0,'ros_gazebo_started':False,'global_csv_writes':False},
 'remaining':['independent r02 review pending','real N06 conversion/native/runtime/source closure and exact release absent','no live qualification','sequential CSV visibility and filesystem/final-check race limits retained; no adversarial concurrent writer theorem','outer host exit unknown']}
write(A/'implementation_report.json',report)
allitems={i['path']:i for key in ('owned_sources','read_only_dependencies','author_evidence') for i in manifest[key]}
if any(b(p)['sha256']!=i['sha256'] for p,i in allitems.items()):raise RuntimeError('postwrite drift')
write(A/'freeze_receipt.json',{'report':b(A/'implementation_report.json'),'manifest':b(A/'source_evidence_manifest.json'),'unique_resources':len(allitems),'postwrite_match':True,'authority':False})
print(json.dumps({'report':b(A/'implementation_report.json'),'manifest':b(A/'source_evidence_manifest.json'),'resources':len(allitems),'sources':[b(p) for p in owned]},indent=2))
