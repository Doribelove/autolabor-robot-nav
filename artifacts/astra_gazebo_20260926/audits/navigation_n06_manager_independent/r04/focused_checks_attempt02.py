"""R04 whole-supervisor late-assessment recovery, no ROS or real stage writes."""
import ast,copy,hashlib,json,sys
from pathlib import Path
from unittest.mock import patch
A=Path(__file__).resolve().parent;S=A.parents[2];P=S/'workspace/scripts/astra_gazebo';sys.path[:0]=[str(P),str(P/'tests')]
import test_navigation_n06_manager_r04 as t
q=t.q;m=t.manager;checks=[];scenarios=[]
def bind(p):
 p=Path(p).resolve();data=p.read_bytes();return {'path':str(p),'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
def add(case,**kwargs):checks.append(dict(case=case,pass_=True,**kwargs))
for version,count,expected in [('r01',352,'50a5af06ff0be1ddd7d94ae17af3b108c41a213a855be51a7d4656cf9b3f8c8b'),('r02',733,'76bd9a6256f927668eb46a5a6e366c6ab3735f4c4fe190480402c37d21dc0415'),('r03',764,'cb48b726ab31680eb2ee61c2721f5e2ccfa169c49f33ed16ac82409096b3fa87'),('r04',791,'47ac2100bdc1d454861fd9ca8aefecc25469626e1c62ee8eda03015735c1d2a6')]:
 p=S/('audits/navigation_n06_manager_'+version)/'source_evidence_manifest.json';assert bind(p)['sha256']==expected
 doc=json.loads(p.read_text());rows={r['path']:r for key in ('owned_sources','read_only_dependencies','author_evidence') for r in doc[key]}
 assert len(rows)==count and all(bind(r['path'])['sha256']==r['sha256'] for r in rows.values());add(version+'_frozen_roster_unchanged',resources=count)
for stem in ('navigation_n06_integrity','navigation_n06_process','navigation_n06_admin_plan'):
 assert (P/(stem+'_r03.py')).read_text().replace('_r03','_r04')==(P/(stem+'_r04.py')).read_text();add(stem+'_only_versioned_identity_rename')
old=(P/'run_navigation_n06_manager_r03.py').read_text().replace('navigation_n06_integrity_r03','navigation_n06_integrity_r04').replace('navigation_n06_process_r03','navigation_n06_process_r04')
new=(P/'run_navigation_n06_manager_r04.py').read_text()
left=ast.parse(old);right=ast.parse(new)
for tree in (left,right):
 for node in tree.body:
  if isinstance(node,ast.FunctionDef) and node.name=='terminal_receipt':
   if tree is right:
    # Python3.8-compatible structural location: only new if at body index4.
    found=[i for i,n in enumerate(node.body) if isinstance(n,ast.If) and isinstance(n.test,ast.BoolOp) and isinstance(n.test.values[0],ast.Compare) and isinstance(n.test.values[0].left,ast.Name) and n.test.values[0].left.id=='report']
    assert len(found)==1
    repair=node.body.pop(found[0]);assert isinstance(repair.body[0],ast.Assign)
assert ast.dump(left)==ast.dump(right)
add('manager_AST_only_retained_report_read_after_integrity_computation')

def setup(root):
 rows=q.prepared_api.schedule();units=[dict(r,contract=q.binding(q.prepared_api.PREPARED/r['run_id']/'contract.json')) for r in rows]
 state=q.initial_state('1'*64,rows);m.persist(state,'allocate');q.consume(state,0);m.persist(state,'consume')
 return rows,{'release_binding':{'sha256':'1'*64},'rows':rows,'prepared':{'units':units}}

for phase in ('complete','timeout','fault','stopped'):
 for negative_report in (False,True):
  with t.JournalTests().journal() as root:
   rows,data=setup(root);t.receipt(rows[0],outcome=phase);run=root/'runs'/rows[0]['run_id'];assessment=run/'assessment.json'
   if negative_report:
    report=json.loads(assessment.read_text());report['capture_integrity_pass']=False;report['integrity_checks']['goal_source_contract']=False;assessment.write_bytes(q.json_bytes(report))
   assessment_before=assessment.read_bytes();assessment_binding=q.binding(assessment)
   code=-15 if phase=='stopped' else -9
   answer=m.reconcile_worker_exit(data,code);state=q.strict_json(m.LEDGER.read_bytes());receipt=q.json_bound(state['units'][0]['receipt'])
   assert answer['missing_receipts_closed']==[rows[0]['run_id']]
   assert state['status']=='terminal' and state['consumed_units']==1 and [r['status'] for r in state['units']]==['consumed']+['retired_unrun']*7
   assert receipt['integrity_pass']is False and state['units'][0]['integrity_pass']is False
   assert receipt['task_outcome']==phase and receipt['roslaunch_exit_code']is None and str(code) in receipt['error']
   assert receipt['assessment']==assessment_binding and assessment.read_bytes()==assessment_before
   assert state['units'][0]['receipt']==q.binding(run/'receipt.json') and q.validate_terminal_receipt(receipt,rows[0],'1'*64)
   assert m.replay_state(data)==state
   snapshot={str(p):(p.read_bytes(),p.stat().st_ino,p.stat().st_mtime_ns) for p in root.rglob('*') if p.is_file()}
   m.reconcile_worker_exit(data,code)
   assert snapshot=={str(p):(p.read_bytes(),p.stat().st_ino,p.stat().st_mtime_ns) for p in root.rglob('*') if p.is_file()}
   # Test authoritative administrative status remains infra negative, even for a recovered complete task label.
   before={name:t.csv_document(fields,[]) for name,fields in t.admin.FIELDS.items()}
   before['seed_registry.csv']=q.reserved_registry_bytes(before['seed_registry.csv'],'1'*64,rows)
   projected=t.admin.projected_tables(before,state,{rows[0]['run_id']:receipt},'1'*64)
   _,exp=t.admin.records(projected['EXPERIMENTS.csv'],t.admin.FIELDS['EXPERIMENTS.csv'])
   assert exp[0][0]['status']=='terminal_integrity_failure' and all(r['status']=='retired_unrun_after_terminal_stop' for r,_ in exp[1:])
   scenarios.append({'phase':phase,'negative_assessment':negative_report,'observed_worker_exit':code,
                     'assessment_before_sha256':assessment_binding['sha256'],'assessment_unchanged':True,
                     'receipt':receipt,'ledger':state,'replayed_identical':True,'second_reconcile_noop':True,
                     'administrative_first_status':exp[0][0]['status'],'temporary_paths_only':True})
add('eight_whole_supervisor_late_assessment_cases_terminal_negative',cases=8)

with t.JournalTests().journal() as root:
 rows,data=setup(root);receipt=t.receipt(rows[0],outcome='timeout');receipt.update(integrity_pass=False,error='original retained cleanup failure')
 run=root/'runs'/rows[0]['run_id'];q.publish(run/'receipt.json',receipt);before=(run/'receipt.json').read_bytes()
 answer=m.reconcile_worker_exit(data,-9);state=q.strict_json(m.LEDGER.read_bytes())
 assert state['status']=='terminal' and q.json_bound(state['units'][0]['receipt'])['error']=='original retained cleanup failure'
 assert (run/'receipt.json').read_bytes()==before;add('existing_failure_receipt_reused_without_rewriting_original_error')
with t.JournalTests().journal() as root:
 rows,data=setup(root);answer=m.reconcile_worker_exit(data,-9);state=q.strict_json(m.LEDGER.read_bytes());receipt=q.json_bound(state['units'][0]['receipt'])
 assert state['status']=='terminal' and receipt['assessment']is None and receipt['task_outcome']is None and receipt['integrity_pass']is False
 add('no_assessment_interruption_still_closes_failure')
with t.JournalTests().journal() as root:
 rows,data=setup(root);t.receipt(rows[0]);run=root/'runs'/rows[0]['run_id'];p=run/'assessment.json';p.write_bytes(b'{malformed');bad=p.read_bytes()
 try:m.reconcile_worker_exit(data,-9);accepted=True
 except (ValueError,json.JSONDecodeError):accepted=False
 assert not accepted and p.read_bytes()==bad and not (run/'receipt.json').exists()
 add('malformed_assessment_not_promoted_declared_unresolved_limit',not_a_total_crash_recovery_proof=True)

result={'schema':'n06_r04_independent_focused_v1','checks':checks,'late_assessment_scenarios':scenarios,
 'fixtures':'actual manager/reconcile/validator/admin functions on temporary FS; source-shaped reports are synthetic, not physics outcomes',
 'r03_counterexample_fixed':True,'ROS_started':False,'execution_release_created':False,'seed_registered':False,'real_CSV_written':False}
(A/'focused_result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'groups':len(checks),'scenarios':len(scenarios),'output':bind(A/'focused_result.json')},indent=2))
