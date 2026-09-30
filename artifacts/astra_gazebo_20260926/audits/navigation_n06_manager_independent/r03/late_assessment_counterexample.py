"""Actual supervisor closure with a retained assessment, synthetic temp files."""
import hashlib,json,sys
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[2];P=S/'workspace/scripts/astra_gazebo'
sys.path[:0]=[str(P),str(P/'tests')]
import test_navigation_n06_manager_r03 as t
q=t.q;m=t.manager
with t.JournalTests().journal() as root:
 rows=q.prepared_api.schedule();units=[dict(r,contract=q.binding(q.prepared_api.PREPARED/r['run_id']/'contract.json')) for r in rows]
 state=q.initial_state('1'*64,rows);m.persist(state,'allocate');q.consume(state,0);m.persist(state,'consume')
 # Simulate assessor completed its atomic assessment write, then worker exited
 # before assigning/returning report and publishing its terminal receipt.
 t.receipt(rows[0]);run=root/'runs'/rows[0]['run_id']
 assessment_before=(run/'assessment.json').read_bytes()
 try:
  m.reconcile_worker_exit({'release_binding':{'sha256':'1'*64},'rows':rows,'prepared':{'units':units}},-9)
  error=None
 except BaseException as exc:error=type(exc).__name__+': '+str(exc)
 state=q.strict_json(m.LEDGER.read_bytes());retained=q.strict_json((run/'receipt.json').read_bytes())
 result={'schema':'n06_r03_late_assessment_independent_counterexample_v1','synthetic_no_ROS':True,
  'actual_function':'reconcile_worker_exit', 'injected_boundary':'assessment.json exists; worker stopped before report/receipt return',
  'exception':error,'assessment_bytes_preserved':(run/'assessment.json').read_bytes()==assessment_before,
  'receipt_has_assessment':retained['assessment'] is not None,'receipt_task_outcome':retained['task_outcome'],
  'receipt_integrity_pass':retained['integrity_pass'],'ledger_status':state['status'],
  'consumed_receipt_bound':state['units'][0]['receipt'] is not None,'retired_count':sum(r['status']=='retired_unrun' for r in state['units']),
  'classification':'unresolved_supervisor_closure_after_retained_assessment' if error else 'closed'}
 assert error and result['receipt_has_assessment'] and result['receipt_task_outcome'] is None
 (A/'late_assessment_counterexample.json').write_text(json.dumps(result,indent=2)+'\n')
 print(json.dumps(result,indent=2))
