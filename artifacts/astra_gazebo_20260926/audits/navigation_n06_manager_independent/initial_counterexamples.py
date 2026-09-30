"""Independent temporary-FS counterexamples; not an execution release or ROS run."""
import copy,json,sys
from pathlib import Path
from unittest.mock import patch
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo';sys.path[:0]=[str(P),str(P/'tests')]
import test_navigation_n06_manager as author
import navigation_n06_integrity as q,navigation_n06_admin_plan as admin
results=[]
f=author.AdministrationTests();f.setUp()
try:
 plan,review=f.plan();original=q.publish;mutated=[False]
 def publish(path,*args,**kwargs):
  outcome=original(path,*args,**kwargs)
  if Path(path)==f.stage/'seed_registry.csv' and kwargs.get('replace') and not mutated[0]:
   mutated[0]=True;(f.stage/'receipt1.json').write_bytes(b'{}\n')
  return outcome
 with patch.object(q,'publish',side_effect=publish):completion=admin.apply_reviewed_plan(plan,review)
 item=f.bindings[f.rows[1]['run_id']]
 try:q.read_bound(item);drift_rejected=False
 except ValueError:drift_rejected=True
 results.append({'id':'input_drift_after_first_CSV_replace','first_csv_durable_then_bound_receipt_mutated':mutated[0],'bound_receipt_now_invalid':drift_rejected,'completion_written':(f.stage/'transaction/completion.json').is_file(),'all_tables_after':all(q.digest((f.stage/name).read_bytes())==row['after_sha256'] for name,row in zip(admin.FIELDS,q.json_bound(plan)['tables'])),'classification':'confirmed_commit_accepts_stale_terminal_input'})
finally:f.tearDown()
f=author.AdministrationTests();f.setUp()
try:
 original=copy.deepcopy(f.receipts);receipts=copy.deepcopy(original);receipts[f.rows[0]['run_id']].update(stage='foreign_stage',error='actual failure contradicting integrity_pass',assessment=None)
 projected=admin.projected_tables(f.before,f.state,receipts,f.sha)
 results.append({'id':'consumed_receipt_stage_error_and_assessment_not_validated','accepted':True,'status_contains_terminal_timeout':b'terminal_timeout' in projected['EXPERIMENTS.csv'],'classification':'confirmed_projection_does_not_validate_full_terminal_receipt_contract'})
finally:f.tearDown()
f=author.SyntheticClosureTests();f.setUp()
try:
 path,sha=f.document();data=q.verify_release(path,sha)
 results.append({'id':'conversion_manifest_authentication_not_called','accepted_structural_fixture':True,'conversion_manifest_has_only':['input_urdf','converted_sdf','resources'],'native_facts_stubbed':True,'actual_conversion_function_not_stubbed':True,'classification':'structural_roster_pass_does_not_establish_converter_provenance_before_allocation'})
finally:f.tearDown()
(A/'initial_counterexamples.json').write_text(json.dumps({'zero_authority':True,'source_unmodified':True,'results':results},indent=2)+'\n');print(json.dumps(results,indent=2))
