#!/usr/bin/env python3
"""Wait only on receipts; execute read-only offline analysis of closed rows."""
import datetime,hashlib,json,subprocess,sys,time,traceback
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];done={};started=time.monotonic()
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
while time.monotonic()-started<1200:
 ledger=json.loads((S/'queue/m12_physics_p07_budget_runtime.json').read_bytes())
 for unit in ledger['units']:
  rid=unit['run_id'];R=S/'runs'/rid;rp=R/'receipt.json'
  if rid in done or not rp.is_file():continue
  receipt=json.loads(rp.read_bytes());report=receipt.get('report')or{}
  assert (R/'cleanup.json').is_file() and not (R/'fixture.bag.active').exists()
  print(json.dumps({'closed_receipt':rid,'outcome':receipt['outcome'],'failed':receipt.get('physical_failed_checks'),'original_peak':report.get('metrics',{}).get('actual_center_rate_radps')}),flush=True)
  if report.get('terminal_exception') or report.get('assessment_exception'):
   item={'status':'original_terminal_exception_needs_diagnosis','receipt':receipt};done[rid]=item;(D/'closed_row_progress.json').write_text(json.dumps(done,indent=2)+'\n');print(json.dumps({'exception_row':rid,'terminal_exception':report.get('terminal_exception')}),flush=True);continue
  results=[]
  for name in ('recompute_row.py','audit_protocol_row.py','audit_watermark_row.py'):
   log=D/(rid+'_'+name.replace('.py','')+'_r01.log')
   with log.open('x') as f:p=subprocess.run(['/usr/bin/python3',str(D/name),'--run-id',rid],stdout=f,stderr=subprocess.STDOUT)
   results.append({'script':name,'exit_code':p.returncode,'log':str(log)})
   if p.returncode!=0:
    done[rid]={'status':'offline_analysis_tool_error','invocations':results};(D/'closed_row_progress.json').write_text(json.dumps(done,indent=2)+'\n');print(json.dumps({'tool_error':rid,'script':name,'log':str(log)}),flush=True);break
  else:
   O=D/'rows'/rid;b=json.loads((O/'independent_raw_diagnosis.json').read_bytes());p=json.loads((O/'protocol_audit.json').read_bytes());w=json.loads((O/'watermark_audit.json').read_bytes());peak=b['steering']['links']['center_rate_radps']['max_abs'];original=report['metrics']['actual_center_rate_radps']['maximum_absolute'];exact=abs(peak-original)<=1e-9
   item={'status':'independent_closed_row_audit_pass' if p['passed']==p['count'] and w['passed']==w['count'] and exact else 'independent_closed_row_has_failures','run_id':rid,'seed':unit['seed'],'arm':unit['arm'],'outcome':receipt['outcome'],'original_failed_checks':[k for k,v in report['checks'].items() if not v],'original_check_count':len(report['checks']),'original_pass_count':sum(report['checks'].values()),'independent_20ms_peak_radps':peak,'original_20ms_peak_radps':original,'peak_recomputed_same':exact,'protocol_passed':p['passed'],'protocol_count':p['count'],'protocol_failed':[k for k,v in p['checks'].items() if not v],'watermark_passed':w['passed'],'watermark_count':w['count'],'watermark_failed':[k for k,v in w['checks'].items() if not v],'actual_source_watermark_ns':w['service_watermark_ns'],'driver_latency_ns':p['source_event_to_driver_receipt_latency_ns'],'legacy_callback_loss_diagnostic':p['legacy_headerless_raw_observations_in_loss_diagnostic_only'],'inputs':[{'path':str(q),'sha256':sha(q)} for q in (rp,O/'independent_raw_diagnosis.json',O/'protocol_audit.json',O/'watermark_audit.json')],'invocations':results};done[rid]=item;(O/'row_audit_summary.json').write_text(json.dumps(item,indent=2)+'\n');(D/'closed_row_progress.json').write_text(json.dumps(done,indent=2)+'\n');print(json.dumps(item),flush=True)
 if ledger['status']!='active' and all(r['run_id'] in done for r in ledger['units'] if r['budget_status']=='consumed'):
  print(json.dumps({'manager_terminal':ledger['status'],'consumed':ledger['consumed_units'],'retired':ledger['retired_units'],'audited':len(done)}),flush=True);break
 time.sleep(5)
else:raise RuntimeError('offline receipt monitor deadline; does not change simulation')
