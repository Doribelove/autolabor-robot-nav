#!/usr/bin/env python3
"""Read only closed N04 receipts; keep all48 planned identities and negatives.

This inventory is not the independent bag/geometry replay. It exposes the
separate original task/safety/evidence/image dimensions without changing them.
"""
import argparse,hashlib,json
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1]
def sha(path):
 h=hashlib.sha256()
 with Path(path).open('rb')as f:
  for b in iter(lambda:f.read(4194304),b''):h.update(b)
 return h.hexdigest()
def load(p):return json.loads(Path(p).read_bytes())
def binding(p):return dict(path=str(Path(p).resolve()),sha256=sha(p))
def main():
 p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);args=p.parse_args()
 if args.output.exists():raise FileExistsError('Append a new dated inventory; do not overwrite')
 plan=load(D/'planned_matrix.json');ledger_path=S/'queue/navigation_n04_ledger.json';ledger=load(ledger_path)
 units={x['run_id']:x for x in ledger['units']};assert len(units)==48
 rows=[]
 for planned in plan['rows']:
  rid=planned['run_id'];unit=units[rid];assert all(unit[k]==planned[k]for k in ('seed','method'))
  row={k:planned[k]for k in ('sequence','run_id','seed','method','family','parent_geometry_id','block_index','replicate')};row.update(ledger_status=unit['status'],closed=False,independently_reviewed=False)
  if unit['status']=='retired_unrun':
   row.update(closed=True,observed_episode=False,retired=True);rows.append(row);continue
  if not unit.get('receipt'):
   row['pending_reason']='No ledger-bound receipt; no run file opened';rows.append(row);continue
  receipt_path=Path(unit['receipt']['path']);assert sha(receipt_path)==unit['receipt']['sha256'];receipt=load(receipt_path)
  assert all(receipt[k]==planned[k]for k in ('run_id','seed','method')) and receipt['attempt']==1 and receipt['retry_allowed']is False
  root=S/'runs'/rid;assert not (root/'fixture.bag.active').exists()
  row.update(closed=True,observed_episode=True,retired=False,receipt=binding(receipt_path),evidence_integrity_pass=receipt['integrity_pass'],original_error=receipt['error'],exit_code=receipt['exit_code'],task_phase=receipt['task_outcome'],assessment_available=False)
  if receipt.get('report'):
   report_path=Path(receipt['report']['path']);assert sha(report_path)==receipt['report']['sha256'];report=load(report_path)
   row.update(assessment_available=True,assessment=binding(report_path),valid_completed=report['valid_completed'],restricted_goal_time_s=report['restricted_goal_time_s'],native_goal_time_s=report['task']['raw_geometric_arrival_time_s'],task=report['task'],integrity_checks=report['integrity_checks'],safety_checks=report['safety_checks'],safety_pass=report['safety_within_declared_engineering_scope_pass'],image_coverage=report['image_coverage'])
  else:row.update(valid_completed=False,restricted_goal_time_s=22.,native_goal_time_s=None,safety_pass=None,image_coverage=None)
  audit_path=D/'rows'/rid/'r01/row_audit.json'
  if audit_path.is_file():
   audit=load(audit_path);row['independently_reviewed']=audit.get('pass')is True;row['independent_audit']=binding(audit_path);row['native_images']=audit.get('native_images',[])
  else:row['native_images']=[]
  if not row['evidence_integrity_pass']:row['failure_class']='infrastructure'
  elif row['valid_completed']:row['failure_class']=None
  else:row['failure_class']=str(row['task_phase'])+'|'+','.join(sorted(k for k,v in row.get('safety_checks',{}).items()if not v))
  rows.append(row)
 summary={'schema':'astra_n04_closed_inventory_v1','scope':'Failure-inclusive inventory; actual replay reports are separate','ledger_snapshot':ledger,'ledger_bytes_sha256':sha(ledger_path),'planned_matrix':binding(D/'planned_matrix.json'),'total_planned':48,'closed_receipts':sum(x.get('observed_episode',False)for x in rows),'retired':sum(x.get('retired',False)for x in rows),'pending':sum(not x['closed']for x in rows),'valid_completed':sum(x.get('valid_completed',False)for x in rows),'independently_reviewed':sum(x.get('independently_reviewed',False)for x in rows),'rows':rows,'script':binding(__file__)}
 args.output.parent.mkdir(parents=True,exist_ok=True)
 with args.output.open('x')as f:json.dump(summary,f,indent=2);f.write('\n')
 print(json.dumps({k:v for k,v in summary.items()if k not in ('rows','ledger_snapshot')},indent=2))
if __name__=='__main__':main()
