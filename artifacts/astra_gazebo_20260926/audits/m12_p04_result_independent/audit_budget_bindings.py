#!/usr/bin/env python3
"""Offline immutable P04 release/budget/archive and retained-evidence checks."""
import csv,hashlib,json
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1]
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
release_path=S/'contracts/m12_physics_p04_release.json';release=json.loads(release_path.read_text());release_sha=sha(release_path);assert release_sha=='cb256ee6105b81d2935ae37d6794df3621b282947c6a28214be53aa05a55f5f5'
schedule=json.loads((S/'queue/m12_physics_p04_schedule.json').read_text());budget=json.loads((S/'queue/m12_physics_p04_budget_runtime.json').read_text());summary=json.loads((S/'audits/m12_physics_p04_summary.json').read_text())
resources=[]
for row in release['resources']:
 p=Path(row['path']);blob=S/'code_blobs'/row['sha256'];resources.append(dict(row,current_matches=p.is_file() and sha(p)==row['sha256'],archive_matches=blob.is_file() and sha(blob)==row['sha256']))
registry=list(csv.DictReader((S/'seed_registry.csv').open()));ids={r['run_id'] for r in schedule};reg=[r for r in registry if r['run_id'] in ids];rows=[]
for scheduled,unit in zip(schedule,budget['units']):
 rid=scheduled['run_id'];R=S/'runs'/rid;item={'run_id':rid,'schedule_fields_match':all(unit[k]==v for k,v in scheduled.items()),'budget_status':unit['budget_status'],'attempt':unit['attempt'],'registry':[r for r in reg if r['run_id']==rid],'run_directory_exists':R.exists()}
 if unit['budget_status']=='consumed':
  receipt=json.loads((R/'receipt.json').read_text());report=json.loads((R/'probe_report.json').read_text());bindings=json.loads((R/'bindings.json').read_text());cleanup=json.loads((R/'cleanup.json').read_text())
  item['receipt_schedule_fields_match']=all(receipt[k]==v for k,v in scheduled.items());item['receipt_outcome']=receipt['outcome'];item['budget_receipt_outcome_match']=unit['outcome']==receipt['outcome'];item['receipt_embedded_report_exact']=receipt['report']==report
  item['receipt_artifact_hashes']={k[:-7]:sha(R/k[:-7])==v for k,v in receipt.items() if k.endswith('_sha256') and (R/k[:-7]).is_file()}
  item['bindings_sha_matches']=sha(R/'bindings.json')==receipt['bindings_sha256'];item['release_binding_same']=bindings['release_sha256']==release_sha
  item['all_release_resources_in_row']=all(row in bindings['resources'] for row in release['resources']);item['row_resource_count']=len(bindings['resources'])
  item['row_resources']=[dict(r,current_matches=sha(r['path'])==r['sha256'],archive_matches=sha(S/'code_blobs'/r['sha256'])==r['sha256']) for r in bindings['resources']]
  item['cleanup_confirmed']=not cleanup['owned_live_processes'] and cleanup['ports_14820_14821_released'];item['bag_closed']=not any(R.glob('*.bag.active'))
 rows.append(item)
checks={'release_450_unique_resources':len(resources)==len({r['path'] for r in resources})==450,'all_release_current_bytes_match':all(r['current_matches'] for r in resources),'all_release_archive_bytes_match':all(r['archive_matches'] for r in resources),'budget_terminal_1_consumed_5_retired':budget['status']=='stopped_infrastructure' and budget['consumed_units']==1 and budget['retired_units']==5 and budget['maximum_units']==6,'six_schedule_rows_exact':len(rows)==6 and all(r['schedule_fields_match'] for r in rows),'all_attempt_one':all(r['attempt']==1 for r in rows),'six_unique_registry_ids_once':len(reg)==6 and len({r['run_id'] for r in reg})==6,'one_consumed_has_receipt_and_matching_hashes':all(r['run_directory_exists'] and r['receipt_schedule_fields_match'] and r['budget_receipt_outcome_match'] and r['receipt_embedded_report_exact'] and all(r['receipt_artifact_hashes'].values()) and r['bindings_sha_matches'] for r in rows[:1]),'all_row_current_and_archive_bytes_match':all(r['current_matches'] and r['archive_matches'] for row in rows[:1] for r in row['row_resources']),'all_release_bindings_preserved_in_rows':all(r['all_release_resources_in_row'] and r['release_binding_same'] for r in rows[:1]),'retired_five_have_no_run_directories':all(not r['run_directory_exists'] and r['budget_status']=='retired' for r in rows[1:]),'cleanup_complete_and_bags_closed':all(r['cleanup_confirmed'] and r['bag_closed'] for r in rows[:1]),'first_control_infrastructure_stop':rows[0]['receipt_outcome']=='infrastructure_failure','no_matrix_or_treatment_promotion':summary['measurement_valid_matrix_complete'] is False and summary['treatment_promoted'] is False and summary['treatment_all_gate_pass_count']==0}
out={'checks':checks,'pass':all(checks.values()),'release_sha256':release_sha,'resources':resources,'budget':budget,'rows':rows,'figure_PHY02_status':'not_complete_preregistered_three_control_median_selection_unavailable_no_substitution','script_sha256':sha(__file__),'command':'/usr/bin/python3 '+str(Path(__file__).resolve()),'scope':'read-only offline; no runtime files modified'}
(D/'budget_binding_audit.json').write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n');print(json.dumps({'pass':out['pass'],'checks':checks,'sha256':sha(D/'budget_binding_audit.json')},indent=2))
