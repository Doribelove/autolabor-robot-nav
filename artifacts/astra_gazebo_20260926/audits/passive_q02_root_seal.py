"""Root seal for the new two-unit passive qualification; never starts ROS."""
import csv,json,os,socket,sys
from pathlib import Path
S=Path(__file__).resolve().parents[1];W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import passive_sweep_q02 as q
import run_passive_sweep_q02 as m
import run_smoke_a02 as common
def unique(path,data):
 with path.open('x') as stream:
  json.dump(data,stream,indent=2,allow_nan=False);stream.write('\n');stream.flush();os.fsync(stream.fileno())
review_path=S/'audits/m12_p07_result_independent/budget_binding_audit.json'
review=json.loads(review_path.read_bytes())
assert review['status']=='independent_P07_complete_result_budget_binding_audit_pass' and review['pass'] is True and review['independent_review'] is True
assert review['treatment_all122_pass_count']==3 and review['treatment_qualified_for_frozen_simulation_actuation_only'] is True
summary_path=S/'audits/m12_physics_p07_summary.json'; summary=json.loads(summary_path.read_bytes())
assert summary['measurement_valid_matrix_complete'] is True and summary['treatment_promoted'] is True and summary['treatment_all_gate_pass_count']==3
budget_path=S/'queue/m12_physics_p07_budget_runtime.json';budget=json.loads(budget_path.read_bytes())
assert budget['status']=='terminal_complete' and budget['consumed_units']==6 and budget['retired_units']==0
assert len(summary['pairs'])==3
for pair in summary['pairs']:
 assert pair['results']['treatment']['outcome']=='qualified' and not pair['results']['treatment']['failed_checks']
 assert pair['results']['treatment']['metrics']['actual_center_rate_radps']['maximum_absolute']<=.85
registry=list(csv.DictReader((S/'seed_registry.csv').open()))
assert not any(row['run_id'] in {x[0] for x in q.FIXTURES} or row['seed'] in {str(x[1]) for x in q.FIXTURES} for row in registry)
assert not (S/'queue/passive_sweep_q02_ledger.json').exists()
assert not any((S/'runs'/x[0]).exists() for x in q.FIXTURES)
for port in (14820,14821):
 with socket.socket() as sock:sock.bind(('127.0.0.1',port))
evidence=[review_path,summary_path,budget_path,S/'contracts/m12_physics_p07_release.json']
for row in budget['units']:
 root=S/'runs'/row['run_id'];evidence.extend([root/'probe_report.json',root/'receipt.json',root/'bindings.json'])
decision_path=S/'contracts/m12_p07_kd6_simulation_parameter_decision.json'
decision={'schema':'astra_simulation_physical_parameter_decision_v1','status':'frozen_simulation_actuation_qualified','created_utc':common.utc(),'resolved_steering_kd':6.,'adopted_factor':'steering damping Kd4 to Kd6 only; all other P05 force-controller and physical settings fixed','source_stage':'astra_m12_physics_p07','complete_matrix':6,'treatment_all122_pass_count':3,'control_physical_negatives':2,'evidence_bindings':[q.binding(x) for x in evidence],'scope':'frozen 36s deterministic Gazebo actuation fixture only; seeds do not represent independent geometry; no navigation/performance/deployment/formal safety claim','real_vehicle_authorized':False,'training_authorized':False}
unique(decision_path,decision)
prepared_path=S/'contracts/passive_sweep_q02_prepared_r01/prepared.json';prepared=json.loads(prepared_path.read_bytes());assert prepared['steering_kd']==6. and not prepared['missing_resources']
source_review_path=S/'audits/passive_sweep_q02_root_source_review.json';source_review=json.loads(source_review_path.read_bytes())
assert source_review['status']=='independently_reviewed'
resources={}
def add(binding):
 q.read_bound(binding);key=str(Path(binding['path']).resolve())
 if key in resources:assert resources[key]['sha256']==binding['sha256'],key
 resources[key]=dict(path=key,sha256=binding['sha256'])
parent=json.loads((S/'contracts/m12_physics_p07_release.json').read_bytes())
for item in parent['resources']+prepared['resources']+source_review['resources']+source_review['engine_module_bindings']+source_review['engine_source_bindings']:add(item)
for unit in prepared['units']:
 for item in unit['files']:add(item)
for key in ('native_engine_report','native_engine_records','upstream_source_manifest'):add(source_review[key])
extras=evidence+[decision_path,prepared_path,source_review_path,Path(__file__),S/'audits/passive_sweep_q02_component.json',S/'contracts/passive_sweep_q02_design.json',S/'logs/passive_sweep_q02_root_tests_r01.log',S/'audits/passive_q02_root_launch_preflight_r02.py']
extras.extend(p for p in (S/'audits/passive_q02_root_launch_preflight_r02').rglob('*') if p.is_file())
for path in extras:add(q.binding(path))
release_path=S/'contracts/passive_sweep_q02_release.json'
release={'stage':q.NAME,'status':'root_reviewed_release','created_utc':common.utc(),'execution_authorized':True,'authority':'Active user long-run goal and CODEX_ASTRA_LONG_RUN_20260926.md; new simulation stage only','maximum_episodes':2,'prepared':q.binding(prepared_path),'source_review':q.binding(source_review_path),'physical_parameter_decision':q.binding(decision_path),'resources':[resources[k] for k in sorted(resources)],'retry_allowed':False,'resume_allowed':False,'training_authorized':False,'real_vehicle_authorized':False}
unique(release_path,release);digest=q.sha(release_path);m.verify_release(release_path,digest)
print(json.dumps(dict(release=str(release_path),sha256=digest,resources=len(resources),verified=True,no_episode_allocated=True)))
