import csv,json,os,socket,sys
from pathlib import Path
S=Path(__file__).resolve().parents[1];W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import passive_sweep_q03 as q
import run_passive_sweep_q03 as m
import run_smoke_a02 as common
def unique(path,data):
 with path.open('x') as f:json.dump(data,f,indent=2,allow_nan=False);f.write('\n');f.flush();os.fsync(f.fileno())
component_path=S/'audits/passive_sweep_q03_component.json';component=json.loads(component_path.read_bytes())
assert component['status']=='offline_ready_for_independent_review' and component['tests']['passed']==124
for item in component['owned_sources']:q.read_bound(item)
assert '124 passed' in (S/'logs/passive_sweep_q03_root_tests.log').read_text()
preflight_path=S/'audits/passive_q03_root_launch_preflight_r02/report.json';preflight=json.loads(preflight_path.read_bytes())
assert len(preflight['runs'])==2 and all(x['pass_'] for x in preflight['runs'])
terminal_path=S/'audits/passive_q02_result_independent/terminal_failure_audit.json';terminal=json.loads(terminal_path.read_bytes())
assert terminal['status']=='independent_Q02_terminal_pre_unpause_failure_closure_pass'
prior=json.loads((S/'contracts/passive_sweep_q02_release.json').read_bytes())
for b in prior['resources']:q.read_bound(b)
old_review=json.loads(q.read_bound(prior['source_review']))
review=dict(old_review,schema='astra_passive_q03_source_review_v1',status='independently_reviewed',reviewer='/root independent of environment_audit author',created_utc=common.utc(),resources=[q.binding(W/name) for name in q.SOURCE_FILES])
review['root_review_evidence']={'scope':'NEW Q03 source identity/inventory repair; prior Q02 native-source/physics review remains exact','component':q.binding(component_path),'root_tests':q.binding(S/'logs/passive_sweep_q03_root_tests.log'),'root_launch_preflight':q.binding(preflight_path),'normalized_diff':component['normalized_diff'],'terminal_predecessor':q.binding(terminal_path),'actual_new_code_review':'Reviewed parser/classifier/deleted code fail-closed, raw-fsync before interpretation, same-fd ELF SHA/BuildID+inode, process start identity, command/collector/manager normalized diff. Root reran124 focused+inherited tests including real unlinked shm/current interpreter, same-process and incomplete throughput. No physical numeric or control changes.','scope_limit':'selected backing ELF and trusted process roster; unknown Q02 deleted mapping provenance remains unknown; no adversarial memory equivalence proof'}
review_path=S/'audits/passive_sweep_q03_root_source_review.json';unique(review_path,review)
assert m.native_review_facts(review)['native_reference_pass'] and m.native_review_facts(review)['upstream_source_review_pass']
prepared_path=S/'contracts/passive_sweep_q03_prepared_r02/prepared.json';prepared=json.loads(prepared_path.read_bytes());assert prepared['steering_kd']==6. and not prepared['missing_resources']
registry=list(csv.DictReader((S/'seed_registry.csv').open()));ids={r[0] for r in q.FIXTURES};seeds={str(r[1]) for r in q.FIXTURES}
assert not any(r['run_id'] in ids or r['seed'] in seeds for r in registry)
assert not (S/'queue/passive_sweep_q03_ledger.json').exists() and not any((S/'runs'/x).exists() for x in ids)
for port in (14820,14821):
 with socket.socket() as sock:sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);sock.bind(('127.0.0.1',port))
resources={}
def add(item):
 q.read_bound(item);key=str(Path(item['path']).resolve())
 if key in resources:assert resources[key]['sha256']==item['sha256'],key
 resources[key]=dict(path=key,sha256=item['sha256'])
for item in prior['resources']+prepared['resources']+review['resources']+component['owned_sources']:add(item)
for unit in prepared['units']:
 for item in unit['files']:add(item)
extras=[Path(__file__),component_path,terminal_path,prepared_path,review_path,S/'contracts/passive_sweep_q02_release.json',S/'queue/passive_sweep_q02_ledger.json',S/'runs/passive_sweep_q02_static/receipt.json',S/'runs/passive_sweep_q02_static/collector_raw.json',S/'runs/passive_sweep_q02_static/collector_report.json',S/'audits/passive_q03_root_launch_preflight_r02.py',S/'logs/passive_sweep_q03_root_tests.log',S/'contracts/passive_sweep_q03_design.json',Path(component['normalized_diff']['path'])]
extras.extend(x for x in preflight_path.parent.rglob('*') if x.is_file())
for path in extras:add(q.binding(path))
release={'stage':q.NAME,'status':'root_reviewed_release','created_utc':common.utc(),'execution_authorized':True,'authority':'Active user long-run goal and CODEX_ASTRA_LONG_RUN_20260926.md; new simulation stage only','maximum_episodes':2,'prepared':q.binding(prepared_path),'source_review':q.binding(review_path),'physical_parameter_decision':prior['physical_parameter_decision'],'resources':[resources[x] for x in sorted(resources)],'retry_allowed':False,'resume_allowed':False,'training_authorized':False,'real_vehicle_authorized':False,'predecessor_terminal':q.binding(terminal_path)}
release_path=S/'contracts/passive_sweep_q03_release.json';unique(release_path,release);digest=q.sha(release_path);m.verify_release(release_path,digest)
print(json.dumps(dict(release=str(release_path),sha256=digest,resources=len(resources),verified=True,no_episode_allocated=True)))
