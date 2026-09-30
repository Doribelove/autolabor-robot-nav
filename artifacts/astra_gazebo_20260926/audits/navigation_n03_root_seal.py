"""Seal one fresh DWA nonholonomic-configuration integration; never allocate/run."""
import csv
import json
import os
from pathlib import Path
import socket
import sys

S=Path(__file__).resolve().parents[1];W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import run_navigation_n03 as manager
import navigation_evidence_n02 as q
import run_smoke_a02 as common

prepared_path=S/'contracts/navigation_n03_prepared_r01/prepared.json'
prepared=json.loads(prepared_path.read_bytes())
review_path=S/'audits/navigation_n03_release_source_review/source_review.json'
review=json.loads(review_path.read_bytes())
assert review['stage']==manager.NAME and review['status']=='independently_reviewed'
assert review['schema']=='astra_navigation_source_review_v1'
prerequisite_path=S/'contracts/passive_q05_navigation_evidence_qualification_r1.json'
prerequisite=json.loads(prerequisite_path.read_bytes())
assert prerequisite['qualified_for_navigation_evidence'] is True
preflight_path=S/'audits/navigation_n03_root_launch_preflight_r01/report.json'
preflight=json.loads(preflight_path.read_bytes())
assert preflight['runner']==q.binding(Path(manager.__file__)) and preflight['prepared']==q.binding(prepared_path)
assert len(preflight['runs'])==1 and preflight['runs'][0]['pass_']
assert '50 passed' in (S/'logs/navigation_evidence_n02_root_tests_r03.log').read_text()
assert '67 passed' in (S/'logs/navigation_n02_runner_root_tests_r01.log').read_text()
core_path=S/'audits/navigation_n02_coordinator_root_independent.json'
core=json.loads(core_path.read_bytes());assert all(x['normalized_AST_equal'] for x in core['checks'])
for row in core['checks']:assert q.sha(W/row['new'])==row['new_sha256']
resources={}
def add(item):
    q.read_bound(item);key=str(Path(item['path']).resolve())
    if key in resources:assert resources[key]['sha256']==item['sha256'],key
    resources[key]=dict(path=key,sha256=item['sha256'])
for path in manager.required_resources():add(q.binding(path))
for key in ('resources','engine_module_bindings','engine_source_bindings'):
    for item in review[key]:add(item)
for key in ('native_engine_report','native_engine_records','upstream_source_manifest'):add(review[key])
for value in review.values():
    if isinstance(value,dict) and 'path' in value and 'sha256' in value:add(value)
for unit in prepared['units']:
    for item in unit['files']+unit['conversion_resources']:add(item)
for item in prerequisite['evidence_bindings']:add(item)
for folder in (review_path.parent,preflight_path.parent):
    for path in folder.rglob('*'):
        if path.is_file():add(q.binding(path))
for path in (Path(__file__).resolve(),prepared_path,review_path,prerequisite_path,core_path,
    S/'audits/r2_sensor_lifecycle_independent/review_r1.json',
    S/'audits/navigation_n02_sensor_lifecycle_integration/implementation_report.json',
    S/'audits/navigation_n01_result_independent/terminal_result_audit.json',
    S/'scenes/navigation_n03_dwa_r01/derivation.json',
    S/'logs/navigation_evidence_n02_root_tests_r03.log',S/'logs/navigation_n02_runner_root_tests_r01.log',
    W/'scripts/astra_gazebo/tests/test_navigation_evidence_n02.py',
    W/'scripts/astra_gazebo/tests/test_navigation_n02_runner.py',
    S/'audits/navigation_n01_dwa_diagnosis/diagnosis.json',
    S/'audits/navigation_n01_dwa_independent/review_r02.json'):
    add(q.binding(path))
registry=list(csv.DictReader((S/'seed_registry.csv').open()))
assert not any(r['seed']==str(manager.SEED) or r['run_id'] in {x[0] for x in manager.identities()} for r in registry)
assert not (S/'queue/navigation_n03_ledger.json').exists()
assert not any((S/'runs'/x[0]).exists() for x in manager.identities())
for port in (14820,14821):
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);sock.bind(('127.0.0.1',port))
release=dict(stage=manager.NAME,status='root_reviewed_release',created_utc=common.utc(),execution_authorized=True,
    authority='Active user long-run goal; new isolated Gazebo DWA integration',maximum_episodes=1,
    prepared=q.binding(prepared_path),source_review=q.binding(review_path),
    passive_geometry_qualification=q.binding(prerequisite_path),resources=[resources[k] for k in sorted(resources)],
    retry_allowed=False,resume_allowed=False,training_authorized=False,real_vehicle_authorized=False,
    scope='DWA official acc_lim_y default restoration with unchanged actual odometry and strict nonholonomic command rejection',
    image_policy=q.IMAGE_POLICY,algorithm_failures_retained=True,formal_result=False,
    limitations=['DWA acc_lim_y=2.5 is a sampling-window parameter, not a physical lateral-acceleration capability.',
                'Zero lateral sampling was verified only within actual odometry |vy|<=0.125 m/s; domain-outside counterexamples remain.',
                'Single clear fixture only; no performance, general safety, deployment or hardware claim.',
                'N01 DWA capture failure remains negative; missing supplementary images are recorded independently in this new stage.'])
path=S/'contracts/navigation_n03_release.json'
with path.open('x') as stream:
    json.dump(release,stream,indent=2,allow_nan=False);stream.write('\n');stream.flush();os.fsync(stream.fileno())
digest=q.sha(path);manager.verify_release(path,digest)
print(json.dumps(dict(release=str(path),sha256=digest,resource_count=len(resources),verified=True,no_episode_allocated=True)))
