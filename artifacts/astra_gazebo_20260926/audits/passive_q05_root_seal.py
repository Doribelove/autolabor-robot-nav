"""Seal the independently reviewed new Q05 stage; do not allocate or execute."""
import csv
import json
import os
from pathlib import Path
import socket
import sys

S=Path(__file__).resolve().parents[1];W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import passive_sweep_q05 as q
import run_passive_sweep_q05 as manager
import run_smoke_a02 as common

def unique(path,value):
    with path.open('x') as stream:
        json.dump(value,stream,indent=2,allow_nan=False);stream.write('\n')
        stream.flush();os.fsync(stream.fileno())

independent_path=S/'audits/dense_sweep_p04_independent/review.json'
independent=json.loads(independent_path.read_bytes())
assert independent['status']=='independent_P04_model_conversion_and_Q05_integration_review_pass'
assert independent['pass'] is True and independent['independent_review'] is True
assert independent['reviewed_p04_sha256']==q.sha(W/'scripts/astra_gazebo/dense_sweep_p04.py')
assert independent['reviewed_q05_runner_sha256']==q.sha(Path(manager.__file__))
assert independent['numeric_profile_unchanged'] is True and independent['model_tolerance_unchanged']==1e-7
assert independent['historical_diagnostic_only'] is True and independent['historical_Q04_not_reclassified'] is True
assert independent['checks'] and all(independent['checks'].values())
prepared_path=S/'contracts/passive_sweep_q05_prepared_r02/prepared.json'
assert independent['q05_prepared_binding']==q.binding(prepared_path)
prepared=json.loads(prepared_path.read_bytes())
assert not prepared['missing_resources'] and prepared['steering_kd']==6.
assert '35 passed' in (S/'logs/dense_sweep_p04_root_system_tests.log').read_text()
assert '107 passed' in (S/'logs/passive_q05_runtime_tests_final.log').read_text()
preflight_path=S/'audits/passive_q05_root_launch_preflight_r02/report.json'
preflight=json.loads(preflight_path.read_bytes())
assert len(preflight['runs'])==2 and all(x['pass_'] for x in preflight['runs'])
terminal_path=S/'audits/passive_q04_result_independent/terminal_failure_audit.json'
terminal=json.loads(terminal_path.read_bytes())
assert terminal['status']=='independent_Q04_terminal_geometry_QC_failure_closure_pass'
cas_path=S/'audits/bound_archive_independent/review_r1.json'
cas=json.loads(cas_path.read_bytes())
assert cas['status']=='independent_content_addressed_archive_filesystem_review_pass'
component_path=S/'audits/dense_sweep_p04_component.json'
component=json.loads(component_path.read_bytes())
prior_path=S/'contracts/passive_sweep_q04_release.json';prior=json.loads(prior_path.read_bytes())
for item in prior['resources']:q.read_bound(item)
old_review=json.loads(q.read_bound(prior['source_review']))
libraries=[]
for unit in prepared['units']:
    conversion=json.loads(q.read_bound(unit['conversion_manifest']))
    libraries.append(conversion['runtime_correspondence_libraries'])
assert libraries[0]==libraries[1] and len(libraries[0])==1
converter=libraries[0][0]
assert converter['role']=='urdf_to_sdf_converter'
review=dict(old_review,schema='astra_passive_q05_source_review_v1',status='independently_reviewed',
    reviewer='/root/historical_audit independent of P04 environment author and Q05/CAS root author',
    created_utc=common.utc(),resources=[q.binding(W/name) for name in q.SOURCE_FILES],
    engine_module_bindings=old_review['engine_module_bindings']+[converter])
review['root_review_evidence']={
    'independent_model_and_integration_review':q.binding(independent_path),
    'independent_archive_review':q.binding(cas_path),
    'component':q.binding(component_path),'predecessor_terminal':q.binding(terminal_path),
    'system_reference_tests':q.binding(S/'logs/dense_sweep_p04_root_system_tests.log'),
    'runtime_fixture_tests':q.binding(S/'logs/passive_q05_runtime_tests_final.log'),
    'root_launch_preflight':q.binding(preflight_path),
    'scope':'Effective converted model and native runtime mass/COM/inertia/collision correspondence; original1e-7 threshold and P03 numeric profile retained; no dynamics change.',
    'test_environment_note':'SciPy independent rotation references use system Python/NumPy/SciPy. Runtime CLI and Q05/CAS tests use isolated .venv; an incompatible mixed SciPy collection failure is retained.',
    'historical_Q04_not_reclassified':True}
review_path=S/'audits/passive_sweep_q05_root_source_review.json';unique(review_path,review)
facts=manager.native_review_facts(review)
assert facts['native_reference_pass'] and facts['upstream_source_review_pass']
registry=list(csv.DictReader((S/'seed_registry.csv').open()))
ids={x[0] for x in q.FIXTURES};seeds={str(x[1]) for x in q.FIXTURES}
assert not any(x['run_id'] in ids or x['seed'] in seeds for x in registry)
assert not (S/'queue/passive_sweep_q05_ledger.json').exists()
assert not any((S/'runs'/x).exists() for x in ids)
for port in (14820,14821):
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);sock.bind(('127.0.0.1',port))
resources={}
def add(item):
    q.read_bound(item);key=str(Path(item['path']).resolve())
    if key in resources:assert resources[key]['sha256']==item['sha256'],key
    resources[key]=dict(path=key,sha256=item['sha256'])
for item in prior['resources']+prepared['resources']+review['resources']+review['engine_module_bindings']:
    add(item)
for unit in prepared['units']:
    for item in unit['files']+unit['conversion_resources']:add(item)
for item in independent['resources']:add(item)
extras=[Path(__file__),independent_path,cas_path,component_path,terminal_path,prepared_path,review_path,prior_path,
    S/'contracts/passive_sweep_q05_design.json',S/'queue/passive_sweep_q04_ledger.json',
    S/'runs/passive_sweep_q04_static/receipt.json',S/'runs/passive_sweep_q04_static/assessment.json',
    S/'runs/passive_sweep_q04_static/run_closure.json',S/'runs/passive_sweep_q04_static/fixture.bag',
    S/'audits/passive_q05_root_launch_preflight_r02.py',
    S/'logs/dense_sweep_p04_root_system_tests.log',S/'logs/passive_q05_runtime_tests_final.log',
    S/'logs/passive_q05_root_tests_r02.log',
    W/'scripts/astra_gazebo/tests/test_dense_sweep_p04.py',
    W/'scripts/astra_gazebo/tests/test_passive_sweep_q05.py',W/'scripts/astra_gazebo/tests/test_bound_archive.py',
    S/'visuals/plots/geometry_contract/F04_selection_preregistration.json']
extras.extend(x for x in preflight_path.parent.rglob('*') if x.is_file())
for path in extras:add(q.binding(path))
release={'stage':q.NAME,'status':'root_reviewed_release','created_utc':common.utc(),
    'execution_authorized':True,'authority':'Active user long-run goal and CODEX_ASTRA_LONG_RUN_20260926.md; fresh simulation stage only',
    'maximum_episodes':2,'prepared':q.binding(prepared_path),'source_review':q.binding(review_path),
    'physical_parameter_decision':prior['physical_parameter_decision'],
    'resources':[resources[x] for x in sorted(resources)],'retry_allowed':False,'resume_allowed':False,
    'training_authorized':False,'real_vehicle_authorized':False,'predecessor_terminal':q.binding(terminal_path)}
release_path=S/'contracts/passive_sweep_q05_release.json';unique(release_path,release)
digest=q.sha(release_path);manager.verify_release(release_path,digest)
print(json.dumps(dict(release=str(release_path),sha256=digest,resources=len(resources),verified=True,no_episode_allocated=True)))
