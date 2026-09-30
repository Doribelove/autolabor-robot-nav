"""Seal the exact reviewed 48-unit development design; no reservation or execution."""
import ast
import csv
import json
import os
from pathlib import Path
import socket
import sys

S=Path(__file__).resolve().parents[1]; W=S/'workspace'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import run_navigation_n04 as manager
import navigation_evidence_n04 as q
import run_smoke_a02 as common

prepared_path=S/'contracts/navigation_n04_prepared_r01/prepared.json'
prepared=json.loads(prepared_path.read_bytes())
review_path=S/'audits/navigation_n04_release_source_review/source_review.json'
review=json.loads(review_path.read_bytes())
assert review['stage']==manager.NAME and review['status']=='independently_reviewed'
assert review['schema']=='astra_navigation_source_review_v1'
scene_review_path=S/'audits/navigation_n04_scenes_independent/review.json'
assert q.sha(scene_review_path)=='9fb563ce18c19f8bf133364c736d8a6e466650afd1002587a1c61e9f1be84192'
scene_review=json.loads(scene_review_path.read_bytes())
assert scene_review['pass'] is True and all(value is True for value in scene_review['checks'].values())
actor_review_path=S/'audits/navigation_n04_actor_independent/review.json'
assert q.sha(actor_review_path)=='5076b048213cbae31f929817e8081cdae1e51d38e6ca8116b84157a08a42e327'
prerequisite_path=S/'contracts/passive_q05_navigation_evidence_qualification_r1.json'
prerequisite=json.loads(prerequisite_path.read_bytes())
assert prerequisite['qualified_for_navigation_evidence'] is True
freeze_path=S/'audits/navigation_n04_root_source_freeze_r02.json'
assert q.sha(freeze_path)=='5fd462b71b27fdb7117eb201d5afba270d53747a7956b16e4ef90e877609cf4f'
for item in json.loads(freeze_path.read_bytes())['files']:q.read_bound(item)
preflight_path=S/'audits/navigation_n04_root_launch_preflight_r01/report.json'
preflight=json.loads(preflight_path.read_bytes())
assert preflight['prepared']==q.binding(prepared_path)
assert len(preflight['runs'])==48 and all(row['pass_'] for row in preflight['runs'])
assert [row['run_id'] for row in preflight['runs']]==[x[0] for x in manager.identities()]
for row in preflight['runs']:q.read_bound(row['parameters'])
snapshot_path=S/'audits/navigation_n04_root_source_r01_snapshot/manifest.json'
snapshot=json.loads(snapshot_path.read_bytes())
old=next(x for x in snapshot['entries'] if x['original']==preflight['runner'])
assert q.sha(old['snapshot'])==old['original']['sha256']
before=ast.parse(Path(old['snapshot']).read_text());after=ast.parse(Path(manager.__file__).read_text())
before_nodes={x.name:ast.dump(x,include_attributes=False) for x in before.body if isinstance(x,(ast.FunctionDef,ast.ClassDef))}
after_nodes={x.name:ast.dump(x,include_attributes=False) for x in after.body if isinstance(x,(ast.FunctionDef,ast.ClassDef))}
assert before_nodes.keys()==after_nodes.keys()
changed=[name for name in before_nodes if before_nodes[name]!=after_nodes[name]]
assert changed==['required_resources'],changed
assert ast.dump(ast.Module(body=[x for x in before.body if not isinstance(x,(ast.FunctionDef,ast.ClassDef))],type_ignores=[]),include_attributes=False)==ast.dump(ast.Module(body=[x for x in after.body if not isinstance(x,(ast.FunctionDef,ast.ClassDef))],type_ignores=[]),include_attributes=False)
assert '94 passed' in (S/'logs/navigation_n04_root_contract_tests_r04.log').read_text()
replay_path=S/'audits/navigation_n04_dense_inherited_replay_check_r02.json'
assert json.loads(replay_path.read_bytes())['pass_'] is True
provenance_path=S/'audits/navigation_n04_root_launch_preflight_r02_provenance.json'
manager.write_new(provenance_path,dict(status='unchanged_launch_and_execution_semantics_with_expanded_source_roster',
    actual_48_launch_preflight=q.binding(preflight_path),old_runner=dict(path=old['snapshot'],sha256=old['original']['sha256']),
    current_runner=q.binding(Path(manager.__file__)),changed_functions=changed,
    all_other_top_level_code_and_function_AST_equal=True,
    added_source_dependencies=['prepare_navigation_n01_scene.py','scene_generator_v2.py','scene_generator.py'],
    actual_launch_parameters_rehashed=48,no_repeated_launch_or_simulation=True))

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
extra_path=review_path.parent/'release_required_resources.json'
extra=json.loads(extra_path.read_bytes())
for item in extra['resources']:add(item)
for unit in prepared['units']:
    for item in unit['files']+unit['conversion_resources']:add(item)
for item in scene_review['source_resources']:add(item)
for item in prerequisite['evidence_bindings']:add(item)
for folder in (review_path.parent,preflight_path.parent,snapshot_path.parent,
               S/'audits/navigation_n04_actor_independent',S/'audits/navigation_n04_scenes_independent'):
    for path in folder.rglob('*'):
        if path.is_file() and '__pycache__' not in path.parts:add(q.binding(path))
for path in (Path(__file__).resolve(),prepared_path,review_path,prerequisite_path,freeze_path,preflight_path,
             provenance_path,replay_path,S/'audits/navigation_n04_dense_inherited_replay_r02/report.json',
             S/'audits/navigation_n04_result_independent/planned_matrix.json',
             S/'audits/navigation_n04_result_independent/figure_selection_preregistration.json',
             S/'logs/navigation_n04_root_contract_tests_r04.log',
             W/'scripts/astra_gazebo/tests/test_navigation_evidence_n04.py',
             W/'scripts/astra_gazebo/tests/test_event_actor_contract_n04.py'):
    add(q.binding(path))
registry=list(csv.DictReader((S/'seed_registry.csv').open()))
ids=manager.identities(); seeds={str(x[1]) for x in ids}; names={x[0] for x in ids}
assert not any(row['seed'] in seeds or row['run_id'] in names for row in registry)
assert not (S/'queue/navigation_n04_ledger.json').exists()
assert not any((S/'runs'/name).exists() for name in names)
for port in (14820,14821):
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);sock.bind(('127.0.0.1',port))
release=dict(stage=manager.NAME,status='root_reviewed_release',created_utc=common.utc(),execution_authorized=True,
    authority='Active user long-run goal; new isolated six-family four-method Gazebo development diagnosis',maximum_episodes=48,
    prepared=q.binding(prepared_path),source_review=q.binding(review_path),
    passive_geometry_qualification=q.binding(prerequisite_path),resources=[resources[k] for k in sorted(resources)],
    retry_allowed=False,resume_allowed=False,training_authorized=False,real_vehicle_authorized=False,
    scope='Six development parent geometries, two repeated blocks each, four methods in counterbalanced order',
    image_policy=q.IMAGE_POLICY,algorithm_failures_retained=True,infrastructure_failure_retires_remaining=True,
    formal_result=False,limitations=[
        'Six parent geometries are not twelve independent geometries or a final performance population.',
        'Current TEB v0 and DWA configuration are integration baselines, not a completed strong-baseline tuning study.',
        'Frozen R2 retains its original global-x route features; guide lengths differ from nominal 9m for detour scenes.',
        'M08 persistent tracking and candidate-conditioned prediction are not part of this release.',
        'Static feasible witness curves establish geometric possibility only, not dynamic or controller success.',
        'Actor update teardown review covers the actual whole-process exit protocol, not arbitrary runtime plugin unloading.',
        'No general safety, training, deployment, hardware or superiority claim is authorized by this development matrix.'])
path=S/'contracts/navigation_n04_release.json'
candidate=S/'audits/navigation_n04_root_release_candidate_r01.json'
manager.write_new(candidate,release)
digest=q.sha(candidate);manager.verify_release(candidate,digest)
os.link(candidate,path)
manager.verify_release(path,digest)
print(json.dumps(dict(release=str(path),sha256=digest,resource_count=len(resources),verified=True,no_episode_allocated=True)))
