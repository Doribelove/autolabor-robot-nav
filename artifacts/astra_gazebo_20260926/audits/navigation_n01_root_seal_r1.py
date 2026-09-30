"""Seal exact new N01 integration after passive qualification; no allocation."""
import csv
import json
import os
from pathlib import Path
import socket
import sys

S = Path(__file__).resolve().parents[1]
W = S / 'workspace'
sys.path.insert(0, str(W / 'scripts/astra_gazebo'))
import run_navigation_n01 as manager
import navigation_evidence_p04 as evidence
import run_smoke_a02 as common


def unique(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n'); stream.flush(); os.fsync(stream.fileno())


prepared_path = S / 'contracts/navigation_n01_prepared_r02/prepared.json'
prepared = json.loads(prepared_path.read_bytes())
review_path = S / 'audits/navigation_n01_release_source_review/source_review.json'
review = json.loads(review_path.read_bytes())
assert review['schema'] == 'astra_navigation_source_review_v1'
assert review['stage'] == manager.NAME and review['status'] == 'independently_reviewed'
qualification_path = S / 'contracts/passive_q05_navigation_evidence_qualification_r1.json'
qualification = json.loads(qualification_path.read_bytes())
assert qualification['qualified_for_navigation_evidence'] is True
assert qualification['resolved_steering_kd'] == 6.
for item in qualification['evidence_bindings']:
    evidence.read_bound(item)

runner_review_path = S / 'audits/navigation_n01_p04_independent/review_r1.json'
runner_review = json.loads(runner_review_path.read_bytes())
assert runner_review['pass'] and runner_review['independent_review']
assert runner_review['reviewed_runner_sha256'] == evidence.sha(Path(manager.__file__))
assert runner_review['tests']['passed'] == 67 and runner_review['tests']['failed'] == 0
for item in runner_review['reviewed_resources']:
    evidence.read_bound(item)
coordinator_review_path = S / 'audits/navigation_coordinator_independent/review_r2.json'
coordinator_review = json.loads(coordinator_review_path.read_bytes())
assert coordinator_review['pass'] and all(coordinator_review['checks'].values())
assert coordinator_review['reviewed_core_sha256'] == evidence.sha(W / 'scripts/astra_gazebo/navigation_coordinator.py')
assert coordinator_review['reviewed_node_sha256'] == evidence.sha(W / 'src/simulation/m2_gazebo/scripts/astra_navigation_coordinator.py')
preflight_path = S / 'audits/navigation_n01_root_launch_preflight_r02/report.json'
preflight = json.loads(preflight_path.read_bytes())
assert preflight['prepared'] == evidence.binding(prepared_path)
assert preflight['runner'] == evidence.binding(Path(manager.__file__))
assert len(preflight['runs']) == 4 and all(row['pass_'] for row in preflight['runs'])
assert '39 passed' in (S / 'logs/navigation_evidence_p04_root_tests.log').read_text()

resources = {}


def add(item):
    evidence.read_bound(item)
    key = str(Path(item['path']).resolve())
    if key in resources:
        assert resources[key]['sha256'] == item['sha256'], key
    resources[key] = dict(path=key, sha256=item['sha256'])


for path in manager.required_resources():
    add(evidence.binding(path))
for key in ('resources', 'engine_module_bindings', 'engine_source_bindings'):
    for item in review[key]:
        add(item)
for key in ('native_engine_report', 'native_engine_records', 'upstream_source_manifest'):
    add(review[key])
for unit in prepared['units']:
    for item in unit['files'] + unit['conversion_resources']:
        add(item)
for item in qualification['evidence_bindings']:
    add(item)
for item in runner_review['reviewed_resources']:
    add(item)
for folder in (review_path.parent, preflight_path.parent):
    for path in folder.rglob('*'):
        if path.is_file():
            add(evidence.binding(path))
for path in (Path(__file__).resolve(), prepared_path, review_path, qualification_path,
             runner_review_path, coordinator_review_path,
             S / 'logs/navigation_evidence_p04_root_tests.log',
             W / 'scripts/astra_gazebo/tests/test_navigation_evidence_p04.py',
             W / 'scripts/astra_gazebo/tests/test_navigation_n01_runner_p04.py'):
    add(evidence.binding(path))

registry = list(csv.DictReader((S / 'seed_registry.csv').open()))
identities = manager.identities()
assert not any(row['seed'] == str(manager.SEED) or row['run_id'] in {x[0] for x in identities}
               for row in registry)
assert not (S / 'queue/navigation_n01_ledger.json').exists()
assert not any((S / 'runs' / row[0]).exists() for row in identities)
for port in (14820, 14821):
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('127.0.0.1', port))

release = dict(stage=manager.NAME, status='root_reviewed_release', created_utc=common.utc(),
    execution_authorized=True,
    authority='Active user long-run goal and CODEX_ASTRA_LONG_RUN_20260926.md; new isolated simulation only',
    maximum_episodes=4, prepared=evidence.binding(prepared_path), source_review=evidence.binding(review_path),
    passive_geometry_qualification=evidence.binding(qualification_path),
    resources=[resources[key] for key in sorted(resources)],
    retry_allowed=False, resume_allowed=False, training_authorized=False, real_vehicle_authorized=False,
    scope='Clear-fixture software, physical evidence, common activation and method integration only',
    algorithm_failures_retained=True, infrastructure_failure_retires_remaining=True,
    formal_result=False, navigation_performance_claim=False,
    limitations=['Frozen R2 retains oracle-dependent readiness/goal stopping and does not consume the full common route.',
                 'Actual TEB and DWA configurations are smoke settings, not tuned strong baselines.',
                 'This four-method integration is not the sensor-only primary comparison or six-family qualification.'])
release_path = S / 'contracts/navigation_n01_release.json'
unique(release_path, release)
digest = evidence.sha(release_path)
manager.verify_release(release_path, digest)
print(json.dumps(dict(release=str(release_path), sha256=digest, resources=len(resources),
                      verified=True, no_episode_allocated=True)))
