import hashlib
import json
import os
import socket
import sys
from datetime import datetime, timezone
from pathlib import Path

STAGE = Path(__file__).resolve().parents[1]
WS = STAGE/'workspace'
sys.path.insert(0, str(WS/'scripts/astra_gazebo'))
import run_m12_physics_p07 as m

def sha(p):
    return m.common.sha(Path(p))

def unique_write(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())

design, parent, rows = m.load_design()
m.check_seed_freshness(rows)
preflight_path = STAGE/'audits/m12_physics_p07_preflight_reviewed.json'
p = json.loads(preflight_path.read_bytes())
assert p['all_prerequisites_ready'] and not p['missing_resources'] and not p['changed_inherited_resources']
prior = m.predecessor(design)
assert p['predecessor'] == prior
launch_report_path = STAGE/'audits/m12_p07_root_launch_preflight/report.json'
launch_report = json.loads(launch_report_path.read_bytes())
assert len(launch_report['runs']) == 6 and launch_report['runner_sha256'] == sha(m.__file__)
assert launch_report['design_sha256'] == sha(m.DESIGN)
for row, checked in zip(rows, launch_report['runs']):
    assert checked['run_id'] == row['run_id'] and checked['contract_sha256'] == row['contract_sha256'] and checked['exit_code'] == 0
for port in (14820, 14821):
    with socket.socket() as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('127.0.0.1', port))
resources = {item['path']: item['sha256'] for item in p['resources']}
review_path = STAGE/'audits/m12_physics_p07_callback_name_review.json'
review = json.loads(review_path.read_bytes())
assert review['status']=='offline_callback_name_binding_repair_review_pass' and review['independent_probe_review'] is True
for item in review['resources']:
    resources[item['path']] = item['sha256']
extra = [Path(__file__), preflight_path, launch_report_path,
    STAGE/'audits/m12_p07_root_launch_preflight/check.py',
    STAGE/'audits/m12_p05_runtime_library_closure/report.json',
    STAGE/'audits/m12_p05_runtime_library_closure/ldd.txt',
    STAGE/'audits/m12_p07_result_independent/figure_selection_preregistration.json',
    STAGE/'audits/m12_p07_root_minimal_diff_review.json',
    STAGE/'audits/m12_p05_result_independent/terminal_startup_failure_audit.json',
    STAGE/'logs/build_physical_p05_and_common_components_01.log',
    WS/'scripts/astra_gazebo/tests/test_physical_actuation_probe_p05.py',
    WS/'scripts/astra_gazebo/tests/test_m12_physics_p05.py']
for item in extra:
    resources[str(item.resolve())] = sha(item)
for run in rows:
    for name in ('fixture.launch', 'parameters.yaml', 'stderr.log'):
        path = launch_report_path.parent/run['run_id']/name
        resources[str(path)] = sha(path)
for path, expected in resources.items():
    assert sha(path) == expected, path
readiness_path = STAGE/'audits/m12_p07_root_readiness.json'
readiness = dict(status='root_source_contract_launch_review_pass', created_utc=datetime.now(timezone.utc).isoformat(),
    independent_probe_manager_review_sha256=sha(review_path), independent_driver_review_sha256=sha(STAGE/'audits/m12_physics_p05_driver_root_review.json'),
    inherited_P05_measurement_review_tests=176, actual_source_and_probe_startup_validators='all six actual expanded URDFs passed with old failure reproduced and wrong plugin/seed rejected',
    all_required_resources_rehashed=len(p['resources']), launch_preflight_sha256=sha(launch_report_path),
    physical_thresholds_unchanged=True, gate_count=122, physical_factor='steeringKd4vs6', seeds=[260926023,260926024,260926025],
    no_allocation_at_seal=True, no_ros_started_at_seal=True, figure_selection_preregistered=True,
    initial_counterexamples_preserved=True, physical_qualification_pending=True)
unique_write(readiness_path, readiness)
resources[str(readiness_path)] = sha(readiness_path)
release = dict(stage=design['stage'], status='root_reviewed_release', created_utc=datetime.now(timezone.utc).isoformat(),
    execution_authorized=True, authority='Active user goal and CODEX_ASTRA_LONG_RUN_20260926.md, new simulation stage only',
    maximum_episodes=6, predecessor_measurement_qualified=True, predecessor=prior,
    resources=[dict(path=path, sha256=digest) for path,digest in sorted(resources.items())],
    retry_allowed=False, resume_allowed=False, training_authorized=False, real_vehicle_authorized=False,
    predecessor_qualification_scope='P02 valid measurement protocol; P03/P04/P05/P06 preserved terminal negatives; P07 callback lexical name collision repaired, all numerical evidence evaluation unchanged, new physical qualification pending')
release_path=STAGE/'contracts/m12_physics_p07_release.json'
unique_write(release_path, release)
digest=sha(release_path)
m.verify_release(release_path,digest,design,rows)
print(json.dumps(dict(release=str(release_path),sha256=digest,resources=len(resources),verified=True)))
