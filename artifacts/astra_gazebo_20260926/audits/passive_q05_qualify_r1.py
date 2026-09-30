"""Root component adoption after exact fresh Q05 independent result review."""
import hashlib
import json
import os
from pathlib import Path

S = Path(__file__).resolve().parents[1]


def binding(path):
    return dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest())


review_path = S / 'audits/passive_q05_result_independent/terminal_result_audit.json'
review = json.loads(review_path.read_bytes())
assert review['status'] == 'independent_Q05_complete_result_budget_binding_audit_pass'
assert review['pass'] is True and review['independent_review'] is True
assert review['original_episode_passes'] == [True, True]
ledger_path = S / 'queue/passive_sweep_q05_ledger.json'
ledger = json.loads(ledger_path.read_bytes())
assert ledger['status'] == 'terminal' and ledger['consumed_units'] == 2
assert [(u['run_id'], u['seed']) for u in ledger['units']] == [
    ('passive_sweep_q05_static', 260926031), ('passive_sweep_q05_actor', 260926032)]
visual_path = S / 'audits/passive_q05_root_visual_qc.json'
visual = json.loads(visual_path.read_bytes())
assert len(visual['images']) == 4 and all(x['root_individually_viewed'] and x['visual_accept'] for x in visual['images'])
paths = [Path(__file__).resolve(), review_path, ledger_path, visual_path,
         S / 'contracts/passive_sweep_q05_release.json',
         S / 'contracts/m12_p07_kd6_simulation_parameter_decision.json',
         S / 'audits/dense_sweep_p04_independent/review.json',
         S / 'workspace/scripts/astra_gazebo/dense_sweep_p04.py']
outcomes = []
for unit in ledger['units']:
    assert unit['status'] == 'consumed' and unit['pass'] is True
    d = S / 'runs' / unit['run_id']
    assert binding(d / 'receipt.json') == unit['receipt']
    receipt = json.loads((d / 'receipt.json').read_bytes())
    assert receipt['pass'] is True and receipt['attempt'] == 1 and receipt['retry_allowed'] is False
    assessment = json.loads((d / 'assessment.json').read_bytes())
    assert assessment['pass'] is True and len(assessment['checks']) == 13 and all(assessment['checks'].values())
    sweep = json.loads((d / 'sweep_evaluation/report.json').read_bytes())
    assert sweep['integrity_pass'] is True and sweep['diagnostic_only'] is False
    assert sweep['dense_solver_intervals'] == 5000 and sweep['minimum_engineering_conservative_swept_xy_m'] > 0.
    assert sweep['comparison_tolerance_unchanged'] == 1e-7
    assert sweep['qualification'] == 'qualified_within_declared_Gazebo_engineering_scope'
    cleanup = json.loads((d / 'cleanup.json').read_bytes())
    assert not cleanup['owned_live_processes'] and cleanup['ports_14820_14821_released'] is True
    outcomes.append(dict(run_id=unit['run_id'], seed=unit['seed'], attempt=1, checks_passed=13,
        window_s=10., solver_intervals=5000,
        minimum_engineering_conservative_swept_xy_m=sweep['minimum_engineering_conservative_swept_xy_m'],
        recorded_illegal_robot_contacts=assessment['contact_audit']['illegal_robot_contacts_count'],
        recorded_actor_environment_contacts=assessment['contact_audit']['actor_environment_contacts_count']))
    paths.extend(d / name for name in ('receipt.json', 'assessment.json', 'sweep_evaluation/report.json',
        'run_closure.json', 'cleanup.json', 'fixture.bag', 'collector_raw.json',
        'images/pulse_start.png', 'images/stopped.png', 'images/pulse_start.rosmsg', 'images/stopped.rosmsg'))
result = dict(schema='astra_passive_q05_navigation_evidence_qualification_v1',
    qualified_for_navigation_evidence=True, resolved_steering_kd=6.,
    evidence_bindings=[binding(p) for p in paths], outcomes=outcomes,
    consumed_units=2, retired_units=0, retry_allowed=False, resume_allowed=False,
    scope='Adopt P04 actual converted-model correspondence and native geometry/contact capture for separately released Gazebo navigation integration.',
    model_scope='gazebo_normal_euler_extension_with_actor_begin_jumps',
    comparison_tolerance=1e-7,
    historical_Q04_failure_preserved=True, formal_safety=False, real_continuous_physical_safety=False,
    navigation_performance_evidence=False, real_vehicle_authorized=False,
    limitations=['Two clear component fixtures, not navigation or scene-general safety.',
                 'Actual wheel-ground support contacts are permitted; absence of illegal contact alone is not proof.',
                 'Kd6 reference rate qualification remains the original P07 sampled scope; no every-step steering-rate claim.',
                 'Each successor must retain actual-model/runtime-map/source-window checks for its own evidence.'])
output = S / 'contracts/passive_q05_navigation_evidence_qualification_r1.json'
with output.open('x') as stream:
    json.dump(result, stream, indent=2, allow_nan=False)
    stream.write('\n'); stream.flush(); os.fsync(stream.fileno())
print(json.dumps(binding(output)))
