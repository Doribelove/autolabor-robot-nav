"""Independent offline B05 checks. No runtime, reservation, or source edits."""
import copy
import csv
import hashlib
import json
import math
import sys
from pathlib import Path

A = Path(__file__).resolve().parent
S = A.parents[1]
P = S / 'workspace/scripts/astra_gazebo'
sys.path.insert(0, str(P))
import baseline_static_qualification_b05 as b

def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def binding(path):
    path = Path(path).resolve()
    return {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size}

def rejected(fn):
    try:
        fn()
    except (ValueError, TypeError):
        return True
    return False

checks = []
manifest_path = S / 'audits/baseline_static_qualification_b05/source_evidence_manifest.json'
assert sha(manifest_path) == '1c4d5496a07dfa122e7b883a339d594cc10368148c9604217f5a8d4408594d78'
resources = json.loads(manifest_path.read_text())['resources']
assert len(resources) == 264 and len({r['path'] for r in resources}) == 264
assert all(sha(r['path']) == r['sha256'] for r in resources)
checks.append({'id': 'frozen_264_resources_rehashed', 'pass': True})
design = json.loads((S / 'contracts/baseline_static_qualification_b05_draft.json').read_text())
scenes = [r['scene_id'] for r in design['scenes']]

# Actual saved caller inputs; no current registry mutation or new proposal file.
saved_registry = S / 'audits/baseline_static_qualification_b05/seed_registry_at_proposal.csv'
registry = list(csv.DictReader(saved_registry.read_text().splitlines()))
n06_path = S / 'contracts/navigation_n06_prepared_r01/prepared.json'
n06 = json.loads(n06_path.read_text())
n06_seeds = [r['seed'] for r in n06['units']]
assert all(type(x) is int for x in n06_seeds)
schedule = b.proposal_schedule(scenes, registry, n06_seeds)
assert schedule == json.loads((S / 'audits/baseline_static_qualification_b05/proposed_schedule.json').read_text())
assert schedule['allocated_budget'] == 0 and schedule['execution_authorized'] is False
checks.append({'id': 'actual_saved_inputs_schedule_equal_and_zero_authority', 'pass': True,
               'registry_rows': len(registry), 'N06_seed_types': ['int'], 'schedule_rows': 20,
               'N06_prepared': binding(n06_path), 'registry_snapshot': binding(saved_registry)})

counterexamples = []
for seed in b.PROPOSED_SEEDS:
    assert rejected(lambda: b.proposal_schedule(scenes, [], [seed]))
    assert rejected(lambda: b.proposal_schedule(scenes, [{'seed': str(seed), 'run_id': 'history'}]))
    result = b.proposal_schedule(scenes, [], [str(seed)])
    assert result == b.proposal_schedule(scenes, [], [])
    counterexamples.append({'proposed_seed': seed, 'other_proposal_string': str(seed),
                           'string_conflict_accepted': True, 'integer_conflict_rejected': True,
                           'CSV_string_history_conflict_rejected': True,
                           'returned_execution_authorized': result['execution_authorized']})
checks.append({'id': 'seed_type_bypass_reproduced_for_all_ten', 'pass': True,
               'classification': 'confirmed_negative_not_API_qualification', 'cases': 10})

# Verify every supplied scalar in the actual B04 namespace is compared, beyond overlays.
def leaves(value, path=()):
    if isinstance(value, dict):
        for key, child in value.items():
            yield from leaves(child, path + (key,))
    elif isinstance(value, list):
        for index, child in enumerate(value):
            yield from leaves(child, path + (index,))
    else:
        yield path, value

def changed(value):
    if type(value) is bool: return not value
    if type(value) is str: return value + '__independent_change'
    if type(value) is int: return value + 1
    if type(value) is float: return value + 0.125
    raise AssertionError(type(value))

leaf_checks = []
for method in b.METHODS:
    path = S / 'audits/baseline_integration_b04/launch_preflight_final_r2' / method / 'effective_move_base.json'
    expected = json.loads(path.read_text())
    assert b.compare_full_parameters(expected, copy.deepcopy(expected))['pass']
    count = 0
    for keys, value in leaves(expected):
        observed = copy.deepcopy(expected)
        container = observed
        for key in keys[:-1]: container = container[key]
        container[keys[-1]] = changed(value)
        result = b.compare_full_parameters(expected, observed)
        assert not result['pass'] and len(result['mismatches']) == 1
        count += 1
    leaf_checks.append({'method': method, 'input': binding(path), 'single_leaf_mutations_rejected': count})
checks.append({'id': 'all_actual_supplied_parameter_leaves_sensitive', 'pass': True, 'methods': leaf_checks})

types = [True, 1, 1.0, '1', [1], {'x': 1}]
for left in types:
    for right in types:
        result = b.compare_full_parameters({'v': left}, {'v': right})
        assert result['pass'] == (type(left) is type(right))
for bad in [None, (1,), {1: 'x'}, float('nan'), float('inf'), -float('inf'), 2**31, -(2**31)-1]:
    assert rejected(lambda: b.compare_full_parameters({'v': 1}, {'v': bad}))
assert not b.compare_full_parameters({'v': 0.}, {'v': -0.})['pass']
escaped = b.compare_full_parameters({'a/b~c': 1}, {'a/b~c': 2})
assert escaped['mismatches'][0]['path'] == '/move_base/a~1b~0c'
checks.append({'id': 'parameter_types_nonfinite_signed_zero_and_escaped_paths', 'pass': True})

phases = {phase: {'v': 1} for phase in b.PHASES}
assert b.compare_parameter_phases(phases, copy.deepcopy(phases))['pass']
for phase in b.PHASES:
    observed = copy.deepcopy(phases); observed[phase]['late_default'] = 2
    assert not b.compare_parameter_phases(phases, observed)['pass']
    del observed[phase]
    assert rejected(lambda: b.compare_parameter_phases(phases, observed))
partial = b.compare_full_parameters({'only_supplied_key': 1}, {'only_supplied_key': 1})
assert partial['pass'] and partial['runtime_default_completion_proved_by_this_function'] is False
checks.append({'id': 'three_phases_required_but_partial_equal_tree_not_proven_complete', 'pass': True,
               'declared_scope_limit': 'full expected runtime defaults must be independently prepared and bound'})

# Independent mixed outcome arithmetic. Labels below are synthetic, not measured outcomes.
rows = []
for index, item in enumerate(schedule['schedule']):
    row = {key: item[key] for key in ('run_id', 'method', 'seed', 'attempt')}
    row.update(record_kind='terminal', phase='complete', capture_integrity_pass=True,
               geometry_pass=True, goal_elapsed_s=3.+index/4., reason='synthetic_only')
    if index % 5 == 0: row.update(record_kind='retired_unstarted', phase=None, capture_integrity_pass=None, geometry_pass=None, goal_elapsed_s=None)
    elif index % 5 == 1: row.update(phase='timeout', goal_elapsed_s=None)
    elif index % 5 == 2: row['capture_integrity_pass'] = False
    elif index % 5 == 3: row['geometry_pass'] = False
    rows.append(row)
result = b.failure_inclusive_summary(schedule['schedule'], list(reversed(rows)))
for method in b.METHODS:
    selected = [r for r in rows if r['method'] == method]
    attempted = [r for r in selected if r['record_kind'] == 'terminal']
    def cost(row):
        valid = row['phase'] == 'complete' and row['capture_integrity_pass'] and row['geometry_pass']
        return row['goal_elapsed_s'] if valid else 22.
    actual = result['methods'][method]
    assert actual['attempted'] == len(attempted)
    assert actual['all_schedule_administrative_cap_mean_s'] == sum(map(cost, selected))/10
    assert actual['attempted_failure_inclusive_mean_s'] == sum(map(cost, attempted))/len(attempted)
for pair in result['pairs']:
    group = rows[2*pair['block_index']:2*pair['block_index']+2]
    by_method = {r['method']: r for r in group}
    assert pair['dwa_minus_teb_administrative_s'] == cost(by_method[b.METHODS[1]]) - cost(by_method[b.METHODS[0]])
    assert pair['both_attempted'] == all(r['record_kind'] == 'terminal' for r in group)
assert not result['qualification_determined'] and not result['performance_claim']
checks.append({'id': 'mixed_failures_retirements_pair_arithmetic_independent', 'pass': True,
               'synthetic_result': result['methods'], 'pair_count': len(result['pairs'])})

for change in ('missing', 'duplicate', 'extra_key', 'retired_fake_goal', 'private_success', 'nan', 'capture_int'):
    mutated = copy.deepcopy(rows)
    if change == 'missing': mutated.pop()
    elif change == 'duplicate': mutated[1] = copy.deepcopy(mutated[0])
    elif change == 'extra_key': mutated[4]['unbound'] = True
    elif change == 'retired_fake_goal': mutated[0]['goal_elapsed_s'] = 0.
    elif change == 'private_success': mutated[4]['phase'] = 'private_success'
    elif change == 'nan': mutated[4]['goal_elapsed_s'] = float('nan')
    elif change == 'capture_int': mutated[4]['capture_integrity_pass'] = 1
    assert rejected(lambda: b.failure_inclusive_summary(schedule['schedule'], mutated)), change
checks.append({'id': 'missing_failure_duplicate_false_retirement_and_types_rejected', 'pass': True, 'cases': 7})

# No authentication is implied by the pure label arithmetic.
fake_goals = []
for item in schedule['schedule']:
    row = {key: item[key] for key in ('run_id', 'method', 'seed', 'attempt')}
    row.update(record_kind='terminal', phase='complete', capture_integrity_pass=True,
               geometry_pass=True, goal_elapsed_s=0., reason='synthetic_no_raw')
    fake_goals.append(row)
labels = b.failure_inclusive_summary(schedule['schedule'], fake_goals)
assert labels['all20_attempted'] and not labels['qualification_determined'] and not labels['performance_claim']
assert all(x['valid_completed'] == 10 for x in labels['methods'].values())
checks.append({'id': 'arithmetic_accepts_labels_without_claiming_authentication', 'pass': True,
               'declared_scope_limit': 'trusted caller/assessor must prove public source goal, physical geometry, complete receipt and stage stop policy'})

gates = {r['id']: r for r in design['physical_and_reference_gates']}
assert 'pending' in gates['actual_footprint']['status'] and '.000406165' in gates['actual_footprint']['status']
assert 'not_implemented_pending' in design['full_parameter_contract']['automatic_default_completion_status']
assert 'pending' in design['command_and_receipt_contract']['implementation_status']
assert not design['execution_authorized'] and not design['execution_ready'] and design['allocated_budget'] == 0
checks.append({'id': 'defaults_nine_shapes_ingress_manager_are_explicitly_pending', 'pass': True})
assert all(sha(r['path']) == r['sha256'] for r in resources)
output = {'schema': 'astra_b05_independent_focused_checks_v1', 'independent_review': True,
          'check_groups': checks, 'confirmed_negative': {'id': 'OTHER_PROPOSED_SEED_STRING_BYPASS', 'cases': counterexamples,
          'current_frozen_input_domain_affected': False, 'future_generic_seed_conflict_gate_qualified': False,
          'new_version_type_validation_required': True},
          'resources_unchanged_after_checks': 264, 'actual_global_csv_written': False,
          'execution_release_created': False, 'ROS_started': False, 'seed_registered': False}
(A / 'focused_results.json').write_text(json.dumps(output, indent=2, allow_nan=False) + '\n')
print(json.dumps({'check_groups': len(checks), 'leaf_mutations': leaf_checks,
                  'negative_cases': len(counterexamples), 'output': binding(A / 'focused_results.json')}, indent=2))
