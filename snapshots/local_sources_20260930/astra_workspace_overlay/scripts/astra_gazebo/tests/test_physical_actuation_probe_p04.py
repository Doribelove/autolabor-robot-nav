"""Terminal capture fault tests; synthetic packets never become episode evidence."""
import copy
import importlib.util
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    'probe_p04', ROOT/'src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p04.py')
m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(m)


@pytest.fixture
def fixture():
    raw = {'schedule_start_ns': 100000000}
    periods = {key: (2000000, 20000000) for key in ('models', 'links')}
    periods.update({key: (20000000, 20000000) for key in (
        'odom', 'joints', 'targets', 'backend_status', 'commanded_efforts')})
    periods.update({key: (2000000, 2000000) for key in (
        'dense_models', 'dense_links', 'contacts')})
    for key, (first, period) in periods.items():
        raw[key] = [{'ns': ns} for ns in range(first, 144000001, period)]
    contract = {'world_fixture': {'max_step_s': .002, 'fixed_world_models': ['ground_plane']},
                'model_name': 'm2', 'total_schedule_s': .04}
    physics = {'success': True, 'pause': True, 'time_step': .002, 'status_message': ''}
    world = {'success': True, 'sim_time': .144, 'model_names': ['ground_plane', 'm2'], 'status_message': ''}
    raw['final_drain'] = dict(m.final_stream_watermarks(raw, 144000000),
        physics=physics, world_queries=[copy.deepcopy(world), copy.deepcopy(world)],
        source_watermark_ns=144000000, stable_world_query_ns=[144000000, 144000000], wall_elapsed_s=.1)
    return raw, contract


def test_exact_different_publication_phases_are_required(fixture):
    raw, contract = fixture
    assert m.final_drain_integrity(raw, contract)
    tail = m.final_stream_watermarks(raw, 144000000)
    assert tail['required_last_ns']['models'] == 142000000
    assert tail['required_last_ns']['odom'] == 140000000
    assert tail['required_last_ns']['dense_links'] == 144000000


@pytest.mark.parametrize('stream', ['models', 'links', 'odom', 'joints', 'targets',
    'backend_status', 'commanded_efforts', 'dense_models', 'dense_links', 'contacts'])
def test_each_missing_tail_fails_even_when_formal_window_complete(stream, fixture):
    raw, contract = fixture
    raw[stream].pop()
    assert not m.final_drain_integrity(raw, contract)


@pytest.mark.parametrize('mutation', ['interior_loss', 'duplicate', 'off_grid', 'future', 'boolean'])
def test_dense_stream_count_endpoints_cannot_hide_missing_or_invalid_slot(mutation, fixture):
    raw, contract = fixture
    rows = raw['dense_links']
    if mutation == 'interior_loss': rows.pop(10)
    if mutation == 'duplicate': rows[10] = rows[9].copy()
    if mutation == 'off_grid': rows[10]['ns'] += 1
    if mutation == 'future': rows.append({'ns': 146000000})
    if mutation == 'boolean': rows[10]['ns'] = True
    assert not m.final_drain_integrity(raw, contract)


@pytest.mark.parametrize('mutation', ['unpaused', 'physics_failure', 'world_failure', 'changed_step',
    'missing_model', 'duplicate_model', 'nonfinite', 'boolean_time', 'offgrid_time', 'before_end',
    'changed_second_time', 'query_absent', 'pause_error', 'timeout', 'declared_tail_changed', 'absent'])
def test_invalid_service_or_closure_record_fails_closed(mutation, fixture):
    raw, contract = fixture
    drain = raw['final_drain']; physics = drain['physics']; world = drain['world_queries'][1]
    if mutation == 'unpaused': physics['pause'] = False
    if mutation == 'physics_failure': physics['success'] = False
    if mutation == 'world_failure': world['success'] = False
    if mutation == 'changed_step': physics['time_step'] = .001
    if mutation == 'missing_model': world['model_names'] = ['m2']
    if mutation == 'duplicate_model': world['model_names'].append('m2')
    if mutation == 'nonfinite': world['sim_time'] = float('nan')
    if mutation == 'boolean_time': world['sim_time'] = True
    if mutation == 'offgrid_time': world['sim_time'] = .143
    if mutation == 'before_end': world['sim_time'] = .138
    if mutation == 'changed_second_time': world['sim_time'] = .146
    if mutation == 'query_absent': drain['world_queries'].pop()
    if mutation == 'pause_error': raw['pause_error'] = 'timeout'
    if mutation == 'timeout': drain['wall_elapsed_s'] = 5.001
    if mutation == 'declared_tail_changed': drain['observed_last_ns']['dense_links'] = 140000000
    if mutation == 'absent': raw.pop('final_drain')
    assert not m.final_drain_integrity(raw, contract)


def test_actual_double_representation_on_grid_accepted(fixture):
    raw, contract = fixture
    world = raw['final_drain']['world_queries'][0]
    world['sim_time'] = 37.144000000000005
    assert m.paused_world_source_ns(raw['final_drain']['physics'], world, contract, 37000000000) == 37144000000


def test_old_failure_gate_implementation_and_numeric_assessment_unchanged():
    import ast
    old = ast.parse((ROOT/'src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p02.py').read_text())
    new = ast.parse(Path(m.__file__).read_text())
    functions = lambda tree: {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}
    before, after = functions(old), functions(new)
    assert ast.dump(before['assess']) == ast.dump(after['assess'])
    first = before['source_packet_integrity']; second = copy.deepcopy(after['source_packet_integrity'])
    # The added initial gate is the sole allowed change to old source checks.
    assignments = [n for n in second.body if isinstance(n, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'checks' for t in n.targets)]
    assert len(assignments) == 1
    assignments[0].value = ast.Dict(keys=[], values=[])
    assert ast.dump(first) == ast.dump(second)
