"""Independent N02 release probes: actual saved inputs and actual collector AST.

No ROS modules, live services, registry allocation, or execution entry point.
"""
import ast
import copy
import json
from pathlib import Path
import sys
import types

import pytest

S = Path(__file__).resolve().parents[2]
W = S / 'workspace'
sys.path.insert(0, str(W / 'scripts/astra_gazebo'))
import navigation_evidence_n02 as evidence


def terminal_code(filename):
    path = W / 'src/simulation/m2_gazebo/scripts' / filename
    tree = ast.parse(path.read_text())
    main = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'main')
    block = next(n for n in main.body if isinstance(n, ast.Try))
    start = next(i for i, n in enumerate(block.body)
                 if isinstance(n, ast.If) and isinstance(n.test, ast.Compare)
                 and isinstance(n.test.left, ast.Name) and n.test.left.id == 'terminal')
    return compile(ast.Module(body=block.body[start:], type_ignores=[]), str(path), 'exec')


def actual_short_terminal_environment(tmp_path, images, image_errors):
    historical = json.loads((S / 'runs/astra_navigation_n01_dwa_common_route/collector_raw.json').read_text())
    prepared = json.loads((S / 'contracts/navigation_n02_prepared_r01/prepared.json').read_text())
    contract = json.loads(Path(prepared['units'][0]['files'][0]['path']).read_text())
    raw = copy.deepcopy(historical)
    raw['images'] = images
    calls = []
    saved = {}
    watermark = historical['final_drain']['source_watermark_ns']

    def gateway(name, kind):
        calls.append(name)
        if name.endswith('get_physics_properties'):
            return types.SimpleNamespace(pause=True, time_step=.002)
        return types.SimpleNamespace(sim_time=watermark / 1e9)

    def wait_until(predicate, seconds, label):
        assert seconds > 0
        assert predicate(), label

    def atomic(path, value):
        saved[Path(path).name] = value

    def mark(label):
        calls.append(label)
        return []

    # The original run cannot be requalified here. Only run the new source's
    # terminal control-flow fragment with a fake service transport.
    return dict(terminal=raw['terminal'], start=raw['start_ns'], end=raw['end_ns'],
                phase=['active'], drain_deadline=[None], q=evidence, value=contract,
                raw=raw, image_errors=image_errors, gateway=gateway, Trigger=object,
                Empty=object, GetPhysicsProperties=object, GetWorldProperties=object,
                time=types.SimpleNamespace(sleep=lambda _:None),
                latest={t:watermark for t in evidence.DENSE_TOPICS}, wait_until=wait_until,
                require_roster=lambda:mark('roster'), process_roster=lambda _:mark('process_roster'),
                actual_planner_parameters=lambda:{}, common=types.SimpleNamespace(atomic=atomic),
                root=tmp_path, pid=12345, lifecycle_deadline=evidence.WallDeadline(60),
                calls=calls, saved=saved)


@pytest.mark.parametrize('image_case', ['one_actual', 'none', 'export_error'])
def test_actual_short_fault_new_terminal_fragment_reaches_final_maps(tmp_path, image_case):
    old = json.loads((S / 'runs/astra_navigation_n01_dwa_common_route/collector_raw.json').read_text())
    images = old['images'] if image_case == 'one_actual' else []
    errors = [{'error':'OSError: diagnostic injected PNG export failure'}] if image_case == 'export_error' else []
    env = actual_short_terminal_environment(tmp_path, images, errors)
    # Parent manager acknowledgement is transport-only and remains a separate
    # required wait in the extracted source.
    (tmp_path / 'maps_final_ready.json').write_text('{}')
    exec(terminal_code('astra_navigation_collector_n02.py'), env)
    assert env['calls'][:2] == ['/astra/navigation/stop', '/gazebo/pause_physics']
    assert env['calls'][-2:] == ['roster', 'process_roster']
    assert env['saved']['ready_for_maps_final.json']['phase'] == 'terminal'
    assert env['raw']['image_coverage']['two_distinct_frames_available'] is False
    assert env['raw']['image_coverage']['capture_integrity_dependency'] is False
    assert env['raw']['terminal']['phase'] == 'fault'
    assert env['end'] - env['start'] == 260_000_000


def test_same_one_frame_reproduces_original_n01_failure_without_editing_it(tmp_path):
    old = json.loads((S / 'runs/astra_navigation_n01_dwa_common_route/collector_raw.json').read_text())
    env = actual_short_terminal_environment(tmp_path, old['images'], [])
    with pytest.raises(RuntimeError, match='native image snapshots absent'):
        exec(terminal_code('astra_navigation_collector_p04.py'), env)
    assert 'ready_for_maps_final.json' not in env['saved']


def test_new_image_policy_never_exempts_missing_dense_tail(tmp_path):
    env = actual_short_terminal_environment(tmp_path, [], [])
    del env['latest'][evidence.DENSE_TOPICS[0]]
    with pytest.raises(AssertionError, match='final source watermark drain'):
        exec(terminal_code('astra_navigation_collector_n02.py'), env)
    assert not env['saved']


def test_shared_terminal_is_still_fault_not_visual_success():
    prepared = json.loads((S / 'contracts/navigation_n02_prepared_r01/prepared.json').read_text())
    value = json.loads(Path(prepared['units'][0]['files'][0]['path']).read_text())
    start, end = 7_440_000_000, 7_700_000_000
    terminal = dict(schema='astra_common_navigation_v1', run_id=value['run_id'], seed=value['seed'],
                    method=value['method'], phase='fault', terminal_source_ns=end, gate_source_ns=start,
                    terminal_reason='input_fault', terminal_time_authority=evidence.TERMINAL_AUTHORITIES['fault'])
    samples = [(stamp,(-3.9,0,0,0,0,0,1)) for stamp in range(start,end+1,evidence.STEP_NS)]
    result = evidence.navigation_outcome(samples,[5,0,0],start,end,terminal,value)
    assert result['task_phase'] == 'fault' and result['raw_geometric_arrival_source_ns'] is None
    assert result['failure_restricted_time_s'] == 22
