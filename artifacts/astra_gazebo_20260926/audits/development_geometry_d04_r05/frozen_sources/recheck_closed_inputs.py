#!/usr/bin/env python3
"""Author read-only replay of sealed D04 inputs; no generator/seed invocation.

Allocation is independently re-encoded here. Geometry replay intentionally uses
the declared component model and is not an independent-review claim.
"""
import collections
import csv
import hashlib
import json
import math
import platform
import sys
from pathlib import Path

AUDIT = Path(__file__).resolve().parent
STAGE = AUDIT.parent.parent
WORK = STAGE / 'workspace'
sys.path.insert(0, str(WORK / 'scripts/astra_gazebo'))
import prepare_development_geometry_d04_r05 as author
import numpy as np
import shapely

OUTPUT = STAGE / 'scenes/development_d04_geometry_r05'

def read(p):
    return json.loads(Path(p).read_text())

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')

def cell_key(cell):
    return 'actor_' + hashlib.sha256(encoded(cell)).hexdigest()[:24]

def allocation(parent, actors):
    keys = [cell_key(a['actual_cell']) for a in actors]
    if not keys:
        joint_cell = 'actor_none_v1'
    elif len(keys) == 1:
        joint_cell = keys[0]
    else:
        ordered = [[a['model_name'], key] for a, key in zip(actors, keys)]
        joint_cell = 'actor_tuple_v1_' + hashlib.sha256(encoded(ordered)).hexdigest()
    payload = ('astra_gazebo_20260926|' + parent + '|' + joint_cell + '|allocation_v1').encode('utf-8')
    digest = hashlib.sha256(payload).hexdigest()
    bucket = int(digest[:16], 16) % 100
    split = next(name for lo, hi, name in ((0,60,'train'),(60,80,'development'),(80,90,'validation'),(90,100,'final_test')) if lo <= bucket < hi)
    return joint_cell, {'payload_utf8': payload.decode('utf-8'), 'payload_utf8_hex': payload.hex(), 'sha256': digest, 'bucket': bucket, 'split': split}

def close(a, b):
    assert math.isclose(a, b, rel_tol=0., abs_tol=1e-12), (a, b)

def main():
    report_path = AUDIT / 'closed_input_recheck.json'
    if report_path.exists():
        raise FileExistsError('retain previous recheck')
    report = read(AUDIT / 'generation_report.json')
    initial = read(AUDIT / 'initial_ownership.json')
    final = read(AUDIT / 'final_ownership.json')
    layouts = read(OUTPUT / 'layouts.json')
    prior = read(author.PRIOR / 'layouts.json')
    prior_by_id = {s['parent_geometry_id']: s for s in prior}
    authenticated = {}
    for row in report['source_resources'] + report['retained_prior_bindings'] + [report['source'], report['design']]:
        assert sha(row['path']) == row['sha256'], row['path']
        authenticated[row['path']] = row['sha256']
    # Preserve every old r04 source, input, failed attempt and author report byte.
    r04_manifest = read(STAGE / 'audits/development_geometry_d04/r04/source_evidence_manifest.json')
    r04_count = 0
    for group in r04_manifest.values():
        if isinstance(group, list):
            for row in group:
                assert sha(row['path']) == row['sha256'], row['path']
                r04_count += 1
    for group in ('parent_groups', 'actual_actor_cells'):
        assert all(final[group].get(k) == v for k, v in initial[group].items())
    assert 'actor_none_v1' not in final['actual_actor_cells']
    counts = collections.Counter(s['family'] for s in layouts)
    assert len(layouts) == 24 and set(counts) == set(author.old.v1.FAMILIES)
    assert set(counts.values()) == {4}
    assert len({s['parent_geometry_id'] for s in layouts}) == 24
    priors, _, _, _ = author.prior_data()
    selected = []
    actor_rows = []
    parent_rows = []
    for s in layouts:
        pid = s['parent_geometry_id']
        actual = read(OUTPUT / s['layout_id'] / 'geometry_manifest.json')
        assert actual == s
        assert author.old.v1.parent_id(s) == pid
        assert author.old.v1.digest(author.old.v1.canonical_geometry(s, False)) == s['canonical_geometry_sha256']
        key, joint = allocation(pid, s['actors'])
        assert key == s['actor_parameter_cell_id'] and joint == s['joint_allocation']
        assert joint['split'] == 'development' == s['split'] == final['parent_groups'][pid]
        assert s['execution_seed'] is None and s['new_seed_allocated'] is False and s['execution_authorized'] is False
        assert len(s['actors']) == len(s['actor_geometry'])
        for i, (actor, geom) in enumerate(zip(s['actors'], s['actor_geometry'])):
            assert actor['model_name'] == 'benchmark_actor_' + str(i + 1)
            assert all(actor[k] == v for k, v in geom.items())
            cell = actor['actual_cell']
            cid = cell_key(cell)
            assert actor['parameter_cell_id'] == cid and final['actual_actor_cells'][cid] == 'development'
            assert cid not in initial['actual_actor_cells']
            assert actor['policy'] == cell['policy']
            for value, field, divisor in (('base_speed_mps','speed_cell',20),('response_distance_m','response_distance_cell',5),('response_time_margin_s','margin_cell',10),('gate_delay_s','delay_cell',5)):
                close(actor[value], cell[field] / divisor)
            arrival = actor['gate_delay_s'] + math.dist(actor['start'], actor['conflict']) / actor['base_speed_mps'] - (actor['conflict'][0] + 4) / .8
            assert abs(arrival) <= 2.
            if s['family'] == 'sequential_interaction' and i > 0:
                assert .8 <= actor['gate_delay_s'] <= 1.
            actor_rows.append({'layout_id':s['layout_id'], 'model_name':actor['model_name'], 'cell_id':cid, 'nominal_arrival_difference_s':arrival})
        if s['parent_geometry_retained_from_r04']:
            assert author.old.v1.canonical_geometry(s, False) == author.old.v1.canonical_geometry(prior_by_id[pid], False)
        else:
            assert s['geometry_rng_seed'] is None and pid not in initial['parent_groups']
            for p in priors + selected:
                distance = author.old.separation(s, p)
                assert not author.old.v1.near_duplicate(s, p)
                assert distance is None or distance >= 2.
        selected.append(s)
        parent_rows.append({'layout_id':s['layout_id'], 'parent_geometry_id':pid, 'joint_bucket':joint['bucket'], 'retained_geometry':s['parent_geometry_retained_from_r04']})
    assert len({a['cell_id'] for a in actor_rows}) == len(actor_rows) == 20
    assert len({(s['parent_geometry_id'], s['actor_parameter_cell_id']) for s in layouts}) == 24
    assert sum(not s['parent_geometry_retained_from_r04'] for s in layouts) == 7
    robot = author.old.ReferenceRobot((author.old.REFERENCE/'robot_description.xml').read_text(), (author.old.REFERENCE/'conversion/conversion.sdf').read_text())
    envelope = author.old.WitnessEnvelope(robot)
    geometry_rows = []
    raster_rows = []
    for s in layouts:
        folder = OUTPUT / s['layout_id']
        qc = author.old.full_geometry_checks(s, robot, envelope)
        assert qc == read(folder/'geometry_checks.json') and qc['passed']
        native = read(folder/'reference_witness.json')
        assert native['passed'] and native['selected']['sample_count_checked'] == 901
        geometry_rows.append({'layout_id':s['layout_id'], 'full9_actor_sweep_occlusion_replay':True, 'native_witness_lower_m':native['selected']['continuous_conservative_lower_m']})
        if s['family'] != 'static_avoidance':
            continue
        f = folder / 'raster_and_guide'
        rr = read(f/'raster_guide_checks.json')
        grid, route, _ = author.load_bundle(f/'common_bundle')
        blocked = author.occupied_union(grid)
        guide = author.guide_check(route, blocked, envelope)
        assert guide == rr['guide'] and guide['passed']
        selected_profile = rr['raster_witness']['selected']
        values = np.loadtxt(str(f/'raster_witness.csv'), delimiter=',', skiprows=1)
        assert values.shape == (901, 6)
        expected = author.old.curve(selected_profile['amplitude_m']) if selected_profile['peak_rear_x_m'] is None else author.old.asymmetric_curve(selected_profile['amplitude_m'], selected_profile['peak_rear_x_m'])
        assert np.array_equal(values, np.asarray(expected))
        gap = min(envelope.at(v[2:5]).distance(blocked) for v in values)
        lower = gap - selected_profile['point_lipschitz_per_rear_x'] * .005 - .33
        close(lower, selected_profile['continuous_conservative_lower_m'])
        assert lower > 0 and selected_profile['central_steering_abs_bound_rad'] <= .4964 and selected_profile['central_steering_rate_bound_radps'] <= .75
        native_values = np.loadtxt(str(folder/'reference_witness.csv'), delimiter=',', skiprows=1)
        original_lower = min(envelope.at(v[2:5]).distance(blocked) for v in native_values) - native['selected']['point_lipschitz_per_rear_x']*.005 - .33
        close(original_lower, rr['original_native_witness_conservative_lower_after_raster_m'])
        native_union = author.unary_union([p for _, p in author.old.v2.static_shapes(s)])
        assert native_union.difference(blocked.buffer(1e-12)).area <= 1e-10
        for binding in rr['source_map_and_guide_bindings']:
            assert sha(binding['path']) == binding['sha256']
            assert sha(f/'common_bundle'/Path(binding['path']).name) == binding['sha256']
        raster_rows.append({'layout_id':s['layout_id'], 'original_native_witness_after_raster_m':original_lower, 'selected_raster_witness_lower_m':lower, 'exported_guide_sweep_lower_m':guide['minimum_full_footprint_conservative_gap_m'], 'native_witness_survives':original_lower>0, 'Ackermann_corner_feasibility_claim':False})
    assert len(raster_rows) == 4 and sum(not r['native_witness_survives'] for r in raster_rows) == 1
    deps = []
    for mod in tuple(sys.modules.values()):
        p = getattr(mod, '__file__', None)
        if p and Path(p).is_file() and str(Path(p).resolve()).startswith(str(WORK)) and '/.venv' not in str(p):
            deps.append(author.bind(Path(p)))
    result = {'status':'author_closed_input_replay_pass_independent_review_pending', 'independent_review':False,
        'authenticated_current_resources':len(authenticated), 'unchanged_r04_manifest_resources':r04_count,
        'per_family':dict(counts), 'parent_groups':parent_rows, 'actual_actor_cells':actor_rows, 'geometry_checks':geometry_rows,
        'static_raster_rechecks':raster_rows, 'direct_imported_workspace_sources':sorted(deps,key=lambda x:x['path']),
        'offline_environment':{'executable':sys.executable,'python':platform.python_version(),'numpy':np.__version__,'shapely':shapely.__version__},
        'scope':'literal allocation replay plus declared-model geometry replay, not independent review, runtime qualification, or complete historical registry audit',
        'ROS_started':False,'new_seeds_allocated':0,'execution_authorized':False,'final_outcomes_read':False}
    author.write(report_path, result)
    print(json.dumps({'status':result['status'],'report':str(report_path),'sha256':sha(report_path),'r04_resources':r04_count,'raster':raster_rows},indent=2))

if __name__ == '__main__':
    main()
