#!/usr/bin/env python3
"""Verify saved execution of installed C++ generator and raw-command matches."""
import hashlib
import json
import struct
from pathlib import Path

P = Path(__file__).resolve().parent
rows = [json.loads(s) for s in (P / 'native_probe_output.jsonl').read_text().splitlines()]
odom = [json.loads(s) for s in (P / 'source_odometry.jsonl').read_text().splitlines()]
by_case = {r['case']: r for r in rows}
checks = []

def check(name, condition):
    assert condition, name
    checks.append(name)

def f32(v):
    return struct.unpack('f', struct.pack('f', v))[0]

check('all_1212_requested_native_cases_completed', len(rows) == len(by_case) == 1212)
for sign in ('positive', 'negative'):
    row = by_case[sign + '_a0.0']
    check(sign + '_zero_acc_inverts_bounds', row['dynamic_lower'] > row['dynamic_upper'])
    check(sign + '_one_requested_sample_still_has_two_and_nonzero_trajectory',
          len(row['y_samples']) == 2 and any(v['vy'] != 0 and v['has_generator_accepted_trajectory'] for v in row['y_samples']))
check('zero_vy_control_single_zero', by_case['zero_a0.0']['y_samples'] == [{'vy': 0, 'has_generator_accepted_trajectory': True}])
real = [r for r in rows if r['case'].startswith('odom_')]
check('all_393_actual_odom_nonzero_with_acc0', sum(any(y['vy'] != 0 for y in r['y_samples']) for r in real if r['acc_lim_y'] == 0) == 393)
check('acc06_not_a_sufficient_fix_for_all_actual_odom', sum(any(y['vy'] != 0 for y in r['y_samples']) for r in real if r['acc_lim_y'] == .6) == 1)
check('acc25_zero_only_for_all_393_actual_odom', sum(all(y['vy'] == 0 for y in r['y_samples']) for r in real if r['acc_lim_y'] == 2.5) == 393)
for label in ('acc25_above_domain', 'acc25_negative_outside'):
    check(label + '_still_admits_nonzero_trajectory', any(y['vy'] != 0 and y['has_generator_accepted_trajectory'] for y in by_case[label + '_a2.5']['y_samples']))
matches = []
commands = json.loads((P / 'initial_timing.json').read_text())['method_commands']
for cmd in commands[1:]:
    matched = [o for o in odom if o['source_ns'] <= cmd['bag_receipt_ns'] and f32(o['vy']) == cmd['vy']]
    check('command_' + str(cmd['bag_receipt_ns']) + '_unique_raw_odom_float32_vy', len(matched) == 1)
    row = by_case['odom_' + str(matched[0]['source_ns']) + '_a0.0']
    check('command_' + str(cmd['bag_receipt_ns']) + '_native_nonzero_sample_accepted', any(y['vy'] == cmd['vy'] and y['has_generator_accepted_trajectory'] for y in row['y_samples']))
    matches.append({'command_bag_receipt_ns': cmd['bag_receipt_ns'], 'command_vy': cmd['vy'], 'matching_odom': matched[0], 'match_is_not_a_planner_generation_timestamp': True})
inputs = [P/'native_probe_inputs.txt', P/'native_probe_output.jsonl', P/'source_odometry.jsonl', P/'initial_timing.json', P/'native_dynamic_window_probe.cpp', P/'native_dynamic_window_probe', P/'native_probe.maps.txt']
result = {'pass': True, 'checks': checks, 'check_count': len(checks), 'native_cases': len(rows), 'actual_raw_matches': matches, 'inputs': [{'path': str(p), 'sha256': hashlib.sha256(p.read_bytes()).hexdigest()} for p in inputs], 'qualification': 'offline generator mechanism only; no costmap winner or live repair qualification'}
(P/'native_probe_verification.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
print(json.dumps({'pass':True,'checks':len(checks),'native_cases':len(rows)}))
