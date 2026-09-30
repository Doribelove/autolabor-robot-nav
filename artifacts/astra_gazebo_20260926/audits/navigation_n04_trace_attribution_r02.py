"""Versioned N04 descriptive replay with map-frame terminal distance."""
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import rosbag

S = Path(__file__).resolve().parents[1]
OUT = S / 'audits/navigation_n04_trace_attribution_r02'
OLD = S / 'audits/navigation_n04_trace_attribution'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def map_pose(odom_xy, transform):
    tx, ty, angle = transform
    ox, oy = odom_xy
    return tx + math.cos(angle) * ox - math.sin(angle) * oy, ty + math.sin(angle) * ox + math.cos(angle) * oy


def distance_to_goal(odom_xy, transform, goal):
    x, y = map_pose(odom_xy, transform)
    return math.hypot(x - goal[0], y - goal[1])


def main():
    if OUT.exists():
        raise FileExistsError('N04 attribution r02 exists')
    ledger_path = S / 'queue/navigation_n04_ledger.json'
    ledger = json.loads(ledger_path.read_bytes())
    rows = []
    for unit in ledger['units']:
        if unit['method'] != 'astra_analytic_route_v1' or unit['status'] != 'consumed' or unit.get('integrity_pass') is not True:
            continue
        root = S / 'runs' / unit['run_id']
        assessment = json.loads((root / 'assessment.json').read_bytes())
        raw = json.loads((root / 'collector_raw.json').read_bytes())
        contract = json.loads((root / 'contract.json').read_bytes())
        bundle_path = Path(contract['shared_bundle']['path'])
        if sha(bundle_path) != contract['shared_bundle']['sha256']:
            raise ValueError('common route bundle changed')
        bundle = json.loads(bundle_path.read_bytes())
        if (assessment['capture_integrity_pass'] is not True or
                assessment['task']['task_phase'] != 'timeout' or
                raw['gate']['goal_pose'] != bundle['goal_xy_yaw']):
            raise ValueError('frozen N04 task/goal identity differs')
        start_ns, end_ns = int(raw['start_ns']), int(raw['end_ns'])
        if end_ns - start_ns != 22_000_000_000:
            raise ValueError('N04 restricted window changed')
        all_chosen = Counter(); all_reasons = Counter(); bounded_chosen = Counter(); bounded_reasons = Counter()
        tail_odom = None; terminal_odom = None; tail_stamp = None
        with rosbag.Bag(str(root / 'fixture.bag')) as bag:
            for topic, msg, _ in bag.read_messages(topics=['/astra/route_runtime/trace', '/odom']):
                if topic == '/odom':
                    stamp = msg.header.stamp.to_nsec()
                    xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)
                    if tail_stamp is None or stamp > tail_stamp:
                        tail_stamp, tail_odom = stamp, xy
                    if stamp == end_ns:
                        terminal_odom = xy
                    continue
                record = json.loads(msg.data)
                reason = str(record.get('stop_reason'))
                all_chosen[str(record['executed'])] += 1
                all_reasons[reason] += 1
                if start_ns <= int(record['stamp_ns']) < end_ns:
                    bounded_chosen[str(record['executed'])] += 1
                    bounded_reasons[reason] += 1
        if terminal_odom is None or tail_odom is None or tail_stamp < end_ns:
            raise ValueError('exact terminal and bag-tail odom required')
        transform = bundle['map_from_odom_xy_yaw']
        goal = raw['gate']['goal_pose']
        rows.append(dict(run_id=unit['run_id'], seed=unit['seed'],
                         all_bag_trace_ticks=sum(all_chosen.values()),
                         all_bag_executed_stop_ticks=all_chosen['0'],
                         all_bag_backup_revalidation_failed_ticks=all_reasons['backup_revalidation_failed'],
                         all_bag_backup_stop_requested_ticks=all_reasons['backup_stop_requested'],
                         all_bag_transaction_deadline_drop_ticks=all_reasons['transaction_deadline_drop'],
                         restricted_half_open_trace_ticks=sum(bounded_chosen.values()),
                         restricted_half_open_executed_stop_ticks=bounded_chosen['0'],
                         restricted_half_open_backup_revalidation_failed_ticks=bounded_reasons['backup_revalidation_failed'],
                         restricted_half_open_backup_stop_requested_ticks=bounded_reasons['backup_stop_requested'],
                         restricted_half_open_transaction_deadline_drop_ticks=bounded_reasons['transaction_deadline_drop'],
                         terminal_source_ns=end_ns, bag_tail_odom_source_ns=tail_stamp,
                         terminal_map_goal_xy_distance_m=round(distance_to_goal(terminal_odom, transform, goal), 9),
                         bag_tail_map_goal_xy_distance_m=round(distance_to_goal(tail_odom, transform, goal), 9),
                         map_from_odom_xy_yaw=json.dumps(transform, separators=(',', ':')),
                         bundle_sha256=sha(bundle_path), bag_sha256=sha(root / 'fixture.bag'),
                         assessment_sha256=sha(root / 'assessment.json'),
                         collector_raw_sha256=sha(root / 'collector_raw.json')))
    if len(rows) != 5:
        raise ValueError('five pre-pause complete N04 analytic cases required')
    totals = {key:sum(row[key] for row in rows) for key in rows[0] if key.endswith('_ticks')}
    if (totals['all_bag_trace_ticks'], totals['all_bag_executed_stop_ticks'],
        totals['all_bag_backup_revalidation_failed_ticks'],
        totals['restricted_half_open_trace_ticks'], totals['restricted_half_open_executed_stop_ticks'],
        totals['restricted_half_open_backup_revalidation_failed_ticks']) != (2190,1509,1017,2182,1501,1014):
        raise ValueError('independently observed N04 trace totals differ')
    OUT.mkdir()
    with (OUT / 'five_valid_analytic_replay.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    report = dict(status='N04_partial_development_descriptive_r02_frame_corrected',
                  old_r01_invalid_distance=dict(script_sha256=sha(S/'audits/navigation_n04_trace_attribution.py'),
                                                report_sha256=sha(OLD/'report.json'),
                                                error='odom-frame position was directly subtracted from map-frame goal'),
                  valid_analytic_attempts=5, paused_assessment_interrupted_excluded=1,
                  retired_unrun_analytic_excluded=6,
                  distance_frame='map',distance_time='exact shared terminal odom source timestamp',
                  trace_scope='both all-bag descriptive and half-open shared [start,end) window, never interchangeable',
                  totals=totals,rows=rows,
                  source_ledger_sha256=sha(ledger_path),
                  interpretation='Historical partial development mechanism counts only; no causal effect, completed matrix, safety or performance superiority claim.',
                  formal_result=False,N04_rerun_allowed=False)
    (OUT/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(dict(status=report['status'], totals=totals,
                          terminal_map_distances_m=[r['terminal_map_goal_xy_distance_m'] for r in rows])))


if __name__ == '__main__':
    main()
