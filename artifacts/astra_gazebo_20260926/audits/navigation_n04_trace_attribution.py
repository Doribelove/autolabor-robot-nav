"""Descriptive replay of valid N04 analytic development traces; no outcome repair."""
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path

import rosbag

S = Path(__file__).resolve().parents[1]
OUT = S / 'audits/navigation_n04_trace_attribution'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUT.exists():
        raise FileExistsError('N04 attribution already exists')
    rows = []
    ledger = json.loads((S / 'queue/navigation_n04_ledger.json').read_bytes())
    for unit in ledger['units']:
        if unit['method'] != 'astra_analytic_route_v1' or unit['status'] != 'consumed' or not unit.get('integrity_pass'):
            continue
        root = S / 'runs' / unit['run_id']
        assessment = json.loads((root / 'assessment.json').read_bytes())
        raw = json.loads((root / 'collector_raw.json').read_bytes())
        if assessment['capture_integrity_pass'] is not True or assessment['task']['task_phase'] != 'timeout':
            raise ValueError('unexpected N04 analytic source status')
        chosen = Counter(); stops = Counter(); selection = Counter(); transport = Counter(); odom = []
        with rosbag.Bag(str(root / 'fixture.bag')) as bag:
            for topic, msg, _ in bag.read_messages(topics=['/astra/route_runtime/trace', '/odom']):
                if topic == '/odom':
                    odom.append((msg.pose.pose.position.x, msg.pose.pose.position.y))
                    continue
                record = json.loads(msg.data)
                chosen[str(record['executed'])] += 1
                stops[str(record.get('stop_reason'))] += 1
                selection[str(record.get('selection_reason'))] += 1
                if record.get('transaction_deadline_dropped'):
                    transport['transaction_deadline_dropped'] += 1
                if record.get('inference_deadline_dropped'):
                    transport['inference_deadline_dropped'] += 1
        if not odom:
            raise ValueError('odom missing')
        goal = raw['gate']['goal_pose']
        distance = math.hypot(odom[-1][0] - goal[0], odom[-1][1] - goal[1])
        rows.append(dict(run_id=unit['run_id'], seed=unit['seed'], task_phase='timeout',
                         trace_ticks=sum(chosen.values()), executed_stop=chosen['0'],
                         backup_revalidation_failed=stops['backup_revalidation_failed'],
                         backup_stop_requested=stops['backup_stop_requested'],
                         no_prediction_model_backup_pass=selection['no_prediction_model_backup_pass'],
                         transaction_deadline_dropped=transport['transaction_deadline_dropped'],
                         inference_deadline_dropped=transport['inference_deadline_dropped'],
                         final_odom_goal_xy_distance_m=round(distance, 6),
                         bag_sha256=sha(root / 'fixture.bag'),
                         assessment_sha256=sha(root / 'assessment.json'),
                         collector_raw_sha256=sha(root / 'collector_raw.json')))
    if len(rows) != 5:
        raise ValueError('only five pre-pause valid N04 analytic results expected')
    OUT.mkdir()
    fields = list(rows[0])
    with (OUT / 'valid_analytic_ticks.csv').open('x', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)
    total = sum(r['trace_ticks'] for r in rows)
    report = dict(status='N04_partial_development_descriptive_trace_replay_only',
                  valid_analytic_attempts=5, paused_interrupted_excluded=1,
                  retired_unrun_excluded=6,
                  trace_ticks=total,
                  executed_stop_ticks=sum(r['executed_stop'] for r in rows),
                  backup_revalidation_failed_ticks=sum(r['backup_revalidation_failed'] for r in rows),
                  backup_stop_requested_ticks=sum(r['backup_stop_requested'] for r in rows),
                  transaction_deadline_dropped_ticks=sum(r['transaction_deadline_dropped'] for r in rows),
                  source_ledger_sha256=sha(S / 'queue/navigation_n04_ledger.json'),
                  rows=rows,
                  interpretation='Observed N04 source-stamped trace category counts only; reasons can overlap executed-stop and cannot establish causal attribution or N05 benefit.',
                  N04_rerun_allowed=False, formal_result=False)
    (OUT / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(dict(status=report['status'], ticks=total,
                          executed_stop_ticks=report['executed_stop_ticks'],
                          backup_revalidation_failed_ticks=report['backup_revalidation_failed_ticks'])))


if __name__ == '__main__':
    main()
