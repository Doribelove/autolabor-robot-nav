"""Failure-inclusive descriptive replay of frozen N05 paired development units."""
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path

import rosbag

S = Path(__file__).resolve().parents[1]
OUT = S / 'audits/navigation_n05_terminal_replay'
LEDGER = S / 'queue/navigation_n05_ledger.json'
RELEASE = S / 'contracts/navigation_n05_release.json'
METHODS = ('astra_analytic_route_v1', 'astra_analytic_preview_eligible_n05')


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def map_xy(odom_xy, transform):
    tx, ty, angle = transform
    x, y = odom_xy
    return tx + math.cos(angle)*x - math.sin(angle)*y, ty + math.sin(angle)*x + math.cos(angle)*y


def replay_one(unit, release_sha):
    root = S / 'runs' / unit['run_id']
    receipt_path = root / 'receipt.json'
    receipt = json.loads(receipt_path.read_bytes())
    if (receipt['release_sha256'] != release_sha or receipt['run_id'] != unit['run_id'] or
            receipt['seed'] != unit['seed'] or receipt['method'] != unit['method'] or
            receipt['attempt'] != 1 or receipt['integrity_pass'] is not True or
            unit['integrity_pass'] is not True):
        raise ValueError('N05 immutable attempt/receipt integrity differs: '+unit['run_id'])
    if sha(receipt_path) != unit['receipt']['sha256']:
        raise ValueError('N05 ledger receipt binding differs')
    assessment = json.loads((root/'assessment.json').read_bytes())
    raw = json.loads((root/'collector_raw.json').read_bytes())
    contract = json.loads((root/'contract.json').read_bytes())
    bundle_path = Path(contract['shared_bundle']['path'])
    if sha(bundle_path) != contract['shared_bundle']['sha256']:
        raise ValueError('shared map/route bundle binding differs')
    bundle = json.loads(bundle_path.read_bytes())
    if (assessment['capture_integrity_pass'] is not True or
            assessment['run_id'] != unit['run_id'] or assessment['seed'] != unit['seed'] or
            assessment['method'] != unit['method'] or
            raw['gate']['goal_pose'] != bundle['goal_xy_yaw']):
        raise ValueError('N05 assessment or goal identity differs')
    trace = Counter(); reasons = Counter(); eligible = Counter(); source = Counter()
    terminal_odom = None
    start_ns, end_ns = int(raw['start_ns']), int(raw['end_ns'])
    if end_ns-start_ns > 22_000_000_000:
        raise ValueError('N05 cap exceeded')
    with rosbag.Bag(str(root/'fixture.bag')) as bag:
        for topic,msg,_ in bag.read_messages(topics=['/astra/route_runtime/trace','/astra/route_runtime/candidates','/odom']):
            if topic == '/odom':
                if msg.header.stamp.to_nsec() == end_ns:
                    terminal_odom = (msg.pose.pose.position.x,msg.pose.pose.position.y)
                continue
            data = json.loads(msg.data)
            stamp = int(data['stamp_ns'])
            if not start_ns <= stamp < end_ns:
                continue
            if topic.endswith('/trace'):
                trace[str(data['executed'])] += 1
                reasons[str(data.get('stop_reason'))] += 1
                if data.get('transaction_deadline_dropped'):
                    source['transaction_deadline_dropped'] += 1
            else:
                source['candidate_decisions'] += 1
                audit = data.get('preview_selection_audit')
                if audit is not None:
                    if audit.get('true_backup_mask_changed') is not False:
                        raise ValueError('N05 treatment altered hard backup mask')
                    permitted = audit['selection_eligibility']
                    backup = audit['backup_mask']
                    if len(permitted)!=8 or len(backup)!=8 or any(permitted[i] and not backup[i] for i in range(8)):
                        raise ValueError('N05 selection eligibility escaped original backup mask')
                    eligible['moving_eligible_count_sum'] += sum(permitted[1:])
                    eligible['moving_backup_count_sum'] += sum(backup[1:])
                    if audit['original_analytic'] != audit['chosen']:
                        eligible['selection_changed_decisions'] += 1
                    if audit['chosen']==0:
                        eligible['selected_stop_decisions'] += 1
    if terminal_odom is None:
        raise ValueError('exact terminal source odom missing: '+unit['run_id'])
    map_position = map_xy(terminal_odom,bundle['map_from_odom_xy_yaw'])
    goal = bundle['goal_xy_yaw']
    return dict(run_id=unit['run_id'],seed=unit['seed'],method=unit['method'],scene_id=contract['scene_id'],
                task_phase=assessment['task']['task_phase'],
                valid_completed=assessment['valid_completed'],
                restricted_goal_time_s=assessment['restricted_goal_time_s'],
                safety_within_declared_engineering_scope_pass=assessment['safety_within_declared_engineering_scope_pass'],
                trace_ticks=sum(trace.values()),executed_stop_ticks=trace['0'],
                backup_revalidation_failed_ticks=reasons['backup_revalidation_failed'],
                backup_stop_requested_ticks=reasons['backup_stop_requested'],
                transaction_deadline_drop_ticks=reasons['transaction_deadline_drop'],
                candidate_decisions=source['candidate_decisions'],
                treatment_selection_changed_decisions=eligible['selection_changed_decisions'],
                treatment_moving_backup_count_sum=eligible['moving_backup_count_sum'],
                treatment_moving_eligible_count_sum=eligible['moving_eligible_count_sum'],
                terminal_map_goal_xy_distance_m=round(math.dist(map_position,goal[:2]),9),
                terminal_source_ns=end_ns,
                receipt_sha256=sha(receipt_path),assessment_sha256=sha(root/'assessment.json'),
                bag_sha256=sha(root/'fixture.bag'),bundle_sha256=sha(bundle_path))


def main():
    if OUT.exists():
        raise FileExistsError('N05 terminal interpretation identity exists')
    ledger = json.loads(LEDGER.read_bytes())
    release_sha = sha(RELEASE)
    if (ledger['status'] != 'terminal' or ledger['release_sha256'] != release_sha or
            len(ledger['units']) != 24):
        raise ValueError('N05 must be terminal at exact release before interpretation')
    consumed = [u for u in ledger['units'] if u['status']=='consumed']
    retired = [u for u in ledger['units'] if u['status']=='retired_unrun']
    rows = [replay_one(u,release_sha) for u in consumed if u.get('integrity_pass') is True]
    bad = [u for u in consumed if u.get('integrity_pass') is not True]
    by_seed = defaultdict(dict)
    for row in rows:
        if row['method'] not in METHODS or row['method'] in by_seed[row['seed']]:
            raise ValueError('N05 pair identity differs')
        by_seed[row['seed']][row['method']] = row
    pairs = []
    for seed,items in sorted(by_seed.items()):
        if set(items)!=set(METHODS):
            continue
        control,treatment=(items[m] for m in METHODS)
        if control['scene_id']!=treatment['scene_id']:
            raise ValueError('N05 paired scene differs')
        pairs.append(dict(seed=seed,scene_id=control['scene_id'],
                          control_run_id=control['run_id'],treatment_run_id=treatment['run_id'],
                          control_task_phase=control['task_phase'],treatment_task_phase=treatment['task_phase'],
                          restricted_time_treatment_minus_control_s=treatment['restricted_goal_time_s']-control['restricted_goal_time_s'],
                          final_goal_distance_treatment_minus_control_m=treatment['terminal_map_goal_xy_distance_m']-control['terminal_map_goal_xy_distance_m'],
                          stop_fraction_treatment_minus_control=treatment['executed_stop_ticks']/treatment['trace_ticks']-control['executed_stop_ticks']/control['trace_ticks']))
    OUT.mkdir()
    for filename,data in (('episodes.csv',rows),('complete_pairs.csv',pairs)):
        if not data:
            continue
        with (OUT/filename).open('x',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(data[0]))
            writer.writeheader();writer.writerows(data)
    report=dict(status='N05_terminal_development_descriptive_replay',
                release_sha256=release_sha,ledger_sha256=sha(LEDGER),
                consumed=len(consumed),integrity_valid=len(rows),
                integrity_failed=len(bad),retired_unrun=len(retired),
                complete_pairs=len(pairs),
                method_outcomes={method:dict(valid_rows=sum(r['method']==method for r in rows),
                    completed=sum(r['method']==method and r['valid_completed'] is True for r in rows),
                    timeout=sum(r['method']==method and r['task_phase']=='timeout' for r in rows)) for method in METHODS},
                all_safety_scoped_pass=all(r['safety_within_declared_engineering_scope_pass'] is True for r in rows),
                paired_mean_restricted_time_difference_s=None if not pairs else sum(p['restricted_time_treatment_minus_control_s'] for p in pairs)/len(pairs),
                rows=rows,pairs=pairs,
                scope='six previously used development parent geometries with two paired fresh seeds each; incomplete blocks and all algorithm failures retained',
                formal_result=False,performance_improvement_demonstrated=False,
                general_safety_or_deployment_claim=False)
    (OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(status=report['status'],consumed=len(consumed),valid=len(rows),
                          retired=len(retired),complete_pairs=len(pairs),
                          outcomes=report['method_outcomes'])))


if __name__=='__main__':
    main()
