"""Failure-inclusive descriptive replay of frozen N05 paired development units."""
import csv
import hashlib
import json
import math
from collections import Counter, defaultdict
from pathlib import Path
import sys

import rosbag

S = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(S/'workspace/scripts/astra_gazebo'))
import run_navigation_n05 as manager
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


def validate_consumed_receipt(unit, release_sha):
    binding = unit.get('receipt')
    if type(binding) is not dict or type(binding.get('path')) is not str or type(binding.get('sha256')) is not str:
        raise ValueError('N05 consumed unit lacks receipt binding: '+unit['run_id'])
    path = Path(binding['path'])
    expected = (S/'runs'/unit['run_id']/'receipt.json') if path.name == 'receipt.json' else (S/'queue'/(unit['run_id']+'_preparation_failure.json'))
    if path.resolve()!=expected.resolve() or sha(path)!=binding['sha256']:
        raise ValueError('N05 consumed receipt path/hash differs: '+unit['run_id'])
    receipt = json.loads(path.read_bytes())
    if ((receipt.get('stage'),receipt.get('run_id'),receipt.get('seed'),receipt.get('method'),
         receipt.get('attempt'),receipt.get('release_sha256')) !=
        (manager.NAME,unit['run_id'],unit['seed'],unit['method'],1,release_sha) or
        receipt.get('retry_allowed') is not False or
        type(receipt.get('integrity_pass')) is not bool or
        receipt['integrity_pass'] is not unit.get('integrity_pass') or
        receipt.get('task_outcome') != unit.get('task_outcome') or
        receipt.get('task_outcome') not in (None,'complete','timeout','fault','stopped')):
        raise ValueError('N05 consumed receipt identity/outcome differs: '+unit['run_id'])
    report_binding = receipt.get('report')
    if report_binding is not None:
        report_path = S/'runs'/unit['run_id']/'assessment.json'
        if (type(report_binding) is not dict or
                report_binding.get('path') != str(report_path.resolve()) or
                report_binding.get('sha256') != sha(report_path)):
            raise ValueError('N05 consumed assessment binding differs: '+unit['run_id'])
        report = json.loads(report_path.read_bytes())
        if (type(report) is not dict or
                (report.get('run_id'),report.get('seed'),report.get('method')) !=
                (unit['run_id'],unit['seed'],unit['method']) or
                report.get('task',{}).get('task_phase') != receipt.get('task_outcome')):
            raise ValueError('N05 consumed assessment identity/outcome differs: '+unit['run_id'])
    if receipt['integrity_pass'] is True:
        if receipt.get('error') is not None or receipt.get('task_outcome') is None:
            raise ValueError('N05 valid consumed receipt error/outcome differs: '+unit['run_id'])
        if report_binding is None:
            raise ValueError('N05 valid consumed assessment missing: '+unit['run_id'])
    elif report_binding is None and (type(receipt.get('error')) is not str or not receipt['error']):
        raise ValueError('N05 failed consumed receipt lacks error/report evidence: '+unit['run_id'])
    return receipt


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
    required=(root/'assessment.json',root/'collector_raw.json',root/'contract.json',root/'fixture.bag')
    artifacts={}
    for item in receipt['artifacts']:
        path=Path(item['path']).resolve()
        if path in artifacts:raise ValueError('duplicate N05 receipt artifact path')
        artifacts[path]=item['sha256']
    for path in required:
        if artifacts.get(path.resolve())!=sha(path):
            raise ValueError('N05 consumed raw input absent from immutable receipt: '+str(path))
    if (receipt.get('report') != {'path':str((root/'assessment.json').resolve()),
                                 'sha256':sha(root/'assessment.json')}
            or receipt.get('error') is not None):
        raise ValueError('N05 valid assessment receipt does not bind report or has error')
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
            raw['gate']['goal_pose'] != bundle['goal_xy_yaw'] or
            receipt['task_outcome'] != assessment['task']['task_phase'] or
            (contract['run_id'],contract['seed'],contract['method']) !=
               (unit['run_id'],unit['seed'],unit['method'])):
        raise ValueError('N05 assessment or goal identity differs')
    trace = Counter(); reasons = Counter(); eligible = Counter(); source = Counter()
    terminal_odom = []
    start_ns, end_ns = raw['start_ns'], raw['end_ns']
    if (type(start_ns) is not int or type(end_ns) is not int or start_ns<=0 or
            end_ns<=start_ns or end_ns-start_ns>22_000_000_000 or
            raw['gate']['source_ns']!=start_ns or
            assessment['task']['terminal_source_ns']!=end_ns):
        raise ValueError('N05 positive exact shared gate/terminal source window differs')
    with rosbag.Bag(str(root/'fixture.bag')) as bag:
        for topic,msg,_ in bag.read_messages(topics=['/astra/route_runtime/trace','/astra/route_runtime/candidates','/odom']):
            if topic == '/odom':
                if msg.header.stamp.to_nsec() == end_ns:
                    if msg.header.frame_id.lstrip('/')!='odom' or msg.child_frame_id.lstrip('/')!='base_link':
                        raise ValueError('N05 terminal odom frame differs')
                    terminal_odom.append((msg.pose.pose.position.x,msg.pose.pose.position.y))
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
                if unit['method']==METHODS[1]:
                    if (type(audit) is not dict or audit.get('schema')!='astra_analytic_preview_eligibility_n05_v1' or
                            audit.get('method')!=METHODS[1] or
                            audit.get('true_backup_mask_changed') is not False or
                            audit.get('physical_safety_certified') is not False or
                            any(type(audit.get(k)) is not int or not 0<=audit[k]<8
                                for k in ('chosen','original_analytic')) or
                            (audit.get('proposed') is not None and
                             (type(audit['proposed']) is not int or not 0<=audit['proposed']<8))):
                        raise ValueError('N05 treatment preview audit missing/changed hard backup mask')
                    permitted=audit['selection_eligibility'];backup=audit['backup_mask']
                    full=audit['full_preview_clearances_m']
                    if (any(type(items) is not list or len(items)!=8 for items in (permitted,backup,full)) or
                            any(type(v) is not bool for v in permitted+backup) or
                            any(type(v) not in (float,int) or not math.isfinite(v) for v in full) or
                            backup!=data['mask'] or full!=data['full_preview_clearances_m'] or
                            permitted != [backup[i] and (i==0 or full[i]>=0.) for i in range(8)]):
                        raise ValueError('N05 treatment eligibility is not exact full-preview predicate')
                    if (any(permitted) and not permitted[audit['chosen']]) or (not any(permitted) and audit['chosen']!=0):
                        raise ValueError('N05 treatment choice escaped eligibility/fallback')
                    eligible['moving_eligible_count_sum'] += sum(permitted[1:])
                    eligible['moving_backup_count_sum'] += sum(backup[1:])
                    if audit['original_analytic'] != audit['chosen']:
                        eligible['selection_changed_decisions'] += 1
                    if audit['chosen']==0:
                        eligible['selected_stop_decisions'] += 1
                elif audit is not None:
                    raise ValueError('N05 control unexpectedly has treatment-only preview audit')
    if len(terminal_odom)!=1:
        raise ValueError('unique exact terminal source odom missing: '+unit['run_id'])
    map_position = map_xy(terminal_odom[0],bundle['map_from_odom_xy_yaw'])
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
                transaction_deadline_dropped_flag_ticks=source['transaction_deadline_dropped'],
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
    release,prepared,_=manager.verify_release(RELEASE,release_sha)
    if (ledger['status'] != 'terminal' or ledger['release_sha256'] != release_sha or
            ledger['stage']!=manager.NAME or len(ledger['units']) != 24 or
            [(u['run_id'],u['seed'],u['method']) for u in ledger['units']]!=
              [(u['run_id'],u['seed'],u['method']) for u in prepared['units']] or
            any(u['status'] not in ('consumed','retired_unrun') for u in ledger['units'])):
        raise ValueError('N05 must be terminal at exact release before interpretation')
    consumed = [u for u in ledger['units'] if u['status']=='consumed']
    retired = [u for u in ledger['units'] if u['status']=='retired_unrun']
    receipts = {u['run_id']:validate_consumed_receipt(u,release_sha) for u in consumed}
    rows = [replay_one(u,release_sha) for u in consumed if u.get('integrity_pass') is True]
    bad = [u for u in consumed if u.get('integrity_pass') is not True]
    bad_rows = [dict(run_id=u['run_id'],seed=u['seed'],method=u['method'],
                     task_outcome=receipts[u['run_id']]['task_outcome'],
                     error=receipts[u['run_id']]['error'],
                     receipt_sha256=u['receipt']['sha256']) for u in bad]
    retired_rows = [dict(run_id=u['run_id'],seed=u['seed'],method=u['method']) for u in retired]
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
                          stop_fraction_treatment_minus_control=None if not treatment['trace_ticks'] or not control['trace_ticks'] else treatment['executed_stop_ticks']/treatment['trace_ticks']-control['executed_stop_ticks']/control['trace_ticks']))
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
                all_safety_scoped_pass=None if not rows else all(r['safety_within_declared_engineering_scope_pass'] is True for r in rows),
                paired_mean_restricted_time_difference_s=None if not pairs else sum(p['restricted_time_treatment_minus_control_s'] for p in pairs)/len(pairs),
                rows=rows,pairs=pairs,integrity_failed_rows=bad_rows,
                retired_unrun_rows=retired_rows,
                scope='six previously used development parent geometries with two paired fresh seeds each; incomplete blocks and all algorithm failures retained. Pair endpoint differences compare each method at its own terminal source time, not the same elapsed state.',
                formal_result=False,performance_improvement_demonstrated=False,
                general_safety_or_deployment_claim=False)
    (OUT/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(status=report['status'],consumed=len(consumed),valid=len(rows),
                          retired=len(retired),complete_pairs=len(pairs),
                          outcomes=report['method_outcomes'])))


if __name__=='__main__':
    main()
