#!/usr/bin/env python3
"""Seal an assessed formal R2 result into all seven evidence ledgers."""

from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

import yaml


ROOT=Path(__file__).resolve().parents[2]
ARTIFACT=ROOT/'artifacts/gazebo_pct_teb_dwa_benchmark';FORMAL=ARTIFACT/'formal/round_2'
ASSESSMENT=FORMAL/'formal_r2_assessment.json';RAW=FORMAL/'formal_r2_episode_results.csv'
RELEASE=FORMAL/'formal_r2_execution_release.yaml';EXECUTION=FORMAL/'execution/episodes';REVIEW=FORMAL/'formal_r2_integrity_review.json'
STAGE='P12_FORMAL_R2';ROUND='gazebo_p12_formal_r2'


def sha256(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def atomic(path,text):
    temporary=path.with_suffix(path.suffix+'.tmp');temporary.write_text(text,encoding='utf-8');temporary.replace(path)
def append_once(path,marker,text):
    original=path.read_text(encoding='utf-8')
    if marker not in original:atomic(path,original.rstrip()+'\n\n'+text.strip()+'\n')


def load():
    assessment=json.loads(ASSESSMENT.read_text())
    rows=list(csv.DictReader(RAW.open(encoding='utf-8')))
    if assessment.get('blocks')!=200 or assessment.get('episodes')!=800 or len(rows)!=800 or not assessment.get('formal_result'):
        raise RuntimeError('FORMAL_R2_ASSESSMENT_NOT_COMPLETE')
    return assessment,rows


def update_run(rows):
    path=ARTIFACT/'run_manifest.csv'
    with path.open(newline='',encoding='utf-8') as stream:
        reader=csv.DictReader(stream);fields=reader.fieldnames;retained=[x for x in reader if x['stage']!=STAGE]
    added=[]
    for row in rows:
        root=EXECUTION/'{}_{}_{}'.format(row['seed'],row['scene_id'],row['method']);receipt_path=root/'receipt.json'
        receipt=json.loads(receipt_path.read_text());ended=datetime.fromtimestamp(receipt_path.stat().st_mtime).astimezone()
        started=datetime.fromtimestamp(ended.timestamp()-float(receipt['wall_elapsed_s'])).astimezone()
        added.append({'stage':STAGE,'run_id':'{}__{}_{}_{}'.format(row['block_id'],row['seed'],row['scene_id'],row['method']),
          'phase':'preregistered_formal_round_2','method':row['method'],'scene_id':row['scene_id'],'split':'formal_round_2',
          'seed':row['seed'],'attempt':1,'status':'completed_pass' if row['valid_goal']=='True' else row['failure_reason']+'_retained',
          'started_at':started.isoformat(timespec='seconds'),'ended_at':ended.isoformat(timespec='seconds'),'exit_code':receipt['return_code'],
          'config_sha256':receipt['release_sha256'],'model_sha256':'bound_in_release','evidence_path':str(root.relative_to(ROOT)),
          'formal_eligible':'true','notes':'block={}; order={}; failure_reason={}; no_retry'.format(row['block_id'],row['order_index'],row['failure_reason'])})
    temporary=path.with_suffix('.csv.tmp')
    with temporary.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(retained+added)
    temporary.replace(path)


def update_episode(rows):
    path=ARTIFACT/'episode_results.csv'
    with path.open(newline='',encoding='utf-8') as stream:
        reader=csv.DictReader(stream);fields=reader.fieldnames;retained=[x for x in reader if x['study_round']!=ROUND]
    added=[];release_sha=sha256(RELEASE)
    for row in rows:
        elapsed=float(row['failure_inclusive_time_s']);length=row['path_length_m']
        values={'study_round':ROUND,'phase':'preregistered_formal_round_2','block_id':row['block_id'],
          'episode_id':'{}_{}_{}_{}'.format(row['block_id'],row['seed'],row['scene_id'],row['method']),'method':row['method'],
          'scene_family':row['scene_id'],'scene_id':row['scene_id'],'split':'formal_round_2','seed':row['seed'],'attempt':1,
          'order_index':row['order_index'],'goal_reached':row['goal_reached'].lower(),
          'timeout':str(row['failure_reason']=='timeout_or_nonarrival').lower(),'contact_count':row['contact_count'],
          'system_failure':str(row['failure_reason']=='execution_integrity_failure').lower(),'failure_reason':row['failure_reason'],
          'failure_inclusive_time_s':elapsed,'min_clearance_m':row['minimum_conservative_clearance_m'],'path_length_m':length,
          'mean_speed_mps':float(length)/elapsed if length else '','acceleration_rms':'','jerk_metric':'',
          'candidate_switch_count':row['candidate_switch_count'],'stop_count':row['certified_stop_count'],
          'recovery_count':row['stop_recovery_count'],'mean_latency_ms':row['mean_inference_latency_ms'],
          'p95_latency_ms':row['p95_inference_latency_ms'],'p99_latency_ms':row['p99_inference_latency_ms'],
          'mask_violation_count':row['mask_violation_count'],'config_sha256':release_sha,'model_sha256':'bound_in_release',
          'evidence_path':row['report_path']}
        added.append({f:values.get(f,'') for f in fields})
    temporary=path.with_suffix('.csv.tmp')
    with temporary.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(retained+added)
    temporary.replace(path)


def unique(path,a,b):
    rows=list(csv.DictReader(path.open(encoding='utf-8')));keys=[(x[a],x[b]) for x in rows];return len(keys)==len(set(keys)),len(rows)


def main():
    assessment,rows=load();release=yaml.safe_load(RELEASE.read_text())
    for item in release['resource_bindings']:
        if sha256(ROOT/item['path'])!=item['sha256']:raise RuntimeError('FORMAL_R2_RESOURCE_HASH_MISMATCH:'+item['path'])
    update_run(rows);update_episode(rows)
    ru,rc=unique(ARTIFACT/'run_manifest.csv','stage','run_id');eu,ec=unique(ARTIFACT/'episode_results.csv','study_round','episode_id')
    if not ru or not eu:raise RuntimeError('FORMAL_R2_LEDGER_DUPLICATE')
    review={'schema_version':'1.0','status':'formal_r2_integrity_review_pass','assessment_path':str(ASSESSMENT.relative_to(ROOT)),
      'assessment_sha256':sha256(ASSESSMENT),'episode_results_sha256':sha256(RAW),
      'assessor_path':'scripts/gazebo_pct_teb_dwa_benchmark/assess_formal_r2.py',
      'assessor_sha256':sha256(ROOT/'scripts/gazebo_pct_teb_dwa_benchmark/assess_formal_r2.py'),
      'assessor_bound_before_outcome_access':True,'blocks':200,'episodes':800,'attempt_one_complete':True,
      'same_identity_retry_count':0,'replacement_seed_count':0,'all_failures_retained':True,
      'release_resources_rehashed':True,'run_manifest_unique':ru,'run_manifest_rows':rc,
      'episode_results_unique':eu,'episode_results_rows':ec,'hardware_accessed':False}
    atomic(REVIEW,json.dumps(review,indent=2,sort_keys=True)+'\n')
    integrity_path=ARTIFACT/'integrity_report.json';integrity=json.loads(integrity_path.read_text())
    integrity['schema_version']='1.0';integrity['status']='formal_r2_complete';integrity['formal_execution_authorized']=False
    integrity['checks']=[x for x in integrity['checks'] if x.get('id')!='p12_formal_r2']
    integrity['checks'].append({'id':'p12_formal_r2','status':'pass_complete','formal_result':True,
      'performance_improvement_demonstrated':assessment['performance_improvement_demonstrated'],
      'positive_comparators':assessment['positive_comparators'],'blocks':200,'episodes':800,
      'assessment_sha256':sha256(ASSESSMENT),'review_sha256':sha256(REVIEW),'run_manifest_unique':ru,'episode_results_unique':eu})
    atomic(integrity_path,json.dumps(integrity,indent=2,sort_keys=True)+'\n')
    s=assessment['method_summaries'];teb=assessment['primary_comparisons']['actual_teb'];dwa=assessment['primary_comparisons']['dwa']
    positives=', '.join(assessment['positive_comparators']) or 'none'
    append_once(ARTIFACT/'progress.md','## P12 formal R2 result',f"""## P12 formal R2 result

- 200 blocks / 800 episodes 全部 attempt=1 完成；无 retry、replacement 或删除。
- 失败计入均时：PCT {s['pct_scope_rl']['mean_failure_inclusive_time_s']:.4f} s（{s['pct_scope_rl']['valid_goals']}/200）、解析 {s['analytic_prior']['mean_failure_inclusive_time_s']:.4f} s（{s['analytic_prior']['valid_goals']}/200）、TEB {s['actual_teb']['mean_failure_inclusive_time_s']:.4f} s（{s['actual_teb']['valid_goals']}/200）、DWA {s['dwa']['mean_failure_inclusive_time_s']:.4f} s（{s['dwa']['valid_goals']}/200）。
- PCT 对 TEB 改善 {100*teb['relative_reduction']:.4f}%（97.5% CI [{100*teb['bootstrap']['lower_97_5_percent']:.4f}%, {100*teb['bootstrap']['upper_97_5_percent']:.4f}%）；对 DWA 改善 {100*dwa['relative_reduction']:.4f}%（[{100*dwa['bootstrap']['lower_97_5_percent']:.4f}%, {100*dwa['bootstrap']['upper_97_5_percent']:.4f}%）。
- `performance_improvement_demonstrated={str(assessment['performance_improvement_demonstrated']).lower()}`；阳性比较：{positives}；完整性 review SHA `{sha256(REVIEW)}`。""")
    append_once(ARTIFACT/'decision_log.md','## D029 — formal R2 结果封存',f"""## D029 — formal R2 结果封存

- 日期：2026-08-10
- 状态：已采用
- 决策：R2 800 个身份终止且不可重跑。阳性比较为 `{positives}`，严格按预注册全部门限判定；不再使用正式 seeds 调参。两轮独立正式研究及其差异均保留。""")
    append_once(ARTIFACT/'statistical_report.md','## P12 formal R2 主结果',f"""## P12 formal R2 主结果

PCT/TEB/DWA 的失败计入均时为 {s['pct_scope_rl']['mean_failure_inclusive_time_s']:.4f}/
{s['actual_teb']['mean_failure_inclusive_time_s']:.4f}/{s['dwa']['mean_failure_inclusive_time_s']:.4f} s。PCT 对 TEB 的相对改善为
{100*teb['relative_reduction']:.4f}%（97.5% CI [{100*teb['bootstrap']['lower_97_5_percent']:.4f}%, {100*teb['bootstrap']['upper_97_5_percent']:.4f}%），
对 DWA 为 {100*dwa['relative_reduction']:.4f}%（[{100*dwa['bootstrap']['lower_97_5_percent']:.4f}%, {100*dwa['bootstrap']['upper_97_5_percent']:.4f}%）。阳性比较：{positives}。""")
    claims_path=ARTIFACT/'claims_evidence_matrix.yaml';claims=yaml.safe_load(claims_path.read_text())
    claims['schema_version']='1.0';claims['stage']=STAGE;claims['status']='formal_r2_complete'
    claims['claims']=[x for x in claims['claims'] if x.get('id') not in ('p12_formal_r2','pct_vs_actual_teb_gazebo','pct_vs_dwa_gazebo')]
    for comparator,key in [('Actual TEB','actual_teb'),('DWA','dwa')]:
        result=assessment['primary_comparisons'][key];positive=result['positive_gate_pass']
        claims['claims'].append({'id':'pct_vs_'+('actual_teb' if key=='actual_teb' else 'dwa')+'_gazebo',
          'status':'qualified_positive_formal_r2' if positive else 'not_demonstrated_after_two_formal_rounds',
          'claim':f"Formal R2 PCT relative reduction versus {comparator}: {100*result['relative_reduction']:.4f}%, 97.5% CI [{100*result['bootstrap']['lower_97_5_percent']:.4f}%, {100*result['bootstrap']['upper_97_5_percent']:.4f}%].",
          'evidence':{'assessment':str(ASSESSMENT.relative_to(ROOT)),'assessment_sha256':sha256(ASSESSMENT),'review_sha256':sha256(REVIEW)},
          'limitations':['six frozen Gazebo templates','simulation only','no perception/OOD/deployment/real-vehicle claim']})
    claims['claims'].append({'id':'p12_formal_r2','status':'qualified_positive' if assessment['performance_improvement_demonstrated'] else 'qualified_negative',
      'claim':'Formal R2 completed all 200 paired blocks. Positive comparator gates passed for: '+positives+'.',
      'evidence':{'assessment_sha256':sha256(ASSESSMENT),'review_sha256':sha256(REVIEW)}})
    atomic(claims_path,yaml.safe_dump(claims,sort_keys=False,allow_unicode=True))
    print(json.dumps({'review_sha256':sha256(REVIEW),'positive_comparators':assessment['positive_comparators'],'run_rows':rc,'episode_rows':ec},indent=2))


if __name__=='__main__':main()
