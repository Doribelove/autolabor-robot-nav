#!/usr/bin/env python3
"""Deterministically assess the locked 200-block formal-round-2 matrix."""

from __future__ import annotations

import csv
import json

import assess_formal_r1 as common
import yaml


ROOT = common.ROOT
FORMAL = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark/formal/round_2"
SCHEDULE = FORMAL / "formal_r2_schedule.csv"
PREREG = FORMAL / "formal_r2_preregistration.yaml"
RELEASE = FORMAL / "formal_r2_execution_release.yaml"
EXECUTION = FORMAL / "execution/episodes"
REPORT = FORMAL / "formal_r2_assessment.json"
EPISODES = FORMAL / "formal_r2_episode_results.csv"
common.EPISODES = EPISODES


def load():
    schedule = list(csv.DictReader(SCHEDULE.open(encoding="utf-8")))
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    release_sha = common.sha256(RELEASE)
    rows=[]
    for item in schedule:
        root=EXECUTION/"{}_{}_{}".format(item['seed'],item['scene_id'],item['method'])
        report_path=root/'episode_report.yaml';receipt_path=root/'receipt.json'
        if not report_path.is_file() or not receipt_path.is_file():
            raise RuntimeError('FORMAL_R2_INCOMPLETE_IDENTITY:'+root.name)
        report=yaml.load(report_path.read_text(encoding='utf-8'),Loader=yaml.CSafeLoader)
        receipt=json.loads(receipt_path.read_text(encoding='utf-8'))
        if (receipt.get('release_sha256')!=release_sha or receipt.get('episode_report_sha256')!=common.sha256(report_path)
            or report.get('stage')!='PCT-BENCH-FORMAL-R2' or report.get('split')!='formal_round_2'
            or report.get('candidate_contract_variant')!='r2_candidate_v060' or int(item['attempt'])!=1):
            raise RuntimeError('FORMAL_R2_IDENTITY_INTEGRITY_FAILURE:'+root.name)
        reason=common.failure_reason(report);result=report['result']
        rows.append({'block_id':item['block_id'],'seed':int(item['seed']),'scene_id':item['scene_id'],'method':item['method'],
          'attempt':1,'order_index':int(item['order_index']),'goal_reached':bool(result['goal_reached']),
          'execution_integrity_pass':bool(report['execution_integrity_pass']),'valid_goal':reason=='goal','failure_reason':reason,
          'failure_inclusive_time_s':float(result['restricted_elapsed_s']) if reason=='goal' else 22.0,
          'minimum_conservative_clearance_m':float(result['minimum_conservative_clearance_m']),
          'contact_count':int(result.get('contact_state_count',0)),'mask_violation_count':int(result.get('mask_violation_count',0)),
          'path_length_m':result.get('path_length_m'),'candidate_switch_count':result.get('candidate_switch_count'),
          'certified_stop_count':result.get('certified_stop_count'),'stop_recovery_count':result.get('stop_recovery_count'),
          'mean_inference_latency_ms':result.get('mean_inference_latency_ms'),'p95_inference_latency_ms':result.get('p95_inference_latency_ms'),
          'p99_inference_latency_ms':result.get('p99_inference_latency_ms'),'report_path':str(report_path.relative_to(ROOT)),
          'report_sha256':common.sha256(report_path),'receipt_path':str(receipt_path.relative_to(ROOT)),
          'receipt_sha256':common.sha256(receipt_path)})
    identities={(x['seed'],x['scene_id'],x['method'],x['attempt']) for x in rows}
    if len(schedule)!=800 or len(rows)!=800 or len(identities)!=800:
        raise RuntimeError('FORMAL_R2_MATRIX_COMPLETENESS_FAILURE')
    return prereg,rows


def main():
    if REPORT.exists() or EPISODES.exists(): raise FileExistsError('FORMAL_R2_ASSESSMENT_ALREADY_EXISTS')
    prereg,rows=load();summaries={m:common.summary(rows,m) for m in common.METHODS}
    comparisons={name:common.compare(rows,prereg,name) for name in ('actual_teb','dwa')}
    common.write_episode_csv(rows)
    positive=[name for name,value in comparisons.items() if value['positive_gate_pass']]
    report={'schema_version':'1.0','stage':'P12_FORMAL_R2_ASSESSMENT','status':'formal_round_2_complete',
      'formal_result':True,'performance_improvement_demonstrated':bool(positive),'positive_comparators':positive,
      'blocks':200,'episodes':800,'attempt_one_complete':True,'release_sha256':common.sha256(RELEASE),
      'preregistration_sha256':common.sha256(PREREG),'schedule_sha256':common.sha256(SCHEDULE),
      'episode_results_path':str(EPISODES.relative_to(ROOT)),'episode_results_sha256':common.sha256(EPISODES),
      'method_summaries':summaries,'primary_comparisons':comparisons,'scene_strata':common.strata(rows),
      'all_failures_retained':True,'same_identity_retry_count':0,'replacement_seed_count':0,
      'hardware_accessed':False,'real_vehicle_used':False,
      'claim_boundary':'independent fresh single-robot Gazebo R2 population over six frozen templates; no perception/OOD/deployment/real-vehicle claim',
      'per_identity_evidence':rows}
    REPORT.write_text(json.dumps(report,indent=2,sort_keys=True)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k not in ('per_identity_evidence','scene_strata')},indent=2))


if __name__=='__main__':main()
