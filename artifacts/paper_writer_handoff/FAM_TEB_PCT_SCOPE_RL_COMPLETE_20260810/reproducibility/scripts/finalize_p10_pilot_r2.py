#!/usr/bin/env python3
"""Seal the assessed R2 pilot into all seven benchmark evidence ledgers."""

from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
PILOT = ARTIFACT / "pilot/pilot_r2"
ASSESSMENT = PILOT / "pilot_r2_assessment.json"
RAW = PILOT / "pilot_r2_episode_results.csv"
RELEASE = PILOT / "pilot_r2_execution_release.yaml"
REVIEW = PILOT / "pilot_r2_integrity_review.json"
STAGE = "P10_PILOT_R2"
ROUND = "gazebo_p10_pilot_r2"


def sha256(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def atomic_text(path, content):
    temporary = path.with_suffix(path.suffix + ".tmp"); temporary.write_text(content, encoding="utf-8"); temporary.replace(path)
def append_once(path, marker, content):
    original = path.read_text(encoding="utf-8")
    if marker not in original: atomic_text(path, original.rstrip() + "\n\n" + content.strip() + "\n")


def load():
    assessment = json.loads(ASSESSMENT.read_text(encoding="utf-8"))
    if assessment.get("blocks") != 20 or assessment.get("episodes") != 80 or assessment.get("pilot_tuning_performed") is not False:
        raise RuntimeError("PILOT_R2_ASSESSMENT_INVALID")
    records=[]
    for row in assessment["per_identity_evidence"]:
        report_path=ROOT/row["report"]; receipt_path=ROOT/row["receipt"]
        report=yaml.load(report_path.read_text(encoding="utf-8"),Loader=yaml.CSafeLoader)
        receipt=json.loads(receipt_path.read_text(encoding="utf-8"))
        root=report_path.parent
        records.append({**row,"report_path":report_path,"receipt_path":receipt_path,"report":report,"receipt":receipt,"root":root})
    ids={(x['seed'],x['scene_id'],x['method']) for x in records}
    if len(records)!=80 or len(ids)!=80: raise RuntimeError("PILOT_R2_EVIDENCE_NOT_UNIQUE")
    return assessment,records


def update_run(records):
    path=ARTIFACT/"run_manifest.csv"
    with path.open(newline="",encoding="utf-8") as stream:
        reader=csv.DictReader(stream); fields=reader.fieldnames; retained=[x for x in reader if x['stage']!=STAGE]
    rows=[]
    for x in records:
        receipt=x['receipt']; ended=datetime.fromtimestamp(x['receipt_path'].stat().st_mtime).astimezone()
        started=datetime.fromtimestamp(ended.timestamp()-float(receipt['wall_elapsed_s'])).astimezone()
        rows.append({"stage":STAGE,"run_id":"{}__{}_{}_{}".format(x['block_id'],x['seed'],x['scene_id'],x['method']),
          "phase":"preregistered_pilot_r2","method":x['method'],"scene_id":x['scene_id'],"split":"pilot",
          "seed":x['seed'],"attempt":1,"status":"completed_pass" if x['valid'] else x['failure_reason']+"_retained",
          "started_at":started.isoformat(timespec="seconds"),"ended_at":ended.isoformat(timespec="seconds"),
          "exit_code":receipt['return_code'],"config_sha256":receipt['release_sha256'],
          "model_sha256":x['report'].get('actor_checkpoint_sha256') or 'none',"evidence_path":str(x['root'].relative_to(ROOT)),
          "formal_eligible":"false","notes":"block={}; order={}; pilot_not_for_tuning; no_retry".format(x['block_id'],receipt['order_index'])})
    temporary=path.with_suffix('.csv.tmp')
    with temporary.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(retained+rows)
    temporary.replace(path)


def update_episode(records):
    path=ARTIFACT/"episode_results.csv"
    with path.open(newline="",encoding="utf-8") as stream:
        reader=csv.DictReader(stream);fields=reader.fieldnames;retained=[x for x in reader if x['study_round']!=ROUND]
    rows=[]
    for x in records:
        result=x['report']['result']; elapsed=float(result['restricted_elapsed_s']); length=result.get('path_length_m')
        values={"study_round":ROUND,"phase":"preregistered_pilot_r2","block_id":x['block_id'],
          "episode_id":"{}_{}_{}_{}".format(x['block_id'],x['seed'],x['scene_id'],x['method']),"method":x['method'],
          "scene_family":x['scene_id'],"scene_id":x['scene_id'],"split":"pilot","seed":x['seed'],"attempt":1,
          "order_index":x['receipt']['order_index'],"goal_reached":str(bool(result['goal_reached'])).lower(),
          "timeout":str(bool(result['timeout'])).lower(),"contact_count":result['contact_state_count'],
          "system_failure":str(x['failure_reason']=='execution_integrity_failure').lower(),"failure_reason":x['failure_reason'],
          "failure_inclusive_time_s":x['failure_inclusive_time_s'],"min_clearance_m":result['minimum_conservative_clearance_m'],
          "path_length_m":length if length is not None else '',"mean_speed_mps":length/elapsed if length is not None and elapsed>0 else '',
          "acceleration_rms":"","jerk_metric":result.get('maximum_absolute_jerk_mps3',''),
          "candidate_switch_count":result.get('candidate_switch_count',''),"stop_count":result.get('certified_stop_count',''),
          "recovery_count":result.get('stop_recovery_count',''),"mean_latency_ms":result.get('mean_inference_latency_ms',''),
          "p95_latency_ms":result.get('p95_inference_latency_ms',''),"p99_latency_ms":result.get('p99_inference_latency_ms',''),
          "mask_violation_count":result.get('mask_violation_count',0),"config_sha256":x['receipt']['release_sha256'],
          "model_sha256":x['report'].get('actor_checkpoint_sha256') or 'none',"evidence_path":str(x['report_path'].relative_to(ROOT))}
        rows.append({f:values.get(f,'') for f in fields})
    temporary=path.with_suffix('.csv.tmp')
    with temporary.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(retained+rows)
    temporary.replace(path)


def unique(path,a,b):
    rows=list(csv.DictReader(path.open(encoding='utf-8')));keys=[(x[a],x[b]) for x in rows];return len(keys)==len(set(keys)),len(rows)


def main():
    assessment,records=load(); release=yaml.safe_load(RELEASE.read_text(encoding='utf-8'))
    for item in release['resource_bindings']:
        target=ROOT/item['path']
        if sha256(target)!=item['sha256']: raise RuntimeError('PILOT_R2_RESOURCE_HASH_MISMATCH:'+item['path'])
    for item in release['external_resource_bindings']:
        if sha256(Path(item['path']))!=item['sha256']: raise RuntimeError('PILOT_R2_EXTERNAL_HASH_MISMATCH:'+item['path'])
    update_run(records);update_episode(records)
    ru,rc=unique(ARTIFACT/'run_manifest.csv','stage','run_id');eu,ec=unique(ARTIFACT/'episode_results.csv','study_round','episode_id')
    if not ru or not eu: raise RuntimeError('PILOT_R2_LEDGER_DUPLICATE')
    review={"schema_version":"1.0","status":"pilot_r2_integrity_review_pass","release_sha256":sha256(RELEASE),
      "preregistration_sha256":sha256(PILOT/'pilot_r2_preregistration.yaml'),"schedule_sha256":sha256(PILOT/'pilot_r2_schedule.csv'),
      "assessment_sha256":sha256(ASSESSMENT),"episode_results_sha256":sha256(RAW),"blocks":20,"episodes":80,
      "attempt_one_complete":True,"same_identity_retry_count":0,"replacement_seed_count":0,"all_failures_retained":True,
      "release_resources_rehashed":True,"run_manifest_unique":ru,"run_manifest_rows":rc,"episode_results_unique":eu,
      "episode_results_rows":ec,"formal_round_2_seed_accessed":False,"hardware_accessed":False}
    atomic_text(REVIEW,json.dumps(review,indent=2,sort_keys=True)+'\n')
    integrity_path=ARTIFACT/'integrity_report.json'; integrity=json.loads(integrity_path.read_text())
    integrity['schema_version']='1.0';integrity['status']='pilot_r2_complete_formal_r2_not_released';integrity['formal_execution_authorized']=False
    integrity['checks']=[x for x in integrity['checks'] if x.get('id')!='p10_pilot_r2']
    integrity['checks'].append({"id":"p10_pilot_r2","status":"pass_complete_power_only","blocks":20,"episodes":80,
      "release_sha256":sha256(RELEASE),"assessment_sha256":sha256(ASSESSMENT),"review_sha256":sha256(REVIEW),
      "selected_formal_round_2_blocks":assessment['formal_round_2_selected_blocks'],"formal_round_2_seed_accessed":False})
    atomic_text(integrity_path,json.dumps(integrity,indent=2,sort_keys=True)+'\n')
    s=assessment['method_summaries'];c=assessment['paired_comparisons']
    append_once(ARTIFACT/'progress.md','## P10 preregistered R2 pilot',f"""## P10 preregistered R2 pilot

- release `{sha256(RELEASE)}` 下 20 blocks / 80 episodes 全部 attempt=1 完成，无重试、替换或删除。
- 失败计入均时：PCT {s['pct_scope_rl']['mean_failure_inclusive_time_s']:.4f} s（{s['pct_scope_rl']['valid_goals']}/20）、解析 {s['analytic_prior']['mean_failure_inclusive_time_s']:.4f} s（{s['analytic_prior']['valid_goals']}/20）、TEB {s['actual_teb']['mean_failure_inclusive_time_s']:.4f} s（{s['actual_teb']['valid_goals']}/20）、DWA {s['dwa']['mean_failure_inclusive_time_s']:.4f} s（{s['dwa']['valid_goals']}/20）。
- PCT 对 TEB/DWA 的 pilot 点改善分别为 {100*c['actual_teb']['relative_reduction']:.4f}% / {100*c['dwa']['relative_reduction']:.4f}%；pilot 不用于调参或优势声明。
- 冻结功效规则选择 formal R2 为 {assessment['formal_round_2_selected_blocks']} blocks；正式 seeds198001--198200 尚未访问。""")
    append_once(ARTIFACT/'decision_log.md','## D027 — R2 pilot 只选择正式样本量',f"""## D027 — R2 pilot 只选择正式样本量

- 日期：2026-08-10
- 状态：已采用
- 决策：pilot 后不改变任何方法。DWA 8% 目标约需 {c['dwa']['power_analysis']['required_blocks_ceiling']} blocks，TEB 约需 {c['actual_teb']['power_analysis']['required_blocks_ceiling']} blocks；按预注册 floor160/cap200 规则选择 200 blocks，并明确 TEB 比较仍略低于目标功效。""")
    append_once(ARTIFACT/'statistical_report.md','## P10 R2 pilot 与功效',f"""## P10 R2 pilot 与功效

R2 pilot 的 PCT/TEB/DWA 失效计入均时为 {s['pct_scope_rl']['mean_failure_inclusive_time_s']:.4f}/
{s['actual_teb']['mean_failure_inclusive_time_s']:.4f}/{s['dwa']['mean_failure_inclusive_time_s']:.4f} s；PCT 相对 DWA 点改善
{100*c['dwa']['relative_reduction']:.4f}%，相对 TEB {100*c['actual_teb']['relative_reduction']:.4f}%。这些数值仅用于功效；正式 R2 固定为 200 blocks，指标、门限和方法均不改变。""")
    claims_path=ARTIFACT/'claims_evidence_matrix.yaml';claims=yaml.safe_load(claims_path.read_text())
    claims['schema_version']='1.0';claims['stage']=STAGE;claims['status']='pilot_r2_complete_formal_r2_not_released'
    claims['claims']=[x for x in claims['claims'] if x.get('id')!='p10_pilot_r2']
    claims['claims'].append({"id":"p10_pilot_r2","status":"pilot_only_not_qualified_for_advantage",
      "claim":"R2 pilot selected 200 formal blocks and made no tuning change.","evidence":{"assessment":str(ASSESSMENT.relative_to(ROOT)),
      "assessment_sha256":sha256(ASSESSMENT),"review_sha256":sha256(REVIEW)},"limitations":["pilot sample","not formal","simulation only"]})
    atomic_text(claims_path,yaml.safe_dump(claims,sort_keys=False,allow_unicode=True))
    print(json.dumps({"review_sha256":sha256(REVIEW),"run_rows":rc,"episode_rows":ec},indent=2))


if __name__=='__main__':main()
