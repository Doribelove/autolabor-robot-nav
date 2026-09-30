#!/usr/bin/env python3
"""Record the pre-outcome formal-R2 release in all seven ledgers."""

from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

import yaml


ROOT=Path(__file__).resolve().parents[2]
ARTIFACT=ROOT/'artifacts/gazebo_pct_teb_dwa_benchmark';FORMAL=ARTIFACT/'formal/round_2'
RELEASE=FORMAL/'formal_r2_execution_release.yaml';PREREG=FORMAL/'formal_r2_preregistration.yaml'
SCHEDULE=FORMAL/'formal_r2_schedule.csv';REVIEW=FORMAL/'formal_r2_release_review.json';STAGE='P11_FORMAL_R2_RELEASE'


def sha256(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def atomic(path,text):
    temporary=path.with_suffix(path.suffix+'.tmp');temporary.write_text(text,encoding='utf-8');temporary.replace(path)
def append_once(path,marker,text):
    original=path.read_text(encoding='utf-8')
    if marker not in original:atomic(path,original.rstrip()+'\n\n'+text.strip()+'\n')


def main():
    review=json.loads(REVIEW.read_text());release=yaml.safe_load(RELEASE.read_text())
    if review['status']!='formal_r2_release_offline_review_pass' or release['execution_authorized'] is not True:
        raise RuntimeError('FORMAL_R2_RELEASE_REVIEW_INVALID')
    if (FORMAL/'execution').exists():raise RuntimeError('FORMAL_R2_OUTCOME_PATH_ALREADY_EXISTS')
    for item in release['resource_bindings']:
        if sha256(ROOT/item['path'])!=item['sha256']:raise RuntimeError('FORMAL_R2_RELEASE_RESOURCE_MISMATCH:'+item['path'])
    run=ARTIFACT/'run_manifest.csv'
    with run.open(newline='',encoding='utf-8') as stream:
        reader=csv.DictReader(stream);fields=reader.fieldnames;rows=[x for x in reader if x['stage']!=STAGE]
    stamp=datetime.now().astimezone().isoformat(timespec='seconds')
    rows.append({'stage':STAGE,'run_id':'formal_r2_preoutcome_release','phase':'formal_r2_release','method':'four_method_matrix',
      'scene_id':'six_frozen_core_templates','split':'formal_round_2','seed':'198001--198200_unaccessed','attempt':1,
      'status':'released_not_executed','started_at':stamp,'ended_at':stamp,'exit_code':0,'config_sha256':sha256(RELEASE),
      'model_sha256':'bound_in_release','evidence_path':str(RELEASE.relative_to(ROOT)),'formal_eligible':'true',
      'notes':'200 blocks; 800 episodes; preoutcome release; no formal R2 outcome accessed'})
    temporary=run.with_suffix('.csv.tmp')
    with temporary.open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    temporary.replace(run)
    run_rows=list(csv.DictReader(run.open()));run_keys=[(x['stage'],x['run_id']) for x in run_rows]
    episode_rows=list(csv.DictReader((ARTIFACT/'episode_results.csv').open()));episode_keys=[(x['study_round'],x['episode_id']) for x in episode_rows]
    if len(run_keys)!=len(set(run_keys)) or len(episode_keys)!=len(set(episode_keys)):raise RuntimeError('P11_LEDGER_DUPLICATE')
    integrity_path=ARTIFACT/'integrity_report.json';integrity=json.loads(integrity_path.read_text())
    integrity['schema_version']='1.0';integrity['status']='formal_r2_released_preoutcome';integrity['formal_execution_authorized']=True
    integrity['checks']=[x for x in integrity['checks'] if x.get('id')!='p11_formal_r2_release']
    integrity['checks'].append({'id':'p11_formal_r2_release','status':'pass_preoutcome_release','blocks':200,'episodes':800,
      'release_sha256':sha256(RELEASE),'preregistration_sha256':sha256(PREREG),'schedule_sha256':sha256(SCHEDULE),
      'formal_r2_outcome_accessed':False,'run_manifest_unique':True,'episode_results_unique':True})
    atomic(integrity_path,json.dumps(integrity,indent=2,sort_keys=True)+'\n')
    append_once(ARTIFACT/'progress.md','## P11 formal R2 release',f"""## P11 formal R2 release

- 在任何 R2 formal outcome 访问前冻结 200 blocks / 800 episodes，fresh seeds198001--198200；四方法在四个顺序位置各 50 次。
- 日程 SHA `{sha256(SCHEDULE)}`，预注册 SHA `{sha256(PREREG)}`，唯一 release SHA `{sha256(RELEASE)}`。
- 相对 R1 只改变 actions4/5 空间候选速度 0.4→0.6 m/s；PPO actor、解析公式、硬安全门、TEB-v0、DWA-v0、统计门和失败记账均不变。
- 正式 R2 outcome 尚未访问。""")
    append_once(ARTIFACT/'decision_log.md','## D028 — formal R2 独立发布',"""## D028 — formal R2 独立发布

- 日期：2026-08-10
- 状态：已采用
- 决策：R2 只允许 200 个 fresh blocks、attempt=1；任一传统比较需同时达到 8% 点改善、97.5% bootstrap 下界>0、完成率非劣、contact 与 mask 门。禁止重试、替换、窥视后调参或回收 R1/R2 identities。""")
    append_once(ARTIFACT/'statistical_report.md','## P11 formal R2 预注册',"""## P11 formal R2 预注册

正式 R2 为 200 paired blocks、800 episodes。两项锁定主比较、估计量、8% 实效门、97.5% 配对
bootstrap、完成率 -5pp 非劣门及安全门与 R1 相同；bootstrap seeds 独立为 20261991/20261992。
所有失败统一计 22 s，正式结果不得用于继续调参。""")
    claims_path=ARTIFACT/'claims_evidence_matrix.yaml';claims=yaml.safe_load(claims_path.read_text())
    claims['schema_version']='1.0';claims['stage']=STAGE;claims['status']='formal_r2_released_preoutcome'
    claims['claims']=[x for x in claims['claims'] if x.get('id')!='p11_formal_r2_release']
    claims['claims'].append({'id':'p11_formal_r2_release','status':'preregistered_pending_execution_no_result_claim',
      'claim':'Formal R2 is hash-bound for 200 fresh paired blocks and grants no result claim before execution.',
      'evidence':{'release_sha256':sha256(RELEASE),'preregistration_sha256':sha256(PREREG),'schedule_sha256':sha256(SCHEDULE)}})
    atomic(claims_path,yaml.safe_dump(claims,sort_keys=False,allow_unicode=True))
    print(json.dumps({'release_sha256':sha256(RELEASE),'run_rows':len(run_rows),'episode_rows':len(episode_rows)},indent=2))


if __name__=='__main__':main()
