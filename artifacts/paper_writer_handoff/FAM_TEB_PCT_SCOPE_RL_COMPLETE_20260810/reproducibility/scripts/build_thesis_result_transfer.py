#!/usr/bin/env python3
"""Build deterministic paper-transfer tables, SVGs and Chinese report."""

from __future__ import annotations

import csv
from collections import Counter
from datetime import datetime
import hashlib
import html
import json
from pathlib import Path
import statistics

import yaml


ROOT=Path(__file__).resolve().parents[2]
ARTIFACT=ROOT/'artifacts/gazebo_pct_teb_dwa_benchmark';OUT=ARTIFACT/'final_transfer'
R1=ARTIFACT/'formal/round_1/formal_r1_assessment.json';R2=ARTIFACT/'formal/round_2/formal_r2_assessment.json'
R2_RAW=ARTIFACT/'formal/round_2/formal_r2_episode_results.csv'
DEV=ARTIFACT/'development/r2_candidate_coverage_d0/assessment.json';PILOT=ARTIFACT/'pilot/pilot_r2/pilot_r2_assessment.json'
TARGET=ROOT/'THESIS_RESULT_TRANSFER.md';STAGE='P13_THESIS_TRANSFER'
METHOD_LABEL={'pct_scope_rl':'PCT-SCOPE-RL','analytic_prior':'解析先验','actual_teb':'Actual TEB','dwa':'DWA'}
COLORS={'pct_scope_rl':'#2563eb','analytic_prior':'#7c3aed','actual_teb':'#059669','dwa':'#d97706'}


def sha256(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def atomic(path,text):
    path.parent.mkdir(parents=True,exist_ok=True);temporary=path.with_suffix(path.suffix+'.tmp');temporary.write_text(text,encoding='utf-8');temporary.replace(path)
def append_once(path,marker,text):
    original=path.read_text(encoding='utf-8')
    if marker not in original:atomic(path,original.rstrip()+'\n\n'+text.strip()+'\n')
def pct(x):return 100.0*float(x)


def svg_start(width,height,title):
    return [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
      '<rect width="100%" height="100%" fill="white"/>',f'<text x="{width/2}" y="28" text-anchor="middle" font-family="sans-serif" font-size="18" font-weight="bold">{html.escape(title)}</text>']


def effect_svg(r1,r2,path):
    rows=[]
    for label,data in [('R1 vs TEB',r1['primary_comparisons']['actual_teb']),('R1 vs DWA',r1['primary_comparisons']['dwa']),
                       ('R2 vs TEB',r2['primary_comparisons']['actual_teb']),('R2 vs DWA',r2['primary_comparisons']['dwa'])]:
        rows.append((label,pct(data['relative_reduction']),pct(data['bootstrap']['lower_97_5_percent']),pct(data['bootstrap']['upper_97_5_percent']),data['positive_gate_pass']))
    width,height=760,270;left,right=170,720;xmin,xmax=-15,20
    x=lambda v:left+(v-xmin)/(xmax-xmin)*(right-left)
    s=svg_start(width,height,'两轮正式研究：PCT 相对失败计入时间降低（97.5% CI）')
    for tick in (-15,-10,-5,0,5,8,10,15,20):
        xx=x(tick);s.append(f'<line x1="{xx:.1f}" y1="45" x2="{xx:.1f}" y2="230" stroke="#e5e7eb"/>')
        s.append(f'<text x="{xx:.1f}" y="250" text-anchor="middle" font-family="sans-serif" font-size="11">{tick}%</text>')
    s.append(f'<line x1="{x(0):.1f}" y1="45" x2="{x(0):.1f}" y2="230" stroke="#111827" stroke-width="1.5"/>')
    s.append(f'<line x1="{x(8):.1f}" y1="45" x2="{x(8):.1f}" y2="230" stroke="#dc2626" stroke-dasharray="5 4"/>')
    for i,(label,point,lo,hi,positive) in enumerate(rows):
        y=70+i*43;color='#16a34a' if positive else '#64748b'
        s.append(f'<text x="160" y="{y+4}" text-anchor="end" font-family="sans-serif" font-size="12">{label}</text>')
        s.append(f'<line x1="{x(lo):.1f}" y1="{y}" x2="{x(hi):.1f}" y2="{y}" stroke="{color}" stroke-width="4"/>')
        s.append(f'<circle cx="{x(point):.1f}" cy="{y}" r="6" fill="{color}"/>')
        s.append(f'<text x="{min(735,x(hi)+8):.1f}" y="{y+4}" font-family="sans-serif" font-size="11">{point:.2f}%</text>')
    s.append('</svg>');atomic(path,'\n'.join(s)+'\n')


def outcomes_svg(r2,path):
    methods=('pct_scope_rl','analytic_prior','actual_teb','dwa');width,height=760,330;left=90;base=250
    s=svg_start(width,height,'正式 R2：失败计入均时与有效到达率')
    for i,m in enumerate(methods):
        summary=r2['method_summaries'][m];x=left+i*165;bar=summary['mean_failure_inclusive_time_s']/22*180
        s.append(f'<rect x="{x}" y="{base-bar:.1f}" width="58" height="{bar:.1f}" fill="{COLORS[m]}"/>')
        s.append(f'<text x="{x+29}" y="{base-bar-7:.1f}" text-anchor="middle" font-family="sans-serif" font-size="12">{summary["mean_failure_inclusive_time_s"]:.2f}s</text>')
        s.append(f'<text x="{x+29}" y="{base+20}" text-anchor="middle" font-family="sans-serif" font-size="12">{METHOD_LABEL[m]}</text>')
        rate=100*summary['completion_rate'];s.append(f'<text x="{x+29}" y="{base+39}" text-anchor="middle" font-family="sans-serif" font-size="11" fill="#374151">到达 {rate:.1f}%</text>')
    s.append(f'<line x1="65" y1="{base}" x2="735" y2="{base}" stroke="#111827"/>');s.append('</svg>');atomic(path,'\n'.join(s)+'\n')


def scene_svg(r2,path):
    scenes=list(sorted(r2['scene_strata']));methods=('pct_scope_rl','analytic_prior','actual_teb','dwa')
    width,height=1000,430;left=80;base=330;group=145;barw=25
    s=svg_start(width,height,'正式 R2 六场景失败计入均时（秒）')
    for tick in (0,5,10,15,20):
        y=base-tick/22*250;s.append(f'<line x1="60" y1="{y:.1f}" x2="970" y2="{y:.1f}" stroke="#e5e7eb"/>');s.append(f'<text x="52" y="{y+4:.1f}" text-anchor="end" font-family="sans-serif" font-size="11">{tick}</text>')
    for i,scene in enumerate(scenes):
        gx=left+i*group
        for j,m in enumerate(methods):
            value=r2['scene_strata'][scene][m]['mean_failure_inclusive_time_s'];h=value/22*250;x=gx+j*barw
            s.append(f'<rect x="{x}" y="{base-h:.1f}" width="{barw-2}" height="{h:.1f}" fill="{COLORS[m]}"/>')
        label=scene.replace('core_','').replace('_',' ');s.append(f'<text x="{gx+48}" y="{base+18}" text-anchor="middle" font-family="sans-serif" font-size="10">{html.escape(label)}</text>')
    for j,m in enumerate(methods):
        x=180+j*180;s.append(f'<rect x="{x}" y="380" width="14" height="14" fill="{COLORS[m]}"/><text x="{x+20}" y="392" font-family="sans-serif" font-size="11">{METHOD_LABEL[m]}</text>')
    s.append('</svg>');atomic(path,'\n'.join(s)+'\n')


def ablation_svg(dev,path):
    effects=dev['effects'];cats=[('总体',effects['overall_mean_improvement_s']),('静态',effects['static_mean_improvement_s']),
      ('动态',-effects['dynamic_mean_worsening_s']),('宽度变化',effects['width_control_mean_s']-effects['width_treatment_mean_s'])]
    width,height=700,290;zero=130;scale=75
    s=svg_start(width,height,'R2 development 单因素消融：绝对时间改善（秒）')
    s.append(f'<line x1="{zero}" y1="50" x2="{zero}" y2="245" stroke="#111827"/>')
    for i,(label,value) in enumerate(cats):
        y=70+i*44;x2=zero+value*scale;color='#2563eb' if value>=0 else '#dc2626'
        s.append(f'<text x="115" y="{y+14}" text-anchor="end" font-family="sans-serif" font-size="12">{label}</text>')
        s.append(f'<rect x="{min(zero,x2):.1f}" y="{y}" width="{abs(x2-zero):.1f}" height="22" fill="{color}"/>')
        s.append(f'<text x="{x2+(8 if value>=0 else -8):.1f}" y="{y+15}" text-anchor="{"start" if value>=0 else "end"}" font-family="sans-serif" font-size="11">{value:+.3f}s</text>')
    s.append('</svg>');atomic(path,'\n'.join(s)+'\n')


def build_diagnostics(r2,raw):
    diagnostics=[]
    for scene in sorted(r2['scene_strata']):
        pct_rows=[row for row in raw if row['method']=='pct_scope_rl' and row['scene_id']==scene]
        mean=lambda key:statistics.mean(float(row[key]) for row in pct_rows)
        strata=r2['scene_strata'][scene]
        failures=Counter(row['failure_reason'] for row in pct_rows)
        diagnostics.append({'scene':scene,'episodes':len(pct_rows),
          'pct_mean_failure_inclusive_time_s':strata['pct_scope_rl']['mean_failure_inclusive_time_s'],
          'actual_teb_mean_failure_inclusive_time_s':strata['actual_teb']['mean_failure_inclusive_time_s'],
          'dwa_mean_failure_inclusive_time_s':strata['dwa']['mean_failure_inclusive_time_s'],
          'pct_minus_actual_teb_s':strata['pct_scope_rl']['mean_failure_inclusive_time_s']-strata['actual_teb']['mean_failure_inclusive_time_s'],
          'pct_minus_dwa_s':strata['pct_scope_rl']['mean_failure_inclusive_time_s']-strata['dwa']['mean_failure_inclusive_time_s'],
          'pct_valid_goals':strata['pct_scope_rl']['valid_goals'],
          'pct_candidate_switch_mean':mean('candidate_switch_count'),'pct_certified_stop_mean':mean('certified_stop_count'),
          'pct_stop_recovery_mean':mean('stop_recovery_count'),'pct_timeout_count':failures['timeout_or_nonarrival'],
          'pct_clearance_violation_count':failures['conservative_clearance_violation']})
    return diagnostics


def write_csvs(r1,r2,dev,diagnostics):
    path=OUT/'formal_round_summary.csv';fields=['round','blocks','method','valid_goals','completion_rate','mean_failure_inclusive_time_s','minimum_clearance_m','contact_count','mask_violation_count']
    with path.open('w',newline='',encoding='utf-8') as stream:
        w=csv.DictWriter(stream,fieldnames=fields);w.writeheader()
        for name,data in [('R1',r1),('R2',r2)]:
            for method,summary in data['method_summaries'].items():w.writerow({'round':name,'blocks':data['blocks'],'method':method,'valid_goals':summary['valid_goals'],'completion_rate':summary['completion_rate'],'mean_failure_inclusive_time_s':summary['mean_failure_inclusive_time_s'],'minimum_clearance_m':summary['minimum_conservative_clearance_m'],'contact_count':summary['contact_count'],'mask_violation_count':summary['mask_violation_count']})
    path=OUT/'formal_effect_summary.csv';fields=['round','baseline','relative_reduction','lower_97_5','upper_97_5','completion_difference_pp','positive_gate_pass']
    with path.open('w',newline='',encoding='utf-8') as stream:
        w=csv.DictWriter(stream,fieldnames=fields);w.writeheader()
        for name,data in [('R1',r1),('R2',r2)]:
            for baseline,result in data['primary_comparisons'].items():w.writerow({'round':name,'baseline':baseline,'relative_reduction':result['relative_reduction'],'lower_97_5':result['bootstrap']['lower_97_5_percent'],'upper_97_5':result['bootstrap']['upper_97_5_percent'],'completion_difference_pp':result['completion_rate_difference_percentage_points'],'positive_gate_pass':result['positive_gate_pass']})
    path=OUT/'formal_r2_scene_strata.csv';fields=['scene','method','episodes','valid_goals','mean_failure_inclusive_time_s','minimum_clearance_m']
    with path.open('w',newline='',encoding='utf-8') as stream:
        w=csv.DictWriter(stream,fieldnames=fields);w.writeheader()
        for scene,methods in r2['scene_strata'].items():
            for method,result in methods.items():w.writerow({'scene':scene,'method':method,**result})
    path=OUT/'formal_r2_failure_diagnostics.csv';fields=list(diagnostics[0])
    with path.open('w',newline='',encoding='utf-8') as stream:
        w=csv.DictWriter(stream,fieldnames=fields);w.writeheader();w.writerows(diagnostics)


def main():
    r1=json.loads(R1.read_text());r2=json.loads(R2.read_text());dev=json.loads(DEV.read_text());pilot=json.loads(PILOT.read_text())
    raw=list(csv.DictReader(R2_RAW.open(encoding='utf-8')));diagnostics=build_diagnostics(r2,raw);diagnostic={row['scene']:row for row in diagnostics}
    if r2.get('episodes')!=800 or not r2.get('formal_result'):raise RuntimeError('FORMAL_R2_NOT_READY_FOR_TRANSFER')
    OUT.mkdir(parents=True,exist_ok=True);write_csvs(r1,r2,dev,diagnostics)
    effect_svg(r1,r2,OUT/'formal_effects_97_5_ci.svg');outcomes_svg(r2,OUT/'formal_r2_method_outcomes.svg')
    scene_svg(r2,OUT/'formal_r2_scene_strata.svg');ablation_svg(dev,OUT/'r2_development_ablation.svg')
    s=r2['method_summaries'];teb=r2['primary_comparisons']['actual_teb'];dwa=r2['primary_comparisons']['dwa']
    positives=', '.join(r2['positive_comparators']) or '无'
    conclusion=(f"在预注册的六类 Gazebo 模板总体中，R2 PCT-SCOPE-RL 相对 DWA 的失败计入时间降低 {pct(dwa['relative_reduction']):.2f}%（双侧 97.5% CI [{pct(dwa['bootstrap']['lower_97_5_percent']):.2f}%, {pct(dwa['bootstrap']['upper_97_5_percent']):.2f}%），并通过全部预注册门限；因此支持该限定总体内优于 DWA。" if dwa['positive_gate_pass'] else "两轮独立正式研究均未按预注册门限建立 PCT-SCOPE-RL 优于 DWA。")
    teb_text=("同时通过 Actual TEB 比较门。" if teb['positive_gate_pass'] else f"相对 Actual TEB 的改善为 {pct(teb['relative_reduction']):.2f}%（97.5% CI [{pct(teb['bootstrap']['lower_97_5_percent']):.2f}%, {pct(teb['bootstrap']['upper_97_5_percent']):.2f}%），未建立优于 Actual TEB。")
    report=f"""# PCT-SCOPE-RL、Actual TEB 与 DWA Gazebo 结果回填

## 可直接回填的实验环境

实验在 Ubuntu 20.04、ROS Noetic、Gazebo Classic 11 的独立 thesis 工作区完成。机器人为 Autolabor M2 Ackermann 模型，所有方法共享 1.20 m/s 最高速度、0.60 m/s² 加速度、0.90 m/s² 减速度、相同底盘/执行器动态、15 Hz 360° LaserScan、同一 warmup gate、目标、22 s 时限和 `/cmd_vel_navigation` 控制边界。研究仅为仿真，不使用真实车辆。

## 方法与公平性配置

- PCT-SCOPE-RL：冻结 PPO actor SHA `{sha256(ARTIFACT/'development/training/pct_benchmark_r1_seed9701/actor_final.pt')}`；8 个 Ackermann 可行候选，R2 唯一修改是左右低速空间候选由 0.4 调为 0.6 m/s。0.20 m 备份可行余量、0.33 m 膨胀、0.35 s 响应延迟与 1.50 s 停车后验证均不变。
- 解析先验：使用与 PCT 相同的候选生成、候选条件预测和硬验证，只以冻结解析排序选择。
- Actual TEB：`teb_local_planner/TebLocalPlannerROS` 0.8.4，开发集最强 v0 配置 SHA `{sha256(ROOT/'src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml')}`。
- DWA：开发集最强 v0 配置 SHA `{sha256(ROOT/'src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml')}`。
- 公平性：传统规划器获得独立且充分的 development 调参预算；最终选择按六场景失败计入均时，未降低其 1.20 m/s 上限。四方法不使用 Gazebo actor 真值做规划，actor state 只供事后评价。所有超时、未到达、接触、负保守净空和系统失败按 22 s 进入主指标，无删除、补跑或替换。

## 研究设计与样本量

R1 在 pilot 功效分析后使用 160 个 fresh paired blocks（640 episodes，seeds197001–197160）；R2 以互斥 development 单因素消融和新 pilot 为基础，按冻结功效规则使用 200 个 fresh paired blocks（800 episodes，seeds198001–198200）。每块含 PCT、解析先验、Actual TEB、DWA，顺序平衡。两项主比较分别使用 200,000 次配对 block bootstrap 和双侧 97.5% 区间；阳性需点改善≥8%、区间下界>0、完成率差≥-5pp、PCT contact 不更高且 mask violation=0。

## 主结果

| 方法 | R2 有效到达 | 完成率 | 失败计入均时 (s) | 最小保守净空 (m) | contact | mask violation |
|---|---:|---:|---:|---:|---:|---:|
"""+"\n".join(f"| {METHOD_LABEL[m]} | {v['valid_goals']}/200 | {100*v['completion_rate']:.1f}% | {v['mean_failure_inclusive_time_s']:.4f} | {v['minimum_conservative_clearance_m']:.4f} | {v['contact_count']} | {v['mask_violation_count']} |" for m,v in s.items())+f"""

| 正式研究 | 比较 | 相对降低 | 双侧 97.5% CI | 完成率差 | 全部门通过 |
|---|---|---:|---:|---:|---:|
| R1 | PCT vs TEB | {pct(r1['primary_comparisons']['actual_teb']['relative_reduction']):.2f}% | [{pct(r1['primary_comparisons']['actual_teb']['bootstrap']['lower_97_5_percent']):.2f}%, {pct(r1['primary_comparisons']['actual_teb']['bootstrap']['upper_97_5_percent']):.2f}%] | {r1['primary_comparisons']['actual_teb']['completion_rate_difference_percentage_points']:.1f} pp | {str(r1['primary_comparisons']['actual_teb']['positive_gate_pass']).lower()} |
| R1 | PCT vs DWA | {pct(r1['primary_comparisons']['dwa']['relative_reduction']):.2f}% | [{pct(r1['primary_comparisons']['dwa']['bootstrap']['lower_97_5_percent']):.2f}%, {pct(r1['primary_comparisons']['dwa']['bootstrap']['upper_97_5_percent']):.2f}%] | {r1['primary_comparisons']['dwa']['completion_rate_difference_percentage_points']:.1f} pp | {str(r1['primary_comparisons']['dwa']['positive_gate_pass']).lower()} |
| R2 | PCT vs TEB | {pct(teb['relative_reduction']):.2f}% | [{pct(teb['bootstrap']['lower_97_5_percent']):.2f}%, {pct(teb['bootstrap']['upper_97_5_percent']):.2f}%] | {teb['completion_rate_difference_percentage_points']:.1f} pp | {str(teb['positive_gate_pass']).lower()} |
| R2 | PCT vs DWA | {pct(dwa['relative_reduction']):.2f}% | [{pct(dwa['bootstrap']['lower_97_5_percent']):.2f}%, {pct(dwa['bootstrap']['upper_97_5_percent']):.2f}%] | {dwa['completion_rate_difference_percentage_points']:.1f} pp | {str(dwa['positive_gate_pass']).lower()} |

R2 阳性比较：{positives}。R1 是按原候选支持的合格阴性结果，不与 R2 合并；R2 使用全新 seeds 独立验证开发阶段预注册的单因素修复。

## 消融与机制诊断

development 配对消融的 R1 control/0.6 m/s treatment 均时为 {dev['method_summaries']['r1_control_v040']['mean_failure_inclusive_time_s']:.4f}/{dev['method_summaries']['r2_candidate_v060']['mean_failure_inclusive_time_s']:.4f} s；静态改善 {dev['effects']['static_mean_improvement_s']:.4f} s，动态均值变化 {dev['effects']['dynamic_mean_worsening_s']:.4f} s（负值为改善）。候选覆盖修复减少平均 certified-stop 数 {dev['method_summaries']['r1_control_v040']['mean_certified_stop_count']:.2f}→{dev['method_summaries']['r2_candidate_v060']['mean_certified_stop_count']:.2f}，同时 treatment contact=0、mask violation=0。首个 control 在语义前发生 catkin 导入失败，按 22 s 保留且不重试；删除整对 B001 的敏感性方向一致。

## 失败案例与限制

- PCT 保留 18 次超时/未到达与 2 次保守净空违例；后者使最小保守净空为 {s['pct_scope_rl']['minimum_conservative_clearance_m']:.4f} m，因而不支持形式安全声明。PCT contact=0 且 mask violation=0。
- 解析先验保留 29 次超时/未到达与 21 次执行完整性失败；这些系统失败均按 22 s 进入主指标。
- Actual TEB 保留 81 次、DWA 保留 100 次“到达但负保守净空”失败；没有删除、补跑或替换。
- R2 候选覆盖缩短静态绕障和部分恢复，但未消除 PCT 在连续交互中的安全等待/超时。
- 结论总体仅包含六个冻结 Gazebo 模板与随机种子，不是独立几何、感知噪声/OOD、多机器人、形式安全、部署或真实车辆证据。
- Actual TEB 的比较在 pilot 推算中即使 200 blocks 仍略低于 80% 目标功效；阴性不得解释为等效。

## 阴性结果归因与下一步方案

封存后的分层归因显示，PCT 相对 DWA 的总体收益来自 partially-occluded（{diagnostic['core_partially_occluded']['pct_minus_dwa_s']:.4f} s）和 static-avoidance（{diagnostic['core_static_avoidance']['pct_minus_dwa_s']:.4f} s），负值为 PCT 更快；但在 head-on-yield 与 single-crossing 分别慢 {diagnostic['core_head_on_yield']['pct_minus_dwa_s']:.4f} s 和 {diagnostic['core_single_crossing']['pct_minus_dwa_s']:.4f} s。head-on-yield 平均有 {diagnostic['core_head_on_yield']['pct_certified_stop_mean']:.2f} 次 certified stop；single-crossing 平均有 {diagnostic['core_single_crossing']['pct_candidate_switch_mean']:.2f} 次候选切换和 {diagnostic['core_single_crossing']['pct_certified_stop_mean']:.2f} 次 stop。sequential-interaction 中则有 {diagnostic['core_sequential_interaction']['pct_timeout_count']} 次超时，平均 {diagnostic['core_sequential_interaction']['pct_candidate_switch_mean']:.2f} 次切换、{diagnostic['core_sequential_interaction']['pct_certified_stop_mean']:.2f} 次 stop 和 {diagnostic['core_sequential_interaction']['pct_stop_recovery_mean']:.2f} 次恢复。这是与保守停车/切换抖动一致的机制证据，不是独立随机化的因果证明。

可证伪假设是：在不降低任何硬净空、contact 或 mask 门限的前提下，仅修改“安全候选间的切换滞回/停车恢复释放”，应能在新的 development head-on、single-crossing 和 sequential 集上同时降低 stop、switch 与失败计入时间，而不恶化最小净空。最小实验应只比较一个冻结滞回/恢复因子，事先要求动态均时、stop 和 switch 三者改善，且 contact=0、mask=0、净空不低于 control。解析先验的 21 次 integrity failure 由 LaserScan 时间戳 TF 合同失败触发，应作为独立的非行为性 TF-buffer/warmup 健壮性因子在 development 测试。任何未来实验都不得使用 seeds 197001–197160 或 198001–198200 调参；需重新冻结、功效分析、预注册并使用全新 formal seeds。

## 图表与机器可读表

- [图1：两轮正式效应与 97.5% CI](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_effects_97_5_ci.svg)
- [图2：R2 方法均时与完成率](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_r2_method_outcomes.svg)
- [图3：R2 六场景分层](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_r2_scene_strata.svg)
- [图4：R2 development 单因素消融](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/r2_development_ablation.svg)
- [两轮方法汇总 CSV](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_round_summary.csv)
- [正式效应 CSV](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_effect_summary.csv)
- [R2 场景分层 CSV](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_r2_scene_strata.csv)
- [R2 失败与机制诊断 CSV](artifacts/gazebo_pct_teb_dwa_benchmark/final_transfer/formal_r2_failure_diagnostics.csv)

## 推荐论文结论措辞

{conclusion} {teb_text} 该结论仅适用于冻结的单机器人 Gazebo 场景总体；不应扩展为一般 TEB 优势、形式安全、感知/OOD 泛化、部署或真实车辆结论。

## 核心证据散列

- R1 assessment: `{sha256(R1)}`
- R1 integrity review: `{sha256(ARTIFACT/'formal/round_1/formal_r1_integrity_review.json')}`
- R2 release: `{sha256(ARTIFACT/'formal/round_2/formal_r2_execution_release.yaml')}`
- R2 assessment: `{sha256(R2)}`
- R2 integrity review: `{sha256(ARTIFACT/'formal/round_2/formal_r2_integrity_review.json')}`
"""
    atomic(TARGET,report)
    hashes={p.name:sha256(p) for p in sorted(OUT.iterdir()) if p.is_file() and p.name!='final_transfer_integrity.json'};hashes[TARGET.name]=sha256(TARGET)
    final_review={'schema_version':'1.0','status':'thesis_result_transfer_pass','formal_r2_positive_comparators':r2['positive_comparators'],
      'transfer_path':str(TARGET.relative_to(ROOT)),'transfer_sha256':sha256(TARGET),'outputs':hashes,'hardware_accessed':False}
    atomic(OUT/'final_transfer_integrity.json',json.dumps(final_review,indent=2,sort_keys=True)+'\n')

    run=ARTIFACT/'run_manifest.csv'
    with run.open(newline='',encoding='utf-8') as stream:
        reader=csv.DictReader(stream);fields=reader.fieldnames;rows=[x for x in reader if x['stage']!=STAGE]
    stamp=datetime.now().astimezone().isoformat(timespec='seconds');rows.append({'stage':STAGE,'run_id':'thesis_result_transfer',
      'phase':'paper_transfer','method':'all','scene_id':'six_frozen_core_templates','split':'offline','seed':'none','attempt':1,
      'status':'completed_pass','started_at':stamp,'ended_at':stamp,'exit_code':0,'config_sha256':sha256(R2),'model_sha256':'bound_in_formal_releases',
      'evidence_path':str(TARGET.relative_to(ROOT)),'formal_eligible':'false','notes':'deterministic Chinese thesis transfer; no new execution'})
    temporary=run.with_suffix('.csv.tmp')
    with temporary.open('w',newline='',encoding='utf-8') as stream:
        w=csv.DictWriter(stream,fieldnames=fields);w.writeheader();w.writerows(rows)
    temporary.replace(run)
    run_rows=list(csv.DictReader(run.open()));episode_rows=list(csv.DictReader((ARTIFACT/'episode_results.csv').open()))
    if len({(x['stage'],x['run_id']) for x in run_rows})!=len(run_rows) or len({(x['study_round'],x['episode_id']) for x in episode_rows})!=len(episode_rows):raise RuntimeError('FINAL_LEDGER_DUPLICATE')
    integrity_path=ARTIFACT/'integrity_report.json';integrity=json.loads(integrity_path.read_text());integrity['status']='thesis_result_transfer_complete';integrity['formal_execution_authorized']=False
    integrity['checks']=[x for x in integrity['checks'] if x.get('id')!='p13_thesis_transfer'];integrity['checks'].append({'id':'p13_thesis_transfer','status':'pass_complete','transfer_sha256':sha256(TARGET),'review_sha256':sha256(OUT/'final_transfer_integrity.json'),'run_manifest_unique':True,'episode_results_unique':True})
    atomic(integrity_path,json.dumps(integrity,indent=2,sort_keys=True)+'\n')
    append_once(ARTIFACT/'progress.md','## P13 thesis result transfer','## P13 thesis result transfer\n\n- 中文论文回填、四张 SVG、四张机器表与终端完整性清单已生成；无新仿真、训练或硬件访问。\n- `THESIS_RESULT_TRANSFER.md` SHA 将由 `final_transfer/final_transfer_integrity.json` 绑定。')
    append_once(ARTIFACT/'decision_log.md','## D030 — 终端声明边界','## D030 — 终端声明边界\n\n- 日期：2026-08-10\n- 状态：终端\n- 决策：只使用各轮预注册门支持的比较措辞；不合并 R1/R2、不删除失败、不扩展到感知/OOD/部署/真实车辆。')
    append_once(ARTIFACT/'statistical_report.md','## P13 论文回填','## P13 论文回填\n\n主结果、区间、分层和失败清单已确定性导出至 `THESIS_RESULT_TRANSFER.md` 及 `final_transfer/`。')
    claims_path=ARTIFACT/'claims_evidence_matrix.yaml';claims=yaml.safe_load(claims_path.read_text());claims['stage']=STAGE;claims['status']='thesis_result_transfer_complete'
    claims['claims']=[x for x in claims['claims'] if x.get('id')!='p13_thesis_transfer'];claims['claims'].append({'id':'p13_thesis_transfer','status':'qualified_paper_transfer','evidence':{'path':str(TARGET.relative_to(ROOT)),'sha256':sha256(TARGET),'review_sha256':sha256(OUT/'final_transfer_integrity.json')}})
    atomic(claims_path,yaml.safe_dump(claims,sort_keys=False,allow_unicode=True))
    print(json.dumps(final_review,indent=2,sort_keys=True))


if __name__=='__main__':main()
