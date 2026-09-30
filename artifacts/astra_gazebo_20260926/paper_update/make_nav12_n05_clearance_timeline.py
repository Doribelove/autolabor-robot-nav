#!/usr/bin/env python3
"""Show when the four frozen N05 conservative-clearance failures occurred."""

import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


STAGE=Path(__file__).resolve().parents[1]
INDEPENDENT=STAGE/'audits/navigation_n05_terminal_independent/review.json'
OUT=STAGE/'paper_update/figures/NAV12_n05_clearance_timeline_r01'
METHODS=(('astra_analytic_route_v1','route-v1','#236E9A'),
         ('astra_analytic_preview_eligible_n05','preview','#D17D40'))


def binding(path):
    path=Path(path).resolve(strict=True)
    return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    assert INDEPENDENT.is_file()
    if OUT.exists():raise FileExistsError('preserve previous figure revision')
    OUT.mkdir(parents=True)
    sources=[];series=[];rows=[]
    for block in ('b04','b05'):
        for method,label,color in METHODS:
            run_id=f'astra_navigation_n05_{block}_{method}'
            root=STAGE/'runs'/run_id
            assessment_path=root/'assessment.json'
            assessment=json.loads(assessment_path.read_text())
            report_path=root/'sweep_evaluation/report.json'
            report=json.loads(report_path.read_text())
            interval_path=root/'sweep_evaluation/intervals.jsonl'
            assert assessment['run_id']==run_id and assessment['capture_integrity_pass'] is True
            assert assessment['safety_within_declared_engineering_scope_pass'] is False
            assert report['integrity_pass'] is True and report['clearance_status']=='fail_or_unavailable'
            assert report['formal_safety'] is False and report['real_continuous_physical_safety'] is False
            first,last=report['source_window_ns']
            assert first==assessment['physical_status_audit']['coverage']['first_ns']
            assert last==assessment['task']['terminal_source_ns']
            coarse=report['coarse_intervals']
            assert len(coarse)==1100 and coarse[0]['start_ns']==first and coarse[-1]['end_ns']==last
            assert all(a['end_ns']==b['start_ns'] for a,b in zip(coarse,coarse[1:]))
            assert all(x['end_ns']-x['start_ns']==20000000 for x in coarse)
            values=[]
            for x in coarse:
                v=float(x['conditional_clearance_lower_bound_m'])
                assert math.isfinite(v)
                t=(x['start_ns']+x['end_ns']-2*first)/2e9
                values.append((t,v))
                rows.append((run_id,block,method,x['start_ns'],x['end_ns'],t,v))
            minimum=min(v for _,v in values)
            assert abs(minimum-report['minimum_engineering_conservative_swept_xy_m'])<1e-10
            negative=[t for t,v in values if v<0]
            assert negative
            series.append({'run_id':run_id,'block':block,'method':method,'label':label,
                           'color':color,'values':values,'minimum_m':minimum,
                           'negative_interval_count':len(negative),
                           'first_negative_elapsed_s':negative[0],
                           'last_negative_elapsed_s':negative[-1]})
            sources.extend(map(binding,(assessment_path,report_path,interval_path)))
    with (OUT/'coarse_clearance_values.csv').open('w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(('run_id','block','method','start_ns','end_ns','mid_elapsed_s','conservative_lower_m'))
        writer.writerows(rows)
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':7,
                         'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,axes=plt.subplots(2,1,figsize=(6.7,5.0),dpi=300)
    fig.subplots_adjust(left=.12,right=.985,top=.85,bottom=.17,hspace=.44)
    for ax in axes:
        ax.axhspan(-.3,0,color='#F2DFDA',alpha=.8,zorder=0)
        ax.axhline(0,color='#A95747',lw=.8,zorder=1)
        ax.grid(alpha=.24,lw=.4)
        ax.set_ylabel('保守净距下界 / m',fontsize=7)
        ax.tick_params(labelsize=6.5)
    axes[0].axvspan(5.5,9.5,color='#E9F0F3',alpha=.55,zorder=0)
    for x in series:
        style='-' if x['block']=='b04' else (0,(4,2))
        legend=f"{x['block']} {x['label']}"
        for ax in axes:
            ax.plot([t for t,_ in x['values']],[v for _,v in x['values']],
                    color=x['color'],ls=style,lw=1.35,label=legend)
    axes[0].set_xlim(0,22);axes[0].set_ylim(-.28,1.75)
    axes[0].set_title('完整22 s任务窗口：四条旧开发失败回合',fontsize=8.3,pad=5)
    axes[1].set_xlim(5.5,9.5);axes[1].set_ylim(-.22,1.05)
    axes[1].set_title('局部放大：负下界均出现在6–9 s附近',fontsize=8.3,pad=5)
    axes[1].set_xlabel('从各回合任务窗起点起 / s',fontsize=7)
    axes[0].legend(ncol=2,loc='upper right',fontsize=6.2,framealpha=.95)
    fig.suptitle('NAV12 N05 四次保守净距失败发生的源时间',fontsize=10.2,weight='bold',y=.963)
    fig.text(.5,.095,'实线b04、虚线b05；蓝route-v1、橙preview；红色区域为工程下界<0，灰蓝区为下图放大范围。',
             ha='center',fontsize=6.4,color='#485B64')
    fig.text(.5,.054,'负下界触发工程安全门失败；不等同实际接触，也不是形式或实车安全证明。',
             ha='center',fontsize=6.4,color='#87513E')
    for ext in ('png','pdf','svg'):
        fig.savefig(OUT/('NAV12.'+ext),dpi=600 if ext=='png' else None)
    plt.close(fig)
    report={'schema':'astra_nav12_n05_clearance_timeline_v1','status':'negative_development_safety_diagnostic',
            'episode_count':4,'interval_count':len(rows),'interval_duration_ns':20000000,
            'plot_values_source':'sweep_evaluation/report.json coarse_intervals conditional_clearance_lower_bound_m',
            'summaries':[{k:v for k,v in x.items() if k not in ('values','color')} for x in series],
            'source_bindings':sources,'independent_terminal_review':binding(INDEPENDENT),
            'values':binding(OUT/'coarse_clearance_values.csv'),'generator':binding(Path(__file__)),
            'images':{ext:binding(OUT/('NAV12.'+ext)) for ext in ('png','pdf','svg')},
            'claim_limit':'four N05 conservative engineering lower-bound failures; no actual-contact/formal/real safety inference'}
    (OUT/'source_manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'episodes':4,'intervals':len(rows),'output':str(OUT)}))


if __name__=='__main__':main()
