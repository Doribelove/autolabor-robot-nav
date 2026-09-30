#!/usr/bin/env python3
"""Exploratory actual-speed audit of all terminal N05 development episodes."""

import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import rosbag


STAGE=Path(__file__).resolve().parents[1]
PREPARED=STAGE/'contracts/navigation_n05_prepared_r01/prepared.json'
INDEPENDENT=STAGE/'audits/navigation_n05_terminal_independent/review.json'
OUT=STAGE/'paper_update/figures/NAV11_n05_actual_speed_r02'
FAMILIES=[('b00–01','静态绕障'),('b02–03','单人横穿'),('b04–05','迎面让行'),
          ('b06–07','连续交互'),('b08–09','遮挡出现'),('b10–11','宽度变化')]
METHODS={'astra_analytic_route_v1':('route-v1','#236E9A'),
         'astra_analytic_preview_eligible_n05':('preview','#D17D40')}
SLOW_MPS=.05


def binding(path):
    path=Path(path).resolve(strict=True)
    return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


def samples(path,first_ns,last_ns):
    result=[];previous=None
    with rosbag.Bag(str(path)) as bag:
        assert bag.get_type_and_topic_info().topics['/odom'].msg_type=='nav_msgs/Odometry'
        for _,msg,_ in bag.read_messages(topics=['/odom']):
            ns=msg.header.stamp.to_nsec()
            if previous is not None and ns<=previous:raise ValueError('nonmonotone actual odom source')
            previous=ns
            if ns<first_ns or ns>last_ns:continue
            v=float(msg.twist.twist.linear.x)
            if not math.isfinite(v):raise ValueError('nonfinite actual speed')
            result.append((ns,v))
    assert result[0][0]==first_ns and result[-1][0]==last_ns
    assert len(result)==(last_ns-first_ns)//20000000+1
    assert all(b[0]-a[0]==20000000 for a,b in zip(result,result[1:]))
    return result


def main():
    prepared=json.loads(PREPARED.read_text())
    assert len(prepared['units'])==24 and INDEPENDENT.is_file()
    if OUT.exists():raise FileExistsError('preserve prior figure revision')
    OUT.mkdir(parents=True)
    sources=[];summaries=[];raw=[]
    for u in prepared['units']:
        path=STAGE/'runs'/u['run_id']
        assessment_path=path/'assessment.json'
        assessment=json.loads(assessment_path.read_text())
        assert assessment['capture_integrity_pass'] is True and assessment['run_id']==u['run_id']
        first=assessment['physical_status_audit']['coverage']['first_ns']
        last=assessment['physical_status_audit']['coverage']['last_ns']
        assert assessment['task']['terminal_source_ns']==last
        bag=path/'fixture.bag'
        rows=samples(bag,first,last)
        absolute=sorted(abs(v) for _,v in rows)
        slow=sum(v<SLOW_MPS for v in absolute)/len(absolute)
        p90=absolute[int(.9*(len(absolute)-1))]
        block=u['run_id'].split('_b',1)[1][:2]
        summaries.append({'block':block,'run_id':u['run_id'],'method':u['method'],
                          'task_phase':assessment['task']['task_phase'],
                          'first_ns':first,'terminal_ns':last,'sample_count':len(rows),
                          'slow_sample_fraction':slow,'abs_linear_speed_p90_mps':p90})
        raw.extend((u['run_id'],block,u['method'],ns,v,abs(v)) for ns,v in rows)
        sources.extend((binding(bag),binding(assessment_path)))
    assert len(summaries)==24 and len(sources)==48
    assert sum(x['task_phase']=='timeout' for x in summaries)==20
    with (OUT/'actual_odom_speed_samples.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(('run_id','block','method','source_ns','linear_x_mps','absolute_linear_x_mps'));w.writerows(raw)
    with (OUT/'per_episode_values.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(summaries[0]));w.writeheader();w.writerows(summaries)
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':7,
                         'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,axes=plt.subplots(1,2,figsize=(6.7,4.35),dpi=300)
    fig.subplots_adjust(left=.17,right=.985,top=.81,bottom=.24,wspace=.30)
    for i,((family,title)) in enumerate(FAMILIES):
        blocks={f'{2*i:02d}',f'{2*i+1:02d}'}
        members=[x for x in summaries if x['block'] in blocks]
        assert len(members)==4
        for block in sorted(blocks):
            pair=[x for x in members if x['block']==block]
            assert {x['method'] for x in pair}==set(METHODS)
            y=i+(-.13 if int(block)%2==0 else .13)
            for ax,key in zip(axes,('slow_sample_fraction','abs_linear_speed_p90_mps')):
                ax.plot([x[key] for x in pair],[y,y],color='#B5BEC2',lw=.8,zorder=1)
                for x in pair:
                    _,color=METHODS[x['method']]
                    marker='o' if x['task_phase']=='complete' else 'x'
                    ax.plot(x[key],y,marker=marker,ms=4.8,mew=1.2,color=color,zorder=3)
    for ax in axes:
        ax.set_ylim(5.55,-.55)
        ax.set_yticks(range(6))
        ax.set_yticklabels([f'{n}  {f}' for f,n in FAMILIES],fontsize=6.8)
        ax.grid(axis='x',alpha=.26,lw=.4)
        ax.tick_params(labelsize=6.5)
    axes[1].tick_params(labelleft=False)
    axes[0].set_xlim(-.035,1.035)
    axes[0].set_xlabel('|实际纵向速度| < 0.05 m/s 的样本比例',fontsize=7)
    axes[1].set_xlim(-.05,1.30)
    axes[1].set_xlabel('|实际纵向速度| 的第90百分位 / m/s',fontsize=7)
    axes[0].legend(handles=[Line2D([],[],color=METHODS[m][1],marker='x',ls='',label=METHODS[m][0]) for m in METHODS],
                   loc='lower right',fontsize=6.2,framealpha=.95)
    fig.suptitle('NAV11 N05 真实里程计速度：停滞与任务结果并列',fontsize=10.1,weight='bold',y=.955)
    fig.text(.5,.129,'每族两组独立开发配对（上下两个灰色连接线）；×超时、○完成；所有24回合均显示。',
             ha='center',fontsize=6.5,color='#455965')
    fig.text(.5,.083,'50 Hz完整源时间窗；0.05 m/s仅探索性描述阈值。速度不证明碰撞安全或正式性能。',
             ha='center',fontsize=6.5,color='#87513E')
    for ext in ('png','pdf','svg'):
        fig.savefig(OUT/('NAV11.'+ext),dpi=600 if ext=='png' else None)
    plt.close(fig)
    report={'schema':'astra_nav11_n05_actual_speed_v1','status':'descriptive_development_only',
            'episode_count':24,'sample_count':len(raw),'source_rate_hz':50,
            'slow_threshold_mps_exploratory':SLOW_MPS,
            'source_topic':'/odom nav_msgs/Odometry; twist.twist.linear.x',
            'task_window':'physical_status_audit.coverage.first_ns to task.terminal_source_ns inclusive',
            'summaries':summaries,'source_bindings':sources,
            'prepared':binding(PREPARED),'independent_terminal_review':binding(INDEPENDENT),
            'raw_samples':binding(OUT/'actual_odom_speed_samples.csv'),
            'values':binding(OUT/'per_episode_values.csv'),'generator':binding(Path(__file__)),
            'images':{ext:binding(OUT/('NAV11.'+ext)) for ext in ('png','pdf','svg')},
            'claim_limit':'N05 development odometry speed diagnostic, no formal performance or safety inference'}
    (OUT/'source_manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'episodes':24,'samples':len(raw),'output':str(OUT)}))


if __name__=='__main__':main()
