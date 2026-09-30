#!/usr/bin/env python3
"""Plot goal-distance histories from all 24 frozen N05 Gazebo model-state bags."""

import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rosbag
import yaml


STAGE = Path(__file__).resolve().parents[1]
PREPARED = STAGE/'contracts/navigation_n05_prepared_r01/prepared.json'
INDEPENDENT = STAGE/'audits/navigation_n05_terminal_independent/review.json'
OUT = STAGE/'paper_update/figures/NAV10_n05_goal_progress_r01'
FAMILIES = [('b00','静态绕障'),('b02','单人横穿'),('b04','迎面让行'),
            ('b06','连续交互'),('b08','遮挡出现'),('b10','宽度变化')]
METHODS = {'astra_analytic_route_v1':('route-v1','#236E9A'),
           'astra_analytic_preview_eligible_n05':('preview','#D17D40')}


def binding(path):
    path = Path(path).resolve(strict=True)
    return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


def world_samples(path, start_ns, end_ns, goal):
    rows=[];last_ns=None
    with rosbag.Bag(str(path)) as bag:
        for _,msg,_ in bag.read_messages(topics=['/astra/world_models']):
            assert msg.header.frame_id=='world' and msg.name.count('autolabor_m2')==1
            ns=msg.header.stamp.to_nsec()
            if last_ns is not None and ns<=last_ns:
                raise ValueError('nonmonotone world-model source stamp')
            last_ns=ns
            if ns<start_ns or ns>end_ns:continue
            pose=msg.pose[msg.name.index('autolabor_m2')]
            x,y=float(pose.position.x),float(pose.position.y)
            if not all(math.isfinite(v) for v in (x,y)):
                raise ValueError('nonfinite actual model-state position')
            rows.append((ns,(ns-start_ns)/1e9,math.hypot(x-goal[0],y-goal[1])))
    if len(rows)<300 or rows[0][0]-start_ns>40000000 or end_ns-rows[-1][0]>40000000:
        raise ValueError('incomplete actual model-state task window')
    return rows


def main():
    prepared=json.loads(PREPARED.read_text())
    assert INDEPENDENT.is_file() and len(prepared['units'])==24
    if OUT.exists():raise FileExistsError('preserve prior figure revision')
    OUT.mkdir(parents=True)
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':7,
                         'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,axes=plt.subplots(2,3,figsize=(6.7,5.25),dpi=300)
    fig.subplots_adjust(left=.10,right=.985,top=.84,bottom=.16,wspace=.32,hspace=.50)
    all_rows=[];panels=[];sources=[]
    for ax,(family,title) in zip(axes.flat,FAMILIES):
        family_blocks={family[1:],f'{int(family[1:])+1:02d}'}
        members=[u for u in prepared['units']
                 if u['run_id'].split('_b',1)[1][:2] in family_blocks]
        assert len(members)==4
        assert {u['method'] for u in members}==set(METHODS)
        family_panels=[]
        for u in sorted(members,key=lambda x:x['run_id']):
            cpath=Path(u['files'][0]['path'])
            contract=json.loads(cpath.read_text())
            scene_path=Path(contract['scene_manifest']['path'])
            scene=yaml.safe_load(scene_path.read_text())
            assessment_path=STAGE/'runs'/u['run_id']/'assessment.json'
            assessment=json.loads(assessment_path.read_text())
            assert assessment['capture_integrity_pass'] is True
            assert assessment['run_id']==u['run_id']
            start=assessment['physical_status_audit']['coverage']['first_ns']
            end=assessment['physical_status_audit']['coverage']['last_ns']
            assert assessment['task']['terminal_source_ns']==end
            goal=scene['route']['goal']
            bag=STAGE/'runs'/u['run_id']/'fixture.bag'
            samples=world_samples(bag,start,end,goal)
            block=u['run_id'].split('_b',1)[1][:2]
            assert int(block) in (int(family[1:]),int(family[1:])+1)
            label,color=METHODS[u['method']]
            style='-' if int(block)%2==0 else (0,(3,2))
            xs=[row[1] for row in samples];ys=[row[2] for row in samples]
            ax.plot(xs,ys,color=color,ls=style,lw=1.5,alpha=.95,
                    label=label if int(block)%2==0 else None)
            marker='o' if assessment['task']['task_phase']=='complete' else 'x'
            ax.plot(xs[-1],ys[-1],marker=marker,ms=3.5,color=color,mew=1.1)
            for ns,t,d in samples:all_rows.append([family,block,u['run_id'],u['method'],ns,t,d])
            family_panels.append({'block':block,'run_id':u['run_id'],'method':u['method'],
                                  'task_phase':assessment['task']['task_phase'],
                                  'first_ns':start,'terminal_ns':end,'source_samples':len(samples),
                                  'last_distance_m':ys[-1],
                                  'canonical_geometry_sha256':scene['canonical_geometry_sha256']})
            sources.extend(map(binding,(bag,assessment_path,cpath,scene_path)))
        assert len({x['canonical_geometry_sha256'] for x in family_panels if x['block']==family[1:]})==1
        assert len({x['canonical_geometry_sha256'] for x in family_panels if x['block']==f'{int(family[1:])+1:02d}'})==1
        ax.axhline(.25,color='#74917D',lw=.7,ls=':',zorder=0)
        ax.set_xlim(0,22.3)
        ax.set_ylim(bottom=0)
        ax.grid(alpha=.22,lw=.4)
        n_timeout=sum(x['task_phase']=='timeout' for x in family_panels)
        ax.set_title(f'{title} · {family}–b{int(family[1:])+1:02d} · {n_timeout}/4超时',fontsize=7.5,pad=4)
        ax.tick_params(labelsize=6)
        panels.extend(family_panels)
    axes[0,0].legend(loc='upper right',fontsize=6.2,framealpha=.95)
    for ax in axes[1]:ax.set_xlabel('从状态覆盖首样本起 / s',fontsize=6.5)
    for ax in axes[:,0]:ax.set_ylabel('距目标 / m',fontsize=6.5)
    fig.suptitle('NAV10 N05 全24回合目标距离历程：失败不删样本',fontsize=10.2,weight='bold',y=.96)
    fig.text(.5,.091,'蓝/橙：route-v1/preview；实线/虚线：各族第1/2配对；×超时，○完成；绿虚线为0.25 m位置容差。',
             ha='center',fontsize=6.3,color='#445764')
    fig.text(.5,.052,'仅实际Gazebo模型位置；位置容差不含朝向或安全判据；开发诊断，不能推断正式性能。',
             ha='center',fontsize=6.3,color='#87513E')
    for ext in ('png','pdf','svg'):
        fig.savefig(OUT/('NAV10.'+ext),dpi=600 if ext=='png' else None)
    plt.close(fig)
    with (OUT/'goal_distance_samples.csv').open('w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(('family','block','run_id','method','source_ns','from_status_first_s','goal_distance_m'))
        writer.writerows(all_rows)
    assert len(panels)==24 and sum(x['task_phase']=='timeout' for x in panels)==20
    report={'schema':'astra_nav10_n05_goal_progress_v1','status':'descriptive_development_only',
            'episode_count':24,'family_count':6,'source_sample_count':len(all_rows),
            'window':'physical_status_audit.coverage.first_ns through task.terminal_source_ns inclusive',
            'world_source_topic':'/astra/world_models; autolabor_m2 in world frame',
            'panels':panels,'source_bindings':sources,'prepared':binding(PREPARED),
            'independent_terminal_review':binding(INDEPENDENT),
            'samples':binding(OUT/'goal_distance_samples.csv'),'generator':binding(Path(__file__)),
            'images':{ext:binding(OUT/('NAV10.'+ext)) for ext in ('png','pdf','svg')},
            'claim_limit':'all 24 N05 development episodes, no final performance or safety inference'}
    (OUT/'source_manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'episodes':len(panels),'samples':len(all_rows),'output':str(OUT)}))


if __name__=='__main__':main()
