#!/usr/bin/env python3
"""Six selected N05 development pairs from actual WorldState bag positions."""

import csv
import hashlib
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import rosbag
import yaml


STAGE = Path(__file__).resolve().parents[1]
PREPARED = STAGE/'contracts/navigation_n05_prepared_r01/prepared.json'
INDEPENDENT = STAGE/'audits/navigation_n05_terminal_independent/review.json'
OUT = STAGE/'paper_update/figures/NAV09_n05_executed_pairs_r01'
BLOCKS = [('b00','静态绕障'),('b02','单人横穿'),('b04','迎面让行'),
          ('b06','连续交互'),('b08','遮挡出现'),('b10','宽度变化')]
METHODS = [('astra_analytic_route_v1','route-v1','#266F9A','-'),
           ('astra_analytic_preview_eligible_n05','preview','#D4793E','--')]


def binding(path):
    path=Path(path).resolve(strict=True)
    return {'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


def trajectory(path):
    rows=[]
    with rosbag.Bag(str(path)) as bag:
        for _,msg,_ in bag.read_messages(topics=['/astra/world_models']):
            assert msg.header.frame_id=='world' and msg.name.count('autolabor_m2')==1
            pose=msg.pose[msg.name.index('autolabor_m2')]
            t=msg.header.stamp.to_nsec()
            xy=(float(pose.position.x),float(pose.position.y))
            if rows and t<=rows[-1][0]:raise ValueError('nonincreasing model source time')
            if not all(math.isfinite(x) for x in xy):raise ValueError('nonfinite executed pose')
            rows.append((t,*xy))
    if len(rows)<500:raise ValueError('insufficient actual model-state samples')
    return rows


def main():
    assert INDEPENDENT.is_file()
    prepared=json.loads(PREPARED.read_text())
    if OUT.exists():raise FileExistsError('preserve figure revision')
    OUT.mkdir(parents=True)
    all_rows=[];source=[];panels=[]
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':7.0,
                         'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,axes=plt.subplots(2,3,figsize=(6.7,5.45),dpi=300)
    fig.subplots_adjust(left=.10,right=.985,top=.865,bottom=.15,wspace=.30,hspace=.49)
    for ax,(block,title) in zip(axes.flat,BLOCKS):
        selected=[u for u in prepared['units'] if '_'+block+'_' in u['run_id']]
        assert len(selected)==2 and {u['method'] for u in selected}=={m[0] for m in METHODS}
        contracts={}
        for u in selected:
            cpath=Path(u['files'][0]['path']);contract=json.loads(cpath.read_text())
            scene_path=Path(contract['scene_manifest']['path'])
            scene=yaml.safe_load(scene_path.read_text())
            contracts[u['method']]=(u,cpath,contract,scene_path,scene)
        a=contracts[METHODS[0][0]][4];b=contracts[METHODS[1][0]][4]
        assert a['canonical_geometry_sha256']==b['canonical_geometry_sha256']
        assert a['layout_id']==b['layout_id'] and a['route']==b['route']
        for box in a['static_boxes']:
            ax.add_patch(Rectangle((box['x']-box['length']/2,box['y']-box['width']/2),box['length'],box['width'],
                                   facecolor='#99A4AA',edgecolor='#68757B',linewidth=.25))
        for method,label,color,style in METHODS:
            u,cpath,contract,scene_path,scene=contracts[method]
            root=STAGE/'runs'/u['run_id'];bag=root/'fixture.bag';assessment=root/'assessment.json'
            report=json.loads(assessment.read_text())
            assert report['capture_integrity_pass'] is True and report['run_id']==u['run_id']
            path=trajectory(bag)
            for t,x,y in path:all_rows.append([block,u['run_id'],u['seed'],method,t,x,y])
            xs=[r[1] for r in path];ys=[r[2] for r in path]
            ax.plot(xs,ys,color=color,lw=1.7,ls=style,label=label)
            ax.scatter([xs[-1]],[ys[-1]],marker='o',s=14,color=color,zorder=4)
            source.extend([binding(bag),binding(assessment),binding(cpath),binding(scene_path)])
            goal=scene['route']['goal'];distance=math.hypot(xs[-1]-goal[0],ys[-1]-goal[1])
            panels.append({'block':block,'run_id':u['run_id'],'method':method,'source_samples':len(path),
                           'source_start_ns':path[0][0],'source_end_ns':path[-1][0],
                           'goal_distance_m_from_last_model_sample':distance,
                           'task_phase':report['task']['task_phase'],
                           'scene_geometry_sha256':scene['canonical_geometry_sha256']})
        start=a['route']['spawn'];goal=a['route']['goal']
        ax.plot(start[0],start[1],marker='s',ms=4,color='#2662B8')
        ax.plot(goal[0],goal[1],marker='*',ms=7,color='#26836D')
        ax.set_xlim(-6,6);ax.set_ylim(-3.6,3.6)
        ax.set_aspect('equal',adjustable='box')
        ax.grid(alpha=.25,lw=.4)
        ax.set_title(f'{title} · {block}',fontsize=8.2,pad=4)
        ax.tick_params(labelsize=6)
    axes[0,0].legend(loc='lower left',fontsize=6.3,framealpha=.95)
    for ax in axes[1]:ax.set_xlabel('x / m',fontsize=6.7)
    for ax in axes[:,0]:ax.set_ylabel('y / m',fontsize=6.7)
    fig.suptitle('NAV09 真实 model-state 轨迹：N05 两臂六任务族开发配对',
                 fontsize=10.5,weight='bold',y=.965)
    fig.text(.5,.085,'每族仅选首个开发块；两臂同布局/seed，线为真实模型位置，圆点为末样本；灰色=静态输入障碍。',
             ha='center',fontsize=6.5,color='#465A65')
    fig.text(.5,.047,'不是五方法正式比较；六个块含失败，不能由路径长度推断安全或有效完成。',
             ha='center',fontsize=6.5,color='#85513E')
    for ext in ('png','pdf','svg'):
        fig.savefig(OUT/('NAV09.'+ext),dpi=600 if ext=='png' else None)
    plt.close(fig)
    with (OUT/'model_state_samples.csv').open('w',newline='') as f:
        w=csv.writer(f);w.writerow(('block','run_id','seed','method','source_ns','x_m','y_m'));w.writerows(all_rows)
    report={'schema':'astra_nav09_n05_executed_pair_trajectory_v1','status':'descriptive_development_only',
            'block_count':6,'episode_count':12,'sample_count':len(all_rows),
            'selection':'first N05 development replicate of each of six task families, both paired arms',
            'source_topic':'/astra/world_models; autolabor_m2 in world frame',
            'panels':panels,'source_bindings':source,
            'prepared':binding(PREPARED),'independent_terminal_review':binding(INDEPENDENT),
            'samples':binding(OUT/'model_state_samples.csv'),'generator':binding(Path(__file__)),
            'images':{ext:binding(OUT/('NAV09.'+ext)) for ext in ('png','pdf','svg')},
            'claim_limit':'selected N05 development trajectories, not independent/final performance'}
    (OUT/'source_manifest.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(OUT,len(all_rows))


if __name__=='__main__':main()
