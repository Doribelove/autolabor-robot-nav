#!/usr/bin/env python3
"""Plot the six-family D04 development geometry allocation without outcomes."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,Patch

S=Path(__file__).resolve().parents[1]
LAYOUTS=S/'scenes/development_d04_geometry_r05/layouts.json'
REVIEW=S/'audits/development_geometry_d04_r05_independent/review.json'
OUT=S/'paper_update/figures/F05_d04_development_split_r02'

FAMILIES=[('static_avoidance','静态绕障'),('single_crossing','单次横穿'),('head_on_yield','迎面让行'),
          ('sequential_interaction','连续交互'),('occluded_emergence','遮挡出现'),('width_change_corridor','宽度变化')]

def binding(path):return {'path':str(path.resolve()),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}

def main():
    if OUT.exists():raise FileExistsError('preserve prior version')
    rows=json.loads(LAYOUTS.read_text());review=json.loads(REVIEW.read_text())
    assert len(rows)==24 and len({r['parent_geometry_id'] for r in rows})==24
    assert review['status']=='independent_offline_joint_split_dedup_and_four_static_raster_review_pass'
    cells={}
    for r in rows:
        fid=r['family'];idx=int(r['layout_id'].rsplit('_',1)[1]);key=(fid,idx)
        assert key not in cells and fid in dict(FAMILIES) and idx in range(4)
        cells[key]=r
    assert len(cells)==24
    retained=sum(r['parent_geometry_retained_from_r04'] is True for r in rows)
    assert retained==17 and 24-retained==7
    OUT.mkdir(parents=True)
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':7.8,'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,ax=plt.subplots(figsize=(6.7,3.7),dpi=300)
    fig.subplots_adjust(left=.19,right=.93,top=.80,bottom=.27)
    ax.set_xlim(-.3,4.5);ax.set_ylim(-.55,5.55);ax.invert_yaxis();ax.axis('off')
    colors={True:'#DDE2E7',False:'#52A0A2'}
    for y,(fid,label) in enumerate(FAMILIES):
        ax.text(-.42,y,label,ha='right',va='center',fontsize=8.2,color='#273C49')
        fresh=0
        for x in range(4):
            r=cells[(fid,x)];old=r['parent_geometry_retained_from_r04'];fresh+=not old
            ax.add_patch(Rectangle((x-.40,y-.34),.80,.68,facecolor=colors[old],edgecolor='white',linewidth=1.5))
            ax.text(x,y,f'{x:03d}',ha='center',va='center',fontsize=8.5,weight='bold',color='#173A45' if old else 'white')
        ax.text(3.72,y,f'+{fresh} new',ha='left',va='center',fontsize=7.2,color='#377F83')
    for x in range(4):ax.text(x,-.48,f'layout {x:03d}',ha='center',va='center',fontsize=7.0,color='#4A5C68')
    ax.legend(handles=[Patch(facecolor=colors[True],label='r04 retained parent'),Patch(facecolor=colors[False],label='new D04 parent')],
              loc='lower center',bbox_to_anchor=(.46,-.24),ncol=2,frameon=False,fontsize=7.4)
    fig.suptitle('F05 D04 开发几何：六类场景 × 四个父布局',x=.51,y=.96,fontsize=11,weight='bold')
    fig.text(.04,.135,'24 个开发集父布局/组合：17 个继承 r04，7 个 D04 新增；同一布局的执行随机种子不等于新几何。',
             fontsize=6.8,color='#3E5260')
    fig.text(.04,.065,'仅离线场景与分组检查；无训练、Gazebo 物理运行、导航成功率或独立几何泛化结论。',
             fontsize=6.6,color='#86503F')
    for ext in ('png','pdf','svg'):fig.savefig(OUT/f'F05.{ext}',dpi=600 if ext=='png' else None)
    plt.close(fig)
    manifest={'schema':'astra_f05_d04_development_split_source_manifest_v1','figure_id':'F05',
              'scope':'offline six-family development parent layout allocation; not sampled execution seeds',
              'layouts':binding(LAYOUTS),'independent_review':binding(REVIEW),'generator':binding(Path(__file__)),
              'count':{'families':6,'parents':24,'retained':17,'new':7},
              'images':{ext:binding(OUT/f'F05.{ext}') for ext in ('png','pdf','svg')}}
    (OUT/'source_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'source_manifest':str(OUT/'source_manifest.json'),'png':str(OUT/'F05.png')}))

if __name__=='__main__':main()
