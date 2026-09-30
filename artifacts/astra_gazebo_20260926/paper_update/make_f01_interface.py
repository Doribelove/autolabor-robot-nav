#!/usr/bin/env python3
"""Source-bound interface schematic; intentionally no runtime/hardware claim."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

REPO=Path(__file__).resolve().parents[3]
S=Path(__file__).resolve().parents[1]
SOURCE=REPO/'docs/thesis_experiment/CURRENT_V3_SCOPE_FOUNDATION_HANDOFF.md'
OUT=S/'paper_update/figures/F01_command_boundary_r01'


def binding(path):
    return {'path':str(path.resolve()),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    assert SOURCE.is_file()
    content=SOURCE.read_text()
    for marker in ('/cmd_vel_navigation','/cmd_vel_gps','FOD mode arbiter','M2 sanitizer'):
        assert marker in content
    if OUT.exists():raise FileExistsError('preserve figure version')
    OUT.mkdir(parents=True)
    plt.rcParams.update({'font.family':'Noto Sans CJK JP','font.size':7.2,'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,ax=plt.subplots(figsize=(6.7,3.45),dpi=300)
    fig.subplots_adjust(left=.02,right=.98,top=.83,bottom=.12)
    ax.set_xlim(0,100);ax.set_ylim(0,63);ax.axis('off')
    blue=('#DFEEF4','#377993');green=('#E8F0E7','#5E8874');gray='#344653'
    def box(x,y,w,h,title,subtitle,palette):
        fill,edge=palette
        ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.35,rounding_size=1.0',
                                    facecolor=fill,edgecolor=edge,linewidth=1.2))
        ax.text(x+w/2,y+h*.63,title,ha='center',va='center',fontsize=8.0,weight='bold',color=gray)
        ax.text(x+w/2,y+h*.28,subtitle,ha='center',va='center',fontsize=6.3,color=gray)
    def arrow(a,b,label=None):
        ax.add_patch(FancyArrowPatch(a,b,arrowstyle='-|>',mutation_scale=10,linewidth=1.15,color='#53636B'))
        if label:
            ax.text((a[0]+b[0])/2,(a[1]+b[1])/2+2.2,label,ha='center',fontsize=6.2,color='#53636B')
    top=[(2,'感知与任务输入','扫描/里程计/TF'),(22,'场景与轨迹','静态几何/动态历史'),
         (42,'可行候选','Ackermann + stop'),(62,'交互预测','候选条件/占据管'),(82,'硬安全验证','净空/制动/接口')]
    for x,t,s in top:box(x,42,16,12,t,s,blue)
    for x in (18,38,58,78):arrow((x+.5,48),(x+3.1,48))
    box(82,14,16,12,'排序与跟踪','安全候选 → 命令',blue)
    arrow((90,41.3),(90,27.0))
    box(62,14,16,12,'导航命令接口','/cmd_vel_navigation',green)
    box(42,14,16,12,'目标限速与停锁','/cmd_vel_gps',green)
    box(22,14,16,12,'FOD 控制权仲裁','/cmd_vel',green)
    box(2,14,16,12,'M2 驱动边界','sanitizer / actuator',green)
    for x in (82,62,42,22):arrow((x-.5,20),(x-3.1,20))
    ax.text(50,31.5,'蓝色：候选方法接口    绿色：保留的实车命令边界',ha='center',fontsize=7.0,color='#435563')
    ax.text(50,4.0,'接口设计示意；未证明当前仿真已连通整条实车链，也未取得 shadow / 硬件验证。',
            ha='center',fontsize=6.5,color='#86503F')
    fig.suptitle('F01 规划器与既有 M2 命令安全边界',x=.5,y=.96,fontsize=11,weight='bold')
    for ext in ('png','pdf','svg'):fig.savefig(OUT/f'F01.{ext}',dpi=600 if ext=='png' else None)
    plt.close(fig)
    manifest={'schema':'astra_f01_interface_source_manifest_v1','figure_id':'F01',
              'claim_scope':'architecture/interface schematic only; no live physical chain or hardware qualification',
              'source':binding(SOURCE),'generator':binding(Path(__file__)),
              'images':{ext:binding(OUT/f'F01.{ext}') for ext in ('png','pdf','svg')}}
    (OUT/'source_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'manifest':str(OUT/'source_manifest.json'),'png':str(OUT/'F01.png')}))


if __name__=='__main__':main()
