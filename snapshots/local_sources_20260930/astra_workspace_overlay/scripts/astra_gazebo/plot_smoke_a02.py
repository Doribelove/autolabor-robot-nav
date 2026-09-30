#!/usr/bin/env python3
"""Source-bound actual Gazebo overview panel, not a performance result."""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from matplotlib.backends.backend_pdf import PdfPages
import numpy as np
from PIL import Image

S = Path(__file__).resolve().parents[3]
OUT = S/'visuals/panels/bootstrap_a02/r01'
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
fp = FontProperties(fname=FONT)
plt.rcParams.update({'font.family':fp.get_name(),'font.size':8,'pdf.fonttype':42,'svg.fonttype':'none'})
METHODS = ['pct_scope_rl','analytic_prior','actual_teb','dwa']
TITLES = ['冻结 R2 PCT','解析先验','实际 TEB 0.8.4','实际 DWA 1.17.3']


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    OUT.mkdir(parents=True,exist_ok=False)
    chosen=[];all_rows=[];sources=[]
    for i,method in enumerate(METHODS):
        root=S/'runs'/('smoke_a02_{:02d}_'.format(i+1)+method)
        path=root/'evidence/frames.jsonl'
        rows=[json.loads(t) for t in path.read_text().splitlines()]
        for r in rows:
            assert Image.open(r['image_path']).size==(2560,1440)
            assert sha(r['image_path'])==r['image_sha256']
        all_rows+=rows
        row=min((r for r in rows if r['machine_qc']=='pass'),key=lambda r:abs(r['models']['autolabor_m2']['x']-.5))
        chosen.append(row)
        sources.extend([{'path':str(p),'sha256':sha(p)} for p in (path,root/'receipt.json',root/'scene_manifest.yaml')])
    # All raw frames are visible in the overview; only selected four are proposed
    # for delivery and receive separate individual-image/final-size review.
    with PdfPages(OUT/'all_raw_overview.pdf') as pdf:
        for offset in range(0,len(all_rows),16):
            fig,axs=plt.subplots(4,4,figsize=(12,8))
            for ax in axs.flat:ax.axis('off')
            for ax,r in zip(axs.flat,all_rows[offset:offset+16]):
                ax.imshow(Image.open(r['image_path']))
                ax.set_title('{} / t={:.3f}s / {}'.format(r['run_id'].replace('smoke_a02_',''),r['image_stamp_s'],r['machine_qc']),fontsize=6)
            fig.tight_layout();pdf.savefig(fig)
            fig.savefig(OUT/'overview_{:02d}.png'.format(offset//16),dpi=120)
            plt.close(fig)
    fig,axs=plt.subplots(2,2,figsize=(170/25.4,120/25.4))
    fig.subplots_adjust(left=.015,right=.985,top=.91,bottom=.17,wspace=.03,hspace=.25)
    fig.suptitle('A02 四方法 Gazebo 链路实拍检查',fontproperties=fp,fontsize=11,y=.98)
    for i,(ax,r,title) in enumerate(zip(axs.flat,chosen,TITLES)):
        ax.imshow(Image.open(r['image_path']));ax.axis('off')
        ax.set_title('({}) {} | t = {:.3f} s'.format('abcd'[i],title,r['image_stamp_s']),fontproperties=fp,fontsize=8,pad=5)
        focal=2560/(2*math.tan(r['camera']['horizontal_fov_rad']/2))
        def project(x,y):return (1280+focal*x/15,720-focal*y/15)
        for x,lab,color,marker in [(-4,'计时门','#0072B2','o'),(5,'目标','#D55E00','*')]:
            u,v=project(x,0)
            ax.scatter([u],[v],s=22,c=color,marker=marker,edgecolors='white',linewidths=.5)
            ax.text(u,v+135,lab,fontproperties=fp,fontsize=7,ha='center',color='#17232A',bbox={'facecolor':'white','alpha':.85,'edgecolor':'none','pad':1})
        x0,y0=project(-6,-1.7);x1,_=project(-4,-1.7)
        ax.plot([x0,x1],[y0,y0],color='#17232A',lw=1.3)
        ax.text((x0+x1)/2,y0+95,'2 m（地面）',fontproperties=fp,fontsize=6.7,ha='center')
    fig.text(.03,.10,'统一真实相机；每回合选机器质检通过且最接近路线中点 x = 0.5 m 的一帧。',fontproperties=fp,fontsize=7.3)
    fig.text(.03,.066,'图中仅一台简化 M2 实体；圆点/星号与比例尺为投影标注，不是新增模型。',fontproperties=fp,fontsize=7.3)
    fig.text(.03,.032,'本图验证记录链路与显示一致性；无转弯或动态障碍，不支持性能或完整安全结论。',fontproperties=fp,fontsize=7.3)
    for ext in ('pdf','svg','png'):fig.savefig(OUT/('Q01_smoke_overview.'+ext),dpi=600)
    plt.close(fig)
    subprocess.run(['pdftoppm','-png','-r','150','-singlefile',str(OUT/'Q01_smoke_overview.pdf'),str(OUT/'Q01_print_150dpi')],check=True)
    caption='图 Q01：四种控制链路的真实 Gazebo 俯视截图。使用同一无动态参与者的通道、相机、机器人与新种子260926002；每种方法仅一次管线资格回合。图像选择规则为机器质检通过且机器人x最接近路线中点0.5 m，标注实际全局仿真时间，各方法并非同一到达事件。模型为当前简化M2几何；圆点、星号及按地面投影计算的2 m比例尺仅为附加标注。原生2560×1440未裁剪原图全部保留。俯视画面不证明轮地接触，接触/高度须结合独立原始数据核算。四回合均到达目标，但该试验不含转弯、障碍互动或性能统计，不能作为性能优势或完整安全证据。'
    (OUT/'caption_zh.md').write_text(caption+'\n')
    (OUT/'include.tex').write_text('\\begin{figure}[htbp]\n\\centering\n\\includegraphics[width=170mm]{Q01_smoke_overview.pdf}\n\\caption{'+caption+'}\n\\label{fig:astra-smoke-pipeline}\n\\end{figure}\n')
    report={'figure_id':'Q01','purpose':'pipeline_only','source_resources':sources,
       'generator':{'path':str(Path(__file__).resolve()),'sha256':sha(__file__)},
       'command':'/usr/bin/python3 '+str(Path(__file__).resolve()),'font_sha256':sha(FONT),
       'raw_frame_count':len(all_rows),'selected_frame_count':len(chosen),'selected_frames':chosen,
       'selection_rule':'machine pass and closest actual robot world x to route midpoint 0.5m',
       'paper_width_mm':170,'paper_height_mm':120,'image_edit':'no raw pixel modification; overlaid projected ground labels and scale; no crop',
       'machine_qc':'source_hashes_and_native_dimensions_pass','visual_qc':'pending_actual_panel_and_print_view',
       'outputs':[{'path':str(p),'sha256':sha(p)} for p in sorted(OUT.iterdir()) if p.is_file()]}
    (OUT/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'output':str(OUT),'raw_frames':len(all_rows),'selected':len(chosen)}))


if __name__=='__main__':main()
