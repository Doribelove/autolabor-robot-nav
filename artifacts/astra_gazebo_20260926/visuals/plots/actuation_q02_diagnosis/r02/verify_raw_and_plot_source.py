#!/usr/bin/env python3
"""Exact source bag/JSON comparison and paper-size export; no ROS initialization."""
from pathlib import Path
import csv,json,hashlib,sys,subprocess,math
import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from PIL import Image
OUT=Path(__file__).resolve().parent;S=OUT.parents[2];RUN=S/'runs/actuation_q02';PANELS=S/'visuals/panels/actuation_q02_diagnosis';PANELS.mkdir(parents=True,exist_ok=True)
raw=json.loads((RUN/'probe_raw.json').read_text());num=json.loads((OUT/'numeric_diagnosis.json').read_text());report=json.loads((RUN/'probe_report.json').read_text());sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
# Independent serialized ROS bag comparison, with header source times as keys.
bytopic={'/astra/world_links':('links',{r['ns']:r for r in raw['links']}),'/joint_states':('joints',{r['ns']:r for r in raw['joints']})}
bagcheck={};bag=rosbag.Bag(str(RUN/'fixture.bag'),'r')
for topic,(kind,rows) in bytopic.items():
 seen=set();matches=0;missing=[];mismatches=[]
 for _,msg,receiver_time in bag.read_messages(topics=[topic]):
  ns=msg.header.stamp.to_nsec();seen.add(ns)
  if ns not in rows:missing.append(ns);continue
  a=rows[ns]
  if kind=='links':
   poses=[[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w] for p in msg.pose]
   twists=[[p.linear.x,p.linear.y,p.linear.z,p.angular.x,p.angular.y,p.angular.z] for p in msg.twist]
   equal=list(msg.name)==a['names'] and poses==a['poses'] and twists==a['twists']
  else:equal=dict(zip(msg.name,msg.position))==a['positions']
  if equal:matches+=1
  else:mismatches.append(ns)
 bagcheck[kind]={'bag_unique_source_samples':len(seen),'json_samples':len(rows),'exact_matches':matches,'json_not_in_bag':sorted(set(rows)-seen),'bag_not_in_json':missing,'mismatches':mismatches}
bag.close()
with (OUT/'source_time_steering.csv').open() as f:rows=list(csv.DictReader(f))
with (OUT/'source_time_rates.csv').open() as f:rates=list(csv.DictReader(f))
a={k:np.array([float(r[k]) for r in rows]) for k in rows[0] if k!='violation_endpoint'};b={k:np.array([float(r[k]) for r in rates]) for k in rates[0]}
font=Path('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf');font_manager.fontManager.addfont(str(font));fontname=font_manager.FontProperties(fname=str(font)).get_name()
plt.rcParams.update({'font.family':fontname,'font.size':8,'axes.titlesize':8.6,'axes.labelsize':8,'xtick.labelsize':7.5,'ytick.labelsize':7.5,'legend.fontsize':7.1,'axes.spines.top':False,'axes.spines.right':False,'axes.unicode_minus':False,'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none','lines.linewidth':1.0,'axes.linewidth':.6,'xtick.major.width':.6,'ytick.major.width':.6})
blue='#17699E';red='#B64032';orange='#B47B18';gray='#5A6167';green='#267B61'
fig,axs=plt.subplots(2,2,figsize=(170/25.4,145/25.4));fig.subplots_adjust(left=.105,right=.982,bottom=.15,top=.845,hspace=.58,wspace=.30)
peak=num['peak'];p0=peak['from']['source_s'];p1=peak['to']['source_s'];lo=p0-.14;hi=p1+.14
fig.suptitle('执行资格负面诊断：发布转角与实际关节姿态不一致',y=.986,fontsize=10)
fig.text(.5,.943,'Q02 / seed {} / 单次 36 s 仿真资格测试 / {} 项检查中 {} 项未通过'.format(raw['seed'],len(report['checks']),sum(not x for x in report['checks'].values())),ha='center',fontsize=7.6,color=gray)
fig.text(.5,.910,'仅离线重算既有 raw；保留原始失败与冻结门限，不构成导航性能或实车证据',ha='center',fontsize=7.1,color=gray)
ax=axs[0,0];ax.plot(a['source_s'],a['actual_center_rad'],c=blue,label='实际中心角（相对四元数）');ax.plot(a['joint_source_ns']*1e-9,a['published_center_rad'],c=red,ls='--',lw=.9,label='发布中心角（内部状态）');ax.axvspan(lo,hi,color=orange,alpha=.2);ax.set(title='(a) 完整执行回合',xlabel='仿真源时间 / s',ylabel='中心转角 / rad');ax.legend(loc='lower left',frameon=False);ax.grid(alpha=.2,lw=.4)
ax=axs[0,1];mask=(a['source_s']>=lo)&(a['source_s']<=hi)
ax.step(a['joint_source_ns'][mask]*1e-9,a['published_center_rad'][mask],where='post',c=red,label='内部状态');ax.plot(a['source_s'][mask],a['actual_center_rad'][mask],c=blue,marker='o',ms=3,label='转向架姿态');ax.plot(a['source_s'][mask],a['wheel_axis_center_rad'][mask],c=green,marker='x',ms=3,ls=':',label='前轮轴向复核');ax.axvspan(p0,p1,color=orange,alpha=.16);ax.set(title='(b) 最高斜率附近：源时刻对齐',xlabel='仿真源时间 / s',ylabel='中心转角 / rad');ax.set_xlim(lo,hi);ax.legend(frameon=False,loc='upper right');ax.grid(alpha=.2,lw=.4)
ax=axs[1,0];maskb=(b['source_midpoint_s']>=lo)&(b['source_midpoint_s']<=hi)
ax.plot(b['source_midpoint_s'][maskb],b['actual_center_rate_radps'][maskb],c=blue,marker='o',ms=3,label='实际姿态，20 ms 差分');ax.plot(b['source_midpoint_s'][maskb],b['published_center_rate_radps'][maskb],c=red,ls='--',label='内部状态，20 ms 差分');ax.axhline(-num['gate_radps'],c=gray,ls=':',lw=.9,label='冻结下界 −{:.2f} rad/s'.format(num['gate_radps']));ax.axvspan(p0,p1,color=orange,alpha=.16);ax.set(title='(c) 同一源时间间隔的变化率',xlabel='差分区间中点 / s',ylabel='转角变化率 / (rad/s)');ax.set_xlim(lo,hi);ax.legend(frameon=False,loc='lower left');ax.grid(alpha=.2,lw=.4)
ax=axs[1,1]
for side,col,label in [('left',blue,'左转向架'),('right',green,'右转向架')]:ax.plot(a['source_s'][mask],(a[side+'_heading_rad'][mask]-a[side+'_published_rad'][mask])*1000,c=col,marker='o',ms=3,label=label)
ax.axhline(0,c=gray,lw=.7);ax.axvspan(p0,p1,color=orange,alpha=.16);ax.set(title='(d) 实际角 − 最近发布角',xlabel='仿真源时间 / s',ylabel='角度残差 / mrad');ax.set_xlim(lo,hi);ax.legend(frameon=False,loc='upper right');ax.grid(alpha=.2,lw=.4)
fig.text(.105,.084,'峰值 |Δδ/Δt| = {:.6f} rad/s；冻结门 = {:.2f} rad/s；全部 {:d}/{:d} 个区间超门。'.format(num['max_actual_rate_radps'],num['gate_radps'],num['violating_intervals'],num['source_samples']-1),fontsize=7.2,color=red)
fig.text(.105,.050,'实际姿态源间隔恒为 20 ms；对应内部状态恒早 2 ms。阴影为最高斜率区间。',fontsize=7.1,color=gray)
fig.text(.105,.018,'采样仅覆盖固定物理相位；缺少逐 2 ms 姿态，不能推定间隔内的完整运动或因果。',fontsize=7.1,color=gray)
revision=OUT/'r02';revision.mkdir(exist_ok=True);ident='AQC01'
for ext in ('pdf','svg','png'):fig.savefig(revision/(ident+'.'+ext),dpi=600,metadata={'Creator':'Q02 offline source-stamp diagnosis'} if ext=='pdf' else None)
plt.close(fig)
subprocess.run(['pdftoppm','-scale-to-x','1004','-scale-to-y','-1','-png','-singlefile',str(revision/(ident+'.pdf')),str(PANELS/'AQC01_r02_170mm')],check=True)
fonts=subprocess.check_output(['pdffonts',str(revision/(ident+'.pdf'))],text=True);pdfinfo=subprocess.check_output(['pdfinfo',str(revision/(ident+'.pdf'))],text=True)
caption=('Q02 单回合执行资格测试的转向变化率失败。蓝线由 Gazebo WorldUpdateEnd 的 base_link 与前转向架相对四元数恢复左右转角，再按有效轴距和轮距反推中心角；红线为实际发布的 joint_states 内部目标状态。前轮滚转轴方向独立复核实际中心角。全部 '+str(num['source_samples'])+' 个姿态样本源间隔均为 20 ms，匹配的内部状态恒早 2 ms。最高区间 '+format(p0,'.3f')+'–'+format(p1,'.3f')+' s 内，实际变化率 '+format(peak['rates_radps']['actual_center_rad'],'.9f')+' rad/s，内部状态为 '+format(peak['rates_radps']['published_center_rad'],'.3f')+' rad/s；残差差分解释二者之差。冻结门限为 ±'+format(num['gate_radps'],'.2f')+' rad/s，'+str(num['violating_intervals'])+'/'+str(num['source_samples']-1)+' 个区间超门，原测试不通过。四元数 Z 轴 twist 与 Euler heading 的变化率差不足 '+format(num['max_actual_twist_rate_difference_radps'],'.3g')+' rad/s，前轮轴向也复现同一峰值，故无法以 Euler 投影误差解释失败。旧驱动每 20 ms 设定一次关节位置，ODE 仍可在物理更新中改变关节姿态；此解释符合原始残差和相对角速度，但缺少逐 2 ms 关节姿态及接触力，不能重建完整中间波形或分离每项动力学因果。此图仅为既有 Q02 原始数据的离线诊断，不改变门限、回执或历史结论。')
(revision/'caption_zh.md').write_text(caption+'\n');(revision/'latex_insert.tex').write_text('\\begin{figure}[htbp]\n\\centering\n\\includegraphics[width=170mm]{AQC01.pdf}\n\\caption{'+caption+'}\n\\label{fig:aqc01-actuation-diagnostic}\n\\end{figure}\n')
qc={'schema_version':1,'figure_id':ident,'source_numeric_sha256':sha(OUT/'numeric_diagnosis.json'),'figure_width_mm':170,'figure_height_mm':145,'native_dpi':600,'native_png_pixels':list(Image.open(revision/(ident+'.png')).size),'pdf_fonts':fonts,'pdf_info':pdfinfo,'bag_vs_json':bagcheck,'visual_inspection':'pending_actual_view_image','script_sha256':sha(__file__),'sources':num['sources'],'outputs':[{'path':str(p),'sha256':sha(p)} for p in [revision/(ident+'.pdf'),revision/(ident+'.svg'),revision/(ident+'.png'),revision/'caption_zh.md',PANELS/'AQC01_r02_170mm.png']]}
(OUT/'figure_qc.json').write_text(json.dumps(qc,indent=2,ensure_ascii=False)+'\n');print(json.dumps({'bagcheck':bagcheck,'pdf_info':pdfinfo,'fonts':fonts,'pixels':qc['native_png_pixels']},indent=2,ensure_ascii=False))
