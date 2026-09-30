#!/usr/bin/env python3
"""Offline negative-candidate comparison, preserving Q02 and Q03 evidence."""
from pathlib import Path
import json,csv,hashlib,subprocess,math
import numpy as np,rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from PIL import Image
O=Path(__file__).resolve().parent;S=O.parents[2];Q2=O.parent/'actuation_q02_diagnosis';P=S/'visuals/panels/actuation_q03_comparison';P.mkdir(exist_ok=True,parents=True);R=O/'r02';R.mkdir(exist_ok=True);sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
D=[]
for name,directory in [('actuation_q02',Q2),('actuation_q03',O)]:
 raw=json.loads((S/'runs'/name/'probe_raw.json').read_text());nr=json.loads((directory/'numeric_diagnosis.json').read_text());re=json.loads((S/'runs'/name/'probe_report.json').read_text())
 with (directory/'source_time_steering.csv').open() as f:rs=list(csv.DictReader(f))
 with (directory/'source_time_rates.csv').open() as f:rr=list(csv.DictReader(f))
 a={k:np.array([float(r[k]) for r in rs]) for k in rs[0] if k!='violation_endpoint'};b={k:np.array([float(r[k]) for r in rr]) for k in rr[0]}
 D.append(dict(raw=raw,n=nr,report=re,a=a,b=b,name=name))
raw=D[1]['raw'];bagcheck={}
with rosbag.Bag(str(S/'runs/actuation_q03/fixture.bag'),'r') as bag:
 for topic,kind in [('/astra/world_links','links'),('/joint_states','joints')]:
  rows={r['ns']:r for r in raw[kind]};seen=set();matches=0;bad=[]
  for _,m,_ in bag.read_messages(topics=[topic]):
   ns=m.header.stamp.to_nsec();seen.add(ns)
   if ns not in rows:bad.append({'ns':ns,'reason':'no JSON'});continue
   x=rows[ns]
   if kind=='links':eq=(list(m.name)==x['names'] and [[p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w] for p in m.pose]==x['poses'] and [[v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z] for v in m.twist]==x['twists'])
   else:eq=dict(zip(m.name,m.position))==x['positions']
   if eq:matches+=1
   else:bad.append({'ns':ns,'reason':'mismatch'})
  bagcheck[kind]={'bag_unique_source_samples':len(seen),'json_samples':len(rows),'exact_matches':matches,'missing_json_source_ns_from_bag':sorted(set(rows)-seen),'mismatches':bad}
font=Path('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf');font_manager.fontManager.addfont(str(font));fn=font_manager.FontProperties(fname=str(font)).get_name();plt.rcParams.update({'font.family':['DejaVu Sans',fn],'font.size':8,'axes.titlesize':8.6,'axes.labelsize':8,'xtick.labelsize':7.5,'ytick.labelsize':7.5,'legend.fontsize':7.1,'axes.spines.top':False,'axes.spines.right':False,'axes.unicode_minus':False,'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none','lines.linewidth':1.,'axes.linewidth':.6,'xtick.major.width':.6,'ytick.major.width':.6})
blue='#17699E';red='#B64032';gray='#5A6167';green='#267B61';orange='#B47B18'
fig,axs=plt.subplots(3,2,figsize=(170/25.4,180/25.4));fig.subplots_adjust(left=.105,right=.98,bottom=.195,top=.842,hspace=.64,wspace=.30)
fig.suptitle('同一冻结转向门：原驱动与保持候选均未通过',y=.987,fontsize=10)
fig.text(.5,.958,'Q02 与 Q03 各 1 个独立新身份回合；只作失败机制诊断，无统计性能结论',ha='center',fontsize=7.3,color=gray)
fig.legend(handles=[Line2D([0],[0],color=blue,label='实际中心角'),Line2D([0],[0],color=red,ls='--',label='内部状态'),Line2D([0],[0],color=green,ls=':',label='前轮轴向复核'),Line2D([0],[0],color=gray,ls=':',label='冻结界限')],loc='upper center',bbox_to_anchor=(.54,.93),ncol=4,frameon=False,handlelength=1.8,columnspacing=1.)
for col,d in enumerate(D):
 a,b,n=d['a'],d['b'],d['n'];p=n['peak'];lo=p['from']['source_s']-.14;hi=p['to']['source_s']+.14;mm=(a['source_s']>=lo)&(a['source_s']<=hi);mr=(b['source_midpoint_s']>=lo)&(b['source_midpoint_s']<=hi)
 ax=axs[0,col];ax.plot(a['source_s'],a['actual_center_rad'],c=blue);ax.plot(a['joint_source_ns']*1e-9,a['published_center_rad'],c=red,ls='--');ax.axvspan(lo,hi,color=orange,alpha=.2);ax.set(title=('Q02 原驱动' if col==0 else 'Q03 位置＋零角速度保持')+'\nseed '+str(d['raw']['seed']),xlabel='仿真源时间 / s',ylabel='中心转角 / rad');ax.set_ylim(-.56,.56)
 ax=axs[1,col];ax.plot(b['source_midpoint_s'][mr],b['actual_center_rate_radps'][mr],c=blue,marker='o',ms=3);ax.plot(b['source_midpoint_s'][mr],b['wheel_axis_center_rate_radps'][mr],c=green,ls=':',marker='x',ms=3);ax.plot(b['source_midpoint_s'][mr],b['published_center_rate_radps'][mr],c=red,ls='--');ax.axhline(-n['gate_radps'],c=gray,ls=':');ax.set(title='最坏区间 |变化率| = {:.6f}'.format(n['max_actual_rate_radps']),xlabel='差分区间中点 / s',ylabel='变化率 / (rad/s)');ax.set_xlim(lo,hi);ax.set_ylim(-1.075,-.35)
 ax=axs[2,col];ax.plot(a['source_s'][mm],1000*a['actual_minus_published_rad'][mm],c=blue,marker='o',ms=3);ax.axhline(0,c=gray,lw=.7);ax.set(title='原门超限：{}/{} 个 20 ms 区间'.format(n['violating_intervals'],n['source_samples']-1),xlabel='仿真源时间 / s',ylabel='实际 − 内部角 / mrad');ax.set_xlim(lo,hi);ax.set_ylim(-7,7)
 for row in range(3):
  ax=axs[row,col];ax.grid(alpha=.2,lw=.4)
  if row:ax.axvspan(p['from']['source_s'],p['to']['source_s'],color=orange,alpha=.16)
fig.text(.105,.100,'两回合均 {}/{} 门通过；唯一失败仍为实际中心转角变化率超过 ±{:.2f} rad/s。'.format(sum(D[0]['report']['checks'].values()),len(D[0]['report']['checks']),D[0]['n']['gate_radps']),fontsize=7.2,color=red)
fig.text(.105,.069,'全程最大角残差：Q02 {:.3f} mrad，Q03 {:.3f} mrad；残差幅度下降未使变化率门通过。'.format(D[0]['n']['max_actual_minus_published_rad']*1000,D[1]['n']['max_actual_minus_published_rad']*1000),fontsize=7.1,color=gray)
fig.text(.105,.039,'姿态源间隔恒 20 ms，内部状态恒早 2 ms。两列按各自最坏区间展开，横轴窗口不同。',fontsize=7.1,color=gray)
fig.text(.105,.013,'Q03 候选未采纳；不改门、不删失败、不以平滑差分替代冻结指标。逐 2 ms 完整姿态仍缺失。',fontsize=7.1,color=gray)
for ext in ('pdf','svg','png'):fig.savefig(R/('AQC02.'+ext),dpi=600)
plt.close(fig);subprocess.run(['pdftoppm','-scale-to-x','1004','-scale-to-y','-1','-png','-singlefile',str(R/'AQC02.pdf'),str(P/'AQC02_r02_170mm')],check=True)
caption='Q02 原驱动与独立 Q03 转向关节保持候选的离线对照。Q03 每个物理步投影两个转向关节位置并归零角速度，保持50Hz内部目标、原动作序列与全部资格门限不变。两次fresh身份各完成一个36s回合，均79/80门通过且唯一实际中心转角变化率门失败。第一行展示完整实际与内部角轨迹；第二、三行分别展示各自最坏源时间区间附近的20ms变化率与实际−发布角残差，窗口按各回合最坏区间居中，横轴不能视作同一绝对区间。前轮轴向独立复核剔除滚转后同样复现超门。Q02/Q03峰值分别为'+format(D[0]['n']['max_actual_rate_radps'],'.9f')+'/'+format(D[1]['n']['max_actual_rate_radps'],'.9f')+'rad/s；冻结门限±0.85rad/s，超门18/1800与22/1800。Q03全程最大角残差虽从9.232降到5.420mrad，其局部残差变化仍造成更高记录峰值，说明角误差缩小并不等于变化率资格通过。这里仅报告两个单回合，不将不同新seed的差异推断为统计因果或性能改善。原负面证据完整保留，Q03未被采纳；缺少500Hz姿态与接触力，不能断言完整子步运动或力学归因。'
caption=caption.replace('均79/80门通过','均{}/{}门通过'.format(sum(D[0]['report']['checks'].values()),len(D[0]['report']['checks']))).replace('超门18/1800与22/1800','超门{}/{}与{}/{}'.format(D[0]['n']['violating_intervals'],D[0]['n']['source_samples']-1,D[1]['n']['violating_intervals'],D[1]['n']['source_samples']-1)).replace('从9.232降到5.420mrad','从{:.3f}降到{:.3f}mrad'.format(D[0]['n']['max_actual_minus_published_rad']*1000,D[1]['n']['max_actual_minus_published_rad']*1000))
(R/'caption_zh.md').write_text(caption+'\n');(R/'latex_insert.tex').write_text('\\begin{figure}[htbp]\n\\centering\n\\includegraphics[width=170mm]{AQC02.pdf}\n\\caption{'+caption+'}\n\\label{fig:aqc02-rejected-hold}\n\\end{figure}\n')
report={'status':'both_negative_results_independently_preserved','figure_id':'AQC02','q02_numeric':D[0]['n'],'q03_numeric':D[1]['n'],'q03_bag_json_comparison':bagcheck,'mechanism_conclusion':'Steering position/zero-velocity projection did not satisfy the frozen actual-center slew qualification. Smaller maximum angle residual did not prevent larger local residual derivatives. This rejects adoption of this candidate, not proof that all ODE residuals were uniquely identified.','candidate_adopted':False,'claims_not_supported':['continuous steering limit qualification','force-driven wheel-actuation qualification','performance improvement','causal magnitude from two different fresh single-seed episodes'],'figure_qc':{'revision':'r02','view_image_status':'pending','size_mm':[170,180],'native_dpi':600,'native_pixels':list(Image.open(R/'AQC02.png').size),'font_report':subprocess.check_output(['pdffonts',str(R/'AQC02.pdf')],text=True),'pdf_info':subprocess.check_output(['pdfinfo',str(R/'AQC02.pdf')],text=True)},'sources':[r for d in D for r in d['n']['sources']],'scripts':[{'path':str(p),'sha256':sha(p)} for p in [O/'diagnose_slew.py',Path(__file__)]],'outputs':[{'path':str(p),'sha256':sha(p)} for p in [R/'AQC02.pdf',R/'AQC02.svg',R/'AQC02.png',R/'caption_zh.md',P/'AQC02_r02_170mm.png']],'no_runtime_inputs_modified':True,'no_ROS_started':True,'generation_commands':['workspace/.venv/bin/python visuals/plots/actuation_q03_comparison/diagnose_slew.py','workspace/.venv/bin/python visuals/plots/actuation_q03_comparison/compare_and_plot.py'],'working_directory':str(S)}
(O/'comparison_report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(json.dumps({'bag':bagcheck,'figure_qc':report['figure_qc'],'figure_path':str(R/'AQC02.pdf')},indent=2,ensure_ascii=False))
