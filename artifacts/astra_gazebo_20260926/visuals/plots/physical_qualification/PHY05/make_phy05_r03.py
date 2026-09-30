#!/usr/bin/env python3
"""Preregistered PHY05 actual P07 six-unit qualification figure; no simulation."""
import csv,hashlib,json,math,subprocess
from pathlib import Path
import numpy as np
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from PIL import Image
O=Path(__file__).resolve().parent;S=O.parents[3];D=S/'audits/m12_p07_result_independent';R=O/'r03';R.mkdir(parents=True,exist_ok=True);PAN=S/'visuals/panels/physical_qualification/PHY05';PAN.mkdir(parents=True,exist_ok=True)
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def readcsv(p):return list(csv.DictReader(p.open()))
def writecsv(p,rows):
 with p.open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
prereg=D/'figure_selection_preregistration.json';assert sha(prereg)=='a2a55caa9f9a22caea520ca7eaa27d8105212594255b97d41afaddb07adc133d'
progress=json.loads((D/'closed_row_progress.json').read_bytes());budget=json.loads((D/'budget_binding_audit.json').read_bytes());schedule=json.loads((S/'queue/m12_physics_p07_schedule.json').read_bytes());assert len(progress)==6 and all(progress[r['run_id']]['status']=='independent_closed_row_audit_pass' for r in schedule);assert budget['pass'] and budget['budget']['status']=='terminal_complete' and budget['budget']['consumed_units']==6 and budget['budget']['retired_units']==0
inputs=[prereg,D/'closed_row_progress.json',D/'budget_binding_audit.json',S/'queue/m12_physics_p07_schedule.json'];rows=[];phaserows=[];steeringrows=[];summaryrows=[]
for unit in schedule:
 rid=unit['run_id'];A=D/'rows'/rid;run=S/'runs'/rid;c=yaml.safe_load((run/'actuation_contract.yaml').read_bytes());a=progress[rid];b=json.loads((A/'independent_raw_diagnosis.json').read_bytes());ss=[r for r in readcsv(A/'source_steering.csv') if r['grid']=='links'];t=np.array([float(r['elapsed_s']) for r in ss]);ns=np.array([int(r['source_ns']) for r in ss],dtype=np.int64);center=np.array([float(r['center_rad']) for r in ss]);times=ns/1e9;rate=np.diff(center)/np.diff(times);mid=(ns[1:]+ns[:-1])/2;start=b['schedule_start_ns_rounded'];peak=float(abs(rate).max());assert abs(peak-a['independent_20ms_peak_radps'])<1e-12
 traj=readcsv(A/'models_model_trajectory.csv');target=readcsv(A/'source_target_actual.csv');data=dict(unit,contract=c,steer_ns=ns,t=t,center=center,rate=rate,mid_t=(mid-start)/1e9,peak=peak,traj=traj,target=target,start_ns=start,body=b);rows.append(data)
 at=0
 for phase in c['phase_schedule']:
  lo=start+round(at*1e9);at+=phase['duration_s'];hi=start+round(at*1e9);tail=hi-round(c['settled_tail_s']*1e9);mask=(ns>=tail)&(ns<hi);desired=float(phase.get('center_steering_request_rad',0.));desired=max(-c['effective_plugin_parameters']['maxSteer'],min(desired,c['effective_plugin_parameters']['maxSteer']));error=float(abs(center[mask]-desired).max());phaserows.append({'run_id':rid,'seed':unit['seed'],'arm':unit['arm'],'phase':phase['name'],'tail_start_ns':tail,'tail_end_ns_exclusive':hi,'native_sample_count':int(mask.sum()),'requested_clipped_center_rad':desired,'maximum_tail_absolute_error_rad':error,'original_moving_steering_gate_applies':bool(phase.get('speed_mps',0.)),'original_gate_rad':c['gates']['steady_actual_steering_error_rad']})
 for i,n in enumerate(ns):steeringrows.append({'run_id':rid,'seed':unit['seed'],'arm':unit['arm'],'source_ns':int(n),'elapsed_s':t[i],'center_rad':center[i],'ending_interval_rate_radps':rate[i-1] if i else ''})
 summaryrows.append({'run_id':rid,'seed':unit['seed'],'arm':unit['arm'],'Kd':unit['steering_kd'],'actual_20ms_peak_radps':peak,'threshold_radps':c['effective_plugin_parameters']['maxSteeringRate']+c['gates']['actual_center_slew_tolerance_radps'],'original_gate_passes':a['original_pass_count'],'original_gate_count':a['original_check_count'],'original_outcome':a['outcome'],'original_failed_checks':'|'.join(a['original_failed_checks'])})
 inputs.extend([A/'source_steering.csv',A/'source_target_actual.csv',A/'models_model_trajectory.csv',A/'independent_raw_diagnosis.json',A/'protocol_audit.json',A/'watermark_audit.json',run/'actuation_contract.yaml',run/'probe_report.json',run/'receipt.json',run/'probe_raw.json'])
controls=sorted([r for r in rows if r['arm']=='control'],key=lambda r:(r['peak'],r['seed']));selected=controls[1]['seed'];pair={r['arm']:r for r in rows if r['seed']==selected};worst=int(np.argmax(abs(pair['control']['rate'])));peakmid=float(pair['control']['mid_t'][worst]);selection={'rule':'middle control of three sorted by actual native20ms peak, ties integer seed','sorted_controls':[{'seed':r['seed'],'peak_radps':r['peak']} for r in controls],'selected_seed':selected,'control_peak_midpoint_elapsed_s':peakmid,'treatment_not_used_for_selection':True,'all_six_original_points_preserved':True};(R/'PHY05_selection.json').write_text(json.dumps(selection,indent=2)+'\n');writecsv(R/'PHY05_peaks.csv',summaryrows);writecsv(R/'PHY05_all_phase_tails.csv',phaserows);writecsv(R/'PHY05_native_steering_all6.csv',steeringrows)
font=Path('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf');font_manager.fontManager.addfont(str(font));fn=font_manager.FontProperties(fname=str(font)).get_name();plt.rcParams.update({'font.family':['DejaVu Sans',fn],'font.size':8,'axes.titlesize':8.3,'axes.labelsize':7.6,'xtick.labelsize':7,'ytick.labelsize':7,'legend.fontsize':7,'axes.spines.top':False,'axes.spines.right':False,'axes.unicode_minus':False,'pdf.fonttype':42,'svg.fonttype':'none','lines.linewidth':.8,'axes.linewidth':.6,'xtick.major.width':.6,'ytick.major.width':.6})
colors={'control':'#17699E','treatment':'#B46917'};gray='#667078';red='#B64032';markers={260926023:'o',260926024:'s',260926025:'^'};seeds=sorted(markers);gate=rows[0]['contract']['effective_plugin_parameters']['maxSteeringRate']+rows[0]['contract']['gates']['actual_center_slew_tolerance_radps'];steady=rows[0]['contract']['gates']['steady_actual_steering_error_rad']
fig=plt.figure(figsize=(170/25.4,220/25.4));gs=fig.add_gridspec(4,2,left=.12,right=.98,top=.85,bottom=.105,hspace=.79,wspace=.33,height_ratios=[1.,.85,1.,1.]);a=fig.add_subplot(gs[0,0]);b=fig.add_subplot(gs[0,1]);cax=fig.add_subplot(gs[1,:]);d=fig.add_subplot(gs[2,:]);e=fig.add_subplot(gs[3,:]);axes=[a,b,cax,d,e]
fig.suptitle('PHY05  有限扭矩转向的三配对执行资格',y=.985,fontsize=10)
fig.text(.5,.962,'唯一控制因素：steeringKd  4 → 6；原生 WorldUpdateEnd 源时间',ha='center',fontsize=7.5,color=gray)
handles=[Line2D([0],[0],color=colors[k],lw=1.3,label=('Kd4 对照' if k=='control' else 'Kd6 处理')) for k in colors]+[Line2D([0],[0],color=gray,marker=markers[s],ls='none',ms=3.2,label='seed '+str(s)[-3:]) for s in seeds]
fig.legend(handles=handles,loc='upper center',bbox_to_anchor=(.52,.941),ncol=5,frameon=False,handlelength=1.4,columnspacing=1.)
fig.text(.5,.891,'代表 pair：seed {}（三个对照峰值的中位规则）'.format(selected),ha='center',fontsize=7.5,color=gray)
for i,seed in enumerate(seeds):
 rr={r['arm']:r for r in rows if r['seed']==seed};a.plot([i-.11,i+.11],[rr['control']['peak'],rr['treatment']['peak']],color=gray,lw=.6,zorder=1)
 for arm,dx in [('control',-.11),('treatment',.11)]:a.scatter(i+dx,rr[arm]['peak'],marker=markers[seed],s=25,color=colors[arm],edgecolors='white',linewidths=.35,zorder=3)
a.axhline(gate,color=red,ls='--',lw=.8);a.text(.02,.94,'冻结门 {:.2f} rad/s'.format(gate),transform=a.transAxes,fontsize=7,color=red,va='top');a.set(title='a  三配对：实际 20 ms 峰值',ylabel='中心角变化率 / rad/s',xlabel='seed 尾号',xticks=range(3),xticklabels=[str(s)[-3:] for s in seeds],xlim=(-.45,2.45),ylim=(min(.79,min(r['peak'] for r in rows)-.015),max(.92,max(r['peak'] for r in rows)+.02)))
for arm,r in pair.items():
 tx=np.array([float(z['x_m']) for z in r['traj']]);ty=np.array([float(z['y_m']) for z in r['traj']]);b.plot(tx,ty,color=colors[arm],lw=.85);b.scatter([tx[0],tx[-1]],[ty[0],ty[-1]],color=colors[arm],marker=markers[selected],s=15,zorder=3)
b.set(title='b  代表 pair：实际轨迹',xlabel='世界 x / m',ylabel='世界 y / m');b.set_aspect('equal',adjustable='box')
labels={'accelerate':'加速','turn_positive':'正转','turn_negative':'负转','saturation_positive':'正饱和','saturation_negative':'负饱和','reaccelerate_brake':'制动前\n再加速','reaccelerate_loss':'失联前\n再加速'};names=list(labels)
for i,name in enumerate(names):
 for r in rows:
  z=next(x for x in phaserows if x['run_id']==r['run_id'] and x['phase']==name);dx=(-.14 if r['arm']=='control' else .14)+(seeds.index(r['seed'])-1)*.07;cax.scatter(i+dx,z['maximum_tail_absolute_error_rad'],c=colors[r['arm']],marker=markers[r['seed']],s=16,edgecolors='white',linewidths=.25,zorder=3)
cax.axhline(steady,color=red,ls='--',lw=.8);cax.text(.98,.85,'原门 {:.2f} rad'.format(steady),ha='right',transform=cax.transAxes,color=red,fontsize=7);cax.set(title='c  七个移动相位末 0.75 s：六回合全部误差点',ylabel='最大角误差 / rad',xticks=range(len(names)),xticklabels=[labels[n] for n in names],xlim=(-.55,6.55),ylim=(-.001,max(.036,max(x['maximum_tail_absolute_error_rad'] for x in phaserows if x['original_moving_steering_gate_applies'])*1.2)))
for arm,r in pair.items():
 d.plot(r['t'],r['center'],color=colors[arm],lw=.72,marker='.',markersize=.85,label='实际 '+arm)
 tt=np.array([float(x['elapsed_s']) for x in r['target']]);target=np.array([float(x['target_center_rad']) for x in r['target']]);d.plot(tt,target,color=gray,lw=.65,ls='--' if arm=='control' else ':',alpha=.7)
 e.plot(r['mid_t'],r['rate'],color=colors[arm],lw=.72,marker='.',markersize=.8)
d.set(title='d  代表 pair：实际中心角与自身源时刻参考（灰虚线）',ylabel='中心转向角 / rad',xlabel='正式开始后源时间 / s',xlim=(0,36),xticks=np.arange(0,37,6),ylim=(-.56,.56))
e.axhline(gate,color=red,ls='--',lw=.8);e.axhline(-gate,color=red,ls='--',lw=.8);e.set(title='e  代表 pair：相邻 20 ms 实际差分，未平滑',ylabel='中心角变化率 / rad/s',xlabel='实际 world-link 区间中点 / s',xlim=(0,36),xticks=np.arange(0,37,6),ylim=(-max(1.,max(abs(r['rate']).max() for r in pair.values())*1.1),max(1.,max(abs(r['rate']).max() for r in pair.values())*1.1)))
for ax in (d,e):
 ax.axvspan(32,36,color=gray,alpha=.075);ax.text(.975,.70 if ax is e else .92,'命令失联段',ha='right',va='top',transform=ax.transAxes,fontsize=6.8,color=gray)
 for x in (2,6,9,12,15,18,21,25,28,32):ax.axvline(x,color=gray,alpha=.16,lw=.4,zorder=0)
for ax in axes:ax.grid(axis='y',alpha=.18,lw=.4)
negative=[r for r in summaryrows if r['original_outcome']!='qualified'];fig.text(.12,.034,'六回合测量完整；物理负结果 {} 个全部保留。三配对仅用于本执行资格。'.format(len(negative)),fontsize=7,color=gray);fig.text(.12,.013,'固定几何与无抖动设置；三个 seed 不代表独立几何或一般性能。',fontsize=7,color=gray)
for ext in ('pdf','svg','png'):fig.savefig(R/('PHY05.'+ext),dpi=600)
plt.close(fig);subprocess.run(['pdftoppm','-r','150','-png','-singlefile',str(R/'PHY05.pdf'),str(PAN/'PHY05_r03_170mm')],check=True)
pairs=[]
for seed in seeds:
 rr={r['arm']:r for r in rows if r['seed']==seed};pairs.append('{}：{:.6f}/{:.6f}'.format(seed,rr['control']['peak'],rr['treatment']['peak']))
caption='P07三配对有限扭矩执行资格，唯一控制因素为steeringKd4→6，六回合均保留。a展示三个fresh seed的原生20ms实际world-link中心转向斜率绝对峰值；对照/处理依次为'+ '，'.join(pairs)+' rad/s，红虚线为冻结{:.2f}rad/s门。b展示预注册规则选出的seed{}实际世界轨迹：按三个control峰值升序（完全同值按seed）取中位，处理结果不参与选择。c为七个具有原稳定转向门的移动相位，每相位展示六回合全部点；误差来自固定末0.75s整数半开窗口内原生实际角的最大绝对偏差，红线为{:.2f}rad原门，全部11相位数字另附CSV。d/e展示该代表pair完整36s、未平滑未重采样的实际中心角及20ms差分，参考角保留各自50Hz源时刻；曲线不移到joint/target相位。中心角由base_link相对左右steer_link四元数，经独立左右Ackermann曲率平均反演。六回合均通过source、bag、暂停watermark和ingress测量审计；{}个原物理负结果完整保留，原失败检查为{}。该图不将3个仿真seed配对外推为一般导航性能、物理安全或真实车结果；500Hz诊断不替代冻结20ms资格门。'.format(gate,selected,steady,len(negative),'; '.join(r['run_id']+': '+r['original_failed_checks'] for r in negative) or '无')
(R/'PHY05_caption_zh.md').write_text(caption+'\n');(R/'PHY05_latex_insert.tex').write_text('\\begin{figure}[htbp]\n\\centering\n\\includegraphics[width=170mm]{PHY05.pdf}\n\\caption{'+caption.replace('_',r'\_')+'}\n\\label{fig:phy05-actuation-qualification}\n\\end{figure}\n')
dense_note=' 补充采样尺度诊断：三个Kd6回合的原生2ms峰值依次为'+', '.join('{:.6f}'.format(r['body']['steering']['dense_links']['center_rate_radps']['max_abs']) for r in sorted([x for x in rows if x['arm']=='treatment'],key=lambda x:x['seed']))+' rad/s；seed023的2ms峰仍超过0.85，故20ms资格通过不代表每physicsstep都满足同一界。固定几何和无噪声/无抖动条件下，seed024/025两臂20ms峰完全相同，三对不是独立几何或泛化统计。';caption+=dense_note
(R/'PHY05_caption_zh.md').write_text(caption+'\n');(R/'PHY05_latex_insert.tex').write_text('\\begin{figure}[htbp]\n\\centering\n\\includegraphics[width=170mm]{PHY05.pdf}\n\\caption{'+caption.replace('_',r'\_')+'}\n\\label{fig:phy05-actuation-qualification}\n\\end{figure}\n')
inputs.append(font);meta={'figure_id':'PHY05','revision':'r03','size_mm':[170,220],'native_dpi':600,'native_pixels':list(Image.open(R/'PHY05.png').size),'selection':selection,'physical_negative_rows':negative,'preregistration_sha256':sha(prereg),'inputs':[{'path':str(p),'sha256':sha(p)} for p in inputs],'outputs':[{'path':str(p),'sha256':sha(p)} for p in sorted(R.glob('PHY05*'))]+[{'path':str(PAN/'PHY05_r03_170mm.png'),'sha256':sha(PAN/'PHY05_r03_170mm.png')}],'script_sha256':sha(__file__),'command':'OPENBLAS_NUM_THREADS=1 '+str(S/'workspace/.venv/bin/python')+' '+str(Path(__file__).resolve()),'pdf_info':subprocess.check_output(['pdfinfo',str(R/'PHY05.pdf')],text=True),'pdf_fonts':subprocess.check_output(['pdffonts',str(R/'PHY05.pdf')],text=True),'actual_view_image_status':'pending','global_manifest_untouched':True};(O/'PHY05_visual_qc_pending.json').write_text(json.dumps(meta,indent=2,ensure_ascii=False)+'\n');print(json.dumps({'selected_seed':selected,'negative_count':len(negative),'outputs':meta['outputs'],'native_pixels':meta['native_pixels']},indent=2))
