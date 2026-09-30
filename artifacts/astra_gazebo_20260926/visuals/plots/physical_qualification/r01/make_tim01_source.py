#!/usr/bin/env python3
"""P01/P02 offline measurement diagnosis, not causal performance evidence."""
from pathlib import Path
import csv,json,hashlib,subprocess
import numpy as np
import rosbag
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from PIL import Image
O=Path(__file__).resolve().parent;S=O.parents[2];R=O/'r01';P=S/'visuals/panels/physical_qualification';R.mkdir(exist_ok=True,parents=True);P.mkdir(exist_ok=True,parents=True)
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def csvwrite(p,rows):
 with p.open('w') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def gaps(ns):return np.diff(np.array(ns,dtype=np.int64))
D=[];inputs=[];gaprows=[];boundaryrows=[]
for stage in ('p01','p02'):
 run=S/'runs'/('m12_physics_'+stage);raw=json.loads((run/'probe_raw.json').read_text());report=json.loads((run/'probe_report.json').read_text());assert (run/'cleanup.json').exists();receipt=json.loads((run/'receipt.json').read_text());assert receipt['probe_raw.json_sha256']==sha(run/'probe_raw.json')
 start=round(raw['schedule_start_s']*1e9);end=start+36000000000;loss=start+32000000000
 fault=[round(r['t']*1e9) for r in raw['backend_fault'] if start<=round(r['t']*1e9)<=end];sourcekey='backend_status' if stage=='p02' else 'targets';source=[r['ns'] for r in raw[sourcekey] if start<=r['ns']<=end]
 commands=[dict(r,ns=r.get('ns',round(r['t']*1e9))) for r in raw['commands_sent'] if start<=r.get('ns',round(r['t']*1e9))<end]
 pair=[(a,b) for a,b in zip(commands,commands[1:]) if a['phase']==b['phase']];cx=np.array([(a['ns']+b['ns'])/2 for a,b in pair]);cg=np.array([b['ns']-a['ns'] for a,b in pair]);bagbool=[];bagcommands={'/cmd_vel_navigation':[],'/cmd_vel':[]}
 with rosbag.Bag(str(run/'fixture.bag'),'r') as bag:
  for topic,m,t in bag.read_messages(topics=['/m2_driver/physical_backend_fault','/cmd_vel_navigation','/cmd_vel']):
   ns=t.to_nsec()
   if topic=='/m2_driver/physical_backend_fault':
    if start<=ns<=end:bagbool.append(ns)
   else:bagcommands[topic].append({'ns':ns,'v':m.linear.x,'w':m.angular.z})
 for name,ns in [('probe_Bool_receipt',fault),('source_'+sourcekey,source),('bag_Bool_receipt',bagbool)]:
  for a,b in zip(ns,ns[1:]):gaprows.append({'stage':stage,'series':name,'from_ns':a,'to_ns':b,'elapsed_mid_s':((a+b)/2-start)/1e9,'gap_ms':(b-a)/1e6})
 for a,b in pair:gaprows.append({'stage':stage,'series':'same_phase_command','from_ns':a['ns'],'to_ns':b['ns'],'elapsed_mid_s':((a['ns']+b['ns'])/2-start)/1e9,'gap_ms':(b['ns']-a['ns'])/1e6})
 series={'command_event':commands,**bagcommands}
 for kind,rs in series.items():
  for row in rs:
   if loss-160000000<=row['ns']<=loss+160000000:boundaryrows.append({'stage':stage,'series':kind,'ns':row['ns'],'relative_loss_ms':(row['ns']-loss)/1e6,'v_mps':row['v'],'w_radps':row['w'],'inside_loss':loss<=row['ns']<end})
 D.append({'stage':stage,'raw':raw,'report':report,'start':start,'loss':loss,'fault':np.array(fault),'source':np.array(source),'bagbool':bagbool,'cx':cx,'cg':cg,'commands':commands,'series':series,'stats':{'raw_Bool_receipt_max_gap_ms':int(max(gaps(fault)))/1e6,'raw_Bool_duplicate_receipts':int(sum(gaps(fault)==0)),'bag_Bool_receipt_max_gap_ms':int(max(gaps(bagbool)))/1e6,'source_gap_ms':sorted(set((gaps(source)/1e6).tolist())),'within_phase_max_command_gap_ms':int(max(cg))/1e6,'within_phase_command_gaps_above_frozen_gate':int(sum(cg>raw['contract']['gates']['maximum_publish_gap_s']*1e9)),'commands_at_or_inside_loss':sum(loss<=r['ns']<end for r in commands)}})
 inputs += [{'path':str(run/p),'sha256':sha(run/p)} for p in ('probe_raw.json','probe_report.json','receipt.json','fixture.bag')]
csvwrite(R/'TIM01_gaps.csv',gaprows);csvwrite(R/'TIM01_loss_boundary.csv',boundaryrows)
font=Path('/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf');font_manager.fontManager.addfont(str(font));fn=font_manager.FontProperties(fname=str(font)).get_name();plt.rcParams.update({'font.family':['DejaVu Sans',fn],'font.size':8,'axes.titlesize':8.4,'axes.labelsize':8,'xtick.labelsize':7.2,'ytick.labelsize':7.2,'legend.fontsize':7.1,'axes.spines.top':False,'axes.spines.right':False,'axes.unicode_minus':False,'pdf.fonttype':42,'svg.fonttype':'none','lines.linewidth':.8,'axes.linewidth':.6,'xtick.major.width':.6,'ytick.major.width':.6})
blue='#17699E';orange='#B46917';red='#B64032';gray='#5A6167';green='#267B61'
fig,ax=plt.subplots(3,2,figsize=(170/25.4,180/25.4));fig.subplots_adjust(left=.115,right=.985,bottom=.195,top=.825,wspace=.31,hspace=.65)
fig.suptitle('TIM01  从接收时钟与浮点调度转向源时间协议',y=.985,fontsize=10)
fig.text(.5,.956,'P01 与 P02 各一个新身份回合：测量诊断，原失败保留',ha='center',fontsize=7.5,color=gray)
fig.legend(handles=[Line2D([0],[0],c=orange,label='probe Bool 接收'),Line2D([0],[0],c=blue,label='50 Hz 源包'),Line2D([0],[0],c=green,marker='o',ls='none',ms=3,label='命令源事件')],loc='upper center',bbox_to_anchor=(.53,.926),ncol=3,frameon=False)
for j,d in enumerate(D):
 start=d['start'];f=d['fault'];s=d['source'];stat=d['stats'];a=ax[0,j]
 a.plot(((f[1:]+f[:-1])/2-start)/1e9,gaps(f)/1e6,c=orange,lw=.55);a.plot(((s[1:]+s[:-1])/2-start)/1e9,gaps(s)/1e6,c=blue,lw=.8)
 a.set(title=d['stage'].upper()+'  seed '+str(d['raw']['seed'])+'\n接收最大间隔 {:.0f} ms；重复 {} 次'.format(stat['raw_Bool_receipt_max_gap_ms'],stat['raw_Bool_duplicate_receipts']),xlabel='正式开始后时间 / s',ylabel='相邻包间隔 / ms',xlim=(0,36),ylim=(-4,160));a.set_xticks([0,12,24,36])
 a=ax[1,j];a.scatter((d['cx']-start)/1e9,d['cg']/1e6,s=4,c=green,alpha=.8,linewidths=0);gate=d['raw']['contract']['gates']['maximum_publish_gap_s']*1000;a.axhline(gate,c=red,ls='--',lw=.8);a.text(.98,.91,'原门 {:.0f} ms'.format(gate),transform=a.transAxes,ha='right',color=red,fontsize=7.2)
 a.set(title='同一相位命令间隔：峰值 {:.0f} ms\n超原门 {} 个'.format(stat['within_phase_max_command_gap_ms'],stat['within_phase_command_gaps_above_frozen_gate']),xlabel='正式开始后时间 / s',ylabel='命令间隔 / ms',xlim=(0,36),ylim=(35,175));a.set_xticks([0,12,24,36])
 a=ax[2,j];a.axvspan(0,165,color=red,alpha=.075);a.axvline(0,color=red,ls='--',lw=.8)
 for y,(kind,rows) in enumerate(d['series'].items()):
  points=[r for r in rows if d['loss']-160000000<=r['ns']<=d['loss']+160000000]
  xs=[(r['ns']-d['loss'])/1e6 for r in points];a.scatter(xs,[y]*len(xs),s=19,c=green if y==0 else gray,marker=('o' if y==0 else '|'),zorder=3)
 a.set(yticks=[0,1,2],yticklabels=['命令源','bag 导航','bag relay'],ylim=(-.55,2.6),xlim=(-165,165),xticks=[-150,-75,0,75,150],xlabel='相对 loss 起点 / ms',title='32 s 边界：loss 内命令 {} 个'.format(stat['commands_at_or_inside_loss']));a.invert_yaxis()
 for i in range(3):ax[i,j].grid(axis='y',alpha=.18,lw=.4)
fig.text(.115,.110,'两回合 bag Bool 接收间隔均为 20 ms；probe 回调接收抖动不等于物理源降频。',fontsize=7.1,color=gray)
fig.text(.115,.082,'P01 的浮点边界仍发送 1.2 m/s；P02 整数纳秒调度在 [32, 36) s 不发送。',fontsize=7.1,color=gray)
fig.text(.115,.054,'P02 记录器握手＋1 s 预热使正式源数据完整；真实转向变化率门仍未通过。',fontsize=7.1,color=red)
fig.text(.115,.025,'不同 fresh seed 的单回合诊断，不作因果效应或导航性能统计；bag 命令时刻为接收时钟。',fontsize=6.9,color=gray)
for ext in ('pdf','svg','png'):fig.savefig(R/('TIM01.'+ext),dpi=600)
plt.close(fig);subprocess.run(['pdftoppm','-scale-to-x','1004','-scale-to-y','-1','-png','-singlefile',str(R/'TIM01.pdf'),str(P/'TIM01_r01_170mm')],check=True)
caption='P01与P02测量通道和命令时序的离线诊断。两列来自各自独立的新seed单回合，不能用于估计修复的统计因果效应。上排比较probe无Header Bool回调接收时钟与具有Header的50Hz源包；P01源包用actuator_target，P02用physical_status。接收最大间隔分别为{:.0f}/{:.0f}ms、重复接收时刻{}/{}次，但两次bag Bool接收最大间隔均为20ms，说明回调调度抖动不能直接解释为物理发布降频。中排展示同一相位内全部相邻命令间隔，P01/P02峰值{:.0f}/{:.0f}ms，超过冻结75ms门{}/{}个；故意无命令的loss段不作为“丢帧”计算。下排显示32s整数loss起点附近的全部源命令事件及bag导航/relay接收，P01仍在边界发送1.2m/s，P02在[32,36)s严格不发送。P01原Bool完整性失败保持，其AST诊断另外保留命令间隔、loss边界及实际转向变化率失败；P02正式源流和命令协议通过，但实际20ms中心角变化率0.88636rad/s仍超过0.85rad/s门。P01起始bag缺包与P02完整覆盖的独立审计见引用数据SHA。源事件时刻为Python publish前采样；bag导航、relay和Bool时刻为接收时钟，不视为真实驱动接收时间。'.format(D[0]['stats']['raw_Bool_receipt_max_gap_ms'],D[1]['stats']['raw_Bool_receipt_max_gap_ms'],D[0]['stats']['raw_Bool_duplicate_receipts'],D[1]['stats']['raw_Bool_duplicate_receipts'],D[0]['stats']['within_phase_max_command_gap_ms'],D[1]['stats']['within_phase_max_command_gap_ms'],D[0]['stats']['within_phase_command_gaps_above_frozen_gate'],D[1]['stats']['within_phase_command_gaps_above_frozen_gate'])
# Bind numerical physical result from audited data rather than manually supplied value.
prior=json.loads((S/'audits/m12_p02_raw_independent/independent_raw_diagnosis.json').read_text());caption=caption.replace('0.88636',format(prior['steering']['links']['center_rate_radps']['max_abs'],'.5f'))
(R/'TIM01_caption_zh.md').write_text(caption+'\n');(R/'TIM01_latex_insert.tex').write_text('\\begin{figure}[htbp]\n\\centering\n\\includegraphics[width=170mm]{TIM01.pdf}\n\\caption{'+caption+'}\n\\label{fig:tim01-measurement-clock}\n\\end{figure}\n')
inputs.extend({'path':str(p),'sha256':sha(p)} for p in [S/'audits/m12_p01_raw_independent/independent_raw_diagnosis.json',S/'audits/m12_p02_raw_independent/protocol_audit.json',S/'audits/m12_p02_raw_independent/independent_raw_diagnosis.json',font])
report={'figure_id':'TIM01','revision':'r01','scope':'historical offline measurement diagnosis; original negative outcomes preserved','raw_recomputed_stats':{d['stage']:d['stats'] for d in D},'size_mm':[170,180],'native_dpi':600,'native_pixels':list(Image.open(R/'TIM01.png').size),'view_image_status':'pending','inputs':inputs,'script':{'path':str(Path(__file__).resolve()),'sha256':sha(__file__)},'outputs':[{'path':str(p),'sha256':sha(p)} for p in sorted(R.glob('TIM01*'))]+[{'path':str(P/'TIM01_r01_170mm.png'),'sha256':sha(P/'TIM01_r01_170mm.png')}],'font_report':subprocess.check_output(['pdffonts',str(R/'TIM01.pdf')],text=True),'pdf_info':subprocess.check_output(['pdfinfo',str(R/'TIM01.pdf')],text=True),'command':'OPENBLAS_NUM_THREADS=1 /usr/bin/python3 '+str(Path(__file__).resolve()),'PHY01':'not_complete; preregistered three-control median pair unavailable; not substituted'}
(O/'TIM01_visual_qc.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');print(json.dumps({'stats':report['raw_recomputed_stats'],'outputs':report['outputs'][:3]},indent=2))
