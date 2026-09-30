#!/usr/bin/env python3
"""Actual N04 native PNG panels only, no synthetic scene drawing or replacement.

Consumes an immutable completed independent inventory, not live run files.
Follows the separate frozen pre-outcome selection rule. A family with missing
independent audits fails closed. Missing images in selected rows are retained.
"""
import argparse,collections,hashlib,json,math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np
from PIL import Image

HERE=Path(__file__).resolve().parent;S=HERE.parents[2]
D=S/'audits/navigation_n04_result_independent'
NAMES={'static_avoidance':'静态绕障','single_crossing':'单体横穿','head_on_yield':'迎面让行','sequential_interaction':'连续交互','occluded_emergence':'遮挡出现','width_change_corridor':'宽度变化'}
METHODS={'astra_analytic_route_v1':'解析方法','actual_teb_common_route':'TEB','frozen_r2_v060_sensor_lifecycle_v1':'冻结R2','dwa_common_route':'DWA'}
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def bind(p):return dict(path=str(Path(p).resolve()),sha256=sha(p))
def load(p):return json.loads(Path(p).read_bytes())
def sortkey(row,timekey):return (row[timekey],row['seed'],row['sequence'],row['run_id'])
def median(rows,timekey):
 rows=sorted(rows,key=lambda r:sortkey(r,timekey));return rows[(len(rows)-1)//2]
def choose(rows):
 selected=[];valid=[x for x in rows if x.get('valid_completed')]
 if valid:selected.append(('有效完成中位例',median(valid,'native_goal_time_s')))
 failures=[x for x in rows if x.get('observed_episode')and not x.get('valid_completed')]
 if failures:
  c=collections.Counter(x['failure_class']for x in failures);key=sorted(c,key=lambda x:(-c[x],x))[0]
  selected.append(('最大失败类中位例',median([x for x in failures if x['failure_class']==key],'restricted_goal_time_s')))
 return selected
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--inventory',type=Path,required=True);ap.add_argument('--figure-id',choices=['F06','F07','F08','F09','F10','F11'],required=True);ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
 if args.output.exists():raise FileExistsError('Preserve every figure revision')
 rule=load(D/'figure_selection_preregistration.json');inv=load(args.inventory);family=rule['figures'][args.figure_id];rows=[r for r in inv['rows']if r['family']==family]
 assert len(rows)==8 and all(x['closed']for x in rows),'family not terminal'
 assert all(x.get('retired') or x['independently_reviewed']for x in rows),'family independent review incomplete'
 selected=choose(rows);panels=[];selection=[]
 for role,row in selected:
  images=sorted(row.get('native_images',[]),key=lambda x:x['source_ns']);by_ns={x['source_ns']:x for x in images};images=[by_ns[k]for k in sorted(by_ns)]
  keep=images[:1]+images[-1:] if len(images)>1 else images
  selection.append(dict(role=role,run_id=row['run_id'],method=row['method'],seed=row['seed'],failure_class=row['failure_class'],selected_frame_count=len(keep),missing_images=not keep))
  for i,item in enumerate(keep):
   assert item['native_pixel_equality'] and item['source_stamp_equal'] and item['original_hashes_equal']
   png=Path(item['png']['path']);assert sha(png)==item['png']['sha256']
   with Image.open(png)as im:assert im.size==(2560,1440)
   panels.append((role,row,item,'首保留帧' if i==0 else '末保留帧'))
 args.output.mkdir(parents=True)
 manifest={'schema':'astra_n04_actual_scene_figure_v1','figure_id':args.figure_id,'family':family,'selection_rule':bind(D/'figure_selection_preregistration.json'),'inventory':bind(args.inventory),'all_eight_original_rows':rows,'selection':selection,'native_sources':[p[2]for p in panels],'script':bind(__file__),'status':'not_complete_no_actual_images'if not panels else'generated_pending_actual_visual_QC','visual_QC_complete':False,'no_new_Gazebo_images_created':True,'geometry_scope':'one reused parent, two parameter realizations, four methods','sources_unchanged':True}
 if not panels:
  (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n');return
 font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');font_manager.fontManager.addfont(str(font));name=font_manager.FontProperties(fname=str(font)).get_name()
 plt.rcParams.update({'font.family':name,'font.size':8,'axes.titlesize':8,'axes.linewidth':.6,'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none','savefig.facecolor':'white'})
 nrows=math.ceil(len(panels)/2);height_mm=45*nrows+62
 fig=plt.figure(figsize=(170/25.4,height_mm/25.4));gs=fig.add_gridspec(nrows+1,2,left=.025,right=.975,top=.90,bottom=.04,hspace=.26,wspace=.035,height_ratios=[1]*nrows+[.64])
 fig.suptitle(args.figure_id+'  '+NAMES[family]+'：实际Gazebo原始帧',x=.025,y=.985,ha='left',fontsize=10,fontweight='bold')
 for index,(role,row,item,frame_role)in enumerate(panels):
  ax=fig.add_subplot(gs[index//2,index%2]);ax.imshow(np.asarray(Image.open(item['png']['path'])));ax.axis('off')
  ax.set_title('%s · %s\n%s  t源=%.3f s'%(METHODS[row['method']],role,frame_role,item['source_ns']/1e9),loc='left',pad=3)
 ax=fig.add_subplot(gs[-1,:]);ax.axis('off')
 valid=sum(x.get('valid_completed',False)for x in rows);failed=sum(x.get('observed_episode',False)and not x.get('valid_completed',False)for x in rows);retired=sum(x.get('retired',False)for x in rows)
 details=['原计划8行：有效完成%d；保留失败%d；未运行退休%d。'%(valid,failed,retired),'代表例按事前中位规则选取；缺图不替换。源时间为图像Header，不是接收墙钟。','同一旧parent几何；图像不构成独立几何泛化或动态安全证明。']
 for x in selection:
  if x['missing_images']:details.append('%s / %s：缺原始帧，保留缺失。'%(METHODS[x['method']],x['role']))
 ax.text(0,1,'\n'.join(details),ha='left',va='top',fontsize=8,linespacing=1.5,transform=ax.transAxes)
 outputs=[]
 for ext in ('pdf','svg','png'):
  target=args.output/(args.figure_id+'.'+ext);fig.savefig(str(target),dpi=600);outputs.append(bind(target))
 plt.close(fig)
 caption='%s展示%s族的实际原始记录。有效完成代表按实际几何goal时间取下中位；失败代表来自最大失败类并按限制时间取下中位。保留全部八个预定身份及缺图，不按画面好坏替换。共用的是一个旧parent几何，两组参数实现及四种方法；名义起终点纵向距离9米，绕障引导线和实际轨迹不等同于该名义距离。原始PNG与源Header均可追溯，排版仅缩放显示；本图不证明物理安全、实际方法公平性或场景泛化。'%(args.figure_id,NAMES[family])
 cap=args.output/'caption_zh.txt';cap.write_text(caption+'\n');outputs.append(bind(cap))
 tex=args.output/'figure.tex';tex.write_text('\\begin{figure}[tb]\n\\centering\n\\includegraphics[width=170mm]{'+args.figure_id+'.pdf}\n\\caption{'+caption+'}\n\\label{fig:'+args.figure_id+'}\n\\end{figure}\n');outputs.append(bind(tex))
 manifest.update(outputs=outputs,width_mm=170,height_mm=height_mm,png_dpi=600,caption=caption,font_source=bind(font),manual_QC_requirements=['view native source frames and full170mm PDF render','confirm entity counts, landmarks and source time labels','check Chinese glyphs and absence of clipping/legend overlap','keep negative/missing selected examples visible'])
 (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False)+'\n')
 print(json.dumps({'figure_id':args.figure_id,'status':manifest['status'],'panel_count':len(panels),'manifest':bind(args.output/'manifest.json')},indent=2))
if __name__=='__main__':main()
