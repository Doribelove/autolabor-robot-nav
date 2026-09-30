#!/usr/bin/env python3
"""Three paper-sized figures from audited historical R2 evidence, without ROS.

All values come from raw reports or the independently replayed audit CSV files.
This exports offline diagnostics, never new Gazebo evidence or screenshots.
"""
import argparse
import collections
import csv
import hashlib
import json
import math
import re
import subprocess
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from PIL import Image
import numpy as np
import yaml

FONT=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
BLUE='#0072B2'; ORANGE='#D55E00'; GREEN='#009E73'; PURPLE='#AA4499'; GREY='#707070'
FAMILY={'core_static_avoidance':'静态绕障','core_single_crossing':'单参与者横穿',
        'core_head_on_yield':'迎面让行','core_sequential_interaction':'连续交互',
        'core_partially_occluded':'遮挡出现','core_width_change':'宽度变化'}


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read_csv(path):return list(csv.DictReader(Path(path).open()))
def save_csv(path,rows):
    with Path(path).open('w',newline='') as stream:
        w=csv.DictWriter(stream,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def sane(value):
    if isinstance(value,(np.integer,np.floating)):return value.item()
    if isinstance(value,Path):return str(value)
    raise TypeError(type(value).__name__)


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--workspace',type=Path,required=True)
    parser.add_argument('--revision',default='r01');args=parser.parse_args()
    root=args.workspace.resolve();stage=root/'artifacts/astra_gazebo_20260926'
    out=Path(__file__).resolve().parent;panels=stage/'visuals/panels/historical_diagnosis';panels.mkdir(parents=True,exist_ok=True)
    data_dir=out/'data';data_dir.mkdir(exist_ok=True)
    revision=out/args.revision;revision.mkdir(exist_ok=False)
    sources={};derived={};text_issues={};figs=[]
    def source(path):
        path=Path(path);sources[str(path.relative_to(root))]=sha(path);return path
    audit=stage/'audits/historical'
    geometric=read_csv(source(audit/'r2_failure_geometry.csv'))
    failures=read_csv(source(audit/'r2_pct_failures.csv'))
    tf_failures=read_csv(source(audit/'r2_tf_failures.csv'))
    figure_plan=source(stage/'figure_plan.csv')
    reports={}
    for r in failures:
        p=source(root/r['report_path']);reports[int(r['seed'])]=yaml.load(p.read_text(),Loader=yaml.CSafeLoader)
    selected=[198136,198146]  # All historical PCT conservative-clearance failures, not selected successes.
    assert {int(r['seed']) for r in failures if r['failure_reason']=='conservative_clearance_violation'}==set(selected)
    source(root/'src/simulation/m2_gazebo/urdf/m2.urdf.xacro')
    source(root/'src/simulation/m2_gazebo/urdf/m2_base.xacro')
    source(audit/'recompute_historical.py')
    plt.rcParams.update({'font.family':font_manager.FontProperties(fname=str(FONT)).get_name(),
        'font.size':8.2,'axes.titlesize':8.8,'axes.labelsize':8.2,'xtick.labelsize':7.7,'ytick.labelsize':7.7,
        'legend.fontsize':7.6,'axes.spines.top':False,'axes.spines.right':False,
        'axes.linewidth':.6,'lines.linewidth':1.2,'xtick.major.width':.6,'ytick.major.width':.6,
        'pdf.fonttype':42,'ps.fonttype':42,'svg.fonttype':'none','axes.unicode_minus':False,
        'figure.facecolor':'white','savefig.facecolor':'white'})

    def decorate(ax):
        ax.grid(axis='y',color='#DDDDDD',linewidth=.45);ax.set_axisbelow(True)
    def finish(fig,ident,title,mapping,caption,statistics,case_rule):
        fig.canvas.draw();renderer=fig.canvas.get_renderer();bounds=fig.bbox
        issues=[]
        for item in fig.findobj(matplotlib.text.Text):
            if not item.get_visible() or not item.get_text():continue
            box=item.get_window_extent(renderer)
            if box.x0<bounds.x0-1 or box.x1>bounds.x1+1 or box.y0<bounds.y0-1 or box.y1>bounds.y1+1:
                issues.append({'text':item.get_text(),'bbox':list(box.bounds)})
        text_issues[ident]=issues
        paths={}
        for ext in ('pdf','svg','png'):
            p=revision/(ident+'.'+ext);fig.savefig(p,dpi=600 if ext=='png' else 150)
            paths[ext]={'path':str(p.relative_to(root)),'sha256':sha(p)}
        render=panels/(ident+'_'+args.revision+'_print_150dpi')
        subprocess.run(['pdftoppm','-scale-to-x','1004','-scale-to-y','-1','-png','-singlefile',str(revision/(ident+'.pdf')),str(render)],check=True,stdout=subprocess.DEVNULL)
        render_path=render.with_suffix('.png')
        paths['print_render']={'path':str(render_path.relative_to(root)),'sha256':sha(render_path)}
        image=Image.open(revision/(ident+'.png'))
        pdf_info=subprocess.check_output(['pdfinfo',str(revision/(ident+'.pdf'))],text=True)
        fonts=subprocess.check_output(['pdffonts',str(revision/(ident+'.pdf'))],text=True)
        entry={'figure_id':ident,'title_zh':title,'figure_plan_related_ids':mapping,
            'status':'generated_pending_actual_visual_review','study':'historical Gazebo-PCT-R2 offline diagnosis',
            'new_Gazebo_result':False,'real_screenshot_count':0,'case_selection_rule':case_rule,
            'caption_zh':caption,'statistics':statistics,'files':paths,
            'native_png_pixels':list(image.size),'native_export_dpi':600,'paper_width_mm':170,
            'paper_height_mm':fig.get_size_inches()[1]*25.4,'text_outside_canvas':issues,
            'font_file':str(FONT),'font_sha256':sha(FONT),'pdf_info':pdf_info,'pdf_fonts':fonts,
            'minimum_font_size_pt':7.2,'suggested_chapter':'历史失效诊断与新研究动机'}
        figs.append(entry)
        (revision/(ident+'_caption.md')).write_text('# '+title+'\n\n'+caption+'\n\n数据身份：历史 R2；离线诊断。无新增仿真样本；不是 Gazebo 截图。\n')
        plt.close(fig)

    # H01: full failure traces. Shared horizontal extent prevents selective truncation.
    fig,axes=plt.subplots(3,2,figsize=(170/25.4,185/25.4))
    fig.subplots_adjust(left=.115,right=.985,bottom=.125,top=.86,wspace=.31,hspace=.37)
    fig.suptitle('历史 R2 两次净空失败：几何、运动与安全候选',x=.52,y=.984,fontsize=10)
    fig.text(.5,.952,'离线诊断  /  全部两次 PCT 保守净空失败  /  不是新 Gazebo 结果',ha='center',fontsize=7.2,color=GREY)
    fig.legend(handles=[Line2D([],[],color=ORANGE,label='旧保守圆净空'),Line2D([],[],color=BLUE,label='独立车体—参与者二维间距')],
               loc='upper center',bbox_to_anchor=(.53,.938),ncol=2,frameon=False)
    counts={};h01=[]
    for col,seed in enumerate(selected):
        report=reports[seed];start=report['control_trace'][0]['time_s'];decision=report['decision_trace']
        g=[r for r in geometric if int(r['seed'])==seed];ts=np.array([float(r['time_s'])-start for r in g])
        old=np.array([float(r['recorded_conservative_clearance_m']) for r in g]);gap=np.array([float(r['chassis_actor_signed_gap_m']) for r in g])
        speed=np.array([float(r['speed_mps']) for r in g]);dt=np.array([d['time_s']-start for d in decision])
        masks=np.array([sum(d['action_mask'][1:]) for d in decision]);stop=np.array([d['executed_action']==0 for d in decision])
        for ax in axes[:,col]:
            decorate(ax);ax.set_xlim(0,22);ax.set_xticks([0,5,10,15,20])
            for j in np.where(stop)[0]:ax.axvspan(dt[j],min(dt[j]+.25,report['result']['elapsed_s']),color='#D7D7D7',alpha=.38,lw=0,zorder=0)
        name=FAMILY[report['scene_id']]
        axes[0,col].set_title(('(a) ' if col==0 else '(b) ')+name+' · '+str(seed),loc='left',pad=6)
        axes[0,col].plot(ts,old,color=ORANGE);axes[0,col].plot(ts,gap,color=BLUE)
        axes[0,col].axhline(0,color=GREY,lw=.7,ls='--');axes[0,col].set_ylim(-.6,6);axes[0,col].set_yticks([0,2,4,6])
        worst=int(np.argmin(gap));counts[str(seed)]={'samples':len(g),'min_legacy_clearance_m':float(old.min()),'min_planar_gap_m':float(gap.min()),'overlap_samples':int((gap<0).sum()),'contact_messages':report['result']['contact_state_count']}
        label='二维最小 {:.2f} mm\n保守最小 {:.2f} mm'.format(gap.min()*1000,old.min()*1000)
        axes[0,col].annotate(label,xy=(ts[worst],gap[worst]),xytext=(.05,.84),textcoords='axes fraction',fontsize=7.7,
              color=BLUE,arrowprops={'arrowstyle':'-','color':BLUE,'linewidth':.6})
        axes[1,col].plot(ts,speed,color=BLUE);axes[1,col].set_ylim(-.04,1.27);axes[1,col].set_yticks([0,.4,.8,1.2])
        axes[2,col].step(dt,masks,where='post',color=GREEN);axes[2,col].set_ylim(-.3,7.5);axes[2,col].set_yticks([0,2,4,6])
        axes[2,col].set_xlabel('自策略门起的仿真时间 / s')
        for r in g:h01.append(dict(r,elapsed_s=float(r['time_s'])-start))
    for i,label in enumerate(['有符号间距 / m','机器人速度 / (m/s)','可用非停车候选数']):axes[i,0].set_ylabel(label)
    fig.text(.115,.043,'灰底：选择停车动作的决策区间；停车可调用并不等于停车包络已通过校验。',fontsize=7.2,color=GREY)
    fig.text(.115,.022,'二维负间距仅表示日志重建的投影相交，不能单独认定物理接触；无误差带。',fontsize=7.2,color=GREY)
    cap1=('图 H01：历史正式研究 R2 中全部两次 PCT 保守净空失败的离线多信号诊断。两列分别为连续交互 seed198136 和单参与者横穿 seed198146。'
        '第一行同时绘制原评价器的有裕量圆净空和按记录位姿独立计算的 1.04×0.70 m 车体与 0.72×0.72 m 参与者方盒二维间距；分离时为欧氏最短间距，相交时为负 SAT 最小平移深度。'
        '第二行为原日志机器人速度，第三行为每次决策 mask 中可用非停车动作数。灰色区间为执行 action0 的决策周期，并非独立停车事件。'
        '两次二维最小值分别为 '+', '.join('{:.3f} mm'.format(counts[str(s)]['min_planar_gap_m']*1000) for s in selected)+'；'+
        'seed198146 的二维投影相交仅占 {}/{} 个采样点。'.format(counts['198146']['overlap_samples'],counts['198146']['samples'])+
        '原始 contact 计数均为零，历史日志缺少完整 z、roll/pitch 和部件姿态，故不能将投影相交升级为确定物理接触。图为全记录时间线，无平滑、无置信区间，不替代新阶段验证。')
    finish(fig,'H01','两次历史净空失败的多信号诊断',['F04','F30','F39'],cap1,counts,'include both and all historical PCT conservative-clearance failures; full episode trace')
    save_csv(data_dir/'H01_trace.csv',h01)

    # H02: deterministic event window, common definition for both failure episodes.
    fig,axes=plt.subplots(2,2,figsize=(170/25.4,140/25.4))
    fig.subplots_adjust(left=.118,right=.985,bottom=.195,top=.815,wspace=.31,hspace=.49)
    fig.suptitle('方向性安全预测与参与者实际运动的失配',y=.985,fontsize=10)
    fig.text(.5,.95,'历史 R2 离线诊断：从策略门到首次持续停车开始后的 1 s',ha='center',fontsize=7.2,color=GREY)
    fig.legend(handles=[Line2D([],[],color=BLUE,label='扫描跟踪估计'),Line2D([],[],color=ORANGE,ls='--',label='日志 actor 1 / 实际路径'),
                        Line2D([],[],color=PURPLE,ls=':',label='首次持续停车开始')],
               loc='upper center',bbox_to_anchor=(.53,.931),ncol=3,frameon=False)
    h02=[];summary={}
    for col,seed in enumerate(selected):
        report=reports[seed];d=report['decision_trace'];start=d[0]['time_s']
        idx=next(i for i in range(len(d)-3) if all(x['executed_action']==0 for x in d[i:i+4]))
        onset=d[idx]['time_s']-start;end=onset+1;rows=[]
        for j,t in enumerate(d):
            elapsed=t['time_s']-start
            if elapsed>end+1e-8:break
            actor=t['actor_poses'][0];tr=t['directional_tracker_audit'];center=tr['center_world_m'];vec=tr['safety_direction_world']
            prev=d[max(0,j-1)]['actor_poses'][0];dx=actor['x_m']-prev['x_m'];dy=actor['y_m']-prev['y_m']
            measured=math.degrees(math.atan2(dx,dy)) if math.hypot(dx,dy)>1e-6 else None
            angle=math.degrees(math.atan2(vec[0],vec[1])) if math.hypot(*vec)>.5 else None
            rows.append({'seed':seed,'elapsed_s':elapsed,'tracker_x_error_m':center[0]-actor['x_m'] if center else None,
                        'true_actor_x_error_m':0.,'direction_angle_from_plus_y_deg':angle,
                        'actual_actor1_direction_from_plus_y_deg':measured,'tracker_valid':tr['valid'],
                        'executed_action':t['executed_action'],'sustained_stop_onset_s':onset})
        h02.extend(rows);x=[r['elapsed_s'] for r in rows]
        axes[0,col].plot(x,[r['tracker_x_error_m'] for r in rows],color=BLUE,marker='o',ms=2.3)
        axes[0,col].plot(x,[0]*len(x),color=ORANGE,ls='--')
        axes[1,col].plot(x,[r['direction_angle_from_plus_y_deg'] for r in rows],color=BLUE,marker='o',ms=2.3)
        axes[1,col].plot(x,[r['actual_actor1_direction_from_plus_y_deg'] for r in rows],color=ORANGE,ls='--',marker='.',ms=2)
        axes[0,col].set_title(('(a) ' if col==0 else '(b) ')+FAMILY[report['scene_id']]+' · '+str(seed),loc='left',pad=6)
        for ax in axes[:,col]:
            decorate(ax);ax.axvline(onset,color=PURPLE,ls=':',lw=1);ax.set_xlim(0,end);ax.set_xticks(np.arange(0,math.floor(end)+1))
        axes[0,col].set_ylim(-.2,.2);axes[0,col].set_yticks([-.2,-.1,0,.1,.2]);axes[1,col].set_ylim(-180,180);axes[1,col].set_yticks([-180,-90,0,90,180])
        axes[1,col].set_xlabel('自策略门起的仿真时间 / s')
        before=rows[idx-1];summary[str(seed)]={'window_end_s':end,'sustained_stop_onset_s':onset,
            'last_pre_stop_action':before['executed_action'],'last_pre_stop_estimated_angle_deg':before['direction_angle_from_plus_y_deg'],
            'last_pre_stop_actual_angle_deg':before['actual_actor1_direction_from_plus_y_deg'],
            'last_pre_stop_tracker_x_error_m':before['tracker_x_error_m']}
    axes[0,0].set_ylabel('估计中心 x 偏差 / m');axes[1,0].set_ylabel('相对 +y 的方向角 / °')
    fig.text(.118,.075,'+y 为 actor 1 的实际横穿方向；真实静止采样不赋予方向角。',fontsize=7.2,color=GREY)
    fig.text(.118,.041,'持续停车：连续 4 个停车周期（标称 1 s）；仅诊断关联，不证明因果。',fontsize=7.2,color=GREY)
    cap2=('图 H02：两次历史 PCT 保守净空失败的扫描跟踪中心与方向诊断。使用同一事件规则：从策略门开始，截取至首次连续四次停车决策开始后的 1 s；完整回合见 H01。'
        '上排显示跟踪中心 x 相对日志 actor1 x 的偏差，下排显示安全预测方向与实际 actor1 位移方向相对 +y 的角度；真实位移小于 1 μm 的采样不赋方向角。'
        '两例持续停车前最后一次 action'+str(summary['198136']['last_pre_stop_action'])+' 决策的估计方向分别偏离 +y '+
        ' / '.join('{:.2f}°'.format(summary[str(s)]['last_pre_stop_estimated_angle_deg']) for s in selected)+'，实际参与者沿 +y 横穿。'
        '这些事实支持检验“可见表面中心变化导致方向估计失配”的假设，但不是随机化因果证明；没有重跑旧种子、没有重训或新 Gazebo 样本，曲线不作平滑。')
    finish(fig,'H02','方向估计与真实参与者运动对照',['F29','F39'],cap2,summary,'both failure identities; same event-defined prefix window, four consecutive action0 decisions')
    save_csv(data_dir/'H02_direction.csv',h02)

    # H03: all timeout and TF-failure identities; diagnostic group rules are executable.
    timeout_rows=[];warnings=[];groups=['静态：候选耗尽','连续：候选耗尽','连续：近目标未到达'];group_count=collections.Counter()
    for r in failures:
        if r['failure_reason']!='timeout_or_nonarrival':continue
        report=reports[int(r['seed'])];trace=report['control_trace'];last=report['decision_trace'][-1]
        safe=sum(last['action_mask'][1:]);pose=report['result']['final_pose']
        category=groups[0] if r['scene_id']=='core_static_avoidance' and safe==0 else groups[1] if r['scene_id']=='core_sequential_interaction' and safe==0 else groups[2] if r['scene_id']=='core_sequential_interaction' and safe>0 and pose['x_m']>3.5 else '其他'
        group_count[category]+=1
        time=np.array([t['time_s'] for t in trace]+[trace[0]['time_s']+report['result']['elapsed_s']]);dur=np.maximum(np.diff(time),0)
        fraction=float(np.sum(dur*np.array([abs(t['speed_mps'])<=.05 for t in trace]))/dur.sum())
        timeout_rows.append({'seed':r['seed'],'scene_id':r['scene_id'],'category':category,'stationary_time_fraction':fraction,
                             'last_safe_nonstop_count':safe,'final_x_m':pose['x_m'],'final_y_m':pose['y_m']})
    assert sum(group_count.values())==18 and set(group_count)<=set(groups)
    for r in tf_failures:
        lp=source(root/r['log_path']);report_path=source(lp.parent/'episode_report.yaml')
        report=yaml.load(report_path.read_text(),Loader=yaml.CSafeLoader);start=report['control_trace'][0]['time_s'];end=start+report['result']['elapsed_s']
        for line in lp.read_text(errors='replace').splitlines():
            match=re.search(r'extrapolation ([0-9.]+)s into the future.*Requested time ([0-9.]+)',line)
            if match and start<=float(match[2])<=end:
                warnings.append({'seed':r['seed'],'scene_id':r['scene_id'],'future_extrapolation_ms':float(match[1])*1000,'requested_sim_time_s':float(match[2])})
    byscene=collections.Counter(r['scene_id'] for r in tf_failures)
    total_reject=sum(int(r['scan_timestamp_transform_failure_count']) for r in tf_failures)
    fig,axes=plt.subplots(2,2,figsize=(170/25.4,164/25.4))
    fig.subplots_adjust(left=.115,right=.982,bottom=.125,top=.86,wspace=.39,hspace=.71)
    fig.suptitle('历史 R2 超时与 TF 合同失败：分开记账、分开诊断',y=.981,fontsize=9.8)
    fig.text(.5,.943,'全部 PCT 超时 n={}；全部解析先验 TF 完整性失败 n={}（两总体不合并）'.format(len(timeout_rows),len(tf_failures)),ha='center',fontsize=7.2,color=GREY)
    positions=np.arange(3);counts=[group_count[g] for g in groups]
    axes[0,0].bar(positions,counts,color=[BLUE,GREEN,ORANGE],width=.62)
    axes[0,0].set_xticks(positions);axes[0,0].set_xticklabels(['静态\n候选耗尽','连续\n候选耗尽','连续\n近目标未到达'])
    axes[0,0].set_ylabel('超时回合数');axes[0,0].set_title('(a) PCT：末端状态分类',loc='left',pad=7);axes[0,0].set_ylim(0,max(counts)+2)
    for x,n in zip(positions,counts):axes[0,0].text(x,n+.15,str(n),ha='center',fontsize=8)
    families=[s for s in FAMILY if byscene[s]];tc=[byscene[s] for s in families]
    axes[0,1].bar(range(len(families)),tc,color=PURPLE,width=.65)
    axes[0,1].set_xticks(range(len(families)));axes[0,1].set_xticklabels([FAMILY[s].replace('单参与者横穿','横穿').replace('迎面让行','迎面').replace('遮挡出现','遮挡').replace('宽度变化','变宽') for s in families])
    axes[0,1].set_ylabel('完整性失败回合数');axes[0,1].set_title('(b) 解析先验：TF 失败分布',loc='left',pad=7);axes[0,1].set_ylim(0,max(tc)+3)
    for x,n in enumerate(tc):axes[0,1].text(x,n+.2,str(n),ha='center',fontsize=8)
    for j,(scene,color) in enumerate([('core_static_avoidance',BLUE),('core_sequential_interaction',GREEN)]):
        subset=[r for r in timeout_rows if r['scene_id']==scene];vals=np.array([r['stationary_time_fraction'] for r in subset])*100
        x=j+np.linspace(-.13,.13,len(vals));axes[1,0].scatter(x,vals,s=18,color=color,alpha=.85,linewidth=.3,edgecolor='white')
        axes[1,0].hlines(np.median(vals),j-.23,j+.23,color='black',lw=1.2)
    axes[1,0].set_xlim(-.5,1.5);axes[1,0].set_ylim(0,100);axes[1,0].set_yticks([0,25,50,75,100]);axes[1,0].set_ylabel('近静止时长占比 / %')
    axes[1,0].set_xticks([0,1]);axes[1,0].set_xticklabels(['静态（n={}）'.format(sum(r['scene_id']=='core_static_avoidance' for r in timeout_rows)),
                          '连续（n={}）'.format(sum(r['scene_id']=='core_sequential_interaction' for r in timeout_rows))]);axes[1,0].set_title('(c) PCT：近静止时间负担',loc='left',pad=7)
    delays=[r['future_extrapolation_ms'] for r in warnings];hist=collections.Counter(round(v,3) for v in delays)
    axes[1,1].bar(list(hist),list(hist.values()),width=1.2,color=PURPLE)
    axes[1,1].set_xlim(0,max(delays)+3);axes[1,1].set_xlabel('TF 向未来外推缺口 / ms');axes[1,1].set_ylabel('运行期日志条数')
    axes[1,1].set_title('(d) 解析先验：时间戳缺口',loc='left',pad=7)
    axes[1,1].text(.98,.95,'日志 n={}\n计数器拒绝 n={}'.format(len(warnings),total_reject),transform=axes[1,1].transAxes,ha='right',va='top',fontsize=7.4)
    for ax in axes.flat:decorate(ax)
    fig.text(.115,.059,'候选耗尽：末次非停车 mask 全拒绝；近静止：记录速度绝对值 ≤0.05 m/s。',fontsize=7.2,color=GREY)
    fig.text(.115,.032,'(c) 点为完整回合，横线为中位数；(d) 日志条不是独立实验单位；均为描述性分析。',fontsize=7.2,color=GREY)
    stats={'timeout_categories':dict(group_count),'TF_failure_families':dict(byscene),'counter_rejected_scans':total_reject,
           'runtime_log_extrapolation_rows':len(warnings),'future_gap_ms_range':[min(delays),max(delays)],
           'all_tf_failures_latest_transform_success_rate_one':all(float(r['latest_tf_success_rate'])==1 for r in tf_failures)}
    cap3=('图 H03：全部历史 R2 PCT 超时与解析先验 TF 完整性失败的分类诊断。PCT 的 '+str(len(timeout_rows))+' 次超时按原始末次 mask 与位姿分类：'+
          '、'.join(g+' '+str(group_count[g])+' 次' for g in groups)+'。解析先验另有 '+str(len(tf_failures))+' 回合违反 scan 时间戳 TF 合同，不能与 PCT 超时合并成一个失败总体。'
          '面板(c)以实际记录时间间隔加权计算 |v|≤0.05 m/s 的近静止时长比例，点为完整回合，短横线为组内中位数；不将每次 stop 决策误当独立停车事件。'
          '面板(d)仅使用 episode 运行时间内原日志明确报告的向未来外推警告，共 '+str(len(warnings))+' 条，缺口 '+
          '{:.0f}–{:.0f} ms'.format(min(delays),max(delays))+'；报告计数器累计拒绝 '+str(total_reject)+' 个 scan。所有这些 TF 失败回合的最新 TF 查询成功率仍为100%，说明应分别审计最新可用TF与scan采样时刻TF。'
          '这些分类支持后继恢复与时间戳队列假设，不构成因果结论或新评估结果；失败均继续保留原22s主指标记账。')
    finish(fig,'H03','超时与扫描时间戳失败的分类诊断',['F32','F33','F39'],cap3,stats,'all 18 PCT timeout identities and all 21 analytic TF-integrity failures; no exclusion')
    save_csv(data_dir/'H03_timeout_classes.csv',timeout_rows);save_csv(data_dir/'H03_runtime_TF_warnings.csv',warnings)
    (revision/'source_manifest.json').write_text(json.dumps({'source_sha256':sources,'script_sha256':sha(__file__),
       'command':'python3 '+str(Path(__file__).resolve().relative_to(root))+' --workspace '+str(root)+' --revision '+args.revision,
       'revision':args.revision,'figures':figs,'text_extent_machine_check':text_issues},indent=2,ensure_ascii=False,default=sane)+'\n')
    thumbs=[]
    for f in figs:
        img=Image.open(root/f['files']['print_render']['path']).convert('RGB');img.thumbnail((500,600));thumbs.append(img)
    atlas=Image.new('RGB',(1500,650),'#EEEEEE')
    for i,img in enumerate(thumbs):atlas.paste(img,(i*500,10))
    atlas.save(panels/('contact_sheet_'+args.revision+'.png'))
    (revision/'latex_inserts.tex').write_text('\n\n'.join('\\begin{figure}[htbp]\n\\centering\n\\includegraphics[width=170mm]{'+f['figure_id']+'.pdf}\n\\caption{'+f['caption_zh']+'}\n\\label{fig:historical-'+f['figure_id'].lower()+'}\n\\end{figure}' for f in figs)+'\n')
    print(json.dumps({'revision':args.revision,'figures':[f['figure_id'] for f in figs],'text_extent_issues':text_issues,'statistics':[f['statistics'] for f in figs]},indent=2,ensure_ascii=False))


if __name__=='__main__':main()
