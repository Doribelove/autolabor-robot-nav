#!/usr/bin/python3 -s
"""Offline pose curves for a recorded MID360/FAST-LIO/ICP navigation test."""
import argparse, collections, datetime, json, math
from pathlib import Path
import numpy as np
import pandas as pd
import rosbag
from scipy.spatial.transform import Rotation, Slerp
from scipy.signal import savgol_filter
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('dataset',type=Path)
args=parser.parse_args();root=args.dataset.resolve();out=root/'analysis';out.mkdir(exist_ok=True)
bags=sorted(root.glob('*.bag'));assert bags
if (root/'crop_manifest.json').exists():
    start_ns=json.loads((root/'crop_manifest.json').read_text())['start_ns']
else:
    with rosbag.Bag(str(bags[0])) as first_bag:
        start_ns=next(first_bag.read_messages(raw=True)).timestamp.to_nsec()
font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if font.exists():plt.rcParams['font.family']=FontProperties(fname=str(font)).get_name()
plt.rcParams.update({'axes.unicode_minus':False,'font.size':10,'savefig.dpi':170,'pdf.fonttype':42})
pose_topics={'/localization':'global','/Odometry':'lio','/odom':'wheel'}
pose_rows=collections.defaultdict(list);quality=[];correction=[];wheel_feedback=collections.defaultdict(list)
cmd=[];events=[];initial_poses=[];coverage=[];extrinsics=[]
selected=list(pose_topics)+['/tf','/fast_lio/localization_status','/cmd_vel',
    '/m2_driver/left_wheel_vel','/m2_driver/right_wheel_vel','/coverage/status','/initialpose','/rosout']
def sec(stamp):return (stamp.to_nsec()-start_ns)/1e9
def xyzw(q):return [q.x,q.y,q.z,q.w]
def xyz(p):return [p.x,p.y,p.z]
cache=out/'extracted_pose.json'
cache_keys=['pose_rows', 'quality', 'correction', 'wheel_feedback', 'cmd', 'events', 'initial_poses', 'coverage', 'extrinsics']
identity=[(str(p.name),p.stat().st_size,p.stat().st_mtime_ns) for p in bags]
cached=json.loads(cache.read_text()) if cache.exists() else None
if cached is not None and cached['identity']==[list(v) for v in identity]:
    for key in cache_keys:globals()[key]=cached['data'][key]
    print('Using extracted pose cache',flush=True)
else:
    for bag_path in bags:
        print('Reading poses:',bag_path.name,flush=True)
        with rosbag.Bag(str(bag_path)) as bag:
            for topic,msg,stamp in bag.read_messages(topics=selected):
                t=sec(stamp)
                if topic in pose_topics:
                    p=msg.pose.pose
                    pose_rows[pose_topics[topic]].append([sec(msg.header.stamp),t,*xyz(p.position),
                        *xyzw(p.orientation),msg.twist.twist.linear.x,msg.twist.twist.angular.z,
                        msg.pose.covariance[0],msg.pose.covariance[7],msg.pose.covariance[35]])
                elif topic=='/tf':
                    for tf in msg.transforms:
                        pair=(tf.header.frame_id.lstrip('/'),tf.child_frame_id.lstrip('/'))
                        if pair==('body','base_link'):extrinsics.append([t,*xyz(tf.transform.translation),*xyzw(tf.transform.rotation)])
                        elif pair==('map','camera_init'):correction.append([t,*xyz(tf.transform.translation),*xyzw(tf.transform.rotation)])
                elif topic=='/fast_lio/localization_status':
                    fields=dict(item.split('=',1) for item in msg.data.split(';') if '=' in item)
                    quality.append([t,fields.get('state','UNKNOWN'),*[float(fields.get(k,'nan')) for k in ('overlap','rmse','inliers')]])
                elif topic=='/cmd_vel':cmd.append([t,msg.linear.x,msg.angular.z])
                elif topic.startswith('/m2_driver/'):
                    wheel_feedback[topic].append([t,msg.data])
                elif topic=='/coverage/status':coverage.append([t,msg.state,msg.active,msg.current_segment,msg.total_segments,msg.detail])
                elif topic=='/initialpose':initial_poses.append({'t':t,'frame':msg.header.frame_id,'xyz':xyz(msg.pose.pose.position),'quaternion':xyzw(msg.pose.pose.orientation)})
                elif topic=='/rosout' and msg.level>=4:events.append({'t':t,'node':msg.name,'level':int(msg.level),'message':msg.msg})
    cache.write_text(json.dumps({'identity':identity,'data':{key:globals()[key] for key in cache_keys}}))
assert extrinsics,'Missing body -> base_link transform'
extr=np.array(extrinsics);offset=extr[0,1:4];r_offset=Rotation.from_quat(extr[0,4:8])
assert np.max(np.linalg.norm(extr[:,1:4]-offset,axis=1))<1e-7,'Vehicle extrinsic changed during test'
assert np.max(np.linalg.norm((Rotation.from_quat(extr[:,4:8])*r_offset.inv()).as_rotvec(),axis=1))<1e-7
columns=['t_header','t_bag','x','y','z','qx','qy','qz','qw','vx_reported','wz_reported','cov_x','cov_y','cov_yaw']
poses={};rates={}
for name,rows in pose_rows.items():
    frame=pd.DataFrame(rows,columns=columns)
    rates[name]={'raw_messages':len(frame),'duplicate_header_stamps':int(frame.t_header.duplicated().sum()),
        'header_age_median_s':float(np.median(frame.t_bag-frame.t_header)),
        'header_age_p95_s':float(np.percentile(frame.t_bag-frame.t_header,95))}
    # Curves use source measurement time; retain original bag times in each CSV.
    frame=frame[(frame.t_header>=0)&(frame.t_header<300)].sort_values('t_header',kind='mergesort').drop_duplicates('t_header',keep='last').reset_index(drop=True)
    assert len(frame)>20
    rotations=Rotation.from_quat(frame[['qx','qy','qz','qw']].values)
    positions=frame[['x','y','z']].values
    if name!='wheel':
        positions=positions+rotations.apply(np.broadcast_to(offset,positions.shape))
        rotations=rotations*r_offset
    angles=rotations.as_euler('xyz');angles[:,2]=np.unwrap(angles[:,2])
    for i,key in enumerate(('x_base','y_base','z_base')):frame[key]=positions[:,i]
    for i,key in enumerate(('roll_deg','pitch_deg','yaw_deg')):frame[key]=np.rad2deg(angles[:,i])
    frame.to_csv(out/(name+'_pose.csv'),index=False,float_format='%.10f')
    poses[name]={'df':frame,'t':frame.t_header.values,'p':positions,'r':rotations,'angles':angles}
    dt=np.diff(frame.t_header.values)
    rates[name].update(unique_samples=len(frame),median_dt_s=float(np.median(dt)),max_gap_s=float(dt.max()))
quality=pd.DataFrame(quality,columns=['t','state','overlap','rmse','inliers']);quality.to_csv(out/'localization_quality.csv',index=False)
correction=pd.DataFrame(correction,columns=['t','x','y','z','qx','qy','qz','qw']).sort_values('t').drop_duplicates('t',keep='last')
correction['yaw_deg']=np.rad2deg(np.unwrap(Rotation.from_quat(correction[['qx','qy','qz','qw']].values).as_euler('xyz')[:,2]))
correction.to_csv(out/'map_to_camera_init.csv',index=False)
common_start=max(v['t'][0] for v in poses.values());common_end=min(v['t'][-1] for v in poses.values())
grid=np.arange(math.ceil(common_start*10)/10,common_end,.1)
assert len(grid)>100
interp={}
for name,data in poses.items():
    interp[name]={'p':np.column_stack([np.interp(grid,data['t'],data['p'][:,i]) for i in range(3)]),
        'r':Slerp(data['t'],data['r'])(grid)}
g=interp['global'];aligned={'global':g['p']};aligned_r={'global':g['r']};alignment={}
for name in ('lio','wheel'):
    item=interp[name]
    if name=='lio':r=g['r'][0]*item['r'][0].inv()
    else:r=Rotation.from_euler('z',g['r'][0].as_euler('xyz')[2]-item['r'][0].as_euler('xyz')[2])
    aligned[name]=r.apply(item['p']-item['p'][0])+g['p'][0]
    aligned_r[name]=r*item['r']
    alignment[name]={'time_s':float(grid[0]),'method':'one initial pose, '+('SE3' if name=='lio' else 'SE2 yaw and translation'),
        'rotation_xyzw':r.as_quat().tolist(),'translation':(g['p'][0]-r.apply(item['p'][0])).tolist()}
angles={k:np.column_stack((r.as_euler('xyz')[:,:2],np.unwrap(r.as_euler('xyz')[:,2]))) for k,r in aligned_r.items()}
comparison=pd.DataFrame({'t':grid})
for name in poses:
    for i,key in enumerate(('x','y','z')):comparison[name+'_'+key]=aligned[name][:,i]
    comparison[name+'_yaw_deg']=np.rad2deg(angles[name][:,2])
    if name!='global':
        comparison[name+'_xy_difference']=np.linalg.norm(aligned[name][:,:2]-g['p'][:,:2],axis=1)
        comparison[name+'_yaw_difference_deg']=np.rad2deg(np.angle(np.exp(1j*(angles[name][:,2]-angles['global'][:,2]))))
comparison.to_csv(out/'aligned_comparison_10hz.csv',index=False,float_format='%.10f')
# Derived speed is an offline 1.1-second local polynomial derivative, not zero FAST-LIO twist.
smooth_velocity=savgol_filter(g['p'],11,3,deriv=1,delta=.1,axis=0)
speed=np.linalg.norm(smooth_velocity[:,:2],axis=1)
wheel=poses['wheel'];v=np.interp(grid,wheel['t'],wheel['df'].vx_reported);w=np.interp(grid,wheel['t'],wheel['df'].wz_reported)
stationary=(np.abs(v)<.03)&(np.abs(w)<.02)
segments=[]
for ids in np.split(np.arange(len(grid)),np.where(np.diff(stationary.astype(int))!=0)[0]+1):
    if len(ids)==0 or not stationary[ids[0]] or len(ids)*.1<5:continue
    p=g['p'][ids];a=angles['global'][ids]
    segments.append({'start_s':float(grid[ids[0]]),'end_s':float(grid[ids[-1]]),'duration_s':float(len(ids)*.1),
        'xy_net_drift_m':float(np.linalg.norm(p[-1,:2]-p[0,:2])),
        'xy_radius_from_mean_max_m':float(np.max(np.linalg.norm(p[:,:2]-p[:,:2].mean(axis=0),axis=1))),
        'x_range_m':float(np.ptp(p[:,0])),'y_range_m':float(np.ptp(p[:,1])),
        'z_range_m':float(np.ptp(p[:,2])),'yaw_range_deg':float(np.rad2deg(np.ptp(a[:,2])))})
pd.DataFrame(segments).to_csv(out/'stationary_segments.csv',index=False)
labels={'global':'ICP 全局定位（map）','lio':'FAST-LIO（首帧对齐）','wheel':'轮式里程计（首帧对齐）'}
colors={'global':'#1764c0','lio':'#ef8b2c','wheel':'#2d9663'}
def save(fig,name):
    fig.savefig(out/(name+'.png'),bbox_inches='tight');fig.savefig(out/(name+'.svg'),bbox_inches='tight');plt.close(fig)
fig,ax=plt.subplots(figsize=(10,8))
# Optional map context; these files are an existing prior, not a generated map.
try:
    import yaml
    from PIL import Image
    config=yaml.safe_load((root/'map_reference/map_fused_2d/map.yaml').read_text())
    image=np.asarray(Image.open(root/'map_reference/map_fused_2d'/Path(config['image']).name))
    origin=config['origin'];res=config['resolution']
    if abs(origin[2])<1e-9:
        ax.imshow(np.flipud(image),cmap='gray',vmin=0,vmax=255,origin='lower',alpha=.42,
            extent=[origin[0],origin[0]+image.shape[1]*res,origin[1],origin[1]+image.shape[0]*res],zorder=0)
except Exception as exc:print('Map background unavailable:',exc,flush=True)
for name in ('wheel','lio','global'):ax.plot(aligned[name][:,0],aligned[name][:,1],color=colors[name],lw=1.4,label=labels[name])
ax.scatter(*g['p'][0,:2],s=85,color='#222222',marker='o',label='分析窗口起点',zorder=8)
ax.scatter(*g['p'][-1,:2],s=110,color='#d52d37',marker='X',label='分析窗口终点',zorder=8)
annotation_groups=[]
for second in (60,120,180,240):
    idx=np.argmin(abs(grid-second));point=g['p'][idx,:2]
    matching=next((group for group in annotation_groups if np.linalg.norm(group[0]-point)<1.5),None)
    if matching is None:annotation_groups.append([point,[second]])
    else:matching[1].append(second)
for point,seconds in annotation_groups:
    ax.annotate('/'.join(str(v) for v in seconds)+' s',point,xytext=(5,5),textcoords='offset points',fontsize=9)
all_xy=np.concatenate([p[:,:2] for p in aligned.values()]);lo=all_xy.min(axis=0);hi=all_xy.max(axis=0);pad=np.maximum((hi-lo)*.09,1.5)
ax.set_xlim(lo[0]-pad[0],hi[0]+pad[0]);ax.set_ylim(lo[1]-pad[1],hi[1]+pad[1]);ax.set_aspect('equal',adjustable='box')
ax.set(xlabel='map X / m',ylabel='map Y / m',title='前 5 分钟车体轨迹（base_link）\n对齐后的里程计用于相互比较，非外部真值')
ax.grid(alpha=.25);ax.legend(loc='best',fontsize=9);save(fig,'01_xy_trajectory')
fig,axes=plt.subplots(3,2,figsize=(14,10),sharex=True)
for i in range(3):
    for name in ('global','lio','wheel'):
        if name!='wheel' or i!=2:
            axes[i,0].plot(grid,aligned[name][:,i],lw=1,color=colors[name],label=labels[name])
        if name!='wheel' or i==2:axes[i,1].plot(grid,np.rad2deg(angles[name][:,i]),lw=1,color=colors[name],label=labels[name])
    axes[i,0].set_ylabel(('X','Y','Z')[i]+' / m');axes[i,1].set_ylabel(('横滚 Roll','俯仰 Pitch','连续航向 Yaw')[i]+' / °')
for ax in axes.ravel():ax.grid(alpha=.25);ax.set_xlim(0,300)
axes[0,0].legend(fontsize=8);axes[0,1].legend(fontsize=8);axes[2,1].legend(fontsize=8)
for ax in axes[-1]:ax.set_xlabel('相对录制起点的源时间 / s')
fig.suptitle('位姿六自由度时间曲线（统一车体参考点；首帧对齐）',y=.985)
fig.tight_layout(rect=(0,0,1,.96));save(fig,'02_pose_time_curves')
fig,axes=plt.subplots(4,1,figsize=(13,11),sharex=True)
for name in ('lio','wheel'):
    axes[0].plot(grid,comparison[name+'_xy_difference'],color=colors[name],label=labels[name])
    axes[1].plot(grid,comparison[name+'_yaw_difference_deg'],color=colors[name],label=labels[name])
for name in ('global','lio','wheel'):
    d=poses[name]['df'];axes[2].plot(d.t_header,d.t_bag-d.t_header,color=colors[name],lw=.8,label=labels[name])
axes[3].plot(grid,speed,color=colors['global'],label='定位位移导出速度（1.1s平滑）')
axes[3].plot(grid,np.abs(v),color=colors['wheel'],label='底盘反馈 |v|')
if cmd:
    c=np.asarray(cmd);axes[3].plot(c[:,0],np.abs(c[:,1]),color='#9d45b0',alpha=.65,lw=.8,label='最终指令 |v|')
for ax,ylabel in zip(axes,('与全局定位的 XY 差 / m','航向差 / °','记录时间－源时间 / s','平面速度 / (m/s)')):
    ax.set_ylabel(ylabel);ax.grid(alpha=.25);ax.legend(fontsize=8,loc='best');ax.set_xlim(0,300)
axes[-1].set_xlabel('相对录制起点的时间 / s');fig.suptitle('运动一致性与时序（差值不是绝对定位误差）',y=.985)
fig.tight_layout(rect=(0,0,1,.96));save(fig,'03_consistency_and_speed')
fig,axes=plt.subplots(4,1,figsize=(13,10),sharex=True)
for ax,col,label in zip(axes[:3],('overlap','rmse','inliers'),('ICP 重叠率','ICP 残差 RMSE / m','ICP 内点数')):
    ax.plot(quality.t,quality[col],lw=1,color=colors['global']);ax.set_ylabel(label)
axes[3].plot(correction.t,correction.x-correction.x.iloc[0],label='map→camera_init ΔX / m')
axes[3].plot(correction.t,correction.y-correction.y.iloc[0],label='map→camera_init ΔY / m')
axes[3].set_ylabel('全局修正平移变化 / m');axes[3].legend(fontsize=8)
for ax in axes:ax.grid(alpha=.25);ax.set_xlim(0,300)
axes[-1].set_xlabel('相对录制起点的记录时间 / s');fig.suptitle('定位配准质量与全局修正（RMSE为配准残差）',y=.985)
fig.tight_layout(rect=(0,0,1,.96));save(fig,'04_localization_quality')
def stats(values):
    a=np.asarray(values);a=a[np.isfinite(a)]
    return {'min':float(np.min(a)),'median':float(np.median(a)),'p95':float(np.percentile(a,95)),'max':float(np.max(a)),'mean':float(np.mean(a))}
metrics={'requested_seconds':300,'curve_interval_s':[float(grid[0]),float(grid[-1])],
    'body_to_base_link':{'xyz_m':offset.tolist(),'quaternion_xyzw':r_offset.as_quat().tolist()},
    'source_frames':{'global':'map/body -> map/base_link','lio':'camera_init/body -> aligned map/base_link','wheel':'odom/base_link -> aligned map/base_link'},
    'header_stamp_handling':'source-time curves; exclude source stamps outside [0,300); stable sort and keep last duplicate stamp',
    'rates_and_timing':rates,'alignments':alignment,'path_length_xy_m':float(np.linalg.norm(np.diff(g['p'][:,:2],axis=0),axis=1).sum()),
    'net_displacement_xy_m':float(np.linalg.norm(g['p'][-1,:2]-g['p'][0,:2])),
    'start_xyz_m':g['p'][0].tolist(),'end_xyz_m':g['p'][-1].tolist(),
    'xyz_ranges_m':np.ptp(g['p'],axis=0).tolist(),'roll_pitch_yaw_deg_ranges':[stats(np.rad2deg(angles['global'][:,i])) for i in range(3)],
    'wheel_signed_speed_mps':stats(v),'derived_global_planar_speed_mps':stats(speed),
    'moving_seconds_abs_v_gt_0_03':float(np.count_nonzero(np.abs(v)>.03)*.1),
    'stationary_segments':segments,'localization_state_message_counts':quality.state.value_counts().to_dict(),
    'icp_overlap':stats(quality.overlap),'icp_rmse_m':stats(quality.rmse),'icp_inliers':stats(quality.inliers),
    'consistency':{name:{'xy_m':stats(comparison[name+'_xy_difference']),
        'yaw_difference_deg':stats(comparison[name+'_yaw_difference_deg']),
        'final_xy_m':float(comparison[name+'_xy_difference'].iloc[-1])} for name in ('lio','wheel')},
    'largest_global_pose_steps':[],
    'initial_pose_messages_in_crop':initial_poses,
    'limits':['No independent ground truth: inter-estimator differences and ICP residuals are not absolute positioning error.',
        '/localization copies FAST-LIO covariance and twist; covariance is not calibrated ICP accuracy, and zero twist is not vehicle standstill.',
        'Initial alignment is frozen; wheel comparison can include wheel slip, heading/scale error and localization corrections.',
        'Only the first 300 s by bag record time are retained. Curves use header time and omit initial queued older measurements.']}
steps=np.linalg.norm(np.diff(g['p'][:,:2],axis=0),axis=1)
for idx in np.argsort(steps)[-10:][::-1]:metrics['largest_global_pose_steps'].append({'t_s':float(grid[idx+1]),'delta_t_s':.1,'xy_step_m':float(steps[idx]),'wheel_speed_mps':float(v[idx+1])})
first_turn_index=np.argmin(np.abs(grid-12.))
metrics['first_turn_0_to_12s']={
    'start_s':float(grid[0]),'end_s':float(grid[first_turn_index]),
    'global_yaw_change_deg':float(np.rad2deg(angles['global'][first_turn_index,2]-angles['global'][0,2])),
    'wheel_yaw_change_deg':float(np.rad2deg(angles['wheel'][first_turn_index,2]-angles['wheel'][0,2])),
    'wheel_yaw_difference_deg':float(comparison.wheel_yaw_difference_deg.iloc[first_turn_index])}
wheel_steps=np.linalg.norm(np.diff(poses['wheel']['p'][:,:2],axis=0),axis=1)
metrics['wheel_largest_adjacent_xy_step_m']=float(wheel_steps.max())
(out/'metrics.json').write_text(json.dumps(metrics,ensure_ascii=False,indent=2)+'\n')
(out/'warnings_and_errors.json').write_text(json.dumps(events,ensure_ascii=False,indent=2)+'\n')
pd.DataFrame(coverage,columns=['t','state','active','segment','total','detail']).to_csv(out/'coverage_context.csv',index=False)
report=['前5分钟位姿与运动曲线分析','',
    '分析对象：'+str(root),'原始消息截取区间：记录起点后[0,300)秒。',
    '三维点云等全部话题保留在 localization_first_300s.bag；图表仅提取位姿相关数据。',
    '曲线使用消息源时间；初始排队的较早消息不纳入曲线，CSV同时保存源时间和记录时间。',
    '车体统一到base_link；FAST-LIO固定初始SE3对齐，轮式里程计固定初始航向/平移对齐。','',
    '全局定位XY累计路程：%.3f m；起终点直线距离：%.3f m。'%(metrics['path_length_xy_m'],metrics['net_displacement_xy_m']),
    '底盘反馈速度：最小%.3f、最大%.3f m/s；|v|>0.03 m/s时长约%.1f s。'%(np.min(v),np.max(v),metrics['moving_seconds_abs_v_gt_0_03']),
    '全局定位状态消息：'+str(metrics['localization_state_message_counts']),
    'ICP overlap：最小%.4f，中位%.4f；配准RMSE：中位%.4f m、最大%.4f m。'%(quality.overlap.min(),quality.overlap.median(),quality.rmse.median(),quality.rmse.max()),
    '与首帧对齐FAST-LIO的XY差：中位%.3f m、95分位%.3f m、最大%.3f m。'%(metrics['consistency']['lio']['xy_m']['median'],metrics['consistency']['lio']['xy_m']['p95'],metrics['consistency']['lio']['xy_m']['max']),
    '与首帧对齐轮式里程计的XY差：中位%.3f m、95分位%.3f m、最大%.3f m。'%(metrics['consistency']['wheel']['xy_m']['median'],metrics['consistency']['wheel']['xy_m']['p95'],metrics['consistency']['wheel']['xy_m']['max']),
    '', '连续静止段（底盘|v|<0.03m/s且|w|<0.02rad/s，至少5秒）：']
for s in segments:report.append('%.1f–%.1f s：XY净漂移%.3f m，距均值最大半径%.3f m，Z范围%.3f m，航向范围%.3f°。'%(s['start_s'],s['end_s'],s['xy_net_drift_m'],s['xy_radius_from_mean_max_m'],s['z_range_m'],s['yaw_range_deg']))
if not segments:report.append('无满足条件的连续静止段。')
turn=metrics['first_turn_0_to_12s']
report+=['','关键差异：',
    '首次转弯（%.1f–%.1fs），全局航向变化%.2f°，底盘里程计航向变化%.2f°，两者航向差%.2f°。'%(turn['start_s'],turn['end_s'],turn['global_yaw_change_deg'],turn['wheel_yaw_change_deg'],turn['wheel_yaw_difference_deg']),
    '轮式里程计单次相邻位置步长最大%.3fm，曲线差异主要在转弯中逐步累计，未发现米级突发位置跳变。'%metrics['wheel_largest_adjacent_xy_step_m'],
    '建议后续核查底盘转角/角速度与航向积分的匹配；此为曲线推断，本次不修改底盘或导航。']
report+=['','图表：01_xy_trajectory，02_pose_time_curves，03_consistency_and_speed，04_localization_quality（PNG/SVG）。',
    '原始提取位姿和10Hz对齐表以CSV保存；计算和时间处理口径见metrics.json。','',
    '解释边界：没有独立真值，不能由这些曲线给出绝对定位精度。ICP RMSE是配准残差，',
    '/localization的协方差与零twist继承自FAST-LIO，不能分别当作ICP误差保证或停车证据。',
    '全局定位和FAST-LIO共享同一雷达输入，二者差异仅说明内部一致性，不能互相充当独立真值。',
    'XYZ高度来自历史地图坐标原点，不把非零/负高度直接判断为标定错误。',
    '复现：python3 -s scripts/analyze_localization_pose.py '+str(root)]
(out/'report.txt').write_text('\n'.join(report)+'\n')
print(json.dumps({k:metrics[k] for k in ('path_length_xy_m','net_displacement_xy_m','xyz_ranges_m','stationary_segments','localization_state_message_counts','consistency','largest_global_pose_steps')},ensure_ascii=False,indent=2),flush=True)
