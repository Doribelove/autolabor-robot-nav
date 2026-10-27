#!/usr/bin/python3 -s
"""Read-only offline extraction and coverage/control analysis of split ROS bags."""
import argparse
import collections
import csv
import datetime
import hashlib
import json
import math
from pathlib import Path

import numpy as np
import rosbag
from scipy.spatial.transform import Rotation


def dump(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def path_points(path):
    return [[p.pose.position.x, p.pose.position.y, yaw(p.pose.orientation)] for p in path.poses]


def extract(root, out):
    bags = sorted(root.glob('*.bag'), key=lambda p: int(p.stem.rsplit('_', 1)[1]))
    assert bags and not list(root.glob('*.bag.active')), 'Stop and finalize recording first'
    identity = [[p.name, p.stat().st_size, p.stat().st_mtime_ns] for p in bags]
    cache = out/'extracted.json'
    if cache.exists():
        cached = json.loads(cache.read_text())
        if cached['identity'] == identity:
            print('Using offline extraction cache', flush=True)
            return cached['data']
    with rosbag.Bag(str(bags[0])) as b:
        epoch = b.get_start_time()
    rows = collections.defaultdict(list)
    counts = collections.Counter()
    bag_info = []
    pose_topics = ['/localization', '/Odometry', '/odom']
    twist_topics = ['/cmd_vel_teb', '/cmd_vel_unlocalized', '/cmd_vel_navigation', '/cmd_vel_safe', '/cmd_vel']
    scalar_topics = ['/m2_driver/wheel_angle', '/m2_driver/left_wheel_vel', '/m2_driver/right_wheel_vel', '/coverage/hybrid_tracking_error']
    parameters = '/move_base/TebLocalPlannerROS/parameter_updates'
    paths = ['/coverage/planned_path', '/coverage/executed_path', '/coverage/hybrid_transition_path', '/coverage/first_entry_path']
    selected = pose_topics + twist_topics + scalar_topics + paths + [parameters, '/coverage/status', '/coverage/enforced_path',
        '/fast_lio/localization_status', '/m2_driver/chassis_info', '/m2_driver/chassis_monitor', '/rosout', '/tf', '/tf_static',
        '/initialpose', '/move_base/TebLocalPlannerROS/local_plan', '/move_base/CoverageGlobalPlanner/plan_hybrid_transitions/result']
    last_status = last_enforced = last_chassis = last_monitor = None
    last_local = -1e9
    last_path = {}
    for p in bags:
        print('Extracting', p.name, flush=True)
        with rosbag.Bag(str(p)) as b:
            topic_info = b.get_type_and_topic_info().topics
            per_topic = {t: i.message_count for t, i in topic_info.items()}
            counts.update(per_topic)
            bag_info.append({'name': p.name, 'bytes': p.stat().st_size, 'start': b.get_start_time(), 'end': b.get_end_time(), 'messages': sum(per_topic.values()), 'topics': per_topic})
            for topic, m, stamp in b.read_messages(topics=selected):
                t = stamp.to_sec() - epoch
                if topic in pose_topics:
                    pose = m.pose.pose; q = pose.orientation; pos = pose.position
                    rows[topic].append([t, m.header.stamp.to_sec()-epoch, pos.x, pos.y, pos.z, q.x, q.y, q.z, q.w, m.twist.twist.linear.x, m.twist.twist.angular.z])
                elif topic in twist_topics:
                    rows[topic].append([t, m.linear.x, m.angular.z])
                elif topic in scalar_topics:
                    rows[topic].append([t, m.data])
                elif topic == '/coverage/status':
                    keys = ['state', 'plan_id', 'active', 'paused', 'current_segment', 'total_segments', 'blocked_segments', 'coverage_ratio',
                        'requested_area_m2', 'reachable_area_m2', 'traversed_area_m2', 'chassis_ready', 'chassis_detail', 'avoidance_ready', 'localized', 'detail']
                    d = {k: getattr(m, k) for k in keys}
                    if d != last_status:
                        rows['status'].append(dict(t=t, **d)); last_status = d
                elif topic == '/coverage/enforced_path':
                    key = [m.active, m.planner_mode, m.plan_id, m.segment_index, m.path_generation, m.expected_gear]
                    if key != last_enforced:
                        rows['enforced'].append({'t': t, 'active': m.active, 'mode': m.planner_mode, 'plan_id': m.plan_id, 'segment': m.segment_index,
                            'generation': m.path_generation, 'gear': m.expected_gear, 'path': path_points(m.path)})
                        last_enforced = key
                elif topic == parameters:
                    rows['teb_parameters'].append({'t': t, 'values': {p.name: p.value for group in [m.bools, m.ints, m.doubles, m.strs] for p in group}})
                elif topic == '/fast_lio/localization_status':
                    f = dict(s.split('=', 1) for s in m.data.split(';') if '=' in s)
                    rows['quality'].append([t, f.get('state'), *[float(f.get(k, 'nan')) for k in ['overlap', 'rmse', 'inliers']]])
                elif topic in ['/m2_driver/chassis_info', '/m2_driver/chassis_monitor']:
                    d = {k: getattr(m, k) for k in m.__slots__ if k != 'header'}
                    if topic.endswith('chassis_info'):
                        d = {k: v for k, v in d.items() if 'emergency' in k}
                        if d != last_chassis: rows['chassis'].append(dict(t=t, **d)); last_chassis = d
                    elif d != last_monitor:
                        rows['controller'].append(dict(t=t, **d)); last_monitor = d
                elif topic == '/rosout':
                    if m.level >= 4 or any(s in m.msg for s in ['fixed-gear coverage', 'coverage task', 'coverage segment']):
                        rows['logs'].append({'t': t, 'stamp': m.header.stamp.to_sec()-epoch, 'node': m.name, 'level': m.level, 'message': m.msg})
                elif topic in ['/tf', '/tf_static']:
                    for tf in m.transforms:
                        if (tf.header.frame_id.lstrip('/'), tf.child_frame_id.lstrip('/')) == ('body', 'base_link') and not rows['body_to_base']:
                            q = tf.transform.rotation; v = tf.transform.translation
                            rows['body_to_base'].append([v.x, v.y, v.z, q.x, q.y, q.z, q.w])
                elif topic in paths:
                    pts = path_points(m)
                    key = hashlib.sha256(np.asarray(pts, dtype=np.float64).tobytes()).hexdigest()
                    if key != last_path.get(topic):
                        rows['paths'].append({'t': t, 'topic': topic, 'points': pts}); last_path[topic] = key
                elif topic == '/move_base/TebLocalPlannerROS/local_plan':
                    if t-last_local >= .5:
                        rows['local_plans'].append({'t': t, 'frame': m.header.frame_id, 'points': path_points(m)}); last_local = t
                elif topic == '/initialpose':
                    rows['initialpose'].append({'t': t, 'frame': m.header.frame_id, 'x': m.pose.pose.position.x, 'y': m.pose.pose.position.y, 'yaw': yaw(m.pose.pose.orientation)})
                elif topic.endswith('/result'):
                    rows['hybrid_results'].append({'t': t, 'status': m.status.status, 'text': m.status.text})
    data = {'epoch': epoch, 'duration': max(x['end'] for x in bag_info)-epoch, 'bags': bag_info, 'topic_counts': dict(counts), 'rows': dict(rows)}
    dump(cache, {'identity': identity, 'data': data})
    print('Extraction saved', flush=True)
    return data


def intervals(times, mask, end):
    if len(times) == 0: return []
    edges = np.r_[0, np.flatnonzero(np.diff(mask.astype(int)))+1, len(times)]
    return [[float(times[a]), float(times[b] if b < len(times) else end)] for a, b in zip(edges[:-1], edges[1:]) if mask[a]]


def hold(times, source, column=1, default=0):
    if not len(source): return np.full(len(times), default)
    a = np.asarray(source, float); idx = np.searchsorted(a[:, 0], times, side='right')-1
    values = a[np.maximum(idx, 0), column].copy(); values[idx < 0] = default
    return values


def analyze(root, out, data):
    r = data['rows']; duration = data['duration']; epoch = data['epoch']
    grid = np.arange(0, duration, .05)
    status = r.get('status', []); enforced = r.get('enforced', [])
    active = hold(grid, [[x['t'], int(x['active'])] for x in status]).astype(bool)
    paused = hold(grid, [[x['t'], int(x['paused'])] for x in status]).astype(bool)
    # Hybrid deliberately sets EnforcedPath.active=false to route through the
    # Hybrid global planner; planner_mode still identifies the current profile.
    mode = hold(grid, [[x['t'], x['mode']] for x in enforced], default=-1)
    gear = hold(grid, [[x['t'], x['gear']] for x in enforced])
    final = hold(grid, r['/cmd_vel']); final_w = hold(grid, r['/cmd_vel'], 2)
    v_wheel = hold(grid, r['/odom'], 9); w_wheel = hold(grid, r['/odom'], 10)
    zero = (np.abs(final) < 1e-4) & (np.abs(final_w) < 1e-4)
    hybrid = active & ~paused & (mode == 2)
    working = active & ~paused
    stops = intervals(grid, hybrid & zero, duration)
    def seconds(mask): return round(float(np.count_nonzero(mask)*.05), 3)
    def localtime(t): return datetime.datetime.fromtimestamp(epoch+t).astimezone().isoformat(timespec='milliseconds')
    windows = intervals(grid, active, duration)
    phases = intervals(grid, hybrid, duration)
    logs = r.get('logs', [])
    patterns = {'gear_violation': 'optimized command violated fixed gear', 'gear_stabilizing': 'fixed-gear stabilization is holding zero',
        'low_progress_replan': 'made less than 0.10 m progress', 'suffix_invalid': 'invalidated its cached suffix',
        'path_deviation_replan': 'left its fixed-gear path', 'retry_exhausted': 'exhausted one retry window',
        'control_timeout': 'VCU ControlTimeout', 'mux_zero': 'Hybrid TEB mux is holding zero'}
    events = {k: [x for x in logs if s in x['message']] for k, s in patterns.items()}
    stats = {'recording_start': localtime(0), 'recording_end': localtime(duration), 'duration_s': duration,
        'bags': len(data['bags']), 'bytes': sum(x['bytes'] for x in data['bags']), 'messages': sum(data['topic_counts'].values()),
        'topics': len(data['topic_counts']), 'active_windows': [{'start_s':a,'end_s':b,'start':localtime(a),'end':localtime(b)} for a,b in windows],
        'working_unpaused_s': seconds(working), 'active_paused_s': seconds(active & paused), 'hybrid_unpaused_s': seconds(hybrid),
        'hybrid_final_zero_s': seconds(hybrid & zero), 'hybrid_final_zero_fraction': float(np.mean(zero[hybrid])) if hybrid.any() else None,
        'hybrid_zero_intervals_ge_80ms': [x for x in stops if x[1]-x[0] >= .08], 'hybrid_phases': phases,
        'event_counts': {k: len(v) for k,v in events.items()}, 'last_status': status[-1], 'initialposes': r.get('initialpose', []),
        'peak_abs_final_v_mps': float(np.max(np.abs(final))), 'peak_abs_final_w_rps': float(np.max(np.abs(final_w))),
        'wheel_distance_integral_whole_m': float(np.sum(np.abs(v_wheel))*.05), 'wheel_distance_integral_working_m': float(np.sum(np.abs(v_wheel[working]))*.05),
        'event_count_note': 'Counts are recorded log messages; throttled logs are not optimizer solve counts.',
        'duration_method': '20 Hz offline zero-order-hold sampling, independent of actual controller rate.'}
    dump(out/'events.json', events)
    # Pose conversion matches the recorded body -> base_link extrinsic.
    pose = np.array(r['/localization'], float)
    tf = np.array(r['body_to_base'][0], float)
    rot = Rotation.from_quat(pose[:, 5:9]); base = pose[:, 2:5] + rot.apply(np.tile(tf[:3], (len(pose), 1)))
    heading = np.unwrap((rot * Rotation.from_quat(tf[3:])).as_euler('xyz')[:, 2])
    moving_pose = np.abs(hold(pose[:, 0], r['/odom'], 9)) > .03
    dp = np.linalg.norm(np.diff(base[:, :2], axis=0), axis=1)
    stats['global_pose_path_length_whole_m'] = float(dp.sum())
    stats['global_pose_path_length_wheel_moving_m'] = float(dp[moving_pose[1:] & moving_pose[:-1]].sum())
    stats['global_pose_net_displacement_m'] = float(np.linalg.norm(base[-1,:2]-base[0,:2]))
    stats['pose_length_note'] = 'Raw sampled localization trajectory includes drift/corrections; no independent ground truth.'
    quality = r.get('quality', [])
    localized = [x for x in quality if x[1] == 'LOCALIZED']
    if localized:
        stats['localization'] = {'first_localized_s':localized[0][0], 'last_localized_s':localized[-1][0],
            'overlap_min':float(min(x[2] for x in localized)), 'rmse_max_m':float(max(x[3] for x in localized)),
            'rmse_median_m':float(np.median([x[3] for x in localized])),
            'nonlocalized_after_first':sum(x[1]!='LOCALIZED' for x in quality if x[0]>=localized[0][0])}
    # Capture actual dynamic-reconfigure values for Hybrid segments, not ordinary navigation defaults.
    checks=[]
    updates=r.get('teb_parameters', [])
    for x in events['gear_violation']:
        previous=[p for p in updates if p['t']<=x['t']]
        vals=previous[-1]['values'] if previous else {}
        checks.append({'t':x['t'],'weight':vals.get('weight_kinematics_forward_drive'),'direction_mode':vals.get('motion_direction_mode'),
            'max_vel_x_backwards':vals.get('max_vel_x_backwards'),'message':x['message']})
    for x in checks:
        x['planner_mode']=int(hold(np.array([x['t']]),[[p['t'],p['mode']] for p in enforced],default=-1)[0])
    stats['fixed_gear_violation_parameter_checks']=checks
    stats['hybrid_weight_values_at_violations']=sorted({x['weight'] for x in checks if x['planner_mode']==2 and x['weight'] is not None})
    stats['parameter_check_note']='Last recorded dynamic-reconfigure configuration at each fixed-gear violation; inspect direction/profile before attributing to Hybrid.'
    stats['violation_profiles']={str(k):sum(x['planner_mode']==k for x in checks) for k in [2,3]}
    pauses=[x for x in status if x['active'] and x['paused']]
    stats['first_pause']=pauses[0] if pauses else None
    stages=[]
    for s in status:
        if not s['active'] or s['paused']:continue
        if not stages or stages[-1]['segment']!=s['current_segment']:
            if stages:stages[-1]['end_s']=s['t']
            stages.append({'segment':s['current_segment'],'start_s':s['t'],'initial_state':s['state']})
    if stages:stages[-1]['end_s']=pauses[0]['t'] if pauses else duration
    for s in stages:s['duration_s']=s['end_s']-s['start_s']
    stats['segment_durations']=stages
    sweep_metrics=[]
    for s in [x for x in enforced if x['mode']==3 and len(x['path'])>1]:
        stop=next((x['t'] for x in enforced if x['t']>s['t'] and x['mode']!=3),pauses[0]['t'] if pauses else duration)
        ids=(pose[:,0]>=s['t'])&(pose[:,0]<stop)
        p0=np.array(s['path'][0][:2]);p1=np.array(s['path'][-1][:2]);direction=(p1-p0)/np.linalg.norm(p1-p0)
        relative=base[ids,:2]-p0
        lateral=np.abs(relative[:,0]*direction[1]-relative[:,1]*direction[0])
        along=relative@direction
        angle=np.abs(np.angle(np.exp(1j*(math.atan2(direction[1],direction[0])-heading[ids]))))
        if not len(lateral):continue
        sweep_metrics.append({'segment':s['segment']+1,'start_s':s['t'],'end_s':stop,'reference_length_m':float(np.linalg.norm(p1-p0)),
            'lateral_median_m':float(np.median(lateral)),'lateral_p95_m':float(np.percentile(lateral,95)),'lateral_max_m':float(lateral.max()),
            'start_lateral_m':float(lateral[0]),'start_heading_deg':float(np.rad2deg(angle[0])),
            'end_lateral_m':float(lateral[-1]),'end_heading_deg':float(np.rad2deg(angle[-1])),
            'end_along_m':float(along[-1]),'max_along_m':float(along.max()),
            'metric_note':'Localization-based lateral distance to the reference sweep line, not independent absolute accuracy.'})
    stats['sweep_tracking']=sweep_metrics
    progress_windows=[]
    for e in events['low_progress_replan']:
        mask=(grid>=e['t']-2)&(grid<e['t'])
        if not mask.any():continue
        progress_windows.append({'t':e['t'],'final_nonzero_fraction':float(np.mean(np.abs(final[mask])>.05)),
            'final_v_median_mps':float(np.median(final[mask])),
            'wheel_abs_distance_2s_m':float(np.sum(np.abs(v_wheel[mask]))*.05),
            'command_abs_distance_2s_m':float(np.sum(np.abs(final[mask]))*.05)})
    stats['low_progress_nonzero_80pct_wheel_under_10cm']=sum(x['final_nonzero_fraction']>=.8 and x['wheel_abs_distance_2s_m']<.1 for x in progress_windows)
    dump(out/'low_progress_command_feedback.json',progress_windows)
    unique_changes=[];last=None
    for s in status:
        key=(s['plan_id'],s['state'],s['current_segment'],s['total_segments'],s['detail'])
        if key!=last: unique_changes.append(s);last=key
    dump(out/'status_timeline.json',unique_changes)
    with (out/'status_timeline.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['time_local','t_s','state','segment','total','ratio','area_m2','detail']);writer.writeheader()
        for s in unique_changes:writer.writerow(dict(time_local=localtime(s['t']),t_s=s['t'],state=s['state'],segment=s['current_segment'],total=s['total_segments'],ratio=s['coverage_ratio'],area_m2=s['traversed_area_m2'],detail=s['detail']))
    np.savetxt(out/'global_base_pose.csv',np.c_[pose[:,0],base,heading],delimiter=',',header='t_bag_s,x_m,y_m,z_m,yaw_rad',comments='')
    np.savetxt(out/'control_20hz.csv',np.c_[grid,final,final_w,v_wheel,w_wheel,active,paused,mode,gear],delimiter=',',header='t_s,final_v_mps,final_w_rps,wheel_v_mps,wheel_w_rps,active,paused,planner_mode,gear',comments='')
    dump(out/'recording_summary.json', {k:data[k] for k in ['epoch','duration','bags','topic_counts']})
    dump(out/'metrics.json',stats)
    plot(out,r,stats,grid,base,pose[:,0],final,final_w,v_wheel,w_wheel,mode,hybrid,zero)
    print(json.dumps(stats,ensure_ascii=False,indent=2),flush=True)


def plot(out,r,stats,grid,base,pose_t,final,final_w,v_wheel,w_wheel,mode,hybrid,zero):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.font_manager import FontProperties
    font=Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists(): plt.rcParams['font.family']=FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'axes.unicode_minus':False,'savefig.dpi':160,'font.size':10})
    def save(fig,name):
        fig.savefig(out/(name+'.png'),bbox_inches='tight');fig.savefig(out/(name+'.svg'),bbox_inches='tight');plt.close(fig)
    fig,ax=plt.subplots(figsize=(12,9))
    for p in r.get('paths',[]):
        if p['topic']=='/coverage/planned_path' and len(p['points']):
            a=np.array(p['points']);ax.plot(a[:,0],a[:,1],color='#a1a7af',lw=1,alpha=.6)
    ax.plot(base[:,0],base[:,1],color='#1764c0',lw=1.4,label='ICP定位换算至车体中心的实际轨迹')
    moving=(np.abs(hold(pose_t,r['/odom'],9))>.03)
    ax.scatter(base[moving,0],base[moving,1],c=pose_t[moving],s=3,cmap='viridis',label='运动采样点（颜色随时间）')
    ax.scatter(base[0,0],base[0,1],marker='o',s=70,color='green',label='定位轨迹起点');ax.scatter(base[-1,0],base[-1,1],marker='X',s=90,color='red',label='录制终点')
    ax.set_aspect('equal');ax.grid(alpha=.25);ax.legend();ax.set_xlabel('map X / m');ax.set_ylabel('map Y / m');ax.set_title('本轮覆盖清扫：规划参考与实际定位轨迹（非独立真值）');save(fig,'01_coverage_trajectory')
    fig,axes=plt.subplots(4,1,figsize=(15,12),sharex=True)
    axes[0].plot(grid,final,label='最终速度指令',lw=.8);axes[0].plot(grid,v_wheel,label='轮速里程计反馈',lw=.8,alpha=.8);axes[0].set_ylabel('线速度 / m/s')
    axes[1].plot(grid,final_w,label='最终角速度指令',lw=.8);axes[1].plot(grid,w_wheel,label='轮角速度反馈',lw=.8,alpha=.8);axes[1].set_ylabel('角速度 / rad/s')
    angle=np.array(r['/m2_driver/wheel_angle']);axes[2].plot(angle[:,0],np.rad2deg(angle[:,1]),lw=.7,label='实际前轮转角');axes[2].set_ylabel('转角 / 度')
    state_names=list(dict.fromkeys(s['state'] for s in r['status']));sy={s:i for i,s in enumerate(state_names)}
    st=r['status'];axes[3].step([s['t'] for s in st],[sy[s['state']] for s in st],where='post',lw=1.2,label='覆盖状态');axes[3].set_yticks(list(sy.values()),list(sy));axes[3].set_ylabel('任务状态')
    for a in axes:a.grid(alpha=.25);a.legend(loc='upper right')
    axes[-1].set_xlabel('相对录包起点 / s');fig.suptitle('完整实验时间线：控制指令、执行反馈与覆盖状态');fig.tight_layout();save(fig,'02_experiment_timeline')
    if stats['hybrid_phases']:
        low=max(0,stats['hybrid_phases'][0][0]-3);high=stats['hybrid_phases'][-1][1]+3
        fig,axes=plt.subplots(4,1,figsize=(15,12),sharex=True)
        for topic,color,label in [('/cmd_vel_teb','#db8625','TEB已发布输出'),('/cmd_vel','#2469b3','最终底盘指令')]:
            a=np.array(r.get(topic,[]),float)
            if not len(a):continue
            mask=(a[:,0]>=low)&(a[:,0]<=high);a=a[mask]
            axes[0].step(a[:,0],a[:,1],where='post',lw=.8,label=label,color=color)
            axes[1].step(a[:,0],a[:,2],where='post',lw=.8,label=label,color=color)
        axes[0].plot(grid,v_wheel,color='#268555',alpha=.7,lw=.8,label='轮速反馈')
        axes[2].plot(angle[:,0],np.rad2deg(angle[:,1]),lw=.8,label='实际前轮转角')
        axes[3].step(grid,mode,where='post',label='模式：2=Hybrid转场，3=清扫直线');axes[3].fill_between(grid,0,1,where=hybrid&zero,color='red',alpha=.3,label='转场期间最终零指令')
        for ax,label in zip(axes,['线速度 / m/s','角速度 / rad/s','转角 / 度','规划模式']):ax.set_ylabel(label);ax.set_xlim(low,high);ax.grid(alpha=.25);ax.legend(loc='upper right')
        axes[-1].set_xlabel('相对录包起点 / s');fig.suptitle('覆盖转场：TEB发布值、最终指令与底盘反馈（负的内部候选解另见事件日志）');fig.tight_layout();save(fig,'03_transit_control')
    q=[x for x in r['quality'] if x[1]=='LOCALIZED']
    if q:
        a=np.array([[x[0],*x[2:]] for x in q],float);fig,axes=plt.subplots(2,1,figsize=(14,6),sharex=True)
        axes[0].plot(a[:,0],a[:,1],lw=.8);axes[0].set_ylabel('ICP overlap');axes[1].plot(a[:,0],a[:,2],lw=.8);axes[1].set_ylabel('ICP RMSE / m');axes[1].set_xlabel('相对录包起点 / s')
        for ax in axes:ax.grid(alpha=.25)
        fig.suptitle('定位匹配质量（ICP残差不等于绝对定位精度）');fig.tight_layout();save(fig,'04_localization_quality')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('dataset',type=Path);args=p.parse_args()
    root=args.dataset.resolve();out=root/'analysis_coverage';out.mkdir(exist_ok=True)
    analyze(root,out,extract(root,out))
