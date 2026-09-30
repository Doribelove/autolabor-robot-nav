#!/usr/bin/env python3
"""Read-only exploratory mechanism diagnosis of preserved P01, no reassessment."""
from pathlib import Path
import csv
import hashlib
import json
import math

STAGE=Path(__file__).resolve().parents[1]
RAW=STAGE/'runs/m12_physics_p01/probe_raw.json'
OUT=STAGE/'audits/m12_p01_steering_mechanism'
OUT.mkdir(exist_ok=True)
data=json.loads(RAW.read_text());L=.65;a=.3

def center(l,r):
    tl,tr=math.tan(l),math.tan(r)
    return math.atan(L*(tl/(L+a*tl)+tr/(L-a*tr))/2)

def center_rate(l,r,ld,rd):
    tl,tr=math.tan(l),math.tan(r);km=(tl/(L+a*tl)+tr/(L-a*tr))/2
    kd=(L*(1+tl*tl)*ld/(L+a*tl)**2+L*(1+tr*tr)*rd/(L-a*tr)**2)/2
    return L*kd/(1+(L*km)**2)

def relative_q(parent,child):
    ax,ay,az,aw=-parent[0],-parent[1],-parent[2],parent[3];bx,by,bz,bw=child
    return (aw*bx+ax*bw+ay*bz-az*by,aw*by-ax*bz+ay*bw+az*bx,
            aw*bz+ax*by-ay*bx+az*bw,aw*bw-ax*bx-ay*by-az*bz)

def yaw(q):
    x,y,z,w=q;return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))

def dense_state(row):
    indices={name:i for i,name in enumerate(row['names'])};base=indices['autolabor_m2::base_link'];bq=row['poses'][base][3:]
    x,y,z,w=bq;axis=(2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y));ba=row['twists'][base][3:]
    states=[]
    for side in ('left','right'):
        i=indices['autolabor_m2::front_'+side+'_steer_link'];q=relative_q(bq,row['poses'][i][3:]);ang=row['twists'][i][3:]
        states.append((yaw(q),sum((ang[k]-ba[k])*axis[k] for k in range(3))))
    return states

joints={r['ns']:r for r in data['joints']};targets={r['ns']:r for r in data['targets']};efforts={r['ns']:r for r in data['commanded_efforts']}
dense={r['ns']:dense_state(r) for r in data['dense_links']}
rows=[];full=[];previous=None
for ns,j in sorted(joints.items()):
    left=j['positions']['front_left_steer_joint'];right=j['positions']['front_right_steer_joint']
    ld=j['velocities']['front_left_steer_joint'];rd=j['velocities']['front_right_steer_joint'];actual=center(left,right)
    target=targets.get(ns);effort=efforts.get(ns)
    if target is None or effort is None:continue
    row={'t_s':ns/1e9,'relative_schedule_s':ns/1e9-data['schedule_start_s'],'actual_center_rad':actual,
         'actual_center_rate_from_joint_velocity_radps':center_rate(left,right,ld,rd),
         'target_center_rad':target['positions']['center_steering'],'target_center_rate_radps':target['velocities']['center_steering'],
         'left_actual_rad':left,'right_actual_rad':right,'left_rate_radps':ld,'right_rate_radps':rd,
         'left_target_rad':target['positions']['front_left_steering'],'right_target_rad':target['positions']['front_right_steering'],
         'left_target_rate_radps':target['velocities']['front_left_steering'],'right_target_rate_radps':target['velocities']['front_right_steering'],
         'left_commanded_torque_nm':effort['effort'][0],'right_commanded_torque_nm':effort['effort'][1]}
    if previous:row.update(actual_center_secant_rate_radps=(actual-previous[1])/((ns-previous[0])/1e9),interval_start_s=previous[0]/1e9)
    previous=ns,actual
    if ns-2_000_000 in dense:
        prior=dense[ns-2_000_000]
        for i,side in enumerate(('left','right')):
            error=row[side+'_target_rad']-prior[i][0];rate_error=row[side+'_target_rate_radps']-prior[i][1]
            prop=60*error;deriv=4*rate_error;pred=max(-20,min(20,prop+deriv))
            row.update({side+'_position_torque_nm':prop,side+'_velocity_torque_nm':deriv,
                        side+'_previous_step_velocity_radps':prior[i][1],side+'_replayed_torque_nm':pred,
                        side+'_torque_replay_error_nm':row[side+'_commanded_torque_nm']-pred})
    full.append(row)
    if 18.0<=row['t_s']<=19.3:rows.append(row)
columns=sorted(set().union(*(r.keys() for r in rows)))
with (OUT/'steering_window.csv').open('w') as f:
    writer=csv.DictWriter(f,fieldnames=columns);writer.writeheader();writer.writerows(rows)
dense_rows=[];previous=None
for ns,states in sorted(dense.items()):
    value=center(states[0][0],states[1][0]);row={'t_s':ns/1e9,'center_rad':value}
    if previous:row.update(interval_start_s=previous[0]/1e9,secant_rate_radps=(value-previous[1])/((ns-previous[0])/1e9))
    previous=ns,value;dense_rows.append(row)
valid=[r for r in full if 'actual_center_secant_rate_radps' in r]
peak=max(valid,key=lambda r:abs(r['actual_center_secant_rate_radps']));dpeak=max((r for r in dense_rows if 'secant_rate_radps'in r),key=lambda r:abs(r['secant_rate_radps']))
link50=[];previous=None
for packet in data['links']:
    ns=packet['ns'];states=dense_state(packet);value=center(states[0][0],states[1][0])
    if previous:link50.append({'t_s':ns/1e9,'interval_start_s':previous[0]/1e9,'center_rad':value,
        'secant_rate_radps':(value-previous[1])/((ns-previous[0])/1e9)})
    previous=ns,value
link50peak=max(link50,key=lambda r:abs(r['secant_rate_radps']))
report={'scope':'Exploratory diagnosis of preserved failed P01 only, not a revised outcome or gate','raw_path':str(RAW),'raw_sha256':hashlib.sha256(RAW.read_bytes()).hexdigest(),
 'start_s':data['schedule_start_s'],'physical_steering_kp':60,'physical_steering_kd':4,'physical_torque_limit_nm':20,
 'frozen_center_target_slew_limit_radps':.75,'frozen_actual_center_slew_gate_radps':.85,
 'peak50Hz_joint_state_grid':peak,'peak50Hz_world_link_grid':link50peak,'peak500Hz_pose_secant':dpeak,
 'count50Hz_joint_grid_above_gate':sum(abs(r['actual_center_secant_rate_radps'])>.85+1e-6 for r in valid),
 'count50Hz_world_link_grid_above_gate':sum(abs(r['secant_rate_radps'])>.85+1e-6 for r in link50),
 'commands_at_zero_yaw_transition':[r for r in data['commands_sent'] if 18.05<=r['t']<=18.25],
 'max_abs_target_center_rate_radps':max(abs(r['target_center_rate_radps']) for r in full),
 'max_abs_actual_joint_velocity_derived_center_rate_radps':max(abs(r['actual_center_rate_from_joint_velocity_radps']) for r in full),
 'max_abs_commanded_steering_torque_nm':max(abs(r[k]) for r in full for k in ('left_commanded_torque_nm','right_commanded_torque_nm')),
 'window_previous_dense_torque_replay_max_error_nm':max(abs(r.get(side+'_torque_replay_error_nm',0)) for r in rows for side in ('left','right')),
 'window_previous_dense_torque_replay_mean_abs_error_nm':sum(abs(r.get(side+'_torque_replay_error_nm',0)) for r in rows for side in ('left','right'))/(2*len(rows)),
 'notes':['50Hz joints and50Hz world links have2ms phase offset; their20ms secant maxima/counts differ. Original gate uses the world-link grid; joint grid is additional diagnosis, not a substitute reassessment.','Commanded torque is prior Begin control, paired here with previous2ms dense End state and current target. Replay error diagnoses whether this timestamp assumption holds; do not assume exactness.','Dense Link angular velocities are post-step measured velocities, not certified within-step derivative bounds.','No ROS, new simulation, gate modification or P01/P02 controller modification performed.']}
(OUT/'mechanism_report.json').write_text(json.dumps(report,indent=2)+'\n')
try:
    import matplotlib;matplotlib.use('Agg');import matplotlib.pyplot as plt
    fig,axs=plt.subplots(4,1,figsize=(9,9),sharex=True,constrained_layout=True)
    t=[r['t_s'] for r in rows]
    axs[0].plot(t,[r['actual_center_rad'] for r in rows],label='Actual center');axs[0].plot(t,[r['target_center_rad'] for r in rows],label='Target');axs[0].set_ylabel('Steering [rad]');axs[0].legend()
    window=[r for r in dense_rows if 18.0<=r['t_s']<=19.3 and 'secant_rate_radps'in r]
    axs[1].plot([r['t_s'] for r in window],[r['secant_rate_radps'] for r in window],label='2 ms pose secant',alpha=.6)
    axs[1].plot(t,[r.get('actual_center_secant_rate_radps',float('nan')) for r in rows],label='20 ms joint-state secant')
    axs[1].plot(t,[r['target_center_rate_radps'] for r in rows],label='Target rate');axs[1].axhline(.85,color='r',linestyle='--',label='Frozen actual gate');axs[1].axhline(-.85,color='r',linestyle='--');axs[1].set_ylabel('Center rate [rad/s]');axs[1].legend(loc='upper right')
    for side in ('left','right'):
        axs[2].plot(t,[r[side+'_target_rad']-r[side+'_actual_rad'] for r in rows],label=side+' target−actual')
        axs[3].plot(t,[r[side+'_commanded_torque_nm'] for r in rows],label=side+' commanded torque')
    axs[2].set_ylabel('Angle error [rad]');axs[2].legend();axs[3].set_ylabel('Torque [Nm]');axs[3].legend();axs[3].set_xlabel('Simulation source time [s]')
    for ax in axs:ax.grid(alpha=.2);ax.axvline(18.12,color='gray',linestyle=':')
    fig.suptitle('P01 preserved data: steering return during zero-speed yaw request\nExploratory mechanism diagnosis; original failed outcome retained')
    fig.savefig(OUT/'steering_return_diagnosis.svg');fig.savefig(OUT/'steering_return_diagnosis.png',dpi=150);plt.close(fig)
except ImportError:pass
print(json.dumps(report,indent=2))
