#!/usr/bin/env python3
"""Read-only Q03 source-stamped quaternion diagnosis. No ROS node or simulator calls."""
from pathlib import Path
import json,hashlib,bisect,csv,math,sys,xml.etree.ElementTree as ET
import numpy as np
class Rotation:
    def __init__(self,q):
        self.q=np.asarray(q,dtype=float);self.q=self.q/np.linalg.norm(self.q)
    @classmethod
    def from_quat(cls,q):return cls(q)
    @classmethod
    def from_rotvec(cls,v):
        v=np.asarray(v,dtype=float);a=np.linalg.norm(v)
        return cls([0,0,0,1] if a==0 else [*(v/a*np.sin(a/2)),np.cos(a/2)])
    def inv(self):return Rotation([-self.q[0],-self.q[1],-self.q[2],self.q[3]])
    def __mul__(self,b):
        av=self.q[:3];bv=b.q[:3];aw=self.q[3];bw=b.q[3]
        return Rotation([*(aw*bv+bw*av+np.cross(av,bv)),aw*bw-np.dot(av,bv)])
    def as_quat(self):return self.q.copy()
    def apply(self,v):
        v=np.asarray(v);q=self.q[:3];w=self.q[3]
        return v+2*w*np.cross(q,v)+2*np.cross(q,np.cross(q,v))
    def magnitude(self):return 2*math.atan2(np.linalg.norm(self.q[:3]),abs(self.q[3]))
S=Path(__file__).resolve().parents[3];W=S/'workspace';RUN=S/'runs/actuation_q03';OUT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
raw=json.loads((RUN/'probe_raw.json').read_text());contract=raw['contract'];report=json.loads((RUN/'probe_report.json').read_text());start=raw['schedule_start_s'];end=start+contract['total_schedule_s'];L=contract['effective_plugin_parameters']['wheelbase'];track=contract['effective_plugin_parameters']['track']
links=[r for r in raw['links'] if start<=r['t']<=end];joints=raw['joints'];jns=[r['ns'] for r in joints]
def center(a,b):
 kl=np.tan(a)/(L+track/2*np.tan(a));kr=np.tan(b)/(L-track/2*np.tan(b));return np.arctan(L*(kl+kr)/2)
def relative(parent,child):return Rotation.from_quat(parent).inv()*Rotation.from_quat(child)
def heading(rot):
 x=rot.apply([1.,0.,0.]);return math.atan2(x[1],x[0])
def axleheading(rot):
 y=rot.apply([0.,1.,0.]);return math.atan2(-y[0],y[1])
rows=[]
for r in links:
 by=dict(zip(r['names'],r['poses']));tw=dict(zip(r['names'],r['twists']));b=by['autolabor_m2::base_link'];base_rot=Rotation.from_quat(b[3:]);ix=bisect.bisect_right(jns,r['ns'])-1;j=joints[ix];jp=j['positions']
 row={'source_ns':r['ns'],'source_s':r['t'],'elapsed_s':r['t']-start,'joint_source_ns':j['ns'],'joint_age_s':(r['ns']-j['ns'])*1e-9,'published_center_rad':float(center(jp['front_left_steer_joint'],jp['front_right_steer_joint'])),'base_tilt_rad':float(math.acos(np.clip(base_rot.apply([0.,0.,1.])[2],-1,1)))}
 for side in ('left','right'):
  name='autolabor_m2::front_'+side+'_steer_link';rot=relative(b[3:],by[name][3:]);q=rot.as_quat();twistz=2*math.atan2(q[2],q[3]);offaxis=(Rotation.from_rotvec([0,0,-twistz])*rot).magnitude()
  row[side+'_heading_rad']=heading(rot);row[side+'_twist_z_rad']=twistz;row[side+'_offaxis_rad']=float(offaxis);row[side+'_published_rad']=jp['front_'+side+'_steer_joint']
  omega=base_rot.inv().apply(np.array(tw[name][3:])-np.array(tw['autolabor_m2::base_link'][3:]));row[side+'_relative_body_omega_z_radps']=float(omega[2])
 for axle in ('front','rear'):
  for side in ('left','right'):
   name='autolabor_m2::'+axle+'_'+side+'_wheel_link';rot=relative(b[3:],by[name][3:]);axis=rot.apply([0.,1.,0.]);row[axle+'_'+side+'_wheel_axis_heading_rad']=axleheading(rot);row[axle+'_'+side+'_wheel_axis_tilt_rad']=math.asin(np.clip(axis[2],-1,1))
 for name,cols in [('actual_center_rad',('left_heading_rad','right_heading_rad')),('twist_center_rad',('left_twist_z_rad','right_twist_z_rad')),('wheel_axis_center_rad',('front_left_wheel_axis_heading_rad','front_right_wheel_axis_heading_rad'))]:row[name]=float(center(row[cols[0]],row[cols[1]]))
 row['actual_minus_published_rad']=row['actual_center_rad']-row['published_center_rad'];rows.append(row)
t=np.array([r['source_ns'] for r in rows],dtype=np.int64)*1e-9;dt=np.diff(np.array([r['source_ns'] for r in rows],dtype=np.int64))*1e-9
cols={k:np.array([r[k] for r in rows]) for k in rows[0]}
rates={k:np.diff(cols[k])/dt for k in ['actual_center_rad','twist_center_rad','wheel_axis_center_rad','published_center_rad','actual_minus_published_rad']}
i=int(np.argmax(np.abs(rates['actual_center_rad'])));gate=.75+contract['gates']['actual_center_slew_tolerance_radps']
peak={'from':rows[i],'to':rows[i+1],'duration_s':float(dt[i]),'rates_radps':{k:float(v[i]) for k,v in rates.items()},'gate_radps':gate,'excess_radps':float(abs(rates['actual_center_rad'][i])-gate),'interval_index':i}
summary={'source_samples':len(rows),'source_dt_unique_ns':sorted(set(int(b['source_ns']-a['source_ns']) for a,b in zip(rows,rows[1:]))),'joint_age_unique_ns':sorted(set(int(r['source_ns']-r['joint_source_ns']) for r in rows)),'gate_radps':gate,'violating_intervals':int(sum(abs(rates['actual_center_rad'])>gate)),'max_actual_rate_radps':float(max(abs(rates['actual_center_rad']))),'max_published_rate_radps':float(max(abs(rates['published_center_rad']))),'max_actual_twist_angle_difference_rad':float(max(abs(cols['actual_center_rad']-cols['twist_center_rad']))),'max_actual_twist_rate_difference_radps':float(max(abs(rates['actual_center_rad']-rates['twist_center_rad']))),'max_actual_wheel_axis_angle_difference_rad':float(max(abs(cols['actual_center_rad']-cols['wheel_axis_center_rad']))),'max_actual_wheel_axis_rate_difference_radps':float(max(abs(rates['actual_center_rad']-rates['wheel_axis_center_rad']))),'max_base_tilt_rad':float(max(abs(cols['base_tilt_rad']))),'max_steer_offaxis_rad':float(max(max(cols['left_offaxis_rad']),max(cols['right_offaxis_rad']))),'max_rear_wheel_axis_heading_rad':float(max(max(abs(cols['rear_left_wheel_axis_heading_rad'])),max(abs(cols['rear_right_wheel_axis_heading_rad'])))),'max_actual_minus_published_rad':float(max(abs(cols['actual_minus_published_rad']))),'peak':peak}
# Wider intervals are sensitivity diagnostics only, never replacements for the frozen 20ms gate.
summary['stride_sensitivity']={}
for stride in (1,2,5,10):
 v=(cols['actual_center_rad'][stride:]-cols['actual_center_rad'][:-stride])/(t[stride:]-t[:-stride]);summary['stride_sensitivity'][str(stride)]={'interval_s':stride*.02,'max_rate_radps':float(max(abs(v))),'over_frozen_gate':int(sum(abs(v)>gate))}
sourcepaths=[RUN/'probe_raw.json',RUN/'probe_raw.jsonl',RUN/'probe_report.json',RUN/'receipt.json',RUN/'fixture.bag',RUN/'actuation_contract.yaml',W/'src/simulation/m2_gazebo/src/m2_ackermann_plugin.cpp',W/'src/simulation/m2_gazebo/src/astra_steering_hold_plugin.cpp',W/'src/simulation/m2_gazebo/src/astra_evidence_plugin.cpp',W/'src/simulation/m2_gazebo/scripts/astra_actuation_probe_q02.py']
summary['sources']=[{'path':str(p),'sha256':sha(p)} for p in sourcepaths];summary['negative_result_preserved']={'report_pass':report['pass'],'failed_checks':[k for k,v in report['checks'].items() if not v]};summary['script_sha256']=sha(__file__)
for r in rows:r['violation_endpoint']=False
for k in np.flatnonzero(abs(rates['actual_center_rad'])>gate):rows[k+1]['violation_endpoint']=True
with (OUT/'source_time_steering.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
rate_rows=[dict(source_from_ns=rows[k]['source_ns'],source_to_ns=rows[k+1]['source_ns'],source_midpoint_s=(t[k]+t[k+1])/2,elapsed_midpoint_s=(t[k]+t[k+1])/2-start,**{n.replace('_rad','_rate_radps'):float(v[k]) for n,v in rates.items()}) for k in range(len(dt))]
with (OUT/'source_time_rates.csv').open('w') as f:w=csv.DictWriter(f,fieldnames=list(rate_rows[0]));w.writeheader();w.writerows(rate_rows)
(OUT/'numeric_diagnosis.json').write_text(json.dumps(summary,indent=2)+'\n');print(json.dumps(summary,indent=2))
