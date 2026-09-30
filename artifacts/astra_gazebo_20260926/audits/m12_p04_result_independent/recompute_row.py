#!/usr/bin/env python3
"""Independent source-time P04 diagnosis. Pure rosbag reading; never ROS init."""
from collections import Counter,defaultdict
import csv
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import traceback
import numpy as np
import rosbag

S=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser();parser.add_argument('--run-id',required=True);args=parser.parse_args()
assert args.run_id.startswith('m12_physics_p04_') and '/' not in args.run_id
HERE=Path(__file__).resolve().parent/'rows'/args.run_id;HERE.mkdir(parents=True,exist_ok=True)
RUN=S/'runs'/args.run_id

def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def stat(a):
    a=np.array(a,dtype=float)
    if len(a)==0:return {'count':0}
    return dict(count=len(a),min=float(a.min()),max=float(a.max()),mean=float(a.mean()),p95=float(np.percentile(a,95)),max_abs=float(abs(a).max()))
def angle(q):
    x,y,z,w=q
    return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def rel(q,r):
    a,b,c,d=-q[0],-q[1],-q[2],q[3];e,f,g,h=r
    return (d*e+a*h+b*g-c*f,d*f-a*g+b*h+c*e,d*g+a*f-b*e+c*h,d*h-a*e-b*f-c*g)
def rotation(q):
    x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def center(a,L,track):
    tl,tr=np.tan(a[:,0]),np.tan(a[:,1]);kl=tl/(L+track/2*tl);kr=tr/(L-track/2*tr)
    return np.arctan(L*(kl+kr)/2),abs(kl-kr)
def csvwrite(path,rows):
    with path.open('w') as f:
        if not rows:return
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def pp(p):return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
def twist(v):return [v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z]
def bodystate(msg):return {'names':list(msg.name),'poses':[pp(p) for p in msg.pose],'twists':[twist(v) for v in msg.twist]}

def main():
    raw=json.loads((RUN/'probe_raw.json').read_text());receipt=json.loads((RUN/'receipt.json').read_text());report=json.loads((RUN/'probe_report.json').read_text())
    assert receipt['status'] in ('completed','infrastructure_failure')
    assert (RUN/'cleanup.json').exists(), 'wait for parent terminal cleanup before reading bag'
    assert sha(RUN/'probe_raw.json')==report['raw_sha256']==receipt['probe_raw.json_sha256']
    c=raw['contract'];par=c['effective_plugin_parameters'];g=c['gates'];start=raw['schedule_start_s'];end=start+c['total_schedule_s'];start_ns=round(start*1e9);end_ns=round(end*1e9)
    # Preserve assessor's exact inclusive floating bounds for 50Hz comparisons;
    # independently report integer-ns schedule coverage as a sensitivity below.
    inside=lambda r:start<=r['t']<=end
    window={k:[r for r in raw[k] if inside(r)] for k in ['models','links','dense_models','dense_links','odom','joints','targets','contacts','commanded_efforts','backend_fault','backend_status']}
    L=par['wheelbase'];track=par['track'];robot=c['model_name']
    source={}
    for key,rows in window.items():
        if key=='backend_fault':continue
        ns=np.array([r['ns'] for r in rows],dtype=np.int64);dt=np.diff(ns)
        source[key]={'count':len(rows),'first_ns':int(ns[0]),'last_ns':int(ns[-1]),'gap_ns':stat(dt),'duplicate_count':int(np.sum(dt==0)),'regression_count':int(np.sum(dt<0)),
                     'integer_ns_window_count':sum(start_ns<=r['ns']<=end_ns for r in raw[key]),'seconds_ns_max_error_s':max(abs(r['t']-r['ns']/1e9) for r in rows)}
    dense_by_ns={r['ns']:r for r in raw['dense_links']};dense_models_by_ns={r['ns']:r for r in raw['dense_models']}
    exact={}
    for low,lookup in [('links',dense_by_ns),('models',dense_models_by_ns)]:
        exact[low]={'count':len(raw[low]),'exact':sum(r['ns'] in lookup and all(r[k]==lookup[r['ns']][k] for k in ['names','poses','twists']) for r in raw[low])}
    steering={};csv_rows=[];all_arrays={}
    for key in ['links','dense_links']:
        rows=window[key];times=np.array([r['ns']/1e9 for r in rows]);aa=[];twists=[];omega=[]
        for r in rows:
            p=dict(zip(r['names'],r['poses']));v=dict(zip(r['names'],r['twists']));base=p[robot+'::base_link'];angles=[];zt=[];om=[]
            for side in ['left','right']:
                name=robot+'::front_'+side+'_steer_link';q=rel(base[3:],p[name][3:]);angles.append(angle(q));zt.append(2*math.atan2(q[2],q[3]));om.append((rotation(base[3:]).T@(np.array(v[name][3:])-np.array(v[robot+'::base_link'][3:])))[2])
            aa.append(angles);twists.append(zt);omega.append(om)
        aa=np.array(aa);cc,ke=center(aa,L,track);zcc,_=center(np.array(twists),L,track);rates=np.diff(cc)/np.diff(times);zr=np.diff(zcc)/np.diff(times);worst=int(np.argmax(abs(rates)))
        steering[key]={'source_samples':len(rows),'angles_left':stat(aa[:,0]),'angles_right':stat(aa[:,1]),'center_rad':stat(cc),'center_rate_radps':stat(rates),'curvature_pair_disagreement_inv_m':stat(ke),
          'fixed_0_85_rate_exceedances':int(np.sum(abs(rates)>.85)),'maximum_z_twist_vs_yaw_rate_difference_radps':float(max(abs(zr-rates))),
          'peak_interval':{'a_ns':int(rows[worst]['ns']),'b_ns':int(rows[worst+1]['ns']),'a_center_rad':float(cc[worst]),'b_center_rad':float(cc[worst+1]),'rate_radps':float(rates[worst]),'elapsed_mid_s':float((times[worst]+times[worst+1])/2-start)},
          'relative_steer_omega_z_radps':{'left':stat(np.array(omega)[:,0]),'right':stat(np.array(omega)[:,1])},
          'interpretation':'Original frozen 50Hz gate' if key=='links' else 'Supplementary 500Hz evidence; not a substituted qualification gate'}
        all_arrays[key]=(times,aa,cc,rates)
        for i,r in enumerate(rows):
            csv_rows.append({'grid':key,'source_ns':r['ns'],'elapsed_s':r['ns']/1e9-start,'left_rad':aa[i,0],'right_rad':aa[i,1],'center_rad':cc[i],'ending_interval_rate_radps':rates[i-1] if i else '', 'curvature_pair_disagreement_inv_m':ke[i]})
    csvwrite(HERE/'source_steering.csv',csv_rows)
    odom=window['odom'];targets=window['targets'];tt=np.array([r['ns']/1e9 for r in targets]);tv=np.array([r['positions']['speed'] for r in targets]);tc=np.array([r['positions']['center_steering'] for r in targets]);ot=np.array([r['ns']/1e9 for r in odom]);ov=np.array([r['v'] for r in odom]);ovy=np.array([r['vy'] for r in odom]);ow=np.array([r['w'] for r in odom])
    av=np.diff(ov)/np.diff(ot);jerk=np.diff(av)/np.diff((ot[1:]+ot[:-1])/2)
    actual_errors=[];joint_errors=[];target_center_errors=[];motion_rows=[]
    for r in odom:
        packet=dense_by_ns[r['ns']];i=packet['names'].index(robot+'::base_link');p=packet['poses'][i];v=packet['twists'][i];R=rotation(p[3:]);lin=R.T@np.array(v[:3]);ang=R.T@np.array(v[3:]);actual_errors.append(max(abs(r['v']-lin[0]),abs(r['vy']-lin[1]),abs(r['w']-ang[2])))
    dj={r['ns']:r for r in raw['joints']};dtar={r['ns']:r for r in targets}
    for r in window['dense_links']:
        if r['ns'] not in dj:continue
        bodies=dict(zip(r['names'],r['poses']));base=bodies[robot+'::base_link'];actual=np.array([[angle(rel(base[3:],bodies[robot+'::'+link][3:])) for link in c['front_link_names']]])
        cc,_=center(actual,L,track)
        for name,v in zip(c['front_joint_names'],actual[0]):joint_errors.append(abs(dj[r['ns']]['positions'][name]-v))
        if r['ns'] in dtar:target_center_errors.append(cc[0]-dtar[r['ns']]['positions']['center_steering'])
    for r in odom:
        tr=dtar[r['ns']];motion_rows.append({'source_ns':r['ns'],'elapsed_s':r['t']-start,'actual_v_mps':r['v'],'actual_vy_mps':r['vy'],'actual_yaw_rate_radps':r['w'],'target_speed_mps':tr['positions']['speed'],'target_center_rad':tr['positions']['center_steering']})
    csvwrite(HERE/'source_target_actual.csv',motion_rows)
    motion={'actual_odom_speed_mps':stat(ov),'actual_odom_lateral_speed_mps':stat(ovy),'actual_odom_acceleration_mps2':stat(av),'actual_odom_jerk_mps3':stat(jerk),
            'target_speed_acceleration_mps2':stat(np.diff(tv)/np.diff(tt)),'target_center_rate_radps':stat(np.diff(tc)/np.diff(tt)),
            'same_source_odom_vs_dense_body_velocity_max_error':max(actual_errors),'same_source_joint_vs_relative_quaternion_max_error_rad':max(joint_errors),
            'actual_minus_target_speed_mps':stat(ov-tv),'actual_minus_target_center_rad':stat(target_center_errors),'nonzero_actual_target_speed_difference_count':int(np.sum(abs(ov-tv)>1e-6)),
            'actual_acceleration_and_jerk_gate':None,'interpretation':'Actual body velocity and finite-difference dynamics; reference .6/.9 limits do not guarantee physical .6/.9 compliance.'}
    # Model-pose secants provide a second observed physical motion channel.
    pose_metrics={}
    for key in ['models','dense_models']:
        rows=window[key];times=np.array([r['ns']/1e9 for r in rows]);poses=np.array([r['poses'][r['names'].index(robot)] for r in rows]);xyv=np.diff(poses[:,:2],axis=0)/np.diff(times)[:,None];head=np.unwrap([angle(p[3:]) for p in poses]);yr=np.diff(head)/np.diff(times)
        csvwrite(HERE/(key+'_model_trajectory.csv'),[{'source_ns':r['ns'],'elapsed_s':r['ns']/1e9-start,'x_m':p[0],'y_m':p[1],'z_m':p[2],'yaw_rad':angle(p[3:])} for r,p in zip(rows,poses)])
        pose_metrics[key]={'unsigned_planar_secant_speed_mps':stat(np.linalg.norm(xyv,axis=1)),'yaw_secant_rate_radps':stat(yr),'x_y_z_range_m':[[float(v.min()),float(v.max())] for v in poses[:,:3].T]}
    phases=[];at=0
    lowt,_,lowc,_=all_arrays['links'];model=window['models'];mt=np.array([r['ns']/1e9 for r in model]);mp=np.array([r['poses'][r['names'].index(robot)] for r in model])
    for phase in c['phase_schedule']:
        lo=start+at;at+=phase['duration_s'];hi=start+at;mask=(ot>=hi-c['settled_tail_s'])&(ot<hi);sm=(lowt>=hi-c['settled_tail_s'])&(lowt<hi)
        item={'name':phase['name'],'start_s':lo,'end_s':hi,'tail_speed_mps':stat(ov[mask]),'tail_center_rad':stat(lowc[sm])}
        if phase.get('speed_mps',0)==0:
            cm=(ot>=lo)&(ot<hi);ix=np.flatnonzero(cm);stopped=[int(i) for i in ix if np.all(abs(ov[i:ix[-1]+1])<=g['stopped_tail_speed_mps'])]
            item['first_persistent_under_0_02_elapsed_s']=float(ot[stopped[0]]-lo) if stopped else None
            if stopped:
                path=(mt>=lo)&(mt<=ot[stopped[0]]);item['path_until_persistent_stop_m']=float(np.linalg.norm(np.diff(mp[path,:2],axis=0),axis=1).sum())
        phases.append(item)
    csvwrite(HERE/'phase_tail_summary.csv',[{'phase':p['name'],'start_s':p['start_s'],'end_s':p['end_s'],'tail_speed_min_mps':p['tail_speed_mps']['min'],'tail_speed_max_mps':p['tail_speed_mps']['max'],'tail_center_min_rad':p['tail_center_rad']['min'],'tail_center_max_rad':p['tail_center_rad']['max'],'first_persistent_stop_elapsed_s':p.get('first_persistent_under_0_02_elapsed_s','')} for p in phases])
    contact_pairs=Counter();illegal=[];nonground=[];support={robot+'::'+n+'_wheel_link' for n in ['front_left','front_right','rear_left','rear_right']}
    for r in window['contacts']:
        for p in r['pairs']:
            names=[p['a'],p['b']];mods=[v.split('::')[0] for v in names]
            if robot not in mods:continue
            contact_pairs[tuple(sorted(names))]+=1
            if c['world_fixture']['ground_model'] in mods:
                rob=names[mods.index(robot)]
                if '::'.join(rob.split('::')[:2]) not in support:illegal.append({'ns':r['ns'],**p})
            else:nonground.append({'ns':r['ns'],**p})
    gaps=[]
    for i,(a,b) in enumerate(zip(raw['backend_fault'],raw['backend_fault'][1:])):
        if b['t']-a['t']>.06:gaps.append({'index':i,'before_s':a['t'],'after_s':b['t'],'gap_s':b['t']-a['t'],'following_receipt_times_s':[x['t'] for x in raw['backend_fault'][i+1:i+7]]})
    command_gaps=[]
    for a,b in zip(raw['commands_sent'],raw['commands_sent'][1:]):
        if a['phase']==b['phase'] and b['t']-a['t']>g['maximum_publish_gap_s']:
            command_gaps.append({'phase':a['phase'],'before_s':a['t'],'after_s':b['t'],'gap_sim_s':b['t']-a['t'],'gap_wall_s':b['wall_monotonic_s']-a['wall_monotonic_s']})
    loss_start=start+sum(p['duration_s'] for p in c['phase_schedule'][:-1]);boundary=[r for r in raw['commands_sent'] if loss_start<=r['t']<end]
    # Independent closed rosbag cross-check, retaining startup/tail gaps.
    topic_keys={'/astra/world_models':'models','/astra/world_links':'links','/astra/physics_models':'dense_models','/astra/physics_links':'dense_links','/odom':'odom','/joint_states':'joints','/m2_driver/actuator_target':'targets','/m2_driver/actuator_commanded_effort':'commanded_efforts','/astra/physics_contacts':'contacts','/m2_driver/physical_status':'backend_status'}
    indexes={k:{r['ns']:r for r in raw[k]} for k in topic_keys.values()};bagrows={t:[] for t in topic_keys};bagsame=Counter();bagmismatch=defaultdict(list);faultbag=[];commandsbag=defaultdict(list);topic_counts=Counter();pubs=defaultdict(set)
    with rosbag.Bag(str(RUN/'fixture.bag'),'r') as bag:
        for topic,msg,bt in bag.read_messages():
            topic_counts[topic]+=1
            if getattr(msg,'_connection_header',None):pubs[topic].add(msg._connection_header.get('callerid',''))
            if topic=='/m2_driver/physical_backend_fault':faultbag.append({'receipt_s':bt.to_sec(),'value':msg.data})
            if topic in ['/cmd_vel_navigation','/cmd_vel']:commandsbag[topic].append({'receipt_s':bt.to_sec(),'v':msg.linear.x,'w':msg.angular.z})
            if topic not in topic_keys:continue
            ns=msg.header.stamp.to_nsec();bagrows[topic].append(ns);key=topic_keys[topic];r=indexes[key].get(ns)
            if r is None:continue
            if key in ['models','links','dense_models','dense_links']:val=bodystate(msg);same=all(r[k]==v for k,v in val.items())
            elif key=='odom':same=r['p']==pp(msg.pose.pose) and r['v']==msg.twist.twist.linear.x and r['vy']==msg.twist.twist.linear.y and r['w']==msg.twist.twist.angular.z
            elif key=='joints':same=r['positions']==dict(zip(msg.name,msg.position)) and r['velocities']==dict(zip(msg.name,msg.velocity))
            elif key=='targets':same=r['positions']==dict(zip(msg.name,msg.position)) and r['velocities']==dict(zip(msg.name,msg.velocity)) and r['names']==list(msg.name)
            elif key=='commanded_efforts':same=r['names']==list(msg.name) and r['effort']==list(msg.effort)
            elif key=='backend_status':same=r['statuses']==[{'name':x.name,'hardware_id':x.hardware_id,'level':int(x.level),'message':x.message,'values':{v.key:v.value for v in x.values},'value_count':len(x.values)} for x in msg.status]
            elif key=='contacts':same=[(p['a'],p['b'],p['depths'],p['positions'],p['normals']) for p in r['pairs']]==[(p.collision1_name,p.collision2_name,list(p.depths),[[v.x,v.y,v.z] for v in p.contact_positions],[[v.x,v.y,v.z] for v in p.contact_normals]) for p in msg.states]
            same=same and r.get('seq')==msg.header.seq and r.get('frame')==msg.header.frame_id and r.get('t')==msg.header.stamp.to_sec()
            if key=='contacts':same=same and [p['source_info'] for p in r['pairs']]==[p.info for p in msg.states]
            if same:bagsame[topic]+=1
            else:bagmismatch[topic].append(ns)
    bagstats={}
    for topic,key in topic_keys.items():
        bset=set(bagrows[topic]);rset=set(indexes[key]);shared=rset&bset;missing=sorted(rset-bset);extra=sorted(bset-rset)
        bagstats[topic]={'bag_count':len(bagrows[topic]),'raw_count':len(raw[key]),'shared':len(shared),'shared_exact_payload':bagsame[topic],'payload_mismatch_ns':bagmismatch[topic],'raw_only_ns':missing,'bag_only_ns':extra,
          'bag_source_first_ns':bagrows[topic][0],'bag_source_last_ns':bagrows[topic][-1],'bag_source_gaps_ns':stat(np.diff(bagrows[topic])),'ordered_raw_bag_source_ns_exact':[r['ns'] for r in raw[key]]==bagrows[topic],
          'formal_window_raw_only_ns':[ns for ns in missing if start_ns<=ns<=end_ns],'formal_window_bag_only_ns':[ns for ns in extra if start_ns<=ns<=end_ns]}
    fb_gaps=np.diff([r['receipt_s'] for r in faultbag]);fbad=[{'before_s':a['receipt_s'],'after_s':b['receipt_s'],'gap_s':b['receipt_s']-a['receipt_s']} for a,b in zip(faultbag,faultbag[1:]) if b['receipt_s']-a['receipt_s']>.06]
    body_grid_checks={'dense_models_exact_2ms':all(b['ns']-a['ns']==2000000 for a,b in zip(window['dense_models'],window['dense_models'][1:])),
                      'dense_links_exact_2ms':all(b['ns']-a['ns']==2000000 for a,b in zip(window['dense_links'],window['dense_links'][1:])),
                      'dense_model_link_grid_equal':[r['ns'] for r in window['dense_models']]==[r['ns'] for r in window['dense_links']]}
    continuation={'diagnostic_failed_checks':[k for k,v in report.get('checks',{}).items() if not v]}
    out={'status':'independent_offline_P04_row_raw_diagnosis','canonical_result_unchanged':{'pass':report['pass'],'assessment_incomplete':report.get('assessment_incomplete',False),'reason':report.get('reason')},
         'run_id':raw['run_id'],'seed':raw['seed'],'schedule_start_s':start,'schedule_start_ns_rounded':start_ns,'schedule_end_s':end,
         'original_final_drain':raw.get('final_drain'),'source_streams':source,'dense_grid_checks':body_grid_checks,'exact_low_dense_payload':exact,'steering':steering,'motion':motion,'pose_derivatives':pose_metrics,'phase_metrics':phases,
         'contact_counts':{'pairs':[{'names':list(k),'count':v} for k,v in contact_pairs.items()],'nonwheel_ground':len(illegal),'nonground':len(nonground)},
         'fault_timing':{'basis':'probe receipt ROS clock; Bool has no source header or sequence','raw_count':len(raw['backend_fault']),'raw_any_true':any(r['value'] for r in raw['backend_fault']),'raw_gap_over_0_06_s':gaps,'raw_duplicate_receipt_count':sum(b['t']==a['t'] for a,b in zip(raw['backend_fault'],raw['backend_fault'][1:])),
                         'bag_count':len(faultbag),'bag_any_true':any(r['value'] for r in faultbag),'bag_receipt_gap_s':stat(fb_gaps),'bag_gap_over_0_06_s':fbad,'bag_first_receipt_s':faultbag[0]['receipt_s'],
                         'inference':'Probe Bool receipt gaps followed by timestamp bunching coexist with the separate bag receiver at20ms. This constrains the legacy receipt-time issue to collector observation/scheduling; P02 isolated command timing is audited separately and has no inherited P01 main-loop-delay inference. Bool lacks identity so exact per-message source-time completeness cannot be proved.'},
         'command_timing':{'same_phase_gaps_over_frozen_limit':command_gaps,'loss_start_s':loss_start,'publications_in_assessor_loss_interval':boundary,'boundary_float_elapsed':[r['t']-start for r in boundary],'runtime_boundary_rule':'integer ns elapsed<32000000000','assessor_boundary_rule':'schedule_start_ns+32000000000<=event.ns','integer_ns_boundary_rule_result':[round(r['t']*1e9)-start_ns for r in boundary],
                           'mechanism':'P02 uses integer-nanosecond phase boundaries. This list reports any publication actually present in the frozen assessor loss interval; an empty list is required.'},
         'rosbag_exact_payload_crosschecks':bagstats,'bag_topic_counts':dict(topic_counts),'bag_publishers':{k:sorted(v) for k,v in pubs.items()},
         'bag_command_receipts_near_loss':{k:[r for r in v if loss_start-.06<=r['receipt_s']<=loss_start+.15] for k,v in commandsbag.items()},
         'frozen_report_failed_checks':continuation['diagnostic_failed_checks'],
         'limits':['No ROS initialized and no simulation started.','Original raw/report/receipt unchanged; no qualification pass inferred from this diagnostic.','No gate is relaxed. Dense500Hz rates supplement the frozen50Hz gate; P01 remains separate and unchanged.','Actual acceleration/jerk have no preregistered physical compliance gate.','Contact normals/depths retained; no force or measured reaction torque supplied.','Bool recorder/probe receipt clocks are not exact publication stamps.','Finite torque backend remains an uncalibrated simulation candidate, not navigation or vehicle evidence.'],
         'inputs':[{'path':str(p),'sha256':sha(p)} for p in [RUN/'probe_raw.json',RUN/'probe_report.json',RUN/'receipt.json',RUN/'fixture.bag',RUN/'bindings.json',RUN/'actuation_contract.yaml']],
         'script_sha256':sha(__file__),'command':'/usr/bin/python3 '+str(Path(__file__).resolve())+' --run-id '+args.run_id,'python_version':sys.version,'numpy_version':np.__version__}
    (HERE/'independent_raw_diagnosis.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'status':out['status'],'dense_grid':body_grid_checks,'fault':out['fault_timing'],'bag_mismatches':{k:v['payload_mismatch_ns'] for k,v in bagstats.items()},'sha256':sha(HERE/'independent_raw_diagnosis.json')},ensure_ascii=False,indent=2))

if __name__=='__main__':
    try:main()
    except BaseException:
        with (HERE/'invocation_errors.log').open('a') as f:f.write(traceback.format_exc()+'\n')
        raise
