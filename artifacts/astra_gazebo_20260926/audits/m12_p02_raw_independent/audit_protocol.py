#!/usr/bin/env python3
"""Closed P02 bag/raw protocol replay, independent of live ROS services."""
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path
import types
import rosbag

D=Path(__file__).resolve().parent;S=D.parents[1];R=S/'runs/m12_physics_p02'
P=S/'workspace/src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p02.py'
def sha(p):
    h=hashlib.sha256()
    with Path(p).open('rb') as f:
        for b in iter(lambda:f.read(1048576),b''):h.update(b)
    return h.hexdigest()
def stats(v):return {'count':len(v),'minimum':min(v) if v else None,'maximum':max(v) if v else None}
def header(m):return {'ns':m.header.stamp.to_nsec(),'t':m.header.stamp.to_sec(),'frame':m.header.frame_id,'seq':m.header.seq}
def status(m):return dict(header(m),statuses=[{'name':x.name,'hardware_id':x.hardware_id,'level':int(x.level),'message':x.message,'values':{v.key:v.value for v in x.values},'value_count':len(x.values)} for x in m.status])
def event(m):
    x=m.status[0];v={p.key:p.value for p in x.values}
    return dict(header(m),phase=v['phase'],v=float(v['v']),w=float(v['w']),wall_monotonic_s=float(v['wall_monotonic_s']),scheduled_ns=int(v['scheduled_ns']),lateness_ns=int(v['lateness_ns']),missed_slots=int(v['missed_slots']))
def write_csv(p,rows):
    with p.open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)

raw=json.loads((R/'probe_raw.json').read_text());original=json.loads((R/'probe_report.json').read_text());receipt=json.loads((R/'receipt.json').read_text())
assert receipt['status']=='completed' and (R/'cleanup.json').is_file()
assert sha(R/'probe_raw.json')==original['raw_sha256']==receipt['probe_raw.json_sha256']
assert sha(P)==raw['script_sha256']=='68dd16ef4bf5a8ac85bafc8996e52c48f3c145c6a9dc88f490aa3e2d2fbfbe00'
probe=types.ModuleType('frozen_P02_replay');probe.__file__=str(P);exec(compile(P.read_bytes(),str(P),'exec'),vars(probe))
replay=probe.assess(raw,raw['contract']);assert all(original[k]==v for k,v in replay.items())
c=raw['contract'];start=raw['schedule_start_ns'];end=start+36000000000;loss=start+32000000000
events=raw['commands_sent'];formal=[r for r in events if start<=r['ns']<end];terminal=[r for r in events if r['ns']>=end]
status_map={'/m2_driver/physical_status':'backend_status','/astra_fixture/command_source_status':'source_status'}
bags={k:[] for k in status_map};bags['/astra_fixture/command_event']=[];markers=[];vel={'/cmd_vel_navigation':[],'/cmd_vel':[]};clock=[]
all_counts=Counter()
with rosbag.Bag(str(R/'fixture.bag'),'r') as b:
    for topic,msg,ts in b.read_messages():
        all_counts[topic]+=1
        if topic in status_map:bags[topic].append(status(msg))
        elif topic=='/astra_fixture/command_event':bags[topic].append(event(msg))
        elif topic=='/astra_fixture/schedule_start_ns':markers.append(msg.data)
        elif topic in vel:vel[topic].append({'receipt_ns':ts.to_nsec(),'v':msg.linear.x,'w':msg.angular.z})
        elif topic=='/clock':clock.append(msg.clock.to_nsec())
comparisons={}
for topic,key in list(status_map.items())+[('/astra_fixture/command_event','commands_sent')]:
    rr={(r['ns'],r['seq']):r for r in raw[key]};bb={(r['ns'],r['seq']):r for r in bags[topic]};shared=set(rr)&set(bb)
    raw_only=sorted(set(rr)-set(bb));bag_only=sorted(set(bb)-set(rr));bad=[k for k in sorted(shared) if rr[k]!=bb[k]]
    comparisons[topic]={'raw_count':len(raw[key]),'bag_count':len(bags[topic]),'shared':len(shared),'payload_mismatch_keys':bad,'raw_only_keys':raw_only,'bag_only_keys':bag_only,
                        'formal_raw_only_keys':[k for k in raw_only if start<=k[0]<=end],'formal_bag_only_keys':[k for k in bag_only if start<=k[0]<=end]}
ns=lambda key:[r['ns'] for r in raw[key] if start<=r['ns']<=end]
backend_ns=ns('backend_status');bg=[b-a for a,b in zip(backend_ns,backend_ns[1:])]
formal_clock=[v for v in clock if start<=v<=end]
clock_gaps=[b-a for a,b in zip(formal_clock,formal_clock[1:])]
gaps=[b['ns']-a['ns'] for a,b in zip(formal,formal[1:])];phase_gaps=[b['ns']-a['ns'] for a,b in zip(formal,formal[1:]) if a['phase']==b['phase']]
slot_ns=[r['scheduled_ns'] for r in formal];phases=[];elapsed=0;bad_phase=[]
for phase in c['phase_schedule']:
    lo=start+elapsed;elapsed+=round(phase['duration_s']*1e9);hi=start+elapsed;rows=[r for r in formal if lo<=r['ns']<hi]
    desired=None if phase.get('publish') is False else (phase['speed_mps'],phase['yaw_rate_request_radps'] if 'yaw_rate_request_radps' in phase else phase['speed_mps']*math.tan(phase['center_steering_request_rad'])/.65)
    for r in rows:
        if r['phase']!=phase['name'] or desired is None or abs(r['v']-desired[0])>1e-12 or abs(r['w']-desired[1])>1e-12:bad_phase.append(r)
    phases.append({'phase':phase['name'],'start_ns':lo,'end_ns':hi,'commands':len(rows),'nominal_commands':0 if desired is None else round(phase['duration_s']/.05)})
write_csv(D/'command_phase_counts.csv',phases)
write_csv(D/'formal_command_events.csv',formal)
bag_loss={k:[r for r in rows if loss<=r['receipt_ns']<end] for k,rows in vel.items()}
bag_loss_after_grace={k:[r for r in rows if loss+120000000<=r['receipt_ns']<end] for k,rows in vel.items()}
relay_formal=[r for r in vel['/cmd_vel'] if start<=r['receipt_ns']<loss]
relay_gaps=[b['receipt_ns']-a['receipt_ns'] for a,b in zip(relay_formal,relay_formal[1:])]
observed_loss=[r for r in raw['commands_observed'] if loss+120000000<=round(r['t']*1e9)<end]
states=[r['statuses'][0]['values'] for r in raw['source_status']]
barrier=raw['recorder_connection_barrier'];topics=raw['recording_topics']
body=json.loads((D/'independent_raw_diagnosis.json').read_text())
checks={
 'original_frozen_report_replay_exact':True,
 'source_status_formal_50Hz_exact_ns':bool(bg) and set(bg)=={20000000},
 'source_status_same_formal_ns_as_actual_odom_and_target':backend_ns==ns('odom')==ns('targets'),
 'source_status_all_no_backend_fault':all(r['statuses'][0]['values']['backend_fault']=='false' for r in raw['backend_status']),
 'status_and_event_shared_bag_payload_exact':not any(v['payload_mismatch_keys'] for v in comparisons.values()),
 'status_and_event_bag_covers_formal_window':not any(v['formal_raw_only_keys'] or v['formal_bag_only_keys'] for v in comparisons.values()),
 'all_ten_body_topic_formal_bag_coverage':all(not v['formal_window_raw_only_ns'] and not v['formal_window_bag_only_ns'] and not v['payload_mismatch_ns'] for v in body['rosbag_exact_payload_crosschecks'].values()),
 'every_formal_clock_step_2ms':bool(clock_gaps) and set(clock_gaps)=={2000000},
 'clock_covers_inclusive_formal_start_and_end':formal_clock[0]==start and formal_clock[-1]==end,
 'commands_strictly_increasing_source_ns':all(x>0 for x in gaps),
 'commands_at_most_one_per_slot':len(slot_ns)==len(set(slot_ns)),
 'commands_slots_and_source_time_consistent':all(r['scheduled_ns']==start+(r['ns']-start)//50000000*50000000 and r['lateness_ns']==r['ns']-r['scheduled_ns'] for r in formal),
 'all_commands_match_integer_phase_and_values':not bad_phase,
 'no_source_command_in_integer_loss':not any(loss<=r['ns']<end for r in events),
 'no_raw_relay_after_loss_drain_grace':not observed_loss,
 'no_bag_navigation_or_relay_after_loss_drain_grace':not any(bag_loss_after_grace.values()),
 'max_same_phase_command_gap_within_frozen75ms':max(phase_gaps)<=75000000,
 'exactly_one_safe_terminal_zero_at_or_after_end':len(terminal)==1 and terminal[0]['phase']=='terminal_zero' and terminal[0]['v']==terminal[0]['w']==0,
 'marker_exact_trigger_identity':markers==[start] and all(r['data']==start for r in raw['schedule_start_records']),
 'source_full_status_sequence': [r['state'] for r in states]==['bootstrap','running','complete'],
 'recorder_all26_actual_connections_before_unpause':len(topics)==26 and sorted(topics)==barrier['required'] and set(topics)<=set(barrier['connected']) and not barrier['missing'],
 'recorder_all26_topics_present_in_closed_bag':set(topics)<=set(all_counts),
 'warmup_precedes_running_and_schedule':raw['warmup_end_ns']-raw['warmup_start_ns']>=1000000000 and raw['warmup_end_ns']<=raw['source_status'][1]['ns']<start,
 'frozen_physical_failure_preserved':replay['pass'] is False and not replay['assessment_incomplete'] and [k for k,v in replay['checks'].items() if not v]==['actual_center_slew_with_declared_sampling_tolerance'],
}
out={'status':'independent_measurement_protocol_checks_pass_physical_qualification_failed' if all(checks.values()) else 'independent_protocol_review_has_failures',
     'checks':checks,'count':len(checks),'passed':sum(checks.values()),'run_id':raw['run_id'],'seed':raw['seed'],'formal_start_ns':start,'formal_end_ns':end,'loss_start_ns':loss,
     'frozen_assessor_checks':len(replay['checks']),'frozen_assessor_pass_count':sum(replay['checks'].values()),'frozen_failed_checks':[k for k,v in replay['checks'].items() if not v],
     'phase_counts':phases,'formal_source_command_count':len(formal),'maximum_missed_slots_field':max(r['missed_slots'] for r in formal),'source_command_gaps_ns':stats(gaps),'same_phase_source_command_gaps_ns':stats(phase_gaps),'relay_bag_gaps_ns':stats(relay_gaps),'event_lateness_ns':stats([r['lateness_ns'] for r in formal]),
     'source_status_formal_count':len(backend_ns),'source_status_formal_gaps_ns':stats(bg),'formal_clock_count':len(formal_clock),'formal_clock_gaps_ns':stats(clock_gaps),
     'status_event_bag_crosschecks':comparisons,'source_status_history':raw['source_status'],'trigger_response':raw['command_source_start_response'],'recorder_barrier':barrier,'warmup':{'start_ns':raw['warmup_start_ns'],'end_ns':raw['warmup_end_ns']},
     'bag_loss_messages':bag_loss,'bag_loss_after_drain':bag_loss_after_grace,'terminal_command':terminal,
     'raw_source_physics_time_basis':'backend_status/model/link/odom/target have explicit source headers; command event stamps are sampled just before Python publish; Bool and relay observations remain receiver-clock quantities.',
     'measurement_subset_scope':'Post-run independent measurement/protocol evidence supports evaluating a fresh controller candidate with these instruments. It is not an overall P02 qualification pass and is not a navigation, safety, or real-vehicle result.',
     'inputs':[{'path':str(p),'sha256':sha(p)} for p in [P,R/'probe_raw.json',R/'probe_report.json',R/'receipt.json',R/'fixture.bag',R/'bindings.json',D/'independent_raw_diagnosis.json']],
     'command':'/usr/bin/python3 '+str(Path(__file__).resolve()),'script_sha256':sha(__file__),'no_ROS_started':True,'no_run_files_changed':True}
(D/'protocol_audit.json').write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')
print(json.dumps({'status':out['status'],'passed':out['passed'],'count':out['count'],'failed':[k for k,v in checks.items() if not v],'commands':len(formal),'command_gap_ns':out['source_command_gaps_ns'],'backend_status_count':len(backend_ns),'report_sha256':sha(D/'protocol_audit.json')},indent=2))
