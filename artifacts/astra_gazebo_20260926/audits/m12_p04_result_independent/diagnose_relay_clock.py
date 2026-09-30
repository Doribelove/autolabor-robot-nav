#!/usr/bin/env python3
"""Offline P04 callback-clock attribution and all-phase membership audit."""
import csv,hashlib,json
from pathlib import Path
import rosbag
D=Path(__file__).resolve().parent;S=D.parents[1];R=S/'runs/m12_physics_p04_s012_kd4';raw=json.loads((R/'probe_raw.json').read_text());c=raw['contract'];start=raw['schedule_start_s'];start_ns=raw['schedule_start_ns'];end=start+c['total_schedule_s'];end_ns=start_ns+round(c['total_schedule_s']*1e9);loss=start+32;loss_ns=start_ns+32000000000;grace=c['gates']['command_loss_relay_drain_grace_s'];grace_ns=round(grace*1e9)
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
rows=[];bagcmd={'/cmd_vel':[],'/cmd_vel_navigation':[]};event=[]
with rosbag.Bag(str(R/'fixture.bag'),'r') as bag:
 for topic,m,bt in bag.read_messages(topics=['/cmd_vel','/cmd_vel_navigation','/astra_fixture/command_event','/m2_driver/physical_status','/clock']):
  row={'topic':topic,'bag_receipt_ns':bt.to_nsec()}
  if topic in bagcmd:
   row.update(v=m.linear.x,w=m.angular.z);bagcmd[topic].append(row.copy())
  elif topic=='/clock':row['source_ns']=m.clock.to_nsec()
  else:
   row.update(source_ns=m.header.stamp.to_nsec(),values={v.key:v.value for v in m.status[0].values})
   if topic=='/astra_fixture/command_event':event.append(row.copy())
  if loss_ns-100000000<=bt.to_nsec()<=loss_ns+600000000 or end_ns-100000000<=bt.to_nsec()<=end_ns+100000000:rows.append(row)
floatbad=[r for r in raw['commands_observed'] if loss+grace<=r['t']<end];intbad=[r for r in raw['commands_observed'] if loss_ns+grace_ns<=round(r['t']*1e9)<end_ns];sourcebad=[r for r in raw['commands_sent'] if loss_ns<=r['ns']<end_ns];terminal=[r for r in raw['commands_sent'] if r['phase']=='terminal_zero'];assert len(terminal)==1
memberships=[];at=0
streams=['models','links','dense_models','dense_links','odom','joints','targets','contacts','backend_status','commanded_efforts','commands_sent','commands_observed']
for phase in c['phase_schedule']:
 lo=start+at;lo_ns=start_ns+round(at*1e9);at+=phase['duration_s'];hi=start+at;hi_ns=start_ns+round(at*1e9)
 for key in streams:
  for kind,low,low_ns in [('phase',lo,lo_ns),('settled_tail',hi-c['settled_tail_s'],hi_ns-round(c['settled_tail_s']*1e9))]:
   f=[i for i,r in enumerate(raw[key]) if low<=r['t']<hi];n=[i for i,r in enumerate(raw[key]) if low_ns<=r.get('ns',round(r['t']*1e9))<hi_ns]
   differences=sorted(set(f)^set(n));memberships.append({'phase':phase['name'],'stream':key,'window':kind,'float_low_repr':repr(low),'float_hi_repr':repr(hi),'integer_low_ns':low_ns,'integer_hi_ns':hi_ns,'float_count':len(f),'integer_count':len(n),'different_indices':differences,'different_rows':[raw[key][i] for i in differences]})
formal=[]
for key in streams:
 f=[i for i,r in enumerate(raw[key]) if start<=r['t']<=end];n=[i for i,r in enumerate(raw[key]) if start_ns<=r.get('ns',round(r['t']*1e9))<=end_ns];formal.append({'stream':key,'different_indices':sorted(set(f)^set(n))})
matched=[];sent=[r for r in raw['commands_sent'] if start_ns<=r['ns']<end_ns]
for r in sent:
 candidates=[i for i,o in enumerate(raw['commands_observed']) if 0<=o['t']-r['t']<=.12 and abs(o['v']-r['v'])<1e-9 and abs(o['w']-r['w'])<1e-9];matched.append({'event_ns':r['ns'],'candidate_observed_indices':candidates})
uses={}
for row in matched:
 for i in row['candidate_observed_indices']:uses[i]=uses.get(i,0)+1
semantics=[{'gate':'command_loss_no_relay_publication_after_drain','basis':'headerless Twist callback local ROS clock','current_issue':'terminal_zero physically sourced at37.108 is recorded at37.106, misclassified inside loss; integer conversion alone cannot repair this','remedy_scope':'fresh source-stamped driver command ingress; relabel metric actual driver receipt, preserve original failed gate/result'}, {'gate':'command_relay_observed','basis':'existential repeated-value matching between command source stamp and headerless callback receipt; 0..120ms window','current_issue':'receiver clock can precede source; repeated identical commands allow several candidates and reuse an observation; not one-to-one delivery identity','remedy_scope':'fresh ingress sequence/roster/order and same-value ordered proof, retain inherited evidence limitations rather than claiming exact old command delivery'}, {'gate':'physical_backend_never_faulted legacy Bool portion','basis':'any observed true, no phase or rate qualification','current_issue':'phase attribution not used; source-stamped physical_status remains authoritative time channel','remedy_scope':'retain anytrue failure and diagnostic Bool, no new numerical relaxation'}, {'gate':'recorder_warmup_before_start','basis':'probe local ROS clock interval compared with source lifecycle stamp','current_issue':'different process clocks can lag; this run passes, but it is bootstrap observation ordering, not authoritative world time','remedy_scope':'continue explicit warmup/source completeness proof; do not label wall/receiver times as physical source times'}, {'gate':'schedule_start_marker_matches_trigger','basis':'marker.data integer start_ns','current_issue':'receipt_t is descriptive and not used for qualification','remedy_scope':'no timing repair necessary to this field'}]
out={'status':'offline_terminal_zero_callback_clock_alias_confirmed_original_P04_failure_preserved','run_id':raw['run_id'],'canonical_failed_checks':[k for k,v in json.loads((R/'probe_report.json').read_text())['checks'].items() if not v],'float_bounds':{'start_repr':repr(start),'end_repr':repr(end),'loss_with_grace_repr':repr(loss+grace)},'integer_bounds':{'start_ns':start_ns,'loss_ns':loss_ns,'loss_after_grace_ns':loss_ns+grace_ns,'end_ns':end_ns},'offending_float_callback_rows':floatbad,'offending_integerized_callback_rows':intbad,'integer_only_change_removes_failure':not intbad,'terminal_source_event':terminal[0],'bag_terminal_command_receipts':{k:[r for r in v if r['bag_receipt_ns']>=end_ns-20000000] for k,v in bagcmd.items()},'source_commands_inside_loss':sourcebad,'bag_commands_inside_loss_after_grace':{k:[r for r in v if loss_ns+grace_ns<=r['bag_receipt_ns']<end_ns] for k,v in bagcmd.items()},'all_phase_membership_comparisons':memberships,'phase_membership_difference_count':sum(bool(r['different_indices']) for r in memberships),'formal_membership':formal,'formal_membership_difference_count':sum(bool(r['different_indices']) for r in formal),'legacy_relay_match':{'source_command_count':len(sent),'matched_fraction':sum(bool(r['candidate_observed_indices']) for r in matched)/len(sent),'source_commands_with_multiple_possible_observations':sum(len(r['candidate_observed_indices'])>1 for r in matched),'observation_indices_reusable_by_multiple_source_events':sum(n>1 for n in uses.values()),'identifiability_limit':'No message identity on Twist; matching by repeated value does not prove injective delivery.'},'remaining_receipt_gate_inventory':semantics,'near_boundary_bag_packets':rows,'interpretation':'A monotonic authoritative world has reached terminal source ns before that command exists. Its downstream zero cannot physically precede that creation. The probe local cached ROS clock was stale; bag receiver independently records both navigation/relay at37.110, driver source status first clears timeout at37.120. This is a measurement clock alias, not evidence of a relay command in the loss interval. Original frozen infrastructure failure is retained.','limits':['Do not equate independent bag receipt clocks with exact publication/source time.','Driver ingress source-stamped identity is absent in P04, so an exact per-message driver receipt ledger cannot be reconstructed.','No original evidence or gate modified, no ROS/Gazebo started.'],'inputs':[{'path':str(p),'sha256':sha(p)} for p in [R/'probe_raw.json',R/'probe_report.json',R/'fixture.bag',R/'receipt.json']],'script_sha256':sha(__file__),'command':'/usr/bin/python3 '+str(Path(__file__).resolve())}
(D/'relay_clock_diagnosis.json').write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n');print(json.dumps({k:out[k] for k in ('status','offending_float_callback_rows','integer_only_change_removes_failure','phase_membership_difference_count','formal_membership_difference_count','legacy_relay_match')},indent=2))
