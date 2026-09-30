import hashlib,importlib.util,json
from pathlib import Path
import rosbag
D=Path(__file__).resolve().parent;S=D.parents[1];R=S/'runs/m12_physics_p06_s020_kd4';O=D/'rows'/R.name
receipt=json.loads((R/'receipt.json').read_bytes());assert (R/'cleanup.json').exists() and not (R/'fixture.bag.active').exists();raw=json.loads((R/'probe_raw.json').read_bytes());P=S/'workspace/src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p06.py';sp=importlib.util.spec_from_file_location('frozenp06diag',P);m=importlib.util.module_from_spec(sp);sp.loader.exec_module(m)
source=m.source_packet_integrity(raw,raw['contract']);formalend=raw['schedule_start_ns']+36000000000;wm=raw['final_drain']['source_watermark_ns'];tail=m.final_stream_watermarks(raw,wm)
keys={'/astra/world_models':'models','/astra/world_links':'links','/astra/physics_models':'dense_models','/astra/physics_links':'dense_links','/odom':'odom','/joint_states':'joints','/m2_driver/actuator_target':'targets','/m2_driver/actuator_commanded_effort':'commanded_efforts','/astra/physics_contacts':'contacts','/m2_driver/physical_status':'backend_status'}
bagstamps={k:[] for k in keys};tailother={k:[] for k in ('/m2_driver/command_ingress','/astra_fixture/command_event','/astra_fixture/command_source_status','/cmd_vel','/cmd_vel_navigation','/clock')}
with rosbag.Bag(str(R/'fixture.bag'),'r') as b:
 for t,msg,bt in b.read_messages(topics=list(bagstamps)+list(tailother)):
  if t in bagstamps:bagstamps[t].append(msg.header.stamp.to_nsec());continue
  if t=='/clock':row={'ns':msg.clock.to_nsec(),'receipt_ns':bt.to_nsec()}
  elif t in ('/cmd_vel','/cmd_vel_navigation'):row={'receipt_ns':bt.to_nsec(),'v':msg.linear.x,'w':msg.angular.z}
  else:row={'ns':msg.header.stamp.to_nsec(),'seq':msg.header.seq,'receipt_ns':bt.to_nsec(),'statuses':[dict({k:getattr(x,k) for k in ('name','level','message')},values={v.key:v.value for v in x.values}) for x in msg.status]}
  tailother[t].append(row)
streams={}
for topic,k in keys.items():
 rn=[r['ns'] for r in raw[k]];bn=bagstamps[topic];streams[k]={'raw_count':len(rn),'bag_count':len(bn),'raw_first_ns':rn[0],'raw_last_ns':rn[-1],'bag_first_ns':bn[0],'bag_last_ns':bn[-1],'ordered_stamp_equal':rn==bn,'raw_only_ns':sorted(set(rn)-set(bn)),'bag_only_ns':sorted(set(bn)-set(rn)),'required_last_ns':tail['required_last_ns'][k]}
ing=m.parse_command_ingress(raw);out={'formal_start_ns':raw['schedule_start_ns'],'formal_end_ns':formalend,'final_drain':raw['final_drain'],'recomputed_tail':tail,'source_packet_failed_checks':[k for k,v in source.items() if not v],'source_packet_checks':source,'streams':streams,'raw_ingress_count':len(ing),'raw_ingress_tail':raw['command_ingress'][-5:],'raw_terminal_events':[r for r in raw['commands_sent'] if r['ns']>=formalend],'raw_source_status':raw['source_status'],'bag_tail':{k:v[-6:] for k,v in tailother.items()},'bag_other_counts':{k:len(v) for k,v in tailother.items()},'raw_counts':{k:len(v) for k,v in raw.items() if isinstance(v,list)},'closing_conditions':{'tail':tail['complete'],'complete':any(v.get('values',{}).get('state')=='complete' for r in raw['source_status'] for v in r['statuses']),'terminal_event':any(r['phase']=='terminal_zero' for r in raw['commands_sent']),'driver_terminal':any(r['ns']>=formalend and r['v']==r['w']==0. for r in ing)},'raw_sha256':hashlib.sha256((R/'probe_raw.json').read_bytes()).hexdigest(),'script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'offline_diagnosis_original_terminal_failure_unchanged':True}
(O/'terminal_diagnosis.json').write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({k:v for k,v in out.items() if k in ('formal_start_ns','formal_end_ns','final_drain','source_packet_failed_checks','streams','closing_conditions')},indent=2))
