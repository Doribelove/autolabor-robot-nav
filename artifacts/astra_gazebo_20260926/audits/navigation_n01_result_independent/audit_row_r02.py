#!/usr/bin/env python3
"""Independent read-only N01 closed row replay. No ROS runtime mutation."""
import argparse,collections,hashlib,json,math,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace'
sys.path[:0]=[str(W/'scripts/astra_gazebo'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
import rosbag,yaml,numpy as np
from PIL import Image
from sensor_msgs.msg import Image as ROSImage
import navigation_evidence_p04 as nav
import passive_sweep_q05 as q
import dense_sweep_p04 as dense
import evaluate_episode as evaluator
import mapped_elf_inventory_v2 as maps
ap=argparse.ArgumentParser();ap.add_argument('--run-id',required=True);args=ap.parse_args();R=S/'runs'/args.run_id;O=D/'rows'/args.run_id/'r02'
def read(p):return json.loads(Path(p).read_bytes())
cache={}
def sha(p):
 p=Path(p)
 if str(p)not in cache:
  h=hashlib.sha256()
  with p.open('rb')as f:
   for x in iter(lambda:f.read(4194304),b''):h.update(x)
  cache[str(p)]=h.hexdigest()
 return cache[str(p)]
def bind(p):return {'path':str(Path(p).resolve()),'sha256':sha(p)}
def status(m):return {'stamp_ns':m.header.stamp.to_nsec(),'sequence':m.header.seq,'statuses':[{'name':r.name,'hardware_id':r.hardware_id,'level':r.level,'message':r.message,'keys':[x.key for x in r.values],'values':{x.key:x.value for x in r.values}}for r in m.status]}
ledger=read(S/'queue/navigation_n01_ledger.json');u=next(x for x in ledger['units']if x['run_id']==args.run_id)
assert u.get('receipt',{}).get('sha256')==sha(R/'receipt.json') and not(R/'fixture.bag.active').exists()
receipt=read(R/'receipt.json');assert receipt['integrity_pass']is True, 'Use separate failure diagnosis for infrastructure-negative row'
O.mkdir(parents=True,exist_ok=False);raw=read(R/'collector_raw.json');contract=read(R/'contract.json');assessment=read(R/'assessment.json');collector=read(R/'collector_report.json');sweep=read(R/'sweep_evaluation/report.json');cleanup=read(R/'cleanup.json')
release_path=S/'contracts/navigation_n01_release.json';release=read(release_path);release_sha=sha(release_path);assert release_sha=='ae155b33ea2ad6c732163265c266764817ff9268587e136a0c9f9ff93aed9ea5'
start,end=raw['start_ns'],raw['end_ns'];watermark=raw['final_drain']['source_watermark_ns'];checks={}
resources=release['resources']+[release[k]for k in ('prepared','source_review','passive_geometry_qualification')]
archive=[]
for b in resources:
 source=Path(b['path']);cas=S/'code_blobs'/b['sha256'];local=R/'bound_blobs'/b['sha256'];a,c,l=source.stat(),cas.stat(),local.stat()
 archive.append(dict(b,current_SHA_matches=sha(source)==b['sha256'],archive_SHA_matches=sha(cas)==sha(local)==b['sha256'],source_independent=(a.st_dev,a.st_ino)!=(c.st_dev,c.st_ino),CAS_hardlink=(c.st_dev,c.st_ino)==(l.st_dev,l.st_ino),readonly=c.st_mode&0o222==l.st_mode&0o222==0))
checks['all601_release_resources_and_three_extras_real_CAS_bound']=len(release['resources'])==601 and all(all(x[k]for k in ('current_SHA_matches','archive_SHA_matches','source_independent','CAS_hardlink','readonly'))for x in archive)
checks['receipt_all_artifacts_SHA']=all(sha(x['path'])==x['sha256']for x in receipt['artifacts'])and sha(R/'assessment.json')==receipt['report']['sha256']
checks['receipt_identity_first_attempt_consistent']=receipt['release_sha256']==ledger['release_sha256']==release_sha and receipt['seed']==260926026 and receipt['attempt']==1 and receipt['retry_allowed']is False and receipt['method']==contract['method']and receipt['error']is None and receipt['exit_code']==0
checks['raw_collector_integrity_SHA']=sha(R/'collector_raw.json')==collector['raw_sha256']and collector['pass']is True and not raw['errors']
checks['cleanup_owned_processes_and_ports']=cleanup['owned_live_processes']==[]and cleanup['ports_14820_14821_released']
maprows=[]
for phase in ('pre','final'):
 p=R/('elf_'+phase+'.json');v=read(p);m=R/('elf_'+phase+'.maps.txt');replay=maps.inventory_from_text(m.read_text(),q.ELF_ROLES,set(q.ELF_ROLES.values())-{'actor_binaries'})
 key=lambda x:(x.get('original_path',x['path']),x['sha256'],x['build_id'],tuple(x['roles']),x['device_major'],x['device_minor'],x['inode'])
 maprows.append({'phase':phase,'map':bind(m),'raw_SHA_matches':sha(m)==v['raw_maps']['sha256'],'loaded_modules_exact':sorted(map(key,replay['loaded_modules']))==sorted(map(key,v['loaded_modules'])),'all_modules_current_SHA':all(sha(x['path'])==x['sha256']for x in v['loaded_modules']),'graphics_alias_exact':all({k:z for k,z in a.items()if k!='render_driver_bindings'}=={k:z for k,z in b.items()if k!='render_driver_bindings'}and sorted(map(key,a['render_driver_bindings']))==sorted(map(key,b['render_driver_bindings']))for a,b in zip(replay['external_graphics_dynamic_code'],v['external_graphics_dynamic_code']))and len(replay['external_graphics_dynamic_code'])==len(v['external_graphics_dynamic_code'])})
checks['pre_final_native_maps_replay_exact']=all(all(x[k]for k in ('raw_SHA_matches','loaded_modules_exact','all_modules_current_SHA','graphics_alias_exact'))for x in maprows)
checks['same_owned_Gazebo_process_pre_final']=maps.same_process(read(R/'elf_pre.json'),read(R/'elf_final.json'))
counts=collections.Counter();stamps=collections.defaultdict(list);statuses=[];ingress=[];metadata=[];coordinator=collections.defaultdict(list);samples=[];odom=[];scan_stamps=[]
with rosbag.Bag(str(R/'fixture.bag'))as bag:
 for topic,m,receipt_t in bag.read_messages():
  counts[topic]+=1
  if topic in raw['source_stamps']:stamps[topic].append(json.loads(m.data)['stamp_ns']if topic.endswith('step_status')else m.header.stamp.to_nsec())
  if topic=='/m2_driver/physical_status':statuses.append(status(m))
  elif topic=='/m2_driver/command_ingress':ingress.append(status(m))
  elif topic=='/astra/p03/runtime_metadata':metadata.append(json.loads(m.data))
  elif topic.startswith('/astra/navigation/'):coordinator[topic].append(json.loads(m.data))
  elif topic=='/astra/p03/end_links':
   assert m.header.frame_id=='world'and list(m.name).count('autolabor_m2::base_link')==1
   p=m.pose[list(m.name).index('autolabor_m2::base_link')];samples.append((m.header.stamp.to_nsec(),(p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)))
  elif topic=='/odom':
   p=m.pose.pose;odom.append((m.header.stamp.to_nsec(),(p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w)))
  elif topic=='/scan':scan_stamps.append(m.header.stamp.to_nsec())
coverage={}
for topic,values in stamps.items():
 period=2000000 if topic in nav.DENSE_TOPICS else 20000000 if topic.endswith('physical_status')else None
 formal=[x for x in values if start<=x<=end]
 coverage[topic]={'count':len(values),'formal_count':len(formal),'raw_bag_exact':values==raw['source_stamps'][topic],'formal_grid_exact':formal==list(range(start,end+1,period))if period else None,'full_grid_to_watermark':values==list(range(period,watermark+1,period))if period else None,'first_ns':values[0],'last_ns':values[-1]}
checks['all_eight_raw_bag_source_stamp_lists_exact']=len(coverage)==8 and all(x['raw_bag_exact']for x in coverage.values())
checks['six_dense_and50Hz_status_full_formal_and_final_watermark']=all(x['formal_grid_exact']and x['full_grid_to_watermark']for x in coverage.values()if x['formal_grid_exact']is not None)
checks['bag_payloads_status_ingress_metadata_exact']=statuses==raw['status_messages']and ingress==raw['command_ingress']and metadata==raw['metadata']
coordinator_comparisons={}
for topic,rawkey in [('/astra/navigation/status','coordinator_status'),('/astra/navigation/command_trace','command_trace_summary')]:
 a=coordinator[topic];b=raw[rawkey];keys=set(b[0]);project=lambda x:{k:x[k]for k in keys}
 af=[project(x)for x in a if start<=x['publisher_visible_clock_ns']<=end];bf=[x for x in b if start<=x['publisher_visible_clock_ns']<=end]
 coordinator_comparisons[topic]={'bag_total':len(a),'raw_total':len(b),'formal_bag':len(af),'formal_raw':len(bf),'formal_exact':af==bf,'positive_time_raw_subset_of_bag':all(x in [project(y)for y in a]for x in b if x['publisher_visible_clock_ns']>0)}
checks['coordinator_formal_bag_raw_payloads_exact']=all(x['formal_exact']and x['positive_time_raw_subset_of_bag']for x in coordinator_comparisons.values())and coordinator['/astra/navigation/identity']==raw['coordinator_identity']and coordinator['/astra/navigation/gate']==[raw['gate']]
nonproducing={'/astra/common_route/served_path'}if contract['method']not in('actual_teb_common_route','dwa_common_route')else set()
checks['required_producing_record_topics_present']=set(contract['record_topics'])-nonproducing<=set(counts)
checks['recorder_handshakes_before_unpause_and_arm']=raw['recorder_before_unpause']['pass']and raw['recorder_before_unpause']['source_ns']==0 and set(contract['required_before_unpause'])<=set(raw['recorder_before_unpause']['connected'])and raw['recorder_before_arm']['pass']and 0<raw['recorder_before_arm']['source_ns']<contract['activation_source_ns']and set(contract['required_before_arm'])<=set(raw['recorder_before_arm']['connected'])
checks['paused_stable_source_watermark']=raw['final_drain']['queried_source_ns']==[watermark,watermark]and raw['final_drain']['physics_pause']is True and raw['final_drain']['physics_step']==.002 and raw['final_drain']['wall_elapsed_s']<=contract['drain_wall_s']and watermark>=end
checks['exact_shared_gate_terminal_semantics']=nav.gate_record(raw['gate'],contract)==raw['gate']and nav.terminal_record(raw['terminal'],contract)==raw['terminal']and start==raw['gate']['source_ns']and end==raw['terminal']['terminal_source_ns']
terminals=[x for x in raw['coordinator_status']if x['phase']in nav.TERMINAL_PHASES]
checks['first_terminal_never_overwritten']=bool(terminals)and len({(x['phase'],x['terminal_source_ns'],x['terminal_reason'])for x in terminals})==1
trace=raw['command_trace_summary'];checks['terminal_commands_all_zero']=all(x['v']==x['w']==0 for x in trace if x['phase']in nav.TERMINAL_PHASES)
scene=yaml.safe_load((R/'scene_manifest.yaml').read_text());task=nav.navigation_outcome(samples,scene['route']['goal'],start,end,raw['terminal'],contract)
checks['independent_bag_native_goal_replay_exact']=task==assessment['task']
physical=nav.physical_status_audit(statuses,start,end);checks['physical_status_exact_replay']=physical==assessment['physical_status_audit']
_models,_links,contacts,physmeta=evaluator.read_bag(R/'fixture.bag');world=evaluator.WorldCollisions((R/'world.sdf').read_text());errors=[]
contact=evaluator.contact_audit({x['stamp_ns']:x for x in contacts},world,start,end,'autolabor_m2',set(world.actor_speed_bounds),errors);summary=dict(contact)
for k in ('illegal_robot_contacts','wheel_ground_support_contacts','actor_environment_contacts'):summary[k+'_count']=len(summary[k]);summary[k+'_examples']=summary[k][:10];del summary[k]
checks['contact_full_bag_replay_exact']=summary==assessment['contact_audit']and errors==assessment['contact_issues']
integrity={'collector_complete':collector['pass'],'passive_source_geometry_complete':sweep['integrity_pass'],'contact_source_coverage':nav.source_coverage([x['stamp_ns']for x in contacts],start,end)['pass'],'contact_message_contract':not errors,'contact_metadata_complete':all(physmeta.get(k)is True for k in ('contacts_every_physics_step','all_collision_pairs_retained','empty_contact_packets_published')),'physical_status_message_contract':physical['message_contract_pass'],'physical_status_source_coverage':physical['coverage']['pass'],'goal_source_contract':task['goal_geometry_contract_pass']}
safety={'positive_engineering_clearance':sweep['clearance_status']=='pass_within_model_scope','no_illegal_robot_contact':not contact['illegal_robot_contacts'],'no_actor_environment_contact':not contact['actor_environment_contacts'],'no_physical_backend_fault_timeout_brake_emergency':physical['no_fault_timeout_brake_emergency']}
checks['all_eight_integrity_four_safety_gates_exact']=integrity==assessment['integrity_checks']and safety==assessment['safety_checks']and all(integrity.values())==assessment['capture_integrity_pass']and(all(integrity.values())and all(safety.values()))==assessment['safety_within_declared_engineering_scope_pass']
valid=all(integrity.values())and all(safety.values())and task['task_phase']=='complete'and task['raw_geometric_arrival_source_ns']is not None
checks['failure_inclusive_completion_and_time_exact']=valid==assessment['valid_completed']and(task['raw_geometric_arrival_time_s']if valid else 22.)==assessment['restricted_goal_time_s']
images=[]
for x in raw['images']:
 m=ROSImage();m.deserialize(Path(x['raw_message']['path']).read_bytes());p=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
 if m.encoding=='bgr8':p=p[:,:,::-1]
 with Image.open(x['path'])as im:match=np.array_equal(np.array(im),p);size=im.size
 images.append({'png':bind(x['path']),'raw_message':bind(x['raw_message']['path']),'source_ns':x['source_ns'],'size':size,'native_pixel_equality':match,'source_stamp_equal':m.header.stamp.to_nsec()==x['source_ns'],'original_hashes_equal':sha(x['path'])==x['sha256']and sha(x['raw_message']['path'])==x['raw_message']['sha256']})
checks['native_images_lossless_exact_source']=bool(images)and all(x['size']==(2560,1440)and all(x[k]for k in ('native_pixel_equality','source_stamp_equal','original_hashes_equal'))and x['source_ns']in stamps['/astra/p03/end_links']for x in images)
closure=dense.authenticate_closure(R/'run_closure.json',start,end);conversion=dense.authenticate_conversion(R/'conversion_manifest.json',R/'robot_description_runtime.xml',R/'converted_robot.sdf',closure,False)
model=dense.parse_converted_model((R/'robot_description_runtime.xml').read_text(),(R/'converted_robot.sdf').read_text())
checks['fresh_actual_conversion_and_native_seven_bodies']=closure['pass']and conversion['runtime_converter_correspondence_qualified']and all(len(dense.validate_runtime_model(dense.runtime_geometry(m),model))==7 for m in metadata)
out={'status':'independent_N01_closed_row_audit_pass'if all(checks.values())else'independent_N01_closed_row_audit_failed','pass':all(checks.values()),'independent_review':True,'run_id':args.run_id,'method':contract['method'],'checks':checks,'original_assessment_summary':{k:assessment[k]for k in ('capture_integrity_pass','safety_within_declared_engineering_scope_pass','valid_completed','restricted_goal_time_s')},'task':task,'archive':archive,'source_coverage':coverage,'bag_topics':dict(counts),'declared_but_nonproducing_topics':sorted(set(contract['record_topics'])-set(counts)),'coordinator_stream_comparisons':coordinator_comparisons,'native_images':images,'maps':maprows,'contact_counts':{k:v for k,v in summary.items()if k.endswith('_count')},'formal_source_window_ns':[start,end],'final_source_watermark_ns':watermark,'native_dense_intervals':(end-start)//2000000,'minimum_engineering_swept_xy_m':sweep['minimum_engineering_conservative_swept_xy_m'],'source_stamped_odometry_count':len(odom),'scan_source_count':len(scan_stamps),'source_bindings':[bind(p)for p in (Path(__file__),release_path,R/'receipt.json',R/'collector_raw.json',R/'fixture.bag',R/'assessment.json',R/'sweep_evaluation/report.json')],'scope':'Single clear-fixture integration; sensor-only/shared-route fairness limitations of frozen R2 retained; no planner-decision or performance claim; headerless method Twist generation time is unknown.'}
with(O/'row_audit.json').open('x')as f:json.dump(out,f,indent=2,ensure_ascii=False);f.write('\n')
print(json.dumps({'pass':out['pass'],'checks':len(checks),'failed':[k for k,v in checks.items()if not v],'report_SHA':sha(O/'row_audit.json')},indent=2));assert out['pass']
