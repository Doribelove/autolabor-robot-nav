#!/usr/bin/env python3
"""Independent offline closed Q05 evidence audit; never starts ROS or reads live proc."""
import argparse,collections,csv,datetime,hashlib,json,math,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace'
parser=argparse.ArgumentParser();parser.add_argument('--run-id',choices=('passive_sweep_q05_static','passive_sweep_q05_actor'),required=True);args=parser.parse_args();R=S/'runs'/args.run_id;O=D/'rows'/args.run_id
sys.path[:0]=[str(W/'scripts/astra_gazebo'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
import rosbag
import numpy as np
from PIL import Image
from sensor_msgs.msg import Image as ROSImage
import passive_sweep_q05 as q
import mapped_elf_inventory_v2 as maps
import dense_sweep_p04 as dense
import evaluate_episode as evaluator
def read(p):return json.loads(Path(p).read_bytes())
cache={}
def sha(p):
 p=Path(p)
 if str(p) not in cache:
  h=hashlib.sha256()
  with p.open('rb') as f:
   for b in iter(lambda:f.read(4194304),b''):h.update(b)
  cache[str(p)]=h.hexdigest()
 return cache[str(p)]
def bind(p):return {'path':str(Path(p).resolve()),'sha256':sha(p)}
def save(name,value):
 with (O/name).open('x') as f:json.dump(value,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
def status(m):return {'stamp_ns':m.header.stamp.to_nsec(),'sequence':m.header.seq,'statuses':[{'name':r.name,'hardware_id':r.hardware_id,'level':r.level,'message':r.message,'keys':[x.key for x in r.values],'values':{x.key:x.value for x in r.values}}for r in m.status]}
releasepath=S/'contracts/passive_sweep_q05_release.json';release=read(releasepath);release_sha=sha(releasepath)
assert release_sha=='54b1ccf5b1041538098e1e6364ab6fc9f87123c28920371a796488b5f6504f22'
ledger=read(S/'queue/passive_sweep_q05_ledger.json');unit=next(x for x in ledger['units']if x['run_id']==args.run_id);assert 'receipt'in unit and Path(unit['receipt']['path'])==R/'receipt.json' and sha(R/'receipt.json')==unit['receipt']['sha256'];receipt=read(R/'receipt.json');O.mkdir(parents=True,exist_ok=False);raw=read(R/'collector_raw.json');contract=read(R/'contract.json');assessment=read(R/'assessment.json');collector=read(R/'collector_report.json');sweep=read(R/'sweep_evaluation/report.json');cleanup=read(R/'cleanup.json')
assert not(R/'fixture.bag.active').exists()
start,end=raw['start_ns'],raw['end_ns'];watermark=raw['final_drain']['source_watermark_ns'];checks={}
resources=[dict(x,current_matches=Path(x['path']).is_file() and sha(x['path'])==x['sha256'],row_archive_matches=(R/'bound_blobs'/x['sha256']).is_file() and sha(R/'bound_blobs'/x['sha256'])==x['sha256'])for x in release['resources']]
extras=[dict(release[k],role=k,current_matches=sha(release[k]['path'])==release[k]['sha256'],archive_matches=sha(R/'bound_blobs'/release[k]['sha256'])==release[k]['sha256'])for k in ('prepared','source_review','physical_parameter_decision')]
artifacts=[dict(x,current_matches=sha(x['path'])==x['sha256'])for x in receipt['artifacts']]
checks.update(release_946_unique_resources=len(resources)==len({x['path']for x in resources})==946,
 all_release_current_and_archives_match=all(x['current_matches']and x['row_archive_matches']for x in resources),
 extra_release_bindings_match=all(x['current_matches']and x['archive_matches']for x in extras),
 receipt_all_artifacts_match=all(x['current_matches']for x in artifacts),
 release_ledger_receipt_exact=receipt['release_sha256']==ledger['release_sha256']==release_sha and sha(R/'receipt.json')==unit['receipt']['sha256'],
 original_result_consistent=receipt['pass']==assessment['pass']==all(assessment['checks'].values()) and receipt['error']is None and receipt['exit_code']==0,
 exact_fresh_identity_no_retry=receipt['seed']=={'passive_sweep_q05_static':260926031,'passive_sweep_q05_actor':260926032}[args.run_id] and receipt['attempt']==1 and receipt['retry_allowed']is False,
 raw_collector_sha_exact=collector['raw_sha256']==sha(R/'collector_raw.json') and collector['pass']is True and not raw['errors'],
 isolated_cleanup=cleanup['owned_live_processes']==[] and cleanup['ports_14820_14821_released'])
cas=[]
for x in resources+extras:
 source=Path(x['path']);local=R/'bound_blobs'/x['sha256'];target=S/'code_blobs'/x['sha256'];a,b,c=source.stat(),local.stat(),target.stat()
 cas.append({'source_path':str(source),'sha256':x['sha256'],'shared_dev':c.st_dev,'shared_inode':c.st_ino,'links':c.st_nlink,'blocks':c.st_blocks,'size':c.st_size,
 'same_local_CAS_inode':(b.st_dev,b.st_ino)==(c.st_dev,c.st_ino),'independent_source_inode':(a.st_dev,a.st_ino)!=(c.st_dev,c.st_ino),'archive_readonly':b.st_mode&0o222==0 and c.st_mode&0o222==0,'CAS_hash_matches':sha(target)==x['sha256']})
checks['all946_and_extra_archives_are_readonly_independent_CAS_hardlinks']=all(x['same_local_CAS_inode']and x['independent_source_inode']and x['archive_readonly']and x['CAS_hash_matches']for x in cas)
prepared=read(release['prepared']['path']);checks['four_prepared_inputs_exact']=all(sha(R/Path(x['path']).name)==x['sha256']for x in next(u for u in prepared['units']if u['run_id']==args.run_id)['files'])
checks['actual_expanded_urdf_exact']=sha(R/'robot_description_runtime.xml')==sha(R/'robot_description.xml')
map_results=[]
for phase in ('pre','final'):
 p=R/('elf_'+phase+'.json');inventory=read(p);mapfile=R/('elf_'+phase+'.maps.txt');audit=read(str(mapfile)+'.audit.json')
 required=set(q.ELF_ROLES.values())- (set()if contract['actor']else {'actor_binaries'})
 replay=maps.inventory_from_text(mapfile.read_text(),q.ELF_ROLES,required)
 key=lambda x:(x.get('original_path',x['path']),x['sha256'],x['build_id'],tuple(x['roles']),x['device_major'],x['device_minor'],x['inode'])
 rows={'phase':phase,'raw_map':bind(mapfile),'inventory':bind(p),'raw_bound_match':sha(mapfile)==inventory['raw_maps']['sha256']==audit['raw_maps']['sha256'],
 'role_replay_exact':sorted(map(key,replay['loaded_modules']))==sorted(map(key,inventory['loaded_modules'])),
 'all_archived_modules_sha':all(sha(x['path'])==x['sha256']for x in inventory['loaded_modules']),
 'aliases_exact':replay['external_graphics_dynamic_code'][0]['mapping_rows']==inventory['external_graphics_dynamic_code'][0]['mapping_rows'],
 'dynamic_code_not_bytehashed':inventory['external_graphics_dynamic_code'][0]['static_ELF_verified']is False and inventory['external_graphics_dynamic_code'][0]['executable_memory_bytes_read_or_hashed']is False,
 'loaded_roles':sorted({r for x in inventory['loaded_modules']for r in x['roles']}),'external_graphics':inventory['external_graphics_dynamic_code']}
 map_results.append(rows)
checks['pre_final_actual_maps_replay']=all(all(x[k]for k in ('raw_bound_match','role_replay_exact','all_archived_modules_sha','aliases_exact','dynamic_code_not_bytehashed'))for x in map_results)
checks['same_gazebo_process_pre_final']=maps.same_process(read(R/'elf_pre.json'),read(R/'elf_final.json'))
counts=collections.Counter();stamps=collections.defaultdict(list);bag_status=[];bag_ingress=[];bag_events=[];bag_command_status=[];metadata=[];source_marker=[];twists=collections.defaultdict(list)
with rosbag.Bag(str(R/'fixture.bag')) as bag:
 for topic,msg,receipt_time in bag.read_messages():
  counts[topic]+=1
  if topic in raw['source_stamps']:
   stamp=json.loads(msg.data)['stamp_ns']if topic.endswith('step_status')else msg.header.stamp.to_nsec();stamps[topic].append(stamp)
  if topic=='/m2_driver/physical_status':bag_status.append(status(msg))
  elif topic=='/m2_driver/command_ingress':bag_ingress.append(status(msg))
  elif topic=='/astra_q05/command_event':bag_events.append(json.loads(msg.data))
  elif topic=='/astra_q05/command_status':bag_command_status.append(json.loads(msg.data))
  elif topic=='/astra/p03/runtime_metadata':metadata.append(json.loads(msg.data))
  elif topic=='/astra_q05/start_ns':source_marker.append(msg.data)
  elif topic in ('/cmd_vel','/cmd_vel_navigation'):twists[topic].append({'bag_receipt_ns':receipt_time.to_nsec(),'v':msg.linear.x,'w':msg.angular.z})
coverage={}
for topic,values in stamps.items():
 period=2000000 if topic in q.DENSE_TOPICS else 20000000 if topic.endswith('physical_status')else None
 formal=[n for n in values if start<=n<=end]
 coverage[topic]={'bag_count':len(values),'raw_count':len(raw['source_stamps'][topic]),'raw_bag_exact':values==raw['source_stamps'][topic],
 'formal_count':len(formal),'formal_expected':((end-start)//period+1)if period else None,
 'formal_source_grid_exact':formal==list(range(start,end+1,period))if period else None,
 'all_to_watermark_grid_exact':values==list(range(period,watermark+1,period))if period else None,
 'first_ns':values[0],'last_ns':values[-1]}
checks['all_23_required_topics_recorded']=set(contract['required_topics'])<=set(counts) and len(contract['required_topics'])==23
checks['recorder_connected_before_unpause']=raw['recorder_before_unpause']['pass']and raw['recorder_before_unpause']['source_ns']==0 and set(contract['required_topics'])<=set(raw['recorder_before_unpause']['connected'])
checks['all_eight_source_lists_raw_bag_exact']=len(coverage)==8 and all(x['raw_bag_exact']for x in coverage.values())
checks['six_500Hz_streams_and_50Hz_status_complete_to_final_watermark']=all(x['formal_source_grid_exact']and x['all_to_watermark_grid_exact']for x in coverage.values()if x['formal_expected']is not None)
checks['source_payloads_raw_bag_exact']=bag_status==raw['status_messages']and bag_ingress==raw['command_ingress']and bag_events==raw['command_events']and bag_command_status==raw['command_status']and metadata==raw['metadata']
checks['actual_world_watermark_bound']=raw['final_drain']['queried_source_ns']==[watermark,watermark]and raw['final_drain']['physics_pause']is True and raw['final_drain']['physics_step']==.002 and raw['final_drain']['wall_elapsed_s']<=contract['drain_wall_s']and watermark>=end and all(raw['final_drain']['latest'][t]==watermark for t in q.DENSE_TOPICS)
checks['source_start_marker_exact']=source_marker==[start]
formal=[x for x in bag_events if isinstance(x.get('slot'),int)and 0<=x['slot']<200]
command_rows=[]
for i,x in enumerate(formal):
 relative=i*contract['command_period_ns'];p=next(p for p in contract['phase_schedule']if p['start_ns']<=relative<p['end_ns']);w=p['v_mps']*math.tan(p['steering_rad'])/contract['wheelbase_m']
 command_rows.append({'slot':i,'valid':x['slot']==i and x['scheduled_ns']==start+relative and x['phase']==p['name']and x['v']==p['v_mps']and abs(x['w']-w)<1e-15 and x['steering']==p['steering_rad']and 0<=x['actual_ns']-x['scheduled_ns']<=contract['command_max_lateness_ns'],'lateness_ns':x['actual_ns']-x['scheduled_ns']})
checks['independent_200_command_slots_payloads_deadlines']=len(formal)==200 and all(x['valid']for x in command_rows)
checks['frozen_command_assessor_replay_exact']=q.command_audit(raw['command_events'],contract,start)==assessment['command_audit']
checks['frozen_status_assessor_replay_exact']=q.physical_status_audit(raw['status_messages'],start,end)==assessment['physical_status_audit']
images=[]
for record in raw['images']:
 m=ROSImage();m.deserialize(Path(record['raw_message']['path']).read_bytes());pixels=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
 if m.encoding=='bgr8':pixels=pixels[:,:,::-1]
 with Image.open(record['path']) as png:actual=np.array(png);mode=png.mode;size=png.size
 images.append({'name':Path(record['path']).name,'png':bind(record['path']),'raw_message':bind(record['raw_message']['path']),
 'source_ns':m.header.stamp.to_nsec(),'frame_id':m.header.frame_id,'encoding':m.encoding,'size':size,'mode':mode,
 'raw_bound_matches':sha(record['raw_message']['path'])==record['raw_message']['sha256'],'png_bound_matches':sha(record['path'])==record['sha256'],
 'source_stamp_matches':m.header.stamp.to_nsec()==record['source_ns'],'lossless_native_pixel_equality':np.array_equal(actual,pixels),'pixel_std':float(pixels.std())})
checks['both_native_pngs_exact_source_message_pixels']=len(images)==2 and all(x['size']==(2560,1440)and x['mode']=='RGB'and all(x[k]for k in ('raw_bound_matches','png_bound_matches','source_stamp_matches','lossless_native_pixel_equality'))for x in images)
checks['image_source_time_native_dense_grid']=all(start<=x['source_ns']<=end and x['source_ns']in stamps['/astra/p03/end_links']for x in images)
closure=dense.authenticate_closure(R/'run_closure.json',start,end)
checks['run_closure_authentication_itself_passes']=closure['pass']is True
conversion=dense.authenticate_conversion(R/'conversion_manifest.json',R/'robot_description_runtime.xml',R/'converted_robot.sdf',closure,False)
model=dense.parse_converted_model((R/'robot_description_runtime.xml').read_text(),(R/'converted_robot.sdf').read_text());runtime_models=[]
for meta in metadata:runtime_models.append({'revision':meta['revision'],'bodies':dense.validate_runtime_model(dense.runtime_geometry(meta),model)})
checks['actual_runtime_converter_and_native_model_qualified']=conversion['pass']and conversion['runtime_converter_correspondence_qualified']and len(runtime_models)>0 and all(len(x['bodies'])==7 for x in runtime_models)and sweep['diagnostic_only']is False
checks['5000_geometry_intervals_and_original_source_pass']=sweep['dense_solver_intervals']==5000 and sweep['integrity_pass']and sweep['issues']==[]

save('raw_bag_map_image_audit.json',{'checks':checks,'pass':all(checks.values()),'status':'independent_Q05_closed_raw_bag_maps_image_audit_pass'if all(checks.values())else'independent_Q05_closed_audit_failed','independent_review':True,'original_episode_pass':receipt['pass'],'release_sha256':release_sha,'resources':resources,'extra_bindings':extras,'receipt_artifacts':artifacts,'ledger':ledger,'CAS_hardlinks':cas,'conversion_authentication':conversion,'runtime_model_checks':runtime_models,'maps':map_results,'bag_topic_counts':dict(counts),'source_coverage':coverage,'command_rows':command_rows,'native_images':images,'clock_source_scope':'Command publication /clock and actual driver SimTime remain distinct; headerless Twist bag receipt time is not publication or acceptance time.','graphics_scope':'Exact reviewed local GL alias and three driver ELF bindings; dynamic executable bytes were neither read nor hashed.','geometry_scope':'Fresh source/body/conversion qualification within declared Gazebo algorithmic scope only; no real continuous-safety claim.','no_ROS_or_live_process_access':True,'inputs':[bind(p)for p in [releasepath,S/'queue/passive_sweep_q05_ledger.json',R/'receipt.json',R/'collector_raw.json',R/'fixture.bag',R/'assessment.json',R/'sweep_evaluation/report.json',Path(__file__)]],'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()})
print(json.dumps({'pass':all(checks.values()),'checks':len(checks),'failed':[k for k,v in checks.items()if not v],'report_sha256':sha(O/'raw_bag_map_image_audit.json')},indent=2));assert all(checks.values())
