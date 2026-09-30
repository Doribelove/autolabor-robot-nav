#!/usr/bin/env python3
"""Preserve failed N01 DWA outcome; independent saved-evidence diagnosis, not requalification."""
import collections,hashlib,json,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';R=S/'runs/astra_navigation_n01_dwa_common_route'
sys.path[:0]=[str(W/'scripts/astra_gazebo'),str(W/'devel/lib/python3/dist-packages'),'/opt/ros/noetic/lib/python3/dist-packages']
import rosbag,numpy as np,mapped_elf_inventory_v2 as maps,passive_sweep_q05 as q
from PIL import Image
from sensor_msgs.msg import Image as ROSImage
read=lambda p:json.loads(p.read_bytes())
cache={}
def sha(p):
 p=Path(p)
 if str(p)not in cache:
  h=hashlib.sha256()
  with p.open('rb')as f:
   for b in iter(lambda:f.read(4194304),b''):h.update(b)
  cache[str(p)]=h.hexdigest()
 return cache[str(p)]
def bind(p):return {'path':str(p),'sha256':sha(p)}
ledger=read(S/'queue/navigation_n01_ledger.json');unit=next(x for x in ledger['units']if x['run_id']==R.name);assert unit['receipt']['sha256']==sha(R/'receipt.json')
receipt=read(R/'receipt.json');raw=read(R/'collector_raw.json');collector=read(R/'collector_report.json');contract=read(R/'contract.json');release=read(S/'contracts/navigation_n01_release.json');cleanup=read(R/'cleanup.json')
start,end=raw['start_ns'],raw['end_ns'];watermark=raw['final_drain']['source_watermark_ns'];checks={}
checks['original_negative_receipt_unmodified']=receipt['integrity_pass']is False and receipt['report']is None and receipt['task_outcome']is None and receipt['error']=="; assessment FileNotFoundError(2, 'No such file or directory')"and receipt['exit_code']==0
checks['collector_negative_image_error_preserved']=collector['pass']is False and collector['task_outcome']=='fault'and collector['errors']==raw['errors']==['RuntimeError: requested native image snapshots absent']and sha(R/'collector_raw.json')==collector['raw_sha256']
checks['first_fault_source_retained']=raw['terminal']['phase']=='fault'and raw['terminal']['terminal_source_ns']==7700000000 and raw['terminal']['terminal_reason']=='ValueError:nonplanar or nonfinite method command'and start==7440000000 and end-start==260000000
checks['missing_original_final_closure_not_fabricated']=all(not(R/name).exists()for name in ('ready_for_maps_final.json','maps_final_ready.json','elf_final.json','baseline_runtime_inventory.json','run_closure.json','assessment.json','sweep_evaluation/report.json'))
checks['all_receipt_artifacts_bound']=all(sha(x['path'])==x['sha256']for x in receipt['artifacts'])
archive=[]
for x in release['resources']+[release[k]for k in ('prepared','source_review','passive_geometry_qualification')]:
 p=Path(x['path']);c=S/'code_blobs'/x['sha256'];l=R/'bound_blobs'/x['sha256'];a,b,d=p.stat(),c.stat(),l.stat();ok=sha(p)==sha(c)==sha(l)==x['sha256']and (a.st_dev,a.st_ino)!=(b.st_dev,b.st_ino)and(b.st_dev,b.st_ino)==(d.st_dev,d.st_ino)and b.st_mode&0o222==d.st_mode&0o222==0;archive.append(dict(x,pass_=ok))
checks['601_plus3_current_CAS_sources_and_inode_binding']=len(archive)==604 and all(x['pass_']for x in archive)
counts=collections.Counter();stamps=collections.defaultdict(list);statuses=[];method=[]
with rosbag.Bag(str(R/'fixture.bag'))as bag:
 for topic,m,receipt_ns in bag.read_messages():
  counts[topic]+=1
  if topic in raw['source_stamps']:stamps[topic].append(json.loads(m.data)['stamp_ns']if topic.endswith('step_status')else m.header.stamp.to_nsec())
  if topic=='/astra/navigation/status':statuses.append(json.loads(m.data))
  elif topic=='/astra/method_cmd':method.append({'bag_receipt_ns':receipt_ns.to_nsec(),'linear_xyz':[m.linear.x,m.linear.y,m.linear.z],'angular_xyz':[m.angular.x,m.angular.y,m.angular.z]})
coverage={t:{'raw_bag_exact':v==raw['source_stamps'][t],'formal_count':len([x for x in v if start<=x<=end]),'formal_grid_exact':[x for x in v if start<=x<=end]==list(range(start,end+1,2000000)),'full_to_watermark':v==list(range(2000000,watermark+1,2000000))}for t,v in stamps.items()if t in q.DENSE_TOPICS}
checks['diagnostic_six_dense_streams_131formal_source_samples_complete']=len(coverage)==6 and all(x['raw_bag_exact']and x['formal_count']==131 and x['formal_grid_exact']and x['full_to_watermark']for x in coverage.values())
faults=[x for x in statuses if x['phase']=='fault'];checks['bag_first_terminal_immutable_fault']=bool(faults)and len({(x['phase'],x['terminal_source_ns'],x['terminal_reason'])for x in faults})==1 and faults[0]['terminal_source_ns']==end
checks['all_fault_commands_zero']=all(x['v']==x['w']==0 for x in raw['command_trace_summary']if x['phase']=='fault')
checks['drain_stable_world_source_recorded_before_image_failure']=raw['final_drain']['queried_source_ns']==[watermark,watermark]and raw['final_drain']['physics_pause']is True and watermark==7872000000
checks['catch_cleanup_double_stop_pause_visible']=raw['service_audit']['pass']is False and raw['service_audit']['required_call_counts']['/astra/navigation/stop']==raw['service_audit']['required_call_counts']['/gazebo/pause_physics']==2
images=[]
for x in raw['images']:
 m=ROSImage();m.deserialize(Path(x['raw_message']['path']).read_bytes());pixels=np.frombuffer(m.data,dtype=np.uint8).reshape(m.height,m.step)[:,:m.width*3].reshape(m.height,m.width,3)
 if m.encoding=='bgr8':pixels=pixels[:,:,::-1]
 with Image.open(x['path'])as im:equal=np.array_equal(np.array(im),pixels);size=im.size
 images.append({'png':bind(Path(x['path'])),'raw_message':bind(Path(x['raw_message']['path'])),'source_ns':x['source_ns'],'size':size,'pixel_equal':equal,'source_stamp_equal':m.header.stamp.to_nsec()==x['source_ns'],'SHA_bound':sha(x['path'])==x['sha256']and sha(x['raw_message']['path'])==x['raw_message']['sha256']})
checks['one_actual_native_terminal_frame_preserved']=len(images)==1 and images[0]['size']==(2560,1440)and images[0]['source_ns']==7500000000 and all(images[0][k]for k in ('pixel_equal','source_stamp_equal','SHA_bound'))
checks['gate_plus1s_frame_temporally_unreachable']=watermark<start+1000000000
pre=read(R/'elf_pre.json');replay=maps.inventory_from_text((R/'elf_pre.maps.txt').read_text(),q.ELF_ROLES,set(q.ELF_ROLES.values())-{'actor_binaries'})
key=lambda x:(x.get('original_path',x['path']),x['sha256'],x['build_id'],tuple(x['roles']),x['device_major'],x['device_minor'],x['inode'])
checks['actual_Gazebo_pre_maps_and_required_modules_bound']=sha(R/'elf_pre.maps.txt')==pre['raw_maps']['sha256']and sorted(map(key,pre['loaded_modules']))==sorted(map(key,replay['loaded_modules']))and all(sha(x['path'])==x['sha256']for x in pre['loaded_modules'])
checks['cleanup_owned_processes_and_ports']=cleanup['owned_live_processes']==[]and cleanup['ports_14820_14821_released']
out={'status':'independent_N01_DWA_failure_preservation_diagnosis_pass'if all(checks.values())else'independent_failure_audit_failed','pass':all(checks.values()),'independent_review':True,'checks':checks,'original_qualification_pass':False,'original_receipt':receipt,'raw_terminal':raw['terminal'],'diagnostic_formal_window_ns':[start,end],'coverage_diagnostic_only':coverage,'archive':archive,'native_images':images,'bag_topic_counts':dict(counts),'method_command_nonzero_lateral_observations':[x for x in method if x['linear_xyz'][1]!=0],'method_command_clock_scope':'Unstamped Twist bag receipt clock is not exact command-generation or physical-driver receipt source time. External independent transport/parameter root-cause report is separate.','failure_chain':['Coordinator enters fault from rejected nonplanar/nonfinite method command; terminal source names latest actual odom at detection, not command generation.','Fault occurs at 0.26s after gate, before required gate+1s image is possible. One source-bound near-terminal frame saved.','Collector explicitly raises requested native image snapshots absent before emitting final maps marker. Catch cleanup duplicates stop/pause requests, retained in service audit.','Runner closure then cannot open absent final ELF inventory; assessment absent, receipt integrity false. No retrospective promotion.'],'scope_limits':['No complete DWA final Gazebo/move_base map closure or qualified geometry assessment.','Dense diagnostic source completeness cannot override original failed capture result.','Root reported outer manager exec exit143; per-attempt roslaunch exit0 and durable terminal receipts prove those narrower facts only.'], 'inputs':[bind(p)for p in (Path(__file__),R/'receipt.json',R/'collector_raw.json',R/'collector_report.json',R/'fixture.bag',R/'elf_pre.maps.txt',W/'src/simulation/m2_gazebo/scripts/astra_navigation_collector_p04.py')],'no_ROS':True}
with(D/'dwa_failure_audit.json').open('x')as f:json.dump(out,f,indent=2);f.write('\n')
print(json.dumps({'pass':out['pass'],'checks':len(checks),'failed':[k for k,v in checks.items()if not v],'report_SHA':sha(D/'dwa_failure_audit.json')},indent=2));assert out['pass']
