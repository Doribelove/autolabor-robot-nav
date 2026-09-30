#!/usr/bin/env python3
"""Read-only P03 tail attribution and fail-preserving frozen-assessor continuation."""
import ast,bisect,copy,hashlib,json,math,types
from pathlib import Path
import numpy as np
import rosbag
D=Path(__file__).resolve().parent;S=D.parents[1]
P=S/'workspace/src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p02.py'
ALLOW={'links_matches_dense_same_stamp','actual_joint_positions_agree_with_dense_source_pose','actual_odom_twist_agrees_with_dense_base_link'}
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def header(m):return {'ns':m.header.stamp.to_nsec(),'t':m.header.stamp.to_sec(),'frame':m.header.frame_id,'seq':m.header.seq}
def pos(p):return [p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
def vel(v):return [v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z]
def body(m):return dict(header(m),names=list(m.name),poses=[pos(x) for x in m.pose],twists=[vel(x) for x in m.twist])
def rotation(q):
 x,y,z,w=q
 return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
def relative_yaw(q,r):
 a,b,c,d=-q[0],-q[1],-q[2],q[3];e,f,g,h=r
 x,y,z,w=(d*e+a*h+b*g-c*f,d*f-a*g+b*h+c*e,d*g+a*f-b*e+c*h,d*h-a*e-b*f-c*g)
 return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
def cross(raw,dense):
 c=raw['contract'];model=c['model_name'];lookup={r['ns']:r for r in dense};times=[r['ns'] for r in dense];start=raw['schedule_start_ns'];end=start+36000000000
 faults={k:[] for k in ALLOW};joint_errors=[];odom_errors=[]
 def fault(k,r,reason,**kw):faults[k].append(dict(ns=r['ns'],location='before' if r['ns']<start else ('after' if r['ns']>end else 'formal'),reason=reason,**kw))
 for r in raw['links']:
  if r['ns'] not in lookup:fault('links_matches_dense_same_stamp',r,'missing_same_ns_dense')
  elif any(r[k]!=lookup[r['ns']][k] for k in ('names','poses','twists')):fault('links_matches_dense_same_stamp',r,'payload_difference')
 for r in raw['joints']:
  i=bisect.bisect_right(times,r['ns'])-1
  if i<0:continue
  if r['ns']-times[i]>2000000:fault('actual_joint_positions_agree_with_dense_source_pose',r,'newest_dense_too_old',dense_ns=times[i],age_ns=r['ns']-times[i]);continue
  b=dict(zip(dense[i]['names'],dense[i]['poses']));base=b[model+'::base_link'];errors=[]
  for joint,link in zip(c['front_joint_names'],c['front_link_names']):
   v=r['positions'][joint]-relative_yaw(base[3:],b[model+'::'+link][3:]);errors.append(abs(math.atan2(math.sin(v),math.cos(v))))
  error=max(errors);joint_errors.append(error)
  if error>c['gates']['actual_front_joint_angle_tolerance_rad']:fault('actual_joint_positions_agree_with_dense_source_pose',r,'angle_difference',maximum_error_rad=error)
 for r in raw['odom']:
  if r['ns'] not in lookup:fault('actual_odom_twist_agrees_with_dense_base_link',r,'missing_same_ns_dense');continue
  b=lookup[r['ns']];i=b['names'].index(model+'::base_link');R=rotation(b['poses'][i][3:]);v=R.T@np.array(b['twists'][i][:3]);w=R.T@np.array(b['twists'][i][3:]);error=max(abs(r['v']-v[0]),abs(r['vy']-v[1]),abs(r['w']-w[2]));odom_errors.append(error)
  if error>1e-6:fault('actual_odom_twist_agrees_with_dense_base_link',r,'twist_difference',maximum_error=error)
 return {'faults':faults,'formal_all_three_true':not any(x['location']=='formal' for rows in faults.values() for x in rows),'maximum_matched_joint_angle_error_rad':max(joint_errors),'maximum_matched_odom_body_twist_error':max(odom_errors),'all_three_true':not any(faults.values())}
def main():
 assert sha(P)=='68dd16ef4bf5a8ac85bafc8996e52c48f3c145c6a9dc88f490aa3e2d2fbfbe00'
 tree=ast.parse(P.read_bytes(),filename=str(P));probe=types.ModuleType('frozen');probe.__file__=str(P);exec(compile(tree,str(P),'exec'),vars(probe))
 out={'scope':'offline diagnosis only; no qualification repair, no raw replacement, no rerun','runs':{},'inputs':[{'path':str(P),'sha256':sha(P)}]}
 for rid in ('m12_physics_p03_s009_kd4','m12_physics_p03_s009_kd6'):
  R=S/'runs'/rid;raw=json.loads((R/'probe_raw.json').read_text());original=json.loads((R/'probe_report.json').read_text());receipt=json.loads((R/'receipt.json').read_text());assert (R/'cleanup.json').is_file();assert sha(R/'probe_raw.json')==receipt['probe_raw.json_sha256']==original['raw_sha256']
  replay=probe.assess(raw,raw['contract']);assert all(original[k]==v for k,v in replay.items())
  item={'original_replay_exact':True,'canonical_failed_checks':[k for k,v in original['checks'].items() if not v],'canonical_pass':original['pass'],'canonical_assessment_incomplete':original['assessment_incomplete'],'formal_start_ns':raw['schedule_start_ns'],'formal_end_ns':raw['schedule_start_ns']+36000000000,'raw_cross_stream_recomputation':cross(raw,raw['dense_links'])}
  bag_dense=[];clocks=[];counts={}
  with rosbag.Bag(str(R/'fixture.bag'),'r') as b:
   for topic,m,t in b.read_messages(topics=['/astra/physics_links','/clock']):
    counts[topic]=counts.get(topic,0)+1
    if topic=='/astra/physics_links':bag_dense.append(body(m))
    else:clocks.append(m.clock.to_nsec())
  item['bag_dense_counterpart_crosscheck']=cross(raw,bag_dense)
  item['bag_end_clock_ns']=clocks[-1];item['stream_tail']={k:{'count':len(raw[k]),'first_ns':raw[k][0]['ns'],'last_ns':raw[k][-1]['ns']} for k in ('models','links','dense_models','dense_links','contacts','odom','joints','targets','backend_status','commanded_efforts')}
  rawns={r['ns'] for r in raw['dense_links']};item['bag_dense_missing_from_raw_ns']=[r['ns'] for r in bag_dense if r['ns'] not in rawns]
  expected={k:raw[k][0]['ns']+((clocks[-1]-raw[k][0]['ns'])//(2000000 if k in ('dense_models','dense_links','contacts') else 20000000))*(2000000 if k in ('dense_models','dense_links','contacts') else 20000000) for k in item['stream_tail']}
  item['offline_pause_watermark_from_final_bag_clock']={'watermark_ns':clocks[-1],'not_a_recorded_GetWorldProperties_response':True,'expected_stream_last_ns':expected,'streams_short_of_expected':[k for k in expected if raw[k][-1]['ns']!=expected[k]]}
  if rid.endswith('kd6'):
   assert set(item['canonical_failed_checks'])==ALLOW
   derived=copy.deepcopy(tree);assessor=next(n for n in derived.body if isinstance(n,ast.FunctionDef) and n.name=='assess');guards=[n for n in ast.walk(assessor) if isinstance(n,ast.If) and ast.dump(n.test)==ast.dump(ast.parse('not all(checks.values())',mode='eval').body)];assert len(guards)==1
   node=guards[0];before=copy.deepcopy(node.test);expression='not all(checks.values()) and set(k for k, v in checks.items() if not v) != '+repr(ALLOW);node.test=ast.copy_location(ast.parse(expression,mode='eval').body,node.test);ast.fix_missing_locations(derived)
   restore=copy.deepcopy(derived);a=next(n for n in restore.body if isinstance(n,ast.FunctionDef) and n.name=='assess');n=next(n for n in ast.walk(a) if isinstance(n,ast.If) and ast.dump(n.test)==ast.dump(node.test));n.test=before;assert ast.dump(restore)==ast.dump(tree)
   diagnostic=types.ModuleType('diagnostic_only');diagnostic.__file__=str(P);exec(compile(derived,str(P),'exec'),vars(diagnostic));result=diagnostic.assess(raw,raw['contract']);assert result['pass'] is False and all(result['checks'][k] is False for k in ALLOW)
   item['diagnostic_continuation']={'original_false_checks_kept':True,'overall_pass_still_false':True,'no_input_mutation':True,'all_other_AST_fields_equal':True,'changed_predicate_line':node.lineno,'expression':expression,'failed_checks':[k for k,v in result['checks'].items() if not v],'result':result}
  out['runs'][rid]=item
  for name in ('probe_raw.json','probe_report.json','receipt.json','cleanup.json','actuation_contract.yaml','fixture.bag'):out['inputs'].append({'path':str(R/name),'sha256':sha(R/name)})
 out['script_sha256']=sha(__file__);out['command']='OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /usr/bin/python3 '+str(Path(__file__).resolve());(D/'terminal_tail_diagnosis.json').write_text(json.dumps(out,indent=2,ensure_ascii=False)+'\n')
 print(json.dumps({rid:{'raw_cross':v['raw_cross_stream_recomputation'],'bag_cross':v['bag_dense_counterpart_crosscheck'],'diagnostic_failed':v.get('diagnostic_continuation',{}).get('failed_checks')} for rid,v in out['runs'].items()},indent=2))
if __name__=='__main__':main()
