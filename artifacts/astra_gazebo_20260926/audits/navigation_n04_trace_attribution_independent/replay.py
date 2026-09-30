"""Independent descriptive counts and frame-correct, exact-source terminal odometry.
No ROS process, old output write, full geometry reclassification, or new outcomes.
"""
from pathlib import Path
import collections,csv,hashlib,json,math,sys
sys.path.insert(0,'/opt/ros/noetic/lib/python3/dist-packages')
import rosbag
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');A=Path(__file__).resolve().parent

def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
 return h.hexdigest()
def bind(p):return dict(path=str(Path(p).resolve()),sha256=sha(p))
def write(p,x):p.write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
ledger_path=S/'queue/navigation_n04_ledger.json';lb=ledger_path.read_bytes();ledger=json.loads(lb);assert ledger['status']=='terminal';(A/'ledger_snapshot.json').write_bytes(lb)
original=json.loads((A/'r01_original/report.json').read_bytes());assert hashlib.sha256(lb).hexdigest()==original['source_ledger_sha256']
units=[u for u in ledger['units'] if u['method']=='astra_analytic_route_v1'];good=[u for u in units if u['status']=='consumed' and u.get('integrity_pass') is True];excluded=[u for u in units if u not in good]
assert len(units)==12 and len(good)==5
assert [u['run_id'] for u in good]==[r['run_id'] for r in original['rows']]
resources=[bind(A/'ledger_snapshot.json')];results=[];countfields=['trace_ticks','executed_stop','backup_revalidation_failed','backup_stop_requested','no_prediction_model_backup_pass','transaction_deadline_dropped','inference_deadline_dropped'];alltrace=[]
for unit,reported in zip(good,original['rows']):
 d=S/'runs'/unit['run_id'];receipt=json.loads(Path(unit['receipt']['path']).read_bytes());assert sha(unit['receipt']['path'])==unit['receipt']['sha256'];assert receipt['integrity_pass'] is True
 assessment=json.loads((d/'assessment.json').read_bytes());raw=json.loads((d/'collector_raw.json').read_bytes());contract=json.loads((d/'contract.json').read_bytes());bundle_path=Path(contract['shared_bundle']['path']);assert sha(bundle_path)==contract['shared_bundle']['sha256'];bundle=json.loads(bundle_path.read_bytes())
 assert assessment['capture_integrity_pass'] is True and assessment['task']['task_phase']=='timeout';gate=raw['gate']['source_ns'];terminal=assessment['task']['terminal_source_ns'];assert terminal==raw['gate']['deadline_source_ns']==gate+22_000_000_000
 for f,key in [('fixture.bag','bag_sha256'),('assessment.json','assessment_sha256'),('collector_raw.json','collector_raw_sha256')]:
  b=bind(d/f);assert b['sha256']==reported[key];resources.append(b)
 resources.extend([unit['receipt'],bind(d/'contract.json'),bind(bundle_path)])
 traces=[];odoms=[];tf=[]
 with rosbag.Bag(str(d/'fixture.bag')) as bag:
  for topic,msg,t in bag.read_messages(topics=['/astra/route_runtime/trace','/odom','/tf','/tf_static']):
   if topic=='/astra/route_runtime/trace':
    r=json.loads(msg.data);assert r['run_id']==unit['run_id'];r['bag_record_ns']=t.to_nsec();traces.append(r)
   elif topic=='/odom':
    assert msg.header.frame_id=='odom' and msg.child_frame_id=='base_link';p=msg.pose.pose.position
    odoms.append(dict(source_ns=msg.header.stamp.to_nsec(),bag_record_ns=t.to_nsec(),odom_xy=[p.x,p.y],odom_z=p.z))
   else:
    for x in msg.transforms:
     if x.header.frame_id.lstrip('/')=='map' and x.child_frame_id.lstrip('/')=='odom':
      t2=x.transform.translation;q=x.transform.rotation;tf.append(dict(topic=topic,source_ns=x.header.stamp.to_nsec(),translation=[t2.x,t2.y,t2.z],quaternion=[q.x,q.y,q.z,q.w]))
 assert odoms and traces
 counts=dict(trace_ticks=len(traces),executed_stop=sum(r['executed']==0 for r in traces),backup_revalidation_failed=sum(r.get('stop_reason')=='backup_revalidation_failed' for r in traces),backup_stop_requested=sum(r.get('stop_reason')=='backup_stop_requested' for r in traces),no_prediction_model_backup_pass=sum(r.get('selection_reason')=='no_prediction_model_backup_pass' for r in traces),transaction_deadline_dropped=sum(r.get('transaction_deadline_dropped') is True for r in traces),inference_deadline_dropped=sum(r.get('inference_deadline_dropped') is True for r in traces))
 assert all(counts[k]==reported[k] for k in countfields)
 mx,my,yaw=bundle['map_from_odom_xy_yaw'];goal=raw['gate']['goal_pose'];assert goal==bundle['goal_xy_yaw'];assert tf and all(x['translation']==[mx,my,0.] and abs(math.atan2(2*(x['quaternion'][3]*x['quaternion'][2]+x['quaternion'][0]*x['quaternion'][1]),1-2*(x['quaternion'][1]**2+x['quaternion'][2]**2))-yaw)<1e-12 for x in tf)
 def convert(o):
  x,y=o['odom_xy'];xy=[mx+math.cos(yaw)*x-math.sin(yaw)*y,my+math.sin(yaw)*x+math.cos(yaw)*y]
  return dict(o,map_xy=xy,goal_map_xy=goal[:2],map_goal_distance_m=math.hypot(xy[0]-goal[0],xy[1]-goal[1]),mixed_frame_distance_m=math.hypot(x-goal[0],y-goal[1]))
 exact=[o for o in odoms if o['source_ns']==terminal];assert len(exact)==1
 end=convert(exact[0]);tail=convert(odoms[-1]);assert round(tail['mixed_frame_distance_m'],6)==reported['final_odom_goal_xy_distance_m']
 stamps=[r['stamp_ns'] for r in traces];source=[o['source_ns'] for o in odoms]
 assert all(a<b for a,b in zip(stamps,stamps[1:])) and all(a<b for a,b in zip(source,source[1:]))
 bounds=dict(gate_source_ns=gate,terminal_source_ns=terminal,first_trace_clock_ns=stamps[0],last_trace_clock_ns=stamps[-1],trace_before_gate=sum(s<gate for s in stamps),trace_at_or_after_terminal=sum(s>=terminal for s in stamps),all_trace_phases=dict(collections.Counter(r['phase'] for r in traces)),first_odom_source_ns=source[0],last_odom_source_ns=source[-1],odom_header_gaps_ns=dict(collections.Counter(str(b-a) for a,b in zip(source,source[1:]))),trace_clock_gaps_ns=dict(collections.Counter(str(b-a) for a,b in zip(stamps,stamps[1:]))),tail_after_terminal_ns=source[-1]-terminal,state_source_min_ns=min(r['state_source_ns'] for r in traces),state_source_max_ns=max(r['state_source_ns'] for r in traces))
 semantic=dict(action0_with_nonzero_requested_v=sum(r['executed']==0 and r['v']!=0 for r in traces),action0_with_nonzero_requested_w=sum(r['executed']==0 and r['w']!=0 for r in traces),zero_command_pair_ticks=sum(r['v']==0 and r['w']==0 for r in traces),physical_stop_not_inferred=True)
 for r in traces:alltrace.append(dict(run_id=unit['run_id'],seed=unit['seed'],stamp_ns=r['stamp_ns'],state_source_ns=r['state_source_ns'],bag_record_ns=r['bag_record_ns'],executed=r['executed'],v=r['v'],w=r['w'],stop_reason=r.get('stop_reason'),selection_reason=r.get('selection_reason'),transaction_deadline_dropped=r.get('transaction_deadline_dropped'),inference_deadline_dropped=r.get('inference_deadline_dropped')))
 result=dict(run_id=unit['run_id'],seed=unit['seed'],counts=counts,reported_wrong_mixed_frame_distance_m=reported['final_odom_goal_xy_distance_m'],map_from_odom_xy_yaw=bundle['map_from_odom_xy_yaw'],map_transform_source=contract['shared_bundle'],bag_map_odom_transform_count=len(tf),bag_map_odom_transform_example=tf[0],terminal_odom=end,bag_tail_odom=tail,bounds=bounds,semantics=semantic)
 results.append(result);print(json.dumps({'run_id':unit['run_id'],'ticks':len(traces),'wrong_r01':reported['final_odom_goal_xy_distance_m'],'map_distance_at_terminal':end['map_goal_distance_m'],'map_distance_at_bag_tail':tail['map_goal_distance_m'],'tail_lag_ns':bounds['tail_after_terminal_ns']},ensure_ascii=False),flush=True)
summary={k:sum(r['counts'][k] for r in results) for k in countfields};expected={'trace_ticks':'trace_ticks','executed_stop':'executed_stop_ticks','backup_revalidation_failed':'backup_revalidation_failed_ticks','backup_stop_requested':'backup_stop_requested_ticks','transaction_deadline_dropped':'transaction_deadline_dropped_ticks'}
assert all(summary[k]==original[v] for k,v in expected.items());assert len(alltrace)==2190
with (A/'all2190_trace_ticks.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=list(alltrace[0]));w.writeheader();w.writerows(alltrace)
ex=[]
for u in excluded:
 item=dict(run_id=u['run_id'],seed=u['seed'],status=u['status'],integrity_pass=u.get('integrity_pass'),used_in_any_count_or_endpoint=False)
 if u.get('receipt'):
  p=Path(u['receipt']['path']);assert sha(p)==u['receipt']['sha256'];receipt=json.loads(p.read_bytes());resources.append(u['receipt']);item.update(receipt=u['receipt'],receipt_error=receipt['error'],task_outcome=receipt['task_outcome'],assessment_present=(S/'runs'/u['run_id']/'assessment.json').exists())
 else:assert u['status']=='retired_unrun'
 ex.append(item)
assert len([x for x in ex if x['status']=='consumed'])==1 and len([x for x in ex if x['status']=='retired_unrun'])==6
for name in ['navigation_n04_trace_attribution.py','report.json','valid_analytic_ticks.csv']:resources.append(bind(A/'r01_original'/name))
write(A/'recomputed.json',dict(status='r01_counts_confirmed_endpoint_coordinate_bug_proven',rows=results,totals=summary,excluded_analytic=ex,all48_status_counts=dict(collections.Counter(u['status'] for u in ledger['units'])),resources=resources,old_files_unchanged=True,geometry_assessment_reclassified=False,ROS_started=False))
assert ledger_path.read_bytes()==lb
