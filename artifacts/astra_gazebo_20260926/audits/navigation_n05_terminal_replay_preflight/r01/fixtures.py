"""Actual saved replay functions on synthetic files/fake bag only; no live data interpretation."""
from pathlib import Path
from types import SimpleNamespace as NS
import ast,hashlib,json,tempfile,contextlib,io
A=Path(__file__).resolve().parent;source=A/'source_snapshot.py';tree=ast.parse(source.read_text());tree.body=[n for n in tree.body if not(isinstance(n,ast.Import) and any(a.name=='rosbag' for a in n.names))];env={'__file__':str(source),'__name__':'offline_replay_fixture','rosbag':NS()};exec(compile(tree,str(source),'exec'),env)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
write=lambda p,x:p.write_text(json.dumps(x)+'\n')
results={}
with tempfile.TemporaryDirectory(prefix='n05_static_',dir=str(A)) as temp:
 s=Path(temp);d=s/'runs/synthetic_treatment';d.mkdir(parents=True);env['S']=s
 unit=dict(run_id='synthetic_treatment',seed=42,method=env['METHODS'][1],integrity_pass=True)
 bundle=s/'common_route.json';write(bundle,dict(map_from_odom_xy_yaw=[-5.4,0.,0.],goal_xy_yaw=[5.,0.,0.]))
 contract=dict(run_id=unit['run_id'],seed=42,method=unit['method'],scene_id='synthetic_scene',shared_bundle=dict(path=str(bundle),sha256=sha(bundle)))
 assessment=dict(run_id=unit['run_id'],seed=42,method=unit['method'],capture_integrity_pass=True,task={'task_phase':'timeout','terminal_source_ns':23_000_000_000},valid_completed=False,restricted_goal_time_s=22.,safety_within_declared_engineering_scope_pass=True)
 raw=dict(start_ns=1_000_000_000,end_ns=23_000_000_000,gate={'source_ns':1_000_000_000,'goal_pose':[5.,0.,0.]})
 for name,obj in [('contract.json',contract),('assessment.json',assessment),('collector_raw.json',raw)]:write(d/name,obj)
 (d/'fixture.bag').write_bytes(b'SYNTHETIC ONLY; bag reader is stubbed')
 receipt=dict(release_sha256='a'*64,run_id=unit['run_id'],seed=42,method=unit['method'],attempt=1,integrity_pass=True,report=dict(path=str(d/'assessment.json'),sha256=sha(d/'assessment.json')),artifacts=[dict(path=str(d/n),sha256=sha(d/n)) for n in ['assessment.json','collector_raw.json','contract.json','fixture.bag']])
 write(d/'receipt.json',receipt);unit['receipt']=dict(path=str(d/'receipt.json'),sha256=sha(d/'receipt.json'))
 def odom(frame='odom',x=0.):return NS(header=NS(frame_id=frame,stamp=NS(to_nsec=lambda:23_000_000_000)),child_frame_id='base_link',pose=NS(pose=NS(position=NS(x=x,y=0.))))
 trace=dict(stamp_ns=2_000_000_000,run_id=unit['run_id'],executed=0,stop_reason='prepublish_input_health_drop',transaction_deadline_dropped=True)
 candidate=dict(stamp_ns=2_000_000_000,mask=[True]*8) # absent treatment audit
 messages=[('/odom',odom(),None),('/astra/route_runtime/trace',NS(data=json.dumps(trace)),None),('/astra/route_runtime/candidates',NS(data=json.dumps(candidate)),None)]
 class Bag:
  def __init__(self,*a,**k):pass
  def __enter__(self):return self
  def __exit__(self,*a):pass
  def read_messages(self,**k):return iter(messages)
 env['rosbag'].Bag=Bag
 row=env['replay_one'](unit,'a'*64)
 assert row['candidate_decisions']==1 and row['treatment_moving_backup_count_sum']==0
 results['missing_treatment_audit_silently_accepted']=True
 assert row['transaction_deadline_drop_ticks']==0
 results['true_deadline_boolean_lost_when_reason_differs']=True
 assessment['valid_completed']=True;assessment['restricted_goal_time_s']=.01;write(d/'assessment.json',assessment)
 assert sha(d/'assessment.json')!=receipt['report']['sha256']
 row=env['replay_one'](unit,'a'*64);assert row['valid_completed'] is True and row['task_phase']=='timeout' and row['restricted_goal_time_s']==.01
 results['stale_bound_assessment_SHA_and_contradictory_task_accepted']=True
 messages[0]=('/odom',odom('map'),None);row=env['replay_one'](unit,'a'*64)
 results['wrong_odom_frame_accepted']=True
 raw['start_ns']=24_000_000_000;write(d/'collector_raw.json',raw)
 row=env['replay_one'](unit,'a'*64);assert row['trace_ticks']==0
 results['negative_formal_duration_accepted_then_zero_trace_possible']=True
 release=s/'release.json';release.write_text('{}');ledger=s/'ledger.json';write(ledger,dict(status='terminal',release_sha256=sha(release),units=[dict(status='consumed',integrity_pass=False) for _ in range(24)]));env.update(LEDGER=ledger,RELEASE=release,OUT=s/'output')
 with contextlib.redirect_stdout(io.StringIO()):env['main']()
 aggregate=json.loads((s/'output/report.json').read_bytes());assert aggregate['all_safety_scoped_pass'] is True and aggregate['integrity_valid']==0
 results['24_invalid_unscheduled_units_and_empty_safety_true']=True
result=dict(status='synthetic_static_counterexamples_reproduced',source_sha256=sha(source),actual_saved_function_execution=True,transport='fake bag only, wholly synthetic temp files, no real episode content copied',results=results,N05_actual_failure_claim=False,no_ROS=True)
(A/'fixture_results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
