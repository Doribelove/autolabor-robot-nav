"""Execute exact saved r02 functions with synthetic files/fake bag; never read runtime results."""
from pathlib import Path
from types import SimpleNamespace as NS
import ast,hashlib,json,tempfile,contextlib,io,copy
A=Path(__file__).resolve().parent; source=A/'source_snapshot.py'
tree=ast.parse(source.read_text()); tree.body=[n for n in tree.body if not(isinstance(n,ast.Import) and any(a.name in ('rosbag','run_navigation_n05') for a in n.names))]
env={'__file__':str(source),'__name__':'offline_replay_fixture','rosbag':NS(),'manager':NS()};exec(compile(tree,str(source),'exec'),env)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
write=lambda p,x:p.write_text(json.dumps(x)+'\n')
results={};negative_findings={}
with tempfile.TemporaryDirectory(prefix='n05_static_',dir=str(A)) as temp:
 s=Path(temp); d=s/'runs/synthetic_treatment'; d.mkdir(parents=True);env['S']=s
 unit=dict(run_id='synthetic_treatment',seed=42,method=env['METHODS'][1],integrity_pass=True)
 bundle=s/'common_route.json'; write(bundle,dict(map_from_odom_xy_yaw=[-5.4,0.,0.],goal_xy_yaw=[5.,0.,0.]))
 contract=dict(run_id=unit['run_id'],seed=42,method=unit['method'],scene_id='synthetic_scene',shared_bundle=dict(path=str(bundle),sha256=sha(bundle)))
 assessment=dict(run_id=unit['run_id'],seed=42,method=unit['method'],capture_integrity_pass=True,task={'task_phase':'timeout','terminal_source_ns':23_000_000_000},valid_completed=False,restricted_goal_time_s=22.,safety_within_declared_engineering_scope_pass=True)
 raw=dict(start_ns=1_000_000_000,end_ns=23_000_000_000,gate={'source_ns':1_000_000_000,'goal_pose':[5.,0.,0.]})
 (d/'fixture.bag').write_bytes(b'SYNTHETIC ONLY; bag reader is stubbed')
 def sync():
  for name,obj in [('contract.json',contract),('assessment.json',assessment),('collector_raw.json',raw)]:write(d/name,obj)
  receipt=dict(release_sha256='a'*64,run_id=unit['run_id'],seed=42,method=unit['method'],attempt=1,integrity_pass=True,error=None,task_outcome='timeout',report=dict(path=str(d/'assessment.json'),sha256=sha(d/'assessment.json')),artifacts=[dict(path=str(d/n),sha256=sha(d/n)) for n in ['assessment.json','collector_raw.json','contract.json','fixture.bag']])
  write(d/'receipt.json',receipt);unit['receipt']=dict(path=str(d/'receipt.json'),sha256=sha(d/'receipt.json'))
  return receipt
 def odom(frame='odom',x=0.):return NS(header=NS(frame_id=frame,stamp=NS(to_nsec=lambda:23_000_000_000)),child_frame_id='base_link',pose=NS(pose=NS(position=NS(x=x,y=0.))))
 trace=dict(stamp_ns=2_000_000_000,run_id=unit['run_id'],executed=0,stop_reason='prepublish_input_health_drop',transaction_deadline_dropped=True)
 audit=dict(schema='astra_analytic_preview_eligibility_n05_v1',method=env['METHODS'][1],chosen=1,proposed=1,reason=None,backup_mask=[True]*8,selection_eligibility=[True]*8,full_preview_clearances_m=[1.]*8,original_analytic=1,true_backup_mask_changed=False,physical_safety_certified=False)
 candidate=dict(stamp_ns=2_000_000_000,mask=[True]*8,full_preview_clearances_m=[1.]*8,preview_selection_audit=audit)
 messages=[]
 def set_messages(c=candidate,t=trace,o=None):
  messages[:]=[('/odom',odom() if o is None else o,None),('/astra/route_runtime/trace',NS(data=json.dumps(t)),None),('/astra/route_runtime/candidates',NS(data=json.dumps(c)),None)]
 class Bag:
  def __init__(self,*a,**k):pass
  def __enter__(self):return self
  def __exit__(self,*a):pass
  def read_messages(self,**k):return iter(messages)
 env['rosbag'].Bag=Bag
 def run():return env['replay_one'](unit,'a'*64)
 def rejects(name,fn):
  try:fn()
  except (ValueError,KeyError,IndexError,TypeError) as exc: results[name]=dict(rejected=True,exception=type(exc).__name__,message=str(exc));return
  raise AssertionError('must reject '+name)
 sync();set_messages(); row=run();assert row['candidate_decisions']==1 and row['terminal_map_goal_xy_distance_m']==10.4
 results['valid_synthetic_baseline']=True
 assert row['transaction_deadline_drop_ticks']==0 and row['transaction_deadline_dropped_flag_ticks']==1
 results['deadline_reason_and_flag_distinct']=True
 c=copy.deepcopy(candidate);del c['preview_selection_audit'];set_messages(c);rejects('missing_treatment_audit',run)
 set_messages();assessment['valid_completed']=True;write(d/'assessment.json',assessment);rejects('stale_assessment_binding',run);assessment['valid_completed']=False;sync()
 set_messages(o=odom('map'));rejects('wrong_terminal_frame',run)
 set_messages();raw['start_ns']=24_000_000_000;sync();rejects('negative_formal_window',run);raw['start_ns']=1_000_000_000;sync()
 raw['gate']['source_ns']+=1;sync();rejects('gate_source_mismatch',run);raw['gate']['source_ns']-=1;sync()
 assessment['task']['terminal_source_ns']-=1;sync();rejects('terminal_source_mismatch',run);assessment['task']['terminal_source_ns']+=1;sync()
 for f in ['assessment.json','collector_raw.json','contract.json','fixture.bag']:
  p=d/f; original=p.read_bytes();p.write_bytes(original+b' ');rejects('bound_'+f,run);p.write_bytes(original)
 set_messages();messages.append(messages[0]);rejects('duplicate_terminal_odom',run)
 set_messages();messages.extend([('/astra/route_runtime/trace',NS(data=json.dumps(dict(trace,stamp_ns=k))),None) for k in [999999999,1000000000,22999999999,23000000000]])
 row=run();assert row['trace_ticks']==3;results['halfopen_exact_boundaries']=True
 for key,value in [('chosen',-1),('chosen',True),('original_analytic',-1),('schema','wrong'),('method','wrong')]:
  c=copy.deepcopy(candidate);c['preview_selection_audit'][key]=value;set_messages(c);row=run();negative_findings[key+'='+str(value)]=dict(accepted=True,changed_decisions=row['treatment_selection_changed_decisions'])
 # full release API is exercised with a stub that records exact arguments; no live closure hashed.
 release=s/'release.json';write(release,{});ledger=s/'ledger.json'; prepared={'units':[dict(run_id='row'+str(i),seed=i//2,method=env['METHODS'][i%2]) for i in range(24)]};calls=[]
 def verify(path,digest):calls.append((str(path),digest));return {},prepared,None
 env['manager']=NS(NAME='N05',verify_release=verify)
 base=dict(stage='N05',status='terminal',release_sha256=sha(release),units=[dict(u,status='consumed',integrity_pass=False) for u in prepared['units']]);write(ledger,base);env.update(LEDGER=ledger,RELEASE=release,OUT=s/'output')
 with contextlib.redirect_stdout(io.StringIO()):env['main']()
 report=json.loads((s/'output/report.json').read_bytes());assert report['all_safety_scoped_pass'] is None and report['integrity_failed']==24 and calls==[(str(release),sha(release))]
 results['full_release_validator_called_and_empty_safety_unknown']=True
 for name,mutate in [('active_ledger',lambda x:x.update(status='active')),('wrong_schedule',lambda x:x['units'][0].update(run_id='wrong')),('wrong_unit_status',lambda x:x['units'][0].update(status='reserved'))]:
  val=copy.deepcopy(base);mutate(val);write(ledger,val);env['OUT']=s/name;rejects(name,env['main'])
result=dict(status='r02_focused_regression_pass_with_preview_schema_index_gaps',source_sha256=sha(source),actual_saved_function_execution=True,transport='wholly synthetic temp files and fake bag; manager.verify_release stub isolates caller control flow; no real release verification executed',passed_checks=results,negative_findings=negative_findings,N05_actual_failure_claim=False,no_ROS=True)
(A/'fixture_results.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
