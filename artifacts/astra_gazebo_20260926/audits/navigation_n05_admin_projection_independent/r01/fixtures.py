"""Actual administration source under isolated synthetic files; never invoke real N05 main."""
from pathlib import Path
from types import SimpleNamespace as NS
import contextlib,csv,hashlib,io,json,os,tempfile,copy
A=Path(__file__).resolve().parent;source=A/'source_snapshot.py';env={'__file__':str(source),'__name__':'offline_fixture'};exec(compile(source.read_text(),str(source),'exec'),env)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
write=lambda p,x:p.write_text(json.dumps(x)+'\n')
HEADERS=[['namespace','split','purpose','seed','run_id','status','source_binding'],['run_id','pid','process_group','command_json','cwd','started_utc','ended_utc','exit_code','heartbeat_path','receipt_path','bindings_sha256'],['run_id','split','candidate_id','method','scene_id','layout_id','seed','training_seed','status','started_utc','ended_utc','exit_code','code_sha256','config_sha256','model_sha256','report_path','raw_path']]
NAMES=['seed_registry.csv','run_manifest.csv','EXPERIMENTS.csv'];results={}
def csvwrite(p,fields,rows):
 with p.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
def setup(s,consumed=False):
 for sub in ['queue','contracts','runs','workspace/scripts/astra_gazebo']:(s/sub).mkdir(parents=True)
 release=s/'contracts/navigation_n05_release.json';write(release,{}) # intentionally unqualified release accepted by old implementation
 rsha=sha(release);schedule=[dict(run_id='synthetic_n05_'+str(i),seed=4200+i//2,method=('control','treatment')[i%2],scene_id='scene'+str(i//2),layout_id='layout') for i in range(24)]
 units=[dict(run_id=r['run_id'],seed=r['seed'],method=r['method'],status='retired_unrun') for r in schedule]
 ledger=dict(stage='astra_navigation_n05_preview_selection',status='terminal',release_sha256=rsha,units=units)
 write(s/'contracts/navigation_n05_preview_selection_design.json',dict(schedule=schedule));write(s/'queue/navigation_n05_ledger.json',ledger)
 old=[dict(namespace='historical',split='development',purpose='old',seed='101',run_id='historical_keep',status='consumed_terminal',source_binding='original'),dict(run_id='historical_keep',pid='123',command_json='["literal,$(),\\n"]',receipt_path='/frozen/old'),dict(run_id='historical_keep',method='old_method',status='terminal_timeout',scene_id='old_scene')]
 rows=[[old[0]]+[dict(namespace='n05',split='development',purpose='pair',seed=str(r['seed']),run_id=r['run_id'],status='reserved_exact_n05_release',source_binding=rsha) for r in schedule],[old[1]],[old[2]]]
 for name,header,rr in zip(NAMES,HEADERS,rows):csvwrite(s/name,header,rr)
 (s/'workspace/scripts/astra_gazebo/run_navigation_n05.py').write_text('# synthetic runner only\n')
 env.update(S=s,LEDGER=s/'queue/navigation_n05_ledger.json',RELEASE=release,DESIGN=s/'contracts/navigation_n05_preview_selection_design.json',OUTPUT=s/'queue/navigation_n05_admin_closure.json',os=os)
 if consumed:
  u=units[0];u.update(status='consumed',integrity_pass=True,task_outcome='timeout');root=s/'runs'/u['run_id'];root.mkdir()
  report=root/'assessment.json';write(report,dict(run_id=u['run_id'],seed=u['seed'],method=u['method'],task={'task_phase':'timeout'},capture_integrity_pass=True))
  receipt=dict(stage=ledger['stage'],run_id=u['run_id'],seed=u['seed'],method=u['method'],attempt=1,release_sha256=rsha,retry_allowed=False,integrity_pass=True,error=None,task_outcome='timeout',report=dict(path=str(report),sha256=sha(report)),exit_code=0)
  rp=root/'receipt.json';write(rp,receipt);u['receipt']=dict(path=str(rp),sha256=sha(rp));write(env['LEDGER'],ledger)
 return ledger,schedule,old

def run():
 with contextlib.redirect_stdout(io.StringIO()):env['main']()
def table(p):
 with p.open(newline='') as f:return list(csv.DictReader(f))
def failure(fn):
 try:fn()
 except Exception as e:return {'exception':type(e).__name__,'message':str(e)}
 raise AssertionError('expected failure')
with tempfile.TemporaryDirectory(prefix='admin_',dir=str(A)) as temp:
 top=Path(temp)
 s=top/'baseline';s.mkdir();ledger,schedule,old=setup(s,True);run();after={n:sha(s/n) for n in NAMES};again=failure(run)
 for n,o in zip(NAMES,old):
  historical=next(r for r in table(s/n) if r['run_id']=='historical_keep');assert all(historical[k]==v for k,v in o.items())
 assert after=={n:sha(s/n) for n in NAMES}
 report=json.loads(env['OUTPUT'].read_bytes());assert(report['consumed'],report['valid_assessment'],report['retired_unrun'])==(1,1,23)
 results['baseline_retired_failure_inclusive_historical_fields_and_no_duplicate_reexecution']=dict(pass_=True,repeat_error=again,counts=[1,1,23],release_contents={})
 # first replace succeeds; second fails; rerun cannot recover.
 s=top/'partial';s.mkdir();setup(s);before={n:sha(s/n) for n in NAMES};calls=[]
 def fail_second(src,dst):
  calls.append(str(dst))
  if len(calls)==2:raise OSError('SYNTHETIC injected second os.replace failure')
  return os.replace(src,dst)
 env['os']=NS(replace=fail_second,fsync=os.fsync);exc=failure(run);env['os']=os;after={n:sha(s/n) for n in NAMES};retry=failure(run)
 assert before[NAMES[0]]!=after[NAMES[0]] and all(before[n]==after[n] for n in NAMES[1:]) and not env['OUTPUT'].exists()
 results['partial_commit_unrecoverable']=dict(reproduced=True,error=exc,retry=retry,changed_tables=[n for n in NAMES if before[n]!=after[n]],closure_exists=False,pending_files=[p.name for p in s.glob('.*.n05-pending')],registry_terminal_count=sum(r['status']=='retired_unrun_after_terminal_n05' for r in table(s/NAMES[0])))
 # preexisting second temporary path leaves newly written first temporary path.
 s=top/'pending';s.mkdir();setup(s);before={n:sha(s/n) for n in NAMES};collision=s/('.'+NAMES[1]+'.n05-pending');collision.write_text('existing error evidence');exc=failure(run);collision.unlink();retry=failure(run);assert before=={n:sha(s/n) for n in NAMES}
 results['staging_collision_leaves_retry_blocking_partial']=dict(reproduced=True,error=exc,retry=retry,unchanged_csv=True,pending_files=[p.name for p in s.glob('.*.n05-pending')])
 # genuine ledger declaration can disagree with unbound design method/seed and stage.
 s=top/'identity';s.mkdir();ledger,schedule,_=setup(s);ledger['stage']='wrong_stage';schedule[0]['seed']=999;schedule[0]['method']='wrong_method';write(env['LEDGER'],ledger);write(env['DESIGN'],{'schedule':schedule});run();out=json.loads(env['OUTPUT'].read_bytes());assert out['stage']=='wrong_stage'
 results['unqualified_release_wrong_stage_and_schedule_seed_method_accepted']=dict(reproduced=True,release_contents={},output_stage=out['stage'],schedule_seed=999,ledger_seed=ledger['units'][0]['seed'],schedule_method='wrong_method',ledger_method=ledger['units'][0]['method'])
 # report current bytes and receipt canonical location/task evidence are not checked.
 s=top/'receipt';s.mkdir();ledger,_,_=setup(s,True);u=ledger['units'][0];rp=Path(u['receipt']['path']);receipt=json.loads(rp.read_bytes());report_path=Path(receipt['report']['path']);report_path.write_text('{}\n');receipt['stage']='wrong_stage';receipt['retry_allowed']=True;receipt['task_outcome']='complete';outside=s/'outside_receipt.json';write(outside,receipt);u['receipt']=dict(path=str(outside),sha256=sha(outside));write(env['LEDGER'],ledger);run();row=next(r for r in table(s/'EXPERIMENTS.csv') if r['run_id']==u['run_id']);assert row['status']=='terminal_complete'
 results['external_receipt_stale_report_wrong_stage_retry_and_task_mismatch_accepted']=dict(reproduced=True,projected_status=row['status'],ledger_task=u['task_outcome'],receipt_task=receipt['task_outcome'],report_hash_stale=True)
 # consumed failure remains represented; absent receipt rejected before writes.
 s=top/'failed';s.mkdir();ledger,_,_=setup(s,True);u=ledger['units'][0];rp=Path(u['receipt']['path']);receipt=json.loads(rp.read_bytes());u.update(integrity_pass=False,task_outcome=None);receipt.update(integrity_pass=False,error='synthetic bootstrap error',task_outcome=None,report=None);write(rp,receipt);u['receipt']['sha256']=sha(rp);write(env['LEDGER'],ledger);run();out=json.loads(env['OUTPUT'].read_bytes());assert out['integrity_failed']==1 and out['retired_unrun']==23;results['bootstrap_failure_preserved']=True
 s=top/'missing';s.mkdir();ledger,_,_=setup(s,True);del ledger['units'][0]['receipt'];write(env['LEDGER'],ledger);before={n:sha(s/n) for n in NAMES};exc=failure(run);assert before=={n:sha(s/n) for n in NAMES};results['missing_receipt_rejected_before_write']=exc
result=dict(status='r01_admin_projection_requires_revision',source_sha256=sha(source),results=results,transport='Entirely synthetic tempfile roots. Actual saved main called only after constants redirected; no real ledger/CSV/runtime touched.',N05_actual_failure_claim=False,no_ROS=True)
write(A/'fixture_results.json',result);print(json.dumps(result,indent=2))
