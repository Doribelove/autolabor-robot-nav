"""Author fault tests on isolated filesystem; full release API explicitly stubbed.
The independently reviewed r04 receipt validator executes as exact saved AST.
"""
from pathlib import Path
from types import SimpleNamespace as NS
import ast,contextlib,copy,csv,hashlib,io,json,os,tempfile,traceback
A=Path(__file__).resolve().parent;S=A.parents[2];SOURCE=S/'queue/close_navigation_n05_r02.py'
env={'__file__':str(SOURCE),'__name__':'offline_projection_tests'};exec(compile(SOURCE.read_bytes(),str(SOURCE),'exec'),env)
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();write=lambda p,x:p.write_text(json.dumps(x)+'\n')
class Proxy:
 def __init__(self,**changes):self.changes=changes
 def __getattr__(self,k):return self.changes.get(k,getattr(os,k))
def csvwrite(p,fields,rows):
 with p.open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=fields);w.writeheader();w.writerows(rows)
def setup(stage):
 for sub in ['queue','contracts','runs','workspace/scripts/astra_gazebo']:(stage/sub).mkdir(parents=True)
 runner=stage/'workspace/scripts/astra_gazebo/run_navigation_n05.py';runner.write_text('# synthetic runner\n')
 schedule=[dict(run_id='synthetic_n05_'+str(i),seed=4200+i//2,method=('control','treatment')[i%2],scene_id='scene'+str(i//2),layout_id='layout') for i in range(24)]
 design=stage/'contracts/navigation_n05_preview_selection_design.json';write(design,dict(stage='astra_navigation_n05_preview_selection',schedule=schedule))
 prepared={'units':[]}
 for row in schedule:
  p=stage/'contracts'/(row['run_id']+'.json');write(p,dict(row,stage='astra_navigation_n05_preview_selection'))
  prepared['units'].append(dict(row,files=[env['binding'](p)]))
 release={'stage':'astra_navigation_n05_preview_selection','status':'root_reviewed_release','execution_authorized':True,'resources':[env['binding'](design),env['binding'](runner)]+[u['files'][0] for u in prepared['units']]}
 releasepath=stage/'contracts/navigation_n05_release.json';write(releasepath,release);rsha=sha(releasepath)
 units=[dict(run_id=r['run_id'],seed=r['seed'],method=r['method'],status='consumed' if i<2 else 'retired_unrun') for i,r in enumerate(schedule)]
 for i,u in enumerate(units[:2]):
  root=stage/'runs'/u['run_id'];root.mkdir();(root/'contract.json').write_bytes(Path(prepared['units'][i]['files'][0]['path']).read_bytes())
  report=dict(run_id=u['run_id'],seed=u['seed'],method=u['method'],capture_integrity_pass=i==0,task={'task_phase':'timeout' if i==0 else 'fault'})
  write(root/'assessment.json',report);write(root/'allocated.json',{'started_utc':'2026-09-26T00:00:00+00:00'});write(root/'process.json',{'pid':123+i,'pgid':123+i});write(root/'collector_raw.json',{'synthetic':True})
  receipt=dict(stage=release['stage'],run_id=u['run_id'],seed=u['seed'],method=u['method'],attempt=1,release_sha256=rsha,retry_allowed=False,integrity_pass=i==0,error=None if i==0 else 'synthetic infrastructure failure',task_outcome=report['task']['task_phase'],report=env['binding'](root/'assessment.json'),artifacts=[env['binding'](p) for p in sorted(root.iterdir())],exit_code=0 if i==0 else 3)
  write(root/'receipt.json',receipt);u.update(receipt=env['binding'](root/'receipt.json'),integrity_pass=receipt['integrity_pass'],task_outcome=receipt['task_outcome'])
 ledger=dict(stage=release['stage'],status='terminal',release_sha256=rsha,consumed_units=2,units=units);ledgerpath=stage/'queue/navigation_n05_ledger.json';write(ledgerpath,ledger)
 old=[dict(namespace='historical',split='development',purpose='old',seed='101',run_id='historical_keep',status='consumed_terminal',source_binding='original'),dict(run_id='historical_keep',pid='123',command_json='["literal,$(),\\n"]',receipt_path='/frozen/old'),dict(run_id='historical_keep',method='old_method',status='terminal_timeout',scene_id='old_scene')]
 rows=[[old[0]]+[dict(namespace='n05',split='development',purpose='pair',seed=str(r['seed']),run_id=r['run_id'],status='reserved_exact_n05_release',source_binding=rsha) for r in schedule],[old[1]],[old[2]]]
 for name,header,rr in zip(env['TABLES'],env['HEADERS'],rows):csvwrite(stage/name,header,rr)
 calls=[]
 def verify(path,digest):
  calls.append((str(path),digest))
  if digest!=rsha or json.loads(path.read_bytes())!=release:raise ValueError('synthetic full-release validator rejection')
  return release,prepared,{}
 manager=NS(NAME=release['stage'],identities=lambda:[(r['run_id'],r['seed'],r['method']) for r in schedule],verify_release=verify)
 r04=S/'audits/navigation_n05_terminal_replay_r04.py';tree=ast.parse(r04.read_bytes());functions=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name in ('sha','validate_consumed_receipt')]
 venv={'S':stage,'manager':manager,'json':json,'Path':Path,'hashlib':hashlib};exec(compile(ast.Module(body=functions,type_ignores=[]),str(r04),'exec'),venv)
 deps=(manager,venv['validate_consumed_receipt'],[env['binding'](r04),env['binding'](runner)])
 return NS(stage=stage,ledger=ledger,ledgerpath=ledgerpath,releasepath=releasepath,design=design,prepared=prepared,deps=deps,calls=calls,old=old)
def run(f):
 with contextlib.redirect_stdout(io.StringIO()):return env['close'](f.stage,f.deps)
def hashes(f):return {n:sha(f.stage/n) for n in env['TABLES']}
def error(fn):
 try:fn()
 except (OSError,ValueError,KeyError) as e:return {'type':type(e).__name__,'message':str(e)}
 raise AssertionError('expected fail closed')
def inject_second():
 counter=[0]
 def replace(a,b):
  counter[0]+=1
  if counter[0]==2:raise OSError('synthetic second replace')
  os.replace(a,b)
 env['os']=Proxy(replace=replace)
def closed(f):
 out=f.stage/'queue/navigation_n05_admin_closure_r02.json';v=json.loads(out.read_bytes());assert(v['consumed'],v['valid_assessment'],v['integrity_failed'],v['retired_unrun'],v['experiment_rows'],v['run_manifest_rows'])==(2,1,1,22,24,2)
 for n,o in zip(env['TABLES'],f.old):
  with (f.stage/n).open(newline='') as z:r=next(r for r in csv.DictReader(z) if r['run_id']=='historical_keep')
  assert all(r[k]==val for k,val in o.items())
 return v
results={}
def case(name,fn):
 with tempfile.TemporaryDirectory(prefix=name+'_',dir=str(A)) as tmp:
  env['os']=os;f=setup(Path(tmp))
  try:detail=fn(f);results[name]={'pass':True,'detail':detail}
  except Exception as e:results[name]={'pass':False,'error':repr(e),'traceback':traceback.format_exc()}
  finally:env['os']=os

def happy(f):
 v=run(f);closed(f);paths=[f.stage/n for n in env['TABLES']]+list((f.stage/'queue/navigation_n05_admin_transaction_r02').iterdir())+[f.stage/'queue/navigation_n05_admin_closure_r02.json']
 before={str(p):(sha(p),p.stat().st_mtime_ns,p.stat().st_ino) for p in paths}
 def forbidden(*a,**k):raise AssertionError('repeat attempted a write operation')
 env['os']=Proxy(replace=forbidden,link=forbidden,rename=forbidden,fsync=forbidden);assert run(f)==v
 assert before=={str(p):(sha(p),p.stat().st_mtime_ns,p.stat().st_ino) for p in paths};assert len(f.calls)==2
 return {'counts':[2,1,1,22],'repeat_same_hash_mtime_inode':True,'full_release_API_calls':2}
case('success_and_readonly_repeat',happy)

def second(f):
 before=hashes(f);inject_second();e=error(lambda:run(f));partial=hashes(f);assert[before[n]!=partial[n] for n in env['TABLES']]==[True,False,False];assert not(f.stage/'queue/navigation_n05_admin_closure_r02.json').exists();env['os']=os;run(f);closed(f);return {'injected':e,'recovered':True,'partial_changed':['seed_registry.csv']}
case('second_replace_recovery',second)

def first(f):
 before=hashes(f)
 def fail(*a):raise OSError('synthetic first replace')
 env['os']=Proxy(replace=fail);e=error(lambda:run(f));assert hashes(f)==before;env['os']=os;run(f);closed(f);return e
case('first_replace_recovery',first)

def link_failure(f):
 def fail(*a):raise OSError('synthetic closure publish')
 env['os']=Proxy(link=fail);e=error(lambda:run(f));assert not(f.stage/'queue/navigation_n05_admin_closure_r02.json').exists();after=hashes(f)
 def no_replace(*a):raise AssertionError('allafter recovery replaced table')
 env['os']=Proxy(replace=no_replace);run(f);closed(f);assert after==hashes(f);return e
case('all_tables_committed_closure_publish_failure',link_failure)

def linked_then_error(f):
 def fail(a,b):os.link(a,b);raise OSError('synthetic crash after complete closure publication')
 env['os']=Proxy(link=fail);e=error(lambda:run(f));assert(f.stage/'queue/navigation_n05_admin_closure_r02.json').exists();env['os']=os;run(f);closed(f);return e
case('complete_closure_link_then_error',linked_then_error)

def directory_sync(f):
 hit=[False]
 def fail(fd):
  if os.readlink('/proc/self/fd/'+str(fd))==str(f.stage) and not hit[0]:hit[0]=True;raise OSError('synthetic table directory fsync')
  os.fsync(fd)
 env['os']=Proxy(fsync=fail);e=error(lambda:run(f));env['os']=os;run(f);closed(f);return e
case('table_directory_fsync_failure',directory_sync)

def txn_rename(f):
 before=hashes(f)
 def fail(*a):raise OSError('synthetic transaction directory publish')
 env['os']=Proxy(rename=fail);e=error(lambda:run(f));assert hashes(f)==before;assert list((f.stage/'queue').glob('.navigation_n05_admin_r02-*'));env['os']=os;run(f);closed(f);return e
case('orphan_staging_nonblocking_recovery',txn_rename)

def tamper_snapshot(f):
 inject_second();error(lambda:run(f));env['os']=os;p=f.stage/'queue/navigation_n05_admin_transaction_r02/after_run_manifest.csv';p.chmod(0o644);p.write_bytes(p.read_bytes()+b'corrupt');before=hashes(f);e=error(lambda:run(f));assert hashes(f)==before;return e
case('snapshot_corruption_reject_no_more_writes',tamper_snapshot)

def foreign_csv(f):
 inject_second();error(lambda:run(f));env['os']=os;p=f.stage/'EXPERIMENTS.csv';p.write_bytes(p.read_bytes()+b'foreign history addition\n');before=hashes(f);e=error(lambda:run(f));assert hashes(f)==before;return e
case('unrelated_historical_edit_reject_not_overwrite',foreign_csv)

def tamper_receipt(f):
 inject_second();error(lambda:run(f));env['os']=os;p=Path(f.ledger['units'][0]['receipt']['path']);p.write_bytes(p.read_bytes()+b' ');before=hashes(f);e=error(lambda:run(f));assert hashes(f)==before;return e
case('receipt_change_during_recovery_reject',tamper_receipt)

def before_reject(mutator):
 def test(f):
  mutator(f);before=hashes(f);e=error(lambda:run(f));assert hashes(f)==before;assert not(f.stage/'queue/navigation_n05_admin_transaction_r02').exists();return e
 return test
case('active_ledger',before_reject(lambda f:(f.ledger.update(status='active'),write(f.ledgerpath,f.ledger))))
case('wrong_ledger_stage',before_reject(lambda f:(f.ledger.update(stage='wrong'),write(f.ledgerpath,f.ledger))))
case('unqualified_release',before_reject(lambda f:write(f.releasepath,{})))
case('changed_design_binding',before_reject(lambda f:write(f.design,{'stage':'wrong','schedule':[]})))
case('wrong_method_in_ordered_ledger',before_reject(lambda f:(f.ledger['units'][0].update(method='wrong'),write(f.ledgerpath,f.ledger))))
case('wrong_consumed_count',before_reject(lambda f:(f.ledger.update(consumed_units=3),write(f.ledgerpath,f.ledger))))
case('retired_has_attempt_root',before_reject(lambda f:(f.stage/'runs'/f.ledger['units'][2]['run_id']).mkdir()))
case('bad_failed_report',before_reject(lambda f:Path(f.ledger['units'][1]['receipt']['path']).with_name('assessment.json').write_text('{}\n')))

def preexisting(f):
 p=f.stage/'EXPERIMENTS.csv'
 with p.open(newline='') as z:rows=list(csv.DictReader(z))
 rows.append(dict(run_id=f.ledger['units'][0]['run_id'],status='historical_identity_must_not_replace'));csvwrite(p,env['HEADERS'][2],rows)
case('existing_N05_experiment_row',before_reject(preexisting))

def matrix_mode(all_valid):
 def test(f):
  for i,(u,pu) in enumerate(zip(f.ledger['units'],f.prepared['units'])):
   root=f.stage/'runs'/u['run_id']
   if not all_valid:
    # Independent zero-consumption terminal administrative shape.
    if root.exists():
     import shutil
     shutil.rmtree(root)
    u.clear();u.update(run_id=pu['run_id'],seed=pu['seed'],method=pu['method'],status='retired_unrun')
    continue
   root.mkdir(exist_ok=True);(root/'contract.json').write_bytes(Path(pu['files'][0]['path']).read_bytes())
   report=dict(run_id=u['run_id'],seed=u['seed'],method=u['method'],capture_integrity_pass=True,task={'task_phase':'timeout'})
   write(root/'assessment.json',report);write(root/'process.json',{'pid':123+i,'pgid':123+i})
   receipt=dict(stage=f.ledger['stage'],run_id=u['run_id'],seed=u['seed'],method=u['method'],attempt=1,release_sha256=f.ledger['release_sha256'],retry_allowed=False,integrity_pass=True,error=None,task_outcome='timeout',report=env['binding'](root/'assessment.json'),artifacts=[env['binding'](p) for p in sorted(root.iterdir()) if p.name!='receipt.json'],exit_code=0)
   write(root/'receipt.json',receipt);u.update(status='consumed',receipt=env['binding'](root/'receipt.json'),integrity_pass=True,task_outcome='timeout')
  f.ledger['consumed_units']=24 if all_valid else 0;write(f.ledgerpath,f.ledger)
  v=run(f);assert (v['consumed'],v['valid_assessment'],v['integrity_failed'],v['retired_unrun'])==((24,24,0,0) if all_valid else (0,0,0,24));return {'counts':[v[k] for k in ['consumed','valid_assessment','integrity_failed','retired_unrun']]}
 return test
case('all24_valid_physical_timeouts_retained',matrix_mode(True))
case('zero_consumed24retired',matrix_mode(False))

result=dict(status='pass' if all(r['pass'] for r in results.values()) else 'failed',source_sha256=sha(SOURCE),checks=len(results),results=results,scope='Author fault-injection tests only. Fake stage and explicit manager.verify_release interface stub; exact real r04 receipt validator AST. No real release validation or administration invoked.',no_ROS=True)
write(A/'test_results.json',result);print(json.dumps(result,indent=2));assert result['status']=='pass'
