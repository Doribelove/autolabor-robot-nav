"""Focused r03 tests; synthetic files and exact extracted preview block. No whole replay."""
from pathlib import Path
from types import SimpleNamespace as NS
import ast,copy,hashlib,json,tempfile
A=Path(__file__).resolve().parent; S=A.parents[2]; source=A/'source_snapshot.py'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
bind=lambda p:dict(path=str(p),sha256=sha(p))
write=lambda p,x:p.write_text(json.dumps(x)+'\n')
tree=ast.parse(source.read_text());tree.body=[n for n in tree.body if not(isinstance(n,ast.Import) and any(a.name in ('rosbag','run_navigation_n05') for a in n.names))]
env={'__file__':str(source),'__name__':'offline','rosbag':NS(),'manager':NS(NAME='astra_navigation_n05_preview_selection')};exec(compile(tree,str(source),'exec'),env)
# Exact audit assignment+if block, without any surrounding bag read.
replay=next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='replay_one')
block=next(n for n in ast.walk(replay) if isinstance(n,ast.If) and ast.dump(n.test)==ast.dump(ast.parse("unit['method']==METHODS[1]",mode='eval').body))
fun=ast.parse("def check_preview(data,unit):\n eligible=Counter()\n audit=data.get('preview_selection_audit')\n").body[0]
fun.body += [copy.deepcopy(block),ast.Return(value=ast.Call(func=ast.Name(id='dict',ctx=ast.Load()),args=[ast.Name(id='eligible',ctx=ast.Load())],keywords=[]))]
ast.fix_missing_locations(fun);exec(compile(ast.Module(body=[fun],type_ignores=[]),str(source)+':extracted_preview','exec'),env)
checks={};findings={}
def reject(name,fn):
 try:fn()
 except (ValueError,TypeError,KeyError,IndexError,FileNotFoundError) as e:checks[name]={'rejected':True,'exception':type(e).__name__,'message':str(e)};return
 raise AssertionError('must reject '+name)
audit=dict(schema='astra_analytic_preview_eligibility_n05_v1',method=env['METHODS'][1],chosen=1,proposed=1,reason=None,backup_mask=[True]*8,selection_eligibility=[True]*8,full_preview_clearances_m=[1.]*8,original_analytic=1,true_backup_mask_changed=False,physical_safety_certified=False)
candidate=dict(stamp_ns=2_000_000_000,mask=[True]*8,full_preview_clearances_m=[1.]*8,preview_selection_audit=audit)
unit={'method':env['METHODS'][1]};fn=lambda c:env['check_preview'](c,unit)
assert fn(candidate)['moving_eligible_count_sum']==7;checks['valid_moving_preview']=True
for field,values in [('chosen',[-1,True,8,1.0,None]),('original_analytic',[-1,True,8,1.0,None]),('proposed',[-1,True,8,1.0]),('schema',['wrong',None]),('method',['wrong',None]),('physical_safety_certified',[True,None]),('true_backup_mask_changed',[True,None])]:
 for value in values:
  c=copy.deepcopy(candidate);c['preview_selection_audit'][field]=value;reject(field+'='+repr(value),lambda c=c:fn(c))
c=copy.deepcopy(candidate);c['mask']=[False]*8;c['preview_selection_audit'].update(backup_mask=[False]*8,selection_eligibility=[False]*8,chosen=0,proposed=None,original_analytic=0,reason='no_prediction_model_backup_pass');assert fn(c)['selected_stop_decisions']==1;checks['valid_empty_backup_fallback']=True
c=copy.deepcopy(candidate);c['mask']=[False]+[True]*7;c['full_preview_clearances_m']=[-1.]*8;c['preview_selection_audit'].update(backup_mask=c['mask'],selection_eligibility=[False]*8,full_preview_clearances_m=[-1.]*8,chosen=0,proposed=None,original_analytic=1,reason='no_full_preview_eligible_action');assert fn(c)['selected_stop_decisions']==1;checks['valid_no_moving_eligible_fallback']=True
with tempfile.TemporaryDirectory(prefix='receipt_',dir=str(A)) as tmp:
 s=Path(tmp);env['S']=s;(s/'runs/toy').mkdir(parents=True);(s/'queue').mkdir();path=s/'runs/toy/receipt.json';rsha='a'*64
 base=dict(stage=env['manager'].NAME,run_id='toy',seed=42,method=unit['method'],attempt=1,release_sha256=rsha,retry_allowed=False,integrity_pass=False,task_outcome=None,error='RuntimeError: synthetic failure',report=None)
 u=dict(run_id='toy',seed=42,method=unit['method'],integrity_pass=False,task_outcome=None)
 def put(r,p=path):write(p,r);u['receipt']=bind(p)
 def check():return env['validate_consumed_receipt'](u,rsha)
 put(base);assert check()==base;checks['failed_error_receipt_accepted']=True
 prep=s/'queue/toy_preparation_failure.json';put(base,prep);assert check()==base;checks['valid_preparation_failure_path']=True
 put(base);del u['receipt'];reject('missing_receipt_binding',check)
 put(base);u['receipt']['sha256']='0'*64;reject('stale_receipt_sha',check)
 wrong=s/'runs/toy/other.json';put(base,wrong);reject('wrong_receipt_path',check)
 for key,value in [('run_id','other'),('stage','wrong'),('seed',43),('method','wrong'),('attempt',2),('release_sha256','b'*64),('retry_allowed',True),('integrity_pass',True),('task_outcome','timeout')]:
  r=dict(base);r[key]=value;put(r);reject('receipt_'+key,check)
 r=dict(base,error=None);put(r);reject('failed_without_error_or_report',check)
 r=dict(base,integrity_pass=True,task_outcome='timeout',error=None);u.update(integrity_pass=True,task_outcome='timeout');put(r);assert check()==r;checks['valid_integrity_receipt_identity']=True
 r['error']='unexpected';put(r);reject('valid_with_error',check)
 u.update(integrity_pass=False,task_outcome=None)
 for name,value in [('empty_report',{}),('missing_report_file',{'path':str(s/'does_not_exist.json'),'sha256':'0'*64})]:
  r=dict(base,error=None,report=value);put(r);assert check()==r;findings[name]={'accepted':True,'description':'failed receipt with no error passes solely because report is non-null, without report path/hash/schema verification'}
 report_path=s/'runs/toy/assessment.json';write(report_path,{'run_id':'toy','capture_integrity_pass':False,'task':{'task_phase':'fault'}})
 r=dict(base,error='RuntimeError',report=bind(report_path));put(r);report_path.write_text('{}\n');assert check()==r;findings['stale_failed_report_hash']={'accepted':True}
# Actual closed receipt compatibility only: first closed row per method, exact receipt validator.
env['S']=S; ledger=json.loads((S/'queue/navigation_n05_ledger.json').read_bytes());actual=[]
for method in env['METHODS']:
 u=next(u for u in ledger['units'] if u['method']==method and u.get('receipt'))
 receipt=env['validate_consumed_receipt'](u,ledger['release_sha256']);actual.append({'unit':u,'receipt_binding':bind(Path(u['receipt']['path'])),'passed':True})
result={'status':'r03_preview_repair_pass_failed_report_binding_gap','reviewed_source_sha256':sha(source),'checks':checks,'remaining_findings':findings,'actual_closed_receipt_compatibility':actual,'actual_scope':'first ledger-closed receipt per method only; no bag replay or matrix result','pure_extracted_preview_block':True,'no_ROS':True}
write(A/'fixture_results.json',result);print(json.dumps({'checks':len(checks),'remaining_findings':findings,'actual_closed_receipts':len(actual)},indent=2))
