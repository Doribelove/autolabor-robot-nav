"""Independent r05 startup and actual orchestration checks; synthetic FS only."""
import ast,contextlib,copy,hashlib,json,os,subprocess,sys,tempfile
from pathlib import Path
from unittest.mock import Mock,patch
A=Path(__file__).resolve().parent;S=A.parents[2];P=S/'workspace/scripts/astra_gazebo'
sys.path[:0]=[str(P),str(P/'tests')]
import test_navigation_n06_manager_r05 as t
import navigation_n06_bootstrap_r05 as boot
q=t.q;m=t.manager;e=t.exec_env;checks=[];details={}
def bind(p):
 p=Path(p).resolve();b=p.read_bytes();return {'path':str(p),'sha256':hashlib.sha256(b).hexdigest(),'bytes':len(b)}
def b2(p):return {k:v for k,v in bind(p).items() if k!='bytes'}
def add(name,**kw):checks.append(dict(name=name,passed=True,**kw))
def reject(fn):
 try:fn()
 except (ValueError,KeyError,TypeError,FileNotFoundError) as ex:return type(ex).__name__+': '+str(ex)
 raise AssertionError('counterexample unexpectedly accepted')
actual_spec=json.loads((S/'audits/navigation_n06_runtime_closure_r01/environment_r05_attempt01/execution_environment.json').read_text())
data={'runtime':{'execution_environment':actual_spec},'release':{'runtime_closure':{'sha256':'1'*64}}}
base=e.expected_environment(actual_spec,'1'*64)
for revision,count,sha in [('r01',352,'50a5af06ff0be1ddd7d94ae17af3b108c41a213a855be51a7d4656cf9b3f8c8b'),('r02',733,'76bd9a6256f927668eb46a5a6e366c6ab3735f4c4fe190480402c37d21dc0415'),('r03',764,'cb48b726ab31680eb2ee61c2721f5e2ccfa169c49f33ed16ac82409096b3fa87'),('r04',791,'47ac2100bdc1d454861fd9ca8aefecc25469626e1c62ee8eda03015735c1d2a6'),('r05',866,'d04cb1fcccc1ccfc0de3d8d0ef131ec9bfd36d0784f99d1ab35e3544581eef2c')]:
 path=S/('audits/navigation_n06_manager_'+revision)/'source_evidence_manifest.json';assert bind(path)['sha256']==sha
 doc=json.loads(path.read_text());rows={r['path']:r for key in ('owned_sources','read_only_dependencies','author_evidence') for r in doc[key]}
 assert len(rows)==count and all(bind(r['path'])['sha256']==r['sha256'] for r in rows.values())
 add(revision+'_all_frozen_bindings_unchanged',resources=count)

def fixture(root):
 runtime=root/'runtime.json';runtime.write_text(json.dumps({'execution_environment':actual_spec,'python_executable':str((e.WORKSPACE/'.venv/bin/python').resolve()),'python_prefix':str(e.WORKSPACE/'.venv'),'python_no_user_site':True}))
 resources=[b2(P/name) for name in ('navigation_n06_bootstrap_r05.py','navigation_n06_environment_r05.py','navigation_n06_entry_r05.sh','run_navigation_n06_manager_r05.py')]+[b2(runtime)]
 path=root/'synthetic_release_shaped_input.json';path.write_text(json.dumps({'schema':'astra_navigation_n06_exact_release_v1','stage':'astra_navigation_n06_lookahead_static_pilot','runtime_closure':b2(runtime),'resources':resources}))
 return path,runtime
def command(path,sha=None):
 return ['/usr/bin/env','-i']+[k+'='+v for k,v in e.BOOTSTRAP_ENV.items()]+['/bin/bash','--noprofile','--norc',str(P/'navigation_n06_entry_r05.sh'),'--verify','--release',str(path),'--release-sha256',sha or b2(path)['sha256'],'--inspect-bootstrap']
with tempfile.TemporaryDirectory() as temp:
 root=Path(temp);path,runtime=fixture(root);marker=root/'must_not_exist';hook=root/'shell_hook.sh';hook.write_text('touch '+str(marker)+'\n')
 (root/'sitecustomize.py').write_text('from pathlib import Path\nPath('+repr(str(marker))+').touch()\n')
 # No LD loader injection is executed. These harmless hooks would create a marker if loaded.
 parent=dict(os.environ,BASH_ENV=str(hook),PYTHONPATH=str(root),PYTHONHOME=str(root/'absent'),PYTHONUSERBASE=str(root))
 result=subprocess.run(command(path),env=parent,capture_output=True,text=True,timeout=10)
 assert result.returncode==0,result.stderr
 out=json.loads(result.stdout);assert out['manager_imported']is False and out['execution_performed']is False and not marker.exists()
 assert out['environment']==e.expected_environment(actual_spec,b2(runtime)['sha256'])
 details['bootstrap_positive']={'returncode':0,'inspection':out,'shell_python_hook_marker_absent':True}
 add('actual_outer_clean_entry_precedes_shell_python_hooks_and_manager_import')
 original=path.read_bytes();failures=[]
 for variant in ('hash','entry_missing','entry_wrong_hash','duplicate_resource','runtime_unbound','stage','bad_python_prefix','bad_environment_key'):
  path.write_bytes(original);value=json.loads(original);rt=json.loads(runtime.read_text())
  if variant=='hash':cmd=command(path,'0'*64)
  else:
   if variant=='entry_missing':value['resources']=value['resources'][1:]
   elif variant=='entry_wrong_hash':value['resources'][0]['sha256']='0'*64
   elif variant=='duplicate_resource':value['resources'].append(value['resources'][0])
   elif variant=='runtime_unbound':value['resources']=value['resources'][:-1]
   elif variant=='stage':value['stage']='astra_navigation_n05'
   else:
    if variant=='bad_python_prefix':rt['python_prefix']='/usr'
    else:rt['execution_environment']['values']['LD_PRELOAD']='not_executed'
    runtime.write_text(json.dumps(rt));value['runtime_closure']=b2(runtime);value['resources'][-1]=b2(runtime)
   path.write_text(json.dumps(value));cmd=command(path)
  r=subprocess.run(cmd,capture_output=True,text=True,timeout=10);assert r.returncode!=0 and not marker.exists()
  failures.append({'case':variant,'returncode':r.returncode,'last_error':r.stderr.splitlines()[-1]})
  # Restore actual frozen spec for the next isolated case.
  path,runtime=fixture(root);original=path.read_bytes()
 details['bootstrap_faults']=failures;add('eight_actual_bootstrap_faults_before_manager',cases=len(failures))
 direct=['/usr/bin/python3','-I','-S',str(P/'navigation_n06_bootstrap_r05.py'),'--verify','--release',str(path),'--release-sha256',b2(path)['sha256'],'--inspect-bootstrap']
 r=subprocess.run(direct,env=dict(e.BOOTSTRAP_ENV,UNDECLARED='synthetic'),capture_output=True,text=True,timeout=10)
 assert r.returncode!=0 and 'entry required before manager imports' in r.stderr;add('direct_extra_environment_rejected_before_manager')
 # Exercise main's actual execve branch, but intercept that final call in this isolated child.
 driver=root/'capture_exec.py';driver.write_text('import os,runpy,sys,json\n'
  'def capture(exe,argv,env):\n print(json.dumps({"exe":exe,"argv":argv,"env":env,"manager_imported":any(x.startswith(("rospy","rosbag","run_navigation_n06_manager")) for x in sys.modules)}));raise SystemExit(0)\n'
  'os.execve=capture\nsys.argv='+repr([str(P/'navigation_n06_bootstrap_r05.py'),'--verify','--release',str(path),'--release-sha256',b2(path)['sha256']])+'\nrunpy.run_path(sys.argv[0],run_name="__main__")\n')
 r=subprocess.run(['/usr/bin/python3','-I','-S',str(driver)],env=e.BOOTSTRAP_ENV,capture_output=True,text=True,timeout=10)
 assert r.returncode==0,r.stderr
 out=json.loads(r.stdout);assert not out['manager_imported'] and out['env']==e.expected_environment(actual_spec,b2(runtime)['sha256'])
 assert out['argv'][1]==str(P/'run_navigation_n06_manager_r05.py') and out['argv'][2]=='--verify'
 details['captured_execve']=out;add('actual_main_execve_uses_bound_environment_before_manager_import')

injections=[]
for key in sorted(e.FORBIDDEN|{'UNDECLARED'}):
 altered=dict(base);altered[key]='not_loaded_in_dictionary_only'
 with patch.dict(os.environ,altered,clear=True),patch.object(q,'publish') as pub,patch.object(q,'consume') as consume,patch.object(subprocess,'Popen') as spawn:
  error=reject(lambda:m.live_preflight(data));assert 'environment mismatch' in error;pub.assert_not_called();consume.assert_not_called();spawn.assert_not_called()
 injections.append(key)
for key in base:
 altered=dict(base);altered[key]+='changed'
 with patch.dict(os.environ,altered,clear=True),patch.object(q,'publish') as pub,patch.object(q,'consume') as consume,patch.object(subprocess,'Popen') as spawn:
  error=reject(lambda:m.live_preflight(data));pub.assert_not_called();consume.assert_not_called();spawn.assert_not_called()
add('actual_live_preflight_all_forbidden_unknown_and_changed_fields_before_writes',injected_keys=injections,changed_value_count=len(base))
for key in ('LD_PRELOAD','LD_AUDIT','PYTHONHOME','PYTHONUSERBASE','__EGL_VENDOR_LIBRARY_FILENAMES'):
 contaminated=dict(base,**{key:'old_synthetic_not_loaded'})
 assert key in dict(contaminated,ROS_HOME='old_run_home',ROS_LOG_DIR='old_run_log')
 assert 'environment mismatch' in reject(lambda:e.validate_current(data,contaminated))
add('five_old_inherited_environment_counterexamples_closed_by_exact_gate')

with t.JournalTests().journal() as root,patch.dict(os.environ,base,clear=True):
 m.EVENTS.rmdir();d=dict(data,release_binding={'sha256':'1'*64},rows=q.prepared_api.schedule(),prepared={'units':[]})
 with patch.object(q,'verify_release',return_value=d),patch.object(m,'live_preflight'),patch.object(t.process,'enable_subreaper'),patch.object(subprocess,'Popen',side_effect=OSError('independent worker spawn boundary')) as spawn:
  assert m.supervise(root/'synthetic.json','1'*64)==3
 assert spawn.call_args.kwargs['env']==base and spawn.call_args.kwargs['start_new_session'] is True
 state=q.strict_json(m.LEDGER.read_bytes());assert state['consumed_units']==0 and all(x['status']=='retired_unrun' for x in state['units'])
 add('actual_supervisor_worker_spawn_fixed_environment_failure_retires_all')

real_run=subprocess.run
unit_cases=[]
for kind in ('normal','parent_changed_before_launch','parent_changed_before_assessment','assessment_exception'):
 with tempfile.TemporaryDirectory() as temp,contextlib.ExitStack() as stack:
  root=Path(temp);workspace=root/'workspace';workspace.mkdir();(root/'runs').mkdir();inputs=root/'inputs';inputs.mkdir()
  stack.enter_context(patch.object(e,'WORKSPACE',workspace));stack.enter_context(patch.object(m,'STAGE',root))
  spec=t.environment_fixture();d={'runtime':{'execution_environment':spec},'release':{'resources':[],'runtime_closure':{'sha256':'1'*64}},'release_binding':{'path':'/synthetic/not-read','sha256':'1'*64},'review':{},'registry_baseline':b'','rows':[]}
  localbase=e.expected_environment(spec,'1'*64);stack.enter_context(patch.dict(os.environ,localbase,clear=True))
  def file(name,value):
   path=inputs/name;q.publish(path,value);return q.binding(path)
  unit=dict(q.prepared_api.schedule()[0]);unit['contract']=file('contract.json',{'scene_manifest':file('scene.yaml',b'synthetic'),'shared_bundle':{},'wall_cap_s':30.})
  for key in ('launch','world','robot_description','gate_expectation'):unit[key]=file(key,b'synthetic')
  side={key:file(key,{}) for key in ('conversion_manifest','converted_sdf')};(root/'seed_registry.csv').write_bytes(b'synthetic')
  stack.enter_context(patch.object(q,'check_registry'));stack.enter_context(patch('run_passive_sweep_q05.archive'))
  stack.enter_context(patch.object(t.process,'checked_parent',return_value=True));stack.enter_context(patch.object(t.process,'proc_row',return_value={'pid':987654,'pgid':987654,'start_ticks':'synthetic'}))
  stack.enter_context(patch.object(t.process,'cleanup'))
  def port_boundary():
   if kind=='parent_changed_before_launch':os.environ['LD_AUDIT']='dictionary_only_not_loaded'
  stack.enter_context(patch.object(t.process,'assert_ports_free',side_effect=port_boundary))
  def closure(*args):
   if kind=='parent_changed_before_assessment':os.environ['PYTHONUSERBASE']='dictionary_only_not_loaded'
  stack.enter_context(patch.object(m.evidence,'closure',side_effect=closure))
  observations=[]
  def assess(*args):
   # Actual benign child inherits the context; comparison contains no arbitrary parent secrets.
   child=real_run(['/usr/bin/python3','-I','-c','import os,json;print(json.dumps(dict(os.environ)))'],capture_output=True,text=True,timeout=10)
   assert child.returncode==0,child.stderr;observations.append(json.loads(child.stdout))
   if kind=='assessment_exception':raise RuntimeError('independent assessor exception')
   return {'capture_integrity_pass':True,'task':{'task_phase':'timeout'}}
  assess_mock=stack.enter_context(patch.object(m.evidence,'assess',side_effect=assess))
  original_popen=subprocess.Popen;launch_calls=[]
  def fake_ros_only(argv,*args,**kwargs):
   if argv[0]=='/opt/ros/noetic/bin/roslaunch':
    launch_calls.append({'argv':argv,'env':kwargs['env']});fake=Mock(pid=987654,returncode=0);fake.poll.return_value=0;return fake
   assert argv[0]=='/usr/bin/python3';return original_popen(argv,*args,**kwargs)
  stack.enter_context(patch.object(subprocess,'Popen',side_effect=fake_ros_only))
  bound,receipt=m.execute_unit(unit,side,d,Mock(),Mock(signal=None),{'pid':os.getpid(),'start_ticks':'synthetic'})
  assert q.json_bound(bound)==receipt
  if kind=='parent_changed_before_launch':assert not launch_calls and assess_mock.call_count==0 and receipt['integrity_pass']is False
  else:
   assert len(launch_calls)==1
   expected=dict(localbase,ROS_HOME=str(root/'runs'/unit['run_id']/'ros_home'),ROS_LOG_DIR=str(root/'runs'/unit['run_id']/'roslog'),ASTRA_N06_CHILD_NONCE=launch_calls[0]['env']['ASTRA_N06_CHILD_NONCE'])
   assert launch_calls[0]['env']==expected
   if kind=='parent_changed_before_assessment':assert not observations and assess_mock.call_count==0 and receipt['integrity_pass']is False
   else:
    assert observations==[expected] and dict(os.environ)==localbase
    assert receipt['integrity_pass'] is (kind=='normal')
  unit_cases.append({'kind':kind,'launch_calls':len(launch_calls),'assessment_calls':assess_mock.call_count,'real_python_children':len(observations),'receipt':receipt,'temporary_paths_only':True})
details['execute_unit_cases']=unit_cases;add('actual_execute_unit_unstubbed_environment_helper_and_benign_assessment_children',cases=4,actual_python_children=2)

old=ast.parse((P/'run_navigation_n06_manager_r04.py').read_text());new=ast.parse((P/'run_navigation_n06_manager_r05.py').read_text())
get=lambda tree,name:next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name==name)
assert ast.dump(get(old,'terminal_receipt'))==ast.dump(get(new,'terminal_receipt'))
assert ast.dump(get(old,'reconcile_worker_exit'))==ast.dump(get(new,'reconcile_worker_exit'))
late=[]
for phase in ('complete','timeout','fault','stopped'):
 with t.JournalTests().journal() as root:
  rows=q.prepared_api.schedule();units=[dict(r,contract=q.binding(q.prepared_api.PREPARED/r['run_id']/'contract.json')) for r in rows]
  d={'release_binding':{'sha256':'1'*64},'rows':rows,'prepared':{'units':units}}
  state=q.initial_state('1'*64,rows);m.persist(state,'allocation');q.consume(state,0);m.persist(state,'consume')
  t.receipt(rows[0],outcome=phase);run=root/'runs'/rows[0]['run_id'];before=(run/'assessment.json').read_bytes()
  m.reconcile_worker_exit(d,-9);state=q.strict_json(m.LEDGER.read_bytes());saved=q.json_bound(state['units'][0]['receipt'])
  assert state['status']=='terminal' and state['consumed_units']==1 and [x['status'] for x in state['units']]==['consumed']+['retired_unrun']*7
  assert saved['integrity_pass']is False and saved['task_outcome']==phase and '-9' in saved['error'] and saved['assessment']==q.binding(run/'assessment.json') and before==(run/'assessment.json').read_bytes()
  assert m.replay_state(d)==state
  snap={str(p):(p.read_bytes(),p.stat().st_ino,p.stat().st_mtime_ns) for p in root.rglob('*') if p.is_file()};m.reconcile_worker_exit(d,-9)
  assert snap=={str(p):(p.read_bytes(),p.stat().st_ino,p.stat().st_mtime_ns) for p in root.rglob('*') if p.is_file()}
  late.append({'phase':phase,'integrity_pass':False,'one_consumed_seven_retired':True,'assessment_unchanged':True,'repeat_noop':True})
details['late_assessment']=late;add('r04_unchanged_AST_and_four_whole_supervisor_late_assessment_cases',cases=4)
out={'schema':'independent_n06_r05_environment_checks_v1','checks':checks,'details':details,'synthetic_not_simulation':True,'actual_release_created':False,'real_CSV_written':False,'ROS_started':False,'seed_registered':False}
(A/'focused_result.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps({'groups':len(checks),'result':bind(A/'focused_result.json')},indent=2))
