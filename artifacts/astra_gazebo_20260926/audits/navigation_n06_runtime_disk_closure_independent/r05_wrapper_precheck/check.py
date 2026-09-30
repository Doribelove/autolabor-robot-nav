"""Independent frozen r05 wrapper observations only; real gzserver is not run."""
import copy,hashlib,importlib.util,json,pathlib,shlex,subprocess,sys,xml.etree.ElementTree as ET
S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';P=W/'scripts/astra_gazebo';A=pathlib.Path(__file__).parent
D=S/'audits/navigation_n06_runtime_closure_r01/environment_r05_attempt01';READ={};checks=[]
def read(p):
 p=pathlib.Path(p).resolve();b=p.read_bytes();READ[str(p)]={'path':str(p),'sha256':hashlib.sha256(b).hexdigest()};return b
def js(p):return json.loads(read(p))
def bound(r):
 raw=read(r['path']);assert hashlib.sha256(raw).hexdigest()==r['sha256'];return raw
def write(n,x):(A/n).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
def check(name,ok,detail=None):checks.append({'name':name,'pass':bool(ok),'detail':detail});assert ok,(name,detail)
def bindings(x):
 if isinstance(x,dict):
  if isinstance(x.get('path'),str) and isinstance(x.get('sha256'),str):yield x
  for v in x.values():yield from bindings(v)
 elif isinstance(x,list):
  for v in x:yield from bindings(v)
report=js(S/'audits/navigation_n06_manager_r05/implementation_report.json');assert READ[str((S/'audits/navigation_n06_manager_r05/implementation_report.json').resolve())]['sha256']=='bd4e3a6c2700df586ff8894d2796a8d0f8783e30590b9e14ac1076e63ef2d111'
obs=json.loads(bound(report['wrapper_observations']));spec=json.loads(bound(obs['execution_environment']))
for r in bindings(obs):bound(r)
check('frozen_wrapper_report_and_transitive_direct_bindings_match',True)
module=P/'navigation_n06_environment_r05.py';bound(next(r for r in report['sources'] if r['path']==str(module)))
m=importlib.util.spec_from_file_location('review_r05_environment',str(module));policy=importlib.util.module_from_spec(m);m.loader.exec_module(policy)
expected=policy.validate_spec(spec);check('observed_spec_model_URI_nonempty_loopback',expected['GAZEBO_MODEL_DATABASE_URI']=='http://127.0.0.1:9')
prepared=js(S/'contracts/navigation_n06_prepared_r01/prepared.json');units={u['run_id']:u for u in prepared['units']}
assert len(obs['units'])==len(units)==8;assert {u['run_id'] for u in obs['units']}==set(units)
outputs=[]
for row in obs['units']:
 unit=units[row['run_id']]
 for k in ('world','robot_description'):
  assert row[k]==unit[k];xml=ET.fromstring(bound(unit[k]));assert not any((n.text or '').strip() for n in xml.iter('uri'))
  if k=='robot_description':assert not xml.findall('.//mesh')
 shim=pathlib.Path(row['substitution']['only_executable']['path']);script=bound(row['substitution']['only_executable']).decode()
 assert script=='#!/usr/bin/python3\nimport json,os,sys\nprint(json.dumps({"argv":sys.argv,"environment":dict(os.environ),"gazebo_started":False},sort_keys=True))\n'
 assert sorted(p.name for p in shim.parent.iterdir())==['gzserver']
 cmd=js(D/(row['run_id']+'.command.json'));bound(cmd['stdout']);bound(cmd['stderr']);assert cmd['returncode']==0
 assert json.loads(bound(cmd['stdout']))==row['observed']
 assert cmd['argv']==[obs['wrapper']['path']]+shlex.split(row['node']['args'])
 original=dict(expected,ASTRA_N06_RUNTIME_CLOSURE_SHA256='0'*64,ASTRA_N06_CHILD_NONCE='a'*48,ROS_HOME=str(S/'runs'/row['run_id']/'ros_home'),ROS_LOG_DIR=str(S/'runs'/row['run_id']/'roslog'))
 want=dict(original,PATH=str(shim.parent)+':'+original['PATH']);assert cmd['environment']==want
 # Exact actual wrapper, with the sole final-exec shim. No main server path runs.
 r=subprocess.run(cmd['argv'],env=want,cwd=str(W),stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=15)
 (A/(row['run_id']+'.stdout')).write_bytes(r.stdout);(A/(row['run_id']+'.stderr')).write_bytes(r.stderr)
 assert r.returncode==0;out=json.loads(r.stdout);assert out==row['observed']
 normalized=dict(out['environment']);normalized['PATH']=original['PATH'];assert normalized==row['normalized_real_exec_environment']
 assert out['argv'][0]==str(shim) and out['argv'][1:]==shlex.split(row['node']['args'])+['-s','/opt/ros/noetic/lib/libgazebo_ros_paths_plugin.so','-s','/opt/ros/noetic/lib/libgazebo_ros_api_plugin.so']
 assert normalized['GAZEBO_MODEL_DATABASE_URI']=='http://127.0.0.1:9'
 assert not set(normalized)&policy.FORBIDDEN
 # Validate current node scope: root namespace yields no ROS_NAMESPACE addition;
 # future env_args/nonroot namespaces need separately observed derivation.
 assert row['node']['namespace']=='/'
 outputs.append({'run_id':row['run_id'],'exact_stdout_payload_equal':True,'final_environment_equal':True,'final_URI_loopback':True,'gzserver_started':False})
check('eight_actual_frozen_wrapper_invocations_reproduce_exact_payload_and_final_environment',True,outputs)
negative=js(D/'empty_model_database_negative.command.json');bound(negative['stdout']);bound(negative['stderr']);r=subprocess.run(negative['argv'],env=negative['environment'],cwd=str(W),stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=15)
(A/'old_empty_URI.stdout').write_bytes(r.stdout);(A/'old_empty_URI.stderr').write_bytes(r.stderr)
check('old_empty_URI_negative_preserved',r.returncode==0 and json.loads(r.stdout)['environment']['GAZEBO_MODEL_DATABASE_URI']=='http://models.gazebosim.org')
data={'runtime':{'execution_environment':spec},'release':{'runtime_closure':{'sha256':'0'*64}}};clean=policy.expected_environment(spec,'0'*64)
assert policy.validate_current(data,clean)==clean
rejected=[]
for key in ['LD_PRELOAD','LD_AUDIT','PYTHONHOME','PYTHONUSERBASE','__EGL_VENDOR_LIBRARY_FILENAMES']:
 dirty=dict(clean);dirty[key]='/not-loaded/synthetic-value'
 try:policy.validate_current(data,dirty)
 except ValueError:rejected.append(key)
 else:raise AssertionError(key)
check('old_five_synthetic_injections_rejected_by_frozen_pure_environment_gate',len(rejected)==5,rejected)
check('disk_vs_live_scope_retained',obs['ros_gazebo_started'] is False and obs['dynamic_closure_sha_and_nonce_are_explicit_placeholders'] is True)
write('results.json',{'status':'frozen_r05_wrapper_observations_reproduced_final_disk_closure_pending','checks':checks,'independent':True,'full_disk_closure_review':False,'resources':sorted(READ.values(),key=lambda r:r['path']),'ROS_or_Gazebo_started':False,'manager_gate_controlflow_review_owned_by':'historical_audit'})
print(json.dumps({'checks':len(checks),'actual_wrapper_shim_replays':9,'all_pass':True,'full_disk_closure':'pending author freeze'}))
