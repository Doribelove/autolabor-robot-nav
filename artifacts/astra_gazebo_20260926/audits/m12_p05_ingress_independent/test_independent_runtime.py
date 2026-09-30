"""Independent synthetic P05 terminal-path fixtures. No ROS or simulator calls."""
import ast,copy,hashlib,importlib.util,json,threading,time,types
from pathlib import Path
import pytest
HERE=Path(__file__).resolve().parent
S=HERE.parents[1];P=S/'workspace/src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p05.py'
spec=importlib.util.spec_from_file_location('p05_frozen',P);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
TREE=ast.parse(P.read_bytes());RT=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='runtime')
KEYS=('models_matches_dense_same_stamp','links_matches_dense_same_stamp','actual_joint_positions_agree_with_dense_source_pose','actual_odom_twist_agrees_with_dense_base_link')

def ingress_row(ns,seq,v,w):
 return {'ns':ns,'t':ns/1e9,'seq':seq,'frame':'world','statuses':[{'name':'astra_physical_command_ingress','hardware_id':'simulation-only','level':0,'message':'accepted_queued','value_count':7,'values':{'input_topic':'cmd_vel','requested_linear_x':str(v),'requested_angular_z':str(w),'accepted':'true','disposition':'accepted_queued','backend_fault':'false','activation_time_s':str(ns/1e9)}}]}

def fixture():
 c={'world_fixture':{'max_step_s':.002,'fixed_world_models':['ground_plane']},'model_name':'m2','total_schedule_s':.04}
 raw={'schedule_start_ns':100000000,'source_status':[{'statuses':[{'values':{'state':'complete'}}]}],'commands_sent':[{'phase':'moving','ns':100000000,'t':.1,'v':1.,'w':.1},{'phase':'terminal_zero','ns':140000000,'t':.14,'v':0.,'w':0.}],'seed':123,'run_id':'synthetic_P05_no_simulation'}
 for k in ('models','links','odom','joints','targets','backend_status','commanded_efforts','dense_models','dense_links','contacts'):
  period=2000000 if k in ('dense_models','dense_links','contacts') else 20000000;first=2000000 if k in ('models','links','dense_models','dense_links','contacts') else 20000000
  raw[k]=[{'ns':ns} for ns in range(first,144000001,period)]
 raw['command_ingress']=[ingress_row(r['ns'],i,r['v'],r['w']) for i,r in enumerate(raw['commands_sent'])]
 physics={'success':True,'pause':True,'time_step':.002,'status_message':''};world={'success':True,'sim_time':.144,'model_names':['ground_plane','m2'],'status_message':''}
 return raw,c,physics,world
class Clock:
 def __init__(self):self.at=10.;self.on_sleep=None;self.sleeps=[]
 def monotonic(self):return self.at
 def sleep(self,s):
  self.sleeps.append(s);self.at+=s
  if self.on_sleep:self.on_sleep(self)
class Lock:
 def __init__(self):self.depth=0
 def __enter__(self):self.depth+=1
 def __exit__(self,*args):self.depth-=1

def runtime_finally(tmp_path,mutation=None,call_hook=None,join_values=None,join_delay=0.,initial_failure=None,missing_complete=False):
 raw,c,physics,world=fixture();clock=Clock();lock=Lock();calls=[];join_calls=[]
 if mutation:mutation(raw,clock)
 if missing_complete:raw['source_status']=[]
 out=tmp_path;stream=(out/'probe_raw.jsonl').open('x');original=copy.deepcopy(raw)
 def service(name):
  calls.append((name,clock.at,None))
  if call_hook:call_hook(name,clock,physics,world)
 def call_service(name,kind,timeout=5.):
  calls.append((name,clock.at,timeout))
  if timeout<=0:raise RuntimeError('service wall deadline '+name)
  if call_hook:call_hook(name,clock,physics,world)
  if clock.at-calls[-1][1]>timeout:raise RuntimeError('service wall timeout '+name)
  return types.SimpleNamespace(**(physics if 'physics_properties' in name else world))
 def source_integrity(r,contract):
  join_calls.append({'under_lock':lock.depth>0,'clock':clock.at});clock.at+=join_delay
  return dict(zip(KEYS,join_values or [True]*4))
 def assess(r,contract):
  checks=dict(zip(KEYS,join_values or [True]*4));checks['final_paused_source_watermark_drained']=m.final_drain_integrity(r,contract);checks.update(m.command_ingress_integrity(r,contract))
  return {'pass':all(checks.values()),'checks':checks,'assessment_incomplete':not all(checks.values())}
 ctx=dict(vars(m));ctx.update(raw=raw,contract=c,failure=initial_failure,time=clock,service=service,call_service=call_service,GetPhysicsProperties=object(),GetWorldProperties=object(),lock=lock,source_packet_integrity=source_integrity,active=True,stream=stream,subscribers=[],started=10.,output=out,assess=assess)
 tr=next(n for n in RT.body if isinstance(n,ast.Try));block=ast.Module(body=copy.deepcopy(tr.finalbody),type_ignores=[]);exec(compile(ast.fix_missing_locations(block),str(P)+':extracted_exact_finally','exec'),ctx)
 return ctx,calls,join_calls,clock,original

def test_exact_runtime_tail_baseline_and_atomic_close(tmp_path):
 ctx,calls,joins,clock,original=runtime_finally(tmp_path)
 assert ctx['report']['pass'] is True and ctx['active'] is False
 assert joins==[{'under_lock':True,'clock':10.02}]
 worlds=[c for c in calls if 'world_properties' in c[0]];assert worlds[1][1]-worlds[0][1]>=.019999
 assert ctx['raw']['final_drain']['source_watermark_ns']==144000000
 assert ctx['raw']['final_drain']['observed_last_ns']['links']==142000000
 assert ctx['raw']['final_drain']['observed_last_ns']['odom']==140000000
 assert json.loads((tmp_path/'probe_report.json').read_text())['pass'] is True

@pytest.mark.parametrize('stream',['models','links','odom','joints','targets','backend_status','commanded_efforts','dense_models','dense_links','contacts'])
def test_delayed_tail_waits_then_closes_without_trimming(tmp_path,stream):
 def mutate(r,clock):
  pending=r[stream].pop()
  def deliver(c):
   if c.at>=10.1 and pending not in r[stream]:r[stream].append(pending)
  clock.on_sleep=deliver
 ctx,calls,joins,clock,original=runtime_finally(tmp_path,mutation=mutate)
 assert ctx['report']['pass'] is True and len(ctx['raw'][stream])==len(original[stream])+1
 assert ctx['raw']['final_drain']['wall_elapsed_s']>=.1

@pytest.mark.parametrize('kind',['tail_never_delivered','interior_loss','duplicate','future','missing_complete','missing_terminal'])
def test_capture_faults_durable_negative(tmp_path,kind):
 def mutate(r,c):
  if kind=='tail_never_delivered':r['dense_links'].pop()
  if kind=='interior_loss':r['contacts'].pop(20)
  if kind=='duplicate':r['dense_models'][10]=copy.deepcopy(r['dense_models'][9])
  if kind=='future':r['dense_links'].append({'ns':146000000})
  if kind=='missing_terminal':r['commands_sent']=[]
 ctx,calls,joins,clock,original=runtime_finally(tmp_path,mutation=mutate,missing_complete=kind=='missing_complete')
 assert ctx['report']['pass'] is False and ctx['failure'] is not None and ctx['active'] is False
 assert ctx['raw']['dense_links']==original['dense_links'] and ctx['raw']['contacts']==original['contacts']
 assert (tmp_path/'probe_raw.json').exists() and (tmp_path/'probe_report.json').exists()

@pytest.mark.parametrize('kind',['pause_error','physics_error','world_error','unpaused','world_advances','offgrid','slow_queries'])
def test_runtime_service_faults_are_terminal(tmp_path,kind):
 n={'world':0}
 def call(name,clock,physics,world):
  if kind=='pause_error' and 'pause_physics' in name:raise RuntimeError('synthetic pause failure')
  if kind=='physics_error' and 'get_physics_properties' in name:raise RuntimeError('synthetic physics failure')
  if kind=='world_error' and 'get_world_properties' in name:raise RuntimeError('synthetic world failure')
  if kind=='unpaused':physics['pause']=False
  if kind=='offgrid':world['sim_time']=.143
  if 'get_world_properties' in name:
   n['world']+=1
   if kind=='world_advances' and n['world']==2:world['sim_time']=.146
  if kind=='slow_queries' and 'properties' in name:clock.at+=2.
 ctx,calls,joins,clock,original=runtime_finally(tmp_path,call_hook=call)
 assert ctx['report']['pass'] is False and ctx['failure'] is not None
 if kind=='pause_error':assert 'final_drain' not in ctx['raw']

@pytest.mark.parametrize('index',range(4))
def test_failed_source_join_can_never_be_pass(tmp_path,index):
 joins=[True]*4;joins[index]=False
 ctx,calls,recorded,clock,original=runtime_finally(tmp_path,join_values=joins)
 assert ctx['raw']['final_drain']['preclose_source_joins'][KEYS[index]] is False
 assert ctx['report']['pass'] is False and ctx['report']['checks'][KEYS[index]] is False

def test_join_computation_time_counts_in_same_budget(tmp_path):
 ctx,calls,joins,clock,original=runtime_finally(tmp_path,join_delay=5.)
 assert ctx['report']['pass'] is False and 'deadline' in ctx['failure']['message']

def test_existing_failure_not_overwritten_by_pause_failure(tmp_path):
 first={'type':'RuntimeError','message':'original synthetic execution failure'}
 def fail(*args):raise RuntimeError('later pause error')
 ctx,*_=runtime_finally(tmp_path,call_hook=fail,initial_failure=first)
 assert ctx['failure']==first and ctx['raw']['pause_error']=='later pause error'

def test_exact_call_service_helper_wall_timeout_and_exception():
 node=next(n for n in RT.body if isinstance(n,ast.FunctionDef) and n.name=='call_service');space={'threading':threading}
 class R:
  def __init__(self,fn):self.fn=fn
  def ServiceProxy(self,*a):return self.fn
 space['rospy']=R(lambda:types.SimpleNamespace(success=True));exec(compile(ast.fix_missing_locations(ast.Module(body=[copy.deepcopy(node)],type_ignores=[])),str(P)+':exact_service_helper','exec'),space)
 assert space['call_service']('/toy',object(),.1).success
 with pytest.raises(RuntimeError,match='deadline'):space['call_service']('/toy',object(),0)
 def fail():raise ValueError('synthetic service exception')
 space['rospy']=R(fail)
 with pytest.raises(ValueError,match='synthetic service exception'):space['call_service']('/toy',object(),.1)
 release=threading.Event();space['rospy']=R(lambda:release.wait(.1))
 try:
  with pytest.raises(RuntimeError,match='timeout'):space['call_service']('/toy',object(),.002)
 finally:release.set()

@pytest.mark.parametrize('kind',['missing_forever','delayed_terminal','future_terminal','offgrid_terminal','duplicate_terminal','ingress_absent','extra_ingress'])
def test_actual_driver_terminal_controls_final_drain(tmp_path,kind):
 def mutate(r,clock):
  if kind in ('missing_forever','delayed_terminal'):
   pending=r['command_ingress'].pop()
   if kind=='delayed_terminal':
    def deliver(c):
     if c.at>=10.1 and pending not in r['command_ingress']:r['command_ingress'].append(pending)
    clock.on_sleep=deliver
  if kind in ('future_terminal','offgrid_terminal'):
   ns=146000000 if kind=='future_terminal' else 141000000
   r['command_ingress'][-1]=ingress_row(ns,1,0.,0.)
  if kind=='duplicate_terminal':r['command_ingress'].append(ingress_row(142000000,2,0.,0.))
  if kind=='ingress_absent':r['command_ingress']=[]
  if kind=='extra_ingress':r['command_ingress'].insert(1,ingress_row(120000000,1,1.,.1));r['command_ingress'][-1]['seq']=2
 ctx,calls,joins,clock,original=runtime_finally(tmp_path,mutation=mutate)
 if kind=='delayed_terminal':
  assert ctx['report']['pass'] is True and ctx['raw']['final_drain']['wall_elapsed_s']>=.1
 else:assert ctx['report']['pass'] is False
 assert ctx['active'] is False and (tmp_path/'probe_raw.json').exists() and (tmp_path/'probe_report.json').exists()


def protected_context():
 import io,traceback
 fns=[copy.deepcopy(n) for n in RT.body if isinstance(n,ast.FunctionDef) and n.name in ('record','protected')]
 ctx={'lock':threading.RLock(),'raw':{'callback_errors':[],'toy':[]},'active':True,'stream':io.StringIO(),'json':json,'traceback':traceback}
 exec(compile(ast.fix_missing_locations(ast.Module(body=fns,type_ignores=[])),str(P)+':exact_record_and_protected','exec'),ctx)
 return ctx


def test_entire_callback_decode_record_and_error_are_inside_capture_lock():
 ctx=protected_context();parsed=[]
 def normal(message):
  assert ctx['lock']._is_owned();parsed.append(message);ctx['record']('toy',{'x':message})
 ctx['protected'](normal)(1)
 assert parsed==[1] and ctx['raw']['toy']==[{'x':1}]
 def broken(message):
  assert ctx['lock']._is_owned();raise ValueError('synthetic malformed packet')
 ctx['protected'](broken)(2)
 assert len(ctx['raw']['callback_errors'])==1
 with ctx['lock']:ctx['active']=False
 snapshot=copy.deepcopy(ctx['raw']);stream=ctx['stream'].getvalue()
 ctx['protected'](normal)(3);ctx['protected'](broken)(4)
 assert parsed==[1] and ctx['raw']==snapshot and ctx['stream'].getvalue()==stream


def test_pending_callback_after_atomic_close_cannot_decode_or_change_raw():
 ctx=protected_context();started=threading.Event();finished=threading.Event();parsed=[]
 def callback():
  started.set();ctx['protected'](lambda msg:parsed.append(msg))(9);finished.set()
 with ctx['lock']:
  thread=threading.Thread(target=callback);thread.start();assert started.wait(.5)
  ctx['active']=False
 assert finished.wait(.5);thread.join(.5)
 assert parsed==[] and ctx['raw']=={'callback_errors':[],'toy':[]} and ctx['stream'].getvalue()==''
