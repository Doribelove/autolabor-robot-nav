"""Independent synthetic P04 terminal-path fixtures. No ROS or simulator calls."""
import ast,copy,hashlib,importlib.util,json,threading,time,types
from pathlib import Path
import pytest
HERE=Path(__file__).resolve().parent
S=HERE.parents[1];P=S/'workspace/src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p04.py'
spec=importlib.util.spec_from_file_location('p04_frozen',P);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
TREE=ast.parse(P.read_bytes());RT=next(n for n in TREE.body if isinstance(n,ast.FunctionDef) and n.name=='runtime')
KEYS=('models_matches_dense_same_stamp','links_matches_dense_same_stamp','actual_joint_positions_agree_with_dense_source_pose','actual_odom_twist_agrees_with_dense_base_link')

def fixture():
 c={'world_fixture':{'max_step_s':.002,'fixed_world_models':['ground_plane']},'model_name':'m2','total_schedule_s':.04}
 raw={'schedule_start_ns':100000000,'source_status':[{'statuses':[{'values':{'state':'complete'}}]}],'commands_sent':[{'phase':'terminal_zero'}],'seed':123,'run_id':'synthetic_P04_no_simulation'}
 for k in ('models','links','odom','joints','targets','backend_status','commanded_efforts','dense_models','dense_links','contacts'):
  period=2000000 if k in ('dense_models','dense_links','contacts') else 20000000;first=2000000 if k in ('models','links','dense_models','dense_links','contacts') else 20000000
  raw[k]=[{'ns':ns} for ns in range(first,144000001,period)]
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
  checks=dict(zip(KEYS,join_values or [True]*4));checks['final_paused_source_watermark_drained']=m.final_drain_integrity(r,contract)
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

def test_only_new_gate_added_old_assessor_and_all_other_helpers_identical():
 old=ast.parse((S/'workspace/src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p02.py').read_bytes());before={n.name:n for n in old.body if isinstance(n,ast.FunctionDef)};after={n.name:n for n in TREE.body if isinstance(n,ast.FunctionDef)}
 for name in set(before)-{'source_packet_integrity','runtime'}:assert ast.dump(before[name])==ast.dump(after[name])
 new=copy.deepcopy(after['source_packet_integrity']);init=next(n for n in new.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='checks' for t in n.targets));assert [v.value for v in init.value.keys]==['final_paused_source_watermark_drained'];init.value=ast.Dict(keys=[],values=[]);assert ast.dump(new)==ast.dump(before['source_packet_integrity'])
 # Close transition is nested inside the exact same with-lock block as source joins.
 final=next(n for n in RT.body if isinstance(n,ast.Try)).finalbody
 atomic=[n for n in ast.walk(ast.Module(body=final,type_ignores=[])) if isinstance(n,ast.With) and any(isinstance(c,ast.Call) and isinstance(c.func,ast.Name) and c.func.id=='source_packet_integrity' for c in ast.walk(n))]
 assert len(atomic)==1 and any(isinstance(n,ast.Assign) and isinstance(n.value,ast.Constant) and n.value.value is False and any(isinstance(t,ast.Name) and t.id=='active' for t in n.targets) for n in ast.walk(atomic[0]))
