#!/usr/bin/env python3
"""Same-lexical-scope production callback/finally integration, no ROS calls."""
import ast,copy,hashlib,importlib.util,json,sys,threading,types
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';P=W/'src/simulation/m2_gazebo/scripts'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def load(p,name):
 spec=importlib.util.spec_from_file_location(name,p);m=importlib.util.module_from_spec(spec);sys.modules[name]=m;spec.loader.exec_module(m);return m
helper=load(S/'audits/m12_p05_ingress_independent/test_independent_runtime.py','frozen_test_fixture_helpers')
oldpath=P/'astra_physical_actuation_probe_p06.py';newpath=P/'astra_physical_actuation_probe_p07.py';old=load(oldpath,'old_p06_callback_case');new=load(newpath,'new_p07_callback_case')
KEYS=('models_matches_dense_same_stamp','links_matches_dense_same_stamp','actual_joint_positions_agree_with_dense_source_pose','actual_odom_twist_agrees_with_dense_base_link')

def message(ns,kind):
 z=types.SimpleNamespace(x=0.,y=0.,z=0.);q=types.SimpleNamespace(x=0.,y=0.,z=0.,w=1.);pose=types.SimpleNamespace(position=z,orientation=q);twist=types.SimpleNamespace(linear=z,angular=z)
 return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(to_nsec=lambda:ns,to_sec=lambda:ns/1e9),frame_id='world',seq=7),name=['m2' if kind=='models' else 'm2::base_link'],pose=[pose],twist=[twist])

def run_case(mod,path,kind,delivery='after_response'):
 rt=next(n for n in ast.parse(path.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=='runtime');tree=ast.parse(path.read_bytes())
 raw,c,physics,world_payload=helper.fixture();raw['callback_errors']=[];clock=helper.Clock();lock=threading.RLock();pending=[]
 for key in kind:
  end=raw[key].pop()['ns'];pending.append((key,message(end,key)))
 output=D/'fixtures'/((path.stem.rsplit('_',1)[-1])+'_'+('_'.join(kind) or 'no_pending')+'_'+delivery);output.mkdir(parents=True,exist_ok=False);stream=(output/'probe_raw.jsonl').open('x')
 calls=[];delivered=[]
 def deliver(clock,callbacks):
  if delivered:return
  delivered.append(clock.at)
  for key,msg in pending:callbacks[key](msg)
 def service(name):calls.append(name)
 def call_service(name,kind,timeout=5.):
  calls.append(name)
  if timeout<=0:raise RuntimeError('service wall deadline '+name)
  return types.SimpleNamespace(**physics) if 'physics_properties' in name else type('GetWorldPropertiesResponse',(types.SimpleNamespace,),{})(**world_payload)
 def integrity(r,contract):
  assert lock._is_owned();return dict.fromkeys(KEYS,True)
 def assessor(r,contract):
  checks={'no_callback_errors':not r['callback_errors'],'final_paused_source_watermark_drained':mod.final_drain_integrity(r,contract)};checks.update(mod.command_ingress_integrity(r,contract));return {'pass':all(checks.values()),'checks':checks,'synthetic_controlflow_only':True}
 # All original nested definitions, all actual Subscriber callback lambda ASTs,
 # and the complete original finally body share ONE compiled function scope.
 nested=[copy.deepcopy(n) for n in rt.body if isinstance(n,ast.FunctionDef) and n.name in ('record','protected','header','pp','world')]
 callback_expr={}
 for n in ast.walk(rt):
  if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='Subscriber' and n.args and isinstance(n.args[0],ast.Constant) and n.args[0].value in ('/astra/world_models','/astra/world_links'):
   callback_expr['models' if n.args[0].value.endswith('models') else 'links']=copy.deepcopy(n.args[2])
 assert set(callback_expr)=={'models','links'}
 values=dict(raw=raw,contract=c,failure=None,time=clock,service=service,call_service=call_service,lock=lock,active=True,stream=stream,subscribers=[],started=10.,output=output,assess=assessor)
 setup=[ast.parse(k+"=G["+repr(k)+"]").body[0] for k in values]
 install=ast.Assign(targets=[ast.Name(id='callbacks',ctx=ast.Store())],value=ast.Dict(keys=[ast.Constant(value=k) for k in callback_expr],values=list(callback_expr.values())))
 if delivery=='before_response':inject=ast.parse('deliver(time, callbacks)').body
 else:inject=ast.parse('time.on_sleep=lambda c: deliver(c,callbacks)').body
 final=copy.deepcopy(next(n for n in rt.body if isinstance(n,ast.Try)).finalbody)
 ending=ast.parse("return {'raw':raw,'report':report,'failure':failure,'active':active,'callbacks':callbacks}").body
 fn=ast.FunctionDef(name='integrated_runtime_callback_and_complete_finally',args=ast.arguments(posonlyargs=[],args=[],vararg=None,kwonlyargs=[],kw_defaults=[],kwarg=None,defaults=[]),body=setup+nested+[install]+inject+final+ending,decorator_list=[])
 module=ast.fix_missing_locations(ast.Module(body=[fn],type_ignores=[]));namespace=dict(vars(mod));namespace.update(G=values,deliver=deliver,source_packet_integrity=integrity,GetPhysicsProperties=object(),GetWorldProperties=object())
 exec(compile(module,str(path)+':same_lexical_callback_plus_entire_finally','exec'),namespace);result=namespace['integrated_runtime_callback_and_complete_finally']()
 expected_pass=(path==newpath or not pending or delivery=='before_response');assert result['report']['pass'] is expected_pass
 if not expected_pass:
  assert result['failure']['message']=='source evidence terminal drain deadline'
  assert len(result['raw']['callback_errors'])==len(pending) and all(x['error']=="'GetWorldPropertiesResponse' object is not callable" for x in result['raw']['callback_errors'])
 else:assert result['failure'] is None and result['raw']['callback_errors']==[]
 snapshot=json.dumps(result['raw'],sort_keys=True)
 for key in ('models','links'):result['callbacks'][key](message(162000000,key))
 assert json.dumps(result['raw'],sort_keys=True)==snapshot and result['active'] is False
 return {'source_path':str(path),'source_sha256':sha(path),'pending_streams':list(kind),'delivery':delivery,'expected_pass':expected_pass,'actual_pass':result['report']['pass'],'exact_error':result['raw']['callback_errors'],'failure':result['failure'],'closed_raw_immutable':True,'synthetic_only':True,'output_dir':str(output),'integrated_AST_sha256':hashlib.sha256(ast.dump(module).encode()).hexdigest()}

cases=[]
for path,mod in ((oldpath,old),(newpath,new)):
 for keys in ((),('links',),('models',),('models','links')):cases.append(run_case(mod,path,keys))
 cases.append(run_case(mod,path,('models','links'),'before_response'))

def name_audit(path):
 rt=next(n for n in ast.parse(path.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=='runtime');names={n.name for n in rt.body if isinstance(n,ast.FunctionDef)};stores=[]
 def walk(n):
  if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef,ast.ClassDef,ast.Lambda)):return
  if isinstance(n,ast.Name) and isinstance(n.ctx,ast.Store) and n.id in names:stores.append({'name':n.id,'line':n.lineno})
  for child in ast.iter_child_nodes(n):walk(child)
 for n in rt.body:walk(n)
 return {'nested_function_names':sorted(names),'same_scope_reassignments':stores}
name_checks={str(p):name_audit(p) for p in (oldpath,newpath,P/'astra_actuation_command_source_p07.py')};assert name_checks[str(oldpath)]['same_scope_reassignments']==[{'name':'world','line':738}];assert not name_checks[str(newpath)]['same_scope_reassignments'] and not name_checks[str(P/'astra_actuation_command_source_p07.py')]['same_scope_reassignments']
expected=oldpath.read_text().replace('M12-P06','M12-P07').replace("'astra_physical_actuation_probe_p06'","'astra_physical_actuation_probe_p07'").replace("world=call_service('/gazebo/get_world_properties'","world_response=call_service('/gazebo/get_world_properties'").replace('getattr(world,key)','getattr(world_response,key)');assert newpath.read_text()==expected
sourcepath=P/'astra_actuation_command_source_p07.py';assert sourcepath.read_text()==(P/'astra_actuation_command_source_p06.py').read_text().replace("'astra_m12_physics_p06'","'astra_m12_physics_p07'")
result={'status':'independent_same_lexical_callback_finally_regression_pass','cases':cases,'case_count':len(cases),'same_scope_nested_function_reassignment_audit':name_checks,'source_only_stage_change':True,'probe_exact_doc_node_and_response_variable_rename_only':True,'historical_failure_reproduced':True,'new_delayed_models_and_links_both_pass':True,'all_postclose_callbacks_ignored':True,'no_ROS_or_simulation_started':True,'physical_qualification_evidence':False,'script_sha256':sha(__file__)}
(D/'callback_closure_cases.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps({'status':result['status'],'cases':len(cases),'name_audit':name_checks,'sha256':sha(D/'callback_closure_cases.json')},indent=2))
