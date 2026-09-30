#!/usr/bin/env python3
"""Focused independent P06 actual-expanded-URDF startup review; no ROS calls."""
import ast,copy,datetime,difflib,hashlib,importlib.util,json,sys,types,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';C=W/'src/simulation/m2_gazebo/scripts';M=W/'scripts/astra_gazebo/run_m12_physics_p06.py';sys.path.insert(0,str(M.parent))
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb') as f:
  for b in iter(lambda:f.read(1048576),b''):h.update(b)
 return h.hexdigest()
def load(p,n):
 s=importlib.util.spec_from_file_location(n,p);m=importlib.util.module_from_spec(s);sys.modules[n]=m;s.loader.exec_module(m);return m
manager=load(M,'P06_independent_manager');source=load(manager.SOURCE,'P06_independent_source');probe=load(manager.PROBE,'P06_independent_probe');old_source=load(C/'astra_actuation_command_source_p05.py','P05_independent_source')
design,parent,rows=manager.load_design();manager.check_seed_freshness(rows)
expected=(C/'astra_actuation_command_source_p02.py').read_text().replace("'astra_m12_physics_p02'","'astra_m12_physics_p06'").replace('libastra_physical_ackermann_p02_plugin.so','libastra_physical_ackermann_p05_plugin.so')
assert manager.SOURCE.read_text()==expected
old_probe=(C/'astra_physical_actuation_probe_p05.py').read_text();assert manager.PROBE.read_text()==old_probe.replace('M12-P05','M12-P06').replace("'astra_physical_actuation_probe_p05'","'astra_physical_actuation_probe_p06'")
old_names=json.loads((S/'audits/m12_physics_p05_ingress_repair_review.json').read_bytes())['check_names'];assert sorted(design['prerequisite']['full_check_roster'])==old_names and len(old_names)==122
preview=S/'audits/m12_p06_root_launch_preflight';preflight=json.loads((preview/'report.json').read_bytes());assert preflight['runner_sha256']==sha(M) and preflight['design_sha256']==sha(manager.DESIGN)
pre_by_id={x['run_id']:x for x in preflight['runs']}
rt=next(n for n in ast.parse(manager.SOURCE.read_bytes()).body if isinstance(n,ast.FunctionDef) and n.name=='runtime');body=[]
for n in rt.body:
 if isinstance(n,ast.FunctionDef):break
 if not isinstance(n,(ast.Import,ast.ImportFrom)):body.append(copy.deepcopy(n))
prefix=compile(ast.fix_missing_locations(ast.Module(body=body,type_ignores=[])),str(manager.SOURCE)+':exact_runtime_prefix_until_all4publishers_advertised','exec')
class FakeROS:
 def __init__(self,params,pause=True,error=False):self.params=params;self.pause=pause;self.error=error;self.calls=[]
 def init_node(self,n):self.calls.append(('init_node',n))
 def get_param(self,k,default=None):return self.params.get(k,default)
 def wait_for_service(self,name,timeout):self.calls.append(('wait_for_service',name,timeout))
 def ServiceProxy(self,name,kind):
  self.calls.append(('ServiceProxy',name))
  def invoke():
   if self.error:raise RuntimeError('synthetic physics-properties service failure')
   return types.SimpleNamespace(pause=self.pause)
  return invoke
 def Publisher(self,topic,kind,**kw):self.calls.append(('Publisher',topic,kw));return object()
def runtime_prefix(params,pause=True,error=False):
 ros=FakeROS(params,pause,error);scope=dict(vars(source));scope.update(rospy=ros,os=types.SimpleNamespace(environ={'ROS_MASTER_URI':'http://127.0.0.1:14820'}),GetPhysicsProperties=object(),Twist=object(),DiagnosticArray=object(),Int64=object());failure=None
 try:exec(prefix,scope)
 except Exception as exc:failure={'type':type(exc).__name__,'message':str(exc)}
 return ros,failure
cases=[];bindings=[]
def invoke(name,fn,expected_failure=False):
 try:
  fn();case={'case':name,'returned':True,'rejected':False};assert not expected_failure, name+' unexpectedly accepted'
 except (ValueError,RuntimeError,AttributeError,TypeError) as exc:
  if not expected_failure:raise
  case={'case':name,'returned':False,'rejected':True,'error_type':type(exc).__name__,'message':str(exc)}
 cases.append(case)
for row in rows:
 rid=row['run_id'];folder=preview/rid;reference=pre_by_id[rid];pp=folder/'parameters.yaml';lp=folder/'fixture.launch';assert sha(pp)==reference['parameters_sha256'] and sha(lp)==reference['launch_sha256'];params=yaml.safe_load(pp.read_bytes());contract_path=S/row['contract_path'];contract=source.load_contract(contract_path,row['contract_sha256'],row['seed'],rid)
 assert lp.read_text()==manager.render_launch(row,S/'runs'/rid,contract)
 assert params['/actuation_command_source/seed']==params['/fixture_probe/seed']==row['seed'] and params['/actuation_command_source/run_id']==params['/fixture_probe/run_id']==rid
 assert params['/actuation_command_source/contract_sha256']==row['contract_sha256']
 assert params['/actuation_command_source/contract_path']==params['/fixture_probe/contract_path']==str(S/'runs'/rid/'actuation_contract.yaml')
 assert params['/fixture_probe/recording_topics']==manager.TOPICS and set(contract['required_topics'])<=set(manager.TOPICS) and len(manager.TOPICS)==27
 assert contract['gates']==parent['gates'] and contract['phase_schedule']==parent['phase_schedule'] and contract['world_fixture']==parent['world_fixture']
 xml=params['/robot_description'];source_fn=lambda x:source.validate_runtime_environment('http://127.0.0.1:14820',params['/use_sim_time'],params['/m2_gazebo/simulation_only'],x,contract,row['seed']);probe_fn=lambda x:probe.validate_effective_urdf(x,contract,row['seed'])
 invoke(rid+'/actual_expansion/source',lambda:source_fn(xml));invoke(rid+'/actual_expansion/probe',lambda:probe_fn(xml));invoke(rid+'/world',lambda:probe.validate_world(contract['world_fixture']['sdf'],contract))
 def mutation(kind):
  tree=ET.fromstring(xml);plugin=tree.find("gazebo/plugin[@filename='libastra_physical_ackermann_p05_plugin.so']");assert plugin is not None
  if kind=='wrong_plugin':plugin.set('filename','libastra_physical_ackermann_p02_plugin.so')
  elif kind=='wrong_seed':plugin.find('dynamicsSeed').text=str(row['seed']+1)
  elif kind=='wrong_kd':plugin.find('steeringKd').text=str(row['steering_kd']+1)
  elif kind=='wrong_command_timeout':plugin.find('commandTimeout').text='.7'
  elif kind=='wrong_boolean':
   key=next(iter(contract['effective_boolean_parameters']));plugin.find(key).text=str(not contract['effective_boolean_parameters'][key]).lower()
  return ET.tostring(tree,encoding='unicode')
 for kind in ('wrong_plugin','wrong_seed','wrong_kd','wrong_command_timeout','wrong_boolean'):
  changed=mutation(kind);invoke(rid+'/'+kind+'/source',lambda x=changed:source_fn(x),True);invoke(rid+'/'+kind+'/probe',lambda x=changed:probe_fn(x),True)
 invoke(rid+'/frozen_P05_plugin_guard_reproduced',lambda:old_source.validate_runtime_environment('http://127.0.0.1:14820',True,True,xml,contract,row['seed']),True)
 # Execute actual runtime startup statements, with only ROS providers stubbed.
 fake_params={'~seed':row['seed'],'~run_id':rid,'~contract_sha256':row['contract_sha256'],'~contract_path':str(contract_path),'/use_sim_time':params['/use_sim_time'],'/m2_gazebo/simulation_only':params['/m2_gazebo/simulation_only'],'/robot_description':xml}
 ros,failure=runtime_prefix(fake_params);pubs=[x[1] for x in ros.calls if x[0]=='Publisher'];assert failure is None and pubs==['/cmd_vel_navigation','/astra_fixture/command_event','/astra_fixture/command_source_status','/astra_fixture/schedule_start_ns'];cases.append({'case':rid+'/exact_runtime_paused_prefix','returned':True,'rejected':False,'advertised':pubs,'no_publish_or_ros_call':True})
 for kind in ('unpaused','service_failure','runtime_wrong_plugin'):
  args=copy.deepcopy(fake_params)
  if kind=='runtime_wrong_plugin':args['/robot_description']=mutation('wrong_plugin')
  ros,failure=runtime_prefix(args,pause=kind!='unpaused',error=kind=='service_failure');assert failure and not any(x[0]=='Publisher' for x in ros.calls);cases.append({'case':rid+'/'+kind+'/exact_runtime_prefix','returned':False,'rejected':True,'before_any_publishers':True,'failure':failure})
 for kind in ('wrong_hash','wrong_identity','wrong_seed'):
  invoke(rid+'/contract/'+kind,lambda k=kind:source.load_contract(contract_path,'0'*64 if k=='wrong_hash' else row['contract_sha256'],row['seed']+1 if k=='wrong_seed' else row['seed'],rid+'_wrong' if k=='wrong_identity' else rid),True)
 bindings.extend([pp,lp,contract_path]);(D/(rid+'_actual_robot_description.xml')).write_text(xml)
# Manager control flow is unchanged apart from new stage identities, seed roster and exact source filename allowlist.
functions=lambda p:{n.name:n for n in ast.parse(Path(p).read_bytes()).body if isinstance(n,ast.FunctionDef)}
a=functions(M);b=functions(M.with_name('run_m12_physics_p05.py'))
class Identity(ast.NodeTransformer):
 def visit_Constant(self,n):
  if isinstance(n.value,str):n.value=n.value.replace('p06','p05').replace('P06','P05')
  if type(n.value)is int and n.value in (260926020,260926021,260926022):n.value-=3
  return n
same={k:ast.dump(Identity().visit(copy.deepcopy(a[k])))==ast.dump(b[k]) for k in ('render_launch','classify_report','active_inherited_digest','check_seed_freshness','verify_release','append_registry','write_receipt','bind_from_release','execute_row','matrix_summary','execute','main')};assert all(same.values())
terminal=manager.p05_terminal(design);assert terminal['status']=='terminal_startup_failure_preserved'
parent_release=S/design['runtime_resource_parent']['path'];r=json.loads(manager.read_bound(parent_release,design['runtime_resource_parent']['sha256']));resources=[dict(x,current_matches=sha(x['path'])==x['sha256'],archive_matches=sha(S/'code_blobs'/x['sha256'])==x['sha256']) for x in r['resources']];assert len(resources)==527 and all(x['current_matches'] and x['archive_matches'] for x in resources);assert design['updated_build_dependencies']=={}
assert not manager.RUNTIME_LEDGER.exists() and not (S/'contracts/m12_physics_p06_release.json').exists() and all(not (S/'runs'/x['run_id']).exists() for x in rows)
paths=[manager.SOURCE,manager.PROBE,M,manager.DESIGN,C/'astra_actuation_command_source_p02.py',C/'astra_actuation_command_source_p05.py',C/'astra_physical_actuation_probe_p05.py',M.with_name('run_m12_physics_p05.py'),preview/'report.json',preview/'check.py',S/'audits/m12_physics_p05_ingress_repair_review.json',S/'audits/m12_p05_result_independent/terminal_startup_failure_audit.json',parent_release,Path(__file__).resolve()]+bindings
report={'status':'offline_command_source_startup_integration_review_pass','independent_probe_review':True,'independent_source_manager_review':True,'reviewed_probe_sha256':sha(manager.PROBE),'reviewed_source_sha256':sha(manager.SOURCE),'reviewed_manager_sha256':sha(M),'numeric_thresholds_unchanged':True,'physical_numeric_thresholds_unchanged':True,'total_gate_count':122,'check_names':old_names,'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'source_change_exact_stage_and_expected_plugin_filename_only':True,'probe_change_doc_and_node_name_only':True,'all_122_assessment_functions_and_formulas_unchanged':True,'actual_expanded_URDF_all_six_source_and_probe_validators_pass':True,'exact_source_runtime_bootstrap_prefix_executed_with_fake_ROS_providers':True,'actual_P05_failure_reproduced_on_same_expanded_URDF':True,'focused_test_cases':len(cases),'cases':cases,'manager_controlflow_AST_identical_after_identity_substitution':same,'inherited_all527_active_and_archived_match':True,'P05_terminal_negative_preserved':terminal,'no_new_build_or_ROS_or_allocation':True,'P06_release_absent_at_review':True,'limitations':['This pass verifies exact startup compatibility offline. It does not imply successful live capture or physical qualification.','Expanded URDF bytes come from root actual roslaunch --dump-params output, independently rebound to exact current manager launch bytes and per-row SHA. This reviewer invokes no ROS command.','The fake runtime prefix uses a synthetic paused service response and no actual ROS Publisher; all four advertised interfaces and failure-before-advertise behavior are validated, not simulator behavior.','Original P05 offline pass and later startup failure remain separate immutable evidence.','Unchanged physical P05 driver was authored by this reviewer; independent driver review remains root report, not self-certified here.'],'resources':[{'path':str(p),'sha256':sha(p)} for p in paths],'command':'/usr/bin/python3 '+str(Path(__file__).resolve()),'root_primary_review':str(S/'audits/m12_physics_p05_driver_root_review.json')}
(D/'integration_cases.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');(D/'inherited527_resources.json').write_text(json.dumps(resources,indent=2)+'\n')
(S/'audits/m12_physics_p06_startup_integration_review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
(S/'audits/m12_physics_p06_startup_integration_review.md').write_text('# P06 启动集成独立审查\n\n离线审查通过。六份实际launch展开URDF分别调用命令源load_contract/runtime-environment和probe URDF/world验证，全部通过；共'+str(len(cases))+'个聚焦案例。错误插件名、seed、Kd、timeout、boolean以及合同SHA/identity被拒绝。精确runtime启动AST在假paused服务下通过并advertise全部四publisher；未暂停、服务异常和旧插件URDF均在advertise前失败。未调用ROS/Gazebo。\n\nP06命令源相对P02仅改变stage和预期P05插件文件名，所有命令行为不变；probe相对冻结P05仅文档/节点名变化，122门及所有数值计算函数完整不变。相同URDF下旧P05 source guard失败被精确保留。管理器执行/绑定/预算/失败控制流AST除stage和新seed身份完全相等；527继承活动/归档资源无改动；P05一用五退休负结果仍绑定。\n\n本报告验证启动兼容性，不宣称live测量或物理通过。此前P05离线审查遗漏的两套startup validator兼容性检查已在新stage补充，旧报告不改写。\n')
print(json.dumps({'status':report['status'],'focused_cases':len(cases),'probe_sha256':report['reviewed_probe_sha256'],'source_sha256':report['reviewed_source_sha256'],'manager_sha256':report['reviewed_manager_sha256'],'review_sha256':sha(S/'audits/m12_physics_p06_startup_integration_review.json')},indent=2))
