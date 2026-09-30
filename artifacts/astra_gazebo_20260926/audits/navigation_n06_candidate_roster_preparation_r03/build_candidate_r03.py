#!/usr/bin/python3
"""Add exactly eight frozen source/build inputs to the immutable N06 roster."""
import ast,copy,hashlib,json,sys,types
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo';G=W/'src/simulation/m2_gazebo';sys.path.insert(0,str(P))
import navigation_n06_integrity_r05 as q
SUFFIXES={'.py','.launch','.xacro','.cpp','.h','.yaml','.xml'}
RELATIVES=('src/astra_physical_ackermann_p05_plugin.cpp','src/astra_dense_evidence_plugin.cpp','src/astra_physics_metadata_p03_plugin.cpp',
 'include/m2_gazebo/actuator_dynamics.h','include/m2_gazebo/astra_physical_control.h','urdf/m2_physical_base.xacro','urdf/m2_sensors.xacro','CMakeLists.txt')
def checked(path,sha):
 item={'path':str(path),'sha256':sha};return q.json_bound(item),item
def put(root,name,value):
 with(root/name).open('x')as f:json.dump(value,f,indent=2,sort_keys=True);f.write('\n')
def source_archive_cases(old,new):
 source=P/'navigation_evidence_n06.py';tree=ast.parse(source.read_text())
 functions=[x for x in ast.walk(tree)if isinstance(x,ast.FunctionDef)and x.name=='archived_source']
 if len(functions)!=1:raise ValueError('exact live closure helper not unique')
 code=compile(ast.fix_missing_locations(ast.Module(body=[functions[0]],type_ignores=[])),str(source),'exec')
 def call(table,reviewed,relative):
  def archive(root,item):q.read_bound(item);return item
  ns={'WORKSPACE':W,'resources':table,'reviewed':reviewed,'root':Path('/not-created'),
      'inherited':types.SimpleNamespace(archive=archive)}
  exec(code,ns)
  try:return {'accepted':True,'result':ns['archived_source']('src/simulation/m2_gazebo/'+relative)}
  except ValueError as e:return {'accepted':False,'error':str(e)}
 results={}
 for relative in RELATIVES[:3]:
  name=str(G/relative);bad=copy.deepcopy(new);bad[name]['sha256']='0'*64
  results[relative]={'old_roster':call(old,old,relative),'new_roster':call(new,new,relative),'wrong_review_sha':call(new,bad,relative)}
 if not all(not x['old_roster']['accepted']and x['new_roster']['accepted']and not x['wrong_review_sha']['accepted']for x in results.values()):raise AssertionError(results)
 return {'actual_source':q.binding(source),'actual_function':'closure.<locals>.archived_source',
  'archive_copy_explicit_stub':'Reads actual bound source bytes and returns item; no run directory or archival write.',
  'cases':results,'pass':True,'not_a_complete_terminal_closure_execution':True}
def main():
 old,old_binding=checked(S/'audits/navigation_n06_candidate_roster_r05_r02/resources.json','232b9352ea34474c5179ffdee3304a651d0eba96031b2cb823277f0f264855c9')
 scope,scope_binding=checked(S/'audits/navigation_n06_source_lifecycle_scope/required_source_bindings.json','a9075016fc5b7f7d5943e7dab5e768e4e492c3402ba9b04ceef5b2aead44e88c')
 exact={}
 for category in ('custom_native_sources','launch_robot_environment_build_context'):
  for item in scope[category]:
   if item['path']in exact and exact[item['path']]!=item:raise ValueError('contradictory original scope')
   exact[item['path']]=item
 previous={x['path']:x for x in old['resources']};added=[];original_scope_items=[]
 for relative in RELATIVES:
  path=str(G/relative);item=exact[path];binding={k:item[k]for k in ('path','sha256')}
  if path in previous:raise ValueError('expected r02 omission not reproduced: '+path)
  raw=q.read_bound(binding)
  if len(raw)!=item['bytes']:raise ValueError('scope size changed')
  added.append(binding);original_scope_items.append(item)
 for item in old['resources']:q.read_bound(item)
 result=copy.deepcopy(old);new={**previous,**{x['path']:x for x in added}}
 result['resources']=[new[path]for path in sorted(new)];result['resource_count']=len(new)
 result['source_config_roster']=[x for x in result['resources']if Path(x['path']).suffix in SUFFIXES]
 result['source_config_count']=len(result['source_config_roster'])
 for item in added:result['origins'][item['path']]=['frozen_source_lifecycle_scope.eight_omitted_active_inputs']
 if len(new)!=5664 or result['source_config_count']!=2463:raise ValueError('unexpected exact resource counts')
 same=copy.deepcopy(result)
 for key in ('resources','resource_count','source_config_roster','source_config_count','origins'):same[key]=old[key]
 if same!=old:raise ValueError('unrelated roster change')
 out=S/'audits/navigation_n06_candidate_roster_r05_r03';out.mkdir()
 put(out,'resources.json',result)
 if any(q.binding(x['path'])!=x for x in result['resources']):raise ValueError('postwrite byte drift')
 checks={'all_5664_resources_rehashed':True,'only_eight_added_no_removals':set(new)-set(previous)=={str(G/x)for x in RELATIVES}and all(new[p]==x for p,x in previous.items()),
  'all_new_original_scope_sha_and_bytes_match':True,'exact_source_config_count_2463':len(result['source_config_roster'])==2463,
  'original_baseline_unchanged':result['registry_baseline']==old['registry_baseline'],
  'no_live_csv_bound':all(str(S/x)not in new for x in ('seed_registry.csv','run_manifest.csv','EXPERIMENTS.csv')),
  'all_other_metadata_unchanged':same==old,'no_ledger_created':not(S/'queue/navigation_n06_ledger.json').exists(),
  'no_run_allocation':not any((S/'runs'/row['run_id']).exists()for row in q.prepared_api.schedule())}
 if not all(checks.values()):raise AssertionError(checks)
 put(A,'archived_source_regression.json',source_archive_cases(previous,new))
 put(A,'exact_delta.json',{'parent_roster':old_binding,'original_scope':scope_binding,'added_resources':added,'original_scope_items':original_scope_items,'removed_resources':[],
  'extra_build_context_outside_suffix_filter':[new[str(G/'CMakeLists.txt')]],'runtime_resources_changed':False,'qualification_projection_changed':False,'execution_authorized':False})
 put(out,'freeze_receipt.json',{'roster':q.binding(out/'resources.json'),'resource_count':5664,'source_config_count':2463,'postwrite_match':True,'zero_authority':True})
 put(A,'implementation_report.json',{'schema':'astra_n06_candidate_roster_preparation_report_v3','status':'eight_omitted_active_source_inputs_added_pending_independent_review',
  'roster':q.binding(out/'resources.json'),'parent_roster':old_binding,'scope':scope_binding,'checks':checks,
  'actual_archived_source_regression':q.binding(A/'archived_source_regression.json'),'delta':q.binding(A/'exact_delta.json'),
  'resource_count':5664,'source_config_count':2463,'extra_CMake_build_context_count':1,
  'execution_authorized':False,'canonical_release_created':False,'seeds_registered':0,'ROS_or_Gazebo_started':False,
  'remaining':'Final source/lifecycle independent review and real full release preflight remain required.'})
 owned=[q.binding(p)for p in sorted(A.iterdir())if p.is_file()];deps=[old_binding,scope_binding,q.binding(out/'resources.json'),q.binding(out/'freeze_receipt.json')]+added
 put(A,'source_evidence_manifest.json',{'schema':'astra_n06_candidate_roster_preparation_manifest_v3','owned_sources_and_evidence':owned,'read_only_dependencies':deps,'execution_authorized':False})
 if any(q.binding(x['path'])!=x for x in owned+deps):raise ValueError('freeze drift')
 put(A,'freeze_receipt.json',{'report':q.binding(A/'implementation_report.json'),'manifest':q.binding(A/'source_evidence_manifest.json'),'postwrite_match':True,'zero_authority':True})
 print(json.dumps({'roster':q.binding(out/'resources.json'),'report':q.binding(A/'implementation_report.json'),'manifest':q.binding(A/'source_evidence_manifest.json')},indent=2))
if __name__=='__main__':main()
