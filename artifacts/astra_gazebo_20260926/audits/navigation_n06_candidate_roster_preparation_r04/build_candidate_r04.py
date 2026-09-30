#!/usr/bin/python3
"""Add five source-reviewed transitive local headers after include audit."""
import argparse,copy,json,re,sys
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo';G=W/'src/simulation/m2_gazebo';sys.path.insert(0,str(P))
import navigation_n06_integrity_r05 as q
SUFFIXES={'.py','.launch','.xacro','.cpp','.h','.yaml','.xml'}
NEW={'path':str(G/'include/m2_gazebo/ackermann_kinematics.h'),'sha256':'fd47813987561dd6ad0acf03f8c0f734070aaeceb663d5950b6566af904baae0'}
EXTRA_PATHS=[NEW['path']]+[str(W/'devel/include/autolabor_canbus_driver'/name)for name in
 ('ChassisParameterServer.h','ChassisParameterServerRequest.h','ChassisParameterServerResponse.h','ChassisParameter.h')]
def put(root,name,value):
 with(root/name).open('x')as f:json.dump(value,f,indent=2,sort_keys=True);f.write('\n')
def local_graph():
 roots=[G/'src'/x for x in ('astra_physical_ackermann_p05_plugin.cpp','astra_dense_evidence_plugin.cpp','astra_physics_metadata_p03_plugin.cpp')]
 todo=list(roots);seen={};edges=[];external=[]
 while todo:
  path=todo.pop()
  if str(path)in seen:continue
  seen[str(path)]=q.binding(path)
  for number,line in enumerate(path.read_text().splitlines(),1):
   match=re.match(r'\s*#\s*include\s*([<\"])([^>\"]+)[>\"]',line)
   if not match:continue
   name=match.group(2);candidates=[path.parent/name,G/'include'/name,W/'devel/include'/name];local=[p.resolve()for p in candidates if p.is_file()]
   local=list(dict.fromkeys(local))
   if len(local)>1:raise ValueError('ambiguous local include '+name)
   if local:
    if W not in local[0].parents:raise ValueError('local include escaped workspace')
    edges.append({'from':str(path),'line':number,'literal':name,'to':str(local[0])});todo.append(local[0])
   else:
    if name.startswith(('m2_gazebo/','autolabor_canbus_driver/')):raise ValueError('missing local workspace include '+name)
    external.append({'from':str(path),'line':number,'literal':name,'scope':'external/system include; not expanded by this local-source audit'})
 return {'roots':[q.binding(p)for p in roots],'resources':[seen[p]for p in sorted(seen)],'edges':edges,'external_includes':external,
  'scope':'Literal direct #include traversal using current-file, m2_gazebo/include and workspace devel/include roots for three active custom plugin translation units. Not a C preprocessor/system-header closure.'}
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--include-review',type=Path,required=True);parser.add_argument('--include-review-sha',required=True);args=parser.parse_args()
 independent={'path':str(args.include_review.resolve()),'sha256':args.include_review_sha};independent_value=q.json_bound(independent)
 approved={}
 def walk(value):
  if isinstance(value,dict):
   if {'path','sha256'}<=set(value):
    item={k:value[k]for k in ('path','sha256')}
    if item['path']in approved and approved[item['path']]!=item:raise ValueError('independent include audit has conflicting binding')
    approved[item['path']]=item
   for child in value.values():walk(child)
  elif isinstance(value,list):
   for child in value:walk(child)
 walk(independent_value)
 added=sorted([approved[path]for path in EXTRA_PATHS],key=lambda x:x['path'])
 if added!=sorted([{k:x[k]for k in ('path','sha256')}for x in independent_value['missing']],key=lambda x:x['path']):raise ValueError('not exact frozen missing list')
 if approved[NEW['path']]!=NEW:raise ValueError('ackermann frozen SHA differs')
 parent={'path':str(S/'audits/navigation_n06_candidate_roster_r05_r03/resources.json'),'sha256':'97db74b3d9fcf283b249b66fd1c432a6e5c80ee8b5ec9559669a00dd87ace15a'}
 old=q.json_bound(parent);previous={x['path']:x for x in old['resources']}
 for item in added:
  q.read_bound(item)
  if item['path']in previous:raise ValueError('expected r03 omission not reproduced')
 graph=local_graph();missing=[x for x in graph['resources']if x['path']not in previous]
 if len(graph['resources'])!=10 or len(graph['edges'])!=7:raise ValueError('local include graph differs from independently checked boundary')
 if missing!=added:raise ValueError('local graph differs from reviewed five additions: '+repr(missing))
 for item in graph['resources']:
  if item['path']in previous and previous[item['path']]!=item:raise ValueError('local source SHA drift')
 result=copy.deepcopy(old);result['resources']=sorted(old['resources']+added,key=lambda x:x['path'])
 result['resource_count']=len(result['resources']);result['source_config_roster']=[x for x in result['resources']if Path(x['path']).suffix in SUFFIXES];result['source_config_count']=len(result['source_config_roster'])
 for item in added:result['origins'][item['path']]=['actual_local_cpp_include_closure.workspace_and_generated_headers']
 same=copy.deepcopy(result)
 for key in ('resources','resource_count','source_config_roster','source_config_count','origins'):same[key]=old[key]
 if same!=old or result['resource_count']!=5669 or result['source_config_count']!=2468:raise ValueError('nonminimal roster delta')
 for item in result['resources']:q.read_bound(item)
 out=S/'audits/navigation_n06_candidate_roster_r05_r04';out.mkdir();put(out,'resources.json',result)
 if any(q.binding(x['path'])!=x for x in result['resources']):raise ValueError('postwrite drift')
 put(out,'freeze_receipt.json',{'roster':q.binding(out/'resources.json'),'resource_count':5669,'source_config_count':2468,'postwrite_match':True,'zero_authority':True})
 put(A,'local_include_graph.json',graph)
 put(A,'exact_delta.json',{'parent_roster':parent,'independent_include_audit':independent,'added_resources':added,'removed_resources':[],
  'same_prior_5664_bindings':True,'all_other_metadata_unchanged':True,'old_baseline_unchanged':result['registry_baseline']==old['registry_baseline'],
  'system_headers_outside_local_include_scope':True,'execution_authorized':False})
 report={'schema':'astra_n06_candidate_roster_preparation_report_v4','status':'local_include_source_omission_closed_pending_final_source_review',
  'roster':q.binding(out/'resources.json'),'parent_roster':parent,'independent_include_audit':independent,
  'actual_local_include_graph':q.binding(A/'local_include_graph.json'),'delta':q.binding(A/'exact_delta.json'),
  'resource_count':5669,'source_config_count':2468,'extra_CMake_build_context_count':1,'all_resource_hashes_rechecked':True,
  'local_transitive_resources_all_present':True,'local_translation_units':3,'execution_authorized':False,'canonical_release_created':False,'seeds_registered':0,'ROS_or_Gazebo_started':False,
  'remaining':'Final source/lifecycle independent review and real full release preflight remain required. Literal local includes do not qualify all system/preprocessor dependencies.'}
 put(A,'implementation_report.json',report)
 owned=[q.binding(p)for p in sorted(A.iterdir())if p.is_file()];deps=[parent,independent,q.binding(out/'resources.json'),q.binding(out/'freeze_receipt.json')]+added
 put(A,'source_evidence_manifest.json',{'schema':'astra_n06_candidate_roster_preparation_manifest_v4','owned_sources_and_evidence':owned,'read_only_dependencies':deps,'execution_authorized':False})
 if any(q.binding(x['path'])!=x for x in owned+deps):raise ValueError('freeze drift')
 put(A,'freeze_receipt.json',{'report':q.binding(A/'implementation_report.json'),'manifest':q.binding(A/'source_evidence_manifest.json'),'postwrite_match':True,'zero_authority':True})
 print(json.dumps({'roster':q.binding(out/'resources.json'),'report':q.binding(A/'implementation_report.json'),'manifest':q.binding(A/'source_evidence_manifest.json')},indent=2))
if __name__=='__main__':main()
