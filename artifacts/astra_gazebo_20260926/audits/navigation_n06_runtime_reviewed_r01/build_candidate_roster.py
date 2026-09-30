#!/usr/bin/python3
"""Versioned read-only input roster. Never creates execution authority."""
import argparse,csv,hashlib,io,json,sys
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo';sys.path.insert(0,str(P))
import navigation_n06_integrity_r05 as q
SUFFIXES={'.py','.launch','.xacro','.cpp','.h','.yaml','.xml'}

def main():
 parser=argparse.ArgumentParser();parser.add_argument('--reviewed-runtime',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
 if args.output.exists():raise FileExistsError('preserve candidate roster version')
 runtime=q.strict_json(args.reviewed_runtime.read_bytes())
 if runtime.get('status')!='reviewed_disk_dependency_closure' or runtime.get('independent_review')is not True:raise ValueError('actual reviewed runtime envelope required')
 resources={};origins={}
 def add(item,origin):
  item={k:item[k] for k in ('path','sha256')};q.read_bound(item)
  if Path(item['path']) in {S/'seed_registry.csv',S/'run_manifest.csv',S/'EXPERIMENTS.csv'}:raise ValueError('mutable live administrative table is not a release resource')
  old=resources.get(item['path'])
  if old is not None and old!=item:raise ValueError('contradictory resource bytes')
  resources[item['path']]=item;origins.setdefault(item['path'],set()).add(origin)
 def file(path,origin):add(q.binding(path),origin)
 preparation_path=S/'audits/navigation_n06_preparation/source_evidence_manifest.json';prep=q.strict_json(preparation_path.read_bytes())
 for category in ('owned_sources','root_owned_and_frozen_runtime_nodes','prepared_inputs','author_audits_including_failures','read_only_dependencies'):
  for item in prep[category]:add(item,'preparation.'+category)
 for revision in ('r05',):
  path=S/('audits/navigation_n06_manager_'+revision+'/source_evidence_manifest.json');manager=q.strict_json(path.read_bytes())
  for category in ('owned_sources','read_only_dependencies','author_evidence'):
   for item in manager[category]:add(item,'manager_'+revision+'.'+category)
  for name in ('implementation_report.json','source_evidence_manifest.json','freeze_receipt.json'):file(path.parent/name,'manager_'+revision+'.freeze')
 for item in runtime['resources']:add(item,'runtime.resources')
 for path in q.required_manager_paths():file(path,'required_manager_paths')
 file(args.reviewed_runtime,'reviewed_runtime')
 for key in ('author_closure','independent_review_report','independent_review_manifest','export_supplement_review','parent_closure','prior_independent_negative_review','export_input_supplement','author_report','author_manifest'):
  if isinstance(runtime.get(key),dict) and {'path','sha256'}<=set(runtime[key]):add(runtime[key],'runtime.'+key)
 for name in ('implementation_report.json','source_evidence_manifest.json','freeze_receipt.json'):
  file(args.reviewed_runtime.parent/name,'reviewed_envelope.freeze')
 prepared=q.strict_json((q.prepared_api.PREPARED/'prepared.json').read_bytes())
 for unit in prepared['units']:
  for key in ('contract','launch','gate_expectation','robot_description','world'):add(unit[key],'prepared.'+key)
 conversions=S/'contracts/navigation_n06_conversion_r01/bundle.json';bundle=q.strict_json(conversions.read_bytes())
 for unit in bundle['units']:
  for key in ('input_urdf','conversion_manifest','converted_sdf'):add(unit[key],'conversion.'+key)
  for item in unit['conversion_resources']:add(item,'conversion.resources')
 qualification=S/'contracts/passive_q05_navigation_evidence_qualification.json';qual=q.strict_json(qualification.read_bytes())
 if qual.get('qualified_for_navigation_evidence')is not True or qual.get('resolved_steering_kd')!=6:raise ValueError('real qualification identity differs')
 for item in qual['evidence_bindings']:add(item,'passive_qualification.evidence')
 native_reference=S/'audits/navigation_n05_root_release/source_review.json';native=q.strict_json(native_reference.read_bytes())
 native_fields={k:native[k] for k in ('engine_module_bindings','engine_source_bindings','native_engine_report','native_engine_records','upstream_source_manifest')}
 for key,value in native_fields.items():
  for item in value if isinstance(value,list) else [value]:add(item,'inherited_native_only.'+key)
 # This binds native prerequisites only; it does not reuse N05 stage authority.
 file(native_reference,'historical_native_reference_only')
 refs=[q.prepared_api.PREPARED/'prepared.json',q.prepared_api.PREPARED/'schedule.json',preparation_path,conversions,qualification,
  S/'audits/navigation_n06_preparation_independent/review.json',S/'audits/navigation_n06_conversion_independent/review.json',
  S/'audits/development_geometry_d04_r05_independent/review.json',S/'audits/navigation_n06_manager_independent/r05/review.json',S/'queue/navigation_n05_ledger.json']
 for path in refs:file(path,'top_level_prerequisite')
 rows=q.prepared_api.schedule();before=q.regular_bytes(S/'seed_registry.csv');registry=q.csv_rows(before,q.REGISTRY_FIELDS)
 if any(r['run_id'] in {u['run_id'] for u in rows} or r['seed'] in {str(u['seed']) for u in rows} for r in registry):raise ValueError('proposed identities already registered')
 if any((S/'runs'/u['run_id']).exists() for u in rows) or (S/'queue/navigation_n06_ledger.json').exists():raise ValueError('unexpected prior N06 allocation')
 args.output.mkdir();baseline=args.output/'registry_baseline.csv';q.publish(baseline,before)
 if q.regular_bytes(S/'seed_registry.csv')!=before:raise ValueError('registry changed during snapshot; retained baseline is stale')
 file(baseline,'immutable_registry_baseline_snapshot');file(Path(__file__).resolve(),'roster_producer_not_runtime')
 ordered=[resources[p] for p in sorted(resources)];source=[i for i in ordered if Path(i['path']).suffix in SUFFIXES]
 result={'schema':'astra_navigation_n06_candidate_resource_roster_v1','stage':q.NAME,'status':'readonly_complete_input_roster_pending_final_source_review',
  'resources':ordered,'resource_count':len(ordered),'source_config_roster':source,'source_config_count':len(source),
  'origins':{p:sorted(origins[p]) for p in sorted(origins)},'reviewed_runtime':q.binding(args.reviewed_runtime),
  'registry_baseline':q.binding(baseline),'source_review_self_binding_excluded':True,'roster_self_binding_excluded':True,
  'final_source_review_not_created_or_assumed':True,'inherited_native_fields_for_independent_revalidation':native_fields,
  'execution_authorized':False,'seed_registration_authorized':False,'canonical_release_created':False,'ros_gazebo_started':False}
 q.publish(args.output/'resources.json',result)
 if any(q.binding(item['path'])!=item for item in ordered):raise ValueError('roster postwrite drift')
 q.publish(args.output/'freeze_receipt.json',{'roster':q.binding(args.output/'resources.json'),'resource_count':len(ordered),'source_config_count':len(source),'postwrite_match':True,'zero_authority':True})
 print(json.dumps({'roster':q.binding(args.output/'resources.json'),'resources':len(ordered),'source_configs':len(source)},indent=2))
if __name__=='__main__':main()
