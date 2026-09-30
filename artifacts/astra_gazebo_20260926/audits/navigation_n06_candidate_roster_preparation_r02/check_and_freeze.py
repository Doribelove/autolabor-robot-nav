#!/usr/bin/python3
"""Exact read-only candidate roster closure, not a release constructor."""
import copy,difflib,json,sys
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];P=S/'workspace/scripts/astra_gazebo';sys.path.insert(0,str(P))
import navigation_n06_integrity_r05 as q
def put(name,d):
 with (A/name).open('x')as f:json.dump(d,f,indent=2,sort_keys=True);f.write('\n')
def main():
 out=S/'audits/navigation_n06_candidate_roster_r05_r02';roster=q.json_bound(q.binding(out/'resources.json'));resources={x['path']:x['sha256']for x in roster['resources']}
 runtime=q.json_bound(roster['reviewed_runtime']);projection=q.json_bound(q.binding(A/'qualification_projection.json'))
 old=q.json_bound(projection['original']);new=q.json_bound(projection['projected']);check=copy.deepcopy(new);check['evidence_bindings'][0]=old['evidence_bindings'][0]
 checks={'all_5656_resources_rehashed':len(resources)==5656 and all(q.binding(x['path'])==x for x in roster['resources']),
  'all_5217_runtime_resources_present':len(runtime['resources'])==5217 and all(resources.get(x['path'])==x['sha256']for x in runtime['resources']),
  'all_required_manager_paths_present':all(resources.get(str(p.resolve()))==q.binding(p)['sha256']for p in q.required_manager_paths()),
  'exact_source_config_subset':roster['source_config_roster']==[x for x in roster['resources']if Path(x['path']).suffix in {'.py','.launch','.xacro','.cpp','.h','.yaml','.xml'}]and len(roster['source_config_roster'])==2456,
  'no_live_csv_bound':all(str(S/n)not in resources for n in ('seed_registry.csv','run_manifest.csv','EXPERIMENTS.csv')),
  'baseline_matches_live_at_audit':q.read_bound(roster['registry_baseline'])==q.regular_bytes(S/'seed_registry.csv'),
  'qualification_only_one_path_changed':check==old and new['evidence_bindings'][0]['sha256']==old['evidence_bindings'][0]['sha256'],
  'canonical_qualification_evidence_pass':all(q.binding(x['path'])==x for x in new['evidence_bindings']),
  'source_review_self_excluded':roster['source_review_self_binding_excluded']is True,
  'roster_self_excluded':str(out/'resources.json')not in resources,
  'failed_old_output_absent':not(S/'audits/navigation_n06_candidate_roster_r05_r01').exists(),
  'no_n06_ledger':not(S/'queue/navigation_n06_ledger.json').exists(),
  'no_n06_run_allocation':not any((S/'runs'/x['run_id']).exists()for x in q.prepared_api.schedule())}
 if not all(checks.values()):raise AssertionError(checks)
 old_builder=S/'audits/navigation_n06_runtime_reviewed_r01/build_candidate_roster.py'
 delta=''.join(difflib.unified_diff(old_builder.read_text().splitlines(True),(A/'build_candidate_roster.py').read_text().splitlines(True),fromfile=str(old_builder),tofile=str(A/'build_candidate_roster.py')))
 with(A/'builder_delta.patch').open('x')as f:f.write(delta)
 report={'schema':'astra_n06_candidate_roster_preparation_report_v2','status':'readonly_candidate_roster_closure_pass_pending_final_source_review',
  'checks':checks,'roster':q.binding(out/'resources.json'),'registry_baseline':roster['registry_baseline'],
  'qualification_projection':q.binding(A/'qualification_projection.json'),'qualification':projection['projected'],
  'reviewed_runtime':roster['reviewed_runtime'],'resource_count':5656,'source_config_count':2456,
  'original_builder_failure':q.binding(A/'original_builder_negative.json'),'builder_delta':q.binding(A/'builder_delta.patch'),
  'execution_authorized':False,'canonical_release_created':False,'seeds_registered':0,'ROS_or_Gazebo_started':False,
  'remaining':['Exact final source/lifecycle independent review.','Root-owned release construction and actual full verifier without synthetic stand-ins.','Dedicated GAZEBORC immutable absence is not claimed or added; current disk observation remains a limit.','Actual runtime mapped-ELF/native/physics evidence remains future.']}
 put('implementation_report.json',report)
 owned=[q.binding(p)for p in sorted(A.iterdir())if p.is_file()]
 deps=[q.binding(out/'resources.json'),q.binding(out/'freeze_receipt.json'),roster['registry_baseline'],roster['reviewed_runtime'],projection['original'],q.binding(old_builder)]
 put('source_evidence_manifest.json',{'schema':'astra_n06_candidate_roster_preparation_manifest_v2','owned_sources_and_evidence':owned,'read_only_dependencies':deps,'execution_authorized':False})
 if any(q.binding(x['path'])!=x for x in owned+deps):raise ValueError('freeze drift')
 put('freeze_receipt.json',{'report':q.binding(A/'implementation_report.json'),'manifest':q.binding(A/'source_evidence_manifest.json'),'postwrite_match':True,'zero_authority':True})
 print(json.dumps({'report':q.binding(A/'implementation_report.json'),'manifest':q.binding(A/'source_evidence_manifest.json'),'roster':q.binding(out/'resources.json')},indent=2))
if __name__=='__main__':main()
