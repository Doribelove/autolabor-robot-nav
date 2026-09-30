#!/usr/bin/python3
import hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1]
def bind(p):
 p=Path(p).resolve();return {'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
def put(name,value):
 with (A/name).open('x')as f:json.dump(value,f,indent=2,sort_keys=True);f.write('\n')
def main():
 closure=json.loads((A/'closure.json').read_text());author=json.loads(Path(closure['author_closure']['path']).read_text())
 positive=json.loads((A/'reviewed_runtime_probe.json').read_text());negative=json.loads((A/'author_pending_negative.json').read_text());faults=json.loads((A/'runtime_boundary_cases.json').read_text())
 checks={'exact_5217_runtime_resources':len(closure['resources'])==5217 and closure['resources']==author['resources'],
  'exact_runtime_environment':closure['execution_environment']==author['execution_environment'],
  'positive_actual_runtime_branch':positive['accepted']is True and positive['exact_eight_rows_when_accepted']is True,
  'pending_author_rejected':negative['accepted']is False,
  'four_real_runtime_mutations_rejected':len(faults)==4 and all(x['pass']for x in faults.values()),
  'no_release':not(S/'contracts/navigation_n06_release.json').exists(),
  'no_stage_ledger':not(S/'queue/navigation_n06_ledger.json').exists()}
 if not all(checks.values()):raise AssertionError(checks)
 report={'schema':'astra_navigation_n06_reviewed_envelope_report_v1','status':'reviewed_disk_envelope_derived_and_runtime_interface_checked',
  'closure':bind(A/'closure.json'),'independent_disk_review':closure['independent_review_report'],
  'checks':checks,'scope':'Runtime branch passed with real disk envelope; other release prerequisites use explicit test stand-ins and remain independently required.',
  'execution_authorized':False,'runtime_ready':False,'release_created':False,'seeds_registered':0,'ROS_or_Gazebo_started':False,
  'limits':closure['reviewed_envelope_limits'],'candidate_roster_producer':bind(A/'build_candidate_roster.py')}
 put('implementation_report.json',report)
 sources=[bind(p)for p in sorted(A.iterdir())if p.is_file()and p.suffix in ('.py','.md')]
 evidence=[bind(p)for p in sorted(A.iterdir())if p.is_file()and p.suffix=='.json']
 dependencies=[closure[k]for k in ('author_closure','independent_review_report','independent_review_manifest','parent_closure','prior_independent_negative_review','export_input_supplement','author_report','author_manifest')]
 put('source_evidence_manifest.json',{'schema':'astra_navigation_n06_reviewed_envelope_manifest_v1','owned_sources':sources,'author_evidence':evidence,'read_only_dependencies':dependencies,'execution_authorized':False})
 roster=sources+evidence+dependencies
 if any(bind(x['path'])!=x for x in roster):raise ValueError('freeze drift')
 put('freeze_receipt.json',{'report':bind(A/'implementation_report.json'),'manifest':bind(A/'source_evidence_manifest.json'),'resource_count':len({x['path']for x in roster}),'postwrite_match':True,'zero_authority':True})
 print(json.dumps({'report':bind(A/'implementation_report.json'),'manifest':bind(A/'source_evidence_manifest.json'),'closure':bind(A/'closure.json')},indent=2))
if __name__=='__main__':main()
