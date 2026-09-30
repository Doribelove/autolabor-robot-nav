import copy,hashlib,json,re,sys
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];R=S/'audits/navigation_n06_runtime_closure_r01';C=S/'audits/navigation_n06_runtime_closure_r02'
sys.path.insert(0,str(R));from build_disk_closure import binding,write
if (A/'implementation_report.json').exists() or C.exists():raise RuntimeError('preserve frozen version')
r=json.loads((A/'attempt01/supplement.json').read_text());parent=json.loads((R/'closure.json').read_text());resources={b['path']:b for b in parent['resources']}
old_manifest=json.loads((R/'source_evidence_manifest.json').read_text());old={b['path']:b for key in ('owned_sources','runtime_resources','author_evidence','manager_and_risk_bindings') for b in old_manifest[key]}
if not all(binding(p)==b for p,b in old.items()):raise RuntimeError('old author evidence changed')
negative=S/'audits/navigation_n06_runtime_disk_closure_independent/r05_final/review.json'
if binding(negative)['sha256']!='5fc6cb26f7ffe3ca90d257e7e2372ebb371b1dee750e1e870c3e3d07449b21cf':raise RuntimeError('negative review changed')
if r['export_counts']!={'plugin_path':19,'gazebo_media_path':19,'gazebo_model_path':20} or len(r['configured_plugins_in_export_paths'])!=6 or len(r['runtime_resources_added'])!=3 or r['new_configured_ELF_files']!=0:raise RuntimeError('unexpected supplement scope')
for b in r['runtime_resources_added']:
 if binding(b['path'])!=b or b['path'] in resources:raise RuntimeError('addition differs or duplicate')
 resources[b['path']]=b
log=(A/'author_tests.log').read_text()
if not re.search(r'Ran 5 tests.*\n\nOK',log,re.S):raise RuntimeError('focused tests did not pass')
C.mkdir();new=copy.deepcopy(parent);new.update(resources=[resources[p] for p in sorted(resources)],parent_closure=binding(R/'closure.json'),export_input_supplement=binding(A/'attempt01/supplement.json'),prior_independent_negative_review=binding(negative),independent_review=False,status='authored_disk_dependency_closure_independent_review_pending')
write(C/'closure.json',new)
refs=[R/'implementation_report.json',R/'source_evidence_manifest.json',R/'closure.json',negative,S/'audits/navigation_n06_runtime_disk_closure_independent/r05_final/export_gap.json',S/'audits/navigation_n06_runtime_disk_closure_independent/r05_final/rospack_exports_commands.json',Path(r['GAZEBORC']['source']['path']),Path(r['query_tool']['path'])]
items={b['path']:b for b in [binding(p) for p in refs]+list(r['all_exporter_and_gazebo_ros_manifests'].values())+r['query_tool_elf_dependencies']}
owned=[A/p for p in ('collect_exports.py','test_exports.py','freeze_supplement.py','API.md')]
evidence=[p for p in sorted(A.rglob('*')) if p.is_file() and '__pycache__' not in p.parts]
manifest={'schema':'astra_n06_runtime_exports_supplement_source_evidence_v1','owned_sources':[binding(p) for p in owned],'read_only_dependencies':list(items.values()),'author_evidence':[binding(p) for p in evidence],'new_closure':binding(C/'closure.json'),'execution_authorized':False}
write(A/'source_evidence_manifest.json',manifest)
report={'schema':'astra_n06_runtime_exports_supplement_author_report_v1','status':'author_minimal_export_input_closure_pass_independent_review_pending','prior_independent_negative_review':binding(negative),'supplement':binding(A/'attempt01/supplement.json'),'closure_r02':binding(C/'closure.json'),'source_evidence_manifest':binding(A/'source_evidence_manifest.json'),'runtime_resources':len(resources),'added_runtime_resources':r['runtime_resources_added'],'old_author_resources_unchanged':len(old),'focused_tests_passed':5,'actual_export_counts':r['export_counts'],'configured_candidates_same_canonical':6,'new_configured_ELF':0,'GAZEBORC':r['GAZEBORC'],'source_binary_equivalence_new_claim':False,'unused_asset_trees_qualified':False,'live_mapped_ELF_observed':False,'authority':{'release_created':False,'seeds_registered':0,'ros_gazebo_started':False,'live_csv_written':False}}
write(A/'implementation_report.json',report);write(C/'author_report_binding.json',binding(A/'implementation_report.json'))
allitems={b['path']:b for key in ('owned_sources','read_only_dependencies','author_evidence') for b in manifest[key]};allitems.update(resources)
if not all(binding(p)==b for p,b in allitems.items()):raise RuntimeError('postwrite drift')
write(A/'freeze_receipt.json',{'report':binding(A/'implementation_report.json'),'manifest':binding(A/'source_evidence_manifest.json'),'closure':binding(C/'closure.json'),'postwrite_match':True,'unique_resources':len(allitems)})
print(json.dumps({'report':binding(A/'implementation_report.json'),'manifest':binding(A/'source_evidence_manifest.json'),'closure':binding(C/'closure.json'),'runtime_resources':len(resources)},indent=2))
