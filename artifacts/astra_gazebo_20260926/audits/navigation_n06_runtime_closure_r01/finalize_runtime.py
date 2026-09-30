#!/usr/bin/python3
"""Read-only dependency checks plus one-time authored disk closure publication."""
import ast,csv,hashlib,json,re,sys
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';P=W/'scripts/astra_gazebo';C=A/'collection_r05_attempt04';E=A/'environment_r05_attempt01'
sys.path[:0]=[str(A),str(P)]
import build_disk_closure as collector
import navigation_n06_environment_r05 as policy

def b(p):return collector.binding(p)
def write(p,v):collector.write(p,v)
def read(p):return json.loads(p.read_text())
if (A/'implementation_report.json').exists():raise RuntimeError('already frozen')
collection=read(C/'collection.json');resources=collection['resources'];by={i['path']:i for i in resources};checks={}
def check(name,condition):
 checks[name]=bool(condition)
 if not condition:raise ValueError(name)
check('resource_roster_unique',len(by)==len(resources))
check('all_canonical_disk_hashes_match',all(b(i['path'])==i for i in resources))
check('disk_only_no_ros_or_mapped_claim',collection['ros_gazebo_started'] is False and collection['live_mapped_elf_observed'] is False)
profiles={k:read(C/(k+'.json')) for k in ('venv','system_launcher')};spec=read(E/'execution_environment.json');expected=policy.expected_environment(spec,'0'*64)
check('both_import_processes_exact_fixed_environment',all(v['complete_environment']==expected for v in profiles.values()))
check('explicit_venv_prefix_and_disabled_user_site',profiles['venv']['sys_prefix']==str(W/'.venv') and profiles['venv']['no_user_site'] is True and profiles['system_launcher']['sys_prefix']=='/usr')
prepared=read(S/'contracts/navigation_n06_prepared_r01/prepared.json');ids={u['run_id']:u for u in prepared['units']};nodes=profiles['system_launcher']['resolved_nodes']
check('eight_units_eight_nodes_each',len(nodes)==64 and all(sum(n['run_id']==i for n in nodes)==8 for i in ids))
check('node_roster_and_methods_match_prepared',all(n['run_id'] in ids and n['method']==ids[n['run_id']]['method'] for n in nodes))
check('all_node_entries_interpreters_bound',all(n['entry_canonical'] in by and (n['interpreter'] is None or n['interpreter']['canonical'] in by) for n in nodes))
check('no_unbound_roslaunch_environment_overrides',all(not n['env_args'] and not n['roslaunch_environment_delta'] for n in nodes))
check('all_explicit_manager_dependencies_bound',all(str(p.resolve()) in by for p in __import__('navigation_n06_integrity_r05').required_manager_paths()))
check('canonical_gzserver_target_bound',str(Path('/usr/bin/gzserver').resolve()) in by)
missing=[]
for path in by:
 p=Path(path)
 if p.suffix!='.py' or '/workspace/' not in path:continue
 try:tree=ast.parse(p.read_bytes())
 except (SyntaxError,UnicodeError):continue
 for node in ast.walk(tree):
  names=[a.name for a in node.names] if isinstance(node,ast.Import) else [node.module] if isinstance(node,ast.ImportFrom) and node.module else []
  for name in names:
   dep=P/(name.split('.')[0]+'.py')
   if dep.exists() and str(dep) not in by:missing.append((path,str(dep)))
check('recursive_local_workspace_imports_closed',not missing)
obs=read(E/'wrapper_observations.json');gazenv=obs['units'][0]['normalized_real_exec_environment'];plugins=read(C/'configured_plugins.json')
check('actual_wrapper_eight_local_model_database_observations',len(obs['units'])==8 and all(u['normalized_real_exec_environment']['GAZEBO_MODEL_DATABASE_URI']==policy.MODEL_DATABASE for u in obs['units']))
check('old_empty_database_negative_and_no_external_scene_uri',obs['empty_uri_negative_retained'] and obs['no_external_uri_in_eight_worlds_and_urdfs'])
check('nine_configured_plugins_bound_at_final_wrapper_search_order',len(plugins)==9 and all(i['selected'] in by and Path(next(str(Path(d)/i['filename']) for d in gazenv['GAZEBO_PLUGIN_PATH'].split(':') if d and (Path(d)/i['filename']).is_file())).resolve()==Path(i['selected']) for i in plugins))
graph=read(C/'elf_graph.json');gn=graph['nodes'];contexts=set(graph['contexts'])
check('both_loader_contexts_measured',contexts=={'manager_python','gzserver_after_actual_wrapper'} and all(sum(n['loader_context']==k for n in gn)==602 for k in contexts))
check('all_needed_and_interpreter_dependencies_resolved_bound',all(n['file']==by.get(n['file']['path']) and set(n['needed'])<={r['soname'] for r in n['resolved']} and all(r['binding']==by.get(r['binding']['path']) for r in n['resolved']) and all(x==by.get(x['path']) for x in n['program_interpreters']) for n in gn))
context_proof=True
for node in gn:
 for parent in node['inherited_root_contexts']:
  sources=[n for n in gn if n['loader_context']==node['loader_context'] and n['file']==parent]
  context_proof &= any(node['file']['path'] in {d['binding']['path'] for d in n['resolved']} and all(any(d['soname']==name and d['binding'] in [e['binding'] for e in node['resolved'] if e['soname']==name] for d in n['resolved']) for name in node['standalone_unresolved']) for n in sources)
check('private_wheel_libraries_only_use_proven_parent_context',context_proof)
commands=read(C/'commands.json');check('all_3593_commands_succeeded_and_raw_hashes_match',len(commands)==3593 and all(c['returncode']==0 and b(c['stdout']['path'])==c['stdout'] and b(c['stderr']['path'])==c['stderr'] for c in commands))
check('command_environments_bound',all(type(c.get('environment'))is dict for c in commands))
check('alias_link_text_stable',all(Path(x['alias']).resolve()==Path(x['canonical']) and all(Path(l['path']).is_symlink() and __import__('os').readlink(l['path'])==l['link_text'] for l in x['final_component_links']) for x in read(C/'aliases.json')))
manager=S/'audits/navigation_n06_manager_r05';m=read(manager/'source_evidence_manifest.json');old={i['path']:i for k in ('owned_sources','read_only_dependencies','author_evidence') for i in m[k]}
check('all_866_manager_and_prior_sources_unchanged',len(old)==866 and all(b(p)==v for p,v in old.items()))
with (S/'seed_registry.csv').open() as f:registered=list(csv.DictReader(f))
check('no_n06_seed_or_run_allocation',not any(row.get('run_id') in ids or row.get('seed') in {str(u['seed']) for u in ids.values()} for row in registered) and not any((S/'runs'/i).exists() for i in ids) and not (S/'queue/navigation_n06_ledger.json').exists())
check('seven_parser_fault_tests_pass','Ran 7 tests' in (A/'author_parser_tests_final.log').read_text() and (A/'author_parser_tests_final.log').read_text().rstrip().endswith('OK'))
selfcheck={'schema':'astra_n06_disk_closure_selfcheck_v1','status':'author_checks_pass','checks':checks,'resource_count':len(resources),'elf_unique':602,'elf_context_rows':len(gn),'resolved_nodes':len(nodes),'commands':len(commands),'manager_resources_unchanged':len(old),'scope':'Disk-only; source behavior and independent review/live maps remain separate.'}
write(A/'selfcheck.json',selfcheck)
closure={'schema':'astra_navigation_n06_runtime_closure_v1','stage':'astra_navigation_n06_lookahead_static_pilot','status':'authored_disk_dependency_closure_independent_review_pending',
 'manager_revision':'r05','python_executable':profiles['venv']['canonical_executable'],'python_prefix':profiles['venv']['sys_prefix'],'python_no_user_site':True,
 'execution_environment':spec,'execution_environment_source':b(E/'execution_environment.json'),'wrapper_environment_observations':b(E/'wrapper_observations.json'),
 'resources':resources,'collection':b(C/'collection.json'),'elf_graph':b(C/'elf_graph.json'),'selfcheck':b(A/'selfcheck.json'),
 'restricted_entry_sources':[b(P/name) for name in ('navigation_n06_entry_r05.sh','navigation_n06_bootstrap_r05.py','navigation_n06_environment_r05.py')],
 'disk_only':True,'live_mapped_elf_observed':False,'independent_review':False,'execution_authorized':False,
 'scope':'Conservative disk envelope, actual import/launch resolution, two loader environments and actual shell wrapper prefix. No live node/Gazebo/maps qualification.'}
write(A/'closure.json',closure)
owned=[A/name for name in ('build_disk_closure.py','python_import_probe.py','observe_environment_r05.py','finalize_runtime.py','tests/test_disk_closure.py','API.md')]
evidence=[p for p in sorted(A.rglob('*')) if p.is_file() and '__pycache__' not in p.parts]
refs=[manager/'implementation_report.json',manager/'source_evidence_manifest.json',manager/'freeze_receipt.json',S/'audits/navigation_n06_runtime_disk_closure_independent/r03_draft_risks/review.json']
manifest={'schema':'astra_n06_runtime_disk_source_evidence_v1','owned_sources':[b(p) for p in owned],'runtime_resources':resources,'author_evidence':[b(p) for p in evidence],
 'manager_and_risk_bindings':[b(p) for p in refs],'execution_authorized':False,'reviewed_status_not_claimed':True}
write(A/'source_evidence_manifest.json',manifest)
report={'schema':'astra_n06_runtime_disk_author_report_r05_v1','status':'author_disk_closure_pass_independent_review_pending','closure':b(A/'closure.json'),'source_evidence_manifest':b(A/'source_evidence_manifest.json'),
 'selfcheck':b(A/'selfcheck.json'),'resource_count':len(resources),'total_bytes':collection['total_bytes'],'elf_unique':602,'elf_context_rows':len(gn),'commands':len(commands),'node_rows':64,
 'retained_attempts':[{'path':str(A/'collection_attempt01'),'status':'negative_standalone_private_library_RPATH_context_missing'},
 {'path':str(A/'collection_attempt02'),'status':'complete_r03_draft_not_execution_environment_qualified'},
 {'path':str(A/'collection_attempt03'),'status':'complete_r04_draft_not_execution_environment_qualified'},
 {'path':str(C),'status':'complete_r05_fixed_environment_two_context_disk_envelope'}],
 'old_counterexamples_retained':['NumPy private gfortran standalone ldd unresolved quadmath; final uses actual parent root tables, never guessed extra paths','ambient manager inheritance vs sanitized collector','real wrapper empty model URI restores remote default'],
 'limitations':['disk envelope is not live mapped-ELF or absence-of-all-possible-dlopen proof','Python package envelopes include unexecuted lazy code; no claim all bound files execute','later runtime must use exact env-i bootstrap and recheck live native/graphics mappings','shell exec substitute only changes a sole PATH prefix, removed explicitly in normalized environment; no Gazebo binary executed','OS/kernel/graphics hardware and malicious same-account mutation remain outside source-hash theorem','author closure status is deliberately not reviewed; no release or seed authority'],
 'authority':{'release_created':False,'seed_rows_created':0,'ros_gazebo_started':False,'global_csv_writes':False,'live_maps_claimed':False}}
write(A/'implementation_report.json',report)
allitems={i['path']:i for k in ('owned_sources','runtime_resources','author_evidence','manager_and_risk_bindings') for i in manifest[k]}
if not all(b(p)==v for p,v in allitems.items()):raise RuntimeError('final source/evidence drift')
write(A/'freeze_receipt.json',{'report':b(A/'implementation_report.json'),'manifest':b(A/'source_evidence_manifest.json'),'closure':b(A/'closure.json'),'unique_resources':len(allitems),'postwrite_match':True,'execution_authorized':False})
print(json.dumps({'report':b(A/'implementation_report.json'),'manifest':b(A/'source_evidence_manifest.json'),'closure':b(A/'closure.json'),'runtime_resources':len(resources),'all_frozen_resources':len(allitems)},indent=2))
