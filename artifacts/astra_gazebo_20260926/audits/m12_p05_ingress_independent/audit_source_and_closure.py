#!/usr/bin/env python3
"""Read-only P05 structural/inherited closure review; never invokes execute/preflight ROS."""
import ast,copy,csv,difflib,hashlib,importlib.util,json,sys
from pathlib import Path
import yaml
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';P=W/'src/simulation/m2_gazebo/scripts/astra_physical_actuation_probe_p05.py';M=W/'scripts/astra_gazebo/run_m12_physics_p05.py';sys.path.insert(0,str(M.parent))
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
spec=importlib.util.spec_from_file_location('p05_review_manager',M);m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
design,parent,rows=m.load_design();m.check_seed_freshness(rows)
oldP=P.with_name('astra_physical_actuation_probe_p04.py');before=ast.parse(oldP.read_bytes());after=ast.parse(P.read_bytes())
functions=lambda tree:{n.name:n for n in tree.body if isinstance(n,ast.FunctionDef)}
bf,af=functions(before),functions(after)
unchanged={n:ast.dump(bf[n])==ast.dump(af[n]) for n in bf if n not in ('source_packet_integrity','assess','runtime','validate_effective_urdf')};assert all(unchanged.values())
def gate_conditions(fn):
 result={}
 for n in ast.walk(fn):
  if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='gate':result[ast.dump(n.args[0])]=ast.dump(n.args[1])
 return result
bg,ag=gate_conditions(bf['assess']),gate_conditions(af['assess']);common=bg.keys()&ag.keys();same_conditions={k:bg[k]==ag[k] for k in sorted(common)};assert all(same_conditions.values())
def physical_core(fn):
 start=next(i for i,n in enumerate(fn.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='meta' for t in n.targets))
 end=next(i for i,n in enumerate(fn.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='phase_metrics' for t in n.targets))
 return ast.dump(ast.Module(body=fn.body[start:end],type_ignores=[]))
assert physical_core(bf['assess'])==physical_core(af['assess'])
config=[]
for row in rows:
 c=yaml.safe_load(m.read_bound(S/row['contract_path'],row['contract_sha256']));assert c['gates']==parent['gates'] and c['phase_schedule']==parent['phase_schedule'] and c['world_fixture']==parent['world_fixture'];config.append({'run_id':row['run_id'],'contract_sha256':row['contract_sha256'],'all_gates_equal_P02':True,'phase_schedule_equal_P02':True,'world_fixture_equal_P02':True})
inherited=design['runtime_resource_parent'];release_path=S/inherited['path'];release=json.loads(m.read_bound(release_path,inherited['sha256']));resources=[]
for row in release['resources']:
 path=Path(row['path']);current=sha(path);expected=m.active_inherited_digest(design,row);resources.append(dict(row,current_sha256=current,expected_active_sha256=expected,match=current==expected,build_metadata_replacement=expected!=row['sha256']))
assert len(resources)==450 and all(r['match'] for r in resources)
assert {r['path'] for r in resources if r['build_metadata_replacement']}==set(design['updated_build_dependencies'])
old_p03=json.loads((S/'contracts/m12_physics_p03_release.json').read_bytes());old_roster={(r['path'],r['sha256']) for r in old_p03['resources']};parent_roster={(r['path'],r['sha256']) for r in release['resources']};assert len(old_roster)==420 and old_roster<=parent_roster
prior={'P02':m.p02_protocol(design),'P03':m.p03_terminal(design),'P04':m.p04_terminal(design)}
oldM=M.with_name('run_m12_physics_p04.py');oldfunctions=functions(ast.parse(oldM.read_bytes()));newfunctions=functions(ast.parse(M.read_bytes()))
class Identity(ast.NodeTransformer):
 def visit_Constant(self,n):
  if isinstance(n.value,str):n.value=n.value.replace('p05','p04').replace('P05','P04')
  if type(n.value)is int and n.value in (260926017,260926018,260926019):n.value-=5
  return n
manager_methods={}
for name in ('classify_report','render_launch','append_registry','write_receipt','execute_row','matrix_summary','execute','check_seed_freshness','main'):
 a=Identity().visit(copy.deepcopy(newfunctions[name]));b=oldfunctions[name];manager_methods[name]=ast.dump(a)==ast.dump(b)
# render_launch necessarily switches to passive-ingress-enabled spawn; all controller args are checked by contracts and author tests.
assert all(v for k,v in manager_methods.items() if k!='render_launch')
report={'offline_only':True,'ROS_Gazebo_started':False,'build_started':False,'numeric_physical_core_AST_exactly_P04':True,'common_gate_condition_AST_equal':same_conditions,'unchanged_pure_helpers':unchanged,'contracts':config,'inherited_resource_count':len(resources),'all450_bindings_match_explicit_whitelist':True,'old420_roster_retained_in450':True,'updated_build_dependencies':design['updated_build_dependencies'],'old_negative_prerequisites_preserved':prior,'manager_methods_same_after_identity_substitution':manager_methods,'no_P05_runtime_allocated_or_release':not m.RUNTIME_LEDGER.exists() and not (S/'contracts/m12_physics_p05_release.json').exists() and all(not (S/'runs'/r['run_id']).exists() for r in rows),'resources':[{'path':str(p),'sha256':sha(p)} for p in [P,M,oldP,oldM,m.DESIGN,release_path,S/'contracts/m12_physics_p02.yaml',S/'audits/m12_physics_p05_driver_root_review.json']],'command':'/usr/bin/python3 '+str(Path(__file__).resolve()),'script_sha256':sha(__file__)}
assert report['no_P05_runtime_allocated_or_release']
(D/'source_and_closure_review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n');(D/'inherited_450_active_bindings.json').write_text(json.dumps(resources,indent=2)+'\n')
for p in (P,M,m.DESIGN):
 target=D/(p.stem+'_reviewed_'+sha(p)[:12]+p.suffix);target.write_bytes(p.read_bytes())
(D/'probe_final_diff_from_p04.patch').write_text(''.join(difflib.unified_diff(oldP.read_text().splitlines(True),P.read_text().splitlines(True),fromfile='frozen_P04',tofile='new_P05')))
print(json.dumps({'source_sha256':sha(P),'manager_sha256':sha(M),'450bindings':True,'physical_core_AST_equal':True,'all_common_gate_conditions_equal':True,'prior_statuses':{k:v['status'] for k,v in prior.items()},'manager_methods':manager_methods,'no_P05_allocated':True},indent=2))
