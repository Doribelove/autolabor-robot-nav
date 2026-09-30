"""Independent exact derivation and concrete inherited-port evidence; offline."""
from pathlib import Path
from types import SimpleNamespace as NS
import ast,hashlib,json,sys,xml.etree.ElementTree as ET
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';A=Path(__file__).resolve().parent;P=W/'scripts/astra_gazebo';sys.path.insert(0,str(P))
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
der=json.loads((S/'audits/navigation_n05_runtime_derivation/derivation.json').read_bytes());snap=json.loads((A/'source_snapshot_r01.json').read_bytes())
by={Path(r['target']).name:r for r in der['resources']};checks={};details={}
for row in der['resources']:
 assert sha(row['source'])==row['source_sha256'];target=A/'snapshot_r01'/Path(row['target']).relative_to(W);assert sha(target)==row['target_sha256']
checks['9_exact_old_and_snapshot_source_bindings']=True
for name in ['evaluate_episode','dense_sweep','navigation_evidence']:
 row=by[name+'_n05.py'];before=Path(row['source']).read_text();after=(A/'snapshot_r01'/Path(row['target']).relative_to(W)).read_text();expected=before.replace('n04','n05').replace('N04','N05')
 if name=='navigation_evidence':expected=expected.replace('navigation_coordinator_n02','navigation_coordinator_n05').replace("METHODS=('astra_analytic_route_v1',","METHODS=('astra_analytic_preview_eligible_n05','astra_analytic_route_v1',")
 assert after==expected
checks['3_evaluator_geometry_modules_only_namespace_and_method_enum']=True
row=by['astra_navigation_collector_n05.py'];assert (A/'snapshot_r01'/Path(row['target']).relative_to(W)).read_text()==Path(row['source']).read_text().replace('n04','n05').replace('N04','N05').replace('14820','14830')
checks['collector_only_N05_namespace_port14830']=True
for name in ['navigation_coordinator_n05.py','astra_navigation_coordinator_n05.py']:
 row=by[name];before=Path(row['source']).read_text();after=(A/'snapshot_r01'/Path(row['target']).relative_to(W)).read_text()
 expected=before.replace("METHODS=('astra_analytic_route_v1',","METHODS=('astra_analytic_preview_eligible_n05','astra_analytic_route_v1',") if name=='navigation_coordinator_n05.py' else before.replace('navigation_coordinator_n02','navigation_coordinator_n05')
 assert after==expected
checks['coordinator_pure_only_enum_node_only_import']=True
old=Path(by['astra_route_preview_runtime_n05.py']['source']).read_text();new=(A/'snapshot_r01/src/simulation/m2_gazebo/scripts/astra_route_preview_runtime_n05.py').read_text()
def nested(text):
 main=next(x for x in ast.parse(text).body if isinstance(x,ast.FunctionDef) and x.name=='main');return {x.name:ast.dump(x,include_attributes=False) for x in main.body if isinstance(x,ast.FunctionDef)}
assert nested(old)==nested(new);details['identical_runtime_nested_functions']=list(nested(old));checks['all_actual_sensor_health_start_publish_callbacks_AST_identical']=True
# Transaction tail must remain old code after identity declaration except exact added audit payload.
oldtail=old[old.index('    last_wall=time.monotonic();'):]
newtail=new[new.index('    last_wall=time.monotonic();'):]
extra="'full_preview_clearances_m':built['full_preview_clearances_m'],'preview_selection_audit':built['preview_selection_audit'],\n                                "
assert newtail.replace(extra,'')==oldtail
checks['main_locked_selection_command_and_terminal_loop_unchanged_except_audit']=True
assert 'core=PreviewEligibleRuntime(grid,route,urdf,config)' in new and 'core=RouteAwareRuntime' not in new and "if method=='astra_pct_route_v1':" not in new and 'actor_artifact_sha=None' in new
checks['new_node_analytic_selector_only_no_actor']=True
before=ET.fromstring(Path(by['astra_common_navigation_n05.launch']['source']).read_text());after=ET.fromstring((A/'snapshot_r01/src/simulation/m2_gazebo/launch/astra_common_navigation_n05.launch').read_text())
groups=after.findall('group');assert len(groups)==2
assert [g.get('if') for g in groups]==["$(eval arg('method') == 'astra_analytic_route_v1')","$(eval arg('method') == 'astra_analytic_preview_eligible_n05')"]
assert [g.find('node').get('type') for g in groups]==['astra_route_coordinated_runtime.py','astra_route_preview_runtime_n05.py']
for g in groups:
 n=g.find('node');assert n.get('name')=='astra_route_runtime' and n.find("remap[@from='/cmd_vel_navigation']").get('to')=='/astra/method_cmd'
for sel in ['include',"param[@name='robot_description']","node[@name='navigation_command_relay']","node[@name='astra_common_route_provider']","node[@name='m2_robot_state_publisher']"]:
 x,y=before.find(sel),after.find(sel);x.tail=y.tail=None;assert ET.tostring(x)==ET.tostring(y)
assert after.find("node[@name='astra_navigation_collector']").get('required')=='true'
checks['launch_exact_control_treatment_shared_physics_and_required_collector']=True
import run_navigation_n05 as manager
schedule=manager.design()['schedule'];assert len(schedule)==24
for i in range(12):
 pair=schedule[2*i:2*i+2];assert {r['seed'] for r in pair}=={263002001+i} and {r['attempt'] for r in pair}=={1};assert len({r['parent_geometry_id'] for r in pair})==1
 assert [r['method'] for r in pair]==(['astra_analytic_route_v1','astra_analytic_preview_eligible_n05'] if i%2==0 else ['astra_analytic_preview_eligible_n05','astra_analytic_route_v1'])
checks['actual_bound_design_24units_12pairedseeds_counterbalanced']=True
text=(A/'snapshot_r01/scripts/astra_gazebo/run_navigation_n05.py').read_text();execute=next(x for x in ast.parse(text).body if isinstance(x,ast.FunctionDef) and x.name=='execute');lines=text.splitlines();body='\n'.join(lines[execute.lineno-1:execute.end_lineno])
assert body.index("predecessor =")<body.index('UnchangedReleaseSession') and body.index("if ledger.exists()")>body.index('UnchangedReleaseSession')
assert "len(same_seed)!=2" in body and "reserved_exact_n05_release" in body and "if not integrity_pass:" in body and "retired_unrun" in body
checks['liveness_ledger_registry_not_cached_physical_negatives_continue']=True
assert '14830' in body and '14831' in body
# Actual inherited cleanup function with fake transport, no /proc read/signals/socket bind.
import run_smoke_a02 as common
calls=[];records=[]
class Socket:
 def __enter__(self):return self
 def __exit__(self,*a):pass
 def setsockopt(self,*a):pass
 def bind(self,address):calls.append(address)
old_socket,old_owned,old_atomic=common.socket.socket,common.owned_processes,common.atomic
try:
 common.socket.socket=Socket;common.owned_processes=lambda root:[];common.atomic=lambda path,value:records.append((str(path),value));common.stop_group(NS(poll=lambda:0),A/'fake_completed_N05')
finally:common.socket.socket,common.owned_processes,common.atomic=old_socket,old_owned,old_atomic
assert [x[1] for x in calls]==[14820,14821]
checks['N05_actual_terminal_ports_verified']=False
details['concrete_cleanup_counterexample']={'observed_bind_addresses':calls,'written_record':records,'new_stage_ports':[14830,14831],'meaning':'No actual ownedprocess leak proven; inherited helper onlyverifies oldports, notnewstageports. NeedsN05post-cleanup check andtruthfulportrecord.'}
report=dict(status='snapshot_r01_review_has_one_concrete_port_integration_gap',checks=checks,details=details,source_snapshot_sha256=sha(A/'source_snapshot_r01.json'),author83_replay_log_sha256=sha(A/'author_tests_r01.log'),no_ROS=True,no_real_process_or_socket_operations=True)
(A/'checks_r01.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({'checks':checks,'detail':details['concrete_cleanup_counterexample']},indent=2))
