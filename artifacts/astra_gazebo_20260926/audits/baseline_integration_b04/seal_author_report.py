import ast,hashlib,json,sys,difflib,platform
from pathlib import Path
import numpy as np,yaml
A=Path(__file__).resolve().parent;S=A.parents[1];W=S/'workspace';H=W/'scripts/astra_gazebo';G=W/'src/simulation/m2_gazebo'
sys.path.insert(0,str(H))
import baseline_rear_evidence_b04 as e
import navigation_evidence_b04 as q

def b(p):
 p=Path(p).resolve();return {'path':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'bytes':p.stat().st_size}
def save(name,x):(A/name).write_text(json.dumps(x,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
oldreview=S/'audits/baseline_rear_runtime_b03_independent/review.json';old=json.loads(oldreview.read_text())
assert all(b(x['path'])['sha256']==x['sha256'] for x in old['resources'])
check=json.loads((A/'derivation_check_final_r1.log').read_text());assert check['derived_outputs_identical']
new=[H/'baseline_rear_evidence_b04.py',H/'navigation_evidence_b04.py',G/'scripts/astra_navigation_collector_b04.py',G/'launch/astra_baseline_rear_b04.launch',H/'tests/test_baseline_integration_b04.py']
for p in new:
 if p.suffix=='.py':ast.parse(p.read_text())
assert (G/'scripts/astra_navigation_collector_b04.py').stat().st_mode&0o111
assert 'Ran 48 tests' in (A/'tests_final_r2.log').read_text() and (A/'tests_final_r2.log').read_text().rstrip().endswith('OK')
assert 'Ran 27 tests' in (A/'inherited_b03_tests.log').read_text() and (A/'inherited_b03_tests.log').read_text().rstrip().endswith('OK')
original=yaml.safe_load((G/'config/astra_navigation_common_footprint.yaml').read_text());rear=yaml.safe_load((G/'config/astra_baseline_rear_b03.yaml').read_text())
footprints={}
for name in ('global_costmap','local_costmap','TebLocalPlannerROS'):
 oldpoints=original[name]['footprint'] if 'costmap' in name else original[name]['footprint_model']['vertices']
 newpoints=rear[name]['footprint'] if 'costmap' in name else rear[name]['footprint_model']['vertices']
 expected=np.asarray(oldpoints)+np.asarray([.325,0.]);error=float(np.max(np.abs(np.asarray(newpoints)-expected)));assert error<=1e-15
 footprints[name]={'points':len(oldpoints),'translation_x_m':.325,'maximum_component_error_m':error}
assert original['global_costmap']['footprint_padding']==rear['global_costmap']['footprint_padding']==0.
assert original['TebLocalPlannerROS']['min_obstacle_dist']==.33

def functions(p):return {n.name:ast.dump(n,include_attributes=False) for n in ast.parse(p.read_text()).body if isinstance(n,ast.FunctionDef)}
a=functions(H/'navigation_evidence_n02.py');c=functions(H/'navigation_evidence_b04.py')
critical=['navigation_outcome','gate_record','terminal_record','validate_services','readiness_guard','wait_condition','validate_robot_description']
assert all(a[n]==c[n] for n in critical)

def core(p):
 tree=ast.parse(p.read_text());cl=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='NavigationCoordinator');return {n.name:ast.dump(n,include_attributes=False) for n in cl.body if isinstance(n,ast.FunctionDef)}
c3=core(H/'navigation_coordinator_b03.py');c5=core(H/'navigation_coordinator_n05.py');samecore=[n for n in c3 if n in c5 and c3[n]==c5[n]];diffcore=[n for n in c3 if n not in samecore]
assert not diffcore
launch_report=A/'launch_preflight_final_r2/report.json';launch=json.loads(launch_report.read_text());assert len(launch['rows'])==2 and all(x['pass_'] for x in launch['rows'])
for method in e.METHODS:
 out=launch_report.parent/method;value=json.loads((out/'contract.json').read_text());q.validate_contract(value)
 assert not (S/'runs'/value['run_id']).exists()
for oldname,newname in ((H/'navigation_evidence_n02.py',H/'navigation_evidence_b04.py'),(G/'scripts/astra_navigation_collector_n02.py',G/'scripts/astra_navigation_collector_b04.py'),(G/'launch/astra_baseline_rear_b03.launch',G/'launch/astra_baseline_rear_b04.launch')):
 (A/(newname.name+'.diff')).write_text(''.join(difflib.unified_diff(oldname.read_text().splitlines(True),newname.read_text().splitlines(True),fromfile=str(oldname),tofile=str(newname))))
save('inherited_scope_checks.json',{'B03_review':b(oldreview),'B03_53_resources_unchanged':True,'frozen_derivation_inputs':check['frozen_inputs'],'physical_task_functions_AST_identical':critical,'N05_N02_B03_shared_coordinator_methods_identical':samecore,'footprint_translation_recalculation':footprints,'private_goal_containment_m':e.goal_tolerance_contract(),'fresh_Gazebo_or_performance_claim':False})
# Preserve every negative invocation and local revision, while distinguishing it from current author bytes.
resources={str(p.resolve()):b(p) for p in new}
for x in old['resources']:resources[str(Path(x['path']).resolve())]=b(x['path'])
for p in (oldreview,H/'navigation_evidence_n02.py',G/'scripts/astra_navigation_collector_n02.py',H/'navigation_coordinator_n05.py',G/'config/astra_navigation_common_footprint.yaml',G/'config/benchmark_teb_v0.yaml',G/'config/benchmark_dwa_nonholonomic_r1.yaml',G/'src/astra_physical_ackermann_p05_plugin.cpp',H/'common_route.py',G/'src/astra_common_route_global_planner.cpp',G/'scripts/astra_common_route_provider.py'):
 resources[str(p.resolve())]=b(p)
for p in A.rglob('*'):
 if p.is_file() and p.name not in ('source_manifest.json','implementation_report.json','implementation_report.md') and 'roslog' not in p.parts and 'ros_home' not in p.parts:resources[str(p.resolve())]=b(p)
manifest={'schema':'astra_b04_offline_author_resources_v1','scope':'author offline source/test/launch evidence, not executable release or full native dependency closure','new_source_files':[b(p) for p in new],'resources':list(sorted(resources.values(),key=lambda x:x['path']))}
save('source_manifest.json',manifest)
report={'schema':'astra_baseline_integration_b04_author_report_v1','status':'author_static_rear_integration_offline_checks_pass_independent_review_pending','author_implementation':True,'independent_review':False,'execution_authorized':False,'execution_ready':False,'ROS_or_Gazebo_started':False,'seed_registered':False,'execution_release_written':False,'training_started':False,'dynamic_actor_integration_qualified':False,'live_static_integration_qualified':False,'formal_safety':False,'performance_claim':False,'methods':list(e.METHODS),'source_manifest':b(A/'source_manifest.json'),'new_sources':[b(p) for p in new],'design_and_scope':b(A/'API_and_scope.md'),'launch_preflight':b(launch_report),'inherited_checks':b(A/'inherited_scope_checks.json'),'tests':{'new_author_groups':48,'new_log':b(A/'tests_final_r2.log'),'inherited_B03_groups':27,'inherited_log':b(A/'inherited_b03_tests.log'),'combined_groups':75,'actual_coordinator_order_combinations':4,'real_ROS_message_classes_used':True,'synthetic_rosbag_written_and_replayed':True,'Gazebo_evidence':False},'source_and_goal_clock_semantics':{'odom_and_path_source':'retained actual message Header stamps','adapter_received_available':'ROS callback/availability clock, not physical measurement stamp','bag_receipt':'recorder clock, independent of collector callback clock','private_action_result':'receipt clock diagnostic, never shared task authority','task_authority':'original actual base odometry plus independently replayed world base geometry'},'failure_history':[{'path':str(A/'launch_attempt01.log'),'reason':'author assertion compared tuple with ROSLaunchConfig list remaps; parse was successful, harness type normalization repaired'}, {'path':str(A/'tests_final.log'),'reason':'rosbag default messages do not carry caller connection header; explicit return_connection_header=True required'}, {'path':str(A/'tests_final_r1.log'),'reason':'returned ROS bag caller value is bytes; strict UTF-8 decode required before matching live string identity'}], 'retained_B03_negative':'actual publish_pending partial batch first output without receipt reproduced and fails B04 completeness; no synthesized receipt','remaining_blockers':['No B04 execution manager/release/schedule or full native/runtime closure; separate independent review and fresh bounded qualification required.','Dynamic actor methods unsupported; static-only contract rejects direct actors/dynamic models/unexpanded includes.','Tilted yaw-only costmap polygon is not exact full 3D collision projection; known 0.0004061653716359581m example preserved without changing margins.','Transformed oriented guide kinks and stricter private goals require execution qualification; callback order is not a physical timing guarantee.','DWA residual lateral velocity, stop-and-rotate and frozen acc_lim_y2.5 may still fail; no generic repair or performance claim.','Trusted graph/caller observations do not prove adversarial isolation or instantaneous complete transport authority.'],'environment':{'python':sys.version,'executable':sys.executable,'numpy':np.__version__}}
save('implementation_report.json',report)
(A/'implementation_report.md').write_text('# B04 作者离线结论\n\n静态后轴接线、采集完整性和参数展开检查通过，独立审查待完成；没有运行 ROS/Gazebo、登记 seed 或写 execution release。\n\n48 个新用例及 27 个继承 B03 用例通过，覆盖真实消息和合成 rosbag 往返、同源缺包/错 frame/TF/目标/协方差反例、实际部分批输出缺回执、两种方法四个 private/base 终态先后顺序。两份 actual roslaunch 参数展开通过，方法、remap、paused world、required collector、后轴 frame/odom/完整路径/平移 footprint 及原物理参数已核对。53 个继承 B03 资源逐个 rehash 不变。\n\n新 q.assess 同时审核 collector 和 bag 的 rear 证据，并匹配正式区间至暂停 watermark 的原始 odom/后轴 odom/回执字段。测试先发现默认 rosbag 不附 connection header，后发现返回 header caller 为 bytes；两次负例及修复前源快照均保留，最终显式读取并严格 UTF-8 解码。首次 launch harness 的 list/tuple remap 断言失败也保留。\n\n公共原 base goal 及安全/timeout 数值算法不改，私有 SUCCESS 仅诊断。动态 actor 白名单、真实静态运行、完整 native/interpreter/转换闭包、倾斜 footprint 与 DWA stop/rotate/残余 vy 尚未资格；详见 API_and_scope.md。没有形成性能、公平比较有效性、形式安全或实车结论。\n')
print(json.dumps({'report':b(A/'implementation_report.json'),'manifest':b(A/'source_manifest.json'),'new_sources':[b(p) for p in new]},ensure_ascii=False))
