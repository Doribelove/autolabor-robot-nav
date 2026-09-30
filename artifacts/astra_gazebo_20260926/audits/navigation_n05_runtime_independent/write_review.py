from pathlib import Path
import hashlib,json
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';A=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
bind=lambda p:dict(path=str(Path(p).resolve()),sha256=sha(p))
r=json.loads((A/'checks_r02.json').read_bytes());launch=json.loads((A/'saved_launch_checks.json').read_bytes());assert r['all_checks_pass'];assert len(launch['rows'])==24
for log,n in [('author_tests_r02.log',14),('independent_tests_r02.log',7),('scene_author_tests_replay.log',15)]:assert str(n)+' passed in ' in (A/log).read_text()
errors='''Preserved tool/inspection history; none changed author source or experiment state.
1. Before the pause, a functions.exec JavaScript read orchestration had an extra closing parenthesis: SyntaxError: Unexpected token ')'. No shell ran. Corrected orchestration only.
2. Resumed read-only metadata inspection used report['rows'] and reported resource count under the wrong key; actual preparation schema is input_rows/files. Tool ended with KeyError: 'rows' after printing existing keys. Corrected to input_rows/files in check_r02.py before any assessment. First executable independent check_r02 attempt passed; there was no false assessment output.
3. Same metadata inspection emitted environment RequestsDependencyWarning (urllib3/chardet mismatch). The passing tests and offline checks are recorded separately; no dependency change performed.
'''
(A/'tool_error_history.txt').write_text(errors)
cmds=[
 'PYTHONDONTWRITEBYTECODE=1 '+str(W/'.venv/bin/python')+' -m pytest -q -p no:cacheprovider '+str(W/'scripts/astra_gazebo/tests/test_navigation_n05_pipeline.py')+' '+str(W/'scripts/astra_gazebo/tests/test_navigation_process_cleanup_n05.py')+' > '+str(A/'author_tests_r02.log')+' 2>&1',
 'PYTHONDONTWRITEBYTECODE=1 '+str(W/'.venv/bin/python')+' '+str(A/'check_r02.py')+' > '+str(A/'checks_r02_attempt01.log')+' 2>&1',
 'PYTHONDONTWRITEBYTECODE=1 '+str(W/'.venv/bin/python')+' -m pytest -q -p no:cacheprovider '+str(A/'test_r02_independent.py')+' > '+str(A/'independent_tests_r02.log')+' 2>&1',
 'PYTHONDONTWRITEBYTECODE=1 '+str(W/'.venv/bin/python')+' -m pytest -q -p no:cacheprovider '+str(W/'scripts/astra_gazebo/tests/test_prepare_navigation_n05_scenes.py')+' > '+str(A/'scene_author_tests_replay.log')+' 2>&1',
 'PYTHONDONTWRITEBYTECODE=1 '+str(W/'.venv/bin/python')+' '+str(A/'check_saved_launch.py')+' > '+str(A/'saved_launch_checks_attempt01.log')+' 2>&1']
(A/'commands.txt').write_text('\n'.join(cmds)+'\n')
checks={
 'r02_exact14_source_bindings':True,
 'cleanup_function_byte_equal_except_ports_and_report_key':True,
 'runner_only_import_resource_roster_and_cleanup_call_changed':True,
 'r01_oldport_counterexample_preserved_unmodified':True,
 'actual_newport_success_and_both_occupied_failures':True,
 'cleanup_failure_yields_infrastructure_receipt_and_retirement':True,
 'foreign_or_wrong_release_reservation_rejected_before_allocation':True,
 'preparation_failure_consumed_once_retired_remaining':True,
 'control_treatment_callbacks_commands_shared_physics_unchanged_except_selection_adapter_and_audit':True,
 'coordinator_only_method_enum_and_evaluator_only_namespaces':True,
 'cache_excludes_dynamic_predecessor_liveness_ledger_and_registry':True,
 'exact24_schedule_12_paired_seeds_6_parents':True,
 'prepared183_files25_parent_bindings_rehashed':True,
 'all12_worlds_exact_derivation_and_parent_collision_geometry':True,
 'all24_in_memory_event_identities_match_new_schema':True,
 'map_actor_omission_and_byte_equal_N04_static_maps_routes':True,
 'occluded_old_sin4_negative_kept_and_static_only_supplement_scope':True,
 'actual24_saved_launch_parameters_accept_P05_backend_seed_and_9shape_runtime_constructors':True,
 'current_new_and12_inherited_ELF_hashes_match_root_record':True}
limitations=[
 'Offline source/component and prepared-input review only; no actual N05 release verify_release invocation, ROS/Gazebo, native build, benchmark, seed allocation or process control by reviewer.',
 'Pipeline transport/process/ports were stubbed. New cleanup signals/PID ownership logic is byte-inherited and not requalified by a new real process fixture here. Port bind success proves availability only at cleanup-check time, not future reservation.',
 'Root performed actual roslaunch --dump-params and native build. Reviewer read/hash-checked saved outputs and instantiated actual pure runtime constructors from all24 parameter dumps, but did not rerun those commands or observe the pre-build moment. The13 current ELF hashes match recorded before/after facts.',
 'Source snapshot_r02 records native_built=false at its historical source-only creation. Later root native/launch reports are separate evidence, not a retroactive change to that snapshot.',
 'Six reused development parent geometries and two parameter repeats; not12 independent geometries, final held-out outcomes, superiority, generalized safety, or deployment evidence.',
 'Prepared reference URDF has historical seed260926033 and is geometry-only. Actual24 saved URDFs carry proposed per-run seeds and accepted P05 backend; later per-run conversion, mapped implementation and release bindings remain required.',
 'Original occluded sin4 witness remains negative for both repetitions. Previously independently reviewed quintic existence witness transfers only because static geometry and reference model are byte/equivalence checked; no new901-sample witness replay, physical tracking or dynamic avoidance claim.',
 'Actorless world identity is checked through exact schedule plus scene/world closure; no actor metadata exists to embed an execution identity. In-memory binding and prepared files do not allocate, authorize, or consume a run.',
 'Inherited sensor, first-terminal coordinator and zero-command protocol have exact unchanged callbacks/loop evidence and prior fake-transport tests. This is not proof that all possible runtime races or future sensor faults are absent.',
 'Selector is analytic-only: original backup mask stays unchanged, moving choices additionally require original unrounded full-preview clearance>=0, stop remains available under old mask/prior. Rounded trace arrays cannot replace those original values; runtime latency/selection benefit require fresh results.',
 'Nominal gate-to-goal axial distance is9m; recomputed common guide length is9.382549429527351m for static avoidance and9.173214386367965m for occluded emergence. All other four families are9m.',
 'Mutable seed registry/current state and N04 result files are not review resource bindings. Execution guard checks predecessor terminal/liveness and exact fresh reservations at runtime outside immutable cache.']
resources={x['path']:x for x in r['resources']+launch['resources']}
extra=[A/n for n in ['source_snapshot_r01.json','source_snapshot_r02.json','check_r01.py','checks_r01.json','checks_r01.log','author_tests_r01.log','check_r02.py','checks_r02.json','checks_r02_attempt01.log','runner_r01_to_r02.diff','stop_group_port_only.diff','test_r02_independent.py','independent_tests_r02.log','author_tests_r02.log','scene_author_tests_replay.log','check_saved_launch.py','saved_launch_checks.json','saved_launch_checks_attempt01.log','commands.txt','tool_error_history.txt']]
extra += [S/n for n in ['audits/navigation_n05_runtime_derivation/derivation.json','audits/navigation_n05_root_scene_closure_r01/report.json','audits/navigation_n04_scenes_independent/review.json','audits/navigation_n04_scene_preparation/occlusion_static_witness_r1.json']]
extra += [W/'scripts/astra_gazebo/tests/test_prepare_navigation_n05_scenes.py']
for p in extra:resources[str(p.resolve())]=bind(p)
# Freeze review against actual current resource bytes; no mutable global indexes bound.
for p,item in resources.items():assert sha(p)==item['sha256'],p
report=dict(schema='astra_navigation_n05_runtime_independent_review_v1',status='independent_N05_r02_source_and_prepared_input_review_pass',independent_review=True,pass_=True,reviewed_stage='astra_navigation_n05_preview_selection',scope='offline source/component plus saved prepared-input closure; final release/runtime qualification separate',checks=checks,check_count=len(checks),source_snapshot=bind(A/'source_snapshot_r02.json'),reviewed_runner_sha256=sha(W/'scripts/astra_gazebo/run_navigation_n05.py'),reviewed_cleanup_sha256=sha(W/'scripts/astra_gazebo/navigation_process_cleanup_n05.py'),reviewed_treatment_node_sha256=sha(W/'src/simulation/m2_gazebo/scripts/astra_route_preview_runtime_n05.py'),r01_retained_failure=bind(A/'checks_r01.json'),resolved_finding=dict(id='N05_r01_cleanup_checked_old_ports',severity='preexecution_integration_blocker',r01_actual_binds=[14820,14821],r02_actual_binds=[14830,14831],no_actual_process_leak_claim=True,evidence=[bind(A/'stop_group_port_only.diff'),bind(A/'independent_tests_r02.log')]),tests=dict(prior_r01_author_replay=83,r02_author_pipeline_ports=14,new_independent_fault_cases=7,scene_author_replay=15,mechanical_assertions=542,mechanical_assertions_are_not_independent_experimental_trials=True,actual_saved_parameter_runtime_constructors=24),prepared_scene_review=dict(inputs=12,files_rehashed=183,parent_bindings_rehashed=25,per_method_actor_identity_cases=24,independent_geometry_count=6,new_geometry_count=0,rows=r['input_summaries']),root_evidence_attribution=dict(scene_closure='Independently recomputed metadata/XML/collision signatures/map and guide equality, resource hashes; not merely author report summary.',native='Root build and prebuild observations only; independently rehashed1 new+12 inherited current ELFs and bound saved log.',launch='Root actual24 dump-params invocations only; independently rehashed/parsed all24 outputs and constructed both actual pure runtime classes from those URDF/bundle/seed/config inputs.'),limitations=limitations,full_runtime_release_review=False,execution_authorized_by_this_review=False,ROS_started=False,native_build_run=False,seed_allocated=False,resources=list(resources.values()))
with (A/'review.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
text='''N05 r02 独立源码与准备输入审查通过；结论仅覆盖离线组件，不代替最终 release 审查或真实运行资格。

r01 的真实负例完整保留在 checks_r01.json：旧 stop_group 只核查14820/14821，未核查 N05 的14830/14831。r02 新函数与旧函数逐字一致，差异仅两端口及 cleanup.json 对应键；runner仅新增helper导入、资源绑定和调用。没有声称旧版本实际泄漏进程。新端口空闲/占用反例、cleanup失败终态、准备失败、错误预约拒绝均在实际函数上配合假传输执行。

实际验证：r02作者14项、独立7项、场景作者15项均通过；之前r01作者83项记录保留。14份源码冻结SHA全匹配。542条细项断言主要是哈希、字段与逐字等价核查，不能当作542次独立实验。报告绑定代码、实际命令、日志和完整差异。

12个实际准备输入的183个文件和25个parent绑定全部重新哈希；从原parent重新派生的小型XML/metadata与现存world精确一致，碰撞几何保持，全部24个方法事件身份在内存绑定后通过新actor schema。地图不含actor，PGM/map.yaml/common_route.json与原N04逐字一致。两方法每对共享物理URDF，24份实际保存的launch参数都进入对应纯runtime构造器，正确接受P05库、实际seed及9个collision构件；未启动ROS。root执行过的24次真实dump-params和native build仅作为外部记录引用，本次实际完成的是保存参数的解析/构造器调用、13个当前ELF哈希及构建日志哈希检查，没有重做build或观察构建前时刻。

新选择器只改变移动动作选择资格：原backup mask保持；移动动作增加原始未四舍五入full-preview clearance≥0要求，停车资格和旧prior保留。控制方法仍旧节点，处理方法为新节点；17个嵌套传感器/健康/start/publish回调AST相同，锁内选择与命令循环除审计字段外逐字相同。共同coordinator仅新增方法enum；collector/evaluator只声明版本/端口变化。实际动态前驱状态、进程、ledger与registry检查都在不可变文件缓存外；基础设施失败停队列，任务阴性保留继续。

六个旧parent×两个参数重复不等于12个独立几何。遮挡场景原sin⁴见证两行仍为阴性；此前独立审过的quintic静态见证仅因几何/参考模型一致而继承，没有重算901点或证明动态可行。其物理平面区间下界0.47908380119522986m，减0.33m为0.14908380119522985m，steerbound0.28825712577055107rad。名义沿x距离9m；实际共同guide静态绕障9.382549429527351m、遮挡9.173214386367965m，其余四族9m。

边界：prepared参考模型为旧seed260926033，仅几何用途；实际每回合URDF/转换/映射实现仍须最终release和运行证据绑定。actorless身份靠exact schedule与world/scene闭包，不能伪称world内嵌一次性身份。端口反例使用假socket，不是实际进程资格测试；端口可绑定只证明检查当时空闲。未调用真实execute/verify_release、未登记seed、未读取新final outcomes，也未修改作者源码、历史阴性、全局索引或仿真状态。
'''
with (A/'review.md').open('x') as f:f.write(text)
print(json.dumps({'status':report['status'],'report':bind(A/'review.json'),'markdown':bind(A/'review.md'),'semantic_checks':len(checks),'bound_resources':len(resources)},indent=2))
