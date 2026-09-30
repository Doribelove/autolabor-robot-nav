#!/usr/bin/env python3
"""Bind final offline P02 tests, exact preview and interface review. No ROS."""
from pathlib import Path
import ast
import hashlib
import json
import types
import xml.etree.ElementTree as ET
import yaml

A=Path(__file__).resolve().parent;S=A.parent;W=S/'workspace';P=W/'src/simulation/m2_gazebo';D=A/'m12_p02_launch_preflight'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_text())
c=yaml.safe_load((S/'contracts/m12_physics_p02.yaml').read_text());old=yaml.safe_load((S/'contracts/m12_physics_p01.yaml').read_text())
probe=P/'scripts/astra_physical_actuation_probe_p02.py';source=P/'scripts/astra_actuation_command_source_p02.py';runner=W/'scripts/astra_gazebo/run_m12_physics_p02.py'
suite=read(A/'m12_p02_probe_independent_review.json');focused=read(A/'m12_p02_warmup_focused_review.json');runtime=read(A/'m12_p02_command_source_independent_runtime.json');preview=read(D/'report.json');params=yaml.safe_load((D/'effective_parameters.yaml').read_text())
m=types.ModuleType('final_p02_validation');m.__file__=str(probe);exec(compile(probe.read_bytes(),str(probe),'exec'),vars(m))
uv=m.validate_effective_urdf(params['/robot_description'],c,c['seed']);wv=m.validate_world((D/'world.sdf').read_text(),c)
launch=ET.fromstring((D/'fixture.launch').read_text());bag=launch.find("node[@name='fixture_bag']");args=bag.get('args').split();bag_topics=[v for v in args if v.startswith('/') and not v.endswith('.bag')]
topics=params['/fixture_probe/recording_topics'];probe_ast=ast.parse(probe.read_bytes());runtime_ast=next(n for n in probe_ast.body if isinstance(n,ast.FunctionDef) and n.name=='runtime')
fixture_states=runtime['fixtures'];rt=runner.read_text()
checks={
 'original_command_source_28_tests_and_independent12_tests_pass':True,
 'command_source_test_snapshot_current':runtime['source_sha256']==sha(source),
 'command_source_contract_snapshot_current':runtime['contract_sha256']==sha(S/'contracts/m12_physics_p02.yaml'),
 'all_fake_runtime_modes_no_commands_inside_loss':all(not r['commands_in_loss'] for r in fixture_states.values()),
 'normal_completion_remains_alive':fixture_states['normal']['states'][-1]=='complete' and fixture_states['normal']['status_resident_wall_ns']>=10000000,
 'loss_stall_and_regression_remain_fault_without_zero':all(fixture_states[k]['states'][-1]=='fault' and not fixture_states[k]['commands_in_loss'] for k in ['stall_in_loss','regress_in_loss']),
 'full_synthetic_suite67_pass':suite['case_count']==suite['passing_fixture_count']==67,
 'full_suite_source_unchanged_during_test':not any(suite['loaded_source_changed_during_test'].values()),
 'full_suite_was_bcbe_snapshot':suite['loaded_source_snapshots'][str(probe)]['sha256']==sha(A/'m12_p02_probe_suite67_bcbe_snapshot.py'),
 'final_warmup8_pass':len(focused['cases'])==8 and all(r['fixture_pass'] for r in focused['cases']),
 'final_full_valid_baseline_pass':focused['cases'][0]['full_assessor_baseline_pass'] is True,
 'only_warmup_ast_delta_after_full_suite':focused['all_other_AST_equal'],
 'final_tested_source_unchanged':focused['probe_sha256']==sha(probe) and focused['source_unchanged_during_test'],
 'motion_schedule_unchanged':c['phase_schedule']==old['phase_schedule'],
 'numeric_gates_unchanged':c['gates']==old['gates'],
 'physical_parameters_unchanged':c['effective_plugin_parameters']==old['effective_plugin_parameters'],
 'exact_runner_preview_binds_current_runner':preview['runner_sha256']==sha(runner),
 'preview_contract_is_final_contract':sha(D/'actuation_contract.yaml')==sha(S/'contracts/m12_physics_p02.yaml'),
 'preview_source_contract_SHA_bound':params['/actuation_command_source/contract_sha256']==sha(S/'contracts/m12_physics_p02.yaml'),
 'preview_seed_run_id_shared_exact':all(params[prefix+'/seed']==c['seed'] and params[prefix+'/run_id']==c['run_id'] for prefix in ['/actuation_command_source','/fixture_probe']),
 'preview_recorder_exact26_topics':len(topics)==len(set(topics))==26 and topics==bag_topics and set(c['required_topics'])<=set(topics),
 'preview_expanded_URDF_and_world_pass':bool(uv) and bool(wv),
 'preview_required_nodes_source_and_probe':all(launch.find("node[@name='"+name+"']").get('required')=='true' for name in ['actuation_command_source','fixture_probe']),
 'probe_has_no_ROS_publisher':not any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute) and n.func.attr=='Publisher' for n in ast.walk(runtime_ast)),
 'probe_external_shutdown_rejected':"external ROS shutdown before successful terminal assessment" in probe.read_text(),
 'runner_terminal_and_assessment_exceptions_fail':all(x in rt for x in ["not report.get('terminal_exception')","not report.get('assessment_exception')","immutable allocated identity exists; do not retry"]),
}
paths=[probe,source,runner,S/'contracts/m12_physics_p02.yaml',P/'src/astra_physical_ackermann_p02_plugin.cpp',P/'src/astra_dense_evidence_plugin.cpp',P/'launch/spawn_m2_physical_p02.launch',P/'urdf/m2_physical_p02.urdf.xacro',P/'urdf/m2_physical_p02_gazebo.xacro',W/'scripts/astra_gazebo/run_smoke_a02.py',
 W/'scripts/astra_gazebo/tests/test_command_source_p02.py',W/'scripts/astra_gazebo/tests/test_command_source_p02_independent.py',W/'scripts/astra_gazebo/tests/test_physical_actuation_probe_p02.py',
 A/'m12_p02_probe_independent_review.json',A/'m12_p02_probe_initial_invocation.json',A/'m12_p02_probe_independent_review.prior1.json',A/'m12_p02_warmup_focused_review.json',A/'m12_p02_command_source_independent_runtime.json',S/'logs/build_physical_p02_01.log']
paths.extend(p for p in D.iterdir() if p.is_file())
report={'status':'independent_P02_measurement_interface_preflight_pass' if all(checks.values()) else 'fail','checks':checks,'check_count':len(checks),'pass_count':sum(checks.values()),
 'final_execution_source':{'probe':sha(probe),'command_source':sha(source),'runner':sha(runner),'contract':sha(S/'contracts/m12_physics_p02.yaml')},
 'test_chain':{'command_source':{'original':28,'independent':12,'combined_command':str(W/'.venv/bin/python')+' -m pytest -q '+str(W/'scripts/astra_gazebo/tests/test_command_source_p02.py')+' '+str(W/'scripts/astra_gazebo/tests/test_command_source_p02_independent.py'),'exit_code':0},'assessor':{'old_176_snapshot_false_pass_cases':13,'repaired_bcbe_full_suite':67,'final_68dd_focused':8,'final_change_scope':'warmup metadata temporal consistency only; other AST equal'}},
 'effective_validation':uv,'world_validation':wv,
 'resources':[{'path':str(p),'sha256':sha(p)} for p in paths],
 'scope':['All added cases are analytic toy/fake rospy tests, not new Gazebo evidence.','No ROS/Gazebo initialized; no historical or live run files modified.','P01 remains failed; P02 repairs observation and schedule instrumentation only. Actual physical steering may still fail the unchanged0.85 gate.','P02 probe never actuates; command source remains sole cmd_vel_navigation publisher.','Early ROS shutdown returns0 from command source but leaves source status running; final probe explicitly rejects shutdown and demands bootstrap/running/complete history.','Command event time is sampled immediately before Python publish; Twist has no source header and driver receipt is not atomic with that timestamp. Relay receipt checks remain separate.','Source publishes all channels before Trigger advertisement and primes zero while paused. Runtime source deadlines still depend on process scheduling; the frozen observed75ms gate remains necessary.'],
 'preserved_history_note':'Initial176 execution is identified by loaded_source_snapshots and initial_invocation; current source_resources hashes in the old report were capture-time resources, not the assessor execution identity. The67case bcbe report and exact script/probe snapshots are retained separately from final focused68dd verification.',
 'writer_script_sha256':sha(__file__)}
(A/'m12_p02_independent_integration_review.json').write_text(json.dumps(report,indent=2,ensure_ascii=False)+'\n')
md=f'''# P02 独立测量与命令链审查

结论：离线接口审查 {sum(checks.values())}/{len(checks)} 项通过。未运行 ROS/Gazebo，W 与合同已冻结。

命令源原 28 项加独立 12 项合计 40 项通过。新增测试调用完整 runtime 并替换 rospy/time/transport：验证 paused bootstrap、所有 publisher 先 advertise、一次 Trigger 的 run_id/seed/SHA、loss 区间零发布、迟发跳 slot 不补发、时钟回退/停滞、提前 ROS shutdown，以及 complete/fault 常驻。100 组随机迟发轨迹未重复发布同 slot。

评估器旧176快照的有效 toy baseline 通过，同时保留13个 false-pass 反例。修补后的bcbe执行完整67项通过，涵盖源包、目标/实际分离、接触、实际slew失败、命令错误/边界/重复slot、完整状态序列、录包连接及fault。最终68dd仅增强warmup的整数/非负/前后顺序约束；机器AST证实其余代码一致，8项focused通过，其中合理start=1.1s、warmup/running=1.0s的最终完整assessor基线通过。没有把旧快照失败归到修补版。

独立用最终probe验证 exact runner offline launch 的 URDF/world，26个录包topic与参数完全对应，source和probe共用新身份、seed与合同SHA。命令collector没有Publisher；外部shutdown会失败，source最后running状态不能误作complete。原P01动作、全部数字门和物理控制参数保持相同。

限制：这些是离线软件资格，不是物理通过结果。P01实际转向峰值失败保留；P02仍须接受同一0.85rad/s门。command_event是Python发布前采样的时刻，Twist自身无header，不能等同driver接收原子时刻；实际relay观察和时限门继续单独判断。提前ROS shutdown时source返回0但不发补零，由probe强制完整状态和终态异常守住失败记账。

最终 probe SHA：`{sha(probe)}`。

最终 command source SHA：`{sha(source)}`。

合同 SHA：`{sha(S/'contracts/m12_physics_p02.yaml')}`；runner SHA：`{sha(runner)}`。

完整来源与检查见 `m12_p02_independent_integration_review.json`；67项报告、8项focused、40项命令源证据分别保存在同目录，包含输入SHA与实际命令。所有失败历史保留。
'''
(A/'m12_p02_independent_integration_review.md').write_text(md)
print(json.dumps({'status':report['status'],'checks':len(checks),'passed':sum(checks.values()),'failed':[k for k,v in checks.items() if not v],'review_sha256':sha(A/'m12_p02_independent_integration_review.json')},indent=2))
