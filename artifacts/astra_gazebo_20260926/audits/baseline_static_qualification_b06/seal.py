import ast,hashlib,json,pathlib,shutil,sys
S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');P=S/'workspace/scripts/astra_gazebo';A=pathlib.Path(__file__).parent;O=S/'audits/baseline_static_qualification_b05'
def bind(p):
 p=pathlib.Path(p).resolve();raw=p.read_bytes();return {'path':str(p),'sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw)}
def write(n,x):(A/n).write_text(json.dumps(x,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
source=P/'baseline_static_qualification_b06.py';test=P/'tests/test_baseline_static_qualification_b06.py'
assert 'Ran 10 tests' in (A/'author_tests.log').read_text() and (A/'author_tests.log').read_text().rstrip().endswith('OK')
refs=json.loads((O/'source_evidence_manifest.json').read_text())['resources'];rows={}
for r in refs:
 now=bind(r['path']);assert now['sha256']==r['sha256'],r['path'];rows[now['path']]=now
assert len(refs)==264
module=ast.parse(source.read_text());functions=[x for x in module.body if isinstance(x,ast.FunctionDef)]
assert len(functions)==1 and functions[0].name=='proposal_schedule'
last=functions[0].body[-1];assert isinstance(last,ast.Return) and isinstance(last.value,ast.Call)
assert isinstance(last.value.func,ast.Attribute) and last.value.func.attr=='proposal_schedule' and last.value.func.value.id=='_b05'
assert [x.id for x in last.value.args]==['scene_ids','historical_rows','checked']
assert all(not isinstance(x,(ast.ImportFrom,ast.ClassDef)) for x in module.body)
(A/'frozen_sources').mkdir(exist_ok=True)
for p in (source,test):shutil.copyfile(p,A/'frozen_sources'/p.name)
(A/'README.md').write_text('''# B06：B05 提案种子类型门的零权限最小修复

新模块 `baseline_static_qualification_b06.py` 只提供一个 `proposal_schedule` 包装函数。先要求 `other_proposed_seeds` 是 list/tuple，再拷贝为 tuple，并要求每个元素 `type(seed) is int`，最后调用未改的 B05 原函数。没有 coercion、过滤、自动换种子、参数/defaults、物理、回执、manager、collector、ROS 或执行实现。

全部十个 B05 数字字符串绕过反例都保留，B06 对它们拒绝。bool、float、int子类、混合类型都拒绝；字符串、bytes、映射和惰性 iterable 不再被隐式解读成种子容器。有效的 list/tuple 参数保留旧行为；不会另加种子范围/正数等语义变化。原 registry CSV 字符串解析策略仍由 B05 负责。

当前真实 B05 frozen scenes + 原registry快照 + N06整数提案输入，返回值逐项和规范JSON均与原 B05 schedule 相同。B06 是修复组件身份，不是另一个可执行实验阶段；输出保留原 B05 的零权限提案身份、20单元设想、allocated_budget=0。既未复制执行授权，也未登记或占用任何种子。

10项作者纯 CPU tests通过，未启动ROS/Gazebo或训练。B05原264资源、原阴性与独立审查保持原字节。严格参数默认值来源、实际九collision/后轴物理、命令回执及manager/collector的缺口仍全部pending。本包需非作者独审，不把本作者报告当独立通过。
''')
write('source_structure_check.json',{'pass':True,'only_new_function':'proposal_schedule','sole_frozen_delegate':'baseline_static_qualification_b05.proposal_schedule(scene_ids,historical_rows,checked)','old_source_unchanged':bind(P/'baseline_static_qualification_b05.py'),'old264_resources_unchanged':True,'new_math_or_ROS_implementation':False})
write('commands.json',{'command':'PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B -m unittest discover -s '+str(P/'tests')+' -p test_baseline_static_qualification_b06.py -v','exit_code':0,'python_version':sys.version,'GPU_or_ROS_initialized':False})
for p in (source,test,O/'component_report.json',O/'source_evidence_manifest.json',S/'audits/baseline_static_qualification_b05_author_followup/report.json',S/'audits/baseline_static_qualification_b05_author_followup/probe_results.json',S/'audits/baseline_static_qualification_b05_independent/review.json'):
 r=bind(p);rows[r['path']]=r
for p in A.rglob('*'):
 if p.is_file() and p.name not in ('component_report.json','source_manifest.json','postwrite_check.json'):
  r=bind(p);rows[r['path']]=r
write('source_manifest.json',{'schema':'astra_b06_zero_authority_component_resources_v1','execution_release':False,'resources':sorted(rows.values(),key=lambda r:r['path'])})
report={'schema':'astra_baseline_static_qualification_b06_author_report_v1','status':'minimal_exact_int_seed_gate_repair_author_tests_pass_independent_review_pending','author_report':True,'independent_review':False,'execution_authorized':False,'execution_ready':False,'allocated_budget':0,'seed_registered':False,'release_created':False,'ROS_or_Gazebo_started':False,'training_started':False,'source':bind(source),'tests':bind(test),'source_manifest':bind(A/'source_manifest.json'),'README':bind(A/'README.md'),'B05_parent':bind(O/'component_report.json'),'B05_independent_negative':bind(S/'audits/baseline_static_qualification_b05_independent/review.json'),
 'single_factor':'strict list/tuple exact-int other_proposed_seeds gate before frozen delegate; no coercion/filtering','unchanged':'valid accepted list/tuple input returns the exact old unallocated B05 proposal; all B05 sources/design/reports and264resources unchanged','API':'proposal_schedule(scene_ids,historical_rows,other_proposed_seeds=()) -> original B05 zero-authority schedule; invalid container/element or conflict raises ValueError','namespace_scope':'B06 module/repair identity, not a B06 execution stage; returned B05 draft identities are not allocated or reused runtime identities','tests_passed':10,'negative_regression_scope':'all10 historical string/int alias bypasses, bool/float/subclass/mixed/container cases, integer and CSV collisions, immutable inputs and actual old-schedule parity','test_log':bind(A/'author_tests.log'),'source_structure':bind(A/'source_structure_check.json'),'commands':bind(A/'commands.json'),'independent_review_pending':True,
 'unresolved_inherited':['complete prepared initialized runtime defaults and actual full-parameter closure','nine actual collision component rear-yaw containment and physical qualification','complete raw command/driver ingress/actuator receipt association','new B05 manager/collector/runtime/native/source closure','dynamic actors remain unsupported'],
 'no_promotion':'Pure typed-seed fix only; no physical/runtime/defaults/performance qualification or execution authority.'}
write('component_report.json',report)
for r in rows.values():assert bind(r['path'])['sha256']==r['sha256']
write('postwrite_check.json',{'status':'pass','resource_count':len(rows),'report':bind(A/'component_report.json'),'manifest':bind(A/'source_manifest.json')})
print(json.dumps({'report':bind(A/'component_report.json'),'source':bind(source),'manifest':bind(A/'source_manifest.json'),'resource_count':len(rows)},indent=2))
