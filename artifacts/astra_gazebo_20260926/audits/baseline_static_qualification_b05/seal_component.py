import hashlib,json,pathlib,shutil
S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');W=S/'workspace';P=W/'scripts/astra_gazebo';G=W/'src/simulation/m2_gazebo';A=pathlib.Path(__file__).parent

def bind(p):
 p=pathlib.Path(p).resolve();raw=p.read_bytes();return {'path':str(p),'sha256':hashlib.sha256(raw).hexdigest(),'bytes':len(raw)}
def write(name,x):(A/name).write_text(json.dumps(x,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
def collect(x):
 if isinstance(x,dict):
  if isinstance(x.get('path'),str) and isinstance(x.get('sha256'),str):yield x
  for v in x.values():yield from collect(v)
 elif isinstance(x,list):
  for v in x:yield from collect(v)
source=P/'baseline_static_qualification_b05.py';test=P/'tests/test_baseline_static_qualification_b05.py';design=S/'contracts/baseline_static_qualification_b05_draft.json'
x=json.loads(design.read_text());assert x['allocated_budget']==0 and x['execution_authorized'] is False
assert 'Ran 20 tests' in (A/'author_tests_r02.log').read_text() and (A/'author_tests_r02.log').read_text().rstrip().endswith('OK')
(A/'frozen_sources').mkdir(exist_ok=True)
for p in (source,test):shutil.copyfile(p,A/'frozen_sources'/p.name)
readme='''# B05 静态后轴资格：零权限设计与纯组件

B05 是后继候选设计，保留 B04 所有字节。没有执行 release、seed 登记、ROS/Gazebo、训练或新仿真证据。

拟定 clear + 冻结 D04 四个静态 parent，共 5 个几何，各 2 个新模拟种子，TEB/DWA 成对交叉顺序，共 10 块 / 20 回合。263004001–263004010 仅提案，与当前 registry 快照及 N06 提案无冲突；不构成预约，后续必须重新检查。原图/地图/路径绑定保留，重复种子和 N06 共用 parent 不冒充新独立几何。历史任何状态的种子/身份/预算都不能转移；attempt=1，无补跑、替换或续跑。

唯一新代码是无 ROS 的纯校验/记账组件。`compare_full_parameters(prepared, observed)` 对 /move_base 整棵参数树检查精确类型、有限值、列表顺序和所有键；int/bool/double 不互换，浮点不加容差。`compare_parameter_phases` 要求 paused_bootstrap、positive_clock_ready、paused_terminal 全部出现。B04 原窄 gate 能接受的 max_vel_x=17 在这里失败。

期望参数的完整性不是该函数自行证明的：真正 prepare 还须根据冻结 cfg/源码枚举插件/动态重配置/代价图默认值，按阶段生成完整期望并绑定。不能用首次 live 实际值作为期望，也不能忽略意外 auto-default。当前只有 B04 实际离线展开值的比较例子，完整 initialized runtime namespace 尚未实现，仍阻断 release。

公共任务保持原始 actual base_link gate、0.25 m / 0.25 rad goal、22 s cap；私有 rear action 的 SUCCESS 无终态权限。保留 full3D offset/角速度叉乘/真实 vy/协方差与 source/available 区分，原后轴 adapter、TEB/DWA 配置、P05/Kd6 不改。对所有真实九 collision shape、rear-yaw 原始16点/外扩64点 hull 的逐源几何包含和真实 command→relay→driver ingress 顺序/数量/字段闭包列为必须新增的实现门，不能拿源码要求代替实际通过。Ingress receipt 是司机接收/入队时刻，不是力生效；无 header Twist 不凭 bag time 重造 source stamp，重复值按事件计数，末尾未带 trace 的 zero 不能偷偷丢弃。

保留 B03 partial-batch 先输出但无回执的真实负例；B05 同样按缺证失败，不能补回执。真实回执、TF/goal 传输、倾斜 yaw-only footprint、停转/刹停/低速执行均须 fresh 物理资格。动态 actor 不在本设计内。

`proposal_schedule` 只返回 allocated_budget=0 的固定配对提案。`failure_inclusive_summary` 保留20个已闭合身份：实际 valid goal 用源时间，所有已尝试失败用22 s；未执行退休行保持无测量、另标行政22 s上界，不能当作真实失败观测或完整配对。该函数不判资格、不授予性能结论。初始4个 clear 单元未全部有效完成即闭合该资格阶段；后继静态中的算法 timeout 可继续预定顺序，但物理/安全/完整性问题中止并退休余额，不换种子。

20 项作者小 CPU 测试通过。首轮夹具调用 Python3.8 没有的 math.nextafter 失败已保留，仅测试改成明确相邻 IEEE double，生产组件未因此更改。测试包含真实 B04 全参数树和窄 gate 反例、所有类型/缺项/多项/footprint 差异、历史冲突、重复/遗漏失败、真假退休与配对计数。独立审查仍 pending；没有真实 B05 运行或图像，不能声称 rear-base 物理、动态 actor、实时回执、强 baseline 性能、一般安全或实车资格。
'''
(A/'README.md').write_text(readme)
write('commands.json',{'author_tests':'source '+str(W/'scripts/astra_env.sh')+'; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 '+str(W/'.venv_train_cuda/bin/python')+' -B -m unittest discover -s '+str(P/'tests')+' -p test_baseline_static_qualification_b05.py -v','prepare':'PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B '+str(A/'prepare_design.py'),'test_r01_exit_code':1,'test_r02_exit_code':0,'prepare_exit_code':0,'no_GPU_or_ROS_initialization':True})
rows={}
def add(r):
 now=bind(r['path']);assert now['sha256']==r['sha256'],r['path'];rows[now['path']]=now
prior=S/'audits/baseline_integration_b04_independent'
for r in json.loads((prior/'resources.json').read_text())['resources']:add(r)
for r in collect(x):add(r)
for p in (source,test,design,prior/'review.json',prior/'resources.json',S/'audits/baseline_integration_b04/source_manifest.json'):
 r=bind(p);rows[r['path']]=r
for scene in x['scenes']:
 folder=pathlib.Path(scene['common_bundle']['path']).parent
 bundle=json.loads(pathlib.Path(scene['common_bundle']['path']).read_text())
 for name,key in [('map.pgm','map_image_sha256'),('map.yaml','map_yaml_sha256')]:
  r=bind(folder/name);assert r['sha256']==bundle[key];rows[r['path']]=r
for p in A.rglob('*'):
 if p.is_file() and p.name not in ('component_report.json','source_evidence_manifest.json','postwrite_hash_check.json'):
  r=bind(p);rows[r['path']]=r
write('source_evidence_manifest.json',{'schema':'astra_b05_offline_source_design_manifest_v1','execution_release':False,'resources':sorted(rows.values(),key=lambda r:r['path'])})
report={'schema':'astra_baseline_static_qualification_b05_author_report_v1','status':'zero_execution_authority_design_and_pure_component_pass_independent_review_pending','author_report':True,'independent_review':False,'execution_authorized':False,'execution_ready':False,'allocated_budget':0,'release_created':False,'seed_registered':False,'ROS_or_Gazebo_started':False,'training_started':False,'physical_qualification':False,'dynamic_actor_supported':False,'formal_result':False,
 'design':bind(design),'source':bind(source),'tests':bind(test),'source_manifest':bind(A/'source_evidence_manifest.json'),'README':bind(A/'README.md'),
 'proposal':{'geometries':5,'independent_new_geometries_created_by_B05':0,'declared_parent_scope':'one inherited clear + four D04 r05 static parents, shared with N06','paired_blocks':10,'simulator_repeats_per_geometry':2,'methods':x['methods'],'max_episodes':20,'seeds':list(range(263004001,263004011)),'budget_allocated':0,'attempt':1,'schedule':bind(A/'proposed_schedule.json'),'freshness':bind(A/'freshness_check.json')},
 'implemented':['type-sensitive finite complete parameter namespace comparison','all three phase keys mandatory, no learned/ignored default keys','fixed balanced20-row proposal with historical/proposed identity conflict rejection','complete failure-inclusive paired accounting separating observed attempts and administrative unstarted retirement'],
 'tests_summary':{'final_passed':20,'duration_s':.137,'log':bind(A/'author_tests_r02.log'),'preserved_initial_failure':bind(A/'author_tests_r01.log'),'initial_failure_cause':'Python3.8 test-only math.nextafter unavailable; replaced by explicit adjacent IEEE float constant, old fixture saved','test_interpreter':str(W/'.venv_train_cuda/bin/python'),'test_environment_prior_readonly_record':bind(S/'audits/baseline_integration_b04_independent/test_environment.json'),'real_inputs':'two actual B04 offline launch parameter expansions; neither is an actual initialized B05 live namespace','synthetic_results_not_simulation':True},
 'api':{'compare_full_parameters':'prepared and observed complete /move_base contents -> pass, typed digests, every mismatch path; invalid types/NaN raise','compare_parameter_phases':'exact PHASES mapping -> separate comparisons and joint pass','proposal_schedule':'five distinct scene IDs, immutable registry rows and other proposal seed list -> zero-authority20-row counterbalanced proposal','failure_inclusive_summary':'exact fixed20 schedule and20 closed terminal/retired records -> explicit attempted/administrative means and10 pair completeness; no qualification decision'},
 'key_boundaries':['Full expected initialized parameter/default materialization remains pending. Current comparator only checks supplied trees and cannot prove their provenance.','Full runtime baseline/geometry/command joins need a new manager/collector and exact source/native/converter/interpreter closure; no B04 file changed.','Actual rear-yaw raw/padded footprint containment must be checked against all nine physical shapes; prior tilt mismatch is preserved, not silently covered by tolerance.','Public actual-base terminal is sole authority; private narrower tolerances do not guarantee callback arrival order.','Partial-batch missing receipt, raw driver receipt versus actuator application, headerless Twist and final untraced zero need explicit measured closure.','No dynamic actor compatibility, tuning, general safety, performance winner or hardware qualification.'],
 'next_prerequisites':x['pending_before_any_execution'],'history_preserved':True,'historical_resource_rehash_count':len(json.loads((prior/'resources.json').read_text())['resources']),
 'notes':'Draft can be independently reviewed now. It deliberately leaves physical/transport/default preparation pending instead of claiming readiness from pure tests.'}
write('component_report.json',report)
for r in rows.values():assert bind(r['path'])['sha256']==r['sha256']
write('postwrite_hash_check.json',{'status':'pass','resource_count':len(rows),'report':bind(A/'component_report.json'),'manifest':bind(A/'source_evidence_manifest.json'),'design':bind(design)})
print(json.dumps({'report':bind(A/'component_report.json'),'manifest':bind(A/'source_evidence_manifest.json'),'design':bind(design),'resource_count':len(rows)},indent=2))
