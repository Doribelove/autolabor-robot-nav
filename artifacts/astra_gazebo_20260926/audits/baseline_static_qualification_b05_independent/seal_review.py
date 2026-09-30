"""Seal B05 independent component review without changing author inputs."""
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
S = A.parents[1]
P = S / 'workspace/scripts/astra_gazebo'

def bind(path):
    path = Path(path).resolve()
    data = path.read_bytes()
    return {'path': str(path), 'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}

def write(name, value):
    (A / name).write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + '\n')

author = S / 'audits/baseline_static_qualification_b05/component_report.json'
follow = S / 'audits/baseline_static_qualification_b05_author_followup/report.json'
assert bind(author)['sha256'] == '55f09a5694f4683823a34e4d104db1deae8047f2a66735be6d5ccc343b8f60b9'
assert bind(follow)['sha256'] == '8ced613d81d6a5f3dd9d24bf427bef1ed3ebfde96d08d20ec5d220814fee0c40'
ar = json.loads(author.read_text())
manifest = json.loads(Path(ar['source_manifest']['path']).read_text())
assert len(manifest['resources']) == 264
for r in manifest['resources']:
    assert bind(r['path'])['sha256'] == r['sha256'], r['path']
focused = json.loads((A / 'focused_results.json').read_text())
assert len(focused['check_groups']) == 10 and all(r['pass'] for r in focused['check_groups'])
assert (A / 'author_tests_rerun.log').read_text().rstrip().endswith('OK')
assert 'Ran 20 tests' in (A / 'author_tests_rerun.log').read_text()

observations = []
baseline_path = S / 'audits/navigation_n06_manager_independent/r02/global_csv_readonly_observations.json'
for old in json.loads(baseline_path.read_text()):
    path = Path(old['path']); stat = path.stat(); current = bind(path)
    assert current['sha256'] == old['sha256'] and stat.st_ino == old['inode'] and stat.st_mtime_ns == old['mtime_ns']
    observations.append({'original_path': str(path), 'observed_sha256': current['sha256'],
                         'inode': stat.st_ino, 'mtime_ns': stat.st_mtime_ns, 'unchanged': True})
write('global_readonly_observations.json', observations)
write('commands.json', {
    'independent_checks': 'PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 /usr/bin/python3 -B ' + str(A / 'checks.py'),
    'author_tests_rerun': 'source ' + str(S / 'workspace/scripts/astra_env.sh') + '; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 ' + str(S / 'workspace/.venv_train_cuda/bin/python') + ' -B -m unittest discover -s ' + str(P / 'tests') + ' -p test_baseline_static_qualification_b05.py -v',
    'check_exit_code': 0, 'author_test_exit_code': 0,
    'initial_discovery_note': 'Read-only ls of the new independent directory returned ENOENT before creation; no test or source failed and no author byte was changed.',
    'ROS_nodes_started': False, 'GPU_work_started': False})

md = '''# B05 独立离线设计与纯组件复核

结论：**设计范围与参数比较/失败包括性算术有限通过；通用种子冲突 API 有确认缺口，未合格。** B05 保持执行权限、已分配预算、登记种子均为零。未来使用通用排程入口前须新版本修复类型门，不能修改本次冻结字节后仍沿用旧报告。

实际复验了全部 264 资源、20 项作者测试和 10 组独立检查。两份实际 B04 离线展开参数树的每个标量分别单独修改：TEB 417 次、DWA 405 次均被拒绝；覆盖超出原 B04 11 字段 overlay 的值。精确类型、signed zero、非有限值、路径转义和三个必需阶段均符合比较器声明。

确认反例：对全部十个提案 seed，`other_proposed_seeds=[str(seed)]` 都被接受，结果与无冲突输入相同；同值整数及历史 CSV 数字字符串都被拒绝。原因是源码第98行仅对 historical_rows 的 seed 做 int 转换，第100行直接与未验证的 other_proposed_seeds 集合求交。建议在新版本对该参数容器和所有元素做明确类型验证（推荐 exact int，拒绝 bool/string/float），或用显式、经过审核的规范化接口。不能继续声称当前通用 typed 冲突门已闭合。

实际冻结输入不受此反例影响：已绑定 N06 prepared 的 seed 都是整数，和 B05 十个 seed 不相交；历史 registry 快照与原输入重算出的 20 行 proposal 完全相同。该检查不预约号码，未来登记前仍须重新验所有历史状态/其它提案。

失败包括性检查用独立手算复核混合成功、timeout、capture 失败、geometry 失败和未启动退休：所有尝试失败保留22秒；退休仅进入明确标注的行政 cap 表，不进入尝试均值；十对完整性与 DWA−TEB 算术一致。缺行、重复、伪造退休测量、private_success 和错误类型被拒绝。这个函数只消费调用者标签，不认证真实 goal、geometry 或 stop policy；合成零秒全成功仍可算术通过且明确 qualification_determined=false。不能拿标签本身当运行证据。

四项实现边界仍阻断实际运行资格：

- 完整已初始化默认参数树尚未生成。比较器会接受两个相同的部分树，并明确不证明默认值完整性；实际 B04 launch dump 也不等于 initialized runtime namespace。
- 九个实际 collision shape 的逐源 rear-yaw 原始/外扩 footprint 包含检查未实现；此前 tilt 0.000406165m 差异保留，不能新增容差掩盖。
- 真实 command→relay→driver ingress 和 rear partial-batch 回执闭包未实现。driver receipt 表示接收/入队，不等于力生效；headerless Twist、重复值与末端无 trace zero 必须由新采集器明确处理。
- 新 manager/collector、stage stop-policy 实施、完整 native/converter/runtime 闭包以及实际静态物理运行均不存在；动态 actor 不在本设计范围。公共 base goal 仍是唯一终态权限，private rear SUCCESS 不能替代。

审查独立性：本审查者未编写 B05 源码、设计或作者测试；之前仅给过三条边界提醒。审查者曾编写 B04 上游集成包，本报告没有把该上游作者工作重新宣称为独审，而是继承 environment_audit 的已冻结 B04 独审并仅将其实际参数输出作为新比较器测试输入。

全程无 ROS/Gazebo、训练、真实 release 或种子登记；真实三 CSV 的 bytes/inode/mtime 未变。冻结作者文件不改，全部反例保留。
'''
(A / 'review.md').write_text(md)
extra = [author, follow, Path(ar['source_manifest']['path']), baseline_path,
         S / 'audits/baseline_static_qualification_b05_author_followup/probe_results.json',
         S / 'contracts/navigation_n06_prepared_r01/prepared.json',
         S / 'audits/baseline_integration_b04_independent/review.json']
extra += [p for p in A.iterdir() if p.is_file() and p.name not in ('resources.json', 'review.json')]
resources = {r['path']: r for r in manifest['resources']}
for path in extra:
    r = bind(path); resources[r['path']] = r
write('resources.json', {'schema': 'independent_b05_review_resources_v1', 'resources': sorted(resources.values(), key=lambda r:r['path'])})
report = {
    'schema': 'astra_b05_independent_offline_review_v1',
    'status': 'independent_offline_design_and_arithmetic_limited_pass_seed_type_gap_retained',
    'independent_review': True, 'author_report': bind(author), 'author_followup': bind(follow),
    'reviewed_source': bind(P / 'baseline_static_qualification_b05.py'), 'reviewed_design': ar['design'],
    'author_manifest': ar['source_manifest'], 'original_resources_rehashed_before_after_and_seal': 264,
    'source_or_historical_evidence_changed': False,
    'independence': {'B05_author': False, 'prior_input': 'three boundary requirements only', 'upstream_B04_author': True,
                     'upstream_review_inherited_not_self_reviewed': bind(S / 'audits/baseline_integration_b04_independent/review.json')},
    'author_tests_rerun': {'passed': 20, 'log': bind(A / 'author_tests_rerun.log')},
    'independent_checks': {'groups': 10, 'leaf_mutations_TEB_DWA': [417,405], 'confirmed_seed_string_cases': 10,
                           'result': bind(A / 'focused_results.json'), 'script': bind(A / 'checks.py'), 'log': bind(A / 'focused_checks.log')},
    'findings': [{'id': 'B05_GENERIC_PROPOSAL_SEED_TYPE_BYPASS', 'severity': 'new_revision_required_before_generic_caller_use',
                  'source_line': 98, 'counterexample': focused['confirmed_negative'],
                  'repair': 'Versioned exact-type check of other_proposed_seeds container/elements, preferably exact int; preserve r01 and actual frozen schedule.',
                  'current_frozen_schedule_invalidated': False}],
    'qualified_scope': ['Finite type-sensitive equality of supplied complete-or-partial parameter trees.', 'Exact current saved twenty-row zero-authority schedule reproduction with integer N06 inputs.', 'Descriptive failure-inclusive arithmetic retaining unstarted retirement separately.'],
    'generic_seed_conflict_API_qualified': False,
    'pending_not_qualified': ['Complete phase-specific initialized default namespace preparation.', 'Source-matched containment of actual nine collision components including tilt.', 'Physical source command/relay/ingress/rear partial-batch receipt joins and shutdown tail.', 'Manager/collector stage stop-policy, native/converter/runtime release closure.', 'Actual static physical qualification, dynamic actor compatibility, tuning or superiority.'],
    'numeric_thresholds_changed': False,
    'authority': {'execution_authorized': False, 'execution_ready': False, 'allocated_budget': 0,
                  'release_created': False, 'seed_registered': False, 'ROS_Gazebo_started': False,
                  'training_started': False, 'global_CSV_written': False},
    'global_readonly_observations': bind(A / 'global_readonly_observations.json'),
    'resources': bind(A / 'resources.json'), 'resource_count': len(resources), 'narrative': bind(A / 'review.md')}
write('review.json', report)
print(json.dumps({'review': bind(A / 'review.json'), 'resources': bind(A / 'resources.json'), 'status': report['status']}, indent=2))
