"""Seal read-only r02 review; writes only this independent audit directory."""
import ast
import hashlib
import json
import sys
from pathlib import Path

A = Path(__file__).resolve().parent
S = A.parents[2]
P = S / 'workspace/scripts/astra_gazebo'

def bind(path):
    path = Path(path).resolve()
    data = path.read_bytes()
    return {'path': str(path), 'sha256': hashlib.sha256(data).hexdigest(), 'bytes': len(data)}

def write(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + '\n')

report_path = S / 'audits/navigation_n06_manager_r02/implementation_report.json'
assert bind(report_path)['sha256'] == '1a46daaf235abfd97f79cd529fd17546cefc9d05f41bc9431ec6bf86e41477b3'
report = json.loads(report_path.read_text())
rosters = {}
for version, count in [('r01', 352), ('r02', 733)]:
    manifest_path = S / ('audits/navigation_n06_manager_' + version) / 'source_evidence_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    roster = {r['path']: r for key in ('owned_sources', 'read_only_dependencies', 'author_evidence') for r in manifest[key]}
    assert len(roster) == count
    for item in roster.values():
        assert bind(item['path'])['sha256'] == item['sha256'], item['path']
    rosters[version] = roster

queue = [Path(r['path']) for r in report['sources'] if '/tests/' not in r['path']]
seen = set()
edges = []
while queue:
    path = queue.pop()
    if path in seen:
        continue
    seen.add(path)
    for node in ast.walk(ast.parse(path.read_text())):
        names = [x.name for x in node.names] if isinstance(node, ast.Import) else [node.module] if isinstance(node, ast.ImportFrom) and node.module else []
        for name in names:
            local = P / (name.split('.')[0] + '.py')
            if local.is_file():
                edges.append({'from': str(path), 'to': str(local), 'line': node.lineno})
                queue.append(local)
assert all(str(path) in rosters['r02'] for path in seen)
sys.path.insert(0, str(P))
import run_navigation_n06_manager_r02, navigation_n06_admin_plan_r02
observed = sorted({str(Path(module.__file__).resolve()) for module in sys.modules.values() if getattr(module, '__file__', None) and str(Path(module.__file__).resolve()).startswith(str(S / 'workspace'))})
assert all(path in rosters['r02'] for path in observed)
write(A / 'recursive_import_verification.json', {'static_modules': [bind(p) for p in sorted(seen)], 'edges': edges, 'observed_workspace_modules': observed, 'all_in_frozen_r02_roster': True})

baseline = json.loads((A.parent / 'frozen_input_verification.json').read_text())['global_csv_readonly_baseline']
globals_observed = []
for old in baseline:
    path = Path(old['path'])
    now = bind(path)
    stat = path.stat()
    assert now['sha256'] == old['sha256'] and stat.st_ino == old['inode'] and stat.st_mtime_ns == old['mtime_ns']
    globals_observed.append(dict(now, inode=stat.st_ino, mtime_ns=stat.st_mtime_ns, unchanged_from_r01=True))
write(A / 'global_csv_readonly_observations.json', globals_observed)

focused = json.loads((A / 'focused_result.json').read_text())
assert len(focused['checks']) == 18 and all(x['pass'] for x in focused['checks'])
assert len(focused['remaining_gaps']) == 2
assert 'Ran 56 tests' in (A / 'author_tests_rerun.log').read_text() and (A / 'author_tests_rerun.log').read_text().rstrip().endswith('OK')

findings = [
    {'id': 'N06R02_ADMINISTRATIVE_IDENTITY_OVERRIDE', 'severity': 'blocking_before_administrative_projection',
     'source': bind(P / 'navigation_n06_admin_plan_r02.py'), 'line': 67,
     'mechanism': '未验证的 administrative_metadata 经第71–73行覆盖 run_manifest 的 run_id 和路径；第77–78行也接受未绑定 report/raw 路径。',
     'counterexample': focused['remaining_gaps'][0],
     'impact': '临时反例中 exact ledger 身份仍正常，但新行政行变为 historical_duplicate；原有历史行未被写改，新增身份与 ledger 不一致。未发生真实 CSV 损坏。',
     'repair': '关闭 metadata schema；身份由已验证 schedule 构造且不可覆盖，路径与 SHA 从 canonical run root/已绑定 receipt、assessment 和实际资源派生或严格核对。'},
    {'id': 'N06R02_ASSESSMENT_SOURCE_CLOSURE_OPTIONAL', 'severity': 'blocking_before_terminal_integrity_qualification',
     'source': bind(P / 'navigation_n06_integrity_r02.py'), 'line': 123,
     'mechanism': '成功 assessment 只要求一般绝对路径/hash和若干结果字段；source_bindings 与 sweep_report 均可缺省，单个任意命名 true check 能成立。',
     'counterexample': focused['remaining_gaps'][1],
     'impact': '最小 synthetic assessment 并非真实 Gazebo 结果，但可被当前正式 validator 接受；现有校验未形成所声明的实际 assessment/source/sweep 闭包。',
     'repair': '要求 canonical runs/<run_id>/assessment.json；验证实际 assessor 固定必需来源键及 canonical resource 绑定、sweep 绑定、固定 integrity schema。保留真实负 assessment 和无 assessment 的非空 error 失败回执，不将 schema 完整性等同重放物理结果。'}]

md = '''# N06 manager r02 独立离线复核

结论：**两项回执来源/行政身份闭包缺口仍阻断 release 使用**。原 r01 三项反例的指定修补已通过，不能据此宣称 r02 整体已闭合。

实际执行：56 项作者测试、18 组独立检查。输入在首次提交前、三张表各自替换后、末端 progress 写入后变化均拒绝完成；精确恢复输入后可 roll-forward，完成后的重复调用 bytes/inode/mtime 全不变。错误 stage/error/assessment/type 等七种回执拒绝。实际 converter 认证拒绝旧三字段假 manifest；八份保存的真实 N06 双次转换通过（native/其它 runtime 仍为结构 stand-in，不是执行资格）。

1. `administrative_metadata` 可把首条 projected run_id 改成 `historical_duplicate`，并注入 `/tmp/unbound_receipt.json` 与未绑定 report 路径。测试只写临时 FS；原历史行未改，但追加身份已背离 exact ledger。
2. 任意临时绝对路径的最小 passing assessment，仅带 `synthetic_check=true`，没有 `source_bindings` 与 `sweep_report`，仍通过正式回执 validator。应核对真实 assessor 的 canonical 文件与固定来源结构；这项检查不需要重放物理，也不能冒称完成物理重放。

全部 r01 352 与 r02 733 资源在检查及封存时匹配；递归本地 imports 与实际导入均在 r02 roster。manager/process 相比 r01 仅 dependency import 改名，继承已实际执行的 disposable Python child/pidfd/setsid-grandchild 清理证据，不把未重复的进程测试计作本轮新实验。

真实三 CSV 的 bytes、inode、mtime 相对 r01 观测均未变。没有启动 ROS/Gazebo、转换器或训练；没有建立实际 execution release、登记种子或调用真实行政 apply。临时 synthetic release 文档仅作离线 verifier 输入。

限制：三 CSV 仍是可恢复顺序替换，不具联合原子可见性；依赖受控单写入者，不证明任意恶意并发写入安全。未验证现场物理结果、真实 release/runtime 闭包；N05 外层143原因仍未知。r01 与本次两个负例完整保留，后续须新版本修补并针对这两项复测。
'''
(A / 'review.md').write_text(md)

extra = [report_path, S / 'audits/navigation_n06_manager_r02/source_evidence_manifest.json',
         A.parent / 'r01/review.json', A.parent / 'disposable_process_result.json',
         S / 'audits/navigation_n06_conversion_independent/review.json']
extra += sorted(p for p in A.iterdir() if p.is_file() and p.name not in ('resources.json', 'review.json'))
resources = {r['path']: r for r in rosters['r02'].values()}
for path in extra:
    resource = bind(path)
    resources[resource['path']] = resource
write(A / 'resources.json', {'schema': 'independent_review_resources_v1', 'resources': list(sorted(resources.values(), key=lambda x: x['path']))})
review = {
    'schema': 'astra_navigation_n06_manager_independent_review_v2',
    'status': 'independent_r02_review_blocked_two_receipt_closure_gaps',
    'independent_review': True, 'r02_release_usable': False,
    'author_report': bind(report_path), 'author_manifest': bind(S / 'audits/navigation_n06_manager_r02/source_evidence_manifest.json'),
    'reviewed_sources': [bind(r['path']) for r in report['sources']],
    'resources': bind(A / 'resources.json'),
    'r01_resources_unchanged': 352, 'r02_resources_rehashed_before_and_at_seal': 733,
    'recursive_static_modules': len(seen), 'observed_workspace_modules': len(observed),
    'recursive_import_evidence': bind(A / 'recursive_import_verification.json'),
    'author_tests_rerun': {'passed': 56, 'log': bind(A / 'author_tests_rerun.log')},
    'focused_checks': {'passed_groups': 18, 'result': bind(A / 'focused_result.json'), 'script': bind(A / 'focused_checks.py'), 'log': bind(A / 'focused_checks.log')},
    'r01_three_requested_repairs_pass': True, 'findings': findings,
    'inherited_process_evidence': {'binding': bind(A.parent / 'disposable_process_result.json'), 'rerun_in_r02': False, 'basis': 'manager/process bytes identical after dependency-import renaming'},
    'conversion_independent_review': bind(S / 'audits/navigation_n06_conversion_independent/review.json'),
    'global_csv_readonly_observations': bind(A / 'global_csv_readonly_observations.json'),
    'coverage_limits': ['Synthetic native and other-runtime stand-ins are not an executable release.', 'No live physical replay; no arbitrary concurrent writer guarantee; sequential CSV visibility retained.', 'Missing assessment/source provenance is a structural validator failure, not evidence that historical physical measurements were forged.', 'N05 manager0 versus outer143 cause remains unknown.'],
    'authority': {'ROS_started': False, 'training_started': False, 'actual_release_created': False, 'seed_registered': False, 'actual_global_csv_written': False, 'author_source_changed': False},
    'next': 'Versioned receipt/metadata closure repair; preserve r01 and r02 negative evidence.'}
write(A / 'review.json', review)
print(json.dumps({'review': bind(A / 'review.json'), 'resources': bind(A / 'resources.json'), 'status': review['status']}, indent=2))
