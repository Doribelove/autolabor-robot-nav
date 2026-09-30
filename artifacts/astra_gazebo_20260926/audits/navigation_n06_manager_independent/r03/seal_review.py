"""Seal r03 focused independent negative without changing reviewed code."""
import hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[2];P=S/'workspace/scripts/astra_gazebo'
def bind(path):
 path=Path(path).resolve();data=path.read_bytes();return {'path':str(path),'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
def write(name,value):(A/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
author=S/'audits/navigation_n06_manager_r03/implementation_report.json';manifest=author.parent/'source_evidence_manifest.json'
assert bind(author)['sha256']=='6245fb04b8291c9bb8962c8f503d30e8736cac862a03bfdff65c581e2f328e92'
assert bind(manifest)['sha256']=='cb48b726ab31680eb2ee61c2721f5e2ccfa169c49f33ed16ac82409096b3fa87'
rosters={}
for revision,count in [('r01',352),('r02',733),('r03',764)]:
 m=json.loads((S/('audits/navigation_n06_manager_'+revision)/'source_evidence_manifest.json').read_text());rows={r['path']:r for key in ('owned_sources','read_only_dependencies','author_evidence') for r in m[key]}
 assert len(rows)==count and all(bind(r['path'])['sha256']==r['sha256'] for r in rows.values());rosters[revision]=rows
result=json.loads((A/'focused_result.json').read_text());counter=json.loads((A/'late_assessment_counterexample.json').read_text())
assert all(r['pass_'] for r in result['checks']) and all(r['rejected'] for r in result['rejected_mutations'])
log=(A/'author_tests_rerun.log').read_text();assert 'Ran 62 tests' in log and log.rstrip().endswith('OK')
observations=[]
for row in json.loads((A.parent/'r02/global_csv_readonly_observations.json').read_text()):
 path=Path(row['path']);stat=path.stat();actual=bind(path)
 assert actual['sha256']==row['sha256'] and stat.st_ino==row['inode'] and stat.st_mtime_ns==row['mtime_ns']
 observations.append(dict(actual,inode=stat.st_ino,mtime_ns=stat.st_mtime_ns,unchanged=True))
write('global_readonly_observations.json',observations)
write('commands.json',{'author_tests':'source workspace/scripts/astra_env.sh; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 python -m unittest discover -s workspace/scripts/astra_gazebo/tests -p test_navigation_n06_manager_r03.py -v',
 'independent_focused':'source workspace/scripts/astra_env.sh; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 python '+str(A/'focused_checks.py'),
 'counterexample':'source workspace/scripts/astra_env.sh; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 python '+str(A/'late_assessment_counterexample.py'),
 'exit_codes':{'author_tests':0,'focused':0,'counterexample_capture':0},'all_FS_mutations_in_temp_dirs':True})
(A/'review.md').write_text('''# N06 manager r03 独立离线复核

结论：**原 r02 两项修补通过，但发现一个终态故障回收阻断，r03 不可用于 release。** 原 r01/r02 负结果不改。

实际运行62项作者测试和本报告绑定的 focused 检查：metadata 闭15字段，run_id 覆盖及错误 canonical 路径/hash/process 被拒绝；任意路径最小 assessment、缺9checks/6sources/11sweep来源被拒绝。实际 navigation_evidence_n06.assess AST 的9checks和6sources、dense_sweep_n05.main 的11来源与新校验一致（N06 不启用 R2 条件分支）。冻结 world/URDF 即使重算下游报告 hash 仍不能替换；collector/closure身份和结果不一致被拒绝。负 assessment、事后 error 及真正无报告的启动失败均保持阴性；新增process依赖在首CSV替换后变化会阻止后续表/完成，恢复字节可续提交且完成后重复调用不写文件。全部临时报告、bag字节、PID都是明确标注的结构夹具，不是仿真证据。

新反例使用实际 reconcile_worker_exit：先持久 consume 一条；构造与当前 validator 一致的完整 assessment 文件，模拟 assessor 已原子写出报告后 worker 在报告返回/receipt发布前退出。supervisor 调 terminal_receipt(report=None)，该函数发现已有 assessment 并绑定它，但 task_outcome 仍为 None。close_unit 的 validator 因报告 task 与 receipt 不一致抛错。实测 ledger 仍 active、首消费行 receipt 未绑定、退休数0。assessment 字节保留，失败 receipt 文件也留下，但预算终态闭包未完成。

这是退出窗口处理缺口，不要求物理重放，也不允许借读回报告把基础设施失败晋升为成功。最小新版本应从已保留、经完整验证的报告派生 task，保留 interrupted worker 的整体 integrity=false 和原错误；实际 supervisor 回收须完成1 consumed/7 retired并不重跑。r04 由作者另建，本报告不审其中间字节。

全部 r01 352、r02 733、r03 764 资源在检查与封存时重哈希匹配。manager/process 除依赖名外与 r02 字节相同；继承 r01 的实际 disposable-child 清理测试与 r02 的转换认证验证，不冒称本轮重新执行。真正三CSV bytes/inode/mtime未变。无ROS/Gazebo、训练、实际release、seed登记或真实行政apply。

限定：本报告审结构一致性与故障闭合，不认证 synthetic report 内的物理值。最终N06 source/lifecycle/runtime闭包仍需单独新身份审查；三CSV更新仍为可恢复顺序替换，不具联合原子可见性，N05外层143原因仍未知。
''')
extra=[author,manifest,A.parent/'r02/review.json',A.parent/'r01/review.json',A.parent/'disposable_process_result.json',S/'audits/navigation_n06_conversion_independent/review.json']
extra += [p for p in A.iterdir() if p.is_file() and p.name not in ('resources.json','review.json')]
resources=dict(rosters['r03'])
for path in extra:r=bind(path);resources[r['path']]=r
write('resources.json',{'schema':'independent_r03_resources_v1','resources':sorted(resources.values(),key=lambda r:r['path'])})
report={'schema':'astra_navigation_n06_manager_independent_review_v3','status':'independent_r03_review_blocked_late_assessment_reconciliation',
 'independent_review':True,'r03_release_usable':False,'author_report':bind(author),'author_manifest':bind(manifest),
 'reviewed_sources':[bind(r['path']) for r in json.loads(author.read_text())['sources']],
 'old_rosters_unchanged':{'r01':352,'r02':733},'r03_resources_rehashed_before_and_at_seal':764,
 'r02_two_requested_repairs_pass':True,
 'author_tests_rerun':{'passed':62,'log':bind(A/'author_tests_rerun.log')},
 'independent_focused':{'passed_groups':len(result['checks']),'rejected_mutations':len(result['rejected_mutations']),'result':bind(A/'focused_result.json'),'script':bind(A/'focused_checks.py')},
 'findings':[{'id':'N06R03_RETAINED_ASSESSMENT_WORKER_EXIT_NOT_CLOSED','severity':'blocking_before_one_shot_execution_release',
  'source':bind(P/'run_navigation_n06_manager_r03.py'),'source_lines':[70,78,184,187],
  'counterexample':bind(A/'late_assessment_counterexample.json'),'script':bind(A/'late_assessment_counterexample.py'),
  'observed':counter,'repair':'New version: read retained canonical assessment to derive task/identity when recovering an interrupted worker, keep overall integrity false and original error; validate sources then bind consumed receipt and retire unrun tail.'}],
 'inherited_not_rerun':['r01 disposable PID/starttick/setsid child cleanup; manager/process body equivalence retained.', 'r02 eight actual saved conversion diagnostic authentication; unchanged verifier conversion block.'],
 'coverage_limits':['Synthetic assessment consistency fixtures are not Gazebo outcomes.', 'No actual execution release/runtime closure or physical terminal replay.', 'Sequential CSV visibility and sole-writer assumptions remain; N05 outer143 cause unknown.'],
 'global_readonly_observations':bind(A/'global_readonly_observations.json'),'resources':bind(A/'resources.json'),'narrative':bind(A/'review.md'),
 'authority':{'execution_release_created':False,'seed_registered':False,'ROS_started':False,'training_started':False,'real_CSV_written':False,'author_source_modified':False},
 'next':'Versioned r04 late-assessment recovery regression; then separate final source/lifecycle/runtime closure.'}
write('review.json',report);print(json.dumps({'report':bind(A/'review.json'),'resources':bind(A/'resources.json'),'status':report['status']},indent=2))
