"""Seal final bounded r04 manager review; no execution authority."""
import ast,hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[2];P=S/'workspace/scripts/astra_gazebo'
def bind(path):
 path=Path(path).resolve();data=path.read_bytes();return {'path':str(path),'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
def write(name,value):(A/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
author=S/'audits/navigation_n06_manager_r04/implementation_report.json';manifest=author.parent/'source_evidence_manifest.json'
assert bind(author)['sha256']=='47b04f9b6192638fae33440e8950ff0e3b0e821c7b9e49ef46cac96b032900c9'
assert bind(manifest)['sha256']=='47ac2100bdc1d454861fd9ca8aefecc25469626e1c62ee8eda03015735c1d2a6'
rosters={}
for revision,count in [('r01',352),('r02',733),('r03',764),('r04',791)]:
 m=json.loads((S/('audits/navigation_n06_manager_'+revision)/'source_evidence_manifest.json').read_text());rows={r['path']:r for key in ('owned_sources','read_only_dependencies','author_evidence') for r in m[key]}
 assert len(rows)==count and all(bind(r['path'])['sha256']==r['sha256'] for r in rows.values());rosters[revision]=rows
ar=json.loads(author.read_text());result=json.loads((A/'focused_result.json').read_text())
assert len(result['checks'])==12 and all(x['pass_'] for x in result['checks']) and len(result['late_assessment_scenarios'])==8
log=(A/'author_tests_rerun.log').read_text();assert 'Ran 63 tests' in log and log.rstrip().endswith('OK')
pending=[Path(r['path']) for r in ar['sources'] if '/tests/' not in r['path']];seen=set();edges=[]
while pending:
 p=pending.pop()
 if p in seen:continue
 seen.add(p)
 for node in ast.walk(ast.parse(p.read_text())):
  names=[r.name for r in node.names] if isinstance(node,ast.Import) else [node.module] if isinstance(node,ast.ImportFrom) and node.module else []
  for name in names:
   local=P/(name.split('.')[0]+'.py')
   if local.exists():edges.append({'from':str(p),'to':str(local),'line':node.lineno});pending.append(local)
assert all(str(p) in rosters['r04'] for p in seen)
write('recursive_static_imports.json',{'modules':[bind(p) for p in sorted(seen)],'edges':edges,'all_in_r04_roster':True,'not_a_live_runtime_dependency_closure':True})
obs=[]
for row in json.loads((A.parent/'r03/global_readonly_observations.json').read_text()):
 p=Path(row['path']);st=p.stat();actual=bind(p);assert actual['sha256']==row['sha256'] and st.st_ino==row['inode'] and st.st_mtime_ns==row['mtime_ns']
 obs.append(dict(actual,inode=st.st_ino,mtime_ns=st.st_mtime_ns,unchanged=True))
write('global_readonly_observations.json',obs)
write('commands.json',{'author_tests':'source workspace/scripts/astra_env.sh; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 python -m unittest discover -s workspace/scripts/astra_gazebo/tests -p test_navigation_n06_manager_r04.py -v',
 'focused':'source workspace/scripts/astra_env.sh; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 python '+str(A/'focused_checks.py'),
 'exit_codes':{'author':0,'focused_attempt01':1,'focused_attempt02':1,'focused_attempt03':0},
 'retained_reviewer_harness_errors':[{'attempt':1,'cause':'unused AST helper referenced Python3.8-unavailable ast.unparse; removed helper line, no product change','source':bind(A/'focused_checks_attempt01.py'),'log':bind(A/'focused_checks_attempt01.log')},
 {'attempt':2,'cause':'empty temp-stage fixture omitted existing real stage runs/ parent; created parent in setup; no product change','source':bind(A/'focused_checks_attempt02.py'),'log':bind(A/'focused_checks_attempt02.log')}],
 'no_ROS_Gazebo_or_training':True})
(A/'review.md').write_text('''# N06 manager r04 独立离线复核

结论：**本轮最小终态恢复修补通过；最终 source/lifecycle/runtime 闭包尚未合格，不授予运行权限。** 原 r01/r02/r03 负结果全部保留。

63项作者测试通过；12组独立检查包含8个完整 supervisor 场景（complete/timeout/fault/stopped，各含正/负 assessment）。从 durable consume 起，经实际 reconcile_worker_exit 的 -9/-15 中断回收、close_unit、hash-chain journal 重放、行政投影，均形成1条 consumed infrastructure-negative回执+7条 retired。assessment 原字节/hash保持，回执绑定该 assessment，task标签保留；roslaunch退出码仍未知None，worker退出观察保留在原错误中。即使恢复的是 complete 标签，也只投影 terminal_integrity_failure，不被提升为成功。再次 reconcile 的全部 bytes/inode/mtime均不变。

另复测：已落盘错误回执保持原错误和字节，不被supervisor重写；没有assessment的中断仍闭合失败。非法JSON报告拒绝且不覆盖原文，本身仍是 unresolved，需要外部行政诊断；本报告没有声称任意损坏磁盘可自动收口。

独立AST对比确认manager只有原integrity计算之后的已有assessment读取，三helper除版本身份完全相同。原r03的metadata闭schema、canonical报告和9checks/6sources/11sweep、输入漂移保护通过等价继承及作者回归保留；原r01真实一次性Python子进程/pidfd清理和r02八份真实离线转换认证也只作已封继承证据，不计为本轮重新运行。

原r01 352、r02 733、r03 764及当前r04 791资源检查/封存时全部匹配；递归本地imports均在roster。真实三CSV bytes/inode/mtime未变。所有新状态文件和源形状报告均为临时FS synthetic fixture，未读仿真raw或制造物理结果；没有创建实际release、运行目录、登记seed、启动ROS/Gazebo或训练。

保留两次审查夹具错误及源码/日志：Python3.8不支持的AST便利函数、临时stage遗漏真实stage既有runs父目录。只修审查脚本，作者字节不动。最终相同8场景全部重跑通过。

仍需独立冻结磁盘runtime依赖与实际启动环境、N06自身source/lifecycle最终API和未来exact release/freshness检查。三CSV为可恢复顺序替换而非联合原子；受控单写入者前提与N05外层143原因未知限制均保留。
''')
extra=[author,manifest,A.parent/'r03/review.json',A.parent/'r02/review.json',A.parent/'r01/review.json',A.parent/'disposable_process_result.json',S/'audits/navigation_n06_conversion_independent/review.json',S/'audits/navigation_n06_source_lifecycle_scope/scope.json']
extra += [p for p in A.iterdir() if p.is_file() and p.name not in ('resources.json','review.json')]
resources=dict(rosters['r04'])
for p in extra:r=bind(p);resources[r['path']]=r
write('resources.json',{'schema':'independent_r04_resources_v1','resources':sorted(resources.values(),key=lambda r:r['path'])})
report={'schema':'astra_navigation_n06_manager_independent_review_v4',
 'status':'independent_offline_late_assessment_recovery_pass_runtime_closure_pending','independent_review':True,'manager_component_review_pass':True,
 'execution_ready':False,'execution_authorized':False,'author_report':bind(author),'author_manifest':bind(manifest),
 'reviewed_manager_sha256':bind(P/'run_navigation_n06_manager_r04.py')['sha256'],'reviewed_sources':[bind(r['path']) for r in ar['sources']],
 'old_rosters_unchanged':{'r01':352,'r02':733,'r03':764},'r04_resources_rehashed_before_and_at_seal':791,
 'author_tests_rerun':{'passed':63,'log':bind(A/'author_tests_rerun.log')},
 'independent_focused':{'groups':12,'whole_supervisor_scenarios':8,'result':bind(A/'focused_result.json'),'script':bind(A/'focused_checks.py'),'log':bind(A/'focused_checks_attempt03.log')},
 'r03_late_assessment_counterexample_fixed':True,'overall_interrupted_integrity_promoted':False,'assessment_bytes_preserved':True,'closed_one_consumed_seven_retired':True,
 'existing_error_receipt_preserved':True,'reconcile_repeat_noop':True,'recursive_local_import_count':len(seen),'recursive_import_evidence':bind(A/'recursive_static_imports.json'),
 'retained_reviewer_errors':bind(A/'commands.json'),
 'remaining_limits':['Malformed/incomplete retained reports remain unresolved and never become evidence.','Synthetic source-shaped fixtures prove protocol consistency, not bag/physical correctness.','Final N06 runtime/environment and source/lifecycle review pending; no actual release verified.','Sequential CSV visibility and controlled sole-writer assumption retained.','N05 outer-shell143 causal mechanism remains unknown.'],
 'global_readonly_observations':bind(A/'global_readonly_observations.json'),'resources':bind(A/'resources.json'),'narrative':bind(A/'review.md'),
 'authority':{'actual_release_created':False,'seed_registered':False,'ROS_started':False,'training_started':False,'real_CSV_written':False,'author_source_modified':False}}
write('review.json',report);print(json.dumps({'review':bind(A/'review.json'),'resources':bind(A/'resources.json'),'status':report['status']},indent=2))
