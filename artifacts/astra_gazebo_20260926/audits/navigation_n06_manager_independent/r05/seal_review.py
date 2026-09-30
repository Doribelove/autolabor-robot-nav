"""Freeze a bounded manager/environment review, no release or run authority."""
import ast,hashlib,json
from pathlib import Path
A=Path(__file__).resolve().parent;S=A.parents[2];P=S/'workspace/scripts/astra_gazebo'
def bind(p):
 p=Path(p).resolve();data=p.read_bytes();return {'path':str(p),'sha256':hashlib.sha256(data).hexdigest(),'bytes':len(data)}
def write(name,value):(A/name).write_text(json.dumps(value,ensure_ascii=False,indent=2)+'\n')
author=S/'audits/navigation_n06_manager_r05/implementation_report.json';manifest=author.parent/'source_evidence_manifest.json'
assert bind(author)['sha256']=='bd4e3a6c2700df586ff8894d2796a8d0f8783e30590b9e14ac1076e63ef2d111'
assert bind(manifest)['sha256']=='d04cb1fcccc1ccfc0de3d8d0ef131ec9bfd36d0784f99d1ab35e3544581eef2c'
rosters={}
for rev,count in [('r01',352),('r02',733),('r03',764),('r04',791),('r05',866)]:
 doc=json.loads((S/('audits/navigation_n06_manager_'+rev)/'source_evidence_manifest.json').read_text())
 rows={r['path']:r for key in ('owned_sources','read_only_dependencies','author_evidence') for r in doc[key]}
 assert len(rows)==count and all(bind(r['path'])['sha256']==r['sha256'] for r in rows.values());rosters[rev]=rows
result=json.loads((A/'focused_result.json').read_text());assert len(result['checks'])==14 and all(c['passed'] for c in result['checks'])
assert len(result['details']['late_assessment'])==4 and len(result['details']['execute_unit_cases'])==4
log=(A/'author_tests_rerun.log').read_text();assert 'Ran 75 tests' in log and log.rstrip().endswith('OK')
author_doc=json.loads(author.read_text());pending=[Path(r['path']) for r in author_doc['sources'] if r['path'].endswith('.py') and '/tests/' not in r['path']];seen=set();edges=[]
while pending:
 path=pending.pop()
 if path in seen:continue
 seen.add(path)
 for node in ast.walk(ast.parse(path.read_text())):
  names=[r.name for r in node.names] if isinstance(node,ast.Import) else [node.module] if isinstance(node,ast.ImportFrom) and node.module else []
  for name in names:
   local=P/(name.split('.')[0]+'.py')
   if local.exists():edges.append({'from':str(path),'to':str(local),'line':node.lineno});pending.append(local)
assert all(str(p) in rosters['r05'] for p in seen)
write('recursive_static_imports.json',{'modules':[bind(p) for p in sorted(seen)],'edges':edges,'all_in_author_roster':True,'not_a_runtime_disk_or_live_dependency_proof':True})
obs=[]
for row in json.loads((A.parent/'r04/global_readonly_observations.json').read_text()):
 p=Path(row['path']);s=p.stat();r=bind(p);assert r['sha256']==row['sha256'] and s.st_ino==row['inode'] and s.st_mtime_ns==row['mtime_ns'];obs.append(dict(r,inode=s.st_ino,mtime_ns=s.st_mtime_ns,unchanged=True))
write('global_readonly_observations.json',obs)
write('commands.json',{'author_tests':"source artifacts/astra_gazebo_20260926/workspace/scripts/astra_env.sh; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 python -m unittest discover -s artifacts/astra_gazebo_20260926/workspace/scripts/astra_gazebo/tests -p 'test_navigation_n06_*r05.py' -v",'focused':'source artifacts/astra_gazebo_20260926/workspace/scripts/astra_env.sh; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 python '+str(A/'focused_checks.py'),'seal':'PYTHONDONTWRITEBYTECODE=1 python3 '+str(A/'seal_review.py'),'exit_codes':{'author_tests':0,'focused_attempt01':0},'reviewer_test_errors':[],'authority':'Tests use temporary synthetic documents; only benign Python children. No real release or Gazebo/ROS child.'})
wrapper=S/'audits/navigation_n06_runtime_disk_closure_independent/r05_wrapper_precheck/review.json'
assert bind(wrapper)['sha256']=='fbee20870665e0c09241d3bf12e42f370a897a7ed0d684dd06ceaf2bc060ac6a'
(A/'review.md').write_text('''# N06 manager r05 独立离线审查

结论：**受限入口与 manager 环境执行链组件通过；最终 N06 source/lifecycle 和 runtime disk 闭包仍待独立封存。** 本报告不授予 release、种子或 ROS/Gazebo 执行权限。

实际复跑 75 项作者测试，通过。14 组独立检查完成，r01/r02/r03/r04/r05 的 352/733/764/791/866 项资源在测试前及封存时全部匹配。递归本地静态 imports 也全部位于作者清单内；这不替代动态 loader/Python/ROS 依赖闭包。

独立测试调用真实的外层 env -i → bash --noprofile --norc → 系统 Python -I -S → bootstrap。用冻结的实际 execution_environment，而非空字符串 namespace，构造临时 synthetic 输入。无害 BASH_ENV/sitecustomize 标记均未执行；bootstrap 的 inspect 与拦截最终 execve 的真实 main 分支均确认 manager/ROS 尚未导入且传入精确绑定字典。8 类错误（SHA、遗漏/错哈希入口、重复资源、未绑定 runtime、错 stage、错解释器 prefix、禁用字段）均在导入 manager 前拒绝。另复测直接 bootstrap 多环境字段拒绝。

实际 live_preflight 对所有禁用键、未知键和每个已声明字段的改变均在 publish/consume/Popen 前拒绝。旧 LD_PRELOAD/LD_AUDIT/PYTHONHOME/PYTHONUSERBASE/EGL 继承反例在字典层拒绝，未实际加载这些注入值。实际 supervisor 的 worker Popen 收到完整固定字典；模拟 spawn 失败后 0 consumed、8 retired，没有伪造子进程退出观察。

实际 execute_unit 用未 stub 的环境 helper 执行四例：正常、启动前父字典改变、assessment 前父字典改变、assessment 异常。仅 ROS 子进程被替身截获；两次 assessment 内真正启动无害系统 Python，观测完整环境仅包含固定字段及 run 的 ROS_HOME/ROS_LOG_DIR/nonce。正常和异常退出均恢复原精确父字典；两个漂移用例拒绝相应启动/评估，保留失败回执。临时运行路径和 synthetic 报告不能充当真实导航证据。

r04 晚到 assessment 恢复的两个函数 AST 与 r05 完全一致。又在实际 reconcile_worker_exit 上重放 complete/timeout/fault/stopped 四种 retained assessment，均闭合 1 consumed infrastructure failure + 7 retired；保留 assessment 字节/SHA、-9 错误与 task 标签，不将 complete 标签提升为成功。再次 reconcile 对 bytes/inode/mtime 为只读。非法/不完整 assessment 仍是已披露 unresolved 范围，不声称任意损坏磁盘可自动收口。

最终 gazebo_ros wrapper 的八份 shell 观测由另一位审查者独立重放，本报告绑定其 r05_wrapper_precheck 并明确继承；未重复运行 wrapper，也未把环境观测当成完整 disk 闭包。它保留旧空模型 URI 恢复远程地址的负例；当前非空 localhost URI 在观测中保持。完整 runtime 仍待单独报告和最终候选 roster。

精确启动方式包含外层 clean env；直接从任意已加载环境导入 manager 不是替代入口。此方案是可信本机、受控唯一写入者工作流的一致性门，不是抵御恶意同账户修改或未知 OS/loader 攻击的证明。哈希自举依赖外部审过并绑定的入口字节。子进程与评估门会捕获后续父环境漂移，但若漂移在已分配身份之后才出现，则会消费该身份并失败，不宣称回收预算。

真实三张 CSV 的 bytes/inode/mtime 与前审观察一致，未修改作者源码、旧报告或真实账本。N05 outer-shell 143 原因未知等既有限制保留。
''')
resources=dict(rosters['r05'])
extras=[author,manifest,wrapper,A.parent/'r04/review.json',A.parent/'r03/review.json',S/'audits/navigation_n06_source_lifecycle_scope/scope.json']
extras.extend(p for p in A.iterdir() if p.is_file() and p.name not in ('resources.json','review.json'))
for path in extras:r=bind(path);resources[r['path']]=r
write('resources.json',{'schema':'independent_n06_manager_r05_resources_v1','resources':sorted(resources.values(),key=lambda r:r['path'])})
report={'schema':'astra_navigation_n06_manager_independent_review_v5','status':'independent_offline_restricted_startup_and_child_environment_pass_runtime_closure_pending','independent_review':True,'manager_component_review_pass':True,'execution_ready':False,'execution_authorized':False,'stage':'astra_navigation_n06_lookahead_static_pilot','author_report':bind(author),'author_manifest':bind(manifest),'reviewed_manager_sha256':bind(P/'run_navigation_n06_manager_r05.py')['sha256'],'reviewed_sources':[bind(r['path']) for r in author_doc['sources']],
 'author_tests_rerun':{'passed':75,'log':bind(A/'author_tests_rerun.log')},'independent_focused':{'groups':14,'bootstrap_negative_cases':8,'actual_execute_unit_cases':4,'actual_benign_assessment_children':2,'whole_supervisor_late_assessment_cases':4,'script':bind(A/'focused_checks.py'),'result':bind(A/'focused_result.json'),'log':bind(A/'focused_checks_attempt01.log')},
 'old_rosters_unchanged':{'r01':352,'r02':733,'r03':764,'r04':791},'r05_resources_rehashed_before_and_at_seal':866,'recursive_local_import_count':len(seen),'recursive_import_evidence':bind(A/'recursive_static_imports.json'),
 'restricted_first_import_entry_reviewed':True,'preallocation_full_environment_rejection':True,'actual_worker_roslaunch_assessment_environment_paths_reviewed':True,'r04_late_assessment_negative_preserved':True,
 'inherited_wrapper_independent_review':bind(wrapper),'wrapper_tests_independently_rerun_by_this_reviewer':False,'final_runtime_disk_closure_reviewed':False,'final_source_lifecycle_reviewed':False,
 'remaining_limits':['Final runtime disk envelope and N06 source/lifecycle exact roster remain separate pending prerequisites.','Trusted prescribed outer env-i entry is required; arbitrary direct imports are not supported.','No hostile same-account/OS self-authentication claim; final actual mapped-ELF and source-time evidence remain live prerequisites.','Malformed/incomplete retained assessment remains unresolved, never promoted.','Synthetic fixtures are protocol tests, not physical outcomes.','Late environment drift after identity allocation fails without recycling that identity.','N05 host-shell 143 cause remains unknown; manager and host statuses stay separate.'],
 'global_readonly_observations':bind(A/'global_readonly_observations.json'),'resources':bind(A/'resources.json'),'narrative':bind(A/'review.md'),'commands':bind(A/'commands.json'),
 'authority':{'actual_release_created':False,'seed_registered':False,'ROS_started':False,'Gazebo_started':False,'training_started':False,'real_CSV_written':False,'author_source_modified':False}}
write('review.json',report)
print(json.dumps({'report':bind(A/'review.json'),'resources':bind(A/'resources.json'),'status':report['status'],'recursive_modules':len(seen)},indent=2))
