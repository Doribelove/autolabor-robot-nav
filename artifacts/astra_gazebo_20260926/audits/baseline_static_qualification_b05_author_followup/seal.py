import hashlib,json,pathlib
S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');A=pathlib.Path(__file__).parent;O=S/'audits/baseline_static_qualification_b05'
def bind(p):
 p=pathlib.Path(p).resolve();b=p.read_bytes();return {'path':str(p),'sha256':hashlib.sha256(b).hexdigest(),'bytes':len(b)}
def write(name,x):(A/name).write_text(json.dumps(x,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
results=json.loads((A/'probe_results.json').read_text());assert results['retained_negative_count']==1 and results['author_recheck_only'] is True
(A/'README.md').write_text('''# B05 作者补充复核（非独立审查）

原作者复核，不能代替独立审查。原 B05 的264个封存资源均重新核验一致，没有修改源码、设计或历史报告。

保留一项真实 API 输入类型缺口：`proposal_schedule` 的 `other_proposed_seeds` 使用原值构建 set；若调用者给十进制字符串 `'263004001'`，不会与 int 新提案发生相交而被接受。同一个 int 值会被拒，registry 的 CSV 字符串也会因该分支调用 int 而被拒。当前已冻结 prepare 从 N06 JSON取整数，实际当前 registry 无冲突，重建提案与封存 schedule 完全一致。因此当前具体零权限提案没有发现种子冲突，但该 helper 不能声称任意 caller 类型均闭合。后继修订应严格校验整数元素或显式规范化并拒绝歧义；保留本版，不暗改修复。

完整参数比较只证明与传入 prepared 树相等。两个相同但不完整的树仍可通过，并显式返回 `runtime_default_completion_proved_by_this_function=false`；必须由后继 prepare/release 独立认证完整 defaults 和来源。记账 helper 也能对人为提供的布尔/零时目标做算术，但明确 `qualification_determined=false`；不能代替真实源时间/几何/回执验证。

冻结设计确实把 initialized defaults、实际九个 collision 部件在后轴参考下的投影包含、命令到 driver receipt 闭包和新 manager/collector 标为 pending。九个是实际碰撞部件数量，不是九种 primitive。B03 tilt、partial-batch 缺回执、public base终态权限和动态 actor不支持的边界继续保留。

此次只有5组小型离线探针，未启动 ROS/Gazebo、训练、登记种子或创建 release。独立审查应由非作者在 N06 优先任务完成后另行执行。
''')
resources={}
for r in json.loads((O/'source_evidence_manifest.json').read_text())['resources']:
 a=bind(r['path']);assert a['sha256']==r['sha256'];resources[a['path']]=a
for p in [O/'component_report.json',O/'source_evidence_manifest.json']+list(A.iterdir()):
 if p.is_file() and p.name not in ('report.json','resources.json','postwrite_check.json'):
  r=bind(p);resources[r['path']]=r
write('resources.json',{'schema':'astra_b05_author_followup_resources_v1','resources':sorted(resources.values(),key=lambda r:r['path'])})
report={'schema':'astra_b05_author_followup_v1','status':'author_recheck_current_inputs_unchanged_one_generic_API_type_gap_retained','author_review':True,'independent_review':False,'original_report':bind(O/'component_report.json'),'original264_resource_hashes_match':True,'original_files_modified':False,'probe_results':bind(A/'probe_results.json'),'probe_count':5,'retained_negative':{'id':'other_proposed_seed_numeric_string_not_rejected','current_frozen_actual_input_affected':False,'generic_API_qualification':False,'new_revision_before_expanded_caller_use_required':True},'readiness':{'execution_authorized':False,'execution_ready':False,'actual_physical_rear_base_qualified':False,'nine_collision_component_containment_implemented':False,'full_initialized_default_preparation_implemented':False,'B05_command_receipt_manager_collector_implemented':False,'dynamic_actor_qualified':False},'command':'PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 /usr/bin/python3 -B '+str(A/'probes.py'),'command_exit_code':0,'ROS_Gazebo_training_started':False,'seed_registered':False,'release_created':False,'resources':bind(A/'resources.json'),'README':bind(A/'README.md')}
write('report.json',report)
for r in resources.values():assert bind(r['path'])['sha256']==r['sha256']
write('postwrite_check.json',{'status':'pass','resource_count':len(resources),'report':bind(A/'report.json'),'resources':bind(A/'resources.json')})
print(json.dumps({'report':bind(A/'report.json'),'resources':bind(A/'resources.json'),'resource_count':len(resources)},indent=2))
