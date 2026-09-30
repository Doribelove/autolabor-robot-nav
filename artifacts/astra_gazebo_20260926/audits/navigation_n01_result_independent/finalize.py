#!/usr/bin/env python3
"""Final failure-inclusive N01 audit seal; does not reclassify original evidence."""
import csv,datetime,hashlib,json
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];T=D/'terminal_index_snapshot'
def read(p):return json.loads(p.read_bytes())
cache={}
def sha(p):
 p=Path(p)
 if str(p)not in cache:
  h=hashlib.sha256()
  with p.open('rb')as f:
   for x in iter(lambda:f.read(4194304),b''):h.update(x)
  cache[str(p)]=h.hexdigest()
 return cache[str(p)]
def bind(p):return {'path':str(p.resolve()),'sha256':sha(p)}
release_path=S/'contracts/navigation_n01_release.json';release=read(release_path);release_sha=sha(release_path);ledger=read(T/'navigation_n01_ledger.json');registry=[x for x in csv.DictReader((T/'seed_registry.csv').open())if x['run_id'].startswith('astra_navigation_n01_')];state=read(T/'STATE.json');visual_path=S/'audits/navigation_n01_root_visual_qc.json';visual=read(visual_path)
checks={};rows=[]
for method in ('astra_analytic_route_v1','actual_teb_common_route','frozen_r2_v060_shared_activation'):
 run='astra_navigation_n01_'+method;O=D/'rows'/run;R=S/'runs'/run;a=read(O/'r02/row_audit.json');s=read(O/'sweep_replay_audit.json');original=read(R/'assessment.json')
 checks[method+'_24_closed_row_checks']=a['pass']and len(a['checks'])==24 and all(a['checks'].values())
 checks[method+'_exact_full_geometry_replay']=s['pass']and all(s['checks'].values())
 checks[method+'_original_valid_completion_preserved']=all(original[k]is True for k in ('capture_integrity_pass','safety_within_declared_engineering_scope_pass','valid_completed'))
 rows.append({'run_id':run,'method':method,'seed':260926026,'capture_integrity_pass':True,'valid_completed':True,'original_task_phase':original['task']['task_phase'],'raw_native_goal_time_s':original['task']['raw_geometric_arrival_time_s'],'coordinator_actual_odom_goal_time_s':original['task']['coordinator_goal_time_s'],'source_window_ns':a['formal_source_window_ns'],'dense_intervals':a['native_dense_intervals'],'engineering_minimum_swept_xy_m':a['minimum_engineering_swept_xy_m'],'contact_counts':a['contact_counts'],'row_audit':bind(O/'r02/row_audit.json'),'sweep_replay_audit':bind(O/'sweep_replay_audit.json')})
failure=read(D/'dwa_failure_audit.json');checks['DWA_15_negative_preservation_checks']=failure['pass']and len(failure['checks'])==15 and all(failure['checks'].values())
rows.append({'run_id':'astra_navigation_n01_dwa_common_route','method':'dwa_common_route','seed':260926026,'capture_integrity_pass':False,'valid_completed':False,'original_task_outcome_in_receipt':None,'raw_terminal_phase':'fault','raw_terminal_reason':failure['raw_terminal']['terminal_reason'],'source_window_ns':failure['diagnostic_formal_window_ns'],'observed_gate_to_fault_s':.26,'raw_native_goal_time_s':None,'coordinator_actual_odom_goal_time_s':None,'qualified_geometry_assessment_available':False,'failure_audit':bind(D/'dwa_failure_audit.json')})
checks['native_goal_and_TEB_process_plugin_independent_replay']=read(D/'goal_and_baseline_audit.json')['pass']is True
checks['terminal_exact_four_consumed_zero_retired']=ledger['status']=='terminal'and ledger['maximum_units']==ledger['consumed_units']==len(ledger['units'])==4 and all(x['status']=='consumed'for x in ledger['units'])
checks['fresh_paired_seed026_exact_method_roster']=len({x['run_id']for x in ledger['units']})==4 and {x['seed']for x in ledger['units']}=={260926026}and {x['method']for x in ledger['units']}=={x['method']for x in rows}
checks['registry_snapshot_retains_three_pass_one_failure']=len(registry)==4 and all(x['source_binding']==release_sha and int(x['seed'])==260926026 and x['status']==('consumed_terminal_capture_failure'if x['run_id'].endswith('_dwa_common_route')else'consumed_terminal_pass')for x in registry)
checks['global_snapshot31_no_active_jobs']=state['episodes_consumed']==31 and state['active_jobs']==[]
checks['all_snapshot_SHA_bindings']=all(sha(x['immutable_snapshot_path'])==x['sha256']for x in read(T/'manifest.json')['records'])
checks['seven_native_frames_root_actual_view_SHA_bound']=len(visual['images'])==7 and all(x['root_individually_viewed']and x['visual_accept']and sha(x['image']['path'])==x['image']['sha256']and sha(x['raw_message']['path'])==x['raw_message']['sha256']for x in visual['images'])
checks['release601_current_SHA_bound']=release_sha=='ae155b33ea2ad6c732163265c266764817ff9268587e136a0c9f9ff93aed9ea5'and len(release['resources'])==len({x['path']for x in release['resources']})==601 and all(sha(x['path'])==x['sha256']for x in release['resources'])
receipt_rows=[]
for x in ledger['units']:
 p=Path(x['receipt']['path']);r=read(p);ok=sha(p)==x['receipt']['sha256']and r['release_sha256']==release_sha and r['attempt']==1 and r['retry_allowed']is False and all(sha(b['path'])==b['sha256']for b in r['artifacts']);receipt_rows.append({'run_id':x['run_id'],'receipt':bind(p),'unchanged_first_attempt_all_artifacts':ok})
checks['four_receipts_all_artifacts_rehashed_no_retry']=all(x['unchanged_first_attempt_all_artifacts']for x in receipt_rows)
checks['no_extra_N01_run_directories']=sorted(p.name for p in (S/'runs').glob('astra_navigation_n01_*'))==sorted(x['run_id']for x in rows)
blobs={x['sha256']:x for x in release['resources']+[release[k]for k in ('prepared','source_review','passive_geometry_qualification')]};cas=[]
for digest,b in blobs.items():
 c=S/'code_blobs'/digest;s=c.stat();locals=[(S/'runs'/r['run_id']/'bound_blobs'/digest).stat()for r in rows];source=Path(b['path']).stat();cas.append({'sha256':digest,'size':s.st_size,'allocated_bytes':s.st_blocks*512,'inode':s.st_ino,'links':s.st_nlink,'four_rows_same_inode':all((x.st_dev,x.st_ino)==(s.st_dev,s.st_ino)for x in locals),'source_independent':(source.st_dev,source.st_ino)!=(s.st_dev,s.st_ino),'readonly':all(x.st_mode&0o222==0 for x in locals+[s])})
checks['all_four_rows_share_readonly_CAS_distinct_source_inode']=all(x['four_rows_same_inode']and x['source_independent']and x['readonly']for x in cas)
resources=[bind(p)for p in sorted(D.rglob('*'))if p.is_file()and p.name not in ('terminal_result_audit.json','terminal_result_audit.md')and not p.name.startswith('finalize_r')]
out={'status':'independent_N01_terminal_partial_success_failure_inclusive_closure_pass'if all(checks.values())else'independent_N01_terminal_audit_failed','pass':all(checks.values()),'independent_review':True,'stage':'astra_navigation_n01','checks':checks,'release':bind(release_path),'four_method_qualification_pass':False,'original_complete_matrix_valid':False,'original_valid_completed_count':3,'original_capture_integrity_pass_count':3,'budget':{'maximum_units':4,'consumed':4,'retired':0,'retries':0,'attempts_per_identity':1},'rows':rows,'receipt_bindings':receipt_rows,'registry_snapshot':registry,'CAS':{'unique_blobs':len(cas),'logical_bytes_per_row':sum(x['size']for x in cas),'shared_allocated_bytes':sum(x['allocated_bytes']for x in cas),'rows':cas},'native_visual_QC':bind(visual_path),'resources':resources,
 'DWA_failure_chain':failure['failure_chain'],'manager_process_scope':'Parent reported outer exec session exit143; immutable per-attempt receipts show roslaunch exit0 and terminal ledger proves four allocations closed. This review does not infer an unrecorded clean manager-process exit or alter any exit status.',
 'limits':['One clear static fixture and one shared simulator seed across four methods, not independent multi-scene or performance evidence. No superiority inference from descriptive8.562/7.814/8.954s times.',
 'Frozen R2 uses historical truth-dependent readiness/goal stopping and does not consume shared path for action selection; do not claim all methods strict sensor-only or fully shared-route fairness.',
 'DWA original assessment is absent; complete dense diagnostic source packets do not establish final runtime model/ELF closure or qualify the failed row.',
 'Zero recorded illegal contact is not collision-free proof. Engineering XY bound is limited to declared Gazebo model, not real continuous physical safety.',
 'Headerless method Twist generation time is unknown. Coordinator publisher-visible /clock, actual odometry header, native physics-step time and driver ingress time are distinct authorities.',
 'Static ELF mapping plus reviewed GL alias exception does not hash anonymous graphics executable bytes or prove byte origin.'],
 'bookkeeping_scope':'Only immutable terminal_index_snapshot files are long-lived hash bindings. Mutable global originals may advance independently.',
 'offline_harness_history':'r01 overstrict archive-path/full-unfiltered-coordinator/unused-served_path comparisons retained as failed audit report/log/source; r02 corrects only those independently documented observational scopes, no runtime/gate/original evidence changed.',
 'no_ROS_or_live_process_access':True,'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
assert out['pass'],[k for k,v in checks.items()if not v]
with(D/'terminal_result_audit.json').open('x')as f:json.dump(out,f,indent=2,ensure_ascii=False);f.write('\n')
with(D/'terminal_result_audit.md').open('x')as f:f.write('N01 独立终态核查完成：4/4首次身份消费、0退休、0重跑；三回合有效完成，DWA原始采集失败保留，因此四方法整体资格未通过。\n\n前三回合全部24项原始证据核查和完整几何CLI重放通过，原端点/区间逐字节吻合；实际500Hz几何到达8.562/7.814/8.954秒，与50Hz里程计终点语义分别核对。TEB实际加载的插件、固定全局路径插件和move_base同进程证据通过。七张原图源像素一致且主代理实际查看。\n\nDWA在gate后0.26秒因非平面/非有限方法命令进入fault；仅产生一个终态帧，未达到gate+1s首帧时刻。采集器随后缺图检查在final maps标记前抛错，最终ELF和评估缺失；异常清理双stop/pause均保留。该阴性不因诊断源流完整而重判。601项release资源、四回合CAS硬链接/原源不同inode、全部回执及终态预算快照均核验。\n\n仅为单一清空场景集成；旧R2 truth生命周期与不消费shared route的限制保留，不能据三个时间作四方法性能优劣推断。外层session143与各roslaunch回执0分开记载，不推断未记录的manager正常退出。\n')
print(json.dumps({'status':out['status'],'pass':out['pass'],'checks':len(checks),'resources':len(resources),'SHA256':sha(D/'terminal_result_audit.json')},indent=2))
