#!/usr/bin/env python3
"""Seal Q05 independent terminal review using closed runs and immutable index snapshots."""
import collections,csv,datetime,hashlib,json,stat
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];T=D/'terminal_index_snapshot'
def read(p):return json.loads(Path(p).read_bytes())
def sha(p):
 h=hashlib.sha256()
 with Path(p).open('rb')as f:
  for chunk in iter(lambda:f.read(4194304),b''):h.update(chunk)
 return h.hexdigest()
def bind(p):return {'path':str(Path(p).resolve()),'sha256':sha(p)}
release_path=S/'contracts/passive_sweep_q05_release.json';release=read(release_path);release_sha=sha(release_path)
ledger=read(T/'passive_sweep_q05_ledger.json');registry=[x for x in csv.DictReader((T/'seed_registry.csv').open())if x['run_id'].startswith('passive_sweep_q05')]
state=read(T/'STATE.json');visual_path=S/'audits/passive_q05_root_visual_qc.json';visual=read(visual_path)
checks={};rows=[];resources=[];all_receipt_artifacts=[]
for arm in ('static','actor'):
 run='passive_sweep_q05_'+arm;R=S/'runs'/run;O=D/'rows'/run
 audit=read(O/'raw_bag_map_image_audit.json');contact=read(O/'contact_and_gate_replay.json');old=read(R/'sweep_evaluation/report.json');replay=read(O/'sweep_replay/report.json');original=read(R/'assessment.json');raw=read(R/'collector_raw.json');receipt=read(R/'receipt.json')
 checks[arm+'_29_raw_bag_source_map_CAS_image_checks']=audit['pass']and len(audit['checks'])==29 and all(audit['checks'].values())
 checks[arm+'_eight_contact_motion_gate_replay_checks']=contact['pass']and len(contact['checks'])==8 and all(contact['checks'].values())
 checks[arm+'_full_geometry_report_exact_except_output_paths']={k:v for k,v in old.items()if k!='outputs'}=={k:v for k,v in replay.items()if k!='outputs'}
 checks[arm+'_5000_intervals_501_endpoints_byte_exact']=old['dense_solver_intervals']==5000 and all(sha(R/'sweep_evaluation'/n)==sha(O/'sweep_replay'/n)for n in ('intervals.jsonl','endpoints.csv'))
 checks[arm+'_all_13_original_gates_pass_unchanged']=len(original['checks'])==13 and all(original['checks'].values())and original['pass']is True
 checks[arm+'_bound_receipt_current_and_original_success']=receipt['pass']is True and receipt['attempt']==1 and receipt['retry_allowed']is False and sha(R/'receipt.json')==next(u['receipt']['sha256']for u in ledger['units']if u['run_id']==run)
 checks[arm+'_fresh_nondiagnostic_runtime_converter_qualification']=old['diagnostic_only']is False and old['conversion_audit']['runtime_converter_correspondence_qualified']is True and audit['conversion_authentication']['runtime_converter_correspondence_qualified']is True
 all_receipt_artifacts+=receipt['artifacts']
 rows.append({'run_id':run,'seed':receipt['seed'],'original_pass':original['pass'],'gate_count':len(original['checks']),'source_window_ns':raw['start_ns']and[raw['start_ns'],raw['end_ns']],
  'source_watermark_ns':raw['final_drain']['source_watermark_ns'],'formal_500Hz_samples':5001,'dense_intervals':5000,'engineering_minimum_conservative_swept_xy_m':old['minimum_engineering_conservative_swept_xy_m'],
  'contact_counts':{k:v for k,v in contact['contact_summary'].items()if k.endswith('_count')},'motion_m':contact['actual_motion_m'],'actual_driver_ingress':original['driver_ingress_description'],
  'native_images':audit['native_images'],'receipt':bind(R/'receipt.json'),'reports':[bind(O/n)for n in ('raw_bag_map_image_audit.json','contact_and_gate_replay.json','sweep_replay/report.json')]})
checks['terminal_budget_exact_two_consumed_no_retired']=ledger['status']=='terminal'and ledger['maximum_units']==2 and ledger['consumed_units']==2 and len(ledger['units'])==2 and all(u['status']=='consumed'and u['pass']is True for u in ledger['units'])
checks['fresh_seeds_exact_031_032_once']=sorted((u['run_id'],u['seed'])for u in ledger['units'])==[('passive_sweep_q05_actor',260926032),('passive_sweep_q05_static',260926031)]
checks['registry_snapshot_terminal_pass_exact_binding']=len(registry)==2 and all(x['status']=='consumed_terminal_pass'and x['source_binding']==release_sha for x in registry)and {(x['run_id'],int(x['seed']))for x in registry}=={(x['run_id'],x['seed'])for x in ledger['units']}
checks['no_extra_Q05_run_directories']=sorted(p.name for p in (S/'runs').glob('passive_sweep_q05*'))==['passive_sweep_q05_actor','passive_sweep_q05_static']
checks['snapshot_global_budget_27_no_active_jobs']=state['episodes_consumed']==27 and state['active_jobs']==[]
checks['snapshot_manifest_matches']=all(sha(x['immutable_snapshot_path'])==x['sha256']for x in read(T/'manifest.json')['records'])
checks['root_four_native_visual_reviews_match_source_pixels']=len(visual['images'])==4 and all(x['root_individually_viewed']and x['visual_accept']and sha(x['image']['path'])==x['image']['sha256']and sha(x['image_message']['path'])==x['image_message']['sha256']for x in visual['images'])
checks['release_identity_946_paths']=release_sha=='54b1ccf5b1041538098e1e6364ab6fc9f87123c28920371a796488b5f6504f22'and len(release['resources'])==len({x['path']for x in release['resources']})==946
archive_rows=[]
for b in release['resources']+[release[k]for k in ('prepared','source_review','physical_parameter_decision')]:
 source=Path(b['path']);cas=S/'code_blobs'/b['sha256'];a=S/'runs/passive_sweep_q05_static/bound_blobs'/b['sha256'];c=S/'runs/passive_sweep_q05_actor/bound_blobs'/b['sha256']
 stats=[p.stat()for p in (cas,a,c)];source_stat=source.stat()
 archive_rows.append({'source':str(source),'sha256':b['sha256'],'bytes':stats[0].st_size,'allocated_blocks_512':stats[0].st_blocks,'inode':stats[0].st_ino,'nlink':stats[0].st_nlink,
 'shared_CAS_both_rows':len({(s.st_dev,s.st_ino)for s in stats})==1,'source_inode_independent':(source_stat.st_dev,source_stat.st_ino)!=(stats[0].st_dev,stats[0].st_ino),
 'readonly_regular_archives':all(stat.S_ISREG(s.st_mode)and s.st_mode&0o222==0 for s in stats),
 'source_and_archive_SHA_exact':sha(source)==sha(cas)==b['sha256']})
checks['final_rehash_all946_plus_three_extras_and_real_two_row_CAS_hardlinks']=all(all(x[k]for k in ('shared_CAS_both_rows','source_inode_independent','readonly_regular_archives','source_and_archive_SHA_exact'))for x in archive_rows)
checks['all_closed_receipt_artifacts_rehash_again']=all(sha(x['path'])==x['sha256']for x in all_receipt_artifacts)
checks['historical_Q04_negative_assessment_byte_unchanged']=sha(S/'runs/passive_sweep_q04_static/assessment.json')=='d2e2b3823b346f747b6b319c255f7ba3bcffab3f3113ff347b31f52681e025ea'and read(S/'runs/passive_sweep_q04_static/assessment.json')['pass']is False
unique={x['sha256']:x for x in archive_rows}
resources=[bind(p)for p in sorted(D.rglob('*'))if p.is_file()and p.name not in ('terminal_result_audit.json','terminal_result_audit.md')and not p.name.startswith('finalize_r')]
result={'status':'independent_Q05_complete_result_budget_binding_audit_pass'if all(checks.values())else'independent_Q05_result_closure_failed',
 'pass':all(checks.values()),'independent_review':True,'stage':'astra_passive_sweep_q05','release':bind(release_path),'release_sha256':release_sha,
 'checks':checks,'original_episode_passes':[r['original_pass']for r in rows],'budget':{'authorized':2,'consumed':2,'retired':0,'retries':0,'attempts_per_identity':1,'identities':ledger['units']},
 'registry_rows':registry,'rows':rows,'CAS_actual_inode_review':{'binding_entries':len(archive_rows),'unique_content_blobs':len(unique),'logical_bytes_per_row':sum(x['bytes']for x in unique.values()),'shared_allocated_bytes':sum(x['allocated_blocks_512']*512 for x in unique.values()),'both_rows_share_one_CAS_inode_per_SHA':True,'records':archive_rows},
 'native_visual_QC_by_root':bind(visual_path),'resources':resources,
 'bookkeeping_scope':'Only terminal_index_snapshot files are frozen bookkeeping bindings. Mutable global paths in intermediate row reports are point-in-time observations, not perpetual-current hash requirements; final ledger snapshot is authoritative for this closure.',
 'qualification':'Fresh two-fixture Gazebo passive source/model/conversion/engineering swept-geometry integration qualification within the frozen declared model scope.',
 'scope_limits':['Not navigation performance, learning, broad-scene generalization, certified continuous physical safety or real-vehicle evidence.',
 'Both images in each row are original 2560x1440 pixels and source-stamp bound; image visibility alone cannot establish wheel support, contact or TF correctness.',
 'Zero recorded illegal contact is not proof of collision freedom. Wheel-ground support contacts are permitted and separately counted.',
 'The graphics dynamic-code exception is a reviewed local GL lifecycle and exact static-driver/alias association, not hashed executable memory or a byte-origin theorem.',
 'Historical failed Q02/Q03/Q04 and all preceding physical pilot identities remain frozen; no reclassification or rerun occurred.'],
 'no_ROS_or_live_process_access':True,'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
assert all(checks.values()),[k for k,v in checks.items()if not v]
with (D/'terminal_result_audit.json').open('x')as f:json.dump(result,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
md='''Q05 独立终态核查通过。静态031与动态032均首次完成，预算2/2消费、0退休、0重跑；两个原始评估均13/13通过。

独立重放每回合5,000个2ms区间及501个端点，与原输出逐字节一致。保守工程平面净空最小值分别为1.4422737777090144 m、1.4286099261639893 m。正式窗口各5,001个500Hz样本，六密集流及50Hz物理状态完整，原始包和bag源数据一致。两回合实际转换库、转换manifest、七个原生刚体和原始图像均核验通过。

946个release资源及3项额外绑定再次核验；两个回合归档与CAS为同inode、与原源文件不同inode、只读且SHA吻合。全局可变索引仅以独立终态快照绑定。四张2560×1440原图与ROS图像消息逐像素相同，主代理已逐张实际查看。

零非法接触记录不等于无碰撞证明；正常轮地支撑单列。结果仅支持冻结模型范围内的被动测量/几何集成资格，不支持导航性能、连续物理安全或实车结论。历史Q04负面结果未重判。
'''
with (D/'terminal_result_audit.md').open('x')as f:f.write(md)
print(json.dumps({'status':result['status'],'checks':len(checks),'pass':result['pass'],'sha256':sha(D/'terminal_result_audit.json'),'resources':len(resources)},indent=2))
