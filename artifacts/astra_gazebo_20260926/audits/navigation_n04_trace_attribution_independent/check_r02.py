"""Compare new root r02 against our previously sealed independent five-bag replay."""
from pathlib import Path
import ast,csv,hashlib,json,math
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');A=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
bind=lambda p:dict(path=str(Path(p).resolve()),sha256=sha(p))
source=S/'audits/navigation_n04_trace_attribution_r02.py';report_path=S/'audits/navigation_n04_trace_attribution_r02/report.json';r=json.loads(report_path.read_bytes());ind=json.loads((A/'recomputed.json').read_bytes());trace=list(csv.DictReader((A/'all2190_trace_ticks.csv').open()));checks={}
def test(k,v):checks[k]=bool(v);assert v,k
# Execute only actual two pure geometry functions from root AST, no main or bag replay.
nodes=[x for x in ast.parse(source.read_text()).body if isinstance(x,ast.FunctionDef) and x.name in ('map_pose','distance_to_goal')];env={'math':math};exec(compile(ast.Module(body=nodes,type_ignores=[]),str(source),'exec'),env)
for i,(xy,transform,expected) in enumerate([((1,2),(0,0,0),(1,2)),((1,2),(-5.4,0,0),(-4.4,2)),((1,2),(3,4,math.pi/2),(1,5)),((1,2),(3,4,-math.pi/2),(5,3))]):
 got=env['map_pose'](xy,transform);test('pure_SE2_case'+str(i),max(abs(x-y) for x,y in zip(got,expected))<1e-12)
test('exact_five_identities',[x['run_id'] for x in r['rows']]==[x['run_id'] for x in ind['rows']])
keys={'trace_ticks':'trace_ticks','executed_stop_ticks':'executed_stop','backup_revalidation_failed_ticks':'backup_revalidation_failed','backup_stop_requested_ticks':'backup_stop_requested','transaction_deadline_drop_ticks':'transaction_deadline_dropped'}
for row,old in zip(r['rows'],ind['rows']):
 rid=row['run_id'];own=[x for x in trace if x['run_id']==rid];bounded=[x for x in own if old['bounds']['gate_source_ns']<=int(x['stamp_ns'])<old['bounds']['terminal_source_ns']]
 counts=dict(trace_ticks=len(bounded),executed_stop=sum(x['executed']=='0' for x in bounded),backup_revalidation_failed=sum(x['stop_reason']=='backup_revalidation_failed' for x in bounded),backup_stop_requested=sum(x['stop_reason']=='backup_stop_requested' for x in bounded),transaction_deadline_dropped=sum(x['transaction_deadline_dropped']=='True' for x in bounded))
 for suffix,k in keys.items():
  test('all_count:'+rid+suffix,row['all_bag_'+suffix]==old['counts'][k]);test('halfopen_count:'+rid+suffix,row['restricted_half_open_'+suffix]==counts[k])
 test('terminal_time:'+rid,row['terminal_source_ns']==old['terminal_odom']['source_ns'])
 test('terminal_map_distance:'+rid,row['terminal_map_goal_xy_distance_m']==round(old['terminal_odom']['map_goal_distance_m'],9))
 test('tail_map_distance:'+rid,row['bag_tail_map_goal_xy_distance_m']==round(old['bag_tail_odom']['map_goal_distance_m'],9))
 test('tail_time:'+rid,row['bag_tail_odom_source_ns']==old['bag_tail_odom']['source_ns'])
 test('transform:'+rid,json.loads(row['map_from_odom_xy_yaw'])==old['map_from_odom_xy_yaw'])
 root=S/'runs'/rid
 for name,key in [('fixture.bag','bag_sha256'),('assessment.json','assessment_sha256'),('collector_raw.json','collector_raw_sha256')]:
  # Our replay already fully hashed all bags; compare exact resulting bindings here.
  prior=next(x for x in ind['resources'] if x['path']==str(root/name));test('source_SHA:'+rid+name,row[key]==prior['sha256'])
 test('bundle_SHA:'+rid,row['bundle_sha256']==old['map_transform_source']['sha256'])
for k,n in r['totals'].items():test('total:'+k,n==sum(row[k] for row in r['rows']))
oldr=S/'audits/navigation_n04_trace_attribution/report.json';olds=S/'audits/navigation_n04_trace_attribution.py'
test('r01_negative_still_exact',sha(oldr)==r['old_r01_invalid_distance']['report_sha256']==sha(A/'r01_original/report.json') and sha(olds)==r['old_r01_invalid_distance']['script_sha256']==sha(A/'r01_original/navigation_n04_trace_attribution.py'))
test('ledger_snapshot_exact',r['source_ledger_sha256']==sha(A/'ledger_snapshot.json'))
test('analytic_exclusions_1plus6',r['paused_assessment_interrupted_excluded']==1 and r['retired_unrun_analytic_excluded']==6)
test('no_formal_or_rerun_claim',r['formal_result']is False and r['N04_rerun_allowed']is False)
# CSV equality against emitted JSON, preserving its textual numbers and no invented count.
csvp=S/'audits/navigation_n04_trace_attribution_r02/five_valid_analytic_replay.csv';csvrows=list(csv.DictReader(csvp.open()));test('csv_rows_match_JSON',len(csvrows)==len(r['rows']) and all(all(v==str(j[k]) for k,v in c.items()) for c,j in zip(csvrows,r['rows'])))
erratum=dict(affected=bind(A/'review.json'),kind='citation_only',old_source_lines=[47,48],correct_source_lines=[48,49],meaning='r01 goal is line48 and mixed-frame distance is line49; line47 is the missing-odom guard. All numerical findings, inputs and statuses unchanged.')
(A/'citation_erratum.json').write_text(json.dumps(erratum,indent=2)+'\n')
result=dict(status='independent_N04_attribution_r02_counts_frames_and_time_alignment_pass',independent_review=True,pass_=all(checks.values()),checks=checks,check_count=len(checks),reviewed_source=bind(source),reviewed_report=bind(report_path),prior_negative_review=bind(A/'review.json'),citation_erratum=bind(A/'citation_erratum.json'),actual_work='Parsed new source/report/CSV; executed2 actual pure transform functions on4 SE2 cases; compared every row/count/source hash/terminal and tail distance against independent original5-bag replay. No second bag replay needed.',sample_scope='5complete analytic only; one consumed assessment KeyboardInterrupt and six retired analytic excluded. Fullstage24consumed24retired.',time_scope='allbag2190 vs local prepublish ROSclock halfopen2182 are separate descriptive denominators; exact terminal odom source29.460s used for distance, tail120–260ms later separate.',limitations=['Current actual dataset only; zero requested command does not prove physical stop.','Trace stamp is local prepublish ROSclock, not driver-ingress source stamp.','No causal diagnosis, generalized or N05 benefit; no frozen task/safety reclassification.'],resources=[bind(p) for p in [source,report_path,csvp,A/'recomputed.json',A/'all2190_trace_ticks.csv',A/'review.json',A/'citation_erratum.json',Path(__file__)]],ROS_started=False,old_files_modified=False)
(A/'review_r02.json').write_text(json.dumps(result,indent=2)+'\n')
(A/'review_r02.md').write_text('N04 attribution r02 独立通过：新脚本的实际两个SE2函数4项纯数学样例通过；逐行JSON/CSV对照此前独立5bag重放，全部计数、源SHA、exact29.460s终态map距离与bag尾值相同。全bag2190条与局部prepublish ROSclock半开区间2182条明确分开；二者分别用于完整历史消息复现和限定时间窗口的描述统计，不能互换。1个assessment KeyboardInterrupt与6个退休analytic均排除，保留原消费/退休；不声称因果或N05收益。原r01坐标错误及所有旧证据保持。另citation_erratum.json仅将独立首报告原脚本行号从47–48更正为48–49，数值与结论不变。本次无ROS/仿真/训练。\n')
print(json.dumps({'status':result['status'],'checks':len(checks),'review':bind(A/'review_r02.json'),'markdown':bind(A/'review_r02.md'),'r01_negative':bind(A/'review.json'),'citation_erratum':bind(A/'citation_erratum.json')},indent=2))
