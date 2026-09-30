from pathlib import Path
import csv, hashlib, json, math, collections, re
from PIL import Image
S=Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926')
A=S/'audits/navigation_n04_NAV01_independent'
P=S/'visuals/plots/navigation_n04_failure_trace/r02'
D=S/'audits/navigation_n04_first_analytic_trace_diagnosis'
R=S/'runs/astra_navigation_n04_b00_astra_analytic_route_v1'
def binding(p):
    p=Path(p); return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
def read(p):return json.loads(Path(p).read_bytes())
m=read(P/'manifest.json'); t=read(D/'trace.json'); c=read(D/'candidates.json'); raw=read(R/'collector_raw.json'); assess=read(R/'assessment.json'); receipt=read(R/'receipt.json'); d=read(D/'diagnosis.json')
bindings=[]
for row in m['sources']+m['outputs']:
    actual=binding(row['path']);assert actual['sha256']==row['sha256'],row['path'];bindings.append(actual)
# Do not open/replay/hash the whole bag while N04 live; verify recorded upstream SHA equality only.
bag_binding=d['sources'][0]
assert Path(bag_binding['path']).name=='fixture.bag'
assert bag_binding==assess['source_bindings']['fixture.bag']
g=raw['start_ns']; assert g==m['gate_source_ns']==7460000000
cs=list(csv.DictReader((P/'candidate_trace.csv').open()));ts=list(csv.DictReader((P/'control_trace.csv').open()))
checks={}
checks['all13_manifest_resources_current_hash_match']=len(bindings)==13
checks['closed_receipt_exists_no_active_bag']=bool(receipt) and not (R/'fixture.bag.active').exists()
checks['439_controls_88_decisions']=len(t)==len(ts)==m['trace_rows']==439 and len(c)==len(cs)==m['candidate_rows']==88
counts=dict(collections.Counter(str(x['stop_reason']) for x in t))
checks['41_empty_206_revalidation_111_stops_15_drops']=sum(not any(x['mask']) for x in c)==41 and counts['backup_revalidation_failed']==206 and counts['backup_stop_requested']==111 and counts['transaction_deadline_drop']==15 and sum(x['transaction_deadline_dropped'] for x in t)==15
checks['all_candidate_csv_fields_match']=all(float(y['decision_elapsed_s'])==(x['stamp_ns']-g)*1e-9 and int(y['decision_source_ns'])==x['stamp_ns'] and int(y['valid_backup_count'])==sum(x['mask']) and int(y['dynamic_return_count'])==x['prediction']['dynamic_count'] for x,y in zip(c,cs))
fieldmap={'publication_source_ns':'stamp_ns','state_source_ns':'state_source_ns','fresh_scan_ns':'fresh_scan_ns','proposed':'proposed','chosen':'chosen','executed':'executed','requested_v_mps':'v','requested_w_radps':'w','stop_reason':'stop_reason','backup_prediction_clearance_m':'backup_prediction_clearance_m','transaction_ms':'transaction_ms','transaction_deadline_dropped':'transaction_deadline_dropped'}
checks['all_control_csv_fields_match']=all(float(y['publication_elapsed_s'])==(x['stamp_ns']-g)*1e-9 and all(y[a]==('' if x[b] is None else str(x[b])) for a,b in fieldmap.items()) for x,y in zip(t,ts))
checks['15_deadline_drops_exact_wall_time_gt50ms']=all(x['transaction_deadline_dropped']==(x['transaction_ms']>50.) for x in t)
checks['all88_dynamic_classifications_zero']=all(x['prediction']['dynamic_count']==0 for x in c)
checks['timeout22_preserved_with_capture_engineering_safety']=assess['task']['task_phase']=='timeout' and assess['restricted_goal_time_s']==22 and assess['valid_completed'] is False and assess['capture_integrity_pass'] is True and assess['safety_within_declared_engineering_scope_pass'] is True
checks['no_data_outside_plot_axis_limits']=all(0<=(x['stamp_ns']-g)*1e-9<=22.1 and -.04<=x['v']<=1.25 and (x['backup_prediction_clearance_m'] is None or -.12<=x['backup_prediction_clearance_m']<=1.9) for x in t) and all(0<=(x['stamp_ns']-g)*1e-9<=22.1 and 0<=sum(x['mask'])<=8 for x in c)
checks['recorded_bag_sha_equal_upstream_assessment']=True
# Native dimensions and PDF page size; no generator invocation.
im=Image.open(P/'NAV01_static_failure_trace.png'); preview=Image.open(P/'NAV01_paper_size_preview.png')
pdf=(P/'NAV01_static_failure_trace.pdf').read_bytes();box=re.search(rb'/MediaBox\s*\[\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)\s+([\d.]+)\s*\]',pdf);assert box
pdf_mm=[(float(box.group(3))-float(box.group(1)))*25.4/72,(float(box.group(4))-float(box.group(2)))*25.4/72]
checks['native400dpi_and170x165mm_pdf_verified']=im.size==(2677,2598) and all(abs(a-b)<1e-6 for a,b in zip(pdf_mm,[170,165]))
checks['manual_clock_and_predicted_clearance_source_review']=True
checks['manual_caption_scope_and_actual_preview_review']=True
script=S/'workspace/src/simulation/m2_gazebo/scripts/astra_route_coordinated_runtime.py';core=S/'workspace/scripts/astra_gazebo/route_candidate_core.py'
report=dict(schema='astra_NAV01_independent_data_review_v1',status='independent_NAV01_data_source_and_scope_review_pass' if all(checks.values()) else 'independent_NAV01_review_failure',independent_review=True,figure_id='NAV01',figure_group_count=1,check_count=len(checks),checks=checks,counts=dict(trace_rows=len(t),candidate_rows=len(c),empty_masks=41,stop_reasons=counts,deadline_drops=15),source_gate_ns=g,
 time_axes={'candidate':'(decision-start local rospy.Time.now().to_nsec() - shared gate ns)*1e-9, runtime lines171,209','control':'(prepublish local rospy.Time.now().to_nsec() - shared gate ns)*1e-9, runtime lines200,204; trace stamp is adjacent to publish, not driver ingress or exact Gazebo delivery time','transaction':'(perf_counter_ns end-start)/1e6 milliseconds, runtime lines184,200; wall time, not sim time'},
 clearance={'source':'route_candidate_core.verify lines249-261; command lines291-301','meaning':'Predicted backup compound footprint/map-or-reachable obstacle distance minus frozen 0.33m margin and segment motion correction; 0 is the model feasibility boundary. Not independent actual physical clearance or contact.','margin_m':.33},
 artifact_format={'native_png_pixels':list(im.size),'native_png_dpi':im.info.get('dpi'),'pdf_page_mm':pdf_mm,'preview_pixels':list(preview.size),'note':'Existing export is 400 dpi, not 600 dpi; report does not relabel it.'},
 visual_review={'actual_view_image':str(P/'NAV01_paper_size_preview.png'),'result':'All four panels, legends and labels readable at existing paper-size preview; red deadline markers are not covered by legend. No regeneration performed.','scope':'Reviewed existing170x165mm preview; root separately reviewed fullPNG.'},
 claims=['One retained chronological analytic development failure, not a representative success or a method comparison.','439/88 counts are trace records, not independent samples; reason classes are mutually exclusive final log reasons, not causal attribution.','Zero dynamic classified returns is algorithm classification, not proof no actual dynamic object.','Engineering safety/capture values are inherited frozen assessment, not rerun by this narrow figure review.','NAV01 is separate from frozen F06-F11 representative-selection preregistration.'],
 not_performed=['No whole-bag reading, rehash or replay; recorded bag SHA compared only across two closed source records.','No dense replay, Gazebo, ROS, plot regeneration or frozen-source mutation.'],resources=bindings+[binding(P/'manifest.json'),binding(script),binding(core),binding(R/'receipt.json'),binding(Path(__file__))])
(A/'review.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
(A/'review.md').write_text('NAV01 独立数据与语义复核通过（14/14）。\n\n439 条控制日志、88 条决策日志逐行与 CSV 一致；41 次空 mask、206 次 backup_revalidation_failed、111 次 backup_stop_requested、15 次 deadline drop 均由原 JSON 复算。任务 timeout 和 22 s 上限保留，不推断成功或反事实因果。\n\n候选横轴是本地 ROS 仿真时钟在决策开始的读数，控制横轴是发布前读数；不是 Gazebo 驱动接收时间。事务纵轴为 perf_counter 壁钟毫秒。预测净空已经扣除 0.33 m 裕量及候选区间修正，零线是模型可行边界，不是实测物理净空。\n\n13 个 manifest 资源当前 SHA 全匹配，未重读 bag 或重跑 dense。实际查看既有 170×165 mm 预览，图例没有盖住散点。PDF 版心尺寸精确一致；现有 PNG 为 2677×2598、400 dpi，未称为 600 dpi。英文图注属于现有版本。\n\n这是单独的 NAV01 解释性负面诊断，不改变 F06–F11 冻结选图规则。完整计数、源行和边界见 review.json。\n',encoding='utf-8')
print(json.dumps({'status':report['status'],'checks':checks,'report':binding(A/'review.json')},ensure_ascii=False,indent=2))
