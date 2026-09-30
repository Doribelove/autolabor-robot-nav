from pathlib import Path
import json,csv,hashlib
O=Path(__file__).resolve().parent;S=O.parents[2];A=S/'audits';sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest();r=json.loads((O/'comparison_report.json').read_text());x=r['q02_numeric'];y=r['q03_numeric'];p=y['peak'];q=r['figure_qc']
q['view_image_status']='accepted_r02_actual_170mm_PDF_print_render_and_native_600dpi_PNG_viewed';q['revision_history']=[{'revision':'r01','viewed':True,'accepted':False,'reason':'Q03 less-negative local extrema clipped by common y upper limit; footer overlapped xlabel. Original preserved.'},{'revision':'r02','viewed':True,'accepted':True,'reason':'Common y range now includes all local data; footer and labels separated; fonts/legends/source times/negative peaks readable at170mm.'}]
r['source_rehash_pass']=all(sha(Path(z['path']))==z['sha256'] for z in r['sources']);r['scripts'].append({'path':str(Path(__file__)),'sha256':sha(__file__)});r['generation_commands'].append('workspace/.venv/bin/python visuals/plots/actuation_q03_comparison/finalize_comparison.py');r['Q02_original_figure_preserved']=str(O.parent/'actuation_q02_diagnosis/r05/AQC01.pdf');r['Q02_original_negative_diagnosis_preserved']=str(A/'actuation_q02_slew_diagnosis.json')
(A/'actuation_q02_q03_comparison.json').write_text(json.dumps(r,indent=2,ensure_ascii=False)+'\n');(O/'comparison_report.json').write_text(json.dumps(r,indent=2,ensure_ascii=False)+'\n');(O/'r02/comparison_report.json').write_text(json.dumps(r,indent=2,ensure_ascii=False)+'\n');(O/'r02/compare_and_plot_source.py').write_bytes((O/'compare_and_plot.py').read_bytes())
text=f'''# Q02 与 Q03：保持候选未通过原冻结门

两次独立fresh身份回合均完整36s、79/80门通过，唯一失败为实际中心角20ms变化率。Q03未采纳，不以放宽门或追加同类回合选择成功样本。

| 既有回合 | Q02 原驱动 | Q03 位置＋零角速度保持 |
|---|---:|---:|
| seed | 260926005 | 260926006 |
| 记录的最大绝对中心变化率 rad/s | {x['max_actual_rate_radps']:.9f} | {y['max_actual_rate_radps']:.9f} |
| 原门 rad/s | {x['gate_radps']:.2f} | {y['gate_radps']:.2f} |
| 超门区间 / 全部20ms区间 | {x['violating_intervals']}/{x['source_samples']-1} | {y['violating_intervals']}/{y['source_samples']-1} |
| 最大全程实际−发布角残差 mrad | {x['max_actual_minus_published_rad']*1000:.6f} | {y['max_actual_minus_published_rad']*1000:.6f} |

Q03最坏源区间{p['from']['source_s']:.3f}–{p['to']['source_s']:.3f}s，实际变化率{p['rates_radps']['actual_center_rad']:.9f}，内部{p['rates_radps']['published_center_rad']:.9f}，前轮轴向独立复核{p['rates_radps']['wheel_axis_center_rad']:.9f}rad/s。残差从{p['from']['actual_minus_published_rad']*1000:.6f}变为{p['to']['actual_minus_published_rad']*1000:.6f}mrad，额外变化率{p['rates_radps']['actual_minus_published_rad']:.9f}rad/s。Z轴twist与heading最大变化率差{y['max_actual_twist_rate_difference_radps']:.9g}rad/s，不能用Euler投影解释失败。

两回合采样均严格20ms、nearest joint状态恒早2ms。Q03每步位置/角速度投影减小了本回合最大角残差，却未抑制造成超门的快速残差变化。因此“保持即可获得原转向变化率资格”的具体候选未通过；不能由两不同freshseed的单回合比较推断统计因果大小，也不能把该负面推成唯一力学归因。仍缺500Hz完整姿态与接触力，后续真正轮驱动需独立设计，不能复用这两次身份或改写旧结果。

Q03 bag逐源核对：links 1808/1808完全一致；joint_states 1804消息与raw精确一致，bag少最早0.02/0.04/0.06s三包，raw完整保留；峰时刻无缺包。Q02对应链接全部匹配、早期joint bag缺九包已记录在原诊断。全部源SHA在最终报告生成时再次匹配。

对照图AQC02接受r02，170×180mm PDF、SVG、4015×4251原生600dpi PNG、中文独立图注、CSV、脚本和源manifest均在 `visuals/plots/actuation_q03_comparison/`。实际查看r01后发现局部上界裁剪及footer重叠，保留失败版本；r02已实际查看PDF版心渲染和原生PNG，接受。Q02单独AQC01 r05及原诊断保持不变。没有修改主figure_manifest或figure_plan，也未启动ROS。
'''
(A/'actuation_q02_q03_comparison.md').write_text(text)
manifest={'sources':r['sources'],'scripts':r['scripts'],'outputs':r['outputs'],'csvs':[{'path':str(p),'sha256':sha(p)} for p in [O/'source_time_steering.csv',O/'source_time_rates.csv',O/'numeric_diagnosis.json']]};(O/'source_manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
with (O/'figure_manifest_addition.csv').open('w') as f:
 w=csv.DictWriter(f,fieldnames=['figure_id','title','scope','pdf','svg','png','caption','qc_report']);w.writeheader();w.writerow(dict(figure_id='AQC02',title='原驱动与保持候选均未通过冻结转向门',scope='two individual simulator qualification failures; offline diagnosis only',pdf=str(O/'r02/AQC02.pdf'),svg=str(O/'r02/AQC02.svg'),png=str(O/'r02/AQC02.png'),caption=str(O/'r02/caption_zh.md'),qc_report=str(A/'actuation_q02_q03_comparison.json')))
print(json.dumps({'report_sha256':sha(A/'actuation_q02_q03_comparison.json'),'source_rehash_pass':r['source_rehash_pass'],'figure_pdf_sha256':sha(O/'r02/AQC02.pdf'),'figure_manifest':str(O/'figure_manifest_addition.csv')},indent=2))
