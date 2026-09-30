"""Reproducible Chinese caption and insertion metadata for the reviewed r02 plot."""
from pathlib import Path
import hashlib
import json

S=Path(__file__).resolve().parents[3]
HERE=Path(__file__).resolve().parent
P=HERE/'r02'
D=S/'audits/navigation_n04_first_analytic_trace_diagnosis/diagnosis.json'
diagnosis=json.loads(D.read_bytes())
manifest=json.loads((P/'manifest.json').read_bytes())
review_path=S/'audits/navigation_n04_NAV01_independent/review.json'
review=json.loads(review_path.read_bytes())
assert review['status']=='independent_NAV01_data_source_and_scope_review_pass'
stops=diagnosis['stop_reason_counts']
title='静态绕障失败中的候选可行性与控制时限'
caption=(f"N04 开发集首个解析规划器静态绕障回合的四信号诊断。"
         f"{manifest['candidate_rows']} 次候选记录中，{manifest['mask_empty_count']} 次备份硬掩码为空；"
         f"{manifest['trace_rows']} 条控制记录包含 {stops['backup_revalidation_failed']} 次备份复核失败、"
         f"{stops['backup_stop_requested']} 次备份停车请求和 {manifest['deadline_drop_count']} 次计算超时丢弃。"
         "候选横轴采用决策时的仿真源时间，控制横轴采用日志发布时的仿真源时间，均减去共同起始门时刻；"
         "它们不等同于驱动接收时间。速度为方法请求值；备份预测净空包含既定裕量与区间修正，须与实际运动和实测物理净空区分。"
         "本回合记录完整性与规定工程范围内的安全检查通过，但任务在 22 s 上限结束，失败计入时间为 22 s。"
         "所有候选记录的动态回波分类数均为零。该单回合用于解释日志中的停滞与时限现象，不能据此估计各因素的反事实贡献或总体性能。")
text=(f"# 图 NAV01　{title}\n\n{caption}\n\n"
      "建议章节：开发阶段失败分析／执行时序与候选支持。\n\n"
      "方法：冻结的解析路线规划器；场景：静态绕障；回合：astra_navigation_n04_b00_astra_analytic_route_v1；种子：263001001；门后时间：0–22 s。\n\n"
      "版本：r02，170×165 mm。数据图为 PDF/SVG 矢量并附 400 dpi PNG（2677×2598）；本图不属于 Gazebo 原生截图。"
      "选择规则为首个按时间关闭的解析规划器失败回合，独立于 F06–F11 的按族代表案例预注册选择规则。\n")
with (P/'caption_zh.md').open('x') as f:f.write(text)
latex=("% Requires graphicx and a Chinese-capable thesis document. Copy PDF into figures/.\n"
       "\\begin{figure}[htbp]\n  \\centering\n"
       "  \\includegraphics[width=\\linewidth]{figures/NAV01_static_failure_trace.pdf}\n"
       f"  \\caption{{{title}。{caption}}}\n"
       "  \\label{fig:nav01-static-failure-trace}\n\\end{figure}\n")
with (P/'NAV01_zh.tex').open('x') as f:f.write(latex)
def bind(p):return dict(path=str(p.resolve()),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
value=dict(figure_id='NAV01',accepted_distinct_group=True,group_count=1,plot_manifest=bind(P/'manifest.json'),
           independent_data_review=bind(review_path),root_visual_review=bind(S/'audits/navigation_n04_NAV01_root_visual_qc_r02.json'),
           chinese_title=title,outputs=[bind(P/'caption_zh.md'),bind(P/'NAV01_zh.tex')],generator=bind(Path(__file__)),
           latex_integrated_or_compiled=False,scope='one descriptive development failure, no population or causal-effect claim')
with (P/'delivery_manifest.json').open('x') as f:json.dump(value,f,indent=2,ensure_ascii=False);f.write('\n')
print(json.dumps(dict(figure_id='NAV01',accepted_distinct_group=True,chinese_caption=True)))
