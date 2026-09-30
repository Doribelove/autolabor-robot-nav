#!/usr/bin/env python3
"""Validate generated artifacts and record the agent's completed visual review.

This script verifies files; it does not pretend to automate visual judgment.
The manual observations below were made with view_image on all three revisions.
"""
import csv
import datetime
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
from PIL import Image

ROOT = Path('/home/robot/robot_ws_base_rl')
STAGE = ROOT / 'artifacts/astra_gazebo_20260926'
OUT = STAGE / 'visuals/plots/historical_diagnosis'
REV = OUT / 'r03'

def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def main():
    manifest = json.loads((REV / 'source_manifest.json').read_text())
    checks = []
    def check(name, passed, detail=None):
        checks.append({'name': name, 'passed': bool(passed), 'detail': detail})
    check('source_hashes_match', all(sha(ROOT/p) == h for p,h in manifest['source_sha256'].items()),
          len(manifest['source_sha256']))
    check('generator_hash_matches', sha(OUT/'generate_figures.py') == manifest['script_sha256'])
    check('all_declared_figures_present', [f['figure_id'] for f in manifest['figures']] == ['H01','H02','H03'])
    check('all_text_extents_inside_canvas', not any(manifest['text_extent_machine_check'].values()))
    for f in manifest['figures']:
        ident = f['figure_id']
        check(ident+'_artifact_hashes', all(sha(ROOT/item['path']) == item['sha256'] for item in f['files'].values()))
        width,height = map(float,re.search(r'Page size:\s+([\d.]+) x ([\d.]+) pts',f['pdf_info']).groups())
        check(ident+'_actual_PDF_dimensions', abs(width*25.4/72-170)<.01 and abs(height*25.4/72-f['paper_height_mm'])<.01,
              [width*25.4/72,height*25.4/72])
        image = Image.open(ROOT/f['files']['png']['path'])
        check(ident+'_native_600dpi', all(abs(a-b)<1.1 for a,b in zip(image.size,[170/25.4*600,f['paper_height_mm']/25.4*600]))
              and all(abs(v-600)<.1 for v in image.info['dpi']), {'pixels':image.size,'dpi':image.info['dpi']})
        check(ident+'_embedded_CJK_font', 'NotoSansCJKjp-Regular' in f['pdf_fonts'] and 'yes no  yes' in f['pdf_fonts'])
        check(ident+'_historical_identity', not f['new_Gazebo_result'] and f['real_screenshot_count']==0)
    h01,h02,h03=[f['statistics'] for f in manifest['figures']]
    check('H01_all_two_failure_identities', set(h01)=={'198136','198146'})
    check('H01_full_trace_counts', sum(x['samples'] for x in h01.values()) == len(list(csv.DictReader((OUT/'data/H01_trace.csv').open()))))
    check('H02_common_event_rule', all(x['sustained_stop_onset_s']==3 and x['window_end_s']==4 and x['last_pre_stop_action']==3 for x in h02.values()))
    check('H03_all_18_timeouts', sum(h03['timeout_categories'].values())==18)
    check('H03_all_21_TF_failures',sum(h03['TF_failure_families'].values())==21)
    check('H03_warning_count_matches_rejected_scan_count',h03['runtime_log_extrapolation_rows']==h03['counter_rejected_scans']==24)
    revisions = [
        {'revision':'r01','generation_exit_code':1,'visual_status':'fail_retained',
         'issues':['Manifest relative path error after plot generation','H01/H02 bottom footnotes overlapped x-axis labels','H01 upper y limit clipped late full-trace clearance'],
         'source_snapshot':'r01/generate_figures_source_reconstructed.py',
         'source_snapshot_provenance':'reconstructed from exact applied edits, not a contemporaneous source copy'},
        {'revision':'r02','generation_exit_code':0,'visual_status':'superseded_retained',
         'issues':['Machine text-extent check inspected an off-range automatic tick at 25 ms','Added explicit distinction between control-trace minimum and retained result accumulator minimum'],
         'source_snapshot':'r02/generate_figures_source.py'},
        {'revision':'r03','generation_exit_code':0,'visual_status':'pass',
         'issues':[],'source_snapshot':'r03/generate_figures_source.py'}]
    shutil.copy2(OUT/'generate_figures.py', REV/'generate_figures_source.py')
    (OUT/'revision_history.json').write_text(json.dumps(revisions,ensure_ascii=False,indent=2)+'\n')
    manual = [
        {'figure_id':'H01','viewed_path':manifest['figures'][0]['files']['print_render']['path'],
         'judgment':'pass','observations':['Full timeline and 0–22 s shared axis readable','Both clearance curves and full ranges retained','Exact minima annotation readable','No footer/x-label overlap','Projection-only limitation explicit']},
        {'figure_id':'H02','viewed_path':manifest['figures'][1]['files']['print_render']['path'],
         'judgment':'pass','observations':['Both panels use same stop event window','Center error and direction units readable','Actor measured direction separated by dash/color','Stop onset dotted line clear','No footer/x-label overlap']},
        {'figure_id':'H03','viewed_path':manifest['figures'][2]['files']['print_render']['path'],
         'judgment':'pass','observations':['Timeout and analytic TF populations explicitly distinct','Every bar count legible','Dots and median lines distinct at 170 mm width','2–20 ms distribution visible','24 logs explicitly not 24 independent episodes']}]
    report = {'schema_version':1,'status':'historical_diagnostic_figures_qc_pass' if all(c['passed'] for c in checks) else 'failed',
        'reviewer':'historical_audit agent','created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),
        'scope':'Historical offline diagnosis only; no new simulator samples or performance claim',
        'accepted_revision':'r03','accepted_figure_count':3,'new_Gazebo_figure_count':0,'real_screenshot_count':0,
        'machine_checks':checks,'actual_view_image_review':manual,'previous_revisions_retained':revisions,
        'source_manifest_path':str((REV/'source_manifest.json').relative_to(ROOT)),
        'source_manifest_sha256':sha(REV/'source_manifest.json'),'source_count':len(manifest['source_sha256']),
        'generator_path':str((OUT/'generate_figures.py').relative_to(ROOT)),
        'generator_sha256':sha(OUT/'generate_figures.py'),
        'generation_command':manifest['command'],
        'qc_command':'python3 artifacts/astra_gazebo_20260926/visuals/plots/historical_diagnosis/finalize_qc.py',
        'qc_script_sha256':sha(Path(__file__).resolve()),
        'derived_csv_sha256':{str(p.relative_to(ROOT)):sha(p) for p in sorted((OUT/'data').glob('*.csv'))},
        'caveats':['Exact source hashes bind historical raw files and the independent replay CSV.','No historical evidence, results, gates or seeds were modified or rerun.','Projected chassis/actor intersection is not a confirmed physical contact.','Runtime log TF range is 2–20 ms; earlier sampled 4–20 ms range was incomplete.','Control-trace and result-accumulator minima are separately sampled calls and both are retained.','PDF contains embedded CJK font; SVG uses external Noto Sans CJK JP text font.','No inferential error bars are appropriate for these full failure-population diagnostic counts.'],
        'figures':manifest['figures']}
    (STAGE/'audits/historical_visual_qc.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    md = '''# 历史离线诊断图 QC

状态：**通过，采用 r03 的 H01–H03**。只使用冻结历史 R2 数据和独立复算 CSV，无新增 Gazebo 运行、截图或性能结论；没有修改主 figure_plan/figure_manifest。

三图均已用 `view_image` 实际逐张检查 PDF 的 170 mm 版心渲染。字体嵌入，最小字 7.2 pt，曲线 1.2 pt，底注/图例/轴标签清楚，所有文字边界检查通过。原生 PNG 均 600 dpi；PDF/SVG、中文独立图注、LaTeX 插入片段、原始来源 SHA 和命令全部保留于 `visuals/plots/historical_diagnosis/r03/`。图件有源 CSV，不用手写数字。

| 图号 | 主题 | 关联主计划 | 物理尺寸 |
|---|---|---|---|
| H01 | 两次净空失败：旧保守圆与独立二维几何、速度、非停 mask | F04/F30/F39 | 170×185 mm |
| H02 | 扫描中心/安全方向估计与实际 actor 位移方向 | F29/F39 | 170×140 mm |
| H03 | 全部 18 超时和另一个总体的 21 TF 完整性失败 | F32/F33/F39 | 170×164 mm |

r01 的生成异常与底注重叠/曲线裁切都保留；r02 排版修复后补充轨迹/汇总净空差异说明并修正离轴自动刻度的机器检查；r03 通过。修复史见 `revision_history.json`。r01 源快照是在精确编辑记录上重建的，已明确标记；r02/r03 为原样快照。

证据细节：全部运行期日志重新提取的 TF 缺口是 **2–20 ms**，24 条日志与 24 个拒绝计数对应；早先抽样的 4–20 ms 不是完整范围。seed198146 的 control_trace 最小值为 −0.269610573 m，result 累计最小值为 −0.269571175 m，两者独立记录，不重写历史结果。1.36 mm 二维投影相交不宣称已确认物理接触。

可复现命令（原目录防覆盖；再次离线生成须选新 revision）：

```bash
'''+manifest['command']+'''
python3 artifacts/astra_gazebo_20260926/visuals/plots/historical_diagnosis/finalize_qc.py
```

SVG 保留可编辑文本，需要 Noto Sans CJK JP；PDF 字体已嵌入且经 `pdffonts` 验证。实际检查结果、完整 SHA 与机器检查明细见同名 JSON。
'''
    (STAGE/'audits/historical_visual_qc.md').write_text(md)
    rows=[]
    for f in manifest['figures']:
        rows.append({'figure_id':f['figure_id'],'title_zh':f['title_zh'],'related_figure_plan_ids':';'.join(f['figure_plan_related_ids']),
          'evidence_class':'historical_offline_diagnosis','new_Gazebo_result':False,'qc_status':'pass',
          'pdf_path':f['files']['pdf']['path'],'svg_path':f['files']['svg']['path'],'png_path':f['files']['png']['path'],
          'caption_path':str((REV/(f['figure_id']+'_caption.md')).relative_to(ROOT)),
          'source_manifest_path':report['source_manifest_path'],'qc_path':'artifacts/astra_gazebo_20260926/audits/historical_visual_qc.json'})
    with (OUT/'historical_figure_manifest.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(json.dumps({'status':report['status'],'machine_checks':len(checks),'passed':sum(c['passed'] for c in checks),
                      'source_count':report['source_count'],'figures':len(manual)},ensure_ascii=False))
    if report['status']=='failed':raise SystemExit(1)

if __name__=='__main__':main()
