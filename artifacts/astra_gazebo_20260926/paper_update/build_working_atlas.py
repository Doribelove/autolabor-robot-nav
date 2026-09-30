#!/usr/bin/env python3
"""Create a versioned paper atlas from accepted, hash-bound figure rows.

The atlas is a working compilation. Each source image retains its own QC and
provenance; this builder cannot upgrade an unreviewed source into evidence.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

from reportlab.lib.pagesizes import A4
from reportlab.lib.utils import ImageReader
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.cidfonts import UnicodeCIDFont
from reportlab.pdfgen import canvas

S = Path(__file__).resolve().parents[1]
REPO = S.parent.parent
FONT_NAME = 'STSong-Light'
MM = 72/25.4


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def binding(path):
    return {'path': str(Path(path).resolve()), 'sha256': sha(path)}


def resolve_bound_path(path):
    path = Path(path)
    return path if path.is_absolute() else REPO/path


def lines(text, max_width, size):
    result = []
    current = ''
    for char in text:
        candidate = current + char
        if current and pdfmetrics.stringWidth(candidate, FONT_NAME, size) > max_width:
            result.append(current)
            current = char
        else:
            current = candidate
    if current:
        result.append(current)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists():
        raise FileExistsError('preserve previous atlas version')
    manifest_path = S/'figure_manifest.csv'
    plan_path = S/'figure_plan.csv'
    with manifest_path.open(newline='') as f:
        figures = [x for x in csv.DictReader(f) if not x['figure_id'].startswith('RAW-')]
    with plan_path.open(newline='') as f:
        plan = {x['figure_id']: x for x in csv.DictReader(f)}
    if len(figures) != len({x['figure_id'] for x in figures}) or not figures:
        raise ValueError('unique accepted paper figure rows required')
    for row in figures:
        if row['figure_id'] not in plan or not plan[row['figure_id']]['status'].startswith('accepted_distinct_group'):
            raise ValueError('figure not accepted in plan: '+row['figure_id'])
        source = resolve_bound_path(row['source_path'])
        image = resolve_bound_path(row['image_path'])
        if not source.is_file() or sha(source) != row['source_sha256']:
            raise ValueError('figure QC/source changed: '+row['figure_id'])
        if not image.is_file() or sha(image) != row['image_sha256']:
            raise ValueError('figure image changed: '+row['figure_id'])
        if 'pass' not in row['visual_qc']:
            raise ValueError('actual visual review absent: '+row['figure_id'])
    output.mkdir(parents=True)
    pdfmetrics.registerFont(UnicodeCIDFont(FONT_NAME))
    atlas = output/'FIGURE_ATLAS.pdf'
    c = canvas.Canvas(str(atlas), pagesize=A4, pageCompression=1)
    c.setTitle('Astra Gazebo Working Figure Atlas')
    page_width, page_height = A4
    margin = 15*MM
    max_width = page_width-2*margin
    count_note = ('已达至少 30 个不同图组的数量要求；后续正式实验结果仍需另版更新。'
                  if len(figures) >= 30 else '当前图册尚未达到至少 30 个不同图组的数量要求。')
    index = ['# 论文图册索引（阶段性）', '',
             '本图册只收录 `figure_manifest.csv` 与 `figure_plan.csv` 中已通过实际视检的独立图组。'
             '每图数据范围与限制以原图 QC 记录为准；'+count_note, '']
    tex = ['% Working atlas figure snippets; paths relative to repository root.',
           '% Figures are source-bound and individually QC reviewed.', '']
    records = []
    for page, row in enumerate(figures, 1):
        info = plan[row['figure_id']]
        image = resolve_bound_path(row['image_path'])
        reader = ImageReader(str(image))
        iw, ih = reader.getSize()
        title = f"{row['figure_id']}  {info['title_zh']}"
        title_lines = lines(title, max_width, 12)
        c.setFillColorRGB(.1, .16, .22)
        c.setFont(FONT_NAME, 12)
        top = page_height-margin
        for line in title_lines:
            c.drawString(margin, top, line)
            top -= 16
        top -= 10
        caption = info['research_question']
        scope = info['selection_rule']
        caption_lines = lines(caption, max_width, 8.3)
        scope_lines = lines(scope, max_width, 7.4)
        reserved = (len(caption_lines)+len(scope_lines))*11+31
        max_height = top-margin-reserved
        scale = min(max_width/iw, max_height/ih)
        draw_w, draw_h = iw*scale, ih*scale
        x = (page_width-draw_w)/2
        y = top-draw_h
        c.drawImage(reader, x, y, width=draw_w, height=draw_h, preserveAspectRatio=True, mask='auto')
        c.setStrokeColorRGB(.81, .85, .88)
        c.line(margin, y-9, page_width-margin, y-9)
        ty = y-21
        c.setFont(FONT_NAME, 8.3)
        c.setFillColorRGB(.12, .18, .22)
        for line in caption_lines:
            c.drawString(margin, ty, line)
            ty -= 11
        c.setFont(FONT_NAME, 7.4)
        c.setFillColorRGB(.36, .29, .26)
        for line in scope_lines:
            c.drawString(margin, ty, line)
            ty -= 11
        c.setFont(FONT_NAME, 6.8)
        c.setFillColorRGB(.4, .43, .46)
        c.drawString(margin, margin*.6,
                     f"{page:02d}/{len(figures):02d}  PNG SHA256 {row['image_sha256'][:16]}  QC {row['source_sha256'][:16]}")
        c.showPage()
        index.extend([f"## {row['figure_id']} {info['title_zh']}", '',
                      f"- 问题：{caption}", f"- 选择与边界：{scope}",
                      f"- 质检：{row['visual_qc']}",
                      f"- [原图]({image})；[QC来源]({row['source_path']})", ''])
        tex.extend(['\\begin{figure}[tbp]', '  \\centering',
                    f"  \\includegraphics[width=\\linewidth]{{{image.relative_to(REPO)}}}",
                    f"  \\caption{{{info['title_zh']}。{caption}}}",
                    f"  \\label{{fig:astra-{row['figure_id'].lower()}}}",
                    '\\end{figure}', ''])
        records.append({'figure_id': row['figure_id'], 'page': page,
                        'title_zh': info['title_zh'], 'image': binding(image),
                        'qc_source': binding(resolve_bound_path(row['source_path'])), 'rendered_size_pt': [draw_w, draw_h]})
    c.save()
    (output/'FIGURE_INDEX.md').write_text('\n'.join(index), encoding='utf-8')
    (output/'latex_figures.tex').write_text('\n'.join(tex), encoding='utf-8')
    result = {'schema': 'astra_working_figure_atlas_v1', 'status': 'generated_pending_contact_sheet_review',
              'figure_count': len(records), 'final_contract_30_groups_met': len(records) >= 30,
              'source_manifest': binding(manifest_path), 'source_plan': binding(plan_path),
              'font': {'name': FONT_NAME, 'kind': 'ReportLab UnicodeCIDFont'},
              'builder': binding(__file__), 'figures': records,
              'outputs': {p.name: binding(p) for p in (atlas, output/'FIGURE_INDEX.md', output/'latex_figures.tex')}}
    (output/'atlas_manifest.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'pages': len(records), 'atlas': str(atlas), 'sha256': sha(atlas)}))


if __name__ == '__main__':
    main()
