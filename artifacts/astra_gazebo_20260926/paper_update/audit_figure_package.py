#!/usr/bin/env python3
"""Recheck the accepted Astra figure package without upgrading raw frames."""

import argparse
import collections
import csv
import hashlib
import json
from pathlib import Path


STAGE=Path(__file__).resolve().parents[1]
REPO=STAGE.parent.parent
SCREENSHOT_GROUPS=('Q01','NAV05','F06','F07','F08','F09','F10','F11')


def path(value):
    p=Path(value)
    return p if p.is_absolute() else REPO/p


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def binding(p):
    p=Path(p).resolve(strict=True)
    return {'path':str(p),'sha256':sha(p)}


def selected_native_refs(row):
    value=row['raw_image_path']
    try:
        result=json.loads(value)
    except json.JSONDecodeError:
        result=value.split(';')
    if isinstance(result,str):result=[result]
    if not isinstance(result,list) or not all(isinstance(x,str) for x in result):
        raise ValueError('invalid selected native frame listing')
    return result


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',type=Path,required=True)
    args=ap.parse_args()
    out=args.output.resolve()
    if out.exists():raise FileExistsError('preserve prior package audit')
    with (STAGE/'figure_manifest.csv').open(newline='') as f:rows=list(csv.DictReader(f))
    with (STAGE/'figure_plan.csv').open(newline='') as f:plans={x['figure_id']:x for x in csv.DictReader(f)}
    raw=[x for x in rows if x['figure_id'].startswith('RAW-')]
    figures=[x for x in rows if not x['figure_id'].startswith('RAW-')]
    if len(figures)!=30 or len({x['figure_id'] for x in figures})!=30:
        raise ValueError('exactly 30 unique accepted groups expected')
    if len({x['image_sha256'] for x in figures})!=30:
        raise ValueError('duplicate accepted PNG bytes')
    accepted=[]
    for row in figures:
        fig_id=row['figure_id']
        if fig_id not in plans or not plans[fig_id]['status'].startswith('accepted_distinct_group'):
            raise ValueError('unaccepted figure: '+fig_id)
        if (any(not row[k] for k in ('machine_qc','visual_qc','final_size_qc'))
                or 'pass' not in row['visual_qc'].lower()):
            raise ValueError('missing documented three-layer QC: '+fig_id)
        source=path(row['source_path']).resolve(strict=True)
        png=path(row['image_path']).resolve(strict=True)
        if sha(source)!=row['source_sha256'] or sha(png)!=row['image_sha256']:
            raise ValueError('source or PNG hash differs: '+fig_id)
        pdf=png.with_suffix('.pdf');svg=png.with_suffix('.svg')
        if not pdf.is_file() or not svg.is_file():
            raise ValueError('PDF/SVG absent: '+fig_id)
        accepted.append({'figure_id':fig_id,'source':binding(source),
                         'png':binding(png),'pdf':binding(pdf),'svg':binding(svg),
                         'chapter':row['chapter']})
    native=[]
    for fig_id in SCREENSHOT_GROUPS:
        row=next(x for x in figures if x['figure_id']==fig_id)
        for frame in selected_native_refs(row):
            native.append(binding(path(frame)))
    unique_native={x['path']:x for x in native}
    if len(unique_native)!=13:raise ValueError('selected native frame count changed')
    atlas=STAGE/'paper_update/working_atlas_r12'
    atlas_manifest=json.loads((atlas/'atlas_manifest.json').read_text())
    if atlas_manifest['figure_count']!=30 or [x['figure_id'] for x in atlas_manifest['figures']]!=[x['figure_id'] for x in figures]:
        raise ValueError('atlas does not match accepted order')
    report={'schema':'astra_figure_package_audit_v1','status':'figure_count_and_bound_artifacts_pass_research_incomplete',
            'figure_count':30,'selected_screenshot_group_ids':list(SCREENSHOT_GROUPS),
            'selected_screenshot_group_count':len(SCREENSHOT_GROUPS),
            'selected_native_frame_references':len(native),
            'selected_unique_native_frames':len(unique_native),
            'six_core_family_native_panels':['F06','F07','F08','F09','F10','F11'],
            'raw_catalog_entry_count':len(raw),
            'raw_catalog_machine_qc_labels':dict(sorted(collections.Counter(x['machine_qc'] for x in raw).items())),
            'raw_catalog_visual_qc_labels':dict(sorted(collections.Counter(x['visual_qc'] for x in raw).items())),
            'figures':accepted,'selected_native_frames':list(unique_native.values()),
            'figure_manifest':binding(STAGE/'figure_manifest.csv'),
            'figure_plan':binding(STAGE/'figure_plan.csv'),
            'atlas_pdf':binding(atlas/'FIGURE_ATLAS.pdf'),
            'atlas_index':binding(atlas/'FIGURE_INDEX.md'),
            'atlas_latex':binding(atlas/'latex_figures.tex'),
            'atlas_actual_view_qc':binding(atlas/'actual_view_qc.json'),
            'generator':binding(Path(__file__)),
            'limits':['Three-layer selected-figure QC is documented, not a new independent scientific result.',
                      'Raw catalog entries with pending/failed QC are retained and are not promoted.',
                      'Development figures do not establish training, formal performance, general safety, shadow or hardware validation.']}
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'figure_count':30,'screenshot_groups':len(SCREENSHOT_GROUPS),
                      'unique_native_frames':len(unique_native),'raw_catalog':len(raw),
                      'report_sha256':sha(out)}))


if __name__=='__main__':main()
