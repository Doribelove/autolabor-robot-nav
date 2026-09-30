#!/usr/bin/env python3
"""Plot independently reviewed D04 static geometry margins, not run outcomes."""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

S = Path(__file__).resolve().parents[1]
REVIEW = S/'audits/development_geometry_d04_r05_independent/review.json'
REVIEW_SHA = 'a40a0f4f914347e83c57d033a0d20525d79a4811d8d8cca379b28c6d3f7f30a4'
OUT = S/'paper_update/figures/NAV08_d04_static_geometry_margins_r02'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def binding(path):
    return {'path': str(path.resolve()), 'sha256': sha(path)}


def main():
    if sha(REVIEW) != REVIEW_SHA:
        raise ValueError('independent D04 review changed')
    review = json.loads(REVIEW.read_text())
    results = review['static_results']
    if len(results) != 4 or review['status'] != 'independent_offline_joint_split_dedup_and_four_static_raster_review_pass':
        raise ValueError('exact four reviewed static results required')
    rows = []
    for i, result in enumerate(results):
        if result['layout_id'] != f'd04r05_development_static_avoidance_{i:03d}':
            raise ValueError('static roster reordered')
        rows.append({'layout_id': result['layout_id'],
                     'original_native_witness_lower_m': result['native_witness_after_raster']['conservative_lower_m'],
                     'selected_raster_witness_lower_m': result['selected_raster_witness']['conservative_lower_m'],
                     'guide_min_full_footprint_gap_m': result['guide_conservative_lower_m'],
                     'guide_ackermann_corner_feasibility': result['guide_ackermann_corner_feasibility']})
    if not (rows[1]['original_native_witness_lower_m'] < 0
            and all(r['selected_raster_witness_lower_m'] > 0 and r['guide_min_full_footprint_gap_m'] > 0 for r in rows)):
        raise ValueError('D04 failure/selected margin pattern differs')
    if OUT.exists():
        raise FileExistsError('preserve previous NAV08 version')
    OUT.mkdir(parents=True)
    fig, ax = plt.subplots(figsize=(6.8, 3.55))
    x = np.arange(4)
    ax.axhline(0, ls='--', lw=1, color='#7b3434', zorder=1)
    ax.axvspan(.7, 1.3, color='#f6ded6', alpha=.8, zorder=0)
    ax.scatter(x-.14, [r['original_native_witness_lower_m'] for r in rows],
               s=40, marker='o', color='#1c6d89', label='Original native witness after rasterization', zorder=3)
    ax.scatter(x, [r['selected_raster_witness_lower_m'] for r in rows],
               s=43, marker='D', color='#da8029', label='Selected raster witness', zorder=3)
    ax.scatter(x+.14, [r['guide_min_full_footprint_gap_m'] for r in rows],
               s=45, marker='s', facecolor='white', edgecolor='#4f784d', lw=1.8,
               label='Geometric guide minimum', zorder=3)
    ax.annotate('retained negative original', xy=(.86, rows[1]['original_native_witness_lower_m']),
                xytext=(1.14, -.047), fontsize=7.4, ha='left',
                arrowprops={'arrowstyle': '->', 'lw': .7, 'color': '#7b3434'}, color='#7b3434')
    ax.set(xlim=(-.4, 3.4), ylim=(-.063, .148), xticks=x,
           xticklabels=[f'Layout {i:03d}' for i in range(4)],
           ylabel='Conservative lower clearance (m)',
           title='D04 static geometry checks for the N06 development scenes')
    ax.grid(axis='y', color='#d9e0e5', lw=.6, zorder=0)
    handles, labels = ax.get_legend_handles_labels()
    fig.legend(handles, labels, loc='lower center', bbox_to_anchor=(.5, .105),
               ncol=3, fontsize=7.1, frameon=False, columnspacing=1.1, handletextpad=.4)
    fig.text(.5, .026,
             'Offline geometry only; guide corners are not Ackermann-feasible-turn or physical tracking evidence.',
             ha='center', va='center', fontsize=6.9, color='#5b3d37')
    fig.subplots_adjust(left=.115, right=.98, top=.86, bottom=.28)
    for ext in ('png', 'pdf', 'svg'):
        fig.savefig(OUT/f'NAV08_d04_static_geometry_margins.{ext}', dpi=450, facecolor='white')
    plt.close(fig)
    csv_path = OUT/'NAV08_source_rows.csv'
    with csv_path.open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    meta = {'figure_id': 'NAV08', 'status': 'generated_pending_actual_view_qc',
            'claim_scope': 'Four finite D04 static development geometries under inherited conservative envelope; no dynamic, task, formal safety, deployment or real-vehicle claim.',
            'independent_review': binding(REVIEW), 'rows': rows,
            'files': {p.suffix.lstrip('.'): binding(p) for p in OUT.glob('NAV08_d04_static_geometry_margins.*')},
            'source_table': binding(csv_path)}
    (OUT/'metadata.json').write_text(json.dumps(meta, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'figure_id': 'NAV08', 'rows': len(rows), 'output': str(OUT)}))


if __name__ == '__main__':
    main()
