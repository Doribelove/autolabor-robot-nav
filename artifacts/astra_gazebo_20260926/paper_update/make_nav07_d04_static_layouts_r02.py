#!/usr/bin/env python3
"""Draw the four N06 development geometries from frozen D04 manifests.

This is a source-derived layout diagram, not a Gazebo screenshot or a driven
trajectory. The common route is the frozen geometric guide only.
"""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

S = Path(__file__).resolve().parents[1]
SOURCE = S / 'scenes/development_d04_geometry_r05'
REVIEW = S / 'audits/development_geometry_d04_r05_independent/review.json'
REVIEW_SHA = 'a40a0f4f914347e83c57d033a0d20525d79a4811d8d8cca379b28c6d3f7f30a4'
OUT = S / 'paper_update/figures/NAV07_d04_static_layouts_r02'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def binding(path):
    return {'path': str(path.resolve()), 'sha256': sha(path)}


def main():
    if sha(REVIEW) != REVIEW_SHA:
        raise ValueError('D04 independent review changed')
    review = json.loads(REVIEW.read_text())
    if review['status'] != 'independent_offline_joint_split_dedup_and_four_static_raster_review_pass':
        raise ValueError('D04 static source not independently reviewed')
    if OUT.exists():
        raise FileExistsError('preserve previous NAV07 version')
    OUT.mkdir(parents=True)
    fig, axs = plt.subplots(2, 2, figsize=(6.8, 5.45), sharex=True, sharey=True)
    fig.suptitle('N06 development: four new static layouts and common guides', fontsize=10.6, y=.992)
    rows = []
    sources = []
    for i, ax in enumerate(axs.flat):
        layout = f'd04r05_development_static_avoidance_{i:03d}'
        folder = SOURCE / layout
        manifest_path = folder / 'geometry_manifest.json'
        route_path = folder / 'raster_and_guide/common_bundle/common_route.json'
        checks_path = folder / 'raster_and_guide/raster_guide_checks.json'
        map_path = folder / 'raster_and_guide/common_bundle/map.pgm'
        manifest = json.loads(manifest_path.read_text())
        route = json.loads(route_path.read_text())
        checks = json.loads(checks_path.read_text())
        result = next(x for x in review['static_results'] if x['layout_id'] == layout)
        if (manifest['family'] != 'static_avoidance' or manifest['split'] != 'development'
                or manifest['actors'] or len(manifest['static_boxes']) != 1
                or checks['guide']['passed'] is not True
                or result['guide_conservative_lower_m'] <= 0):
            raise ValueError('unexpected static geometry or guide qualification')
        box = manifest['static_boxes'][0]
        if box['yaw_rad'] != 0:
            raise ValueError('this exact layout diagram assumes axis-aligned box')
        ax.set_facecolor('#fafafa')
        ax.axhline(3.2, color='#2b2b2b', lw=3.2, zorder=2)
        ax.axhline(-3.2, color='#2b2b2b', lw=3.2, zorder=2)
        ax.add_patch(Rectangle((box['x'] - box['length']/2, box['y'] - box['width']/2),
                               box['length'], box['width'], fc='#505963', ec='#263039', lw=.7, zorder=4))
        xs, ys = zip(*route['points_xy'])
        ax.plot(xs, ys, color='#d9822b', lw=1.8, zorder=3,
                label='shared geometric guide' if i == 0 else None)
        ax.scatter([-5.4], [0], marker='s', s=20, color='#159a87', zorder=5,
                   label='robot spawn' if i == 0 else None)
        ax.scatter([-4.0], [0], marker='o', s=16, facecolor='white', edgecolor='#2c465b', lw=1, zorder=5,
                   label='route start' if i == 0 else None)
        ax.scatter([5.0], [0], marker='*', s=60, color='#5753a6', zorder=5,
                   label='goal' if i == 0 else None)
        ax.set(xlim=(-6, 6), ylim=(-3.5, 3.5), aspect='equal',
               title=f'({chr(97+i)}) Layout {i:03d} | {box["length"]:.2f} × {box["width"]:.2f} m block')
        ax.grid(color='#dae1e5', lw=.35, zorder=0)
        ax.tick_params(labelsize=7, length=2)
        if i >= 2:
            ax.set_xlabel('World x (m)', fontsize=8)
        if i % 2 == 0:
            ax.set_ylabel('World y (m)', fontsize=8)
        rows.append({'layout_id': layout, 'parent_geometry_id': manifest['parent_geometry_id'],
                     'block_length_m': box['length'], 'block_width_m': box['width'],
                     'block_x_m': box['x'], 'block_y_m': box['y'],
                     'guide_min_conservative_gap_m': result['guide_conservative_lower_m'],
                     'guide_point_count': len(route['points_xy'])})
        sources += [binding(p) for p in (manifest_path, route_path, checks_path, map_path)]
    handles, labels = axs.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, ncol=4, loc='lower center', bbox_to_anchor=(.5, .105), fontsize=7,
               frameon=False, columnspacing=1.25, handlelength=1.8)
    fig.text(.5, .027,
             'Source-derived D04 map geometry and A* guide; no robot trajectory, Ackermann-turn or success claim.',
             ha='center', va='center', fontsize=6.8, color='#5b3d37')
    fig.subplots_adjust(left=.095, right=.99, top=.93, bottom=.225, wspace=.20, hspace=.35)
    basename = 'NAV07_d04_static_layouts'
    for extension in ('png', 'pdf', 'svg'):
        fig.savefig(OUT / f'{basename}.{extension}', dpi=450, facecolor='white')
    plt.close(fig)
    table = OUT / 'NAV07_source_rows.csv'
    with table.open('x', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader(); writer.writerows(rows)
    result = {'figure_id': 'NAV07', 'status': 'generated_pending_actual_view_qc',
              'source_scope': 'Four new D04 development static layouts selected for N06; source-derived diagram only, no simulated outcomes.',
              'independent_review': binding(REVIEW), 'sources': sources, 'rows': rows,
              'files': {p.suffix.lstrip('.'): binding(p) for p in OUT.glob(f'{basename}.*')},
              'source_table': binding(table)}
    (OUT / 'metadata.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps({'figure_id': 'NAV07', 'rows': len(rows), 'output': str(OUT)}))


if __name__ == '__main__':
    main()
