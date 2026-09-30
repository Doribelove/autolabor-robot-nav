#!/usr/bin/env python3
"""Plot frozen M13 synthetic cProfile hotspots, without runtime claims."""
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

S = Path(__file__).resolve().parents[1]
SOURCE = S/'audits/route_tracks_profile_m13/profile_report.json'
OUT = S/'paper_update/figures/LAT01_m13_synthetic_profile_r06'


def binding(path):
    return {'path': str(path.resolve()), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    if OUT.exists():
        raise FileExistsError('preserve previous version')
    report = json.loads(SOURCE.read_text())
    assert report['schema'] == 'astra_m13_read_only_build_profile_v1'
    cases = report['cases']
    assert [x['name'] for x in cases] == ['sparse1440_retained2048', 'dense1440']
    names = [('build', 'Full build'), ('rollout', 'Candidate rollout'),
             ('verify_many', 'Safety verification'), ('project', 'Route projection')]
    values = {}
    for case in cases:
        vals = {}
        for function, _ in names:
            matches = [x for x in case['top_functions'] if x['function'] == function
                       and Path(x['file']).name == ('route_candidate_core.py' if function == 'rollout' else 'route_candidate_core_m08.py')]
            assert len(matches) == 1
            vals[function] = matches[0]['cumulative_ms_profiled']
        values[case['name']] = vals
    OUT.mkdir(parents=True)
    csv_path = OUT/'values.csv'
    with csv_path.open('w', newline='') as f:
        w = csv.writer(f); w.writerow(['synthetic_case', 'function', 'cumulative_ms_profiled'])
        for case in cases:
            for function, _ in names: w.writerow([case['name'], function, format(values[case['name']][function], '.6f')])

    plt.rcParams.update({'font.family':'Noto Sans CJK JP', 'font.size':8.0,
                         'pdf.fonttype':42, 'svg.fonttype':'none'})
    fig, ax = plt.subplots(figsize=(6.7, 4.0), dpi=300)
    fig.subplots_adjust(left=.23, right=.97, top=.78, bottom=.36)
    colors = ['#257E95', '#D86A39']
    labels = ['Sparse: 2,048 tracks', 'Dense: 0 tracks']
    y = list(range(len(names)))[::-1]
    for j, case in enumerate(cases):
        xx = [values[case['name']][key] for key, _ in names]
        offset = .13 if j == 0 else -.13
        ax.scatter(xx, [k+offset for k in y], s=52, marker='o' if j==0 else 'D',
                   color=colors[j], edgecolor='white', linewidth=.65, label=labels[j], zorder=3)
        for x, yp in zip(xx, y):
            ax.text(x+.60, yp+offset, f'{x:.1f}', fontsize=7.0, va='center', color=colors[j])
    ax.set_yticks(y); ax.set_yticklabels([label for _, label in names]); ax.set_ylim(-.6, 3.6)
    ax.set_xlim(0, 49); ax.set_xlabel('cProfile cumulative time per one synthetic build (ms)')
    ax.grid(axis='x', color='#D9E0E5', linewidth=.7); ax.set_axisbelow(True)
    for spine in ('top','right','left'): ax.spines[spine].set_visible(False)
    ax.legend(loc='upper center', bbox_to_anchor=(.5,1.20), ncol=2, frameon=False, fontsize=7.8)
    fig.suptitle('M13 合成输入构建剖析：候选展开与验证是主要热点', x=.51, y=.94, fontsize=11, weight='bold')
    fig.text(.03,.17,'1440 input points · 491 queries · one profiled build per case', fontsize=7.2, color='#394A59')
    fig.text(.03,.105,'累积时间层级重叠，不能相加；cProfile 会增加开销。仅定位优化方向，不能判定运行期限。',
             fontsize=7.0, color='#874D3A')
    fig.text(.03,.035,'Frozen M13 report · no ROS/Gazebo, actor, training, or deadline qualification',
             fontsize=6.8, color='#596A74')
    for ext in ('png','pdf','svg'): fig.savefig(OUT/f'LAT01.{ext}', dpi=600 if ext=='png' else None)
    plt.close(fig)
    manifest = {'schema':'astra_lat01_frozen_source_manifest_v1','figure_id':'LAT01',
                'claim_scope':'one synthetic cProfile call per case; cumulative calls overlap; no runtime qualification',
                'source':binding(SOURCE),'generator':binding(Path(__file__)),
                'values':binding(csv_path),'images':{ext:binding(OUT/f'LAT01.{ext}') for ext in ('png','pdf','svg')}}
    (OUT/'source_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps({'manifest':str(OUT/'source_manifest.json'), 'png':str(OUT/'LAT01.png')}))


if __name__ == '__main__': main()
