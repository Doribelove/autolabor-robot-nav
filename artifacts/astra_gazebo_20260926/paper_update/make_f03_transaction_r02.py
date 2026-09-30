#!/usr/bin/env python3
"""Draw the source-defined M08 candidate-to-command transaction only."""

import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch


STAGE = Path(__file__).resolve().parents[1]
CORE = STAGE / 'workspace/scripts/astra_gazebo/route_candidate_core_m08.py'
BASE = STAGE / 'workspace/scripts/astra_gazebo/route_candidate_core.py'
OUT = STAGE / 'paper_update/figures/F03_candidate_transaction_r02'


def binding(path):
    path = path.resolve(strict=True)
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def main():
    core, base = CORE.read_text(), BASE.read_text()
    for marker in ('prepublish_guard', 'self.verify_many(previews+backups', 'self.command(actual,scan,chosen,command_now)',
                   'certified_stop=False', 'physical_safety_certified=False', 'learned_candidate_conditioned_prediction_implemented=False'):
        assert marker in core
    for marker in ('inference_deadline_ms:float=20.', 'transaction_deadline_ms:float=50.'):
        assert marker in base
    if OUT.exists():
        raise FileExistsError('preserve figure version')
    OUT.mkdir(parents=True)
    plt.rcParams.update({'font.family': 'Noto Sans CJK JP', 'font.size': 7.2,
                         'pdf.fonttype': 42, 'svg.fonttype': 'none'})
    fig, ax = plt.subplots(figsize=(6.7, 4.1), dpi=300)
    fig.subplots_adjust(left=.025, right=.975, top=.865, bottom=.115)
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 72)
    ax.axis('off')
    ink = '#344653'
    palettes = {
        'input': ('#E6F1F5', '#337995'),
        'compute': ('#E8F0E8', '#56846F'),
        'gate': ('#FFF0E0', '#B77842'),
        'stop': ('#FAE8E6', '#AD554F'),
    }

    def box(x, y, w, h, title, sub, kind='compute'):
        face, edge = palettes[kind]
        ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle='round,pad=.3,rounding_size=1.2',
                                    facecolor=face, edgecolor=edge, linewidth=1.15))
        ax.text(x+w/2, y+h*.65, title, ha='center', va='center', fontsize=8, weight='bold', color=ink)
        ax.text(x+w/2, y+h*.28, sub, ha='center', va='center', fontsize=6.35, color=ink)

    def arrow(a, b, label='', color='#5B6C74'):
        ax.add_patch(FancyArrowPatch(a, b, arrowstyle='-|>', mutation_scale=10,
                                     linewidth=1.1, color=color))
        if label:
            ax.text((a[0]+b[0])/2, (a[1]+b[1])/2+1.3, label,
                    ha='center', va='bottom', fontsize=6.1, color=color)

    box(2, 49, 27, 14, '同步观测', 'LaserScan / odom / TF 源时间', 'input')
    box(36.5, 49, 27, 14, '构造候选与备份', '8 条预览 + 8 条备份轨迹')
    box(71, 49, 27, 14, '预测与硬净空门', '备份净空 → 8 位 action mask', 'gate')
    arrow((29.5, 56), (36, 56))
    arrow((64, 56), (70.5, 56))

    box(71, 24, 27, 14, '选择候选', '解析先验 / actor；20 ms 推理门')
    box(36.5, 24, 27, 14, '刷新并重新验证', '新源时间 + 所选备份净空', 'gate')
    box(2, 24, 27, 14, '发布前检查与执行', '50 ms 事务门；记录 executed', 'input')
    arrow((84.5, 48.5), (84.5, 38.5))
    arrow((70.5, 31), (64, 31))
    arrow((36, 31), (29.5, 31))

    box(23, 3, 54, 10, '失效路径：零命令与原因记录',
        '过期 / 时间回退 / 事务超时 / 备份净空失败 / 全掩码拒绝', 'stop')
    arrow((49.5, 23.5), (49.5, 13.5), '拒绝', '#AD554F')
    ax.text(50, 43.5, '审计维度：proposed → chosen → executed；源时间 / 净空 / 时延',
            ha='center', fontsize=6.35, color='#566972')
    fig.suptitle('F03 候选验证与命令执行事务（源码示意）', y=.967,
                 fontsize=11, weight='bold')
    fig.text(.5, .035,
             'M08/N05 开发源码流程；learned candidate-conditioned prediction=false，'
             'certified stop=false，physical safety certified=false。',
             ha='center', fontsize=6.35, color='#86503F')
    for ext in ('png', 'pdf', 'svg'):
        fig.savefig(OUT / ('F03.'+ext), dpi=600 if ext == 'png' else None)
    plt.close(fig)
    manifest = {
        'schema': 'astra_f03_transaction_source_manifest_v1',
        'figure_id': 'F03',
        'claim_scope': 'source-defined development transaction schematic only; no live deadline/safety qualification',
        'sources': [binding(CORE), binding(BASE)],
        'generator': binding(Path(__file__)),
        'images': {ext: binding(OUT/('F03.'+ext)) for ext in ('png', 'pdf', 'svg')},
    }
    (OUT/'source_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n')
    print(OUT)


if __name__ == '__main__':
    main()
