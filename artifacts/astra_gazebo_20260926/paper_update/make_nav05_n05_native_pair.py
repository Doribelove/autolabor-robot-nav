"""Compose four unaltered native Gazebo frames from the worst N05 endpoint pair."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

S=Path(__file__).resolve().parents[1]
REPLAY=S/'audits/navigation_n05_terminal_replay/report.json'
REVIEW=S/'audits/navigation_n05_terminal_independent/review.json'
OUT=S/'paper_update/figures/NAV05_n05_native_pair_r01'
METHODS=('astra_analytic_route_v1','astra_analytic_preview_eligible_n05')
IMAGE_NAMES=('gate_plus1s.png','near_terminal_at_or_after_source_minus500ms.png')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUT.exists():raise FileExistsError('NAV05 figure identity exists')
    replay=json.loads(REPLAY.read_bytes());review=json.loads(REVIEW.read_bytes())
    assert review['independent_review'] is True and review['reviewed_replay_report']['sha256']==sha(REPLAY)
    pairs=replay['pairs'];winner=max(pairs,key=lambda p:p['final_goal_distance_treatment_minus_control_m'])
    assert winner['seed']==263002007 and winner['control_run_id'].split('_b06_')[0]==winner['treatment_run_id'].split('_b06_')[0]
    endpoints={r['run_id']:r['terminal_map_goal_xy_distance_m'] for r in replay['rows']}
    selected=[]
    for method,run_id in zip(METHODS,(winner['control_run_id'],winner['treatment_run_id'])):
        root=S/'runs'/run_id;raw=json.loads((root/'collector_raw.json').read_bytes())
        receipt=json.loads((root/'receipt.json').read_bytes())
        assert receipt['integrity_pass'] is True and receipt['task_outcome']=='timeout'
        for name in IMAGE_NAMES:
            image=next(x for x in raw['images'] if Path(x['path']).name==name)
            path=Path(image['path']);assert sha(path)==image['sha256'] and (image['width'],image['height'])==(2560,1440)
            selected.append(dict(method=method,run_id=run_id,image_name=name,path=str(path),
                image_sha256=image['sha256'],source_ns=image['source_ns'],
                exact_terminal_source_ns=raw['end_ns'],
                exact_terminal_map_goal_distance_m=endpoints[run_id]))
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':7.5,'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,axes=plt.subplots(2,2,figsize=(6.7,4.78),constrained_layout=True)
    for row in range(2):
        for col in range(2):
            item=selected[2*col+row];ax=axes[row,col]
            ax.imshow(plt.imread(item['path']),interpolation='nearest')
            ax.set_xticks([]);ax.set_yticks([])
            for spine in ax.spines.values():spine.set_linewidth(.5);spine.set_color('#536574')
            method='Original analytic' if col==0 else 'Preview eligible'
            moment='Gate +1 s' if row==0 else 'Near terminal'
            ax.set_title(f'{method} · {moment} · image {item["source_ns"]/1e9:.2f} s',fontsize=8,pad=4)
            if row==1:
                ax.set_xlabel(f'Own exact-terminal map goal distance: {item["exact_terminal_map_goal_distance_m"]:.3f} m',fontsize=7.5,labelpad=4)
    fig.suptitle('N05 b06: largest observed paired endpoint gap in the 12 development blocks',fontsize=9.5)
    fig.text(.5,-.02,'Native Gazebo frames, same seed/scene/camera; image time differs from exact task-terminal time.\nBoth methods timeout; this selected failure case is not population evidence.',ha='center',fontsize=7,color='#5b3333')
    OUT.mkdir(parents=True)
    stem=OUT/'NAV05_n05_native_pair'
    for suffix,kwargs in (('png',{'dpi':450}),('pdf',{}),('svg',{})):
        fig.savefig(stem.with_suffix('.'+suffix),bbox_inches='tight',facecolor='white',**kwargs)
    plt.close(fig)
    meta=dict(figure_id='NAV05',status='generated_pending_actual_view_qc',selection_rule='maximum treatment-minus-control exact-terminal map goal distance among all 12 N05 development pairs; tie picks first schedule row',
        selected_seed=winner['seed'],pair_difference_m=winner['final_goal_distance_treatment_minus_control_m'],
        replay_sha256=sha(REPLAY),independent_review_sha256=sha(REVIEW),source_frames=selected,
        claim_scope='Two methods in one selected development failure pair; images near but not at exact own-terminal source time. Both timeout; no effect population, safety or 3D geometry claim.',
        files={suffix:dict(path=str(stem.with_suffix('.'+suffix)),sha256=sha(stem.with_suffix('.'+suffix))) for suffix in ('png','pdf','svg')})
    (OUT/'metadata.json').write_text(json.dumps(meta,indent=2)+'\n')
    print(json.dumps(dict(figure='NAV05',seed=winner['seed'],source_frames=len(selected),difference_m=winner['final_goal_distance_treatment_minus_control_m'])))


if __name__=='__main__':main()
