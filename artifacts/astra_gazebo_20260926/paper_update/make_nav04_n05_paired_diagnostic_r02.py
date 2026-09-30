"""N05 paired development endpoints and overlapping trace-message rates."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

S=Path(__file__).resolve().parents[1]
SOURCE=S/'audits/navigation_n05_result_interpretation/report.json'
REPLAY=S/'audits/navigation_n05_terminal_replay/report.json'
OUT=S/'paper_update/figures/NAV04_n05_paired_diagnostic_r02'
C='astra_analytic_route_v1'
P='astra_analytic_preview_eligible_n05'


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUT.exists():raise FileExistsError('NAV04 figure identity exists')
    d=json.loads(SOURCE.read_bytes());replay=json.loads(REPLAY.read_bytes())
    assert d['replay_report_sha256']==sha(REPLAY) and d['status']=='negative_preview_selection_development_diagnostic_no_promotion'
    rows={(r['seed'],r['method']):r for r in replay['rows']}
    pairs=sorted(replay['pairs'],key=lambda r:r['seed']);assert len(pairs)==12
    control=np.array([rows[(p['seed'],C)]['terminal_map_goal_xy_distance_m'] for p in pairs])
    preview=np.array([rows[(p['seed'],P)]['terminal_map_goal_xy_distance_m'] for p in pairs])
    assert np.all(np.isfinite(control)) and np.all(np.isfinite(preview))
    assert abs(float(np.mean(preview-control))-d['paired_terminal_map_goal_distance_treatment_minus_control_m']['mean'])<1e-9
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':7.5,'axes.linewidth':.7,'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,(a,b)=plt.subplots(1,2,figsize=(6.7,3.55),gridspec_kw={'width_ratios':[1.18,1]},constrained_layout=True)
    x=np.arange(12)
    for i in (4,5):a.axvspan(i-.43,i+.43,color='#f5d9d3',alpha=.75,zorder=0)
    for i in range(12):a.plot([i-.14,i+.14],[control[i],preview[i]],color='#919ba6',linewidth=.9,zorder=1)
    a.scatter(x-.14,control,s=24,color='#176b89',label='Control analytic',zorder=3)
    a.scatter(x+.14,preview,s=24,color='#df8736',marker='D',label='Preview eligible',zorder=3)
    a.set_xticks(x,[f'{i:02d}' for i in range(12)],fontsize=6.5)
    a.set_xlim(-.55,11.55);a.set_ylim(-.1,9.2)
    a.set_xlabel('Paired block b00–b11');a.set_ylabel('Exact-terminal map goal distance (m)')
    a.grid(axis='y',color='#d8dee4',linewidth=.55);a.set_axisbelow(True)
    a.legend(loc='upper left',fontsize=6.5,frameon=False,ncol=1)
    a.set_title('(a) Paired terminal distance',loc='left',fontsize=8.5)
    names=['Executed stop','Backup revalidation failed','Backup stop requested','Transaction deadline drop']
    keys=['executed_stop_ticks','backup_revalidation_failed_ticks','backup_stop_requested_ticks','transaction_deadline_drop_ticks']
    totals=d['method_summaries']
    c=np.array([totals[C]['totals'][key]/totals[C]['totals']['trace_ticks']*100 for key in keys])
    p=np.array([totals[P]['totals'][key]/totals[P]['totals']['trace_ticks']*100 for key in keys])
    pos=np.arange(4);w=.32
    b.bar(pos-w/2,c,w,color='#176b89',label='Control')
    b.bar(pos+w/2,p,w,color='#df8736',label='Preview')
    b.set_xticks(pos,['Stop','Revalidation\nfailed','Backup stop\nrequested','Deadline\ndrop'],fontsize=6.5)
    b.set_ylim(0,100);b.set_ylabel('Share of trace messages (%)')
    b.grid(axis='y',color='#d8dee4',linewidth=.55);b.set_axisbelow(True)
    b.legend(loc='upper right',fontsize=6.5,frameon=False)
    b.set_title('(b) Overlapping event rates',loc='left',fontsize=8.5)
    fig.suptitle('N05: preview eligibility in 12 paired development blocks',fontsize=9.5)
    fig.text(.5,-.045,'b04/b05 shaded: both methods fail conservative clearance; trace rates are not durations.',
             ha='center',fontsize=6.5,color='#622c2c')
    OUT.mkdir(parents=True)
    stem=OUT/'NAV04_n05_paired_diagnostic'
    for suffix,kwargs in (('png',{'dpi':450}),('pdf',{}),('svg',{})):
        fig.savefig(stem.with_suffix('.'+suffix),facecolor='white',bbox_inches='tight',**kwargs)
    plt.close(fig)
    metadata=dict(figure_id='NAV04',status='generated_pending_independent_review_and_visual_qc',
        interpretation_sha256=sha(SOURCE),replay_sha256=sha(REPLAY),blocks=12,
        scope='Six reused development parent geometries, two paired seeds each; 24 valid one-shot episodes. Endpoint distance at each own terminal time. Four b04/b05 episodes fail conservative scoped-safety clearance; neither method passes all safety. Counts overlap and are message fractions, not time. No performance qualification.',
        files={suffix:dict(path=str(stem.with_suffix('.'+suffix)),sha256=sha(stem.with_suffix('.'+suffix))) for suffix in ('png','pdf','svg')})
    (OUT/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(dict(figure_id='NAV04',source_sha256=sha(SOURCE),blocks=12)))


if __name__=='__main__':main()
