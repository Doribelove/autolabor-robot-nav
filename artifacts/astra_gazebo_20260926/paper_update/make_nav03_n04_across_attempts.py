"""Descriptive, failure-inclusive figure for five old valid N04 analytic attempts."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

S = Path(__file__).resolve().parents[1]
SOURCE = S/'audits/navigation_n04_trace_attribution_r02/report.json'
REVIEW = S/'audits/navigation_n04_trace_attribution_independent/review_r02.json'
OUT = S/'paper_update/figures/NAV03_n04_five_attempts_r01'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUT.exists():
        raise FileExistsError('NAV03 output identity already exists')
    review=json.loads(REVIEW.read_bytes()); data=json.loads(SOURCE.read_bytes())
    assert review['independent_review'] is True and review['pass_'] is True
    assert review['reviewed_report']['sha256']==sha(SOURCE)
    rows=data['rows']; assert len(rows)==5 and data['valid_analytic_attempts']==5
    ticks=np.array([r['restricted_half_open_trace_ticks'] for r in rows],dtype=float)
    assert np.all(ticks>0) and sum(ticks)==data['totals']['restricted_half_open_trace_ticks']
    rates={
        'Executed stop':np.array([r['restricted_half_open_executed_stop_ticks'] for r in rows])/ticks,
        'Backup revalidation failed':np.array([r['restricted_half_open_backup_revalidation_failed_ticks'] for r in rows])/ticks,
        'Backup stop requested':np.array([r['restricted_half_open_backup_stop_requested_ticks'] for r in rows])/ticks,
        'Transaction deadline drop':np.array([r['restricted_half_open_transaction_deadline_drop_ticks'] for r in rows])/ticks,
    }
    distance=np.array([r['terminal_map_goal_xy_distance_m'] for r in rows],dtype=float)
    assert np.all(np.isfinite(distance)) and np.all(distance>0)
    labels=[f"{r['seed']%1000:03d}" for r in rows]
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.linewidth':.7,
                         'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,(a,b)=plt.subplots(1,2,figsize=(6.70,3.25),gridspec_kw={'width_ratios':[1.23,1]},
                           constrained_layout=True)
    x=np.arange(len(rows));width=.19
    colors=['#176b89','#e48d33','#b44e5b','#5d7392']
    for i,(name,values) in enumerate(rates.items()):
        a.bar(x+(i-1.5)*width,values*100,width=.185,color=colors[i],label=name)
    a.set_ylim(0,100);a.set_xticks(x,labels);a.set_xlabel('Development seed suffix')
    a.set_ylabel('Share of trace ticks (%)');a.grid(axis='y',color='#d8dee4',linewidth=.55)
    a.set_axisbelow(True);a.set_title('(a) Distinct event rates',loc='left',fontsize=9)
    a.legend(fontsize=6.3,loc='upper right',ncol=2,frameon=False,columnspacing=.7,
             handlelength=1.1,handletextpad=.35)
    b.scatter(x,distance,s=55,color='#176b89',marker='D',zorder=3)
    for i,v in enumerate(distance):b.text(i,v+.13,f'{v:.2f}',ha='center',va='bottom',fontsize=7)
    b.set_xlim(-.5,4.5);b.set_ylim(0,9);b.set_xticks(x,labels)
    b.set_xlabel('Development seed suffix');b.set_ylabel('Terminal map-frame goal distance (m)')
    b.grid(axis='y',color='#d8dee4',linewidth=.55);b.set_axisbelow(True)
    b.set_title('(b) Exact-terminal remaining distance',loc='left',fontsize=9)
    fig.suptitle('N04 analytic static-route attempts: five valid timeouts',fontsize=10)
    OUT.mkdir(parents=True)
    stem=OUT/'NAV03_n04_five_attempts'
    fig.savefig(stem.with_suffix('.png'),dpi=450,facecolor='white')
    fig.savefig(stem.with_suffix('.pdf'),facecolor='white')
    fig.savefig(stem.with_suffix('.svg'),facecolor='white')
    plt.close(fig)
    metadata=dict(figure_id='NAV03',status='generated_pending_machine_and_visual_qc',
                  source_report_sha256=sha(SOURCE),source_independent_review_sha256=sha(REVIEW),
                  source_rows=5,source_trace_window='exact half-open shared gate [start,end)',
                  terminal_distance_frame='map, from exact-terminal odom via bound SE2 transform',
                  event_rates='separate overlapping rates, not stacked/exclusive/duration fractions',
                  failures='all five timeout; sixth consumed analytic assessment interrupted and excluded; six retired unrun excluded',
                  claim_scope='Old N04 development description only; no N05/N06 counterfactual or population performance inference',
                  files={suffix:dict(path=str(stem.with_suffix('.'+suffix)),sha256=sha(stem.with_suffix('.'+suffix))) for suffix in ('png','pdf','svg')})
    (OUT/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(dict(figure='NAV03',rows=5,source_sha256=sha(SOURCE))))


if __name__=='__main__':main()
