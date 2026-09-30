"""Failure-inclusive N05 swept-clearance engineering-gate figure."""
import csv
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
OUT=S/'paper_update/figures/NAV06_n05_clearance_gate_r01'
METHODS=('astra_analytic_route_v1','astra_analytic_preview_eligible_n05')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUT.exists():raise FileExistsError('NAV06 figure identity exists')
    replay=json.loads(REPLAY.read_bytes());review=json.loads(REVIEW.read_bytes())
    assert review['independent_review'] is True and review['reviewed_replay_report']['sha256']==sha(REPLAY)
    rows=[]
    for row in replay['rows']:
        root=S/'runs'/row['run_id'];a_path=root/'assessment.json';assert sha(a_path)==row['assessment_sha256']
        a=json.loads(a_path.read_bytes());s_path=Path(a['sweep_report']['path']);assert sha(s_path)==a['sweep_report']['sha256']
        sweep=json.loads(s_path.read_bytes());value=sweep['minimum_engineering_conservative_swept_xy_m']
        assert type(value) in (float,int) and np.isfinite(value)
        assert a['safety_checks']['positive_engineering_clearance'] is (value>0)
        assert a['contact_audit']['illegal_robot_contacts_count']==0
        rows.append(dict(run_id=row['run_id'],seed=row['seed'],method=row['method'],
            minimum_conservative_swept_xy_m=value,positive_engineering_clearance=value>0,
            illegal_robot_contacts=0,assessment_sha256=sha(a_path),sweep_report_sha256=sha(s_path)))
    rows.sort(key=lambda r:(r['seed'],METHODS.index(r['method'])))
    assert len(rows)==24 and sum(not r['positive_engineering_clearance'] for r in rows)==4
    by={(r['seed'],r['method']):r['minimum_conservative_swept_xy_m'] for r in rows}
    seeds=sorted({r['seed'] for r in rows});assert len(seeds)==12
    c=np.array([by[(s,METHODS[0])] for s in seeds]);p=np.array([by[(s,METHODS[1])] for s in seeds])
    plt.rcParams.update({'font.family':'DejaVu Sans','font.size':7.5,'axes.linewidth':.7,'pdf.fonttype':42,'svg.fonttype':'none'})
    fig,(a,b)=plt.subplots(1,2,figsize=(6.7,3.45),gridspec_kw={'width_ratios':[1.18,1]},constrained_layout=True)
    x=np.arange(12)
    for ax in (a,b):
        ax.axhline(0,color='#963d3d',linewidth=1.1,linestyle='--')
        ax.grid(axis='y',color='#d8dee4',linewidth=.5);ax.set_axisbelow(True)
    for i in (4,5):a.axvspan(i-.45,i+.45,color='#f6e0dc',zorder=0)
    a.scatter(x-.12,c,s=27,color='#176b89',label='Original analytic',zorder=3)
    a.scatter(x+.12,p,s=27,color='#df8736',marker='D',label='Preview eligible',zorder=3)
    a.set_xticks(x,[f'{i:02d}' for i in x],fontsize=6.5);a.set_xlim(-.5,11.5);a.set_ylim(-.35,3.65)
    a.set_ylabel('Minimum conservative swept XY clearance (m)');a.set_xlabel('Paired block b00–b11')
    a.set_title('(a) All paired development blocks',loc='left',fontsize=8.5)
    a.legend(loc='upper right',fontsize=6.5,frameon=False)
    subset=[0,1,4,5,8,9];j=np.arange(len(subset))
    for i in (2,3):b.axvspan(i-.45,i+.45,color='#f6e0dc',zorder=0)
    b.scatter(j-.12,c[subset],s=34,color='#176b89',zorder=3)
    b.scatter(j+.12,p[subset],s=34,color='#df8736',marker='D',zorder=3)
    b.set_xticks(j,[f'{i:02d}' for i in subset],fontsize=7);b.set_ylim(-.25,.28);b.set_xlim(-.5,5.5)
    b.set_xlabel('Near-boundary blocks');b.set_title('(b) Boundary detail',loc='left',fontsize=8.5)
    fig.suptitle('N05: no logged illegal contact does not clear the engineering margin',fontsize=9.5)
    fig.text(.5,-.045,'b04/b05: both methods fail the positive conservative-clearance gate; this is not a formal safety proof.',
             ha='center',fontsize=6.6,color='#653737')
    OUT.mkdir(parents=True)
    stem=OUT/'NAV06_n05_clearance_gate'
    for suffix,kwargs in (('png',{'dpi':450}),('pdf',{}),('svg',{})):
        fig.savefig(stem.with_suffix('.'+suffix),bbox_inches='tight',facecolor='white',**kwargs)
    plt.close(fig)
    csv_path=OUT/'NAV06_swept_clearance_rows.csv'
    with csv_path.open('x',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    metadata=dict(figure_id='NAV06',status='generated_pending_actual_view_qc',
        replay_sha256=sha(REPLAY),independent_review_sha256=sha(REVIEW),episodes=24,
        negative_conservative_clearance_episodes=4,all_illegal_robot_contacts=0,
        claim_scope='Frozen engineering swept XY bound on 24 N05 development episodes; a negative value fails the declared margin despite no logged illegal contacts. No formal or real-world safety claim.',
        sources=rows,files={suffix:dict(path=str(stem.with_suffix('.'+suffix)),sha256=sha(stem.with_suffix('.'+suffix))) for suffix in ('png','pdf','svg')},
        csv=dict(path=str(csv_path),sha256=sha(csv_path)))
    (OUT/'metadata.json').write_text(json.dumps(metadata,indent=2)+'\n')
    print(json.dumps(dict(figure='NAV06',episodes=24,negative=4)))


if __name__=='__main__':main()
