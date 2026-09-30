"""One retained development failure, with separate logged source clocks.

No simulation, interpolation of execution, or counterfactual claim.
"""
from pathlib import Path
import csv
import hashlib
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

S=Path(__file__).resolve().parents[3]
RUN='astra_navigation_n04_b00_astra_analytic_route_v1'
R=S/'runs'/RUN
D=S/'audits/navigation_n04_first_analytic_trace_diagnosis'
OUT=Path(__file__).resolve().parent/'r01'
OUT.mkdir(exist_ok=False)

def binding(p):
    return dict(path=str(p.resolve()),sha256=hashlib.sha256(p.read_bytes()).hexdigest())

diagnosis=json.loads((D/'diagnosis.json').read_bytes())
for row in diagnosis['sources']:
    assert hashlib.sha256(Path(row['path']).read_bytes()).hexdigest()==row['sha256']
traces=json.loads((D/'trace.json').read_bytes())
candidates=json.loads((D/'candidates.json').read_bytes())
raw=json.loads((R/'collector_raw.json').read_bytes())
assessment=json.loads((R/'assessment.json').read_bytes())
assert assessment['capture_integrity_pass'] is True
assert assessment['task']['task_phase']=='timeout' and assessment['restricted_goal_time_s']==22.0
gate=raw['start_ns']
times=[(x['stamp_ns']-gate)*1e-9 for x in traces]
decision_times=[(x['stamp_ns']-gate)*1e-9 for x in candidates]
assert all(x['prediction']['dynamic_count']==0 for x in candidates)
trace_fields=['publication_elapsed_s','publication_source_ns','state_source_ns','fresh_scan_ns','proposed','chosen','executed','requested_v_mps','requested_w_radps','stop_reason','backup_prediction_clearance_m','transaction_ms','transaction_deadline_dropped']
with (OUT/'control_trace.csv').open('x',newline='') as f:
    w=csv.DictWriter(f,fieldnames=trace_fields);w.writeheader()
    for t,row in zip(times,traces):
        w.writerow(dict(publication_elapsed_s=t,publication_source_ns=row['stamp_ns'],state_source_ns=row['state_source_ns'],fresh_scan_ns=row['fresh_scan_ns'],proposed=row['proposed'],chosen=row['chosen'],executed=row['executed'],requested_v_mps=row['v'],requested_w_radps=row['w'],stop_reason=row['stop_reason'],backup_prediction_clearance_m=row['backup_prediction_clearance_m'],transaction_ms=row['transaction_ms'],transaction_deadline_dropped=row['transaction_deadline_dropped']))
with (OUT/'candidate_trace.csv').open('x',newline='') as f:
    w=csv.writer(f);w.writerow(['decision_elapsed_s','decision_source_ns','valid_backup_count','dynamic_return_count'])
    for t,row in zip(decision_times,candidates):w.writerow([t,row['stamp_ns'],sum(row['mask']),row['prediction']['dynamic_count']])

plt.rcParams.update({'font.family':'DejaVu Sans','font.size':8,'axes.labelsize':8,
                     'axes.titlesize':8.5,'pdf.fonttype':42,'ps.fonttype':42,
                     'axes.spines.top':False,'axes.spines.right':False,
                     'xtick.labelsize':7,'ytick.labelsize':7})
fig,axes=plt.subplots(4,1,figsize=(170/25.4,165/25.4),sharex=True,
                      gridspec_kw={'height_ratios':[1,1,1.15,1.15]})
fig.subplots_adjust(left=.125,right=.985,bottom=.095,top=.885,hspace=.40)
fig.suptitle('Static obstacle: preserved analytic timeout',x=.125,ha='left',y=.975,fontsize=11,fontweight='bold')
fig.text(.125,.932,'One development episode  |  22 s cap  |  logs shown at their recorded source times',fontsize=7.8)

ax=axes[0]
ax.step(decision_times,[sum(x['mask']) for x in candidates],where='post',color='#246b8e',lw=1.25)
empty=[t for t,x in zip(decision_times,candidates) if not any(x['mask'])]
ax.scatter(empty,[0]*len(empty),s=9,color='#b23b40',zorder=3)
ax.set_ylim(-.5,8.8);ax.set_yticks([0,4,8]);ax.set_ylabel('Valid backup\ncandidates')
ax.set_title('(a) 41 of 88 decision records have an empty hard mask',loc='left',pad=4)

ax=axes[1]
ax.step(times,[x['v'] for x in traces],where='post',color='#263c54',lw=1)
ax.set_ylim(-.04,1.25);ax.set_yticks([0,.6,1.2]);ax.set_ylabel('Method request\nv (m/s)')
ax.set_title('(b) Logged velocity request; actual velocity is not inferred from it',loc='left',pad=4)

ax=axes[2]
valid=[(t,x['backup_prediction_clearance_m']) for t,x in zip(times,traces) if x['backup_prediction_clearance_m'] is not None]
ax.plot([x[0] for x in valid],[x[1] for x in valid],color='#246b8e',lw=1)
negative=[(t,v) for t,v in valid if v<0]
ax.scatter([x[0] for x in negative],[x[1] for x in negative],s=5,color='#b23b40',zorder=3)
ax.axhline(0,color='#5c6670',lw=.8,ls='--');ax.set_ylim(-.12,1.9)
ax.set_yticks([0,.5,1,1.5]);ax.set_ylabel('Backup predicted\nclearance (m)')
ax.set_title('(c) Prediction and brake feasibility metric; not measured physical clearance',loc='left',pad=4)

ax=axes[3]
ax.plot(times,[x['transaction_ms'] for x in traces],color='#8898a6',lw=.65)
drops=[(t,x['transaction_ms']) for t,x in zip(times,traces) if x['transaction_deadline_dropped']]
ax.scatter([x[0] for x in drops],[x[1] for x in drops],color='#b23b40',s=10,zorder=3,label='Deadline drop')
ax.axhline(50,color='#b23b40',lw=.9,ls='--',label='50 ms limit')
ax.set_ylim(0,max(65,max(x['transaction_ms'] for x in traces)+4))
ax.set_ylabel('Transaction\nwall time (ms)');ax.set_xlabel('Elapsed simulation source time from shared gate (s)')
ax.set_title('(d) 15 of 439 control records discard a late transaction',loc='left',pad=4)
ax.legend(loc='upper right',frameon=False,fontsize=6.8,ncol=2)
for ax in axes:
    ax.set_xlim(0,22.1);ax.set_xticks([0,4,8,12,16,20,22]);ax.grid(axis='y',color='#e7ebee',lw=.6)

for ext in ('pdf','svg','png'):
    fig.savefig(OUT/('NAV01_static_failure_trace.'+ext),dpi=400,facecolor='white')
fig.savefig(OUT/'NAV01_paper_size_preview.png',dpi=180,facecolor='white')
plt.close(fig)
caption=('A retained N04 development failure of the analytic route planner in one static-obstacle scene. '
         'Candidate records use decision source time; control records use trace publication source time, both relative to the shared gate. '
         'The hard mask is empty in 41/88 decisions. Logged control reasons include 206 backup revalidation failures, 111 requested backup stops and 15 transaction deadline drops. '
         'All 88 candidate records classify zero dynamic returns. Capture and engineering-safety checks pass, but the task times out and receives 22 s restricted time. '
         'Method requests and backup predictions are not measured motion or physical clearance. This trace does not establish the counterfactual contribution of any cause to the timeout.')
(OUT/'caption.txt').write_text(caption+'\n')
inputs=[D/'diagnosis.json',D/'trace.json',D/'candidates.json',R/'collector_raw.json',R/'assessment.json',Path(__file__)]
outputs=[p for p in OUT.iterdir() if p.is_file()]
report=dict(figure_id='NAV01',status='generated_pending_actual_visual_and_independent_data_review',run_id=RUN,
            selection='first chronological closed analytic development failure; explanatory diagnostic, not performance-representative sample',
            paper_size_mm=[170,165],gate_source_ns=gate,trace_rows=len(traces),candidate_rows=len(candidates),
            mask_empty_count=len(empty),deadline_drop_count=len(drops),actual_sensor_dynamic_returns_all_zero=True,
            no_execution_motion_inferred=True,source_time_conventions='candidate decision vs control publication',
            sources=[binding(p) for p in inputs],outputs=[binding(p) for p in outputs],figure_group_count=1)
(OUT/'manifest.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(output=str(OUT),rows=[len(traces),len(candidates)],figure_group='NAV01')))
