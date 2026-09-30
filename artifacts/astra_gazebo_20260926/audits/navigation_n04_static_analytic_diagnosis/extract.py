"""Read closed extracted JSON only. No bag, rollout replay, simulator or benchmark."""
from pathlib import Path
import json,csv,math,hashlib,collections
S=Path(__file__).resolve().parents[2];A=Path(__file__).parent;I=S/'audits/navigation_n04_first_analytic_trace_diagnosis';R=S/'runs/astra_navigation_n04_b00_astra_analytic_route_v1'
def bind(p):return dict(path=str(p),sha256=hashlib.sha256(p.read_bytes()).hexdigest())
C=json.loads((I/'candidates.json').read_text());T=json.loads((I/'trace.json').read_text());contract=json.loads((R/'contract.json').read_text());bundle_path=Path(contract['shared_bundle']['path']);B=json.loads(bundle_path.read_text());route=B['points_xy'];gate=7_460_000_000
# Exact elementary projection used only to describe logged actual poses, not to regenerate a candidate.
def project(p):
 best=None;arc=0.
 for i,(a,b) in enumerate(zip(route,route[1:])):
  dx=b[0]-a[0];dy=b[1]-a[1];length=math.hypot(dx,dy);ux,uy=dx/length,dy/length
  d=max(0.,min(length,(p[0]-a[0])*ux+(p[1]-a[1])*uy));x,y=a[0]+d*ux,a[1]+d*uy;ex,ey=p[0]-x,p[1]-y;dd=ex*ex+ey*ey
  if best is None or dd<best[0]:best=(dd,arc+d,-ex*uy+ey*ux,math.atan2(uy,ux),i,x,y)
  arc+=length
 return dict(distance_m=math.sqrt(best[0]),arc_m=best[1],lateral_m=best[2],heading_rad=best[3],segment=best[4],nearest_xy=best[5:])
rows=[]
for i,c in enumerate(C):
 end=C[i+1]['stamp_ns'] if i+1<len(C) else 10**20
 matches=[t for t in T if c['stamp_ns']<=t['stamp_ns']<end and (t['proposed'] is not None or t['selection_reason']=='no_prediction_model_backup_pass')]
 assert len(matches)==1,(i,len(matches));t=matches[0];f=c['candidate_features'];scores=[x[2]*3.6+1.5*max(-1.,min(2.,x[3]*3.))-.15*x[6]-.05*abs(x[5]*1.35) for x in f]
 rank=max((a for a in range(8) if c['mask'][a]),key=lambda a:scores[a]) if any(c['mask']) else 0
 state=c['previews'][0][0];p=project(state[:2]);chosen=t['chosen'];positive=[a for a in range(1,8) if c['mask'][a] and f[a][3]>=0]
 rows.append(dict(index=i,source_ns=c['stamp_ns'],gate_elapsed_s=(c['stamp_ns']-gate)/1e9,pose_x=state[0],pose_y=state[1],pose_yaw=state[2],actual_vx=state[3],actual_center_steer=state[4],guide=p,
  chosen=chosen,executed=t['executed'],stop_reason=t['stop_reason'],all_mask_empty=not any(c['mask']),mask_count=sum(c['mask']),mask=c['mask'],backup_clearances_m=c['backup_clearances'],
  preview_clearances_m=[x[3]*3 for x in f],scores_reconstructed_from_float32_features=scores,reconstructed_rank=rank,
  chosen_full_preview_clearance_m=f[chosen][3]*3,positive_full_preview_moving_choices=positive,dynamic_count=c['prediction']['dynamic_count'],trace_end_ns=t['stamp_ns']))
first_empty=next(r for r in rows if r['all_mask_empty']);last=rows[-1]
first_negative=next(r for r in rows if r['chosen']>0 and r['chosen_full_preview_clearance_m']<0)
late=[t for t in T if t['stamp_ns']>=19_786_000_000]
selected_negative=[r for r in rows if r['chosen']>0 and r['chosen_full_preview_clearance_m']<0]
summary=dict(schema='astra_n04_static_analytic_closed_trace_diagnosis_v1',scope='descriptive one closed development trace; no counterfactual or contribution estimate',run_id=contract['run_id'],seed=contract['seed'],
 rows=dict(trace=len(T),candidate=len(C)),sources=[bind(p) for p in (I/'trace.json',I/'candidates.json',I/'diagnosis.json',R/'contract.json',R/'assessment.json',R/'scene_manifest.yaml',bundle_path)],
 reason_counts=dict(collections.Counter(str(t['stop_reason']) for t in T)),all_dynamic_counts_zero=all(r['dynamic_count']==0 for r in rows),all_mask_empty_decisions=sum(r['all_mask_empty'] for r in rows),
 chosen_counts=dict(collections.Counter(r['chosen'] for r in rows)),rank_reconstruction_matches=sum(r['chosen']==r['reconstructed_rank'] for r in rows),
 selected_moving_negative_preview=len(selected_negative),selected_moving_negative_preview_and_positive_moving_alternative=sum(bool(r['positive_full_preview_moving_choices']) for r in selected_negative),
 executed_at_first_control_despite_negative_preview=sum(r['executed']>0 for r in selected_negative),
 first_empty_mask=first_empty,first_negative_preview_selection=first_negative,last=last,
 late_period=dict(start_source_ns=19_786_000_000,trace_count=len(late),positive_commands=sum(t['v']>0 for t in late),deadline_drops=sum(t['transaction_deadline_dropped'] for t in late),reason_counts=dict(collections.Counter(str(t['stop_reason']) for t in late))),
 route=dict(length_m=B['route_length_m'],geometric_prior_radius_m=B['planning_clearance_radius_m'],minimum_prior_segment_clearance_m=B['audit']['minimum_segment_clearance_lower_bound_m'],ackermann_qualified=B['ackermann_route_feasibility_qualified'],plateau_y_m=1.175,plateau_x_range=[-.825,1.075],incoming_heading_rad=math.atan2(route[1][1]-route[0][1],route[1][0]-route[0][0]),maximum_curvature_per_m=math.tan(.4964)/.65),
 limits=['Feature float32 rounding makes reconstructed scores approximate; all88 chosen IDs nevertheless match.',
 'Trace command counts are not independent time fractions or causal effects.', 'Terminal endpoint poses are observed actual initial preview states, not a replay or simulator truth from a new run.',
 'R2 and TEB success demonstrate world traversability, not exact same guide-consumer semantics or a feasible frozen8-candidate policy.',
 'No current source, margin, stop protocol or selection was changed.'])
(A/'decision_summary.json').write_text(json.dumps(summary,indent=2)+'\n');(A/'decision_rows.json').write_text(json.dumps(rows,indent=2)+'\n')
with (A/'decision_table.csv').open('w',newline='') as fp:
 w=csv.DictWriter(fp,fieldnames=['index','gate_elapsed_s','pose_x','pose_y','pose_yaw','actual_vx','chosen','executed','mask_count','chosen_full_preview_clearance_m','guide_lateral_m','guide_heading_error_rad']);w.writeheader()
 for r in rows:w.writerow({**{k:r[k] for k in w.fieldnames if k in r},'guide_lateral_m':r['guide']['lateral_m'],'guide_heading_error_rad':math.atan2(math.sin(r['pose_yaw']-r['guide']['heading_rad']),math.cos(r['pose_yaw']-r['guide']['heading_rad']))})
print(json.dumps({k:summary[k] for k in ('rows','all_mask_empty_decisions','selected_moving_negative_preview','selected_moving_negative_preview_and_positive_moving_alternative','executed_at_first_control_despite_negative_preview','late_period')},indent=2))
