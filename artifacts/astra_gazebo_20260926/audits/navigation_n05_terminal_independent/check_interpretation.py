"""Independent aggregation of our all-bag rows and saved negative sweep outputs."""
from pathlib import Path
import csv
import hashlib
import json
import math
import statistics

A=Path(__file__).resolve().parent;S=A.parents[1];resources={}
def bind(p):
    p=Path(p).resolve();h=hashlib.sha256()
    with p.open('rb') as f:
        for data in iter(lambda:f.read(4*1024*1024),b''):h.update(data)
    row={'path':str(p),'sha256':h.hexdigest(),'bytes':p.stat().st_size};resources[str(p)]=row;return row
def read(p):bind(p);return json.loads(Path(p).read_bytes())
def bound(row):assert bind(row['path'])['sha256']==row['sha256'];return Path(row['path'])
def close(a,b):assert math.isclose(a,b,rel_tol=0.,abs_tol=1e-12),(a,b)
ours=read(A/'recomputed.json');parent=read(S/'audits/navigation_n05_result_interpretation/report.json')
bind(S/'audits/navigation_n05_interpretation.py')
assert parent['release_sha256']==ours['release']['sha256'] and parent['ledger_sha256']==ours['ledger']['sha256']
assert parent['replay_report_sha256']==bind(S/'audits/navigation_n05_terminal_replay/report.json')['sha256']
assert parent['replay_script_sha256']==bind(S/'audits/navigation_n05_terminal_replay_r04.py')['sha256']
shell=read(S/'queue/navigation_n05_outer_shell_observation.json')
assert parent['outer_shell_observation_sha256']==bind(S/'queue/navigation_n05_outer_shell_observation.json')['sha256']
assert shell['tool_session_id']==1158 and shell['parent_tool_write_stdin_exit_code']==143
assert shell['manager_returned_code']==0 and shell['terminal_ledger_status']=='terminal'
assert not Path(shell['expected_outer_exit_file']).exists() and shell['expected_outer_exit_file_exists'] is False
assert bind(shell['manager_return_path'])['sha256']==shell['manager_return_sha256']
assert bind(shell['terminal_ledger_path'])['sha256']==shell['terminal_ledger_sha256']
results={};failures=[];illegal=0;sweep_aggregations=[]
for method,summary in parent['method_summaries'].items():
    rows=[r for r in ours['rows'] if r['method']==method]
    totals={key:sum(row[key] for row in rows) for key in summary['totals']}
    assert totals==summary['totals']
    assert len(rows)==summary['valid_rows']==12
    assert sum(r['task_phase']=='complete' for r in rows)==summary['complete']==2
    assert sum(r['task_phase']=='timeout' for r in rows)==summary['timeout']==10
    assert sum(r['safety_within_declared_engineering_scope_pass'] for r in rows)==summary['scoped_safety_pass']==10
    mean=statistics.mean(r['terminal_map_goal_xy_distance_m'] for r in rows);median=statistics.median(r['terminal_map_goal_xy_distance_m'] for r in rows)
    close(mean,summary['mean_terminal_map_goal_distance_m']);close(median,summary['median_terminal_map_goal_distance_m'])
    close(totals['executed_stop_ticks']/totals['trace_ticks'],summary['executed_stop_trace_message_fraction'])
    results[method]={'totals':totals,'mean_endpoint_m':mean,'median_endpoint_m':median}
    for row in rows:
        assessment=read(S/'runs'/row['run_id']/'assessment.json');sweep=read(bound(assessment['sweep_report']))
        minimum=min(x['conditional_clearance_lower_bound_m'] for x in sweep['coarse_intervals'])
        close(minimum,sweep['minimum_engineering_conservative_swept_xy_m'])
        assert all(x['covered_exactly'] for x in sweep['coarse_intervals'])
        assert (minimum>=0.) is assessment['safety_checks']['positive_engineering_clearance']
        illegal+=assessment['contact_audit']['illegal_robot_contacts_count']
        if not row['safety_within_declared_engineering_scope_pass']:
            failure=dict(run_id=row['run_id'],seed=row['seed'],method=method,
                minimum_engineering_conservative_swept_xy_m=minimum,safety_checks=assessment['safety_checks'],
                sweep_sha256=bind(assessment['sweep_report']['path'])['sha256'])
            failures.append(failure)
            # Reaggregate saved 2ms variation budgets independently for all four
            # negatives; does not rederive those budgets from raw dynamics.
            outputs={Path(r['path']).name:bound(r) for r in sweep['outputs']}
            with outputs['endpoints.csv'].open(newline='') as f:ep=list(csv.DictReader(f))
            budgets={}
            with outputs['intervals.jsonl'].open() as f:
                for line in f:
                    v=json.loads(line);key=v['start_ns'];assert key not in budgets
                    assert v['end_ns']==key+2_000_000
                    budgets[key]=v['relative_clearance_variation_budget_m']
            pad=sweep['numeric_profile']['endpoint_clearance_pad_m'];assert pad==1e-5
            bounds=[];max_error=0.
            assert len(ep)-1==len(sweep['coarse_intervals'])
            for left,right,declared in zip(ep,ep[1:],sweep['coarse_intervals']):
                a,b=int(left['stamp_ns']),int(right['stamp_ns'])
                total=math.fsum(budgets[t] for t in range(a,b,2_000_000))
                lower=min(float(left['conservative_xy_m']),float(right['conservative_xy_m']))-pad-.5*total
                max_error=max(max_error,abs(lower-declared['conditional_clearance_lower_bound_m']))
                close(lower,declared['conditional_clearance_lower_bound_m']);bounds.append(lower)
            close(min(bounds),minimum)
            sweep_aggregations.append(dict(run_id=row['run_id'],dense_intervals=len(budgets),coarse_intervals=len(bounds),
                independently_reaggregated_minimum_m=min(bounds),report_minimum_m=minimum,max_coarse_abs_difference_m=max_error,
                original_geometry_and_body_displacement_budget_derivation_replayed=False))
assert failures==parent['scoped_safety_failures'] and len(failures)==4
assert illegal==parent['total_illegal_robot_contacts']==0
diffs=[p['final_goal_distance_treatment_minus_control_m'] for p in ours['pairs']]
d=parent['paired_terminal_map_goal_distance_treatment_minus_control_m']
assert diffs==d['values'];close(statistics.mean(diffs),d['mean']);close(statistics.median(diffs),d['median'])
assert results['astra_analytic_preview_eligible_n05']['totals']['treatment_selection_changed_decisions']==504
assert parent['status']=='negative_preview_selection_development_diagnostic_no_promotion'
assert all(parent[k] is False for k in ('formal_result','performance_improvement_demonstrated','runtime_ready'))
assert 'Do not promote' in parent['decision'] and 'Cause unknown' in ' '.join(parent['claim_limits'])
output=dict(schema='astra_n05_interpretation_independent_consistency_v1',pass_=True,
    interpretation=bind(S/'audits/navigation_n05_result_interpretation/report.json'),
    method_results=results,paired_endpoint_mean_m=statistics.mean(diffs),paired_endpoint_median_m=statistics.median(diffs),
    four_negative_sweeps=failures,negative_sweep_reaggregation=sweep_aggregations,
    reported_zero_illegal_contacts_verified_against_hash_bound_assessment_summaries=True,raw_contact_messages_replayed=False,
    parent_outer_observation_note=bind(S/'queue/navigation_n05_outer_shell_observation.json'),
    outer143_independently_recovered_from_native_tool_record=False,normal_outer_exit_proved=False,cause_known=False,
    no_promotion_decision_supported_by_failure_inclusive_descriptive_results=True,
    resources=sorted(resources.values(),key=lambda r:r['path']))
with (A/'interpretation_check.json').open('x') as f:json.dump(output,f,sort_keys=True,indent=2,allow_nan=False);f.write('\n')
print('interpretation agrees; 504 changes;4 negative coarse sweeps independently reaggregated; no-promotion supported')
