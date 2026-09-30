"""Frozen descriptive interpretation of N05 terminal development replay."""
import hashlib
import json
import statistics
from pathlib import Path

S=Path(__file__).resolve().parents[1]
REPORT=S/'audits/navigation_n05_terminal_replay/report.json'
LEDGER=S/'queue/navigation_n05_ledger.json'
SHELL_NOTE=S/'queue/navigation_n05_outer_shell_observation.json'
OUT=S/'audits/navigation_n05_result_interpretation'
METHODS=('astra_analytic_route_v1','astra_analytic_preview_eligible_n05')


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUT.exists():raise FileExistsError('N05 interpretation identity exists')
    report=json.loads(REPORT.read_bytes());ledger=json.loads(LEDGER.read_bytes())
    assert report['release_sha256']==ledger['release_sha256'] and report['ledger_sha256']==sha(LEDGER)
    assert ledger['status']=='terminal' and report['consumed']==report['integrity_valid']==24
    assert report['retired_unrun']==0 and report['integrity_failed']==0 and report['complete_pairs']==12
    rows=report['rows'];pairs=report['pairs']
    assert len(rows)==24 and len(pairs)==12 and len({r['run_id'] for r in rows})==24
    assert {r['seed'] for r in rows}=={p['seed'] for p in pairs}
    summaries={}
    failed=[];all_illegal_contacts=[]
    for method in METHODS:
        subset=[r for r in rows if r['method']==method]
        assert len(subset)==12
        for row in subset:
            root=S/'runs'/row['run_id'];assessment_path=root/'assessment.json'
            assert sha(assessment_path)==row['assessment_sha256']
            assessment=json.loads(assessment_path.read_bytes())
            sweep_path=Path(assessment['sweep_report']['path'])
            assert sha(sweep_path)==assessment['sweep_report']['sha256']
            sweep=json.loads(sweep_path.read_bytes())
            if assessment['safety_within_declared_engineering_scope_pass'] is not True:
                failed.append(dict(run_id=row['run_id'],seed=row['seed'],method=method,
                    minimum_engineering_conservative_swept_xy_m=sweep['minimum_engineering_conservative_swept_xy_m'],
                    safety_checks=assessment['safety_checks'],sweep_sha256=sha(sweep_path)))
            assert assessment['contact_audit']['illegal_robot_contacts_count']==0
            all_illegal_contacts.append(assessment['contact_audit']['illegal_robot_contacts_count'])
        totals={key:sum(r[key] for r in subset) for key in (
            'trace_ticks','executed_stop_ticks','backup_revalidation_failed_ticks',
            'backup_stop_requested_ticks','transaction_deadline_drop_ticks',
            'treatment_selection_changed_decisions','candidate_decisions',
            'treatment_moving_backup_count_sum','treatment_moving_eligible_count_sum')}
        summaries[method]=dict(valid_rows=12,complete=sum(r['task_phase']=='complete' for r in subset),
            timeout=sum(r['task_phase']=='timeout' for r in subset),
            scoped_safety_pass=sum(r['safety_within_declared_engineering_scope_pass'] is True for r in subset),
            mean_terminal_map_goal_distance_m=statistics.mean(r['terminal_map_goal_xy_distance_m'] for r in subset),
            median_terminal_map_goal_distance_m=statistics.median(r['terminal_map_goal_xy_distance_m'] for r in subset),
            totals=totals,executed_stop_trace_message_fraction=totals['executed_stop_ticks']/totals['trace_ticks'])
    assert len(failed)==4 and {x['seed'] for x in failed}=={263002005,263002006}
    assert set(all_illegal_contacts)=={0}
    diffs=[p['final_goal_distance_treatment_minus_control_m'] for p in pairs]
    assert len(diffs)==12
    document=dict(schema='astra_navigation_n05_terminal_development_interpretation_v1',
        status='negative_preview_selection_development_diagnostic_no_promotion',
        release_sha256=ledger['release_sha256'],ledger_sha256=sha(LEDGER),
        replay_report_sha256=sha(REPORT),replay_script_sha256=sha(S/'audits/navigation_n05_terminal_replay_r04.py'),
        outer_shell_observation_sha256=sha(SHELL_NOTE),
        schedule='12 paired fresh-seed blocks over six previously used development parent geometries, two seeds per parent; no new independent geometries',
        attempts='24/24 consumed exactly once; integrity24; retired0; 12 complete pairs; no retry/resume',
        method_summaries=summaries,
        paired_terminal_map_goal_distance_treatment_minus_control_m=dict(
            values=diffs,mean=statistics.mean(diffs),median=statistics.median(diffs),
            note='Each endpoint is measured at its own exact terminal source time; failure-inclusive and descriptive, not matched-state distance.'),
        scoped_safety_failures=failed,total_illegal_robot_contacts=sum(all_illegal_contacts),
        factor_mechanism='The treatment made 504 selection changes and reduced backup_revalidation_failed trace messages but increased executed-stop and backup-stop-requested trace messages. These message counts overlap and are not time durations or proof of why a goal failed.',
        decision='Do not promote N05 preview eligibility into the training/evaluation policy. A later isolated factor may use new development geometries only after a separate reviewed release; N05 seeds and budget remain terminal.',
        claim_limits=['No all-scoped-safety pass because four b04/b05 episodes have negative conservative engineering swept clearance despite zero recorded illegal contact.',
                      'Not a formal safety, navigation performance, six-family, learned policy, sim-to-real, hardware or deployment result.',
                      'Manager returned0 and ledger terminal; parent tool observed outer shell exit143 without the expected outer exit file. Cause unknown.'],
        independent_result_review_pending=True,formal_result=False,
        performance_improvement_demonstrated=False,runtime_ready=False)
    OUT.mkdir(parents=True)
    (OUT/'report.json').write_text(json.dumps(document,indent=2)+'\n')
    print(json.dumps(dict(status=document['status'],mean_endpoint_difference_m=statistics.mean(diffs),
                          scoped_safety_failures=len(failed),report_sha256=sha(OUT/'report.json'))))


if __name__=='__main__':main()
