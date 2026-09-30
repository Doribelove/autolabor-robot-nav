"""Freeze this independent result audit; no execution authority or old writes."""
from pathlib import Path
import hashlib,json

A=Path(__file__).resolve().parent;S=A.parents[1]
def bind(path):
    path=Path(path).resolve();h=hashlib.sha256()
    with path.open('rb') as f:
        for data in iter(lambda:f.read(4*1024*1024),b''):h.update(data)
    return {'path':str(path),'sha256':h.hexdigest(),'bytes':path.stat().st_size}
def read(path):return json.loads(Path(path).read_bytes())
def write(name,data):
    with (A/name).open('x') as f:json.dump(data,f,sort_keys=True,indent=2,allow_nan=False);f.write('\n')

replay=read(A/'recomputed.json');interp=read(A/'interpretation_check.json');inputs=read(A/'input_resources.json')
assert replay['pass_'] and interp['pass_'] and replay['all_parent_fields_and_CSV_exact']
resources={r['path']:r for r in inputs['resources']}
for row in interp['resources']:resources[row['path']]=row
for path in A.iterdir():
    if path.is_file() and path.name not in ('review.json','resources.json','hash_validation.json'):
        row=bind(path);resources[row['path']]=row
for path in (Path('/usr/bin/python3.8'),S/'workspace/scripts/astra_env.sh'):
    row=bind(path);resources[row['path']]=row
for row in resources.values():assert bind(row['path'])==row,row['path']
write('resources.json',{'scope':'independent reviewed input and output hashes; not new execution release','resources':sorted(resources.values(),key=lambda r:r['path'])})
report={
 'schema':'astra_navigation_n05_terminal_independent_review_v1',
 'status':'independent_all24_selected_topic_task_mechanism_and_negative_interpretation_review_pass',
 'independent_review':True,'original_resources_modified':False,'ROS_started':False,'Gazebo_started':False,'training_started':False,
 'release':replay['release'],'ledger':replay['ledger'],
 'reviewed_replay_script':bind(S/'audits/navigation_n05_terminal_replay_r04.py'),
 'reviewed_replay_report':bind(S/'audits/navigation_n05_terminal_replay/report.json'),
 'reviewed_interpretation_report':interp['interpretation'],
 'preflight_review_inherited_with_limited_scope':bind(S/'audits/navigation_n05_terminal_replay_preflight/r04/review.json'),
 'consumed_attempt1':24,'retired':0,'complete_pairs':12,
 'source_binding_checks':{'release_resources':1179,'receipt_artifacts':696,'original_receipts':24,'exact_prepared_ledger_schedule':True},
 'independent_raw_replay':{'bags':24,'entire_selected_topics':replay['topics'],
    'entire_selected_topic_message_counts':replay['bag_topic_message_counts'],'formal_2ms_base_pose_samples':sum(r['dense_window_samples'] for r in replay['details']),
    'all_task_goal_endpoint_trace_candidate_fields_match_original_report_and_CSV':True,
    'first_dense_goal_independently_recomputed':True,'first_actual_odom_terminal_independently_recomputed':True,
    'production_replay_or_navigation_assessor_imported':False,
    'all_treatment_eligibility_masks_and_selected_membership_checked':True,
    'planner_rollouts_and_analytic_priors_recomputed':False},
 'method_outcomes':replay['method_outcomes'],
 'restricted_time_treatment_minus_control_mean_s':replay['mean_restricted_treatment_minus_control_s'],
 'endpoint_treatment_minus_control_mean_m':interp['paired_endpoint_mean_m'],
 'endpoint_treatment_minus_control_median_m':interp['paired_endpoint_median_m'],
 'mechanism':interp['method_results'],
 'failure_inclusive':True,'all_safety_scoped_pass':False,
 'four_negative_sweeps':interp['four_negative_sweeps'],
 'negative_sweep_saved_outputs_independently_reaggregated':interp['negative_sweep_reaggregation'],
 'raw_collision_geometry_and_native_budget_derivation_replayed':False,'raw_contact_messages_replayed':False,'PNG_visual_review_performed':False,
 'manager_and_outer_exit':{'manager_return0_durable_file_and_stdout_verified':True,'terminal_ledger_and24_receipts_verified':True,
   'all24_child_exit0_receipts':True,'expected_outer_exit_file_absent':True,
   'saved_shell_script_checked':True,'parent_tool_session_id':1158,'parent_tool_observed_exit_code':143,
   'original_tool_response_independently_available':False,'observation_note':interp['parent_outer_observation_note'],
   'normal_outer_exit_proven':False,'cause':'unknown','no_inferred_signal_sender_or_termination_cause':True},
 'decision':'No promotion of N05 preview eligibility into training/evaluation policy; terminal negative development diagnostic retained.',
 'formal_result':False,'performance_improvement_demonstrated':False,'runtime_ready':False,'general_safety_or_deployment_claim':False,
 'limits':['six reused development parent geometries, two fresh seeds each; not twelve independent geometries',
   'endpoint differences compare each method at its own terminal source, not matched elapsed state',
   'trace/candidate times are runtime clock metadata, not driver ingress source',
   'stop fractions use recorded trace counts and are not time durations or causal mechanism identification',
   'physical integrity/contact claims inherited from hash-bound assessments; four negative saved clearance aggregations independently checked',
   'parent tool143 is explicitly provenance-limited; missing exit file not fabricated or assigned a cause'],
 'recomputed':bind(A/'recomputed.json'),'interpretation_consistency':bind(A/'interpretation_check.json'),
 'review_text':bind(A/'review.md'),'resource_manifest':bind(A/'resources.json'),
 'commands':[
  'source '+str(S/'workspace/scripts/astra_env.sh')+'; PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 '+str(S/'workspace/.venv_train_cuda/bin/python')+' -B '+str(A/'recompute.py'),
  'PYTHONDONTWRITEBYTECODE=1 PYTHONNOUSERSITE=1 '+str(S/'workspace/.venv_train_cuda/bin/python')+' -B '+str(A/'check_interpretation.py')]}
write('review.json',report)
validation={'all_resources_rehashed_match':True,'resources_verified':len(resources),'review':bind(A/'review.json'),'text':bind(A/'review.md'),'resource_manifest':bind(A/'resources.json')}
write('hash_validation.json',validation)
print(json.dumps(validation,sort_keys=True))
