"""Author follow-up only, not an independent review. Never modifies B05 inputs."""
import copy,csv,hashlib,json,pathlib,sys
S=pathlib.Path('/home/robot/robot_ws_base_rl/artifacts/astra_gazebo_20260926');P=S/'workspace/scripts/astra_gazebo';A=pathlib.Path(__file__).parent
sys.path.insert(0,str(P));import baseline_static_qualification_b05 as b

def bind(p):
 p=pathlib.Path(p).resolve();v=p.read_bytes();return {'path':str(p),'sha256':hashlib.sha256(v).hexdigest(),'bytes':len(v)}
def write(name,x):(A/name).write_text(json.dumps(x,indent=2,allow_nan=False)+'\n')
old=S/'audits/baseline_static_qualification_b05';manifest=json.loads((old/'source_evidence_manifest.json').read_text());errors=[]
for row in manifest['resources']:
 actual=bind(row['path'])
 if actual['sha256']!=row['sha256']:errors.append({'expected':row,'actual':actual})
assert len(manifest['resources'])==264 and not errors
scenes=['clear','static0','static1','static2','static3'];first=b.PROPOSED_SEEDS[0]
observations=[]
# This is not a defect: equality has no provenance/semantic completeness authority.
r=b.compare_full_parameters({'max_vel_x':1.2},{'max_vel_x':1.2})
assert r['pass'] and r['runtime_default_completion_proved_by_this_function'] is False
observations.append({'id':'equal_partial_tree_is_not_runtime_default_proof','result':r,'classification':'declared_scope_limit','inference':'Release must independently bind a complete expected tree. Equality alone cannot distinguish an intentionally partial namespace.'})
# Type coercion asymmetry: registry CSV strings are normalized, other-proposal strings are not.
try:b.proposal_schedule(scenes,[{'seed':str(first),'run_id':'historical'}])
except ValueError:registry_rejected=True
else:registry_rejected=False
try:b.proposal_schedule(scenes,[],[first])
except ValueError:integer_rejected=True
else:integer_rejected=False
try:proposal=b.proposal_schedule(scenes,[],[str(first)])
except ValueError:string_accepted=False
else:string_accepted=True
assert registry_rejected and integer_rejected and string_accepted
observations.append({'id':'other_proposed_seed_string_bypasses_integer_overlap','classification':'retained_API_type_validation_gap','seed':first,'registry_CSV_string_rejected':registry_rejected,'other_proposal_integer_rejected':integer_rejected,'other_proposal_numeric_string_accepted':string_accepted,'execution_authorized_in_return':proposal['execution_authorized'],'inference':'Current actual N06 prepared units supply integers and actual frozen B05 proposal remains fresh. A future generic caller supplying decimal-string proposal seeds can bypass this helper overlap gate; validate exact int elements or fail closed in a new revision before such use. Do not mutate frozen r01.'})
# actual frozen prepare inputs are explicitly checked, no blanket safe-input assumption.
n06=json.loads((S/'contracts/navigation_n06_prepared_r01/prepared.json').read_text());prior_seeds=[r['seed'] for r in n06['units']]
assert all(type(x) is int for x in prior_seeds) and not set(prior_seeds)&set(b.PROPOSED_SEEDS)
reg_path=S/'seed_registry.csv';raw=reg_path.read_bytes();(A/'seed_registry_at_followup.csv').write_bytes(raw)
registry=list(csv.DictReader(raw.decode().splitlines()));schedule=b.proposal_schedule([x['scene_id'] for x in json.loads((S/'contracts/baseline_static_qualification_b05_draft.json').read_text())['scenes']],registry,prior_seeds)
assert schedule==json.loads((old/'proposed_schedule.json').read_text())
observations.append({'id':'actual_frozen_input_domain_has_no_detected_collision','classification':'actual_current_inputs_verified','N06_seed_types':['int'],'schedule_bytes_semantics_unchanged':True,'registry_snapshot':bind(A/'seed_registry_at_followup.csv'),'future_registration_or_authority':False})
# Positive labels cannot be read as a qualification result from this accounting helper.
records=[dict(run_id=r['run_id'],method=r['method'],seed=r['seed'],attempt=1,record_kind='terminal',phase='complete',capture_integrity_pass=True,geometry_pass=True,goal_elapsed_s=0.,reason='synthetic_no_physical_data') for r in schedule['schedule']]
r=b.failure_inclusive_summary(schedule['schedule'],records)
assert all(v['valid_completed']==10 for v in r['methods'].values()) and r['qualification_determined'] is False
observations.append({'id':'accounting_does_not_authenticate_goal_or_physics','classification':'declared_scope_limit','synthetic_zero_time_goals_accepted_for_arithmetic':True,'qualification_determined':r['qualification_determined'],'performance_claim':r['performance_claim'],'inference':'Only a future actual-source assessor may produce these trusted result rows. Caller booleans and zero-time fixtures are not physical evidence; no B05 goal has been measured.'})
# Missing real geometry implementation is declared in the exact frozen design.
design=json.loads((S/'contracts/baseline_static_qualification_b05_draft.json').read_text())
assert design['execution_ready'] is False and 'not_implemented_pending' in design['full_parameter_contract']['automatic_default_completion_status']
footprint=next(x for x in design['physical_and_reference_gates'] if x['id']=='actual_footprint')
assert 'pending' in footprint['status'] and 'pending' in design['command_and_receipt_contract']['implementation_status']
observations.append({'id':'runtime_gaps_are_explicit_not_synthetic_qualified','classification':'declared_pending','full_default_materialization':design['full_parameter_contract']['automatic_default_completion_status'],'actual_nine_collision_shape_containment':footprint['status'],'command_ingress':design['command_and_receipt_contract']['implementation_status'],'manager_collector':'new B05 implementation remains pending','dynamic_actor_supported':False})
write('probe_results.json',{'schema':'astra_b05_author_followup_probes_v1','author_recheck_only':True,'independent_review':False,'original264_resources_match':True,'observations':observations,'retained_negative_count':1,'ROS_started':False,'release_created':False,'seed_registered':False})
print(json.dumps({'original_resources':264,'author_observation_groups':len(observations),'retained_API_gap':'other proposal numeric strings not normalized/rejected','current_actual_schedule_unchanged':True,'output':bind(A/'probe_results.json')},indent=2))
