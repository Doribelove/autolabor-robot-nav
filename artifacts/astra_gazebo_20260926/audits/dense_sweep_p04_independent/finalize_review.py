#!/usr/bin/env python3
"""Seal focused independent P04/Q05 offline evidence; grants no execution authority."""
import ast,datetime,json,sys
from pathlib import Path
D=Path(__file__).resolve().parent;S=D.parents[1];W=S/'workspace';R=S/'runs/passive_sweep_q04_static'
sys.path.insert(0,str(W/'scripts/astra_gazebo'))
import dense_sweep_p04 as p
import dense_sweep_p03 as old
def read(path):return json.loads(Path(path).read_text())
models=read(D/'actual_model_tests.json');conversions=read(D/'actual_conversion_binding_tests.json');wiring=read(D/'q05_wiring_review.json');diagnostic=read(D/'diagnostic_replay/report.json')
source=Path(p.__file__);oldsource=Path(old.__file__);original=read(R/'assessment.json')
functions=lambda path:{n.name:ast.dump(n,include_attributes=False)for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef)}
a,b=functions(oldsource),functions(source);same={n:a[n]==b[n]for n in ('engineering_pose','rotation_delta','point_budget','projections','read_p03_bag')}
checks={'frozen_p04_exact':p.sha(source)=='3bebade4f43d53f0a7cc0be1fd067d42bdb9d9c6251d4d2c728f59993c6f0736',
 'old_p03_unchanged':p.sha(oldsource)=='7de0531ffae59adc226f8de2531367c0e0575191499c00319530f9f8a430de52',
 'numeric_profile_unchanged':p.PROFILE==old.PROFILE,'model_tolerance_unchanged':p.MODEL_TOLERANCE==1e-7,
 'motion_and_projection_functions_ast_identical':all(same.values()),'author35_tests_reexecuted':'35 passed' in (D/'author_tests_r02.log').read_text(),
 'independent_actual_model12_pass':models['pass']is True and len(models['tests'])==12,
 'actual_conversion_and_synthetic_closure7_pass':conversions['pass']is True and len(conversions['tests'])==7,
 'Q05_real_prepared_and_assessment_wiring_pass':wiring['pass']is True,
 'full_historical_bag_diagnostic_complete':diagnostic['integrity_pass']is True and diagnostic['issues']==[] and diagnostic['dense_solver_intervals']==5000,
 'historical_diagnostic_cannot_promote_original':diagnostic['diagnostic_only']is True and diagnostic['runtime_qualification']is False and diagnostic['qualification']=='offline_diagnostic_only_not_a_historical_result_reclassification'and original['pass']is False and sum(original['checks'].values())==11,
 'original_Q04_assessment_hash_unchanged':p.sha(R/'assessment.json')=='d2e2b3823b346f747b6b319c255f7ba3bcffab3f3113ff347b31f52681e025ea',
 'old_Q04_budget_closure_retained':p.sha(S/'audits/passive_q04_result_independent/terminal_failure_audit.json')=='e489dbc848f6324a555eaa1343a3b576d65c29e053aad8599360abe598ed7318'}
resources={}
for item in p.conversion_resources(D/'conversion/manifest.json'):resources[str(Path(item['path']).resolve())]=item
paths=[x for x in D.rglob('*')if x.is_file() and x.name!='review.json' and not (x.name.startswith('finalize_review_') and x.suffix=='.log')]+[source,oldsource,W/'scripts/astra_gazebo/dense_sweep.py',W/'scripts/astra_gazebo/geometry.py',W/'scripts/astra_gazebo/evaluate_episode.py',W/'scripts/astra_gazebo/run_passive_sweep_q05.py',W/'scripts/astra_gazebo/passive_sweep_q05.py',W/'scripts/astra_gazebo/bound_archive.py',W/'scripts/astra_gazebo/tests/test_dense_sweep_p04.py',R/'fixture.bag',R/'world.sdf',R/'robot_description_runtime.xml',R/'run_closure.json',R/'assessment.json',S/'audits/passive_q04_result_independent/terminal_failure_audit.json',S/'audits/passive_q04_result_independent/mass_diagnosis_inputs/runtime_metadata.jsonl',Path(wiring['prepared_binding']['path'])]
for path in paths:resources[str(path.resolve())]=p.binding(path)
report={'status':'independent_P04_model_conversion_and_Q05_integration_review_pass'if all(checks.values())else'independent_P04_review_failed','pass':all(checks.values()),'independent_review':True,'reviewed_p04_sha256':p.sha(source),'reviewed_q05_runner_sha256':wiring['reviewed_runner']['sha256'],'q05_prepared_binding':wiring['prepared_binding'],'numeric_profile_unchanged':p.PROFILE==old.PROFILE,'model_tolerance_unchanged':p.MODEL_TOLERANCE,'historical_diagnostic_only':True,'historical_Q04_not_reclassified':True,'checks':checks,'unchanged_ast_functions':same,
 'tests':{'author':{'passed':35,'python':'/usr/bin/python3','log':p.binding(D/'author_tests_r02.log')},'independent_actual_model':{'passed':12,'report':p.binding(D/'actual_model_tests.json')},'actual_conversion_records_and_synthetic_fresh_closure':{'passed':7,'report':p.binding(D/'actual_conversion_binding_tests.json')},'Q05_integration':{'passed':6,'report':p.binding(D/'q05_wiring_review.json')}},
 'historical_diagnostic':{'report':p.binding(D/'diagnostic_replay/report.json'),'intervals':5000,'source_window_ns':diagnostic['source_window_ns'],'minimum_engineering_conservative_swept_xy_m':diagnostic['minimum_engineering_conservative_swept_xy_m'],'runtime_qualification':False,'original_failed_gate_retained':True},
 'reviewed_change':'Compare actual runtime merged mass, COM, inertia and collision shape identity against exact twice-reproduced converter SDF, map all URDF fixed joints/body members/collisions with explicit quantization, and take an outward union radius around one actual COM. Original comparison tolerance and geometric envelope profile remain unchanged.',
 'integration':'Actual Q05 prepared_r02 contains 64 current source resources and two seven-body/nine-collision converted models. Conversion path/hash/resource roster and rendered launch match. Actual assessor argv includes P04, converted SDF and manifest without historical diagnostic flag. Runtime libsdformat9 role is required; actual Q05 maps still must prove it after fresh execution.',
 'preserved_errors':'Initial author-test rerun under workspace venv could not collect because old system SciPy requires np.typeDict absent in venv NumPy; saved r01 log. System-compatible Python then passed35. Actual converter/model/CLI ran in frozen workspace venv. A pre-freeze missing-test-file discovery is also retained. No runtime evidence failure was hidden. First finalizer incorrectly included its still-open stdout log in resource bindings; that report/script are preserved separately, and finalizer logs are excluded from the final resource roster.',
 'scope':['Offline component review only; no ROS/Gazebo world, learning or vehicle execution.','Fresh-schema validator positive is explicitly synthetic augmentation of old closure for testing, not a live qualification.','Historical Q04 remains failed with one consumed and one retired identity.','Q05 still requires its own release, fresh run, all maps/source/model/contact evidence and terminal result audit.','Engineering envelope remains limited to declared Gazebo normal-Euler extension and bounded Begin jumps, not a formal floating-point or real continuous-safety proof.'],
 'execution_authorized':False,'no_ROS_or_Gazebo_world_started':True,'resources':list(resources.values()),'created_utc':datetime.datetime.now(datetime.timezone.utc).isoformat()}
with (D/'review.json').open('x')as f:json.dump(report,f,indent=2,ensure_ascii=False,allow_nan=False);f.write('\n')
print(json.dumps({'status':report['status'],'checks':len(checks),'pass':report['pass'],'sha256':p.sha(D/'review.json'),'resource_count':len(resources)},indent=2));assert report['pass']
