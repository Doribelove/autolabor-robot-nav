"""V3-07-B0 spatial predictor data bindings."""

import importlib.util
from pathlib import Path

W=Path("/home/robot/robot_ws_base_rl")
PARENT_PATH=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_a3.py"
s=importlib.util.spec_from_file_location("a3_parent_for_b0",PARENT_PATH)
PARENT=importlib.util.module_from_spec(s); s.loader.exec_module(PARENT)
for name in ("V301Error","require","sha256_bytes","yaml_bytes","canonical_sha","read_file_once","load_yaml_path","load_yaml_relative","receipt","absolute_receipt","load_trace","interpolate","check_receipts","actor_direction"):
    globals()[name]=getattr(PARENT,name)

CONTRACT_RELATIVE=Path("config/thesis_experiments/v3/v3_07_b0_spatial_predictor_data_contract.yaml")
PREREGISTRATION_RELATIVE=Path("experiments/manifests/v3/v3_07_b0_spatial_predictor_data_preregistration.yaml")
AUTHORIZATION_RELATIVE=Path("experiments/manifests/v3/v3_07_b0_simulation_authorization.yaml")
ROOT_RELATIVE=Path("artifacts/v3/prediction/v3_07_b0")
READINESS_RELATIVE=ROOT_RELATIVE/"offline_readiness"
SCHEDULE_RELATIVE=READINESS_RELATIVE/"v3_07_b0_exact_schedule.yaml"
BINDINGS_RELATIVE=READINESS_RELATIVE/"v3_07_b0_runtime_bindings.yaml"
CLOSURE_RELATIVE=READINESS_RELATIVE/"v3_07_b0_dependency_closure.yaml"
RELEASE_RELATIVE=READINESS_RELATIVE/"v3_07_b0_execution_release.yaml"
REVIEW_RELATIVE=READINESS_RELATIVE/"v3_07_b0_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE=READINESS_RELATIVE/"v3_07_b0_readiness_freeze.sha256"
EXECUTION_RELATIVE=ROOT_RELATIVE/"execution"
JOURNAL_RELATIVE=EXECUTION_RELATIVE/"v3_07_b0_attempt_journal.jsonl"
DATA_QUALIFICATION_RELATIVE=EXECUTION_RELATIVE/"v3_07_b0_data_qualification.yaml"
EVIDENCE_FREEZE_RELATIVE=EXECUTION_RELATIVE/"v3_07_b0_evidence_freeze.sha256"
DATA_REVIEW_RELATIVE=EXECUTION_RELATIVE/"v3_07_b0_independent_data_review.yaml"
MODEL_RELATIVE=ROOT_RELATIVE/"model/v3_07_b0_spatial_candidate_predictor.yaml"
CALIBRATION_RELATIVE=ROOT_RELATIVE/"model/v3_07_b0_calibration_report.yaml"
MODEL_FREEZE_RELATIVE=ROOT_RELATIVE/"model/v3_07_b0_model_freeze.sha256"
MODEL_REVIEW_RELATIVE=ROOT_RELATIVE/"model/v3_07_b0_independent_model_review.yaml"
TERMINAL_FREEZE_RELATIVE=ROOT_RELATIVE/"v3_07_b0_terminal_freeze.sha256"
TRACE_METRICS_NEEDS_ROW=True
BINARY_RESOURCES=PARENT.BINARY_RESOURCES
SOURCE_RESOURCES=(
    CONTRACT_RELATIVE,PREREGISTRATION_RELATIVE,
    Path("artifacts/v3/spatial/v3_07_a3/v3_07_a3_terminal_freeze.sha256"),
    Path("artifacts/v3/spatial/v3_07_a3/execution/v3_07_a3_safe_overlap_assessment.yaml"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_spatial_candidates.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_a1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_a2.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_a3.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0.py"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_07_b0_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_07_b0_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_07_b0_data.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_07_b0_data.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_07_b0_data.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_07_b0_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_07_b0_predictor.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_07_b0.py"),
)

def validate_contract(contract,prereg):
    require(contract["stage"]=="V3-07-B0" and contract["status"]=="preregistered_before_simulation","B0 contract differs")
    unit=contract["experimental_unit"]
    require(len(unit["scenario_blocks"])==10 and unit["exact_rows"]==50 and unit["training_rows"]==30 and unit["calibration_rows"]==20 and unit["retry_allowed"] is False and unit["resume_allowed"] is False,"B0 population differs")
    require(prereg["status"]=="frozen_before_any_v3_07_b0_gazebo_execution" and prereg["schedule"]["exact_rows"]==50 and prereg["model"]["calibration_not_used_for_coefficients"] is True and prereg["failure_policy"]["no_model_fit_if_data_qualification_fails"] is True,"B0 prereg differs")
    authority=contract["authority_boundary"]
    require(authority["held_out_test_seeds_reserved"]==[6631,6632,6633,6634] and authority["held_out_test_access_authorized"] is False and authority["supervised_model_fit_after_data_qualification_authorized"] is True and authority["reinforcement_learning_authorized"] is False and authority["real_vehicle_authorized"] is False,"B0 authority differs")

def exact_schedule(contract):
    rows=[]; sequence=0
    support=contract["data_support"]["eligible_cells"]
    for block in contract["experimental_unit"]["scenario_blocks"]:
        for policy in contract["experimental_unit"]["policies_evaluator_only"]:
            for candidate in support[policy]:
                sequence+=1
                rows.append({"sequence":sequence,"unit_id":"v3_07_b0_{}_{}_{}".format(block["block_id"],policy,candidate),"block_id":block["block_id"],"role_evaluator_only":block["role"],"seed":int(block["seed"]),"angle_deg":float(block["angle_deg"]),"actor_start_distance_m":float(block["actor_start_distance_m"]),"actor_base_speed_mps":float(block["actor_base_speed_mps"]),"actor_policy_evaluator_only":policy,"candidate_id_evaluator_only":candidate,"ego_speed_mps":(1.2 if candidate=="spatial_left_v120" else 0.0),"attempt":1,"propensity":1.0/len(support[policy])})
    require(len(rows)==50,"B0 schedule differs")
    return {"schema_version":"1.0","stage":"V3-07-B0","schedule_id":"v3_scope_v3_07_b0_exact_schedule_1","status":"frozen_before_execution_release","row_count":50,"training_rows":30,"calibration_rows":20,"held_out_test_rows":0,"reserved_held_out_seeds_not_materialized":[6631,6632,6633,6634],"authorized_attempts_per_identity":1,"retry_allowed":False,"rows":rows,"schedule_digest":canonical_sha(rows)}

def render_world(contract,row,output_csv,done_marker):
    runtime=dict(row); runtime["runtime_candidate_id"]=row["candidate_id_evaluator_only"]
    return PARENT.render_world(contract,runtime,output_csv,done_marker)

def _actor_coordinate(rows,row,time_s):
    dx,dy=actor_direction(row)
    return interpolate(rows,"actor_x_m",time_s)*dx+interpolate(rows,"actor_y_m",time_s)*dy

def trace_metrics(rows,row):
    coordinate=_actor_coordinate(rows,row,0.5)
    speed=(coordinate-_actor_coordinate(rows,row,0.3))/0.2
    mode=PARENT.infer_mode(
        speed,load_yaml_relative(W,CONTRACT_RELATIVE)["observed_mode_gate"]
    )
    candidate=row["candidate_id_evaluator_only"]
    return {"actor_conflict_coordinate_at_observation_m":coordinate,"actor_observed_speed_mps":speed,"observed_mode":mode,"candidate_id":candidate,"spatial_indicator":1.0 if candidate=="spatial_left_v120" else 0.0,"target_actor_conflict_coordinates_m":[_actor_coordinate(rows,row,t) for t in (1.5,2.5,3.5,4.5)],"initial_ego_xy_m":[rows[0]["ego_x_m"],rows[0]["ego_y_m"]]}
