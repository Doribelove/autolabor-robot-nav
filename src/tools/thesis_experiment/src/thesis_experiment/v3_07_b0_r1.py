"""V3-07-B0-R1 schedule-key compatibility repair bindings."""

import copy,importlib.util
from pathlib import Path
W=Path("/home/robot/robot_ws_base_rl"); PARENT_PATH=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0.py"
s=importlib.util.spec_from_file_location("b0_parent_for_r1",PARENT_PATH); PARENT=importlib.util.module_from_spec(s); s.loader.exec_module(PARENT)
for name in ("V301Error","require","sha256_bytes","yaml_bytes","canonical_sha","read_file_once","load_yaml_path","receipt","absolute_receipt","load_trace","interpolate","check_receipts","actor_direction","render_world","trace_metrics"):
    globals()[name]=getattr(PARENT,name)
BASE_LOAD=PARENT.load_yaml_relative
PARENT_CONTRACT=PARENT.CONTRACT_RELATIVE
CONTRACT_RELATIVE=Path("config/thesis_experiments/v3/v3_07_b0_r1_schedule_key_repair_contract.yaml")
PREREGISTRATION_RELATIVE=Path("experiments/manifests/v3/v3_07_b0_r1_schedule_key_repair_preregistration.yaml")
AUTHORIZATION_RELATIVE=Path("experiments/manifests/v3/v3_07_b0_r1_simulation_authorization.yaml")
ROOT_RELATIVE=Path("artifacts/v3/prediction/v3_07_b0_r1"); READINESS_RELATIVE=ROOT_RELATIVE/"offline_readiness"
SCHEDULE_RELATIVE=READINESS_RELATIVE/"v3_07_b0_r1_exact_schedule.yaml"; BINDINGS_RELATIVE=READINESS_RELATIVE/"v3_07_b0_r1_runtime_bindings.yaml"; CLOSURE_RELATIVE=READINESS_RELATIVE/"v3_07_b0_r1_dependency_closure.yaml"; RELEASE_RELATIVE=READINESS_RELATIVE/"v3_07_b0_r1_execution_release.yaml"; REVIEW_RELATIVE=READINESS_RELATIVE/"v3_07_b0_r1_independent_release_review.yaml"; READINESS_FREEZE_RELATIVE=READINESS_RELATIVE/"v3_07_b0_r1_readiness_freeze.sha256"
EXECUTION_RELATIVE=ROOT_RELATIVE/"execution"; JOURNAL_RELATIVE=EXECUTION_RELATIVE/"v3_07_b0_r1_attempt_journal.jsonl"; DATA_QUALIFICATION_RELATIVE=EXECUTION_RELATIVE/"v3_07_b0_r1_data_qualification.yaml"; EVIDENCE_FREEZE_RELATIVE=EXECUTION_RELATIVE/"v3_07_b0_r1_evidence_freeze.sha256"; DATA_REVIEW_RELATIVE=EXECUTION_RELATIVE/"v3_07_b0_r1_independent_data_review.yaml"
MODEL_RELATIVE=ROOT_RELATIVE/"model/v3_07_b0_r1_spatial_candidate_predictor.yaml"; CALIBRATION_RELATIVE=ROOT_RELATIVE/"model/v3_07_b0_r1_calibration_report.yaml"; MODEL_FREEZE_RELATIVE=ROOT_RELATIVE/"model/v3_07_b0_r1_model_freeze.sha256"; MODEL_REVIEW_RELATIVE=ROOT_RELATIVE/"model/v3_07_b0_r1_independent_model_review.yaml"; TERMINAL_FREEZE_RELATIVE=ROOT_RELATIVE/"v3_07_b0_r1_terminal_freeze.sha256"
TRACE_METRICS_NEEDS_ROW=True; BINARY_RESOURCES=PARENT.BINARY_RESOURCES
SOURCE_RESOURCES=(
    CONTRACT_RELATIVE,PREREGISTRATION_RELATIVE,Path("artifacts/v3/prediction/v3_07_b0/v3_07_b0_terminal_freeze.sha256"),
    PARENT_CONTRACT,Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0.py"),Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0_r1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_spatial_candidates.py"),Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_a3.py"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"),Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_a1_readiness.py"),Path("src/tools/thesis_experiment/scripts/review_v3_02_a1_readiness.py"),Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_07_b0_data.py"),Path("src/tools/thesis_experiment/scripts/fit_v3_07_b0_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_07_b0_r1_readiness.py"),Path("src/tools/thesis_experiment/scripts/review_v3_07_b0_r1_readiness.py"),Path("src/tools/thesis_experiment/scripts/run_v3_07_b0_r1_data.py"),Path("src/tools/thesis_experiment/scripts/assess_v3_07_b0_r1_data.py"),Path("src/tools/thesis_experiment/scripts/review_v3_07_b0_r1_data.py"),Path("src/tools/thesis_experiment/scripts/fit_v3_07_b0_r1_predictor.py"),Path("src/tools/thesis_experiment/scripts/review_v3_07_b0_r1_predictor.py"),Path("src/tools/thesis_experiment/tests/test_v3_07_b0_r1.py"),
)
def effective_contract():
    parent=copy.deepcopy(BASE_LOAD(W,PARENT_CONTRACT)); repair=BASE_LOAD(W,CONTRACT_RELATIVE)
    parent["stage"]="V3-07-B0-R1"; parent["stage_purpose"]=repair["stage_purpose"]; parent["contract_id"]=repair["contract_id"]; parent["status"]=repair["status"]; parent["parent_failure"]=repair["parent_failure"]; parent["single_factor_repair"]=repair["single_factor_repair"]
    parent["experimental_unit"]["scenario_blocks"]=repair["fresh_scenario_blocks"]; parent["experimental_unit"]["schedule_rows"]=50; parent["experimental_unit"]["exact_rows"]=50
    parent["authority_boundary"]["held_out_test_seeds_reserved"]=repair["held_out_test_seeds_reserved"]
    parent["claim_if_pass"]="Fresh repaired safe-overlap data qualify only the frozen supervised spatial predictor fit."
    return parent
def load_yaml_relative(workspace,relative):
    if Path(relative)==CONTRACT_RELATIVE: return effective_contract()
    return BASE_LOAD(workspace,relative)
def validate_contract(contract,prereg):
    require(contract["stage"]=="V3-07-B0-R1" and contract["status"]=="single_factor_repair_preregistered_before_simulation","R1 contract differs")
    repair=contract["single_factor_repair"]; require(repair["factor_id"]=="add_schedule_rows_runner_compatibility_alias" and repair["exact_rows_semantic_value"]==repair["schedule_rows_alias_value"]==50 and all(repair[k] is False for k in ("data_support_changed","scene_changed","predictor_preregistration_changed","qualification_gates_changed","safety_radius_changed")),"R1 repair differs")
    require(contract["experimental_unit"]["schedule_rows"]==50 and contract["experimental_unit"]["exact_rows"]==50 and prereg["status"]=="frozen_before_any_v3_07_b0_r1_gazebo_execution" and prereg["repair"]["data_support_model_scene_gates_unchanged"] is True,"R1 prereg differs")
    require(contract["authority_boundary"]["held_out_test_seeds_reserved"]==[6661,6662,6663,6664] and contract["authority_boundary"]["held_out_test_access_authorized"] is False,"R1 firewall differs")
def exact_schedule(contract):
    rows=[]; sequence=0; support=contract["data_support"]["eligible_cells"]
    for block in contract["experimental_unit"]["scenario_blocks"]:
        for policy in contract["experimental_unit"]["policies_evaluator_only"]:
            for candidate in support[policy]:
                sequence+=1; rows.append({"sequence":sequence,"unit_id":"v3_07_b0_r1_{}_{}_{}".format(block["block_id"],policy,candidate),"block_id":block["block_id"],"role_evaluator_only":block["role"],"seed":int(block["seed"]),"angle_deg":float(block["angle_deg"]),"actor_start_distance_m":float(block["actor_start_distance_m"]),"actor_base_speed_mps":float(block["actor_base_speed_mps"]),"actor_policy_evaluator_only":policy,"candidate_id_evaluator_only":candidate,"ego_speed_mps":(1.2 if candidate=="spatial_left_v120" else 0.0),"attempt":1,"propensity":1.0/len(support[policy])})
    require(len(rows)==50,"R1 schedule differs")
    return {"schema_version":"1.0","stage":"V3-07-B0-R1","schedule_id":"v3_scope_v3_07_b0_r1_exact_schedule_1","status":"frozen_before_execution_release","row_count":50,"training_rows":30,"calibration_rows":20,"held_out_test_rows":0,"reserved_held_out_seeds_not_materialized":[6661,6662,6663,6664],"authorized_attempts_per_identity":1,"retry_allowed":False,"rows":rows,"schedule_digest":canonical_sha(rows)}
