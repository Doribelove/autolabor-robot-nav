"""V3-09-Q1 paired performance qualification bindings."""

import copy
import importlib.util
from pathlib import Path

W=Path("/home/robot/robot_ws_base_rl")
PARENT_PATH=W/"src/tools/thesis_experiment/src/thesis_experiment/v3_08_a3.py"
spec=importlib.util.spec_from_file_location("v3_08_a3_parent_for_v3_09_q1",PARENT_PATH)
PARENT=importlib.util.module_from_spec(spec); spec.loader.exec_module(PARENT)
for name in ("V301Error","require","sha256_bytes","canonical_sha","read_file_once","load_yaml_path","receipt","absolute_receipt","interpolate","interpolate_plan","check_receipts","load_trace"):
    globals()[name]=getattr(PARENT,name)
RAW_LOAD=PARENT.RAW_LOAD
BASE_YAML_BYTES=PARENT.BASE_YAML_BYTES
PARENT_CONTRACT=PARENT.CONTRACT_RELATIVE
CONTRACT_RELATIVE=Path("config/thesis_experiments/v3/v3_09_q1_paired_performance_qualification_contract.yaml")
PREREGISTRATION_RELATIVE=Path("experiments/manifests/v3/v3_09_q1_paired_performance_qualification_preregistration.yaml")
AUTHORIZATION_RELATIVE=Path("experiments/manifests/v3/v3_09_q1_simulation_authorization.yaml")
ROOT_RELATIVE=Path("artifacts/v3/qualification/v3_09_q1"); READINESS_RELATIVE=ROOT_RELATIVE/"offline_readiness"
SCHEDULE_RELATIVE=READINESS_RELATIVE/"v3_09_q1_exact_schedule.yaml"; BINDINGS_RELATIVE=READINESS_RELATIVE/"v3_09_q1_runtime_bindings.yaml"; CLOSURE_RELATIVE=READINESS_RELATIVE/"v3_09_q1_dependency_closure.yaml"; RELEASE_RELATIVE=READINESS_RELATIVE/"v3_09_q1_execution_release.yaml"; REVIEW_RELATIVE=READINESS_RELATIVE/"v3_09_q1_independent_release_review.yaml"; READINESS_FREEZE_RELATIVE=READINESS_RELATIVE/"v3_09_q1_readiness_freeze.sha256"
EXECUTION_RELATIVE=ROOT_RELATIVE/"execution"; JOURNAL_RELATIVE=EXECUTION_RELATIVE/"v3_09_q1_attempt_journal.jsonl"; ASSESSMENT_RELATIVE=EXECUTION_RELATIVE/"v3_09_q1_qualification_assessment.yaml"; EVIDENCE_FREEZE_RELATIVE=EXECUTION_RELATIVE/"v3_09_q1_evidence_freeze.sha256"; INDEPENDENT_RESULT_REVIEW_RELATIVE=EXECUTION_RELATIVE/"v3_09_q1_independent_result_review.yaml"; TERMINAL_FREEZE_RELATIVE=ROOT_RELATIVE/"v3_09_q1_terminal_freeze.sha256"
BINARY_RESOURCES=PARENT.BINARY_RESOURCES
SOURCE_RESOURCES=(
    CONTRACT_RELATIVE,PREREGISTRATION_RELATIVE,
    Path("artifacts/v3/planner/v3_08_a3/v3_08_a3_terminal_freeze.sha256"),
    Path("artifacts/v3/planner/v3_08_a0/v3_08_a0_effective_planner_contract.yaml"),
    Path("artifacts/v3/prediction/v3_07_b0_r1/model/v3_07_b0_r1_spatial_candidate_predictor.yaml"),
    Path("artifacts/v3/planner/v3_03_c5/v3_03_c5_effective_planner_contract.yaml"),
    Path("artifacts/v3/prediction/v3_02_d1/model/v3_02_d1_candidate_predictor.yaml"),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/core.py"),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/spatial_core.py"),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/spatial_core_r1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_09_dispatch.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_09_q1.py"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_08_a1_integration.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_09_q1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_09_q1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_09_q1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_09_q1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_09_q1_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_09_q1.py"),
)
def effective_contract():
    parent=copy.deepcopy(PARENT.effective_contract()); design=RAW_LOAD(W,CONTRACT_RELATIVE)
    parent["stage"]="V3-09-Q1"; parent["stage_purpose"]=design["stage_purpose"]; parent["contract_id"]=design["contract_id"]; parent["status"]=design["status"]
    parent["performance_design"]=design
    blocks=copy.deepcopy(design["fresh_population"]["scenario_blocks"])
    parent["experimental_unit"]["scenario_blocks"]=blocks; parent["experimental_unit"]["policies_evaluator_only"]=design["fresh_population"]["actor_policies_evaluator_only"]; parent["experimental_unit"]["methods"]=design["fresh_population"]["methods"]; parent["experimental_unit"]["schedule_rows"]=36; parent["experimental_unit"]["evidence_budget_units"]=36
    parent["scene"]["actor"]["endpoint_after_conflict_m"]=3.5
    parent["qualification_gates"]=design["qualification_gates"]
    parent["baseline_method"]=design["baseline_method"]
    return parent
def load_yaml_relative(workspace,relative):
    if Path(relative)==CONTRACT_RELATIVE: return effective_contract()
    return RAW_LOAD(workspace,relative)
def yaml_bytes(value):
    if isinstance(value,dict) and value.get("status")=="unit_complete" and value.get("stage")=="V3-08-A1":
        value=copy.deepcopy(value); value["stage"]="V3-09-Q1"
    return BASE_YAML_BYTES(value)
def validate_contract(contract,prereg):
    require(contract["stage"]=="V3-09-Q1" and contract["status"]=="qualification_preregistered_before_release","Q1 stage differs")
    require([x["seed"] for x in contract["experimental_unit"]["scenario_blocks"]]==list(range(6711,6717)) and contract["experimental_unit"]["schedule_rows"]==36,"Q1 population differs")
    require(contract["baseline_method"]["planner_sha256"]=="6f2ec1530b67785dfc8f5a397ab416847f35f0c6ffc9e6f45bbf633a50057761" and contract["baseline_method"]["actual_teb"] is False,"Q1 baseline differs")
    require(float(contract["scene"]["actor"]["endpoint_after_conflict_m"])==3.5 and float(contract["closed_loop"]["maximum_episode_duration_s"])==12.5,"Q1 protocol differs")
    require(prereg["status"]=="frozen_before_any_v3_09_q1_outcome_or_release" and prereg["failure_policy"]["no_formal_evaluation_if_any_qualification_gate_fails"] is True,"Q1 prereg differs")
def exact_schedule(contract):
    methods=["v3_spatial_safe","fixed_v120_shielded"]; rows=[]; sequence=0
    for bi,block in enumerate(contract["experimental_unit"]["scenario_blocks"]):
        for pi,policy in enumerate(contract["experimental_unit"]["policies_evaluator_only"]):
            offset=(bi+pi)%2
            for method in methods[offset:]+methods[:offset]:
                sequence+=1; rows.append({"sequence":sequence,"unit_id":"v3_09_q1_{}_{}_{}".format(block["block_id"],policy,method),"block_id":block["block_id"],"seed":int(block["seed"]),"angle_deg":float(block["angle_deg"]),"actor_start_distance_m":float(block["actor_start_distance_m"]),"actor_base_speed_mps":float(block["actor_base_speed_mps"]),"actor_policy_evaluator_only":policy,"method":method,"attempt":1})
    require(len(rows)==36,"Q1 row count differs")
    return {"schema_version":"1.0","stage":"V3-09-Q1","schedule_id":"v3_scope_v3_09_q1_exact_schedule_1","status":"frozen_before_execution_release","row_count":36,"episode_count":36,"block_count":6,"seeds":list(range(6711,6717)),"methods":methods,"authorized_attempts_per_identity":1,"retry_allowed":False,"rows":rows,"schedule_digest":canonical_sha(rows)}
