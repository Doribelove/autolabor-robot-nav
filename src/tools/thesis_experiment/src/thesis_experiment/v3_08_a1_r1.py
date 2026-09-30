"""V3-08-A1-R1 finite diagnostic repair bindings."""

import copy
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1.py"
spec = importlib.util.spec_from_file_location("v3_08_a1_parent_for_r1", PARENT_PATH)
PARENT = importlib.util.module_from_spec(spec)
spec.loader.exec_module(PARENT)
for name in (
    "V301Error", "require", "sha256_bytes", "yaml_bytes", "canonical_sha",
    "read_file_once", "load_yaml_path", "receipt", "absolute_receipt",
    "interpolate", "interpolate_plan", "check_receipts", "load_trace",
):
    globals()[name] = getattr(PARENT, name)

BASE_LOAD = PARENT.load_yaml_relative
PARENT_CONTRACT = PARENT.CONTRACT_RELATIVE
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_08_a1_r1_finite_diagnostic_repair_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_08_a1_r1_finite_diagnostic_repair_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_08_a1_r1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/planner/v3_08_a1_r1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_08_a1_r1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_08_a1_r1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_08_a1_r1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_08_a1_r1_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_08_a1_r1_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_08_a1_r1_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_08_a1_r1_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_08_a1_r1_integration_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_08_a1_r1_evidence_freeze.sha256"
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_08_a1_r1_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_08_a1_r1_terminal_freeze.sha256"
BINARY_RESOURCES = PARENT.BINARY_RESOURCES
SOURCE_RESOURCES = (
    CONTRACT_RELATIVE, PREREGISTRATION_RELATIVE, PARENT_CONTRACT,
    Path("artifacts/v3/planner/v3_08_a1/v3_08_a1_terminal_freeze.sha256"),
    Path("artifacts/v3/planner/v3_08_a0/v3_08_a0_terminal_freeze.sha256"),
    Path("artifacts/v3/planner/v3_08_a0/v3_08_a0_effective_planner_contract.yaml"),
    Path("artifacts/v3/prediction/v3_07_b0_r1/model/v3_07_b0_r1_spatial_candidate_predictor.yaml"),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/spatial_core.py"),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/spatial_core_r1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1_r1.py"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_08_a1_integration.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_08_a1_r1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_08_a1_r1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_08_a1_r1_integration.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_08_a1_integration.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_08_a1_r1_integration.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_08_a1_r1_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_08_a1_r1.py"),
)


def effective_contract():
    parent = copy.deepcopy(BASE_LOAD(W, PARENT_CONTRACT))
    repair = BASE_LOAD(W, CONTRACT_RELATIVE)
    parent["stage"] = "V3-08-A1-R1"
    parent["stage_purpose"] = repair["stage_purpose"]
    parent["contract_id"] = repair["contract_id"]
    parent["status"] = repair["status"]
    parent["parent_failure"] = repair["parent_failure"]
    parent["single_factor_repair"] = repair["single_factor_repair"]
    parent["experimental_unit"]["scenario_blocks"] = repair["fresh_scenario_blocks"]
    return parent


def load_yaml_relative(workspace, relative):
    if Path(relative) == CONTRACT_RELATIVE:
        return effective_contract()
    return BASE_LOAD(workspace, relative)


def validate_contract(contract, preregistration):
    require(
        contract["stage"] == "V3-08-A1-R1"
        and contract["status"] == "single_factor_repair_preregistered_before_release",
        "R1 contract differs",
    )
    factor = contract["single_factor_repair"]
    require(
        factor["factor_id"]
        == "rejected_candidate_nonfinite_diagnostic_to_finite_sentinel"
        and float(factor["new_value"]) == -1.0e9
        and all(
            factor[key] is False for key in (
                "candidate_set_changed", "acceptance_or_rejection_changed",
                "selected_candidate_changed", "trajectory_changed",
                "predictor_changed", "hard_verifier_changed", "ranker_changed",
                "scene_or_protocol_changed", "qualification_gates_changed",
            )
        ),
        "R1 factor differs",
    )
    require(
        [int(x["seed"]) for x in contract["experimental_unit"]["scenario_blocks"]]
        == [6681, 6682, 6683, 6684, 6685, 6686]
        and contract["experimental_unit"]["schedule_rows"] == 18,
        "R1 schedule differs",
    )
    require(
        preregistration["status"]
        == "frozen_before_any_v3_08_a1_r1_simulation_or_release"
        and preregistration["single_factor"][
            "selection_trajectory_model_verifier_ranker_scene_protocol_gates_unchanged"
        ] is True,
        "R1 preregistration differs",
    )


def exact_schedule(contract):
    rows = []
    sequence = 0
    for block in contract["experimental_unit"]["scenario_blocks"]:
        for policy in contract["experimental_unit"]["policies_evaluator_only"]:
            sequence += 1
            rows.append({
                "sequence": sequence,
                "unit_id": "v3_08_a1_r1_{}_{}".format(block["block_id"], policy),
                "block_id": block["block_id"], "seed": int(block["seed"]),
                "angle_deg": float(block["angle_deg"]),
                "actor_start_distance_m": float(block["actor_start_distance_m"]),
                "actor_base_speed_mps": float(block["actor_base_speed_mps"]),
                "actor_policy_evaluator_only": policy, "attempt": 1,
            })
    require(len(rows) == 18, "R1 row count differs")
    return {
        "schema_version": "1.0", "stage": "V3-08-A1-R1",
        "schedule_id": "v3_scope_v3_08_a1_r1_exact_schedule_1",
        "status": "frozen_before_execution_release", "row_count": 18,
        "episode_count": 18,
        "seeds": [6681, 6682, 6683, 6684, 6685, 6686],
        "authorized_attempts_per_identity": 1, "retry_allowed": False,
        "rows": rows, "schedule_digest": canonical_sha(rows),
    }
