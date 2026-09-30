"""V3-08-A3 duration-only integration repair bindings."""

import copy
import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1_r1.py"
spec = importlib.util.spec_from_file_location("v3_08_a1_r1_parent_for_a3", PARENT_PATH)
PARENT = importlib.util.module_from_spec(spec)
spec.loader.exec_module(PARENT)
for name in (
    "V301Error", "require", "sha256_bytes", "canonical_sha", "read_file_once",
    "load_yaml_path", "receipt", "absolute_receipt", "interpolate",
    "interpolate_plan", "check_receipts", "load_trace",
):
    globals()[name] = getattr(PARENT, name)

RAW_LOAD = PARENT.load_yaml_relative
BASE_YAML_BYTES = PARENT.yaml_bytes
PARENT_CONTRACT = PARENT.CONTRACT_RELATIVE
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_08_a3_duration_repair_integration_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_08_a3_duration_repair_integration_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_08_a3_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/planner/v3_08_a3")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_08_a3_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_08_a3_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_08_a3_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_08_a3_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_08_a3_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_08_a3_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_08_a3_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_08_a3_integration_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_08_a3_evidence_freeze.sha256"
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_08_a3_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_08_a3_terminal_freeze.sha256"
BINARY_RESOURCES = PARENT.BINARY_RESOURCES
SOURCE_RESOURCES = (
    CONTRACT_RELATIVE, PREREGISTRATION_RELATIVE,
    Path("artifacts/v3/planner/v3_08_a1_r1/v3_08_a1_r1_terminal_freeze.sha256"),
    Path("artifacts/v3/analysis/v3_08_a2/v3_08_a2_terminal_freeze.sha256"),
    Path("artifacts/v3/planner/v3_08_a0/v3_08_a0_effective_planner_contract.yaml"),
    Path("artifacts/v3/prediction/v3_07_b0_r1/model/v3_07_b0_r1_spatial_candidate_predictor.yaml"),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/spatial_core.py"),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/spatial_core_r1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1_r1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_08_a3.py"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_08_a1_integration.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_08_a3_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_08_a3_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_08_a3_integration.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_08_a1_integration.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_08_a3_integration.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_08_a3_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_08_a3.py"),
)


def effective_contract():
    parent = copy.deepcopy(RAW_LOAD(W, PARENT_CONTRACT))
    repair = RAW_LOAD(W, CONTRACT_RELATIVE)
    parent["stage"] = "V3-08-A3"
    parent["stage_purpose"] = repair["stage_purpose"]
    parent["contract_id"] = repair["contract_id"]
    parent["status"] = repair["status"]
    parent["upstream_repair"] = repair["upstream"]
    parent["single_behavioral_factor"] = repair["single_behavioral_factor"]
    parent["mandatory_nonbehavioral_integrity_repair"] = repair[
        "mandatory_nonbehavioral_integrity_repair"
    ]
    parent["experimental_unit"]["scenario_blocks"] = repair["fresh_scenario_blocks"]
    parent["closed_loop"]["maximum_episode_duration_s"] = 12.5
    parent["closed_loop"]["maximum_decisions_per_episode"] = 24
    return parent


def load_yaml_relative(workspace, relative):
    if Path(relative) == CONTRACT_RELATIVE:
        return effective_contract()
    value = RAW_LOAD(workspace, relative)
    if (
        Path(relative).name == "episode_receipt.yaml"
        and isinstance(value, dict)
        and value.get("stage") == "V3-08-A3"
    ):
        value = copy.deepcopy(value)
        value["stage"] = "V3-08-A1"
    return value


def yaml_bytes(value):
    if (
        isinstance(value, dict)
        and value.get("status") == "unit_complete"
        and value.get("stage") == "V3-08-A1"
    ):
        value = copy.deepcopy(value)
        value["stage"] = "V3-08-A3"
    return BASE_YAML_BYTES(value)


def validate_contract(contract, preregistration):
    require(
        contract["stage"] == "V3-08-A3"
        and contract["status"]
        == "single_behavioral_factor_repair_preregistered_before_release",
        "A3 contract differs",
    )
    factor = contract["single_behavioral_factor"]
    require(
        factor["factor_id"] == "maximum_episode_duration"
        and float(factor["old_value_s"]) == 12.0
        and float(factor["new_value_s"]) == 12.5
        and int(factor["derived_maximum_decisions_new"]) == 24
        and all(
            factor[key] is False for key in (
                "scene_changed", "planner_changed", "predictor_changed",
                "hard_verifier_changed", "ranker_changed",
                "qualification_gates_changed",
            )
        ),
        "A3 factor differs",
    )
    require(
        [int(x["seed"]) for x in contract["experimental_unit"]["scenario_blocks"]]
        == [6691, 6692, 6693, 6694, 6695, 6696]
        and float(contract["closed_loop"]["maximum_episode_duration_s"]) == 12.5
        and int(contract["closed_loop"]["maximum_decisions_per_episode"]) == 24,
        "A3 population or protocol differs",
    )
    require(
        preregistration["status"]
        == "frozen_before_any_v3_08_a3_simulation_or_release"
        and preregistration["integrity_repair"][
            "receipt_stage_must_equal_v3_08_a3"
        ] is True,
        "A3 preregistration differs",
    )


def exact_schedule(contract):
    rows = []
    sequence = 0
    for block in contract["experimental_unit"]["scenario_blocks"]:
        for policy in contract["experimental_unit"]["policies_evaluator_only"]:
            sequence += 1
            rows.append({
                "sequence": sequence,
                "unit_id": "v3_08_a3_{}_{}".format(block["block_id"], policy),
                "block_id": block["block_id"], "seed": int(block["seed"]),
                "angle_deg": float(block["angle_deg"]),
                "actor_start_distance_m": float(block["actor_start_distance_m"]),
                "actor_base_speed_mps": float(block["actor_base_speed_mps"]),
                "actor_policy_evaluator_only": policy, "attempt": 1,
            })
    require(len(rows) == 18, "A3 row count differs")
    return {
        "schema_version": "1.0", "stage": "V3-08-A3",
        "schedule_id": "v3_scope_v3_08_a3_exact_schedule_1",
        "status": "frozen_before_execution_release", "row_count": 18,
        "episode_count": 18,
        "seeds": [6691, 6692, 6693, 6694, 6695, 6696],
        "authorized_attempts_per_identity": 1, "retry_allowed": False,
        "rows": rows, "schedule_digest": canonical_sha(rows),
    }
