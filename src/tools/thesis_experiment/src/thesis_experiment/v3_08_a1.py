"""V3-08-A1 fresh spatial planner integration bindings."""

import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c6.py"
spec = importlib.util.spec_from_file_location("v3_03_c6_parent_for_v3_08_a1", PARENT_PATH)
PARENT = importlib.util.module_from_spec(spec)
spec.loader.exec_module(PARENT)
for name in (
    "V301Error", "require", "sha256_bytes", "yaml_bytes", "canonical_sha",
    "read_file_once", "load_yaml_path", "load_yaml_relative", "receipt",
    "absolute_receipt", "interpolate", "interpolate_plan", "check_receipts",
    "load_trace",
):
    globals()[name] = getattr(PARENT, name)

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_08_a1_spatial_planner_integration_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_08_a1_spatial_planner_integration_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_08_a1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/planner/v3_08_a1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_08_a1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_08_a1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_08_a1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_08_a1_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_08_a1_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_08_a1_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_08_a1_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_08_a1_integration_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_08_a1_evidence_freeze.sha256"
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_08_a1_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_08_a1_terminal_freeze.sha256"
BINARY_RESOURCES = PARENT.BINARY_RESOURCES
SOURCE_RESOURCES = (
    CONTRACT_RELATIVE, PREREGISTRATION_RELATIVE,
    Path("artifacts/v3/planner/v3_08_a0/v3_08_a0_terminal_freeze.sha256"),
    Path("artifacts/v3/planner/v3_08_a0/v3_08_a0_effective_planner_contract.yaml"),
    Path("artifacts/v3/prediction/v3_07_b1/v3_07_b1_terminal_freeze.sha256"),
    Path("artifacts/v3/prediction/v3_07_b0_r1/model/v3_07_b0_r1_spatial_candidate_predictor.yaml"),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/spatial_core.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1.py"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_08_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_08_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_08_a1_integration.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_08_a1_integration.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_08_a1_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_08_a1.py"),
)


def validate_contract(contract, preregistration):
    require(
        contract["stage"] == "V3-08-A1"
        and contract["status"] == "integration_preregistered_before_release",
        "A1 contract differs",
    )
    unit = contract["experimental_unit"]
    require(
        [int(x["seed"]) for x in unit["scenario_blocks"]]
        == [6671, 6672, 6673, 6674, 6675, 6676]
        and unit["schedule_rows"] == 18
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "A1 population differs",
    )
    loop = contract["closed_loop"]
    require(
        float(loop["observation_prefix_s"]) == 0.5
        and float(loop["replanning_period_s"]) == 0.5
        and float(loop["spatial_commitment_duration_s"]) == 7.0
        and float(loop["maximum_episode_duration_s"]) == 12.0,
        "A1 protocol differs",
    )
    require(
        preregistration["status"]
        == "frozen_before_any_v3_08_a1_simulation_or_release"
        and preregistration["fresh_schedule"]["seeds"]
        == [6671, 6672, 6673, 6674, 6675, 6676]
        and preregistration["failure_policy"][
            "no_confirmatory_performance_stage_if_any_gate_fails"
        ] is True,
        "A1 preregistration differs",
    )
    require(
        contract["upstream"]["planner_contract_sha256"]
        == "28cd688d3116114a065bcecdf27e94c0499fbd42a1c8dbe5bd37f89eab7f568c"
        and contract["frozen_model"]["sha256"]
        == "38d8c0fa7c7a696fb9ffdd5c3ce399d760d52fc42a0c6041525c5d70f7445fd4",
        "A1 planner or model differs",
    )


def exact_schedule(contract):
    rows = []
    sequence = 0
    for block in contract["experimental_unit"]["scenario_blocks"]:
        for policy in contract["experimental_unit"]["policies_evaluator_only"]:
            sequence += 1
            rows.append({
                "sequence": sequence,
                "unit_id": "v3_08_a1_{}_{}".format(block["block_id"], policy),
                "block_id": block["block_id"], "seed": int(block["seed"]),
                "angle_deg": float(block["angle_deg"]),
                "actor_start_distance_m": float(block["actor_start_distance_m"]),
                "actor_base_speed_mps": float(block["actor_base_speed_mps"]),
                "actor_policy_evaluator_only": policy, "attempt": 1,
            })
    require(len(rows) == 18, "A1 schedule differs")
    return {
        "schema_version": "1.0", "stage": "V3-08-A1",
        "schedule_id": "v3_scope_v3_08_a1_exact_schedule_1",
        "status": "frozen_before_execution_release", "row_count": 18,
        "episode_count": 18,
        "seeds": [6671, 6672, 6673, 6674, 6675, 6676],
        "authorized_attempts_per_identity": 1, "retry_allowed": False,
        "rows": rows, "schedule_digest": canonical_sha(rows),
    }
