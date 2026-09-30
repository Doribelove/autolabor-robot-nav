"""V3-07-B1 fresh held-out spatial predictor validation bindings."""

import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0_r1.py"
spec = importlib.util.spec_from_file_location("v3_07_b0_r1_parent_for_b1", PARENT_PATH)
PARENT = importlib.util.module_from_spec(spec)
spec.loader.exec_module(PARENT)
for name in (
    "V301Error", "require", "sha256_bytes", "yaml_bytes", "canonical_sha",
    "read_file_once", "load_yaml_path", "load_yaml_relative", "receipt",
    "absolute_receipt", "load_trace", "interpolate", "check_receipts",
    "actor_direction", "render_world", "trace_metrics",
):
    globals()[name] = getattr(PARENT, name)

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_07_b1_heldout_predictor_validation_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_07_b1_heldout_predictor_validation_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_07_b1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/prediction/v3_07_b1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_07_b1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_07_b1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_07_b1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_07_b1_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_07_b1_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_07_b1_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_07_b1_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_07_b1_heldout_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_07_b1_evidence_freeze.sha256"
RESULT_REVIEW_RELATIVE = EXECUTION_RELATIVE / "v3_07_b1_independent_result_review.yaml"
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_07_b1_terminal_freeze.sha256"
MODEL_RELATIVE = Path(
    "artifacts/v3/prediction/v3_07_b0_r1/model/"
    "v3_07_b0_r1_spatial_candidate_predictor.yaml"
)
MODEL_FREEZE_RELATIVE = Path(
    "artifacts/v3/prediction/v3_07_b0_r1/model/v3_07_b0_r1_model_freeze.sha256"
)
TRACE_METRICS_NEEDS_ROW = True
BINARY_RESOURCES = PARENT.BINARY_RESOURCES
SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path("artifacts/v3/prediction/v3_07_b0_r1/v3_07_b0_r1_terminal_freeze.sha256"),
    MODEL_RELATIVE,
    MODEL_FREEZE_RELATIVE,
    Path("artifacts/v3/prediction/v3_07_b0_r1/model/v3_07_b0_r1_independent_model_review.yaml"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_spatial_candidates.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_a3.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_b0_r1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_b1.py"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_07_b1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_07_b1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_07_b1_heldout.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_07_b1_heldout.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_07_b1_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_07_b1.py"),
)


def validate_contract(contract, preregistration):
    require(
        contract["stage"] == "V3-07-B1"
        and contract["status"] == "preregistered_before_heldout_access",
        "B1 contract differs",
    )
    unit = contract["experimental_unit"]
    require(
        len(unit["scenario_blocks"]) == 4
        and unit["schedule_rows"] == 20
        and unit["held_out_test_rows"] == 20
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "B1 population differs",
    )
    require(
        [int(x["seed"]) for x in unit["scenario_blocks"]]
        == [6661, 6662, 6663, 6664],
        "B1 held-out seeds differ",
    )
    upstream = contract["upstream"]
    require(
        upstream["frozen_model_sha256"]
        == "38d8c0fa7c7a696fb9ffdd5c3ce399d760d52fc42a0c6041525c5d70f7445fd4"
        and float(upstream["frozen_conformal_radius_m"])
        == 0.4967404086874341,
        "B1 frozen model binding differs",
    )
    require(
        preregistration["status"]
        == "frozen_before_any_v3_07_b1_heldout_access_or_gazebo_execution"
        and preregistration["model"]["refit_on_heldout_forbidden"] is True
        and preregistration["evaluation"]["global_coverage_min"] == 0.95
        and preregistration["failure_policy"]["no_planner_integration_if_any_gate_fails"]
        is True,
        "B1 preregistration differs",
    )
    require(
        contract["authority_boundary"][
            "held_out_test_seeds_released_only_by_separate_authorization"
        ] == [6661, 6662, 6663, 6664]
        and contract["authority_boundary"]["reinforcement_learning_authorized"]
        is False
        and contract["authority_boundary"]["real_vehicle_authorized"] is False,
        "B1 authority differs",
    )


def exact_schedule(contract):
    rows = []
    sequence = 0
    support = contract["data_support"]["eligible_cells"]
    for block in contract["experimental_unit"]["scenario_blocks"]:
        for policy in contract["experimental_unit"]["policies_evaluator_only"]:
            for candidate in support[policy]:
                sequence += 1
                rows.append({
                    "sequence": sequence,
                    "unit_id": "v3_07_b1_{}_{}_{}".format(
                        block["block_id"], policy, candidate
                    ),
                    "block_id": block["block_id"],
                    "role_evaluator_only": "held_out_test",
                    "seed": int(block["seed"]),
                    "angle_deg": float(block["angle_deg"]),
                    "actor_start_distance_m": float(block["actor_start_distance_m"]),
                    "actor_base_speed_mps": float(block["actor_base_speed_mps"]),
                    "actor_policy_evaluator_only": policy,
                    "candidate_id_evaluator_only": candidate,
                    "ego_speed_mps": 1.2 if candidate == "spatial_left_v120" else 0.0,
                    "attempt": 1,
                    "propensity": 1.0 / len(support[policy]),
                })
    require(len(rows) == 20, "B1 schedule differs")
    return {
        "schema_version": "1.0",
        "stage": "V3-07-B1",
        "schedule_id": "v3_scope_v3_07_b1_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 20,
        "training_rows": 0,
        "calibration_rows": 0,
        "held_out_test_rows": 20,
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
