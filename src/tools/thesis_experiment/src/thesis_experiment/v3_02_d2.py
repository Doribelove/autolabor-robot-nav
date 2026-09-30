"""V3-02-D2 feasible-intervention frozen-predictor held-out bindings."""

import importlib.util
import math
from pathlib import Path
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_t1.py"
)
_PARENT_SPEC = importlib.util.spec_from_file_location(
    "v3_02_t1_base_for_d2", PARENT_PATH
)
PARENT = importlib.util.module_from_spec(_PARENT_SPEC)
_PARENT_SPEC.loader.exec_module(PARENT)

D1_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"
)
_D1_SPEC = importlib.util.spec_from_file_location("v3_02_d1_for_d2", D1_PATH)
D1 = importlib.util.module_from_spec(_D1_SPEC)
_D1_SPEC.loader.exec_module(D1)

for _name in (
    "V301Error",
    "require",
    "sha256_bytes",
    "yaml_bytes",
    "canonical_sha",
    "read_file_once",
    "load_yaml_path",
    "load_yaml_relative",
    "receipt",
    "absolute_receipt",
    "load_trace",
    "interpolate",
    "crossing_time",
    "check_receipts",
    "actor_direction",
    "render_world",
    "feature_vector",
    "trace_metrics",
):
    globals()[_name] = getattr(PARENT, _name)

model_features = D1.model_features
ego_waypoints = D1.ego_waypoints
interpolate_plan = D1.interpolate_plan
planned_kinematics = D1.planned_kinematics

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_02_d2_feasible_intervention_heldout_test_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_02_d2_feasible_intervention_heldout_test_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_02_d2_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/prediction/v3_02_d2")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_02_d2_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_02_d2_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_02_d2_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_02_d2_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_02_d2_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_02_d2_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_02_d2_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_02_d2_heldout_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_02_d2_evidence_freeze.sha256"
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_02_d2_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_02_d2_terminal_freeze.sha256"
TRACE_METRICS_NEEDS_ROW = True

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_calibration_report.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_model_freeze.sha256"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_independent_model_review.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/execution/"
        "v3_02_d1_data_qualification.yaml"
    ),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
    ),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_t1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_d2.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_02_t1.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t1_result.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_d2_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_d2_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_02_d2_test.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_02_d2.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_d2_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_02_d2.py"),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-02-D2", "stage differs")
    require(
        contract["status"] == "preregistered_before_test_release",
        "status differs",
    )
    model = contract["frozen_model"]
    require(
        model["sha256"]
        == "fa4ef83b9b2e534da3031001f5dd0036cd7f983ebfbe73fe20f45c4b73bc55d7"
        and model["feature_basis"] == "complete_quadratic_10_features"
        and model["feasible_generator_id"]
        == "v3_02_d1_straight_acceleration_limited_1"
        and model["refit_or_recalibration_allowed"] is False,
        "frozen model differs",
    )
    unit = contract["experimental_unit"]
    require(
        [row["seed"] for row in unit["scenario_blocks"]]
        == [6281, 6282, 6283]
        and len(unit["ego_interventions"]) == 5
        and unit["schedule_rows"] == 45
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "test roster differs",
    )
    require(
        unit["ego_interventions"][0]["certified_stop"] is True
        and [float(row["speed_mps"]) for row in unit["ego_interventions"]]
        == [0.0, 0.6, 0.8, 1.0, 1.2],
        "candidate support differs",
    )
    branch = contract["scene"]["ego"]["feasible_branch"]
    require(
        branch["generator_id"] == model["feasible_generator_id"]
        and float(branch["maximum_acceleration_mps2"]) == 0.6
        and float(branch["maximum_deceleration_mps2"]) == 0.8,
        "feasible generator differs",
    )
    require(
        contract["authority_boundary"]["model_write_allowed"] is False
        and contract["authority_boundary"]["prior_seed_or_outcome_reuse_allowed"]
        is False
        and contract["failure_policy"]["no_gate_change"] is True,
        "test firewall differs",
    )
    require(
        preregistration["stage"] == "V3-02-D2"
        and preregistration["status"]
        == "frozen_before_any_feasible_intervention_heldout_outcome"
        and preregistration["test"]["seeds"] == [6281, 6282, 6283]
        and preregistration["candidate_atomicity"][
            "certified_stop_in_support"
        ]
        is True
        and preregistration["model_firewall"]["refit_allowed"] is False,
        "preregistration differs",
    )


def exact_schedule(contract: Mapping[str, Any]) -> Dict[str, Any]:
    unit = contract["experimental_unit"]
    rows = []
    sequence = 0
    for block in unit["scenario_blocks"]:
        for policy in unit["policies"]:
            for candidate in unit["ego_interventions"]:
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "unit_id": "v3_02_d2_{}_{}_{}".format(
                            block["block_id"], policy, candidate["candidate_id"]
                        ),
                        "block_id": block["block_id"],
                        "role_evaluator_only": "held_out_test",
                        "seed": block["seed"],
                        "angle_deg": float(block["angle_deg"]),
                        "actor_start_distance_m": float(
                            block["actor_start_distance_m"]
                        ),
                        "actor_base_speed_mps": float(
                            block["actor_base_speed_mps"]
                        ),
                        "actor_policy_evaluator_only": policy,
                        "candidate_id_evaluator_only": candidate["candidate_id"],
                        "certified_stop": bool(candidate["certified_stop"]),
                        "ego_speed_mps": float(candidate["speed_mps"]),
                        "attempt": 1,
                        "propensity": 0.2,
                    }
                )
    return {
        "schema_version": "1.0",
        "stage": "V3-02-D2",
        "schedule_id": "v3_scope_v3_02_d2_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 45,
        "held_out_test_rows": 45,
        "seeds": [6281, 6282, 6283],
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }


def heldout_additional_assessment(contract, schedule, episodes):
    maximum_trace_error = 0.0
    maximum_acceleration = 0.0
    maximum_deceleration = 0.0
    stop_terminal_error = 0.0
    stop_post_terminal_motion = 0.0
    for episode in episodes:
        row = episode["row"]
        root = WORKSPACE / EXECUTION_RELATIVE / "attempts" / row["unit_id"]
        trace = load_trace(root / "raw_pose_trace.csv")
        points = ego_waypoints(contract, row)
        trace_error = max(
            math.hypot(
                raw["ego_x_m"]
                - float(interpolate_plan(points, raw["sim_time_s"])[0]),
                raw["ego_y_m"]
                - float(interpolate_plan(points, raw["sim_time_s"])[1]),
            )
            for raw in trace
        )
        kinematics = planned_kinematics(contract, row)
        maximum_trace_error = max(maximum_trace_error, trace_error)
        maximum_acceleration = max(
            maximum_acceleration, kinematics["maximum_acceleration_mps2"]
        )
        maximum_deceleration = max(
            maximum_deceleration, kinematics["maximum_deceleration_mps2"]
        )
        if row["certified_stop"]:
            stop_terminal_error = max(
                stop_terminal_error,
                abs(trace[-1]["ego_x_m"] - kinematics["terminal_x_m"]),
            )
            post = [
                item["ego_x_m"]
                for item in trace
                if item["sim_time_s"] >= 1.5 - 1e-9
            ]
            stop_post_terminal_motion = max(
                stop_post_terminal_motion, max(post) - min(post)
            )
    metrics = {
        "maximum_ego_trace_position_error_m": maximum_trace_error,
        "maximum_planned_acceleration_mps2": maximum_acceleration,
        "maximum_planned_deceleration_mps2": maximum_deceleration,
        "maximum_stop_terminal_position_error_m": stop_terminal_error,
        "maximum_stop_terminal_motion_after_1_5_s_m": (
            stop_post_terminal_motion
        ),
    }
    thresholds = contract["heldout_gates"]
    gates = {
        "ego_trace_conformity_pass": maximum_trace_error
        <= float(thresholds["maximum_ego_trace_position_error_m"]),
        "planned_acceleration_pass": maximum_acceleration
        <= float(thresholds["maximum_planned_acceleration_mps2"]),
        "planned_deceleration_pass": maximum_deceleration
        <= float(thresholds["maximum_planned_deceleration_mps2"]),
        "stop_terminal_position_pass": stop_terminal_error
        <= float(thresholds["stop_terminal_position_error_m"]),
        "stop_terminal_hold_pass": stop_post_terminal_motion
        <= float(thresholds["stop_terminal_motion_after_1_5_s_m"]),
    }
    return {"metrics": metrics, "gates": gates}
