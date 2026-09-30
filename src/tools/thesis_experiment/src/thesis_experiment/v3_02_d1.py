"""V3-02-D1 acceleration-feasible predictor-data bindings."""

import importlib.util
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
)
_SPEC = importlib.util.spec_from_file_location("v3_02_base_for_d1", BASE_PATH)
BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(BASE)

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
    "ego_waypoints",
    "render_world",
    "feature_vector",
    "trace_metrics",
):
    globals()[_name] = getattr(BASE, _name)

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_02_d1_feasible_intervention_data_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_02_d1_feasible_intervention_data_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_02_d1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/prediction/v3_02_d1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_02_d1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_02_d1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_02_d1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_02_d1_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_02_d1_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_02_d1_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_02_d1_attempt_journal.jsonl"
DATA_QUALIFICATION_RELATIVE = (
    ROOT_RELATIVE / "execution/v3_02_d1_data_qualification.yaml"
)
MODEL_RELATIVE = ROOT_RELATIVE / "model/v3_02_d1_candidate_predictor.yaml"
CALIBRATION_RELATIVE = ROOT_RELATIVE / "model/v3_02_d1_calibration_report.yaml"
MODEL_FREEZE_RELATIVE = ROOT_RELATIVE / "model/v3_02_d1_model_freeze.sha256"
INDEPENDENT_MODEL_REVIEW_RELATIVE = (
    ROOT_RELATIVE / "model/v3_02_d1_independent_model_review.yaml"
)
TRACE_METRICS_NEEDS_ROW = True
PHYSICAL_CLIP = True
CONFORMAL_SCORE = "actual_mode_absolute_residual"

FEATURE_NAMES = [
    "bias",
    "actor_coordinate",
    "actor_speed",
    "candidate_speed",
    "actor_coordinate_squared",
    "actor_speed_squared",
    "candidate_speed_squared",
    "actor_coordinate_x_actor_speed",
    "actor_coordinate_x_candidate_speed",
    "actor_speed_x_candidate_speed",
]

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/prediction/v3_02_c2/"
        "v3_02_c2_terminal_freeze.sha256"
    ),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
    ),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_model.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_d1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_d1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_02_d1_data.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_02_d1_data.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_d1.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_d1_model.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_02_d1.py"),
)
BINARY_RESOURCES = BASE.BINARY_RESOURCES


def model_features(metrics):
    coordinate = float(metrics["actor_conflict_coordinate_at_observation_m"])
    actor_speed = float(metrics["actor_observed_speed_mps"])
    candidate_speed = float(metrics["candidate_ego_speed_mps"])
    return [
        1.0,
        coordinate,
        actor_speed,
        candidate_speed,
        coordinate * coordinate,
        actor_speed * actor_speed,
        candidate_speed * candidate_speed,
        coordinate * actor_speed,
        coordinate * candidate_speed,
        actor_speed * candidate_speed,
    ]


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-02-D1", "stage differs")
    require(
        contract["status"] == "preregistered_before_simulation",
        "status differs",
    )
    unit = contract["experimental_unit"]
    require(
        len(unit["scenario_blocks"]) == 9
        and len(unit["ego_interventions"]) == 5
        and unit["schedule_rows"] == 135
        and unit["training_rows"] == 90
        and unit["calibration_rows"] == 45
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "data roster differs",
    )
    require(
        [float(row["speed_mps"]) for row in unit["ego_interventions"]]
        == [0.0, 0.6, 0.8, 1.0, 1.2]
        and unit["ego_interventions"][0]["certified_stop"] is True,
        "candidate support differs",
    )
    branch = contract["scene"]["ego"]["feasible_branch"]
    require(
        float(branch["initial_speed_mps"]) == 0.8
        and float(branch["maximum_acceleration_mps2"]) == 0.6
        and float(branch["maximum_deceleration_mps2"]) == 0.8
        and float(branch["ramp_waypoint_period_s"]) == 0.05,
        "feasible branch differs",
    )
    require(
        contract["predictor_design"]["scalar_features"] == FEATURE_NAMES
        and contract["predictor_design"]["conformal_score"]
        == "actual_mode_absolute_residual",
        "predictor differs",
    )
    require(
        contract["authority_boundary"]["held_out_test_seeds_reserved"]
        == [6281, 6282, 6283]
        and contract["authority_boundary"]["held_out_test_access_authorized"]
        is False,
        "test firewall differs",
    )
    require(
        preregistration["stage"] == "V3-02-D1"
        and preregistration["status"]
        == "frozen_before_any_v3_02_d1_gazebo_execution"
        and preregistration["schedule"]["exact_rows"] == 135
        and preregistration["candidate_atomicity"][
            "certified_stop_in_support"
        ]
        is True
        and preregistration["held_out_firewall"][
            "test_outcome_access_allowed"
        ]
        is False,
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
                        "unit_id": "v3_02_d1_{}_{}_{}".format(
                            block["block_id"], policy, candidate["candidate_id"]
                        ),
                        "block_id": block["block_id"],
                        "role_evaluator_only": block["role"],
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
    require(len(rows) == 135, "schedule count differs")
    return {
        "schema_version": "1.0",
        "stage": "V3-02-D1",
        "schedule_id": "v3_scope_v3_02_d1_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 135,
        "training_rows": 90,
        "calibration_rows": 45,
        "held_out_test_rows": 0,
        "reserved_held_out_seeds_not_materialized": [6281, 6282, 6283],
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }


def interpolate_plan(
    points: Sequence[Sequence[Any]], time_s: float
) -> Sequence[float]:
    if time_s <= float(points[0][0]):
        return points[0][1]
    for left, right in zip(points, points[1:]):
        if time_s <= float(right[0]) + 1e-12:
            span = float(right[0]) - float(left[0])
            ratio = (time_s - float(left[0])) / span
            return [
                float(left[1][index])
                + ratio * (float(right[1][index]) - float(left[1][index]))
                for index in range(3)
            ]
    return points[-1][1]


def planned_kinematics(
    contract: Mapping[str, Any], row: Mapping[str, Any]
) -> Dict[str, float]:
    points = ego_waypoints(contract, row)
    segments = []
    for left, right in zip(points, points[1:]):
        dt = float(right[0]) - float(left[0])
        require(dt > 0.0, "non-monotonic ego waypoint")
        segments.append(
            {
                "mid_time": 0.5 * (float(left[0]) + float(right[0])),
                "speed": (
                    float(right[1][0]) - float(left[1][0])
                ) / dt,
            }
        )
    accelerations = []
    for left, right in zip(segments, segments[1:]):
        dt = right["mid_time"] - left["mid_time"]
        accelerations.append((right["speed"] - left["speed"]) / dt)
    return {
        "maximum_speed_mps": max(item["speed"] for item in segments),
        "minimum_speed_mps": min(item["speed"] for item in segments),
        "maximum_acceleration_mps2": max(accelerations + [0.0]),
        "maximum_deceleration_mps2": max(
            [-value for value in accelerations] + [0.0]
        ),
        "terminal_x_m": float(points[-1][1][0]),
    }
