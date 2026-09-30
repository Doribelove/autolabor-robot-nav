"""V3-02-C1 common-prefix counterfactual predictor bindings."""

import importlib.util
from pathlib import Path
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
)
_SPEC = importlib.util.spec_from_file_location("v3_02_base", BASE_PATH)
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
    "render_world",
    "feature_vector",
    "trace_metrics",
):
    globals()[_name] = getattr(BASE, _name)

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_02_c1_common_prefix_data_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_02_c1_common_prefix_data_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_02_c1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/prediction/v3_02_c1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_02_c1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_02_c1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_02_c1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_02_c1_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_02_c1_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_02_c1_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_02_c1_attempt_journal.jsonl"
MODEL_RELATIVE = ROOT_RELATIVE / "model/v3_02_c1_candidate_predictor.yaml"
CALIBRATION_RELATIVE = ROOT_RELATIVE / "model/v3_02_c1_calibration_report.yaml"
MODEL_FREEZE_RELATIVE = ROOT_RELATIVE / "model/v3_02_c1_model_freeze.sha256"
INDEPENDENT_MODEL_REVIEW_RELATIVE = (
    ROOT_RELATIVE / "model/v3_02_c1_independent_model_review.yaml"
)
TRACE_METRICS_NEEDS_ROW = True
PHYSICAL_CLIP = True
CONFORMAL_SCORE = "actual_mode_absolute_residual"

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/prediction/v3_02_t2/"
        "v3_03_preintegration_alignment_review.yaml"
    ),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
    ),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_c1.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_model.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_c1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_c1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_02_c1_data.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_c1.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_c1_model.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_02_c1.py"),
)
BINARY_RESOURCES = BASE.BINARY_RESOURCES


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-02-C1", "stage differs")
    require(
        contract["status"] == "preregistered_before_simulation",
        "status differs",
    )
    unit = contract["experimental_unit"]
    require(
        len(unit["scenario_blocks"]) == 9
        and unit["schedule_rows"] == 108
        and unit["evidence_budget_units"] == 108
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "unit roster differs",
    )
    prefix = contract["scene"]["ego"]["common_prefix"]
    require(
        float(prefix["duration_s"]) == 0.5
        and float(prefix["speed_mps"]) == 0.8
        and float(prefix["end_x_m"]) == -2.6,
        "common prefix differs",
    )
    design = contract["predictor_design"]
    require(
        design["conformal_score"] == "actual_mode_absolute_residual"
        and float(design["physical_coordinate_clip"]["upper_bound_m"]) == 3.5,
        "predictor safeguards differ",
    )
    require(
        contract["authority_boundary"]["held_out_test_seeds_reserved"]
        == [6261, 6262, 6263]
        and contract["authority_boundary"]["held_out_test_access_authorized"]
        is False,
        "test firewall differs",
    )
    require(
        preregistration["stage"] == "V3-02-C1"
        and preregistration["status"]
        == "frozen_before_any_v3_02_c1_gazebo_execution"
        and preregistration["candidate_atomicity"]["branch_start_s"] == 0.5
        and preregistration["held_out_firewall"]["test_outcome_access_allowed"]
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
                        "unit_id": "v3_02_c1_{}_{}_{}".format(
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
                        "ego_speed_mps": float(candidate["speed_mps"]),
                        "attempt": 1,
                        "propensity": 0.25,
                    }
                )
    return {
        "schema_version": "1.0",
        "stage": "V3-02-C1",
        "schedule_id": "v3_scope_v3_02_c1_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 108,
        "training_rows": 72,
        "calibration_rows": 36,
        "held_out_test_rows": 0,
        "reserved_held_out_seeds_not_materialized": [6261, 6262, 6263],
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
