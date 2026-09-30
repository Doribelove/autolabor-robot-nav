"""V3-02-C1-R1 complete-quadratic ridge basis repair."""

import copy
import importlib.util
from pathlib import Path
from typing import Any, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_c1.py"
)
_SPEC = importlib.util.spec_from_file_location("v3_02_c1_base", PARENT_PATH)
PARENT = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(PARENT)

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
    "exact_schedule",
    "actor_direction",
    "render_world",
    "feature_vector",
    "trace_metrics",
):
    globals()[_name] = getattr(PARENT, _name)

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_02_c1_r1_quadratic_basis_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_02_c1_r1_quadratic_basis_preregistration.yaml"
)
PARENT_CONTRACT_RELATIVE = PARENT.CONTRACT_RELATIVE
SCHEDULE_RELATIVE = PARENT.SCHEDULE_RELATIVE
EXECUTION_RELATIVE = PARENT.EXECUTION_RELATIVE
JOURNAL_RELATIVE = PARENT.JOURNAL_RELATIVE
ROOT_RELATIVE = Path("artifacts/v3/prediction/v3_02_c1_r1")
MODEL_RELATIVE = ROOT_RELATIVE / "model/v3_02_c1_r1_candidate_predictor.yaml"
CALIBRATION_RELATIVE = (
    ROOT_RELATIVE / "model/v3_02_c1_r1_calibration_report.yaml"
)
MODEL_FREEZE_RELATIVE = ROOT_RELATIVE / "model/v3_02_c1_r1_model_freeze.sha256"
INDEPENDENT_MODEL_REVIEW_RELATIVE = (
    ROOT_RELATIVE / "model/v3_02_c1_r1_independent_model_review.yaml"
)
TRACE_METRICS_NEEDS_ROW = True
PHYSICAL_CLIP = True
CONFORMAL_SCORE = "actual_mode_absolute_residual"
REPAIR_SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/prediction/v3_02_c1/model/"
        "v3_02_c1_terminal_calibration_result.yaml"
    ),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_02_c1_r1.py"
    ),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_c1_r1.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_model.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_c1_r1_model.py"),
)

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


def model_features(metrics):
    coordinate = float(
        metrics["actor_conflict_coordinate_at_observation_m"]
    )
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
    require(contract["stage"] == "V3-02-C1-R1", "stage differs")
    require(
        contract["status"]
        == "pure_offline_single_factor_repair_preregistered",
        "status differs",
    )
    factor = contract["single_factor_repair"]
    require(
        factor["factor_count"] == 1
        and factor["factor_id"] == "ridge_feature_basis"
        and factor["parent_value"] == "partial_quadratic_7_features"
        and factor["repaired_value"] == "complete_quadratic_10_features"
        and factor["complete_features"] == FEATURE_NAMES
        and factor[
            "ridge_conformal_score_physical_clip_and_gates_unchanged"
        ]
        is True,
        "repair factor differs",
    )
    require(
        contract["authority"]["held_out_test_access_authorized"] is False
        and contract["authority"]["reserved_test_seeds"]
        == [6261, 6262, 6263],
        "test firewall differs",
    )
    require(
        preregistration["stage"] == "V3-02-C1-R1"
        and preregistration["status"]
        == "frozen_before_repaired_model_materialization"
        and preregistration["factor_count"] == 1
        and preregistration["held_out_test_access"] is False,
        "preregistration differs",
    )


def effective_contract(repair_contract: Mapping[str, Any]):
    parent = copy.deepcopy(
        load_yaml_relative(WORKSPACE, PARENT_CONTRACT_RELATIVE)
    )
    parent["stage"] = "V3-02-C1-R1"
    parent["predictor_design"]["model"] = (
        "per_mode_per_horizon_complete_quadratic_ridge"
    )
    parent["predictor_design"]["scalar_features"] = list(FEATURE_NAMES)
    return parent
