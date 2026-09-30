"""V3-02-B1 actual-mode conformal-score repair."""

import copy
import importlib.util
from pathlib import Path
from typing import Any, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_a1_r1.py"
)
_SPEC = importlib.util.spec_from_file_location("v3_02_a1_r1_base", PARENT_PATH)
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
    "config/thesis_experiments/v3/v3_02_b1_conformal_score_repair_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_02_b1_conformal_score_repair_preregistration.yaml"
)
PARENT_CONTRACT_RELATIVE = PARENT.PARENT_CONTRACT_RELATIVE
SCHEDULE_RELATIVE = PARENT.SCHEDULE_RELATIVE
EXECUTION_RELATIVE = PARENT.EXECUTION_RELATIVE
JOURNAL_RELATIVE = PARENT.JOURNAL_RELATIVE
ROOT_RELATIVE = Path("artifacts/v3/prediction/v3_02_b1")
MODEL_RELATIVE = ROOT_RELATIVE / "model/v3_02_b1_candidate_predictor.yaml"
CALIBRATION_RELATIVE = ROOT_RELATIVE / "model/v3_02_b1_calibration_report.yaml"
MODEL_FREEZE_RELATIVE = ROOT_RELATIVE / "model/v3_02_b1_model_freeze.sha256"
INDEPENDENT_MODEL_REVIEW_RELATIVE = (
    ROOT_RELATIVE / "model/v3_02_b1_independent_model_review.yaml"
)
TRACE_METRICS_NEEDS_ROW = True
PHYSICAL_CLIP = True
CONFORMAL_SCORE = "actual_mode_absolute_residual"
REPAIR_SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/prediction/v3_02_t1/execution/"
        "v3_02_t1_terminal_result.yaml"
    ),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_02_b1.py"
    ),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_b1.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_model.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_b1_model.py"),
)


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-02-B1", "stage differs")
    require(
        contract["status"]
        == "pure_offline_single_factor_repair_preregistered",
        "status differs",
    )
    factor = contract["single_factor_repair"]
    require(
        factor["factor_count"] == 1
        and factor["factor_id"] == "conformal_nonconformity_score"
        and factor["parent_value"]
        == "outer_latent_mode_envelope_nonconformity"
        and factor["repaired_value"] == "actual_mode_absolute_residual"
        and factor[
            "regression_coefficients_features_physical_clip_and_gates_unchanged"
        ]
        is True,
        "repair factor differs",
    )
    require(
        contract["source_data"]["T1_test_outcomes_used_for_fit_or_radius"]
        is False
        and contract["authority"]["fresh_future_test_seeds_reserved"]
        == [6231, 6232, 6233]
        and contract["authority"]["new_simulation_authorized"] is False,
        "data firewall differs",
    )
    require(
        preregistration["stage"] == "V3-02-B1"
        and preregistration["status"]
        == "frozen_before_repaired_model_materialization"
        and preregistration["heldout_T1_outcomes_forbidden_in_fit"] is True
        and preregistration["fresh_future_test_seeds"] == [6231, 6232, 6233],
        "preregistration differs",
    )


def effective_contract(repair_contract: Mapping[str, Any]):
    parent = copy.deepcopy(
        load_yaml_relative(WORKSPACE, PARENT_CONTRACT_RELATIVE)
    )
    parent["stage"] = "V3-02-B1"
    parent["predictor_design"]["physical_coordinate_clip"] = {
        "lower_bound": "actor_coordinate_at_observation",
        "upper_bound_m": 3.5,
    }
    return parent
