"""V3-02-A1-R1 pure-offline physical-coordinate clipping repair."""

import copy
import importlib.util
from pathlib import Path
from typing import Any, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
)
_SPEC = importlib.util.spec_from_file_location("v3_02_parent", BASE_PATH)
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
    "exact_schedule",
    "actor_direction",
    "render_world",
    "feature_vector",
    "trace_metrics",
):
    globals()[_name] = getattr(BASE, _name)

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_02_a1_r1_physical_clip_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_02_a1_r1_physical_clip_preregistration.yaml"
)
PARENT_CONTRACT_RELATIVE = BASE.CONTRACT_RELATIVE
SCHEDULE_RELATIVE = BASE.SCHEDULE_RELATIVE
EXECUTION_RELATIVE = BASE.EXECUTION_RELATIVE
JOURNAL_RELATIVE = BASE.JOURNAL_RELATIVE
ROOT_RELATIVE = Path("artifacts/v3/prediction/v3_02_a1_r1")
MODEL_RELATIVE = ROOT_RELATIVE / "model/v3_02_a1_r1_candidate_predictor.yaml"
CALIBRATION_RELATIVE = (
    ROOT_RELATIVE / "model/v3_02_a1_r1_calibration_report.yaml"
)
MODEL_FREEZE_RELATIVE = (
    ROOT_RELATIVE / "model/v3_02_a1_r1_model_freeze.sha256"
)
INDEPENDENT_MODEL_REVIEW_RELATIVE = (
    ROOT_RELATIVE / "model/v3_02_a1_r1_independent_model_review.yaml"
)
TRACE_METRICS_NEEDS_ROW = True
PHYSICAL_CLIP = True
REPAIR_SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/prediction/v3_02_a1/model/"
        "v3_02_a1_terminal_calibration_result.yaml"
    ),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_02_a1_r1.py"
    ),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_a1_r1.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_model.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_a1_r1_model.py"),
)


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-02-A1-R1", "repair stage differs")
    require(
        contract["status"]
        == "pure_offline_single_factor_repair_preregistered",
        "repair status differs",
    )
    factor = contract["single_factor_repair"]
    require(
        factor["factor_count"] == 1
        and factor["factor_id"] == "physical_actor_path_coordinate_clip"
        and factor["lower_bound"] == "actor_coordinate_at_observation"
        and float(factor["upper_bound_m"]) == 3.5
        and factor["regression_features_coefficients_and_gates_unchanged"]
        is True,
        "repair factor differs",
    )
    require(
        contract["authority"]["new_simulation_authorized"] is False
        and contract["authority"]["held_out_test_access_authorized"] is False
        and contract["authority"]["reserved_test_seeds"] == [6221, 6222, 6223],
        "repair authority differs",
    )
    require(
        preregistration["stage"] == "V3-02-A1-R1"
        and preregistration["status"]
        == "frozen_before_repaired_model_materialization"
        and preregistration["factor_count"] == 1
        and preregistration["held_out_test_access"] is False,
        "repair preregistration differs",
    )


def effective_contract(repair_contract: Mapping[str, Any]):
    parent = copy.deepcopy(
        load_yaml_relative(WORKSPACE, PARENT_CONTRACT_RELATIVE)
    )
    parent["stage"] = "V3-02-A1-R1"
    parent["predictor_design"]["physical_coordinate_clip"] = {
        "lower_bound": "actor_coordinate_at_observation",
        "upper_bound_m": float(
            repair_contract["single_factor_repair"]["upper_bound_m"]
        ),
    }
    return parent
