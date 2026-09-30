"""V3-02-C2 common-prefix frozen-predictor held-out test bindings."""

import importlib.util
from pathlib import Path
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_t1.py"
)
_PARENT_SPEC = importlib.util.spec_from_file_location(
    "v3_02_t1_base_for_c2", PARENT_PATH
)
PARENT = importlib.util.module_from_spec(_PARENT_SPEC)
_PARENT_SPEC.loader.exec_module(PARENT)

MODEL_MODULE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_c1_r1.py"
)
_MODEL_SPEC = importlib.util.spec_from_file_location(
    "v3_02_c1_r1_model_for_c2", MODEL_MODULE_PATH
)
MODEL_MODULE = importlib.util.module_from_spec(_MODEL_SPEC)
_MODEL_SPEC.loader.exec_module(MODEL_MODULE)

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

model_features = MODEL_MODULE.model_features

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_02_c2_common_prefix_heldout_test_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_02_c2_common_prefix_heldout_test_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_02_c2_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/prediction/v3_02_c2")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_02_c2_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_02_c2_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_02_c2_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_02_c2_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_02_c2_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_02_c2_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_02_c2_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_02_c2_heldout_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_02_c2_evidence_freeze.sha256"
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_02_c2_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_02_c2_terminal_freeze.sha256"
TRACE_METRICS_NEEDS_ROW = True

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/prediction/v3_02_c1_r1/model/"
        "v3_02_c1_r1_candidate_predictor.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_c1_r1/model/"
        "v3_02_c1_r1_calibration_report.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_c1_r1/model/"
        "v3_02_c1_r1_model_freeze.sha256"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_c1_r1/model/"
        "v3_02_c1_r1_independent_model_review.yaml"
    ),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
    ),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_t1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_c1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_c1_r1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_c2.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_02_t1.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t1_result.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_c2_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_c2_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_02_c2_test.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_02_c2.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_c2_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_02_c2.py"),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-02-C2", "stage differs")
    require(
        contract["status"] == "preregistered_before_test_release",
        "status differs",
    )
    model = contract["frozen_model"]
    require(
        model["sha256"]
        == "03cdc5b4d49c90e07bd7d1f0dbbde260dbfedc7c5b4db7c640c27ffb940580df"
        and model["calibration_report_sha256"]
        == "e399f03fff133e76649375a6a0be2d6644777725e598827e998b8d2d4a9c5c2f"
        and model["feature_basis"] == "complete_quadratic_10_features"
        and model["refit_or_recalibration_allowed"] is False,
        "frozen model differs",
    )
    unit = contract["experimental_unit"]
    require(
        [row["seed"] for row in unit["scenario_blocks"]]
        == [6261, 6262, 6263]
        and unit["schedule_rows"] == 36
        and unit["evidence_budget_units"] == 36
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "fresh common-prefix test roster differs",
    )
    prefix = contract["scene"]["ego"]["common_prefix"]
    require(
        float(prefix["duration_s"]) == 0.5
        and float(prefix["speed_mps"]) == 0.8
        and float(prefix["end_x_m"]) == -2.6,
        "common prefix differs",
    )
    require(
        contract["scope"]["common_decision_state_required"] is True
        and contract["authority_boundary"]["model_write_allowed"] is False
        and contract["authority_boundary"]["prior_seed_or_outcome_reuse_allowed"]
        is False
        and contract["failure_policy"]["no_gate_change"] is True,
        "test firewall differs",
    )
    require(
        preregistration["stage"] == "V3-02-C2"
        and preregistration["status"]
        == "frozen_before_any_common_prefix_heldout_outcome"
        and preregistration["test"]["seeds"] == [6261, 6262, 6263]
        and preregistration["candidate_atomicity"]["required_at_test"] is True
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
                        "unit_id": "v3_02_c2_{}_{}_{}".format(
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
                        "ego_speed_mps": float(candidate["speed_mps"]),
                        "attempt": 1,
                        "propensity": 0.25,
                    }
                )
    seeds = [row["seed"] for row in unit["scenario_blocks"]]
    return {
        "schema_version": "1.0",
        "stage": "V3-02-C2",
        "schedule_id": "v3_scope_v3_02_c2_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 36,
        "held_out_test_rows": 36,
        "seeds": seeds,
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
