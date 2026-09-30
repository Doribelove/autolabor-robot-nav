"""V3-02-T2 corrected-model fresh held-out test bindings."""

import importlib.util
from pathlib import Path
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_t1.py"
)
_SPEC = importlib.util.spec_from_file_location("v3_02_t1_base", PARENT_PATH)
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
    "actor_direction",
    "render_world",
    "feature_vector",
    "trace_metrics",
):
    globals()[_name] = getattr(PARENT, _name)

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_02_t2_fresh_heldout_test_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_02_t2_fresh_heldout_test_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_02_t2_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/prediction/v3_02_t2")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_02_t2_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_02_t2_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_02_t2_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_02_t2_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_02_t2_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_02_t2_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_02_t2_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_02_t2_heldout_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = (
    EXECUTION_RELATIVE / "v3_02_t2_evidence_freeze.sha256"
)
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_02_t2_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_02_t2_terminal_freeze.sha256"
TRACE_METRICS_NEEDS_ROW = True

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/prediction/v3_02_b1/model/"
        "v3_02_b1_candidate_predictor.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_b1/model/"
        "v3_02_b1_calibration_report.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_b1/model/"
        "v3_02_b1_model_freeze.sha256"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_b1/model/"
        "v3_02_b1_independent_model_review.yaml"
    ),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
    ),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_t1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_t2.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_02_t1.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t1_result.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_t2_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t2_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_02_t2_test.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_02_t2.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_t2_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_02_t2.py"),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-02-T2", "stage differs")
    require(
        contract["status"] == "preregistered_before_test_release",
        "status differs",
    )
    require(
        contract["frozen_model"]["sha256"]
        == "34458918fd937ef07fc7f87b0954b0f2fdc432d5839c0947ad456b7771ed7ae0"
        and contract["frozen_model"]["conformal_score"]
        == "actual_mode_absolute_residual"
        and contract["frozen_model"]["refit_or_recalibration_allowed"] is False,
        "model freeze differs",
    )
    unit = contract["experimental_unit"]
    require(
        [row["seed"] for row in unit["scenario_blocks"]]
        == [6231, 6232, 6233]
        and unit["schedule_rows"] == 36
        and unit["evidence_budget_units"] == 36
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "fresh test roster differs",
    )
    require(
        contract["authority_boundary"]["T1_seed_or_outcome_reuse_allowed"]
        is False
        and contract["authority_boundary"]["model_write_allowed"] is False,
        "firewall differs",
    )
    require(
        preregistration["stage"] == "V3-02-T2"
        and preregistration["status"]
        == "frozen_before_any_fresh_heldout_test_outcome"
        and preregistration["test"]["seeds"] == [6231, 6232, 6233]
        and preregistration["model_firewall"][
            "T1_outcome_or_seed_reuse_allowed"
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
                        "unit_id": "v3_02_t2_{}_{}_{}".format(
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
    return {
        "schema_version": "1.0",
        "stage": "V3-02-T2",
        "schedule_id": "v3_scope_v3_02_t2_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 36,
        "held_out_test_rows": 36,
        "seeds": [6231, 6232, 6233],
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
