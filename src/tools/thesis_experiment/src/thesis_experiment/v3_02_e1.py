"""V3-02-E1 fresh held-out observable-mode predictor bindings."""

import importlib.util
from pathlib import Path
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_d2.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v3_02_d2_base_for_e1", PARENT_PATH
)
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
    "model_features",
    "ego_waypoints",
    "interpolate_plan",
    "planned_kinematics",
    "heldout_additional_assessment",
):
    globals()[_name] = getattr(PARENT, _name)

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_02_e1_observed_mode_heldout_test_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_02_e1_observed_mode_heldout_test_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_02_e1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/prediction/v3_02_e1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_02_e1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_02_e1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_02_e1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_02_e1_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_02_e1_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = (
    READINESS_RELATIVE / "v3_02_e1_readiness_freeze.sha256"
)
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_02_e1_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_02_e1_heldout_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = (
    EXECUTION_RELATIVE / "v3_02_e1_evidence_freeze.sha256"
)
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_02_e1_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_02_e1_terminal_freeze.sha256"
TRACE_METRICS_NEEDS_ROW = True

SOURCE_RESOURCES = PARENT.SOURCE_RESOURCES + (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/planner/v3_03_b0/"
        "v3_03_b0_effective_planner_contract.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_b0/"
        "v3_03_b0_mode_gate_report.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_b0/"
        "v3_03_b0_independent_review.yaml"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/core.py"
    ),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_02_e1.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/build_v3_02_e1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/review_v3_02_e1_readiness.py"
    ),
    Path("src/tools/thesis_experiment/scripts/run_v3_02_e1_test.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_02_e1.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_e1_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_02_e1.py"),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-02-E1", "stage differs")
    require(
        contract["status"] == "preregistered_before_test_release",
        "status differs",
    )
    model = contract["frozen_model"]
    require(
        model["sha256"]
        == "fa4ef83b9b2e534da3031001f5dd0036cd7f983ebfbe73fe20f45c4b73bc55d7"
        and model["refit_or_recalibration_allowed"] is False,
        "frozen model differs",
    )
    mode = contract["observed_mode_gate"]
    require(
        mode["effective_planner_contract_sha256"]
        == "c1fd1f18c7065039709aa6d35aa5da7b9fd1550c316233d45792e66f8f5655e4"
        and mode["hidden_policy_runtime_input"] is False
        and float(mode["cautious_hesitant_threshold_mps"])
        == 0.274012499989
        and float(mode["hesitant_assertive_threshold_mps"])
        == 0.758912499986
        and float(mode["ambiguity_half_width_mps"]) == 0.02,
        "mode gate differs",
    )
    unit = contract["experimental_unit"]
    require(
        [row["seed"] for row in unit["scenario_blocks"]]
        == [6321, 6322, 6323]
        and len(unit["ego_interventions"]) == 5
        and unit["schedule_rows"] == 45
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "held-out roster differs",
    )
    require(
        unit["ego_interventions"][0]["certified_stop"] is True
        and [float(row["speed_mps"]) for row in unit["ego_interventions"]]
        == [0.0, 0.6, 0.8, 1.0, 1.2],
        "candidate support differs",
    )
    authority = contract["authority_boundary"]
    require(
        authority["model_write_allowed"] is False
        and authority["gate_write_allowed_after_test"] is False
        and authority["prior_seed_or_outcome_reuse_allowed"] is False
        and authority["move_base_authorized"] is False
        and authority["cmd_vel_publish_authorized"] is False
        and authority["real_vehicle_authorized"] is False,
        "test firewall differs",
    )
    require(
        preregistration["stage"] == "V3-02-E1"
        and preregistration["status"]
        == "frozen_before_any_v3_02_e1_heldout_outcome"
        and preregistration["test"]["seeds"] == [6321, 6322, 6323]
        and preregistration["mode_gate_firewall"][
            "gate_update_allowed"
        ]
        is False
        and preregistration["mode_gate_firewall"][
            "test_label_available_to_runtime"
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
                        "unit_id": "v3_02_e1_{}_{}_{}".format(
                            block["block_id"],
                            policy,
                            candidate["candidate_id"],
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
                        "candidate_id_evaluator_only": candidate[
                            "candidate_id"
                        ],
                        "certified_stop": bool(candidate["certified_stop"]),
                        "ego_speed_mps": float(candidate["speed_mps"]),
                        "attempt": 1,
                        "propensity": 0.2,
                    }
                )
    require(len(rows) == 45, "schedule count differs")
    return {
        "schema_version": "1.0",
        "stage": "V3-02-E1",
        "schedule_id": "v3_scope_v3_02_e1_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 45,
        "held_out_test_rows": 45,
        "seeds": [6321, 6322, 6323],
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
