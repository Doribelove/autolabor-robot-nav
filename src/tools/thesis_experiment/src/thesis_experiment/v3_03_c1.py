"""V3-03-C1 segmented closed-loop integration bindings."""

import importlib.util
from pathlib import Path
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_e1.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v3_02_e1_base_for_c1", PARENT_PATH
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
    "interpolate_plan",
    "check_receipts",
):
    globals()[_name] = getattr(PARENT, _name)

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_03_c1_segmented_closed_loop_integration_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_03_c1_segmented_closed_loop_integration_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_03_c1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/planner/v3_03_c1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_03_c1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_03_c1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_03_c1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_03_c1_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_03_c1_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = (
    READINESS_RELATIVE / "v3_03_c1_readiness_freeze.sha256"
)
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_03_c1_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = (
    EXECUTION_RELATIVE / "v3_03_c1_closed_loop_assessment.yaml"
)
EVIDENCE_FREEZE_RELATIVE = (
    EXECUTION_RELATIVE / "v3_03_c1_evidence_freeze.sha256"
)
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_03_c1_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_03_c1_terminal_freeze.sha256"

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/planner/v3_03_c0/"
        "v3_03_c0_effective_planner_contract.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_c0/"
        "v3_03_c0_offline_report.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_c0/"
        "v3_03_c0_independent_review.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_e1/"
        "v3_02_e1_terminal_freeze.sha256"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ),
    Path(
        "src/application/v3_scope_planner/src/v3_scope_planner/core.py"
    ),
    Path(
        "src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"
    ),
    Path(
        "src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"
    ),
    Path(
        "src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"
    ),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c1.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_03_c1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_c1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_03_c1_closed_loop.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_03_c1_closed_loop.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_c1_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/test_v3_03_c1.py"
    ),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-03-C1", "stage differs")
    require(
        contract["status"] == "preregistered_before_execution_release",
        "status differs",
    )
    require(
        contract["upstream"]["planner_contract_sha256"]
        == "92ac141f3eba8dbcf4d008d95bf9e6ab41332b33288c9cb297a40af34b393c8f"
        and contract["frozen_model"]["sha256"]
        == "fa4ef83b9b2e534da3031001f5dd0036cd7f983ebfbe73fe20f45c4b73bc55d7"
        and contract["frozen_model"]["refit_or_recalibration_allowed"]
        is False,
        "frozen planner/model differs",
    )
    unit = contract["experimental_unit"]
    require(
        [row["seed"] for row in unit["scenario_blocks"]]
        == [6331, 6332, 6333]
        and unit["schedule_rows"] == 9
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "fresh schedule differs",
    )
    loop = contract["closed_loop"]
    require(
        float(loop["observation_prefix_s"]) == 0.5
        and float(loop["replanning_period_s"]) == 0.25
        and float(loop["maximum_episode_duration_s"]) == 10.0
        and int(loop["maximum_decisions_per_episode"]) == 38
        and loop["exact_python_core_each_decision"] is True
        and loop["evaluator_policy_passed_to_planner"] is False,
        "closed-loop protocol differs",
    )
    authority = contract["authority_boundary"]
    require(
        authority["training_authorized"] is False
        and authority["model_or_gate_write_authorized"] is False
        and authority["move_base_authorized"] is False
        and authority["cmd_vel_publish_authorized"] is False
        and authority["hardware_device_access_authorized"] is False
        and authority["real_vehicle_authorized"] is False,
        "authority differs",
    )
    require(
        preregistration["stage"] == "V3-03-C1"
        and preregistration["status"]
        == "frozen_before_any_v3_03_c1_simulation"
        and preregistration["fresh_schedule"]["seeds"]
        == [6331, 6332, 6333]
        and preregistration["closed_loop"][
            "planner_receives_policy_label"
        ]
        is False,
        "preregistration differs",
    )


def exact_schedule(contract: Mapping[str, Any]) -> Dict[str, Any]:
    rows = []
    sequence = 0
    for block in contract["experimental_unit"]["scenario_blocks"]:
        for policy in contract["experimental_unit"][
            "policies_evaluator_only"
        ]:
            sequence += 1
            rows.append(
                {
                    "sequence": sequence,
                    "unit_id": "v3_03_c1_{}_{}".format(
                        block["block_id"], policy
                    ),
                    "block_id": block["block_id"],
                    "seed": block["seed"],
                    "angle_deg": float(block["angle_deg"]),
                    "actor_start_distance_m": float(
                        block["actor_start_distance_m"]
                    ),
                    "actor_base_speed_mps": float(
                        block["actor_base_speed_mps"]
                    ),
                    "actor_policy_evaluator_only": policy,
                    "attempt": 1,
                }
            )
    require(len(rows) == 9, "schedule count differs")
    return {
        "schema_version": "1.0",
        "stage": "V3-03-C1",
        "schedule_id": "v3_scope_v3_03_c1_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 9,
        "episode_count": 9,
        "seeds": [6331, 6332, 6333],
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
