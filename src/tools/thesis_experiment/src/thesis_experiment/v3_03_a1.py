"""V3-03-A1 probe-select-execute simulation integration bindings."""

import importlib.util
from pathlib import Path
import sys
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
D2_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_d2.py"
)
_D2_SPEC = importlib.util.spec_from_file_location("v3_02_d2_for_v303", D2_PATH)
D2 = importlib.util.module_from_spec(_D2_SPEC)
_D2_SPEC.loader.exec_module(D2)

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
    "trace_metrics",
):
    globals()[_name] = getattr(D2, _name)

PACKAGE_SOURCE = WORKSPACE / "src/application/v3_scope_planner/src"
if str(PACKAGE_SOURCE) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SOURCE))
from v3_scope_planner import core

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_03_a1_shadow_integration_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_03_a1_shadow_integration_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_03_a1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/planner/v3_03_a1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_03_a1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_03_a1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_03_a1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_03_a1_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_03_a1_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_03_a1_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_03_a1_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_03_a1_integration_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_03_a1_evidence_freeze.sha256"
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_03_a1_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_03_a1_terminal_freeze.sha256"

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path("config/thesis_experiments/v3/v3_03_a0_safe_planner_contract.yaml"),
    Path(
        "artifacts/v3/planner/v3_03_a0/"
        "v3_03_a0_offline_summary.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_a0/"
        "v3_03_a0_evidence_freeze.sha256"
    ),
    Path(
        "artifacts/v3/planner/v3_03_a0/"
        "v3_03_a0_independent_review.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/core.py"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"
    ),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_d2.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_03_a1.py"),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_03_a1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_a1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_03_a1_integration.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_03_a1_integration.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_a1_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_03_a1.py"
    ),
)
BINARY_RESOURCES = D2.BINARY_RESOURCES


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-03-A1", "stage differs")
    require(
        contract["status"] == "preregistered_before_simulation_release",
        "status differs",
    )
    upstream = contract["upstream"]
    require(
        upstream["planner_contract_sha256"]
        == "be8b1e893961a2c8b4f9d77e884f5763906572b44ee6196b28f6f24877f3ebbd"
        and upstream["planner_core_sha256"]
        == "8b5bc8f3a3aba38fb631841d35c7d657f3257af8a802460bfd72272e4b1b7a47"
        and upstream["predictor_sha256"]
        == "fa4ef83b9b2e534da3031001f5dd0036cd7f983ebfbe73fe20f45c4b73bc55d7",
        "upstream freeze differs",
    )
    unit = contract["experimental_unit"]
    require(
        [row["seed"] for row in unit["scene_blocks"]]
        == [6291, 6292, 6293]
        and unit["decisions"] == 9
        and unit["simulation_units"] == 18
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "schedule differs",
    )
    require(
        contract["probe_select_execute"][
            "hidden_actor_policy_available_to_planner"
        ]
        is False
        and contract["authority"]["cmd_vel_publish_authorized"] is False
        and contract["authority"]["move_base_authorized"] is False
        and contract["authority"]["training_authorized"] is False
        and contract["authority"]["real_vehicle_authorized"] is False,
        "authority differs",
    )
    require(
        preregistration["stage"] == "V3-03-A1"
        and preregistration["status"]
        == "frozen_before_any_v3_03_a1_simulation"
        and preregistration["schedule"]["simulation_units"] == 18
        and preregistration["planner_firewall"][
            "actor_policy_runtime_input"
        ]
        is False,
        "preregistration differs",
    )


def exact_schedule(contract: Mapping[str, Any]) -> Dict[str, Any]:
    rows = []
    sequence = 0
    for block in contract["experimental_unit"]["scene_blocks"]:
        for policy in contract["experimental_unit"]["policies_evaluator_only"]:
            decision_id = "v3_03_a1_{}_{}".format(block["block_id"], policy)
            for phase in ("common_prefix_probe", "selected_candidate_execution"):
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "unit_id": "{}_{}".format(decision_id, phase),
                        "decision_id": decision_id,
                        "phase": phase,
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
    return {
        "schema_version": "1.0",
        "stage": "V3-03-A1",
        "schedule_id": "v3_scope_v3_03_a1_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "decision_count": 9,
        "row_count": 18,
        "simulation_unit_count": 18,
        "seeds": [6291, 6292, 6293],
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
