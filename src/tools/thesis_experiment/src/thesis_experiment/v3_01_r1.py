"""V3-01-R1 single-factor pause-flag repair bindings."""

import importlib.util
from pathlib import Path
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_qualification.py"
)
_SPEC = importlib.util.spec_from_file_location("v3_01_base", BASE_PATH)
BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(BASE)

V301Error = BASE.V301Error
require = BASE.require
sha256_bytes = BASE.sha256_bytes
yaml_bytes = BASE.yaml_bytes
canonical_sha = BASE.canonical_sha
read_file_once = BASE.read_file_once
load_yaml_path = BASE.load_yaml_path
load_yaml_relative = BASE.load_yaml_relative
receipt = BASE.receipt
absolute_receipt = BASE.absolute_receipt
render_world = BASE.render_world
load_trace = BASE.load_trace
trace_metrics = BASE.trace_metrics
check_receipts = BASE.check_receipts

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_01_r1_pause_flag_repair_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_01_r1_pause_flag_repair_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_01_r1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/data_qualification/v3_01_r1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_01_r1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_01_r1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_01_r1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_01_r1_execution_release.yaml"
REVIEW_RELATIVE = (
    READINESS_RELATIVE / "v3_01_r1_independent_release_review.yaml"
)
READINESS_FREEZE_RELATIVE = (
    READINESS_RELATIVE / "v3_01_r1_readiness_freeze.sha256"
)
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_01_r1_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = (
    EXECUTION_RELATIVE / "v3_01_r1_qualification_assessment.yaml"
)
EXECUTION_FREEZE_RELATIVE = (
    EXECUTION_RELATIVE / "v3_01_r1_execution_freeze.sha256"
)

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/data_qualification/v3_01/execution/"
        "v3_01_terminal_failure_attribution.yaml"
    ),
    Path("src/simulation/m2_gazebo/package.xml"),
    Path("src/simulation/m2_gazebo/CMakeLists.txt"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/trajectory_interpolation.h"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v3_01_qualification.py"
    ),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_01_r1.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_01_r1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_01_r1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_01_r1.py"),
)
BINARY_RESOURCES = BASE.BINARY_RESOURCES


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["schema_version"] == "1.0", "contract schema differs")
    require(contract["stage"] == "V3-01-R1", "stage differs")
    require(
        contract["status"]
        == "single_factor_repair_preregistered_before_simulation",
        "contract status differs",
    )
    factor = contract["single_factor_repair"]
    require(
        factor["factor_count"] == 1
        and factor["factor_id"] == "gzserver_start_paused_flag"
        and factor["parent_value"] == ["--verbose", "-u", "WORLD"]
        and factor["repaired_value"] == ["--verbose", "WORLD"]
        and factor["all_scene_actor_recorder_thresholds_unchanged"] is True,
        "single repair factor differs",
    )
    unit = contract["experimental_unit"]
    require(
        unit["policies"] == ["cautious", "assertive", "hesitant"]
        and unit["fresh_seeds"] == [6111, 6112, 6113],
        "policy or seed roster differs",
    )
    require(
        unit["schedule_rows"] == 36
        and unit["evidence_budget_units"] == 36
        and unit["attempts_per_identity"] == 1
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "repair budget differs",
    )
    require(
        [float(row["speed_mps"]) for row in unit["ego_interventions"]]
        == [0.6, 0.8, 1.0, 1.2],
        "candidate speeds differ",
    )
    require(
        preregistration["stage"] == "V3-01-R1"
        and preregistration["status"]
        == "frozen_before_any_v3_01_r1_gazebo_execution"
        and preregistration["design"]["fresh_seeds"] == [6111, 6112, 6113]
        and preregistration["design"]["exact_row_count"] == 36
        and preregistration["budget"]["consumed_before_release"] == 0,
        "repair preregistration differs",
    )


def exact_schedule(contract: Mapping[str, Any]) -> Dict[str, Any]:
    unit = contract["experimental_unit"]
    rows = []
    sequence = 0
    for seed in unit["fresh_seeds"]:
        for policy in unit["policies"]:
            for candidate in unit["ego_interventions"]:
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "unit_id": "v3_01_r1_s{}_{}_{}".format(
                            seed, policy, candidate["candidate_id"]
                        ),
                        "seed": seed,
                        "actor_policy_evaluator_only": policy,
                        "candidate_id_evaluator_only": candidate[
                            "candidate_id"
                        ],
                        "ego_speed_mps": float(candidate["speed_mps"]),
                        "attempt": 1,
                        "propensity": 0.25,
                    }
                )
    require(len(rows) == 36, "schedule count differs")
    return {
        "schema_version": "1.0",
        "stage": "V3-01-R1",
        "schedule_id": "v3_scope_v3_01_r1_crossing_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "order": "seed_then_policy_then_increasing_candidate_speed",
        "row_count": 36,
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
