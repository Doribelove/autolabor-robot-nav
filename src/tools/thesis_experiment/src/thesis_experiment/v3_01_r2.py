"""V3-01-R2 uncalibrated threshold-jitter removal stage."""

import importlib.util
from pathlib import Path
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
R1_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_r1.py"
)
_SPEC = importlib.util.spec_from_file_location("v3_01_r1_base", R1_PATH)
R1 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(R1)

V301Error = R1.V301Error
require = R1.require
sha256_bytes = R1.sha256_bytes
yaml_bytes = R1.yaml_bytes
canonical_sha = R1.canonical_sha
read_file_once = R1.read_file_once
load_yaml_path = R1.load_yaml_path
load_yaml_relative = R1.load_yaml_relative
receipt = R1.receipt
absolute_receipt = R1.absolute_receipt
render_world = R1.render_world
load_trace = R1.load_trace
trace_metrics = R1.trace_metrics
check_receipts = R1.check_receipts

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_01_r2_jitter_repair_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_01_r2_jitter_repair_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_01_r2_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/data_qualification/v3_01_r2")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_01_r2_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_01_r2_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_01_r2_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_01_r2_execution_release.yaml"
REVIEW_RELATIVE = (
    READINESS_RELATIVE / "v3_01_r2_independent_release_review.yaml"
)
READINESS_FREEZE_RELATIVE = (
    READINESS_RELATIVE / "v3_01_r2_readiness_freeze.sha256"
)
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_01_r2_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = (
    EXECUTION_RELATIVE / "v3_01_r2_qualification_assessment.yaml"
)
EXECUTION_FREEZE_RELATIVE = (
    EXECUTION_RELATIVE / "v3_01_r2_execution_freeze.sha256"
)

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/data_qualification/v3_01_r1/execution/"
        "v3_01_r1_terminal_result.yaml"
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
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_01_r2.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_01_r1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_01_r1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_01_r2_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_01_r2_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r2_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_01_r2_qualification.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_01_r2.py"),
)
BINARY_RESOURCES = R1.BINARY_RESOURCES


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-01-R2", "stage differs")
    require(
        contract["status"]
        == "single_factor_repair_preregistered_before_simulation",
        "status differs",
    )
    factor = contract["single_factor_repair"]
    require(
        factor["factor_count"] == 1
        and factor["factor_id"]
        == "actor_response_margin_seed_jitter_fraction"
        and float(factor["parent_value"]) == 0.10
        and float(factor["repaired_value"]) == 0.0
        and factor[
            "all_scene_actor_policy_dynamics_recorder_and_gates_unchanged"
        ]
        is True,
        "single factor differs",
    )
    unit = contract["experimental_unit"]
    require(
        unit["policies"] == ["cautious", "assertive", "hesitant"]
        and unit["fresh_seeds"] == [6121, 6122, 6123]
        and unit["schedule_rows"] == 36
        and unit["evidence_budget_units"] == 36
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "unit roster or budget differs",
    )
    require(
        [float(row["speed_mps"]) for row in unit["ego_interventions"]]
        == [0.6, 0.8, 1.0, 1.2],
        "candidate speeds differ",
    )
    require(
        preregistration["stage"] == "V3-01-R2"
        and preregistration["status"]
        == "frozen_before_any_v3_01_r2_gazebo_execution"
        and preregistration["design"]["fresh_seeds"] == [6121, 6122, 6123]
        and preregistration["budget"]["consumed_before_release"] == 0,
        "preregistration differs",
    )


def exact_schedule(contract: Mapping[str, Any]) -> Dict[str, Any]:
    rows = []
    sequence = 0
    unit = contract["experimental_unit"]
    for seed in unit["fresh_seeds"]:
        for policy in unit["policies"]:
            for candidate in unit["ego_interventions"]:
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "unit_id": "v3_01_r2_s{}_{}_{}".format(
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
    return {
        "schema_version": "1.0",
        "stage": "V3-01-R2",
        "schedule_id": "v3_scope_v3_01_r2_crossing_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "order": "seed_then_policy_then_increasing_candidate_speed",
        "row_count": 36,
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
