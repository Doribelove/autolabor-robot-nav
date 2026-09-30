"""V3-03-A2 short-probe parser repair with fresh integration identities."""

import copy
import csv
import importlib.util
import math
from pathlib import Path
from typing import Any, Dict, List, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_a1.py"
)
_SPEC = importlib.util.spec_from_file_location("v3_03_a1_for_a2", PARENT_PATH)
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
    "receipt",
    "absolute_receipt",
    "interpolate",
    "crossing_time",
    "check_receipts",
    "actor_direction",
    "render_world",
    "trace_metrics",
    "core",
):
    globals()[_name] = getattr(PARENT, _name)

BASE_LOAD_YAML_RELATIVE = PARENT.load_yaml_relative
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_03_a2_short_probe_parser_repair_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_03_a2_short_probe_parser_repair_preregistration.yaml"
)
PARENT_CONTRACT_RELATIVE = PARENT.CONTRACT_RELATIVE
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_03_a2_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/planner/v3_03_a2")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_03_a2_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_03_a2_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_03_a2_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_03_a2_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_03_a2_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_03_a2_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_03_a2_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_03_a2_integration_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_03_a2_evidence_freeze.sha256"
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_03_a2_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_03_a2_terminal_freeze.sha256"

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    PARENT_CONTRACT_RELATIVE,
    Path(
        "artifacts/v3/planner/v3_03_a1/"
        "v3_03_a1_terminal_failure_record.yaml"
    ),
    Path("config/thesis_experiments/v3/v3_03_a0_safe_planner_contract.yaml"),
    Path("artifacts/v3/planner/v3_03_a0/v3_03_a0_offline_summary.yaml"),
    Path("artifacts/v3/planner/v3_03_a0/v3_03_a0_evidence_freeze.sha256"),
    Path("artifacts/v3/planner/v3_03_a0/v3_03_a0_independent_review.yaml"),
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
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_03_a2.py"),
    Path(
        "src/tools/thesis_experiment/scripts/build_v3_03_a1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/review_v3_03_a1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/run_v3_03_a1_integration.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/assess_v3_03_a1_integration.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/review_v3_03_a1_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/build_v3_03_a2_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/review_v3_03_a2_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/run_v3_03_a2_integration.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/assess_v3_03_a2_integration.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/review_v3_03_a2_result.py"
    ),
    Path("src/tools/thesis_experiment/tests/test_v3_03_a2.py"),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES


def effective_contract():
    parent = copy.deepcopy(
        BASE_LOAD_YAML_RELATIVE(WORKSPACE, PARENT_CONTRACT_RELATIVE)
    )
    repair = BASE_LOAD_YAML_RELATIVE(WORKSPACE, CONTRACT_RELATIVE)
    parent["stage"] = "V3-03-A2"
    parent["stage_purpose"] = repair["stage_purpose"]
    parent["contract_id"] = repair["contract_id"]
    parent["status"] = repair["status"]
    parent["parent_failure"] = repair["parent"]
    parent["single_factor_repair"] = repair["single_factor_repair"]
    parent["experimental_unit"]["scene_blocks"] = repair[
        "fresh_execution"
    ]["scene_blocks"]
    parent["experimental_unit"]["decisions"] = repair["fresh_execution"][
        "decisions"
    ]
    parent["experimental_unit"]["simulation_units"] = repair[
        "fresh_execution"
    ]["simulation_units"]
    return parent


def load_yaml_relative(workspace, relative):
    if Path(relative) == CONTRACT_RELATIVE:
        return effective_contract()
    return BASE_LOAD_YAML_RELATIVE(workspace, relative)


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-03-A2", "stage differs")
    require(
        contract["status"]
        == "single_factor_repair_preregistered_before_fresh_release",
        "status differs",
    )
    factor = contract["single_factor_repair"]
    require(
        factor["factor_count"] == 1
        and factor["factor_id"] == "probe_trace_minimum_duration_parser"
        and factor["planner_core_changed"] is False
        and factor["predictor_changed"] is False
        and factor["safety_thresholds_changed"] is False
        and factor["integration_gates_changed"] is False,
        "single-factor repair differs",
    )
    require(
        [row["seed"] for row in contract["experimental_unit"]["scene_blocks"]]
        == [6301, 6302, 6303]
        and contract["experimental_unit"]["simulation_units"] == 18,
        "fresh schedule differs",
    )
    require(
        preregistration["stage"] == "V3-03-A2"
        and preregistration["status"]
        == "frozen_before_any_v3_03_a2_simulation"
        and preregistration["fresh_schedule"]["seeds"]
        == [6301, 6302, 6303]
        and preregistration["firewall"]["A1_resume_or_seed_reuse"] is False,
        "preregistration differs",
    )


def exact_schedule(contract: Mapping[str, Any]) -> Dict[str, Any]:
    rows = []
    sequence = 0
    for block in contract["experimental_unit"]["scene_blocks"]:
        for policy in contract["experimental_unit"]["policies_evaluator_only"]:
            decision_id = "v3_03_a2_{}_{}".format(block["block_id"], policy)
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
        "stage": "V3-03-A2",
        "schedule_id": "v3_scope_v3_03_a2_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "decision_count": 9,
        "row_count": 18,
        "simulation_unit_count": 18,
        "seeds": [6301, 6302, 6303],
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }


def load_trace(path: Path) -> List[Dict[str, float]]:
    expected = [
        "sim_time_s",
        "ego_x_m",
        "ego_y_m",
        "ego_yaw_rad",
        "actor_x_m",
        "actor_y_m",
        "actor_yaw_rad",
    ]
    rows = []
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames == expected, "CSV header differs")
        for index, raw in enumerate(reader):
            converted = {}
            for key in expected:
                try:
                    value = float(raw[key])
                except (TypeError, ValueError) as exc:
                    raise V301Error(
                        "{}[{}].{} is not numeric".format(path, index, key)
                    ) from exc
                require(
                    math.isfinite(value),
                    "{}[{}].{} is not finite".format(path, index, key),
                )
                converted[key] = value
            rows.append(converted)
    require(len(rows) >= 11, "trace has fewer than 11 samples")
    times = [row["sim_time_s"] for row in rows]
    require(
        all(after > before for before, after in zip(times, times[1:])),
        "trace time is not strictly increasing",
    )
    require(times[-1] >= 0.50, "trace does not cover decision time")
    return rows
