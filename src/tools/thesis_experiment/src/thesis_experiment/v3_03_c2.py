"""V3-03-C2 duration-aware segment parser repair bindings."""

import copy
import csv
import importlib.util
import math
from pathlib import Path
from typing import Any, Dict, List, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c1.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v3_03_c1_base_for_c2", PARENT_PATH
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
    "receipt",
    "absolute_receipt",
    "interpolate",
    "interpolate_plan",
    "check_receipts",
):
    globals()[_name] = getattr(PARENT, _name)

BASE_LOAD_YAML_RELATIVE = PARENT.load_yaml_relative
PARENT_CONTRACT_RELATIVE = PARENT.CONTRACT_RELATIVE
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_03_c2_segment_parser_repair_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_03_c2_segment_parser_repair_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_03_c2_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/planner/v3_03_c2")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_03_c2_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_03_c2_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_03_c2_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_03_c2_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_03_c2_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = (
    READINESS_RELATIVE / "v3_03_c2_readiness_freeze.sha256"
)
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_03_c2_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = (
    EXECUTION_RELATIVE / "v3_03_c2_closed_loop_assessment.yaml"
)
EVIDENCE_FREEZE_RELATIVE = (
    EXECUTION_RELATIVE / "v3_03_c2_evidence_freeze.sha256"
)
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_03_c2_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_03_c2_terminal_freeze.sha256"

SOURCE_RESOURCES = PARENT.SOURCE_RESOURCES + (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    PARENT_CONTRACT_RELATIVE,
    Path(
        "artifacts/v3/planner/v3_03_c1/"
        "v3_03_c1_terminal_freeze.sha256"
    ),
    Path(
        "artifacts/v3/planner/v3_03_c1/"
        "v3_03_c1_terminal_failure_record.yaml"
    ),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c2.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_03_c2_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_c2_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_03_c2_closed_loop.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_03_c2_closed_loop.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_c2_result.py"
    ),
    Path("src/tools/thesis_experiment/tests/test_v3_03_c2.py"),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES


def effective_contract():
    parent = copy.deepcopy(
        BASE_LOAD_YAML_RELATIVE(WORKSPACE, PARENT_CONTRACT_RELATIVE)
    )
    repair = BASE_LOAD_YAML_RELATIVE(WORKSPACE, CONTRACT_RELATIVE)
    parent["stage"] = "V3-03-C2"
    parent["stage_purpose"] = repair["stage_purpose"]
    parent["contract_id"] = repair["contract_id"]
    parent["status"] = repair["status"]
    parent["parent_failure"] = repair["parent"]
    parent["single_factor_repair"] = repair["single_factor_repair"]
    parent["experimental_unit"]["scenario_blocks"] = repair[
        "fresh_execution"
    ]["scenario_blocks"]
    parent["experimental_unit"]["schedule_rows"] = 9
    parent["experimental_unit"]["evidence_budget_units"] = 9
    parent["claim_if_pass"] = repair["claim_if_pass"]
    parent["forbidden_claims"] = repair["forbidden_claims"]
    return parent


def load_yaml_relative(workspace, relative):
    if Path(relative) == CONTRACT_RELATIVE:
        return effective_contract()
    return BASE_LOAD_YAML_RELATIVE(workspace, relative)


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-03-C2", "stage differs")
    factor = contract["single_factor_repair"]
    require(
        contract["status"]
        == "single_factor_repair_preregistered_before_fresh_release"
        and factor["factor_count"] == 1
        and factor["factor_id"] == "segment_duration_aware_trace_parser"
        and factor["planner_core_changed"] is False
        and factor["predictor_or_mode_gate_changed"] is False
        and factor["cleared_certificate_changed"] is False
        and factor["candidate_or_ranker_changed"] is False
        and factor["safety_thresholds_changed"] is False
        and factor["closed_loop_protocol_changed"] is False
        and factor["integration_gates_changed"] is False,
        "single-factor repair differs",
    )
    require(
        [row["seed"] for row in contract["experimental_unit"]["scenario_blocks"]]
        == [6341, 6342, 6343]
        and contract["experimental_unit"]["schedule_rows"] == 9,
        "fresh schedule differs",
    )
    require(
        preregistration["stage"] == "V3-03-C2"
        and preregistration["status"]
        == "frozen_before_any_v3_03_c2_simulation"
        and preregistration["fresh_schedule"]["seeds"]
        == [6341, 6342, 6343]
        and preregistration["single_factor"]["c1_resume_or_seed_reuse"]
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
                    "unit_id": "v3_03_c2_{}_{}".format(
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
    return {
        "schema_version": "1.0",
        "stage": "V3-03-C2",
        "schedule_id": "v3_scope_v3_03_c2_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 9,
        "episode_count": 9,
        "seeds": [6341, 6342, 6343],
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
    require(len(rows) >= 6, "trace has fewer than six data rows")
    times = [row["sim_time_s"] for row in rows]
    require(
        all(after > before for before, after in zip(times, times[1:])),
        "trace time is not strictly increasing",
    )
    require(times[-1] >= 0.25 - 1e-12, "trace does not cover one segment")
    return rows
