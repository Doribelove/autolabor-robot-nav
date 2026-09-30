"""V3-03-C6 post-conflict reachable integration bindings."""

import copy
import importlib.util
from pathlib import Path
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c4.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v3_03_c4_base_for_c6", PARENT_PATH
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
    "load_trace",
):
    globals()[_name] = getattr(PARENT, _name)

BASE_LOAD_YAML_RELATIVE = PARENT.load_yaml_relative
PARENT_CONTRACT_RELATIVE = PARENT.CONTRACT_RELATIVE
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_03_c6_post_conflict_reachable_integration_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_03_c6_post_conflict_reachable_integration_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_03_c6_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/planner/v3_03_c6")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_03_c6_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_03_c6_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_03_c6_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_03_c6_execution_release.yaml"
REVIEW_RELATIVE = (
    READINESS_RELATIVE / "v3_03_c6_independent_release_review.yaml"
)
READINESS_FREEZE_RELATIVE = (
    READINESS_RELATIVE / "v3_03_c6_readiness_freeze.sha256"
)
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_03_c6_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = (
    EXECUTION_RELATIVE / "v3_03_c6_closed_loop_assessment.yaml"
)
EVIDENCE_FREEZE_RELATIVE = (
    EXECUTION_RELATIVE / "v3_03_c6_evidence_freeze.sha256"
)
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_03_c6_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_03_c6_terminal_freeze.sha256"

SOURCE_RESOURCES = PARENT.SOURCE_RESOURCES + (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    PARENT_CONTRACT_RELATIVE,
    Path("artifacts/v3/planner/v3_03_c4/v3_03_c4_terminal_freeze.sha256"),
    Path(
        "artifacts/v3/planner/v3_03_c4/execution/"
        "v3_03_c4_independent_failure_review.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_c5/"
        "v3_03_c5_effective_planner_contract.yaml"
    ),
    Path("artifacts/v3/planner/v3_03_c5/v3_03_c5_offline_report.yaml"),
    Path(
        "artifacts/v3/planner/v3_03_c5/"
        "v3_03_c5_independent_review.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_c5/"
        "v3_03_c5_evidence_freeze.sha256"
    ),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_03_c6.py"),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_03_c6_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_c6_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_03_c6_closed_loop.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_03_c6_closed_loop.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_03_c6_result.py"
    ),
    Path("src/tools/thesis_experiment/tests/test_v3_03_c6.py"),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES


def effective_contract():
    parent = copy.deepcopy(
        BASE_LOAD_YAML_RELATIVE(WORKSPACE, PARENT_CONTRACT_RELATIVE)
    )
    repair = BASE_LOAD_YAML_RELATIVE(WORKSPACE, CONTRACT_RELATIVE)
    parent["stage"] = "V3-03-C6"
    parent["stage_purpose"] = repair["stage_purpose"]
    parent["contract_id"] = repair["contract_id"]
    parent["status"] = repair["status"]
    parent["parent_failure"] = repair["parent"]
    parent["offline_repair"] = repair["offline_repair"]
    parent["single_factor_repair"] = repair["single_factor_repair"]
    parent["upstream"]["planner_contract_path"] = repair[
        "offline_repair"
    ]["planner_contract_path"]
    parent["upstream"]["planner_contract_sha256"] = repair[
        "offline_repair"
    ]["planner_contract_sha256"]
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
    require(contract["stage"] == "V3-03-C6", "stage differs")
    factor = contract["single_factor_repair"]
    repair = contract["offline_repair"]
    require(
        contract["status"]
        == "single_factor_repair_preregistered_before_fresh_release"
        and factor["factor_count"] == 1
        and factor["factor_id"]
        == "certified_post_conflict_reachable_occupancy"
        and factor["learned_predictor_used_in_bypass"] is False
        and factor["time_indexed_geometric_verification_retained"] is True
        and factor["commitment_invariance_changed"] is False
        and factor["predictor_model_or_mode_gate_changed"] is False
        and factor["cleared_certificate_changed"] is False
        and factor["candidate_generator_changed"] is False
        and factor["ranker_weights_changed"] is False
        and factor["footprint_or_margin_changed"] is False
        and factor["closed_loop_protocol_changed"] is False
        and factor["integration_gates_changed"] is False,
        "single-factor repair differs",
    )
    require(
        repair["stage"] == "V3-03-C5"
        and repair["planner_contract_sha256"]
        == "6f2ec1530b67785dfc8f5a397ab416847f35f0c6ffc9e6f45bbf633a50057761"
        and contract["upstream"]["planner_contract_path"]
        == repair["planner_contract_path"]
        and contract["upstream"]["planner_contract_sha256"]
        == repair["planner_contract_sha256"],
        "C5 planner binding differs",
    )
    require(
        [row["seed"] for row in contract["experimental_unit"]["scenario_blocks"]]
        == [6361, 6362, 6363]
        and contract["experimental_unit"]["schedule_rows"] == 9
        and contract["experimental_unit"]["retry_allowed"] is False
        and contract["experimental_unit"]["resume_allowed"] is False,
        "fresh schedule differs",
    )
    loop = contract["closed_loop"]
    require(
        float(loop["observation_prefix_s"]) == 0.5
        and float(loop["replanning_period_s"]) == 0.25
        and float(loop["maximum_episode_duration_s"]) == 10.0
        and int(loop["maximum_decisions_per_episode"]) == 38,
        "closed-loop protocol differs",
    )
    require(
        preregistration["stage"] == "V3-03-C6"
        and preregistration["status"]
        == "frozen_before_any_v3_03_c6_simulation"
        and preregistration["fresh_schedule"]["seeds"]
        == [6361, 6362, 6363]
        and preregistration["single_factor"]["c4_resume_or_seed_reuse"]
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
                    "unit_id": "v3_03_c6_{}_{}".format(
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
        "stage": "V3-03-C6",
        "schedule_id": "v3_scope_v3_03_c6_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 9,
        "episode_count": 9,
        "seeds": [6361, 6362, 6363],
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
