"""V3-06-C1 low-speed candidate-density calibration bindings."""

import copy
import importlib.util
from pathlib import Path
import sys
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = WORKSPACE / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core


PARENT_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_b0.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v3_06_b0_base_for_v3_06_c1", PARENT_PATH
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
    "v3_06_c1_low_speed_candidate_calibration_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_06_c1_low_speed_candidate_calibration_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_06_c1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/development/v3_06_c1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_06_c1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_06_c1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_06_c1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_06_c1_execution_release.yaml"
REVIEW_RELATIVE = (
    READINESS_RELATIVE / "v3_06_c1_independent_release_review.yaml"
)
READINESS_FREEZE_RELATIVE = (
    READINESS_RELATIVE / "v3_06_c1_readiness_freeze.sha256"
)
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_06_c1_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = (
    EXECUTION_RELATIVE / "v3_06_c1_calibration_assessment.yaml"
)
EVIDENCE_FREEZE_RELATIVE = (
    EXECUTION_RELATIVE / "v3_06_c1_evidence_freeze.sha256"
)
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_06_c1_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_06_c1_terminal_freeze.sha256"

SOURCE_RESOURCES = PARENT.SOURCE_RESOURCES + (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path(
        "artifacts/v3/development/v3_06_c0/"
        "v3_06_c0_terminal_freeze.sha256"
    ),
    Path(
        "artifacts/v3/development/v3_06_c0/"
        "v3_06_c0_effective_planner_contract.yaml"
    ),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_06_c1.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_06_c1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_06_c1_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_06_c1_calibration.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_06_c1_calibration.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_06_c1_result.py"
    ),
    Path("src/tools/thesis_experiment/tests/test_v3_06_c1.py"),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES

METHOD_LEVELS = {
    "sparse_control": (),
    "add_v040": ("feasible_v040",),
    "add_v020_v040": ("feasible_v020", "feasible_v040"),
}
ORIGINAL_IDS = {
    "certified_stop_v000",
    "feasible_v060",
    "feasible_v080",
    "feasible_v100",
    "feasible_v120",
    "teb_fallback_v080",
}


def derived_blocks():
    rows = []
    for block_index in range(4):
        rows.append(
            {
                "block_id": "development_{}".format(6511 + block_index),
                "seed": 6511 + block_index,
                "angle_deg": round(
                    72.0 + ((block_index * 11) % 37), 1
                ),
                "actor_start_distance_m": round(
                    2.80 + ((block_index * 5) % 14) * 0.05, 3
                ),
                "actor_base_speed_mps": round(
                    0.875 + ((block_index * 3) % 7) * 0.05, 3
                ),
            }
        )
    return rows


def effective_contract():
    parent = copy.deepcopy(
        BASE_LOAD_YAML_RELATIVE(WORKSPACE, PARENT_CONTRACT_RELATIVE)
    )
    design = BASE_LOAD_YAML_RELATIVE(WORKSPACE, CONTRACT_RELATIVE)
    parent["stage"] = "V3-06-C1"
    parent["stage_purpose"] = design["stage_purpose"]
    parent["contract_id"] = design["contract_id"]
    parent["status"] = design["status"]
    parent["offline_foundation"] = design["offline_foundation"]
    parent["single_factor"] = design["single_factor"]
    parent["development_selection"] = design["development_selection"]
    parent["methods"] = {
        method: {"enabled_new_candidate_ids": list(level)}
        for method, level in METHOD_LEVELS.items()
    }
    parent["upstream"]["planner_contract_path"] = design[
        "offline_foundation"
    ]["planner_contract_path"]
    parent["upstream"]["planner_contract_sha256"] = design[
        "offline_foundation"
    ]["planner_contract_sha256"]
    parent["experimental_unit"]["scenario_blocks"] = derived_blocks()
    parent["experimental_unit"]["policies_evaluator_only"] = list(
        design["fresh_development_population"][
            "actor_policies_evaluator_only"
        ]
    )
    parent["experimental_unit"]["schedule_rows"] = 36
    parent["experimental_unit"]["evidence_budget_units"] = 36
    parent["integration_gates"]["episode_count"] = 36
    parent["integration_gates"]["goal_success_rate_min"] = 0.0
    parent["authority"] = design["authority"]
    parent["claim_if_pass"] = design["claim_if_pass"]
    parent["forbidden_claims"] = design["forbidden_claims"]
    return parent


def load_yaml_relative(workspace, relative):
    if Path(relative) == CONTRACT_RELATIVE:
        return effective_contract()
    return BASE_LOAD_YAML_RELATIVE(workspace, relative)


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-06-C1", "stage differs")
    factor = contract["single_factor"]
    require(
        contract["status"]
        == "single_factor_development_calibration_preregistered_before_release"
        and factor["factor_count"] == 1
        and factor["factor_id"] == "low_speed_candidate_density"
        and all(
            factor[key] is False
            for key in (
                "hard_verifier_changed",
                "predictor_changed",
                "certified_stop_changed",
                "ranker_changed",
            )
        ),
        "single factor differs",
    )
    require(
        [row["seed"] for row in contract["experimental_unit"]["scenario_blocks"]]
        == [6511, 6512, 6513, 6514]
        and contract["experimental_unit"]["schedule_rows"] == 36,
        "fresh population differs",
    )
    require(
        preregistration["stage"] == "V3-06-C1"
        and preregistration["status"]
        == "frozen_before_any_v3_06_c1_outcome_or_release"
        and preregistration["fresh_population"]["exact_episodes"] == 36,
        "preregistration differs",
    )
    authority = contract["authority"]
    require(
        authority["simulation_requires_separate_release_and_authorization"]
        is True
        and all(
            value is False
            for key, value in authority.items()
            if key != "simulation_requires_separate_release_and_authorization"
        ),
        "execution authority must remain external",
    )


def exact_schedule(contract: Mapping[str, Any]) -> Dict[str, Any]:
    methods = list(METHOD_LEVELS)
    rows = []
    sequence = 0
    for block_index, block in enumerate(
        contract["experimental_unit"]["scenario_blocks"]
    ):
        for policy_index, policy in enumerate(
            contract["experimental_unit"]["policies_evaluator_only"]
        ):
            offset = (block_index + policy_index) % len(methods)
            ordered = methods[offset:] + methods[:offset]
            for method in ordered:
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "unit_id": "v3_06_c1_{}_{}_{}".format(
                            block["block_id"], policy, method
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
                        "method": method,
                        "enabled_new_candidate_ids": list(
                            METHOD_LEVELS[method]
                        ),
                        "attempt": 1,
                    }
                )
    require(len(rows) == 36, "schedule count differs")
    return {
        "schema_version": "1.0",
        "stage": "V3-06-C1",
        "schedule_id": "v3_scope_v3_06_c1_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 36,
        "episode_count": 36,
        "block_count": 4,
        "seeds": [6511, 6512, 6513, 6514],
        "methods": methods,
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }


def apply_selection_method(result, row, planner):
    method = row.get("method", "")
    require(method in METHOD_LEVELS, "unknown density method")
    if method == "add_v020_v040":
        return result
    allowed = ORIGINAL_IDS | set(METHOD_LEVELS[method])
    accepted = [
        item
        for item in result["evaluations"]
        if item["accepted"] is True and item["candidate_id"] in allowed
    ]
    require(accepted, "allowed set has no accepted candidate")
    selected = sorted(
        accepted,
        key=lambda item: (-item["analytic_score"], item["candidate_id"]),
    )[0]
    stop_id = planner["candidate_generator"]["certified_stop_candidate_id"]
    fallback_reason = (
        "ONLY_CERTIFIED_STOP_ACCEPTED"
        if selected["candidate_id"] == stop_id
        else ""
    )
    value = copy.deepcopy(result)
    artifact = {
        "factor_id": "low_speed_candidate_density",
        "method": method,
        "enabled_new_candidate_ids": list(METHOD_LEVELS[method]),
        "selection_scope": "hard_verifier_accepted_allowed_candidate_set",
    }
    decision = {
        "schema_version": "1.0",
        "scene_snapshot_id": result["decision"]["scene_snapshot_id"],
        "candidate_set_digest": result["decision"]["candidate_set_digest"],
        "selected_candidate_id": selected["candidate_id"],
        "policy_kind": method,
        "policy_artifact_digest": core.digest(artifact),
        "safety_fallback_used": bool(fallback_reason),
        "fallback_reason": fallback_reason,
    }
    decision["decision_digest"] = core.digest(decision)
    value["decision"] = decision
    return value
