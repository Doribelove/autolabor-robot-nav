"""V3-06-B0 anticipatory ranker calibration bindings."""

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
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_05_b0.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v3_05_b0_base_for_v3_06_b0", PARENT_PATH
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
    "v3_06_b0_anticipatory_ranker_calibration_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_06_b0_anticipatory_ranker_calibration_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_06_b0_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/development/v3_06_b0")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_06_b0_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_06_b0_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_06_b0_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_06_b0_execution_release.yaml"
REVIEW_RELATIVE = (
    READINESS_RELATIVE / "v3_06_b0_independent_release_review.yaml"
)
READINESS_FREEZE_RELATIVE = (
    READINESS_RELATIVE / "v3_06_b0_readiness_freeze.sha256"
)
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_06_b0_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = (
    EXECUTION_RELATIVE / "v3_06_b0_calibration_assessment.yaml"
)
EVIDENCE_FREEZE_RELATIVE = (
    EXECUTION_RELATIVE / "v3_06_b0_evidence_freeze.sha256"
)
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_06_b0_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_06_b0_terminal_freeze.sha256"

SOURCE_RESOURCES = PARENT.SOURCE_RESOURCES + (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path("artifacts/v3/analysis/v3_06_a0/v3_06_a0_terminal_freeze.sha256"),
    Path(
        "artifacts/v3/analysis/v3_06_a0/"
        "v3_06_a0_posthoc_attribution.yaml"
    ),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/v3_06_b0.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_06_b0_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_06_b0_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_06_b0_calibration.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_06_b0_calibration.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_06_b0_result.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/test_v3_06_b0.py"
    ),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES


METHOD_WEIGHTS = {
    "v3_greedy_control": 0.25,
    "anticipatory_w050": 0.50,
    "anticipatory_w100": 1.00,
    "anticipatory_w200": 2.00,
}


def derived_blocks():
    rows = []
    for block_index in range(4):
        rows.append(
            {
                "block_id": "development_{}".format(6501 + block_index),
                "seed": 6501 + block_index,
                "angle_deg": round(
                    71.5 + ((block_index * 12) % 37), 1
                ),
                "actor_start_distance_m": round(
                    2.775 + ((block_index * 9) % 14) * 0.05, 3
                ),
                "actor_base_speed_mps": round(
                    0.90 + ((block_index * 4) % 7) * 0.05, 3
                ),
            }
        )
    return rows


def effective_contract():
    parent = copy.deepcopy(
        BASE_LOAD_YAML_RELATIVE(WORKSPACE, PARENT_CONTRACT_RELATIVE)
    )
    design = BASE_LOAD_YAML_RELATIVE(WORKSPACE, CONTRACT_RELATIVE)
    parent["stage"] = "V3-06-B0"
    parent["stage_purpose"] = design["stage_purpose"]
    parent["contract_id"] = design["contract_id"]
    parent["status"] = design["status"]
    parent["parent_attribution"] = design["parent_attribution"]
    parent["single_factor"] = design["single_factor"]
    parent["development_selection"] = design["development_selection"]
    parent["methods"] = {
        method: {
            "clearance_weight": weight,
            "hard_accepted_set_only": True,
        }
        for method, weight in METHOD_WEIGHTS.items()
    }
    parent["experimental_unit"]["scenario_blocks"] = derived_blocks()
    parent["experimental_unit"]["policies_evaluator_only"] = list(
        design["fresh_development_population"][
            "actor_policies_evaluator_only"
        ]
    )
    parent["experimental_unit"]["schedule_rows"] = 48
    parent["experimental_unit"]["evidence_budget_units"] = 48
    parent["experimental_unit"]["retry_allowed"] = False
    parent["experimental_unit"]["resume_allowed"] = False
    parent["integration_gates"]["episode_count"] = 48
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
    require(contract["stage"] == "V3-06-B0", "stage differs")
    factor = contract["single_factor"]
    require(
        contract["status"]
        == "single_factor_development_calibration_preregistered_before_release"
        and factor["factor_count"] == 1
        and factor["factor_id"] == "anticipatory_clearance_weight"
        and factor["hard_verifier_changed"] is False
        and factor["candidate_generator_changed"] is False
        and factor["certified_stop_changed"] is False
        and factor["predictor_changed"] is False
        and factor["learning_activated"] is False,
        "single factor differs",
    )
    require(
        [row["seed"] for row in contract["experimental_unit"]["scenario_blocks"]]
        == [6501, 6502, 6503, 6504]
        and contract["experimental_unit"]["schedule_rows"] == 48
        and METHOD_WEIGHTS
        == {
            "v3_greedy_control": 0.25,
            "anticipatory_w050": 0.50,
            "anticipatory_w100": 1.00,
            "anticipatory_w200": 2.00,
        },
        "development population or factor grid differs",
    )
    require(
        preregistration["stage"] == "V3-06-B0"
        and preregistration["status"]
        == "frozen_before_any_v3_06_b0_outcome_or_release"
        and preregistration["fresh_population"]["exact_episodes"] == 48
        and preregistration["failure_policy"][
            "no_formal_claim_from_development_data"
        ]
        is True,
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
    methods = list(METHOD_WEIGHTS)
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
                        "unit_id": "v3_06_b0_{}_{}_{}".format(
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
                        "clearance_weight": METHOD_WEIGHTS[method],
                        "attempt": 1,
                    }
                )
    require(len(rows) == 48, "schedule count differs")
    return {
        "schema_version": "1.0",
        "stage": "V3-06-B0",
        "schedule_id": "v3_scope_v3_06_b0_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 48,
        "episode_count": 48,
        "block_count": 4,
        "seeds": [6501, 6502, 6503, 6504],
        "methods": methods,
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }


def apply_selection_method(result, row, planner):
    method = row.get("method", "")
    require(method in METHOD_WEIGHTS, "unknown ranker method")
    if method == "v3_greedy_control":
        return result
    accepted_moving = [
        item
        for item in result["evaluations"]
        if item["accepted"] is True
        and item["candidate_id"].startswith("feasible_v")
    ]
    stop_id = planner["candidate_generator"]["certified_stop_candidate_id"]
    if accepted_moving:
        weight = METHOD_WEIGHTS[method]

        def score(item):
            clearance = min(
                max(float(item["minimum_clearance_m"]), 0.0), 3.0
            )
            return (
                float(item["progress_m"])
                + weight * clearance
                - 0.1 * float(item["acceleration_cost"])
            )

        selected = sorted(
            accepted_moving,
            key=lambda item: (
                -score(item),
                -float(item["progress_m"]),
                item["candidate_id"],
            ),
        )[0]
        fallback_reason = ""
    else:
        selected = next(
            item
            for item in result["evaluations"]
            if item["candidate_id"] == stop_id
        )
        require(selected["accepted"] is True, "certified stop not accepted")
        fallback_reason = "NO_ACCEPTED_NONSTOP_CANDIDATE"
    value = copy.deepcopy(result)
    artifact = {
        "factor_id": "anticipatory_clearance_weight",
        "method": method,
        "clearance_weight": METHOD_WEIGHTS[method],
        "score": (
            "progress_plus_weight_times_clipped_clearance_minus_"
            "0_1_acceleration_cost"
        ),
        "selection_scope": "hard_verifier_accepted_nonstop_set_then_stop",
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
