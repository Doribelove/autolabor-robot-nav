"""V3-05-A0 formal paired evaluation bindings."""

import copy
import importlib.util
from pathlib import Path
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c6.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v3_03_c6_base_for_v3_05", PARENT_PATH
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
    "v3_05_a0_formal_paired_evaluation_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_05_a0_formal_paired_evaluation_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_05_a0_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/formal/v3_05_a0")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_05_a0_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_05_a0_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_05_a0_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_05_a0_execution_release.yaml"
REVIEW_RELATIVE = (
    READINESS_RELATIVE / "v3_05_a0_independent_release_review.yaml"
)
READINESS_FREEZE_RELATIVE = (
    READINESS_RELATIVE / "v3_05_a0_readiness_freeze.sha256"
)
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_05_a0_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = (
    EXECUTION_RELATIVE / "v3_05_a0_formal_assessment.yaml"
)
EVIDENCE_FREEZE_RELATIVE = (
    EXECUTION_RELATIVE / "v3_05_a0_evidence_freeze.sha256"
)
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_05_a0_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_05_a0_terminal_freeze.sha256"

SOURCE_RESOURCES = PARENT.SOURCE_RESOURCES + (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path("artifacts/v3/planner/v3_03_c6/v3_03_c6_terminal_freeze.sha256"),
    Path("artifacts/v3/learning/v3_04_a0/v3_04_a0_terminal_freeze.sha256"),
    Path(
        "artifacts/v3/learning/v3_04_a0/"
        "v3_04_a0_learning_value_support_audit.yaml"
    ),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_05_a0.py"),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_05_a0_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_05_a0_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_05_a0_formal.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_05_a0_formal.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_05_a0_result.py"
    ),
    Path("src/tools/thesis_experiment/tests/test_v3_05_a0.py"),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES


def derived_blocks():
    rows = []
    for block_index in range(30):
        rows.append(
            {
                "block_id": "formal_{}".format(6401 + block_index),
                "seed": 6401 + block_index,
                "angle_deg": float(70 + ((block_index * 7) % 41)),
                "actor_start_distance_m": round(
                    2.70 + ((block_index * 13) % 15) * 0.05, 2
                ),
                "actor_base_speed_mps": round(
                    0.85 + ((block_index * 11) % 7) * 0.05, 2
                ),
            }
        )
    return rows


def effective_contract():
    parent = copy.deepcopy(
        BASE_LOAD_YAML_RELATIVE(WORKSPACE, PARENT_CONTRACT_RELATIVE)
    )
    design = BASE_LOAD_YAML_RELATIVE(WORKSPACE, CONTRACT_RELATIVE)
    parent["stage"] = "V3-05-A0"
    parent["stage_purpose"] = design["stage_purpose"]
    parent["contract_id"] = design["contract_id"]
    parent["status"] = design["status"]
    parent["upstream_freezes"] = design["upstream"]
    parent["methods"] = design["methods"]
    parent["formal_statistics"] = design["formal_statistics"]
    parent["experimental_unit"]["policies_evaluator_only"] = design[
        "experimental_unit"
    ]["actor_policies_evaluator_only"]
    parent["experimental_unit"]["scenario_blocks"] = derived_blocks()
    parent["experimental_unit"]["schedule_rows"] = 270
    parent["experimental_unit"]["evidence_budget_units"] = 270
    parent["experimental_unit"]["retry_allowed"] = False
    parent["experimental_unit"]["resume_allowed"] = False
    # Pooled baseline outcomes are not required to reach the goal; the formal
    # method-specific task and noninferiority gates below remain frozen.
    parent["integration_gates"]["episode_count"] = 270
    parent["integration_gates"]["goal_success_rate_min"] = 0.0
    parent["claim_if_pass"] = design["claim_if_positive"]
    parent["forbidden_claims"] = design["forbidden_claims"]
    return parent


def load_yaml_relative(workspace, relative):
    if Path(relative) == CONTRACT_RELATIVE:
        return effective_contract()
    return BASE_LOAD_YAML_RELATIVE(workspace, relative)


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-05-A0", "stage differs")
    require(
        contract["status"] == "formal_evaluation_preregistered_before_release",
        "status differs",
    )
    require(
        contract["upstream"]["planner_contract_sha256"]
        == "6f2ec1530b67785dfc8f5a397ab416847f35f0c6ffc9e6f45bbf633a50057761"
        and contract["frozen_model"]["sha256"]
        == "fa4ef83b9b2e534da3031001f5dd0036cd7f983ebfbe73fe20f45c4b73bc55d7",
        "frozen planner/model differs",
    )
    require(
        len(contract["experimental_unit"]["scenario_blocks"]) == 30
        and [row["seed"] for row in contract["experimental_unit"]["scenario_blocks"]]
        == list(range(6401, 6431))
        and contract["experimental_unit"]["schedule_rows"] == 270,
        "formal population differs",
    )
    require(
        sorted(
            value["method_id"]
            for key, value in contract["methods"].items()
            if key != "shared"
        )
        == [
            "fixed_v080_shielded",
            "fixed_v120_shielded",
            "v3_analytic",
        ],
        "method roster differs",
    )
    stats = contract["formal_statistics"]
    require(
        stats["primary_comparison"]
        == "v3_analytic_vs_fixed_v120_shielded"
        and int(stats["bootstrap_resamples"]) == 20000
        and int(stats["bootstrap_seed"]) == 30505
        and float(stats["minimum_point_reduction"]) == 0.05
        and float(stats["minimum_one_sided_lower_reduction"]) == 0.02,
        "formal statistics differ",
    )
    require(
        preregistration["stage"] == "V3-05-A0"
        and preregistration["status"]
        == "frozen_before_any_v3_05_outcome_or_release"
        and preregistration["fresh_population"]["exact_episodes"] == 270,
        "preregistration differs",
    )


def exact_schedule(contract: Mapping[str, Any]) -> Dict[str, Any]:
    methods = [
        "v3_analytic",
        "fixed_v120_shielded",
        "fixed_v080_shielded",
    ]
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
                        "unit_id": "v3_05_a0_{}_{}_{}".format(
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
                        "attempt": 1,
                    }
                )
    require(len(rows) == 270, "schedule count differs")
    return {
        "schema_version": "1.0",
        "stage": "V3-05-A0",
        "schedule_id": "v3_scope_v3_05_a0_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 270,
        "episode_count": 270,
        "block_count": 30,
        "seeds": list(range(6401, 6431)),
        "methods": methods,
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
