"""V3-05-B0 repaired formal evaluation bindings."""

import copy
import importlib.util
from pathlib import Path
from typing import Any, Dict, Mapping


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_05_a0.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v3_05_a0_base_for_b0", PARENT_PATH
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
    "v3_05_b0_repaired_formal_evaluation_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_05_b0_repaired_formal_evaluation_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_05_b0_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/formal/v3_05_b0")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_05_b0_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_05_b0_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_05_b0_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_05_b0_execution_release.yaml"
REVIEW_RELATIVE = (
    READINESS_RELATIVE / "v3_05_b0_independent_release_review.yaml"
)
READINESS_FREEZE_RELATIVE = (
    READINESS_RELATIVE / "v3_05_b0_readiness_freeze.sha256"
)
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_05_b0_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_05_b0_formal_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = (
    EXECUTION_RELATIVE / "v3_05_b0_evidence_freeze.sha256"
)
INDEPENDENT_RESULT_REVIEW_RELATIVE = (
    EXECUTION_RELATIVE / "v3_05_b0_independent_result_review.yaml"
)
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_05_b0_terminal_freeze.sha256"

SOURCE_RESOURCES = PARENT.SOURCE_RESOURCES + (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path("artifacts/v3/formal/v3_05_a0/v3_05_a0_terminal_freeze.sha256"),
    Path(
        "artifacts/v3/formal/v3_05_a0/"
        "v3_05_a0_terminal_failure_record.yaml"
    ),
    Path("artifacts/v3/formal/v3_05_r1/v3_05_r1_port_repair_report.yaml"),
    Path(
        "artifacts/v3/formal/v3_05_r1/"
        "v3_05_r1_independent_review.yaml"
    ),
    Path(
        "artifacts/v3/formal/v3_05_r1/"
        "v3_05_r1_evidence_freeze.sha256"
    ),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_05_b0.py"),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_05_b0_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_05_b0_readiness.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_05_b0_formal.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "assess_v3_05_b0_formal.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_05_b0_result.py"
    ),
    Path("src/tools/thesis_experiment/tests/test_v3_05_b0.py"),
)
BINARY_RESOURCES = PARENT.BINARY_RESOURCES


def derived_blocks():
    rows = []
    for block_index in range(30):
        rows.append(
            {
                "block_id": "formal_{}".format(6451 + block_index),
                "seed": 6451 + block_index,
                "angle_deg": round(
                    70.5 + ((block_index * 11) % 40), 1
                ),
                "actor_start_distance_m": round(
                    2.725 + ((block_index * 7) % 15) * 0.05, 3
                ),
                "actor_base_speed_mps": round(
                    0.875 + ((block_index * 5) % 7) * 0.05, 3
                ),
            }
        )
    return rows


def effective_contract():
    parent = copy.deepcopy(
        BASE_LOAD_YAML_RELATIVE(WORKSPACE, PARENT_CONTRACT_RELATIVE)
    )
    repair = BASE_LOAD_YAML_RELATIVE(WORKSPACE, CONTRACT_RELATIVE)
    parent["stage"] = "V3-05-B0"
    parent["stage_purpose"] = repair["stage_purpose"]
    parent["contract_id"] = repair["contract_id"]
    parent["status"] = repair["status"]
    parent["parent_failure"] = repair["parent"]
    parent["offline_repair"] = repair["offline_repair"]
    parent["single_factor_repair"] = repair["single_factor_repair"]
    parent["runtime_port_allocation"] = copy.deepcopy(
        repair["single_factor_repair"]["runtime_port_allocation"]
    )
    parent["experimental_unit"]["scenario_blocks"] = derived_blocks()
    parent["experimental_unit"]["schedule_rows"] = 270
    parent["experimental_unit"]["evidence_budget_units"] = 270
    return parent


def load_yaml_relative(workspace, relative):
    if Path(relative) == CONTRACT_RELATIVE:
        return effective_contract()
    return BASE_LOAD_YAML_RELATIVE(workspace, relative)


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-05-B0", "stage differs")
    factor = contract["single_factor_repair"]
    require(
        contract["status"]
        == "single_factor_execution_repair_preregistered_before_release"
        and factor["factor_count"] == 1
        and factor["factor_id"]
        == "gazebo_master_port_allocation_below_ephemeral_range"
        and factor["planner_model_or_method_changed"] is False
        and factor["statistical_design_or_gate_changed"] is False
        and factor["closed_loop_semantics_changed"] is False,
        "single factor differs",
    )
    allocation = contract["runtime_port_allocation"]
    require(
        allocation
        == {
            "base_port": 12000,
            "sequence_stride": 50,
            "maximum_decision_index": 38,
            "minimum_allowed_port": 12050,
            "maximum_allowed_port": 25538,
        },
        "runtime port allocation differs",
    )
    require(
        [row["seed"] for row in contract["experimental_unit"]["scenario_blocks"]]
        == list(range(6451, 6481))
        and contract["experimental_unit"]["schedule_rows"] == 270,
        "fresh population differs",
    )
    stats = contract["formal_statistics"]
    require(
        stats["primary_comparison"]
        == "v3_analytic_vs_fixed_v120_shielded"
        and int(stats["bootstrap_resamples"]) == 20000
        and int(stats["bootstrap_seed"]) == 30505
        and float(stats["minimum_point_reduction"]) == 0.05
        and float(stats["minimum_one_sided_lower_reduction"]) == 0.02
        and float(stats["v3_goal_success_rate_min"]) == 0.95,
        "formal gates changed",
    )
    require(
        preregistration["stage"] == "V3-05-B0"
        and preregistration["status"]
        == "frozen_before_any_v3_05_b0_outcome_or_release"
        and preregistration["fresh_population"]["exact_episodes"] == 270
        and preregistration["single_factor"]["a0_resume_or_identity_reuse"]
        is False,
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
                        "unit_id": "v3_05_b0_{}_{}_{}".format(
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
        "stage": "V3-05-B0",
        "schedule_id": "v3_scope_v3_05_b0_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 270,
        "episode_count": 270,
        "block_count": 30,
        "seeds": list(range(6451, 6481)),
        "methods": methods,
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }
