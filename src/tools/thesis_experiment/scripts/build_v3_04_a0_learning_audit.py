#!/usr/bin/env python3
"""Build the V3-04-A0 learning-value and support audit."""

import argparse
import hashlib
import math
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
PACKAGE_SRC = W / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core


CONTRACT = Path(
    "config/thesis_experiments/v3/"
    "v3_04_a0_learning_value_support_audit_contract.yaml"
)
PREREGISTRATION = Path(
    "experiments/manifests/v3/"
    "v3_04_a0_learning_value_support_audit_preregistration.yaml"
)
ROOT = Path("artifacts/v3/learning/v3_04_a0")
REPORT = ROOT / "v3_04_a0_learning_value_support_audit.yaml"
CLOSURE = ROOT / "v3_04_a0_dependency_closure.yaml"
EVIDENCE_FREEZE = ROOT / "v3_04_a0_evidence_freeze.sha256"
INDEPENDENT_REVIEW = ROOT / "v3_04_a0_independent_review.yaml"
TERMINAL_FREEZE = ROOT / "v3_04_a0_terminal_freeze.sha256"
C6_EPISODES = Path("artifacts/v3/planner/v3_03_c6/execution/episodes")

BASE_RESOURCES = (
    CONTRACT,
    PREREGISTRATION,
    Path("artifacts/v3/planner/v3_03_c6/v3_03_c6_terminal_freeze.sha256"),
    Path("artifacts/v3/planner/v3_03_c6/v3_03_c6_terminal_result.yaml"),
    Path(
        "artifacts/v3/planner/v3_03_c5/"
        "v3_03_c5_effective_planner_contract.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/offline_readiness/"
        "v3_02_d1_exact_schedule.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/execution/"
        "v3_02_d1_data_qualification.yaml"
    ),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/core.py"),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_04_a0_learning_audit.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_04_a0_learning_audit.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_04_a0_learning_audit.py"
    ),
)


def read(path):
    return (W / path).read_bytes()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load_yaml(path):
    value = yaml.safe_load(read(path))
    core.require(isinstance(value, dict), "YAML root is not mapping")
    return value


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, width=1000000
    ).encode("utf-8")


def receipt(path):
    payload = read(path)
    return {
        "path": path.as_posix(),
        "sha256": sha(payload),
        "size_bytes": len(payload),
    }


def write_once(path, payload):
    absolute = W / path
    if absolute.exists():
        core.require(
            absolute.read_bytes() == payload,
            "refusing overwrite: {}".format(path),
        )
    else:
        absolute.parent.mkdir(parents=True, exist_ok=True)
        absolute.write_bytes(payload)


def episode_paths():
    return tuple(
        path.relative_to(W)
        for path in sorted((W / C6_EPISODES).glob("*/episode_receipt.yaml"))
    )


def validate(contract, preregistration):
    core.require(contract["stage"] == "V3-04-A0", "stage differs")
    core.require(
        contract["status"] == "pure_offline_preregistered_before_audit",
        "status differs",
    )
    core.require(
        preregistration["stage"] == "V3-04-A0"
        and preregistration["status"]
        == "frozen_before_learning_value_support_audit",
        "preregistration differs",
    )
    core.require(
        contract["audit_sequence"]
        == [
            "supervised_ranker_value",
            "offline_contextual_bandit_support",
            "sequential_offline_rl_support",
        ],
        "learning audit sequence differs",
    )
    disposition = contract["expected_fail_closed_disposition"]
    core.require(
        disposition["if_any_activation_requirement_fails"]
        == "do_not_train"
        and disposition["retain_analytic_ranker"] is True
        and disposition["training_steps"] == 0
        and disposition["model_artifact_allowed"] is False,
        "fail-closed learning disposition differs",
    )
    core.require(
        all(value is False for value in contract["authority"].values()),
        "authority must remain zero",
    )
    for path_key, sha_key in (
        ("v3_03_terminal_freeze_path", "v3_03_terminal_freeze_sha256"),
        ("planner_contract_path", "planner_contract_sha256"),
        ("predictor_path", "predictor_sha256"),
        ("cloned_schedule_path", "cloned_schedule_sha256"),
    ):
        core.require(
            sha(read(Path(contract["upstream"][path_key])))
            == contract["upstream"][sha_key],
            "{} hash differs".format(path_key),
        )


def build():
    contract = load_yaml(CONTRACT)
    preregistration = load_yaml(PREREGISTRATION)
    validate(contract, preregistration)
    planner = load_yaml(Path(contract["upstream"]["planner_contract_path"]))
    model = load_yaml(Path(contract["upstream"]["predictor_path"]))
    cloned_schedule = load_yaml(
        Path(contract["upstream"]["cloned_schedule_path"])
    )
    episodes = [load_yaml(path) for path in episode_paths()]
    selected_total = 0
    selected_accepted = 0
    deterministic = 0
    multi_action_decisions = 0
    alternative_accepted_actions = 0
    maximum_progress_choices = 0
    progress_regrets = []
    nonfinite = 0
    for episode in episodes:
        for row in episode["decisions"]:
            snapshot = row["snapshot_runtime_input_without_policy"]
            first = core.plan(snapshot, model, planner)
            second = core.plan(snapshot, model, planner)
            selected_id = first["decision"]["selected_candidate_id"]
            accepted = [
                item for item in first["evaluations"] if item["accepted"]
            ]
            selected = next(
                item
                for item in accepted
                if item["candidate_id"] == selected_id
            )
            maximum_progress = max(
                float(item["progress_m"]) for item in accepted
            )
            regret = maximum_progress - float(selected["progress_m"])
            selected_total += 1
            selected_accepted += 1
            deterministic += core.canonical_bytes(first) == core.canonical_bytes(
                second
            )
            progress_regrets.append(regret)
            maximum_progress_choices += regret <= 1e-12
            if len(accepted) > 1:
                multi_action_decisions += 1
                alternative_accepted_actions += len(accepted) - 1
            nonfinite += not math.isfinite(regret)

    cloned_rows = cloned_schedule["rows"]
    cloned_propensities = [float(row["propensity"]) for row in cloned_rows]
    cloned_candidates = {
        row["candidate_id_evaluator_only"] for row in cloned_rows
    }
    cloned_roles = {
        row["role_evaluator_only"] for row in cloned_rows
    }
    # C6 is a deterministic deployed-policy log. For every exact context only
    # the selected action was executed; all alternative accepted actions have
    # zero conditional propensity. D1 has randomized one-step clones, but no
    # next-decision action/transition sequence and therefore cannot repair
    # sequential closed-loop support.
    contextual_minimum_propensity = (
        0.0 if alternative_accepted_actions > 0 else 1.0
    )
    sequential_transition_support = False
    thresholds = contract["activation_requirements"]
    stable_regret = max(progress_regrets) >= float(
        thresholds["stable_immediate_progress_regret_m_min"]
    )
    contextual_support = contextual_minimum_propensity >= float(
        thresholds["contextual_action_propensity_min"]
    )
    sequential_support = (
        sequential_transition_support
        if thresholds["sequential_transition_support_required"]
        else True
    )
    activation = stable_regret and contextual_support and sequential_support
    metrics = {
        "c6_episode_count": len(episodes),
        "c6_decision_count": selected_total,
        "multi_safe_action_decision_count": multi_action_decisions,
        "alternative_accepted_action_count": alternative_accepted_actions,
        "analytic_selected_accepted_rate": selected_accepted / selected_total,
        "analytic_maximum_progress_rate": (
            maximum_progress_choices / selected_total
        ),
        "immediate_progress_regret_mean_m": (
            sum(progress_regrets) / len(progress_regrets)
        ),
        "immediate_progress_regret_max_m": max(progress_regrets),
        "c6_contextual_alternative_action_propensity_min": (
            contextual_minimum_propensity
        ),
        "d1_cloned_row_count": len(cloned_rows),
        "d1_cloned_candidate_count": len(cloned_candidates),
        "d1_cloned_propensity_min": min(cloned_propensities),
        "d1_roles": sorted(cloned_roles),
        "d1_sequential_closed_loop_transition_support": (
            sequential_transition_support
        ),
        "audit_deterministic_replay_rate": deterministic / selected_total,
        "nonfinite_output_count": nonfinite,
        "training_steps": 0,
        "learned_model_artifact_count": 0,
    }
    observations = {
        "stable_immediate_regret_requirement_pass": stable_regret,
        "contextual_action_support_requirement_pass": contextual_support,
        "sequential_transition_support_requirement_pass": sequential_support,
        "all_learning_activation_requirements_pass": activation,
    }
    gates = contract["audit_gates"]
    integrity = {
        "frozen_episode_count_pass": metrics["c6_episode_count"]
        == int(gates["c6_episode_count"]),
        "frozen_decision_count_pass": metrics["c6_decision_count"]
        == int(gates["c6_decision_count"]),
        "analytic_selected_accepted_pass": metrics[
            "analytic_selected_accepted_rate"
        ]
        >= float(gates["analytic_selected_accepted_rate_min"]),
        "analytic_maximum_progress_pass": metrics[
            "analytic_maximum_progress_rate"
        ]
        >= float(gates["analytic_maximum_progress_rate_min"]),
        "deterministic_replay_pass": metrics[
            "audit_deterministic_replay_rate"
        ]
        >= float(gates["audit_deterministic_replay_rate_min"]),
        "finite_output_pass": metrics["nonfinite_output_count"]
        <= int(gates["nonfinite_output_count_max"]),
        "fail_closed_no_training_pass": (
            activation is False
            and metrics["training_steps"] == 0
            and metrics["learned_model_artifact_count"] == 0
        ),
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-04-A0",
        "report_id": "v3_04_a0_learning_value_support_audit_1",
        "status": (
            "v3_04_learning_not_justified_offline_closure_pass"
            if all(integrity.values())
            else "v3_04_learning_value_support_audit_fail"
        ),
        "formal_result": False,
        "metrics": metrics,
        "learning_activation_observations": observations,
        "integrity_gates": integrity,
        "overall_pass": all(integrity.values()),
        "disposition": {
            "learning_activated": False,
            "supervised_ranker_trained": False,
            "contextual_bandit_trained": False,
            "offline_rl_trained": False,
            "analytic_ranker_retained": True,
            "reason": (
                "zero_observed_immediate_progress_regret_and_missing_"
                "closed_loop_alternative_action_sequential_support"
            ),
        },
        "claim": {
            "learning_value_support_audit_complete": all(integrity.values()),
            "learned_policy_improvement": False,
            "offline_rl_validated": False,
            "performance_improvement": False,
            "real_vehicle_ready": False,
        },
        "authority": contract["authority"],
    }
    resources = BASE_RESOURCES + episode_paths()
    closure = {
        "schema_version": "1.0",
        "stage": "V3-04-A0",
        "status": "dependency_closure_complete",
        "unresolved_count": 0,
        "resources": [receipt(path) for path in resources],
    }
    outputs = {
        REPORT: yaml_bytes(report),
        CLOSURE: yaml_bytes(closure),
    }
    outputs[EVIDENCE_FREEZE] = (
        "\n".join(
            "{}  {}".format(sha(payload), path.as_posix())
            for path, payload in outputs.items()
        )
        + "\n"
    ).encode("utf-8")
    return outputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build()
    for path, payload in outputs.items():
        if args.check:
            core.require(
                (W / path).is_file() and read(path) == payload,
                "persisted output differs: {}".format(path),
            )
        else:
            write_once(path, payload)
    report = yaml.safe_load(outputs[REPORT])
    print(
        "status={} overall_pass={} report_sha256={}".format(
            report["status"], report["overall_pass"], sha(outputs[REPORT])
        )
    )


if __name__ == "__main__":
    main()
