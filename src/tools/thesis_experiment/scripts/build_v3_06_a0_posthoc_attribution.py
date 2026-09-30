#!/usr/bin/env python3
"""Build the pure-offline V3-06-A0 post-hoc attribution."""

import argparse
import csv
import hashlib
import importlib.util
import io
import math
from collections import Counter
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
    "v3_06_a0_posthoc_attribution_contract.yaml"
)
PLAN = Path(
    "experiments/manifests/v3/v3_06_a0_posthoc_attribution_plan.yaml"
)
ROOT = Path("artifacts/v3/analysis/v3_06_a0")
REPORT = ROOT / "v3_06_a0_posthoc_attribution.yaml"
PAIRED_CSV = ROOT / "v3_06_a0_paired_episode_table.csv"
FAILURE_CSV = ROOT / "v3_06_a0_v3_failure_roster.csv"
CLOSURE = ROOT / "v3_06_a0_dependency_closure.yaml"
EVIDENCE_FREEZE = ROOT / "v3_06_a0_evidence_freeze.sha256"
INDEPENDENT_REVIEW = ROOT / "v3_06_a0_independent_review.yaml"
TERMINAL_FREEZE = ROOT / "v3_06_a0_terminal_freeze.sha256"
RUNNER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "run_v3_03_c1_closed_loop.py"
)

BASE_RESOURCES = (
    CONTRACT,
    PLAN,
    Path("artifacts/v3/formal/v3_05_b0/v3_05_b0_terminal_freeze.sha256"),
    Path(
        "artifacts/v3/formal/v3_05_b0/execution/"
        "v3_05_b0_formal_assessment.yaml"
    ),
    Path(
        "artifacts/v3/formal/v3_05_b0/execution/"
        "v3_05_b0_independent_result_review.yaml"
    ),
    Path(
        "artifacts/v3/formal/v3_05_b0/offline_readiness/"
        "v3_05_b0_exact_schedule.yaml"
    ),
    Path(
        "artifacts/v3/planner/v3_03_c5/"
        "v3_03_c5_effective_planner_contract.yaml"
    ),
    Path(
        "artifacts/v3/prediction/v3_02_d1/model/"
        "v3_02_d1_candidate_predictor.yaml"
    ),
    Path("src/application/v3_scope_planner/src/v3_scope_planner/core.py"),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_03_c1_closed_loop.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_06_a0_posthoc_attribution.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_06_a0_posthoc_attribution.py"
    ),
    Path(
        "src/tools/thesis_experiment/tests/"
        "test_v3_06_a0_posthoc_attribution.py"
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


def load_runner():
    spec = importlib.util.spec_from_file_location(
        "v3_06_a0_frozen_runner", RUNNER_PATH
    )
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def episode_path(row):
    return (
        Path("artifacts/v3/formal/v3_05_b0/execution/episodes")
        / row["unit_id"]
        / "episode_receipt.yaml"
    )


def selected_rows(schedule):
    return [
        row
        for row in schedule["rows"]
        if row["method"] in ("v3_analytic", "fixed_v120_shielded")
    ]


def csv_bytes(rows, fieldnames):
    stream = io.StringIO()
    writer = csv.DictWriter(stream, fieldnames=fieldnames, lineterminator="\n")
    writer.writeheader()
    writer.writerows(rows)
    return stream.getvalue().encode("utf-8")


def validate(contract, plan):
    core.require(contract["stage"] == "V3-06-A0", "stage differs")
    core.require(
        contract["status"]
        == "posthoc_analysis_plan_frozen_before_report_generation",
        "contract status differs",
    )
    core.require(
        plan["stage"] == "V3-06-A0"
        and plan["analysis_kind"] == "posthoc_pure_offline_descriptive"
        and plan["selection_guard"][
            "original_v3_05_b0_formal_result_must_remain_unchanged"
        ]
        is True,
        "posthoc plan differs",
    )
    core.require(
        all(value is False for value in contract["authority"].values()),
        "authority must remain zero",
    )
    for path_key, sha_key in (
        ("terminal_freeze_path", "terminal_freeze_sha256"),
        ("assessment_path", "assessment_sha256"),
        ("independent_review_path", "independent_review_sha256"),
        ("schedule_path", "schedule_sha256"),
        ("planner_contract_path", "planner_contract_sha256"),
        ("predictor_path", "predictor_sha256"),
    ):
        path = Path(contract["upstream"][path_key])
        core.require(
            sha(read(path)) == contract["upstream"][sha_key],
            "{} hash differs".format(path_key),
        )
    rules = contract["interpretation_rules"]
    core.require(
        rules["posthoc_descriptive_only"] is True
        and rules["no_new_confirmatory_inference"] is True
        and rules["preserve_v3_05_negative_result"] is True
        and rules["preserve_hard_verifier_and_certified_stop"] is True,
        "interpretation rules differ",
    )


def episode_summary(row, episode, model, planner, runner):
    method = row["method"]
    selected_counts = Counter()
    accepted_count = 0
    exact_count = 0
    v120_accepted_count = 0
    v120_rejected_nonstop_rescue_count = 0
    preclear_stop_count = 0
    postclear_stop_count = 0
    nonfinite = 0
    for decision in episode["decisions"]:
        snapshot = decision["snapshot_runtime_input_without_policy"]
        result = runner.apply_selection_method(
            core.plan(snapshot, model, planner), row, planner
        )
        selected_id = result["decision"]["selected_candidate_id"]
        selected = next(
            item
            for item in result["evaluations"]
            if item["candidate_id"] == selected_id
        )
        selected_counts[selected_id] += 1
        accepted_count += selected["accepted"] is True
        exact_count += (
            core.digest(result) == decision["planner_result_digest"]
            and result["decision"]["decision_digest"]
            == decision["decision_digest"]
            and selected_id == decision["selected_candidate_id"]
        )
        v120 = next(
            item
            for item in result["evaluations"]
            if item["candidate_id"] == "feasible_v120"
        )
        v120_accepted_count += v120["accepted"] is True
        v120_rejected_nonstop_rescue_count += (
            method == "v3_analytic"
            and v120["accepted"] is False
            and selected_id != "certified_stop_v000"
        )
        if selected_id == "certified_stop_v000":
            if decision["actor_cleared_certificate_active"]:
                postclear_stop_count += 1
            else:
                preclear_stop_count += 1
        values = (
            float(decision["planner_latency_ms"]),
            float(decision["selected_minimum_predicted_clearance_m"]),
            float(decision["state_after"]["ego_x_m"]),
        )
        nonfinite += sum(not math.isfinite(value) for value in values)

    goal_reached = episode["goal_reached"] is True
    final_x = float(episode["final_state"]["ego_x_m"])
    goal_x = 2.5
    failure_time = 10.0
    restricted_time = (
        float(episode["goal_time_s"]) if goal_reached else failure_time
    )
    goal_deficit = max(0.0, goal_x - final_x)
    full_speed_distance_per_cycle = 1.2 * 0.25
    extra_cycles = (
        int(math.ceil(goal_deficit / full_speed_distance_per_cycle - 1e-12))
        if goal_deficit > 0.0
        else 0
    )
    return {
        "goal_reached": goal_reached,
        "restricted_goal_time_s": restricted_time,
        "decision_count": len(episode["decisions"]),
        "final_x_m": final_x,
        "goal_deficit_m": goal_deficit,
        "diagnostic_minimum_full_speed_cycles_for_deficit": extra_cycles,
        "selected_counts": dict(sorted(selected_counts.items())),
        "selected_stop_count": selected_counts["certified_stop_v000"],
        "selected_v120_count": selected_counts["feasible_v120"],
        "selected_lower_nonstop_count": sum(
            count
            for candidate, count in selected_counts.items()
            if candidate
            not in ("certified_stop_v000", "feasible_v120")
        ),
        "preclear_stop_count": preclear_stop_count,
        "postclear_stop_count": postclear_stop_count,
        "v120_accepted_count": v120_accepted_count,
        "v120_rejected_nonstop_rescue_count": (
            v120_rejected_nonstop_rescue_count
        ),
        "selected_candidate_accepted_count": accepted_count,
        "exact_replay_count": exact_count,
        "nonfinite_count": nonfinite,
    }


def build():
    contract = load_yaml(CONTRACT)
    plan = load_yaml(PLAN)
    validate(contract, plan)
    assessment = load_yaml(Path(contract["upstream"]["assessment_path"]))
    schedule = load_yaml(Path(contract["upstream"]["schedule_path"]))
    planner = load_yaml(Path(contract["upstream"]["planner_contract_path"]))
    model = load_yaml(Path(contract["upstream"]["predictor_path"]))
    runner = load_runner()
    rows = selected_rows(schedule)
    core.require(len(rows) == 180, "selected receipt roster differs")

    summaries = {}
    action_totals = {
        "v3_analytic": Counter(),
        "fixed_v120_shielded": Counter(),
    }
    decision_totals = Counter()
    accepted_total = 0
    exact_total = 0
    nonfinite_total = 0
    receipt_paths = []
    for row in rows:
        path = episode_path(row)
        receipt_paths.append(path)
        episode = load_yaml(path)
        summary = episode_summary(row, episode, model, planner, runner)
        key = (
            row["block_id"],
            row["actor_policy_evaluator_only"],
            row["method"],
        )
        summaries[key] = (row, summary)
        action_totals[row["method"]].update(summary["selected_counts"])
        decision_totals[row["method"]] += summary["decision_count"]
        accepted_total += summary["selected_candidate_accepted_count"]
        exact_total += summary["exact_replay_count"]
        nonfinite_total += summary["nonfinite_count"]

    paired_rows = []
    failure_rows = []
    paired_outcomes = Counter()
    v3_failures_by_policy = Counter()
    baseline_failures_by_policy = Counter()
    v3_only_success = 0
    baseline_only_success = 0
    both_success = 0
    both_fail = 0
    paired_relative_effects = []
    paired_time_deltas = []
    for block_id in sorted({row["block_id"] for row in rows}):
        for policy in ("cautious", "assertive", "hesitant"):
            v3_row, v3 = summaries[(block_id, policy, "v3_analytic")]
            baseline_row, baseline = summaries[
                (block_id, policy, "fixed_v120_shielded")
            ]
            core.require(
                v3_row["seed"] == baseline_row["seed"]
                and v3_row["angle_deg"] == baseline_row["angle_deg"]
                and v3_row["actor_start_distance_m"]
                == baseline_row["actor_start_distance_m"]
                and v3_row["actor_base_speed_mps"]
                == baseline_row["actor_base_speed_mps"],
                "paired scene differs",
            )
            if v3["goal_reached"] and baseline["goal_reached"]:
                outcome_class = "both_success"
                both_success += 1
            elif v3["goal_reached"]:
                outcome_class = "v3_only_success"
                v3_only_success += 1
            elif baseline["goal_reached"]:
                outcome_class = "fixed_v120_only_success"
                baseline_only_success += 1
            else:
                outcome_class = "both_fail"
                both_fail += 1
            paired_outcomes[outcome_class] += 1
            delta = (
                baseline["restricted_goal_time_s"]
                - v3["restricted_goal_time_s"]
            )
            relative = delta / baseline["restricted_goal_time_s"]
            paired_time_deltas.append(delta)
            paired_relative_effects.append(relative)
            paired = {
                "block_id": block_id,
                "seed": v3_row["seed"],
                "angle_deg": v3_row["angle_deg"],
                "actor_start_distance_m": v3_row[
                    "actor_start_distance_m"
                ],
                "actor_base_speed_mps": v3_row["actor_base_speed_mps"],
                "actor_policy_evaluator_only": policy,
                "outcome_class": outcome_class,
                "v3_goal_reached": v3["goal_reached"],
                "v3_restricted_goal_time_s": v3[
                    "restricted_goal_time_s"
                ],
                "v3_final_x_m": v3["final_x_m"],
                "v3_goal_deficit_m": v3["goal_deficit_m"],
                "v3_stop_decisions": v3["selected_stop_count"],
                "v3_v120_decisions": v3["selected_v120_count"],
                "v3_lower_nonstop_decisions": v3[
                    "selected_lower_nonstop_count"
                ],
                "v3_v120_rejected_nonstop_rescues": v3[
                    "v120_rejected_nonstop_rescue_count"
                ],
                "fixed_v120_goal_reached": baseline["goal_reached"],
                "fixed_v120_restricted_goal_time_s": baseline[
                    "restricted_goal_time_s"
                ],
                "fixed_v120_final_x_m": baseline["final_x_m"],
                "fixed_v120_goal_deficit_m": baseline["goal_deficit_m"],
                "fixed_v120_stop_decisions": baseline[
                    "selected_stop_count"
                ],
                "paired_time_reduction_s": delta,
                "paired_relative_reduction": relative,
            }
            paired_rows.append(paired)
            if not v3["goal_reached"]:
                v3_failures_by_policy[policy] += 1
                failure_rows.append(
                    {
                        "unit_id": v3_row["unit_id"],
                        "block_id": block_id,
                        "seed": v3_row["seed"],
                        "angle_deg": v3_row["angle_deg"],
                        "actor_start_distance_m": v3_row[
                            "actor_start_distance_m"
                        ],
                        "actor_base_speed_mps": v3_row[
                            "actor_base_speed_mps"
                        ],
                        "actor_policy_evaluator_only": policy,
                        "final_x_m": v3["final_x_m"],
                        "goal_deficit_m": v3["goal_deficit_m"],
                        "diagnostic_minimum_full_speed_cycles_for_deficit": v3[
                            "diagnostic_minimum_full_speed_cycles_for_deficit"
                        ],
                        "stop_decisions": v3["selected_stop_count"],
                        "preclear_stop_decisions": v3[
                            "preclear_stop_count"
                        ],
                        "postclear_stop_decisions": v3[
                            "postclear_stop_count"
                        ],
                        "fixed_v120_goal_reached": baseline[
                            "goal_reached"
                        ],
                        "paired_outcome_class": outcome_class,
                    }
                )
            if not baseline["goal_reached"]:
                baseline_failures_by_policy[policy] += 1

    v3_failure_count = len(failure_rows)
    baseline_failure_count = sum(baseline_failures_by_policy.values())
    v3_decisions = decision_totals["v3_analytic"]
    baseline_decisions = decision_totals["fixed_v120_shielded"]
    total_decisions = v3_decisions + baseline_decisions
    v3_actions = action_totals["v3_analytic"]
    baseline_actions = action_totals["fixed_v120_shielded"]
    baseline_mean = float(
        assessment["method_metrics"]["fixed_v120_shielded"][
            "restricted_goal_time_mean_s"
        ]
    )
    v3_mean = float(
        assessment["method_metrics"]["v3_analytic"][
            "restricted_goal_time_mean_s"
        ]
    )
    observed_mean_reduction = baseline_mean - v3_mean
    target_mean_reduction = baseline_mean * 0.05
    additional_mean_reduction = (
        target_mean_reduction - observed_mean_reduction
    )
    v3_rescue_count = sum(
        summary["v120_rejected_nonstop_rescue_count"]
        for (block, policy, method), (row, summary) in summaries.items()
        if method == "v3_analytic"
    )
    failure_deficits = [row["goal_deficit_m"] for row in failure_rows]
    near_one_cycle_count = sum(value <= 0.3 + 1e-12 for value in failure_deficits)
    metrics = {
        "receipt_count": len(receipt_paths),
        "paired_episode_count": len(paired_rows),
        "v3_episode_count": 90,
        "fixed_v120_episode_count": 90,
        "v3_decision_count": v3_decisions,
        "fixed_v120_decision_count": baseline_decisions,
        "exact_replay_rate": exact_total / total_decisions,
        "selected_candidate_accepted_rate": accepted_total / total_decisions,
        "nonfinite_output_count": nonfinite_total,
        "v3_failure_count": v3_failure_count,
        "fixed_v120_failure_count": baseline_failure_count,
        "v3_failures_by_policy": dict(v3_failures_by_policy),
        "fixed_v120_failures_by_policy": dict(
            baseline_failures_by_policy
        ),
        "paired_outcome_partition": {
            "both_success": both_success,
            "v3_only_success": v3_only_success,
            "fixed_v120_only_success": baseline_only_success,
            "both_fail": both_fail,
        },
        "v3_failure_goal_deficit_mean_m": (
            sum(failure_deficits) / len(failure_deficits)
        ),
        "v3_failure_goal_deficit_max_m": max(failure_deficits),
        "v3_failures_within_one_full_speed_cycle_count": (
            near_one_cycle_count
        ),
    }
    action_composition = {
        "v3_analytic": {
            "counts": dict(sorted(v3_actions.items())),
            "certified_stop_rate": (
                v3_actions["certified_stop_v000"] / v3_decisions
            ),
            "feasible_v120_rate": (
                v3_actions["feasible_v120"] / v3_decisions
            ),
            "lower_nonstop_rate": (
                sum(
                    count
                    for candidate, count in v3_actions.items()
                    if candidate
                    not in ("certified_stop_v000", "feasible_v120")
                )
                / v3_decisions
            ),
            "v120_rejected_nonstop_rescue_count": v3_rescue_count,
            "v120_rejected_nonstop_rescue_rate": (
                v3_rescue_count / v3_decisions
            ),
        },
        "fixed_v120_shielded": {
            "counts": dict(sorted(baseline_actions.items())),
            "certified_stop_rate": (
                baseline_actions["certified_stop_v000"]
                / baseline_decisions
            ),
            "feasible_v120_rate": (
                baseline_actions["feasible_v120"] / baseline_decisions
            ),
        },
    }
    effect_gap = {
        "baseline_restricted_goal_time_mean_s": baseline_mean,
        "v3_restricted_goal_time_mean_s": v3_mean,
        "observed_mean_reduction_s": observed_mean_reduction,
        "observed_point_relative_reduction": float(
            assessment["primary_statistics"]["point_relative_reduction"]
        ),
        "frozen_point_gate": 0.05,
        "target_mean_reduction_s_at_five_percent": target_mean_reduction,
        "additional_mean_reduction_s_needed": additional_mean_reduction,
        "additional_quarter_second_cycles_per_episode_equivalent": (
            additional_mean_reduction / 0.25
        ),
        "interpretation": (
            "The frozen analytic ranker already selects the 1.20 m/s "
            "candidate whenever it is hard-accepted. Its only structural "
            "advantage over the fixed-1.20 policy is choosing a lower-speed "
            "accepted candidate when 1.20 m/s is rejected, so the observed "
            "effect is bounded by relatively sparse rescue decisions."
        ),
    }
    gates_spec = contract["integrity_gates"]
    gates = {
        "receipt_roster_pass": metrics["receipt_count"]
        == int(gates_spec["exact_episode_receipt_count"]),
        "paired_population_pass": metrics["paired_episode_count"]
        == int(gates_spec["exact_paired_episode_count"]),
        "v3_failure_roster_pass": metrics["v3_failure_count"]
        == int(gates_spec["exact_v3_failure_count"]),
        "fixed_v120_failure_roster_pass": metrics[
            "fixed_v120_failure_count"
        ]
        == int(gates_spec["exact_fixed_v120_failure_count"]),
        "exact_replay_pass": metrics["exact_replay_rate"]
        >= float(gates_spec["exact_replay_rate_min"]),
        "selected_candidate_accepted_pass": metrics[
            "selected_candidate_accepted_rate"
        ]
        >= float(gates_spec["selected_candidate_accepted_rate_min"]),
        "finite_output_pass": metrics["nonfinite_output_count"]
        <= int(gates_spec["nonfinite_output_count_max"]),
        "formal_negative_result_preserved_pass": (
            assessment["formal_result"] is True
            and assessment["performance_improvement_demonstrated"] is False
        ),
        "zero_authority_pass": all(
            value is False for value in contract["authority"].values()
        ),
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-A0",
        "report_id": "v3_06_a0_posthoc_attribution_1",
        "status": (
            "v3_06_a0_posthoc_attribution_closure_pass"
            if all(gates.values())
            else "v3_06_a0_posthoc_attribution_fail"
        ),
        "analysis_kind": "posthoc_pure_offline_descriptive",
        "formal_result": False,
        "upstream_formal_result_preserved": True,
        "metrics": metrics,
        "action_composition": action_composition,
        "effect_gate_gap": effect_gap,
        "v3_failure_roster": failure_rows,
        "integrity_gates": gates,
        "overall_pass": all(gates.values()),
        "attribution": {
            "success_gate_failure": (
                "six V3 timeouts leave 93.33% success, below the frozen "
                "95% absolute gate; four of the six final positions are "
                "diagnostically within one 0.25 s full-speed distance of "
                "the goal, but this is not a counterfactual safety claim"
            ),
            "efficiency_gate_failure": (
                "V3 gains only where fixed-v120 must stop and a lower-speed "
                "candidate remains hard-accepted; the frozen greedy ranker "
                "cannot beat fixed-v120 when v120 itself is accepted"
            ),
            "paired_noninferiority_source": (
                "V3 converts some fixed-v120 failures to successes and has "
                "no fixed-v120-only success in the paired outcome partition"
            ),
        },
        "next_single_factor_direction": {
            "factor_id": "anticipatory_stop_avoidance_ranker",
            "change": (
                "replace immediate-progress-only ranking with a deterministic "
                "pre-conflict stop-avoidance score over the same hard-accepted "
                "candidate set"
            ),
            "hard_verifier_changed": False,
            "candidate_generator_changed": False,
            "certified_stop_changed": False,
            "predictor_changed": False,
            "learning_activated": False,
            "required_next_evidence": (
                "fresh bounded development identities followed, only if the "
                "factor gate passes, by a separately preregistered fresh "
                "held-out paired evaluation"
            ),
        },
        "claim": {
            "posthoc_attribution_complete": all(gates.values()),
            "new_performance_improvement": False,
            "actual_teb_superiority": False,
            "general_navigation_safety": False,
            "deployment_ready": False,
            "real_vehicle_ready": False,
        },
        "authority": contract["authority"],
    }

    paired_fields = list(paired_rows[0].keys())
    failure_fields = list(failure_rows[0].keys())
    outputs = {
        REPORT: yaml_bytes(report),
        PAIRED_CSV: csv_bytes(paired_rows, paired_fields),
        FAILURE_CSV: csv_bytes(failure_rows, failure_fields),
    }
    resources = BASE_RESOURCES + tuple(receipt_paths)
    closure = {
        "schema_version": "1.0",
        "stage": "V3-06-A0",
        "status": "dependency_closure_complete",
        "unresolved_count": 0,
        "resources": [receipt(path) for path in resources],
    }
    outputs[CLOSURE] = yaml_bytes(closure)
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
    if args.check:
        for path, payload in outputs.items():
            core.require(
                (W / path).is_file() and (W / path).read_bytes() == payload,
                "output differs: {}".format(path),
            )
    else:
        for path, payload in outputs.items():
            write_once(path, payload)
    report = yaml.safe_load(outputs[REPORT])
    print(
        "status={} report_sha256={}".format(
            report["status"], sha(outputs[REPORT])
        )
    )


if __name__ == "__main__":
    main()
