#!/usr/bin/env python3
"""Replay and assess the V3-05-A0 formal paired evaluation."""

import argparse
import importlib.util
import math
from pathlib import Path
import random
import sys


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_05_a0.py"
BASE_ASSESSOR_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "assess_v3_03_c1_closed_loop.py"
)
RUNNER_PATH = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "run_v3_03_c1_closed_loop.py"
)
PACKAGE_SRC = W / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.write_bytes(payload)


def bootstrap_lower(values, resamples, seed, alpha):
    generator = random.Random(seed)
    count = len(values)
    estimates = []
    for _ in range(resamples):
        sample = [values[generator.randrange(count)] for _ in range(count)]
        estimates.append(sum(sample) / count)
    estimates.sort()
    index = max(0, min(len(estimates) - 1, int(alpha * len(estimates))))
    return estimates[index]


def assess(v, base_assessor, runner):
    pooled = base_assessor.assess(v, runner)
    evidence_resources = pooled.pop("_evidence_resources")
    v.require(
        pooled["execution"]["authorized_units"] == 270
        and pooled["execution"]["consumed_units"] == 270
        and pooled["execution"]["completed_units"] == 270
        and pooled["overall_pass"] is True,
        "pooled execution/integration replay failed",
    )
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    planner = v.load_yaml_path(
        W / contract["upstream"]["planner_contract_path"]
    )
    model = v.load_yaml_path(W / contract["frozen_model"]["path"])
    rows_by_unit = {row["unit_id"]: row for row in schedule["rows"]}
    outcomes_by_unit = {
        row["unit_id"]: row for row in pooled["outcomes"]
    }
    methods = schedule["methods"]
    method_accumulators = {
        method: {
            "episode_count": 0,
            "goal_successes": 0,
            "restricted_times_s": [],
            "decision_count": 0,
            "selected_accepted": 0,
            "exact_replay": 0,
            "collision_episodes": 0,
            "certified_stop_decisions": 0,
        }
        for method in methods
    }
    block_times = {}
    failure_time = float(
        contract["closed_loop"]["maximum_episode_duration_s"]
    )
    for row in schedule["rows"]:
        method = row["method"]
        accumulator = method_accumulators[method]
        outcome = outcomes_by_unit[row["unit_id"]]
        unit_root = v.EXECUTION_RELATIVE / "episodes" / row["unit_id"]
        episode = v.load_yaml_relative(
            W, unit_root / "episode_receipt.yaml"
        )
        accumulator["episode_count"] += 1
        accumulator["goal_successes"] += outcome["goal_reached"]
        restricted_time = (
            float(outcome["goal_time_s"])
            if outcome["goal_reached"]
            else failure_time
        )
        accumulator["restricted_times_s"].append(restricted_time)
        block_times.setdefault(row["block_id"], {}).setdefault(
            method, []
        ).append(restricted_time)
        clearances = [
            float(
                episode["prefix"]["minimum_conservative_clearance_m"]
            )
        ] + [
            float(
                decision["segment"][
                    "minimum_conservative_clearance_m"
                ]
            )
            for decision in episode["decisions"]
        ]
        accumulator["collision_episodes"] += min(clearances) < 0.0
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
            replay = (
                core.digest(result) == decision["planner_result_digest"]
                and result["decision"]["decision_digest"]
                == decision["decision_digest"]
                and selected_id == decision["selected_candidate_id"]
            )
            accumulator["decision_count"] += 1
            accumulator["selected_accepted"] += selected["accepted"] is True
            accumulator["exact_replay"] += replay
            accumulator["certified_stop_decisions"] += (
                selected_id == "certified_stop_v000"
            )

    method_metrics = {}
    for method, values in method_accumulators.items():
        episode_count = values["episode_count"]
        decision_count = values["decision_count"]
        times = values.pop("restricted_times_s")
        method_metrics[method] = {
            "episode_count": episode_count,
            "goal_success_rate": values["goal_successes"] / episode_count,
            "restricted_goal_time_mean_s": sum(times) / episode_count,
            "restricted_goal_time_median_s": sorted(times)[
                episode_count // 2
            ],
            "decision_count": decision_count,
            "selected_candidate_accepted_rate": (
                values["selected_accepted"] / decision_count
            ),
            "exact_replay_rate": values["exact_replay"] / decision_count,
            "collision_episode_count": values["collision_episodes"],
            "certified_stop_decision_rate": (
                values["certified_stop_decisions"] / decision_count
            ),
        }

    block_means = {}
    for block_id, methods_for_block in sorted(block_times.items()):
        block_means[block_id] = {}
        for method in methods:
            values = methods_for_block[method]
            v.require(len(values) == 3, "block-policy pairing differs")
            block_means[block_id][method] = sum(values) / len(values)
    primary_effects = []
    secondary_effects = []
    for values in block_means.values():
        treatment = values["v3_analytic"]
        primary = values["fixed_v120_shielded"]
        secondary = values["fixed_v080_shielded"]
        primary_effects.append((primary - treatment) / primary)
        secondary_effects.append((secondary - treatment) / secondary)
    stats = contract["formal_statistics"]
    point = sum(primary_effects) / len(primary_effects)
    lower = bootstrap_lower(
        primary_effects,
        int(stats["bootstrap_resamples"]),
        int(stats["bootstrap_seed"]),
        float(stats["one_sided_alpha"]),
    )
    secondary_point = sum(secondary_effects) / len(secondary_effects)
    treatment = method_metrics["v3_analytic"]
    baseline = method_metrics["fixed_v120_shielded"]
    method_safety = all(
        values["collision_episode_count"]
        <= int(stats["collision_count_max_each_method"])
        and values["selected_candidate_accepted_rate"]
        >= float(stats["selected_candidate_accepted_rate_min_each_method"])
        and values["exact_replay_rate"]
        >= float(stats["exact_replay_rate_min_each_method"])
        for values in method_metrics.values()
    )
    formal_gates = {
        "complete_paired_matrix_pass": (
            len(block_means) == 30
            and all(
                values["episode_count"] == 90
                for values in method_metrics.values()
            )
        ),
        "shared_safety_integrity_pass": method_safety,
        "v3_goal_success_pass": treatment["goal_success_rate"]
        >= float(stats["v3_goal_success_rate_min"]),
        "goal_success_noninferiority_pass": treatment["goal_success_rate"]
        >= baseline["goal_success_rate"]
        - float(stats["goal_success_noninferiority_margin"]),
        "primary_point_effect_pass": point
        >= float(stats["minimum_point_reduction"]),
        "primary_lower_effect_pass": lower
        >= float(stats["minimum_one_sided_lower_reduction"]),
        "finite_statistics_pass": all(
            math.isfinite(value)
            for value in (point, lower, secondary_point)
        ),
    }
    positive = all(formal_gates.values())
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": contract["stage"],
        "assessment_id": "{}_formal_assessment_1".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "status": (
            "{}_formal_positive_result".format(
                contract["stage"].lower().replace("-", "_")
            )
            if positive
            else "{}_formal_valid_negative_result".format(
                contract["stage"].lower().replace("-", "_")
            )
        ),
        "formal_result": True,
        "performance_improvement_demonstrated": positive,
        "execution": pooled["execution"],
        "pooled_integration_metrics": pooled["metrics"],
        "method_metrics": method_metrics,
        "primary_statistics": {
            "comparison": "v3_analytic_vs_fixed_v120_shielded",
            "statistical_unit_count": len(block_means),
            "point_relative_reduction": point,
            "one_sided_lower_relative_reduction": lower,
            "one_sided_alpha": float(stats["one_sided_alpha"]),
            "bootstrap_resamples": int(stats["bootstrap_resamples"]),
            "bootstrap_seed": int(stats["bootstrap_seed"]),
            "positive_block_fraction": sum(
                value > 0.0 for value in primary_effects
            )
            / len(primary_effects),
        },
        "secondary_descriptive_statistics": {
            "comparison": "v3_analytic_vs_fixed_v080_shielded",
            "point_relative_reduction": secondary_point,
            "inferential_claim": False,
            "actual_teb_claim": False,
        },
        "formal_gates": formal_gates,
        "block_means_restricted_goal_time_s": block_means,
        "claim": {
            "in_scope_performance_improvement": positive,
            "actual_teb_superiority": False,
            "general_navigation_safety": False,
            "cross_family_generalization": False,
            "deployment_ready": False,
            "real_vehicle_ready": False,
        },
        "_evidence_resources": evidence_resources,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(V3_PATH, "v3_05_a0_assessor_v3")
    base_assessor = load(BASE_ASSESSOR_PATH, "v3_05_a0_base_assessor")
    runner = load(RUNNER_PATH, "v3_05_a0_runner_for_assessment")
    value = assess(v, base_assessor, runner)
    evidence_resources = value.pop("_evidence_resources")
    payload = v.yaml_bytes(value)
    output = W / v.ASSESSMENT_RELATIVE
    if args.check:
        v.require(
            output.is_file() and output.read_bytes() == payload,
            "assessment differs",
        )
    else:
        write_once(output, payload)
        resources = evidence_resources + [
            v.ASSESSMENT_RELATIVE,
            Path(
                v.load_yaml_relative(W, v.CONTRACT_RELATIVE)[
                    "upstream"
                ]["planner_contract_path"]
            ),
            Path(
                v.load_yaml_relative(W, v.CONTRACT_RELATIVE)[
                    "frozen_model"
                ]["path"]
            ),
        ]
        freeze = (
            "\n".join(
                "{}  {}".format(
                    v.sha256_bytes(v.read_file_once(W / item)),
                    item.as_posix(),
                )
                for item in resources
            )
            + "\n"
        ).encode("utf-8")
        write_once(W / v.EVIDENCE_FREEZE_RELATIVE, freeze)
    print(
        "status={} positive={} assessment_sha256={}".format(
            value["status"],
            value["performance_improvement_demonstrated"],
            v.sha256_bytes(payload),
        )
    )


if __name__ == "__main__":
    main()
