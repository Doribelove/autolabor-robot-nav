#!/usr/bin/env python3
"""Assess V3-06-C3 fresh minimal-v040 qualification."""

import argparse
import hashlib
import importlib.util
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_c3.py"
BASE_PATH = W / "src/tools/thesis_experiment/scripts/assess_v3_03_c1_closed_loop.py"
RUNNER_PATH = W / "src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"


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


def assess(v, base, runner):
    runner.apply_selection_method = v.apply_selection_method
    pooled = base.assess(v, runner)
    evidence = pooled.pop("_evidence_resources")
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    outcomes = {row["unit_id"]: row for row in pooled["outcomes"]}
    methods = schedule["methods"]
    accumulators = {
        method: {
            "times": [], "success": 0, "decisions": 0,
            "collision": 0, "stop": 0,
        }
        for method in methods
    }
    pairs = {}
    for row in schedule["rows"]:
        method = row["method"]
        outcome = outcomes[row["unit_id"]]
        episode = v.load_yaml_relative(
            W,
            v.EXECUTION_RELATIVE / "episodes" / row["unit_id"]
            / "episode_receipt.yaml",
        )
        restricted = (
            float(outcome["goal_time_s"])
            if outcome["goal_reached"] else 10.0
        )
        pair = "{}:{}".format(
            row["block_id"], row["actor_policy_evaluator_only"]
        )
        pairs.setdefault(pair, {})[method] = restricted
        values = accumulators[method]
        values["times"].append(restricted)
        values["success"] += outcome["goal_reached"]
        values["decisions"] += outcome["decision_count"]
        values["stop"] += sum(
            decision["selected_candidate_id"] == "certified_stop_v000"
            for decision in episode["decisions"]
        )
        clearances = [
            float(episode["prefix"]["minimum_conservative_clearance_m"])
        ] + [
            float(decision["segment"]["minimum_conservative_clearance_m"])
            for decision in episode["decisions"]
        ]
        values["collision"] += min(clearances) < 0.0
    method_metrics = {}
    for method, values in accumulators.items():
        count = len(values["times"])
        method_metrics[method] = {
            "episode_count": count,
            "goal_success_rate": values["success"] / count,
            "restricted_goal_time_mean_s": sum(values["times"]) / count,
            "decision_count": values["decisions"],
            "certified_stop_decision_count": values["stop"],
            "collision_episode_count": values["collision"],
        }
    effects = [
        (values["sparse_control"] - values["minimal_v040"])
        / values["sparse_control"]
        for values in pairs.values()
    ]
    point = sum(effects) / len(effects)
    positive = sum(value > 0.0 for value in effects) / len(effects)
    rules = contract["qualification_gates"]
    treatment = method_metrics["minimal_v040"]
    control = method_metrics["sparse_control"]
    gates = {
        "shared_integration_pass": pooled["overall_pass"] is True,
        "point_effect_pass": point
        >= float(rules["minimum_relative_reduction_vs_control"]),
        "treatment_goal_success_pass": treatment["goal_success_rate"]
        >= float(rules["treatment_goal_success_rate_min"]),
        "goal_success_noninferiority_pass": treatment["goal_success_rate"]
        >= control["goal_success_rate"],
        "positive_pair_fraction_pass": positive
        >= float(rules["positive_pair_fraction_min"]),
        "collision_pass": treatment["collision_episode_count"]
        <= int(rules["collision_count_max"]),
        "selected_candidate_accepted_pass": pooled["metrics"][
            "selected_candidate_accepted_rate"
        ]
        >= float(rules["selected_candidate_accepted_rate_min"]),
        "exact_replay_pass": pooled["metrics"]["exact_core_replay_rate"]
        >= float(rules["exact_replay_rate_min"]),
        "latency_pass": pooled["metrics"]["planner_latency_p99_ms"]
        <= float(rules["planner_latency_p99_max_ms"]),
    }
    qualified = all(gates.values())
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-C3",
        "assessment_id": "v3_06_c3_qualification_assessment_1",
        "status": (
            "v3_06_c3_minimal_v040_qualification_pass"
            if qualified else "v3_06_c3_minimal_v040_qualification_fail"
        ),
        "formal_result": False,
        "execution": pooled["execution"],
        "pooled_integration_metrics": pooled["metrics"],
        "pooled_integration_gates": pooled["gates"],
        "method_metrics": method_metrics,
        "paired_effect": {
            "paired_unit_count": len(effects),
            "point_relative_reduction_vs_control": point,
            "positive_pair_fraction": positive,
        },
        "qualification_gates": gates,
        "qualified_for_fresh_formal_evaluation": qualified,
        "overall_pass": qualified,
        "claim": {
            "formal_performance_improvement": False,
            "actual_teb_superiority": False,
            "general_navigation_safety": False,
            "deployment_ready": False,
            "real_vehicle_ready": False,
        },
        "_evidence_resources": evidence,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(V3_PATH, "v3_06_c3_assessor_v")
    base = load(BASE_PATH, "v3_06_c3_base")
    runner = load(RUNNER_PATH, "v3_06_c3_runner")
    value = assess(v, base, runner)
    evidence = value.pop("_evidence_resources")
    payload = v.yaml_bytes(value)
    output = W / v.ASSESSMENT_RELATIVE
    if args.check:
        v.require(
            output.is_file() and output.read_bytes() == payload,
            "assessment differs",
        )
    else:
        write_once(output, payload)
        resources = evidence + [v.ASSESSMENT_RELATIVE]
        freeze = (
            "\n".join(
                "{}  {}".format(
                    v.sha256_bytes(v.read_file_once(W / item)), item.as_posix()
                )
                for item in resources
            )
            + "\n"
        ).encode("utf-8")
        write_once(W / v.EVIDENCE_FREEZE_RELATIVE, freeze)
    print(
        "status={} qualified={} assessment_sha256={}".format(
            value["status"],
            value["qualified_for_fresh_formal_evaluation"],
            hashlib.sha256(payload).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
