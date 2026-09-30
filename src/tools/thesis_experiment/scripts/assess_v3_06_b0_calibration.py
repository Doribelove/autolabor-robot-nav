#!/usr/bin/env python3
"""Assess the V3-06-B0 fresh ranker calibration."""

import argparse
import hashlib
import importlib.util
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = (
    W / "src/tools/thesis_experiment/src/thesis_experiment/v3_06_b0.py"
)
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


def assess(v, base_assessor, runner):
    runner.apply_selection_method = v.apply_selection_method
    pooled = base_assessor.assess(v, runner)
    evidence_resources = pooled.pop("_evidence_resources")
    v.require(
        pooled["execution"]["authorized_units"] == 48
        and pooled["execution"]["consumed_units"] == 48
        and pooled["execution"]["completed_units"] == 48
        and pooled["overall_pass"] is True,
        "pooled calibration replay failed",
    )
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    planner = v.load_yaml_path(
        W / contract["upstream"]["planner_contract_path"]
    )
    model = v.load_yaml_path(W / contract["frozen_model"]["path"])
    methods = schedule["methods"]
    accumulators = {
        method: {
            "episode_count": 0,
            "goal_success_count": 0,
            "restricted_times": [],
            "decision_count": 0,
            "selected_accepted": 0,
            "exact_replay": 0,
            "collision_count": 0,
            "selected_counts": {},
        }
        for method in methods
    }
    paired_times = {}
    for row in schedule["rows"]:
        method = row["method"]
        values = accumulators[method]
        episode = v.load_yaml_relative(
            W,
            v.EXECUTION_RELATIVE
            / "episodes"
            / row["unit_id"]
            / "episode_receipt.yaml",
        )
        goal = episode["goal_reached"] is True
        restricted = (
            float(episode["goal_time_s"]) if goal else 10.0
        )
        pair_key = "{}:{}".format(
            row["block_id"], row["actor_policy_evaluator_only"]
        )
        paired_times.setdefault(pair_key, {})[method] = restricted
        values["episode_count"] += 1
        values["goal_success_count"] += goal
        values["restricted_times"].append(restricted)
        clearances = [
            float(episode["prefix"]["minimum_conservative_clearance_m"])
        ] + [
            float(
                decision["segment"][
                    "minimum_conservative_clearance_m"
                ]
            )
            for decision in episode["decisions"]
        ]
        values["collision_count"] += min(clearances) < 0.0
        for decision in episode["decisions"]:
            result = v.apply_selection_method(
                core.plan(
                    decision["snapshot_runtime_input_without_policy"],
                    model,
                    planner,
                ),
                row,
                planner,
            )
            selected_id = result["decision"]["selected_candidate_id"]
            selected = next(
                item
                for item in result["evaluations"]
                if item["candidate_id"] == selected_id
            )
            exact = (
                core.digest(result) == decision["planner_result_digest"]
                and result["decision"]["decision_digest"]
                == decision["decision_digest"]
                and selected_id == decision["selected_candidate_id"]
            )
            values["decision_count"] += 1
            values["selected_accepted"] += selected["accepted"] is True
            values["exact_replay"] += exact
            values["selected_counts"][selected_id] = (
                values["selected_counts"].get(selected_id, 0) + 1
            )

    method_metrics = {}
    for method, values in accumulators.items():
        count = values["episode_count"]
        decisions = values["decision_count"]
        method_metrics[method] = {
            "clearance_weight": v.METHOD_WEIGHTS[method],
            "episode_count": count,
            "goal_success_rate": values["goal_success_count"] / count,
            "restricted_goal_time_mean_s": (
                sum(values["restricted_times"]) / count
            ),
            "decision_count": decisions,
            "selected_candidate_accepted_rate": (
                values["selected_accepted"] / decisions
            ),
            "exact_replay_rate": values["exact_replay"] / decisions,
            "collision_episode_count": values["collision_count"],
            "selected_candidate_counts": dict(
                sorted(values["selected_counts"].items())
            ),
        }
    control = method_metrics["v3_greedy_control"]
    effects = {}
    eligible = []
    rules = contract["development_selection"]
    for method in methods:
        if method == "v3_greedy_control":
            continue
        paired_effects = [
            (
                values["v3_greedy_control"] - values[method]
            )
            / values["v3_greedy_control"]
            for values in paired_times.values()
        ]
        point = sum(paired_effects) / len(paired_effects)
        candidate = method_metrics[method]
        gates = {
            "minimum_relative_reduction_pass": point
            >= float(rules["minimum_relative_reduction_vs_control"]),
            "goal_success_noninferiority_pass": candidate[
                "goal_success_rate"
            ]
            >= control["goal_success_rate"],
            "collision_gate_pass": candidate["collision_episode_count"]
            <= int(rules["collision_count_max"]),
            "selected_candidate_accepted_pass": candidate[
                "selected_candidate_accepted_rate"
            ]
            >= float(rules["selected_candidate_accepted_rate_min"]),
            "exact_replay_pass": candidate["exact_replay_rate"]
            >= float(rules["exact_replay_rate_min"]),
        }
        effects[method] = {
            "paired_unit_count": len(paired_effects),
            "point_relative_reduction_vs_control": point,
            "positive_pair_fraction": (
                sum(value > 0.0 for value in paired_effects)
                / len(paired_effects)
            ),
            "development_gates": gates,
            "eligible": all(gates.values()),
        }
        if all(gates.values()):
            eligible.append(method)
    eligible.sort(
        key=lambda method: (
            -effects[method]["point_relative_reduction_vs_control"],
            v.METHOD_WEIGHTS[method],
        )
    )
    winner = eligible[0] if eligible else None
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-06-B0",
        "assessment_id": "v3_06_b0_calibration_assessment_1",
        "status": (
            "v3_06_b0_development_factor_selected"
            if winner
            else "v3_06_b0_development_no_factor_selected"
        ),
        "formal_result": False,
        "execution": pooled["execution"],
        "pooled_integration_metrics": pooled["metrics"],
        "method_metrics": method_metrics,
        "candidate_effects": effects,
        "selection": {
            "selected_method": winner or "",
            "selected_clearance_weight": (
                v.METHOD_WEIGHTS[winner] if winner else None
            ),
            "eligible_methods": eligible,
            "selection_is_development_only": True,
            "fresh_qualification_required": bool(winner),
        },
        "overall_pass": True,
        "claim": {
            "development_calibration_complete": True,
            "formal_performance_improvement": False,
            "actual_teb_superiority": False,
            "general_navigation_safety": False,
            "deployment_ready": False,
            "real_vehicle_ready": False,
        },
        "_evidence_resources": evidence_resources,
    }
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(V3_PATH, "v3_06_b0_assessor_v3")
    base = load(BASE_ASSESSOR_PATH, "v3_06_b0_base_assessor")
    runner = load(RUNNER_PATH, "v3_06_b0_assessor_runner")
    value = assess(v, base, runner)
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
        "status={} selected={} assessment_sha256={}".format(
            value["status"],
            value["selection"]["selected_method"] or "none",
            hashlib.sha256(payload).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
