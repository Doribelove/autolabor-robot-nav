#!/usr/bin/env python3
"""Assess and replay V3-08-A1 integration evidence."""

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys

W = Path("/home/robot/robot_ws_base_rl")
MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1.py"
PACKAGE = W / "src/application/v3_scope_planner/src"
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))
from v3_scope_planner import spatial_core


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


def assess(v):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    preregistration = v.load_yaml_relative(W, v.PREREGISTRATION_RELATIVE)
    v.validate_contract(contract, preregistration)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    v.require(schedule == v.exact_schedule(contract), "schedule differs")
    planner = v.load_yaml_path(W / contract["upstream"]["planner_contract_path"])
    model = v.load_yaml_path(W / contract["frozen_model"]["path"])
    journal = [
        json.loads(line) for line in (W / v.JOURNAL_RELATIVE).read_text().splitlines()
        if line.strip()
    ]
    v.require(
        journal[-1]["event"] == "execution_complete"
        and len([x for x in journal if x["event"] == "unit_complete"]) == 18,
        "journal incomplete",
    )
    goals = selected = stop_available = exact = 0
    cautious_hesitant = cautious_hesitant_spatial = 0
    assertive_count = assertive_spatial = assertive_recovered = 0
    violations = 0
    minimum_clearance = float("inf")
    max_ego_jump = max_actor_jump = max_trace_error = 0.0
    latencies = []
    nonfinite = 0
    evaluator_leak = 0
    evidence = [v.JOURNAL_RELATIVE]
    spatial_id = planner["candidate_generator"]["spatial_candidate_id"]
    straight_id = planner["candidate_generator"]["cleared_straight_candidate_id"]
    for row in schedule["rows"]:
        root = v.EXECUTION_RELATIVE / "episodes" / row["unit_id"]
        receipt_path = root / "episode_receipt.yaml"
        receipt = v.load_yaml_relative(W, receipt_path)
        v.require(receipt["row"] == row and receipt["stage"] == "V3-08-A1",
                  "receipt differs")
        goals += receipt["goal_reached"] is True
        evaluator_leak += receipt["evaluator_policy_not_in_any_planner_snapshot"] is False
        segments = [receipt["prefix"]] + [x["segment"] for x in receipt["decisions"]]
        for segment in segments:
            clearance = float(segment["minimum_conservative_clearance_m"])
            minimum_clearance = min(minimum_clearance, clearance)
            violations += clearance < 0.0
            max_ego_jump = max(
                max_ego_jump, float(segment["boundary"]["ego_position_jump_m"])
            )
            max_actor_jump = max(
                max_actor_jump, float(segment["boundary"]["actor_position_jump_m"])
            )
            max_trace_error = max(
                max_trace_error, float(segment["maximum_ego_trace_error_m"])
            )
        for decision in receipt["decisions"]:
            snapshot = decision["snapshot_runtime_input_without_policy"]
            replay = spatial_core.plan(snapshot, model, planner)
            exact += (
                spatial_core.digest(replay) == decision["planner_result_digest"]
                and replay["decision"]["decision_digest"] == decision["decision_digest"]
            )
            selected += decision["selected_candidate_accepted"] is True
            stop_available += decision["certified_stop_available"] is True
            latencies.append(float(decision["planner_latency_ms"]))
            numeric = [
                decision["planner_latency_ms"],
                decision["selected_minimum_predicted_clearance_m"],
            ]
            nonfinite += sum(not math.isfinite(float(value)) for value in numeric)
        policy = row["actor_policy_evaluator_only"]
        first = receipt["decisions"][0]["selected_candidate_id"]
        if policy in ("cautious", "hesitant"):
            cautious_hesitant += 1
            cautious_hesitant_spatial += first == spatial_id
        else:
            assertive_count += 1
            assertive_spatial += sum(
                item["selected_candidate_id"] == spatial_id
                for item in receipt["decisions"]
            )
            assertive_recovered += (
                receipt["goal_reached"] is True
                and any(
                    item["selected_candidate_id"] == straight_id
                    and item["actor_cleared_certificate_active"] is True
                    for item in receipt["decisions"]
                )
            )
        evidence.append(receipt_path)
        evidence.extend(
            path.relative_to(W)
            for path in sorted((W / root).glob("**/*"))
            if path.is_file() and path.name in ("raw_pose_trace.csv", "world.sdf")
        )
    ordered = sorted(latencies)
    p99 = ordered[min(len(ordered) - 1, int(math.ceil(0.99 * len(ordered))) - 1)]
    decision_count = len(latencies)
    metrics = {
        "episode_count": 18,
        "goal_success_rate": goals / 18.0,
        "selected_candidate_accepted_rate": selected / decision_count,
        "certified_stop_available_rate": stop_available / decision_count,
        "exact_core_replay_rate": exact / decision_count,
        "cautious_hesitant_first_spatial_rate": (
            cautious_hesitant_spatial / cautious_hesitant
        ),
        "assertive_spatial_selection_count": assertive_spatial,
        "assertive_cleared_straight_recovery_rate": (
            assertive_recovered / assertive_count
        ),
        "minimum_conservative_clearance_m": minimum_clearance,
        "conservative_clearance_violation_count": violations,
        "maximum_segment_boundary_ego_position_jump_m": max_ego_jump,
        "maximum_segment_boundary_actor_position_jump_m": max_actor_jump,
        "maximum_selected_trajectory_trace_error_m": max_trace_error,
        "planner_latency_p99_ms": p99,
        "nonfinite_metric_count": nonfinite,
        "evaluator_policy_leak_count": evaluator_leak,
        "decision_count": decision_count,
    }
    rules = contract["integration_gates"]
    gates = {
        "episode_count_pass": metrics["episode_count"] == rules["episode_count"],
        "goal_pass": metrics["goal_success_rate"] >= rules["goal_success_rate_min"],
        "selected_pass": metrics["selected_candidate_accepted_rate"]
        >= rules["selected_candidate_accepted_rate_min"],
        "stop_pass": metrics["certified_stop_available_rate"]
        >= rules["certified_stop_available_rate_min"],
        "replay_pass": metrics["exact_core_replay_rate"]
        >= rules["exact_core_replay_rate_min"],
        "spatial_activation_pass": metrics["cautious_hesitant_first_spatial_rate"]
        >= rules["cautious_hesitant_first_spatial_rate_min"],
        "assertive_fail_closed_pass": metrics["assertive_spatial_selection_count"]
        <= rules["assertive_spatial_selection_count_max"],
        "assertive_recovery_pass": metrics[
            "assertive_cleared_straight_recovery_rate"
        ] >= rules["assertive_cleared_straight_recovery_rate_min"],
        "clearance_pass": violations
        <= rules["conservative_clearance_violation_count_max"]
        and minimum_clearance >= rules["minimum_conservative_clearance_m"],
        "boundary_pass": max_ego_jump
        <= rules["maximum_segment_boundary_ego_position_jump_m"]
        and max_actor_jump
        <= rules["maximum_segment_boundary_actor_position_jump_m"],
        "trace_pass": max_trace_error
        <= rules["maximum_selected_trajectory_trace_error_m"],
        "latency_pass": p99 <= rules["planner_latency_p99_max_ms"],
        "finite_pass": nonfinite <= rules["nonfinite_metric_count_max"],
        "no_hidden_policy_leak_pass": evaluator_leak == 0,
    }
    passed = all(gates.values())
    return {
        "schema_version": "1.0", "architecture_generation": "v3",
        "stage": "V3-08-A1",
        "status": (
            "spatial_planner_simulation_integration_pass"
            if passed else "spatial_planner_simulation_integration_fail"
        ),
        "formal_result": False,
        "execution": {
            "authorized_units": 18, "consumed_units": 18,
            "completed_units": 18, "retry_units": 0,
        },
        "metrics": metrics, "gates": gates, "overall_pass": passed,
        "qualified_for_fresh_performance_evaluation": passed,
        "performance_claim_authorized": False, "runtime_ready": False,
        "real_vehicle_authorized": False, "_evidence": evidence,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(MODULE, "v3_08_a1_assessment_module")
    value = assess(v)
    evidence = value.pop("_evidence")
    assessment_bytes = v.yaml_bytes(value)
    if args.check:
        v.require(
            (W / v.ASSESSMENT_RELATIVE).read_bytes() == assessment_bytes,
            "assessment differs",
        )
    else:
        write_once(W / v.ASSESSMENT_RELATIVE, assessment_bytes)
        freeze = (
            "\n".join(
                "{}  {}".format(v.sha256_bytes(v.read_file_once(W / item)), item)
                for item in evidence + [v.ASSESSMENT_RELATIVE]
            ) + "\n"
        ).encode()
        write_once(W / v.EVIDENCE_FREEZE_RELATIVE, freeze)
    print(
        "status={} qualified={} assessment_sha256={}".format(
            value["status"], value["qualified_for_fresh_performance_evaluation"],
            hashlib.sha256(assessment_bytes).hexdigest(),
        )
    )


if __name__ == "__main__":
    main()
