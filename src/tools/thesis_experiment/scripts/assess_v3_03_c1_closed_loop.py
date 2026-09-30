#!/usr/bin/env python3
"""Replay and assess the V3-03-C1 segmented closed-loop integration."""

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c1.py"
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


def percentile(values, probability):
    ordered = sorted(values)
    index = max(0, int(math.ceil(probability * len(ordered))) - 1)
    return ordered[index]


def assess(v, runner):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    prereg = v.load_yaml_relative(W, v.PREREGISTRATION_RELATIVE)
    v.validate_contract(contract, prereg)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    v.require(schedule == v.exact_schedule(contract), "schedule differs")
    planner_path = W / contract["upstream"]["planner_contract_path"]
    planner_bytes = v.read_file_once(planner_path)
    v.require(
        v.sha256_bytes(planner_bytes)
        == contract["upstream"]["planner_contract_sha256"],
        "planner contract changed",
    )
    planner = v.load_yaml_path(planner_path)
    model_path = W / contract["frozen_model"]["path"]
    model_bytes = v.read_file_once(model_path)
    v.require(
        v.sha256_bytes(model_bytes) == contract["frozen_model"]["sha256"],
        "model changed",
    )
    model = v.load_yaml_path(model_path)
    journal = [
        json.loads(line)
        for line in (W / v.JOURNAL_RELATIVE)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.strip()
    ]
    v.require(
        len([row for row in journal if row["event"] == "unit_complete"])
        == int(schedule["row_count"])
        and journal[-1]["event"] == "execution_complete",
        "execution journal incomplete",
    )
    combined_radius = core._combined_radius(planner)
    latencies = []
    selected_total = 0
    selected_accepted = 0
    stop_available = 0
    exact_replay = 0
    conservative_clearance_violations = 0
    minimum_clearance = float("inf")
    maximum_ego_jump = 0.0
    maximum_actor_jump = 0.0
    maximum_trace_error = 0.0
    outcomes = []
    evidence_resources = [v.JOURNAL_RELATIVE]
    for row in schedule["rows"]:
        unit_root = v.EXECUTION_RELATIVE / "episodes" / row["unit_id"]
        receipt_path = unit_root / "episode_receipt.yaml"
        episode = v.load_yaml_relative(W, receipt_path)
        v.require(episode["row"] == row, "episode row differs")
        v.require(
            episode["evaluator_policy_not_in_any_planner_snapshot"] is True,
            "policy leaked into planner snapshot",
        )
        evidence_resources.append(receipt_path)
        prefix_raw_path = unit_root / "prefix/raw_pose_trace.csv"
        prefix_world_path = unit_root / "prefix/world.sdf"
        prefix_raw = v.read_file_once(W / prefix_raw_path)
        v.require(
            v.sha256_bytes(prefix_raw)
            == episode["prefix"]["raw_csv_sha256"],
            "prefix raw hash differs",
        )
        prefix_trace = v.load_trace(W / prefix_raw_path)
        prefix_clearance = runner.clearance(prefix_trace, combined_radius)
        v.require(
            abs(
                prefix_clearance
                - float(
                    episode["prefix"]["minimum_conservative_clearance_m"]
                )
            )
            <= 1e-12,
            "prefix clearance replay differs",
        )
        minimum_clearance = min(minimum_clearance, prefix_clearance)
        conservative_clearance_violations += prefix_clearance < 0.0
        maximum_ego_jump = max(
            maximum_ego_jump,
            float(episode["prefix"]["boundary"]["ego_position_jump_m"]),
        )
        maximum_actor_jump = max(
            maximum_actor_jump,
            float(episode["prefix"]["boundary"]["actor_position_jump_m"]),
        )
        maximum_trace_error = max(
            maximum_trace_error,
            float(episode["prefix"]["maximum_ego_trace_error_m"]),
        )
        evidence_resources.extend((prefix_raw_path, prefix_world_path))
        recovered = False
        stop_before_clear = False
        for decision in episode["decisions"]:
            snapshot = decision["snapshot_runtime_input_without_policy"]
            v.require(
                "policy" not in json.dumps(snapshot, sort_keys=True),
                "policy leaked into replay input",
            )
            result = runner.apply_selection_method(
                core.plan(snapshot, model, planner), row, planner
            )
            replay_ok = (
                core.digest(result) == decision["planner_result_digest"]
                and result["candidate_set"]["candidate_set_digest"]
                == decision["candidate_set_digest"]
                and result["decision"]["decision_digest"]
                == decision["decision_digest"]
                and result["decision"]["selected_candidate_id"]
                == decision["selected_candidate_id"]
            )
            exact_replay += replay_ok
            selected_total += 1
            selected_eval = next(
                item
                for item in result["evaluations"]
                if item["candidate_id"] == decision["selected_candidate_id"]
            )
            selected_accepted += (
                decision["selected_candidate_accepted"] is True
                and selected_eval["accepted"] is True
            )
            stop_available += decision["certified_stop_available"] is True
            latencies.append(float(decision["planner_latency_ms"]))
            cleared = core.actor_cleared(snapshot, planner)
            moving = (
                decision["selected_candidate_id"] != "certified_stop_v000"
            )
            recovered = recovered or (cleared and moving)
            stop_before_clear = stop_before_clear or (
                not cleared and not moving
            )
            index = int(decision["decision_index"])
            segment_root = unit_root / "decision_{:02d}".format(index)
            raw_path = segment_root / "raw_pose_trace.csv"
            world_path = segment_root / "world.sdf"
            raw = v.read_file_once(W / raw_path)
            v.require(
                v.sha256_bytes(raw)
                == decision["segment"]["raw_csv_sha256"],
                "decision raw hash differs",
            )
            trace = v.load_trace(W / raw_path)
            segment_clearance = runner.clearance(trace, combined_radius)
            v.require(
                abs(
                    segment_clearance
                    - float(
                        decision["segment"][
                            "minimum_conservative_clearance_m"
                        ]
                    )
                )
                <= 1e-12,
                "decision clearance replay differs",
            )
            minimum_clearance = min(minimum_clearance, segment_clearance)
            conservative_clearance_violations += segment_clearance < 0.0
            maximum_ego_jump = max(
                maximum_ego_jump,
                float(
                    decision["segment"]["boundary"]["ego_position_jump_m"]
                ),
            )
            maximum_actor_jump = max(
                maximum_actor_jump,
                float(
                    decision["segment"]["boundary"]["actor_position_jump_m"]
                ),
            )
            maximum_trace_error = max(
                maximum_trace_error,
                float(
                    decision["segment"]["maximum_ego_trace_error_m"]
                ),
            )
            evidence_resources.extend((raw_path, world_path))
        outcomes.append(
            {
                "unit_id": row["unit_id"],
                "seed": row["seed"],
                "actor_policy_evaluator_only": row[
                    "actor_policy_evaluator_only"
                ],
                "goal_reached": episode["goal_reached"],
                "goal_time_s": episode["goal_time_s"],
                "decision_count": episode["decision_count"],
                "yielded_before_clear": stop_before_clear,
                "motion_recovered_after_clear": recovered,
                "final_ego_x_m": episode["final_state"]["ego_x_m"],
                "final_actor_coordinate_m": episode["final_state"][
                    "actor_coordinate_m"
                ],
                "episode_receipt_sha256": v.sha256_bytes(
                    v.read_file_once(W / receipt_path)
                ),
            }
        )
    episode_count = len(outcomes)
    metrics = {
        "episode_count": episode_count,
        "decision_count": selected_total,
        "goal_success_rate": sum(
            row["goal_reached"] for row in outcomes
        )
        / episode_count,
        "selected_candidate_accepted_rate": selected_accepted
        / selected_total,
        "certified_stop_available_rate": stop_available / selected_total,
        "exact_core_replay_rate": exact_replay / selected_total,
        "motion_recovery_after_clear_rate": sum(
            row["motion_recovered_after_clear"] for row in outcomes
        )
        / episode_count,
        "yield_before_clear_rate": sum(
            row["yielded_before_clear"] for row in outcomes
        )
        / episode_count,
        "conservative_clearance_violation_count": (
            conservative_clearance_violations
        ),
        "minimum_conservative_clearance_m": minimum_clearance,
        "maximum_segment_boundary_ego_position_jump_m": maximum_ego_jump,
        "maximum_segment_boundary_actor_position_jump_m": maximum_actor_jump,
        "maximum_selected_trajectory_trace_error_m": maximum_trace_error,
        "planner_latency_p99_ms": percentile(latencies, 0.99),
        "planner_latency_max_ms": max(latencies),
        "maximum_goal_time_s": max(
            float(row["goal_time_s"])
            for row in outcomes
            if row["goal_reached"]
        ),
        "nonfinite_metric_count": 0,
    }
    metrics["nonfinite_metric_count"] = sum(
        not math.isfinite(float(value))
        for key, value in metrics.items()
        if key != "nonfinite_metric_count"
    )
    thresholds = contract["integration_gates"]
    gates = {
        "episode_count_pass": episode_count
        == int(thresholds["episode_count"]),
        "goal_success_pass": metrics["goal_success_rate"]
        >= float(thresholds["goal_success_rate_min"]),
        "selected_candidate_accepted_pass": metrics[
            "selected_candidate_accepted_rate"
        ]
        >= float(thresholds["selected_candidate_accepted_rate_min"]),
        "certified_stop_available_pass": metrics[
            "certified_stop_available_rate"
        ]
        >= float(thresholds["certified_stop_available_rate_min"]),
        "exact_core_replay_pass": metrics["exact_core_replay_rate"]
        >= float(thresholds["exact_core_replay_rate_min"]),
        "motion_recovery_after_clear_pass": metrics[
            "motion_recovery_after_clear_rate"
        ]
        >= float(thresholds["motion_recovery_after_clear_rate_min"]),
        "conservative_clearance_pass": metrics[
            "conservative_clearance_violation_count"
        ]
        <= int(thresholds["conservative_clearance_violation_count_max"])
        and metrics["minimum_conservative_clearance_m"]
        >= float(thresholds["minimum_conservative_clearance_m"]),
        "ego_boundary_continuity_pass": metrics[
            "maximum_segment_boundary_ego_position_jump_m"
        ]
        <= float(
            thresholds["maximum_segment_boundary_ego_position_jump_m"]
        ),
        "actor_boundary_continuity_pass": metrics[
            "maximum_segment_boundary_actor_position_jump_m"
        ]
        <= float(
            thresholds["maximum_segment_boundary_actor_position_jump_m"]
        ),
        "trajectory_trace_pass": metrics[
            "maximum_selected_trajectory_trace_error_m"
        ]
        <= float(thresholds["maximum_selected_trajectory_trace_error_m"]),
        "planner_latency_pass": metrics["planner_latency_p99_ms"]
        <= float(thresholds["planner_latency_p99_max_ms"]),
        "finite_metrics_pass": metrics["nonfinite_metric_count"]
        <= int(thresholds["nonfinite_metric_count_max"]),
    }
    overall = all(gates.values())
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": contract["stage"],
        "assessment_id": "{}_closed_loop_assessment_1".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "status": (
            "{}_segmented_closed_loop_integration_pass".format(
                contract["stage"].lower().replace("-", "_")
            )
            if overall
            else "{}_segmented_closed_loop_integration_fail".format(
                contract["stage"].lower().replace("-", "_")
            )
        ),
        "formal_result": False,
        "execution": {
            "authorized_units": schedule["row_count"],
            "consumed_units": schedule["row_count"],
            "completed_units": schedule["row_count"],
            "retry_units": 0,
        },
        "metrics": metrics,
        "gates": gates,
        "overall_pass": overall,
        "outcomes": outcomes,
        "claim": {
            "fresh_segmented_closed_loop_integration_pass": overall,
            "continuous_runtime_validation": False,
            "general_navigation_safety": False,
            "performance_improvement": False,
            "real_vehicle_ready": False,
        },
        "_evidence_resources": evidence_resources,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load(V3_PATH, "v3_03_c1_assessor")
    runner = load(RUNNER_PATH, "v3_03_c1_runner_for_assessment")
    value = assess(v, runner)
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
            Path(v.load_yaml_relative(W, v.CONTRACT_RELATIVE)[
                "upstream"
            ]["planner_contract_path"]),
            Path(v.load_yaml_relative(W, v.CONTRACT_RELATIVE)[
                "frozen_model"
            ]["path"]),
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
        "status={} overall_pass={} assessment_sha256={}".format(
            value["status"], value["overall_pass"], v.sha256_bytes(payload)
        )
    )


if __name__ == "__main__":
    main()
