#!/usr/bin/env python3
"""Run the bounded V3-08-A1 segmented spatial planner integration."""

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys
import time

W = Path("/home/robot/robot_ws_base_rl")
MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_08_a1.py"
BASE = W / "src/tools/thesis_experiment/scripts/run_v3_03_c1_closed_loop.py"
PACKAGE = W / "src/application/v3_scope_planner/src"
if str(PACKAGE) not in sys.path:
    sys.path.insert(0, str(PACKAGE))
from v3_scope_planner import spatial_core


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", required=True)
    parser.add_argument("--release-sha", required=True)
    parser.add_argument("--authorization-sha", required=True)
    args = parser.parse_args()
    v = load(MODULE, "v3_08_a1_runner_module")
    driver = load(BASE, "v3_08_a1_segment_driver")
    driver.verify(v, args.release_sha, args.authorization_sha)
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    preregistration = v.load_yaml_relative(W, v.PREREGISTRATION_RELATIVE)
    v.validate_contract(contract, preregistration)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    v.require(schedule == v.exact_schedule(contract), "schedule differs")
    closure = v.load_yaml_relative(W, v.CLOSURE_RELATIVE)
    v.check_receipts(W, closure["source_resources"])
    v.check_receipts(W, closure["binary_resources"])
    bindings = v.load_yaml_relative(W, v.BINDINGS_RELATIVE)
    planner_path = W / contract["upstream"]["planner_contract_path"]
    planner_bytes = v.read_file_once(planner_path)
    v.require(
        v.sha256_bytes(planner_bytes)
        == contract["upstream"]["planner_contract_sha256"],
        "planner differs",
    )
    planner = v.load_yaml_path(planner_path)
    model_path = W / contract["frozen_model"]["path"]
    model_bytes = v.read_file_once(model_path)
    v.require(
        v.sha256_bytes(model_bytes) == contract["frozen_model"]["sha256"],
        "model differs",
    )
    model = v.load_yaml_path(model_path)
    execution = W / v.EXECUTION_RELATIVE
    v.require(not execution.exists(), "execution root exists; resume forbidden")
    execution.mkdir(parents=True)
    journal = W / v.JOURNAL_RELATIVE
    journal.touch(exist_ok=False)
    driver.append(journal, {
        "event": "execution_opened",
        "release_sha256": args.release_sha,
        "authorization_sha256": args.authorization_sha,
        "authorized_units": 18, "consumed_units": 0,
    })
    collision_radius = float(
        planner["hard_verifier"]["conservative_combined_radius_m"]
    )
    prefix_duration = float(contract["closed_loop"]["observation_prefix_s"])
    period = float(contract["closed_loop"]["replanning_period_s"])
    spatial_duration = float(
        contract["closed_loop"]["spatial_commitment_duration_s"]
    )
    max_duration = float(contract["closed_loop"]["maximum_episode_duration_s"])
    max_decisions = int(contract["closed_loop"]["maximum_decisions_per_episode"])
    goal_x = float(contract["closed_loop"]["goal_x_m"])
    completed = 0
    for row in schedule["rows"]:
        unit_root = execution / "episodes" / row["unit_id"]
        unit_root.mkdir(parents=True, exist_ok=False)
        driver.append(journal, {
            "event": "unit_consumed", "sequence": row["sequence"],
            "unit_id": row["unit_id"], "attempt": 1,
        })
        state = {
            "global_time_s": 0.0,
            "ego_x_m": float(contract["scene"]["ego"]["initial_pose_m_rad"][0]),
            "ego_y_m": float(contract["scene"]["ego"]["initial_pose_m_rad"][1]),
            "ego_yaw_rad": float(contract["scene"]["ego"]["initial_pose_m_rad"][2]),
            "ego_speed_mps": float(contract["scene"]["ego"]["initial_speed_mps"]),
            "actor_coordinate_m": -float(row["actor_start_distance_m"]),
            "actor_speed_mps": float(row["actor_base_speed_mps"]),
        }
        prefix_waypoints = [
            [0.0, [state["ego_x_m"], state["ego_y_m"], state["ego_yaw_rad"]]],
            [prefix_duration, [
                state["ego_x_m"] + state["ego_speed_mps"] * prefix_duration,
                state["ego_y_m"], state["ego_yaw_rad"],
            ]],
        ]
        try:
            prefix_receipt, prefix_trace = driver.run_segment(
                v, contract, bindings, row, state, prefix_waypoints,
                prefix_duration, unit_root / "prefix",
                driver.gazebo_master_port(contract, row["sequence"]),
            )
            prefix_receipt["boundary"] = driver.boundary_jump(
                v, prefix_trace, row, state
            )
            prefix_receipt["minimum_conservative_clearance_m"] = driver.clearance(
                prefix_trace, collision_radius
            )
            prefix_receipt["maximum_ego_trace_error_m"] = driver.trace_error(
                v, prefix_trace, prefix_waypoints
            )
            state = driver.endpoint_state(
                v, prefix_trace, row, state, prefix_duration,
                contract["scene"]["ego"]["initial_speed_mps"],
            )
            decisions = []
            goal_reached = state["ego_x_m"] >= goal_x
            for decision_index in range(1, max_decisions + 1):
                if goal_reached or state["global_time_s"] >= max_duration:
                    break
                snapshot = {
                    "scene_snapshot_id": "{}_d{:02d}".format(
                        row["unit_id"], decision_index
                    ),
                    "world_model_seq": decision_index,
                    "snapshot_age_s": 0.0,
                    "predictor_latency_ms": 0.0,
                    "verifier_latency_ms": 0.0,
                    "ego": {
                        "x_m": state["ego_x_m"], "y_m": state["ego_y_m"],
                        "yaw_rad": state["ego_yaw_rad"],
                        "speed_mps": state["ego_speed_mps"],
                    },
                    "actor": {
                        "coordinate_m": state["actor_coordinate_m"],
                        "speed_mps": state["actor_speed_mps"],
                        "path_angle_deg": float(row["angle_deg"]),
                        "monotone_coordinate_certified": True,
                    },
                    "health": {
                        "localization_ok": True, "transform_ok": True,
                        "actor_track_ok": True, "command_chain_ok": True,
                    },
                }
                started = time.perf_counter_ns()
                result = spatial_core.plan(snapshot, model, planner)
                latency_ms = (time.perf_counter_ns() - started) / 1e6
                selected_id = result["decision"]["selected_candidate_id"]
                selected = next(
                    item for item in result["candidate_set"]["candidates"]
                    if item["candidate_id"] == selected_id
                )
                selected_eval = next(
                    item for item in result["evaluations"]
                    if item["candidate_id"] == selected_id
                )
                v.require(selected_eval["accepted"] is True,
                          "selected candidate rejected")
                duration = (
                    spatial_duration
                    if selected_id
                    == planner["candidate_generator"]["spatial_candidate_id"]
                    else period
                )
                duration = min(duration, max_duration - state["global_time_s"])
                waypoints = driver.candidate_waypoints(selected, duration)
                planned_endpoint = next(
                    point for point in selected["points"]
                    if abs(float(point["time_from_start_s"]) - duration) <= 1e-12
                )
                segment_root = unit_root / "decision_{:02d}".format(decision_index)
                segment_receipt, trace = driver.run_segment(
                    v, contract, bindings, row, state, waypoints, duration,
                    segment_root,
                    driver.gazebo_master_port(
                        contract, row["sequence"], decision_index
                    ),
                )
                segment_receipt["boundary"] = driver.boundary_jump(
                    v, trace, row, state
                )
                segment_receipt["minimum_conservative_clearance_m"] = (
                    driver.clearance(trace, collision_radius)
                )
                segment_receipt["maximum_ego_trace_error_m"] = driver.trace_error(
                    v, trace, waypoints
                )
                new_state = driver.endpoint_state(
                    v, trace, row, state, duration,
                    planned_endpoint["longitudinal_velocity_mps"],
                )
                decisions.append({
                    "decision_index": decision_index,
                    "snapshot_runtime_input_without_policy": snapshot,
                    "planner_latency_ms": latency_ms,
                    "planner_result_digest": spatial_core.digest(result),
                    "candidate_set_digest": result["candidate_set"][
                        "candidate_set_digest"
                    ],
                    "decision_digest": result["decision"]["decision_digest"],
                    "selected_candidate_id": selected_id,
                    "selected_candidate_digest": selected["candidate_digest"],
                    "selected_candidate_accepted": selected_eval["accepted"],
                    "selected_minimum_predicted_clearance_m": selected_eval[
                        "minimum_clearance_m"
                    ],
                    "certified_stop_available": sum(
                        item["certified_stop"]
                        for item in result["candidate_set"]["candidates"]
                    ) == 1,
                    "actor_cleared_certificate_active": spatial_core.actor_cleared(
                        snapshot, planner
                    ),
                    "segment_duration_s": duration,
                    "segment": segment_receipt,
                    "state_after": new_state,
                })
                state = new_state
                goal_reached = (
                    state["ego_x_m"] >= goal_x
                    and state["ego_speed_mps"] >= 0.0
                )
        except Exception as exc:
            driver.append(journal, {
                "event": "terminal_failure", "sequence": row["sequence"],
                "unit_id": row["unit_id"],
                "error": type(exc).__name__ + ":" + str(exc),
                "consumed_units": row["sequence"],
                "remaining_units_forfeited": 18 - row["sequence"],
            })
            raise
        receipt = {
            "schema_version": "1.0", "architecture_generation": "v3",
            "stage": "V3-08-A1", "status": "unit_complete",
            "row": dict(row),
            "planner_contract_sha256": contract["upstream"][
                "planner_contract_sha256"
            ],
            "frozen_model_sha256": contract["frozen_model"]["sha256"],
            "prefix": prefix_receipt, "decisions": decisions,
            "decision_count": len(decisions), "goal_reached": goal_reached,
            "goal_time_s": state["global_time_s"] if goal_reached else None,
            "final_state": state,
            "evaluator_policy_not_in_any_planner_snapshot": all(
                "policy" not in json.dumps(
                    item["snapshot_runtime_input_without_policy"], sort_keys=True
                )
                for item in decisions
            ),
        }
        receipt_path = unit_root / "episode_receipt.yaml"
        receipt_path.write_bytes(v.yaml_bytes(receipt))
        completed += 1
        driver.append(journal, {
            "event": "unit_complete", "sequence": row["sequence"],
            "unit_id": row["unit_id"],
            "episode_receipt_sha256": v.sha256_bytes(v.read_file_once(receipt_path)),
            "decision_count": len(decisions), "goal_reached": goal_reached,
            "completed_units": completed,
        })
        print(
            "[V3-08-A1] {}/18 {} goal={} time={:.2f}s decisions={}".format(
                completed, row["unit_id"], goal_reached,
                state["global_time_s"], len(decisions)
            ), flush=True,
        )
    driver.append(journal, {
        "event": "execution_complete", "authorized_units": 18,
        "consumed_units": 18, "completed_units": 18,
        "forfeited_units": 0, "retry_units": 0,
    })


if __name__ == "__main__":
    main()
