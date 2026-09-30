#!/usr/bin/env python3
"""Assess D1 common-prefix and acceleration-feasible intervention integrity."""

import argparse
import importlib.util
import json
import math
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
V3_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"


def load():
    spec = importlib.util.spec_from_file_location("v3_02_d1_assessment", V3_PATH)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)


def assess(v):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    schedule = v.load_yaml_relative(W, v.SCHEDULE_RELATIVE)
    journal = [
        json.loads(line)
        for line in (W / v.JOURNAL_RELATIVE)
        .read_text(encoding="utf-8")
        .splitlines()
        if line.strip()
    ]
    v.require(
        journal[-1]["event"] == "execution_complete"
        and journal[-1]["consumed_units"] == schedule["row_count"]
        and journal[-1]["completed_units"] == schedule["row_count"]
        and journal[-1]["retry_units"] == 0,
        "execution journal differs",
    )
    episodes = []
    maximum_trace_error = 0.0
    maximum_acceleration = 0.0
    maximum_deceleration = 0.0
    stop_terminal_error = 0.0
    stop_post_terminal_motion = 0.0
    for row in schedule["rows"]:
        root = W / v.EXECUTION_RELATIVE / "attempts" / row["unit_id"]
        receipt = v.load_yaml_path(root / "attempt_receipt.yaml")
        raw_bytes = v.read_file_once(root / "raw_pose_trace.csv")
        v.require(
            receipt["raw_csv_sha256"] == v.sha256_bytes(raw_bytes),
            "raw hash differs",
        )
        trace = v.load_trace(root / "raw_pose_trace.csv")
        metrics = v.trace_metrics(trace, row)
        v.require(metrics == receipt["metrics_preview"], "metric replay differs")
        points = v.ego_waypoints(contract, row)
        trace_error = max(
            math.hypot(
                raw["ego_x_m"]
                - float(v.interpolate_plan(points, raw["sim_time_s"])[0]),
                raw["ego_y_m"]
                - float(v.interpolate_plan(points, raw["sim_time_s"])[1]),
            )
            for raw in trace
        )
        kinematics = v.planned_kinematics(contract, row)
        maximum_trace_error = max(maximum_trace_error, trace_error)
        maximum_acceleration = max(
            maximum_acceleration, kinematics["maximum_acceleration_mps2"]
        )
        maximum_deceleration = max(
            maximum_deceleration, kinematics["maximum_deceleration_mps2"]
        )
        if row["certified_stop"]:
            terminal_x = kinematics["terminal_x_m"]
            terminal_error = abs(trace[-1]["ego_x_m"] - terminal_x)
            post_stop = [
                item["ego_x_m"]
                for item in trace
                if item["sim_time_s"] >= 1.5 - 1e-9
            ]
            post_motion = max(post_stop) - min(post_stop)
            stop_terminal_error = max(stop_terminal_error, terminal_error)
            stop_post_terminal_motion = max(
                stop_post_terminal_motion, post_motion
            )
        episodes.append(
            {
                "unit_id": row["unit_id"],
                "block_id": row["block_id"],
                "policy_evaluator_only": row[
                    "actor_policy_evaluator_only"
                ],
                "candidate_id": row["candidate_id_evaluator_only"],
                "certified_stop": row["certified_stop"],
                "raw_csv_sha256": receipt["raw_csv_sha256"],
                "maximum_ego_trace_position_error_m": trace_error,
                "planned_kinematics": kinematics,
                "actor_coordinate_at_observation_m": metrics[
                    "actor_conflict_coordinate_at_observation_m"
                ],
                "actor_speed_at_observation_mps": metrics[
                    "actor_observed_speed_mps"
                ],
            }
        )
    coordinate_spreads = []
    speed_spreads = []
    for block_id in sorted({item["block_id"] for item in episodes}):
        for policy in ("cautious", "assertive", "hesitant"):
            group = [
                item
                for item in episodes
                if item["block_id"] == block_id
                and item["policy_evaluator_only"] == policy
            ]
            coordinates = [
                item["actor_coordinate_at_observation_m"] for item in group
            ]
            speeds = [item["actor_speed_at_observation_mps"] for item in group]
            coordinate_spreads.append(max(coordinates) - min(coordinates))
            speed_spreads.append(max(speeds) - min(speeds))
    metrics = {
        "episode_count": len(episodes),
        "maximum_ego_trace_position_error_m": maximum_trace_error,
        "maximum_planned_acceleration_mps2": maximum_acceleration,
        "maximum_planned_deceleration_mps2": maximum_deceleration,
        "maximum_stop_terminal_position_error_m": stop_terminal_error,
        "maximum_stop_terminal_motion_after_1_5_s_m": (
            stop_post_terminal_motion
        ),
        "maximum_common_prefix_actor_coordinate_spread_m": max(
            coordinate_spreads
        ),
        "maximum_common_prefix_actor_speed_spread_mps": max(speed_spreads),
    }
    thresholds = contract["data_qualification_gates"]
    gates = {
        "episode_count_pass": metrics["episode_count"]
        == int(thresholds["episode_count"]),
        "ego_trace_conformity_pass": metrics[
            "maximum_ego_trace_position_error_m"
        ]
        <= float(thresholds["maximum_ego_trace_position_error_m"]),
        "planned_acceleration_pass": metrics[
            "maximum_planned_acceleration_mps2"
        ]
        <= float(thresholds["maximum_planned_acceleration_mps2"]),
        "planned_deceleration_pass": metrics[
            "maximum_planned_deceleration_mps2"
        ]
        <= float(thresholds["maximum_planned_deceleration_mps2"]),
        "stop_terminal_position_pass": metrics[
            "maximum_stop_terminal_position_error_m"
        ]
        <= float(thresholds["stop_terminal_position_error_m"]),
        "stop_terminal_hold_pass": metrics[
            "maximum_stop_terminal_motion_after_1_5_s_m"
        ]
        <= float(thresholds["stop_terminal_motion_after_1_5_s_m"]),
        "common_prefix_coordinate_pass": metrics[
            "maximum_common_prefix_actor_coordinate_spread_m"
        ]
        <= float(
            thresholds["common_prefix_actor_coordinate_spread_max_m"]
        ),
        "common_prefix_speed_pass": metrics[
            "maximum_common_prefix_actor_speed_spread_mps"
        ]
        <= float(thresholds["common_prefix_actor_speed_spread_max_mps"]),
    }
    overall = all(gates.values())
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-02-D1",
        "assessment_id": "v3_scope_v3_02_d1_data_qualification_1",
        "status": (
            "v3_02_d1_feasible_intervention_data_qualification_pass"
            if overall
            else "v3_02_d1_feasible_intervention_data_qualification_fail"
        ),
        "execution": {
            "authorized_units": schedule["row_count"],
            "consumed_units": schedule["row_count"],
            "completed_units": len(episodes),
            "retry_units": 0,
        },
        "metrics": metrics,
        "gates": gates,
        "overall_pass": overall,
        "held_out_test_accessed": False,
        "simulation_rerun_authorized": False,
        "episodes": episodes,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = load()
    value = assess(v)
    payload = v.yaml_bytes(value)
    path = W / v.DATA_QUALIFICATION_RELATIVE
    if args.check:
        v.require(path.is_file() and path.read_bytes() == payload, "assessment differs")
    else:
        write_once(path, payload)
    print(
        "status={} overall_pass={} assessment_sha256={}".format(
            value["status"], value["overall_pass"], v.sha256_bytes(payload)
        )
    )


if __name__ == "__main__":
    main()
