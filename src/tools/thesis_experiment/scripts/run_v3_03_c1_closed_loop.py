#!/usr/bin/env python3
"""Run the bounded V3-03-C1 segmented Gazebo closed-loop integration."""

import argparse
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_c1.py"
PACKAGE_SRC = W / "src/application/v3_scope_planner/src"
if str(PACKAGE_SRC) not in sys.path:
    sys.path.insert(0, str(PACKAGE_SRC))

from v3_scope_planner import core


def module():
    spec = importlib.util.spec_from_file_location("v3_03_c1_runner", M)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def append(path, value):
    payload = (
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
        + "\n"
    ).encode("utf-8")
    with path.open("ab", buffering=0) as stream:
        stream.write(payload)
        os.fsync(stream.fileno())


def apply_selection_method(result, row, planner):
    method = row.get("method", "v3_analytic")
    if method == "v3_analytic":
        return result
    desired_by_method = {
        "fixed_v120_shielded": "feasible_v120",
        "fixed_v080_shielded": "teb_fallback_v080",
    }
    if method not in desired_by_method:
        raise RuntimeError("unknown selection method: {}".format(method))
    desired_id = desired_by_method[method]
    desired = next(
        item
        for item in result["evaluations"]
        if item["candidate_id"] == desired_id
    )
    stop_id = planner["candidate_generator"][
        "certified_stop_candidate_id"
    ]
    stop = next(
        item
        for item in result["evaluations"]
        if item["candidate_id"] == stop_id
    )
    selected = desired if desired["accepted"] else stop
    if not selected["accepted"]:
        raise RuntimeError("shielded baseline has no accepted certified stop")
    value = copy.deepcopy(result)
    decision = {
        "schema_version": "1.0",
        "scene_snapshot_id": result["decision"]["scene_snapshot_id"],
        "candidate_set_digest": result["decision"][
            "candidate_set_digest"
        ],
        "selected_candidate_id": selected["candidate_id"],
        "policy_kind": method,
        "policy_artifact_digest": core.digest(
            {
                "method": method,
                "desired_candidate_id": desired_id,
                "fallback_candidate_id": stop_id,
                "selection_scope": "hard_verifier_accepted_set_only",
            }
        ),
        "safety_fallback_used": not desired["accepted"],
        "fallback_reason": (
            "FIXED_CANDIDATE_REJECTED_CERTIFIED_STOP"
            if not desired["accepted"]
            else ""
        ),
    }
    decision["decision_digest"] = core.digest(decision)
    value["decision"] = decision
    return value


def gazebo_master_port(contract, sequence, decision_index=0):
    allocation = contract.get("runtime_port_allocation")
    if not allocation:
        return 15000 + int(sequence) * 100 + int(decision_index)
    port = (
        int(allocation["base_port"])
        + int(sequence) * int(allocation["sequence_stride"])
        + int(decision_index)
    )
    if (
        int(decision_index) < 0
        or int(decision_index)
        > int(allocation["maximum_decision_index"])
        or port < int(allocation["minimum_allowed_port"])
        or port > int(allocation["maximum_allowed_port"])
    ):
        raise RuntimeError("Gazebo master port allocation out of bounds")
    return port


def stop(process):
    if process.poll() is not None:
        return process.returncode
    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.wait(timeout=5.0)


def verify(v, release_sha, authorization_sha):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    stage = contract["stage"]
    authorized_units = int(contract["experimental_unit"]["schedule_rows"])
    release = v.read_file_once(W / v.RELEASE_RELATIVE)
    review = v.read_file_once(W / v.REVIEW_RELATIVE)
    freeze = v.read_file_once(W / v.READINESS_FREEZE_RELATIVE)
    authorization_bytes = v.read_file_once(W / v.AUTHORIZATION_RELATIVE)
    v.require(v.sha256_bytes(release) == release_sha, "release SHA differs")
    v.require(
        v.sha256_bytes(authorization_bytes) == authorization_sha,
        "authorization SHA differs",
    )
    authorization = v.load_yaml_relative(W, v.AUTHORIZATION_RELATIVE)
    v.require(
        authorization["stage"] == stage
        and authorization["status"]
        == "explicit_bounded_simulation_authorization"
        and authorization["simulation_authorized"] is True
        and authorization["authorized_units"]
        == authorized_units
        and authorization["attempts_per_identity"] == 1
        and authorization["retry_authorized"] is False
        and authorization["resume_authorized"] is False
        and authorization["training_authorized"] is False
        and authorization["move_base_authorized"] is False
        and authorization["cmd_vel_publish_authorized"] is False
        and authorization["hardware_device_access_authorized"] is False
        and authorization["real_vehicle_authorized"] is False,
        "authorization boundary differs",
    )
    v.require(
        authorization["release_sha256"] == release_sha
        and authorization["independent_review_sha256"]
        == v.sha256_bytes(review)
        and authorization["readiness_freeze_sha256"]
        == v.sha256_bytes(freeze),
        "authorization binding differs",
    )


def pose_text(x, y, z, yaw):
    return "{:.9f} {:.9f} {:.9f} 0 0 {:.9f}".format(x, y, z, yaw)


def box_model(world, name, size, pose):
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = pose_text(
        float(pose[0]), float(pose[1]), float(size[2]) / 2.0, float(pose[2])
    )
    link = ET.SubElement(model, "link", {"name": "body"})
    for tag in ("collision", "visual"):
        item = ET.SubElement(link, tag, {"name": tag})
        geometry = ET.SubElement(item, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = "{:.6f} {:.6f} {:.6f}".format(
            *[float(value) for value in size]
        )
    return model


def indent(element, level=0):
    indentation = "\n" + level * "  "
    child_indent = "\n" + (level + 1) * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_indent
        for child in element:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = indentation
    if level and (not element.tail or not element.tail.strip()):
        element.tail = indentation


def render_segment(
    contract,
    row,
    state,
    ego_waypoints,
    duration,
    raw,
    marker,
    segment_id,
):
    scene = contract["scene"]
    ego = scene["ego"]
    actor = scene["actor"]
    angle = math.radians(float(row["angle_deg"]))
    dx, dy = math.cos(angle), math.sin(angle)
    actor_coordinate = float(state["actor_coordinate_m"])
    actor_endpoint = max(
        float(actor["endpoint_after_conflict_m"]), actor_coordinate + 0.10
    )
    actor_start_xy = (dx * actor_coordinate, dy * actor_coordinate)
    actor_end_xy = (dx * actor_endpoint, dy * actor_endpoint)
    sdf = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(sdf, "world", {"name": segment_id})
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    physics = ET.SubElement(
        world, "physics", {"name": "v3_c1_segment", "type": "ode"}
    )
    ET.SubElement(physics, "max_step_size").text = "{:.9f}".format(
        float(scene["max_step_size_s"])
    )
    ET.SubElement(physics, "real_time_update_rate").text = str(
        int(scene["real_time_update_rate_hz"])
    )
    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", {"name": "link"})
    collision = ET.SubElement(link, "collision", {"name": "collision"})
    geometry = ET.SubElement(collision, "geometry")
    plane = ET.SubElement(geometry, "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "30 30"

    ego_model = box_model(
        world,
        ego["model_name"],
        ego["footprint_m"],
        [
            state["ego_x_m"],
            state["ego_y_m"],
            state["ego_yaw_rad"],
        ],
    )
    ego_plugin = ET.SubElement(
        ego_model,
        "plugin",
        {
            "name": "segmented_ego_trajectory",
            "filename": "libv2_trajectory_actor_plugin.so",
        },
    )
    ET.SubElement(ego_plugin, "loop").text = "false"
    ET.SubElement(ego_plugin, "z").text = "{:.6f}".format(
        float(ego["footprint_m"][2]) / 2.0
    )
    for time_s, pose in ego_waypoints:
        waypoint = ET.SubElement(ego_plugin, "waypoint")
        ET.SubElement(waypoint, "time").text = "{:.9f}".format(float(time_s))
        ET.SubElement(waypoint, "x").text = "{:.9f}".format(float(pose[0]))
        ET.SubElement(waypoint, "y").text = "{:.9f}".format(float(pose[1]))
        ET.SubElement(waypoint, "yaw").text = "{:.9f}".format(float(pose[2]))

    actor_model = box_model(
        world,
        actor["model_name"],
        actor["size_m"],
        [actor_start_xy[0], actor_start_xy[1], angle],
    )
    actor_plugin = ET.SubElement(
        actor_model,
        "plugin",
        {
            "name": "segmented_reactive_actor",
            "filename": "libv3_reactive_actor_plugin.so",
        },
    )
    ET.SubElement(actor_plugin, "ego_model").text = ego["model_name"]
    ET.SubElement(actor_plugin, "policy").text = row[
        "actor_policy_evaluator_only"
    ]
    ET.SubElement(actor_plugin, "seed").text = str(row["seed"])
    ET.SubElement(actor_plugin, "start").text = "{:.9f} {:.9f}".format(
        *actor_start_xy
    )
    ET.SubElement(actor_plugin, "end").text = "{:.9f} {:.9f}".format(
        *actor_end_xy
    )
    ET.SubElement(actor_plugin, "conflict").text = "0 0"
    ET.SubElement(actor_plugin, "base_speed").text = "{:.9f}".format(
        float(row["actor_base_speed_mps"])
    )
    ET.SubElement(actor_plugin, "initial_speed").text = "{:.9f}".format(
        float(state["actor_speed_mps"])
    )
    ET.SubElement(actor_plugin, "time_offset").text = "{:.9f}".format(
        float(state["global_time_s"])
    )
    ET.SubElement(actor_plugin, "response_distance").text = "{:.9f}".format(
        float(actor["response_distance_m"])
    )
    ET.SubElement(actor_plugin, "response_time_margin").text = "{:.9f}".format(
        float(actor["response_time_margin_s"])
    )
    ET.SubElement(actor_plugin, "z").text = "{:.6f}".format(
        float(actor["size_m"][2]) / 2.0
    )

    recorder = ET.SubElement(
        world,
        "plugin",
        {
            "name": "segmented_pose_recorder",
            "filename": "libv3_episode_recorder_plugin.so",
        },
    )
    ET.SubElement(recorder, "ego_model").text = ego["model_name"]
    ET.SubElement(recorder, "actor_model").text = actor["model_name"]
    ET.SubElement(recorder, "output_csv").text = str(raw.resolve())
    ET.SubElement(recorder, "done_marker").text = str(marker.resolve())
    ET.SubElement(recorder, "duration").text = "{:.9f}".format(float(duration))
    ET.SubElement(recorder, "sample_period").text = "{:.9f}".format(
        float(scene["sample_period_s"])
    )
    indent(sdf)
    return '<?xml version="1.0"?>\n' + ET.tostring(
        sdf, encoding="unicode"
    ) + "\n"


def coordinate(v, trace, row, time_s):
    angle = math.radians(float(row["angle_deg"]))
    return (
        v.interpolate(trace, "actor_x_m", time_s) * math.cos(angle)
        + v.interpolate(trace, "actor_y_m", time_s) * math.sin(angle)
    )


def actor_speed(v, trace, row, duration):
    h = min(0.05, float(duration) / 3.0)
    end = coordinate(v, trace, row, duration)
    previous = coordinate(v, trace, row, duration - h)
    earlier = coordinate(v, trace, row, duration - 2.0 * h)
    return max(0.0, (3.0 * end - 4.0 * previous + earlier) / (2.0 * h))


def candidate_waypoints(candidate, duration):
    points = [
        point
        for point in candidate["points"]
        if float(point["time_from_start_s"]) <= duration + 1e-12
    ]
    if not points or float(points[-1]["time_from_start_s"]) < duration - 1e-12:
        raise RuntimeError("candidate does not cover segment")
    return [
        [
            float(point["time_from_start_s"]),
            [
                float(point["x_m"]),
                float(point["y_m"]),
                float(point["yaw_rad"]),
            ],
        ]
        for point in points
    ]


def run_segment(
    v,
    contract,
    bindings,
    row,
    state,
    waypoints,
    duration,
    root,
    port,
):
    root.mkdir(parents=True, exist_ok=False)
    raw = root / "raw_pose_trace.csv"
    marker = root / "done.marker"
    world = root / "world.sdf"
    log = root / "gzserver.log"
    world_bytes = render_segment(
        contract,
        row,
        state,
        waypoints,
        duration,
        raw,
        marker,
        root.name,
    ).encode("utf-8")
    world.write_bytes(world_bytes)
    runtime_home = root / "runtime_home"
    runtime_home.mkdir()
    environment = dict(bindings["environment"])
    environment.update(
        {
            "HOME": str(runtime_home),
            "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(port),
            "GAZEBO_MODEL_DATABASE_URI": "",
        }
    )
    with log.open("wb") as stream:
        process = subprocess.Popen(
            [bindings["gazebo_executable"]["path"], "--verbose", str(world)],
            cwd=str(root),
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        deadline = time.monotonic() + 30.0
        while (
            time.monotonic() < deadline
            and process.poll() is None
            and not marker.is_file()
        ):
            time.sleep(0.05)
        marker_seen = marker.is_file()
        code = stop(process)
    if not marker_seen or not raw.is_file():
        raise RuntimeError(
            "segment failed marker={} raw={} code={}".format(
                marker_seen, raw.is_file(), code
            )
        )
    trace = v.load_trace(raw)
    return {
        "raw_csv_sha256": v.sha256_bytes(v.read_file_once(raw)),
        "raw_csv_size_bytes": len(v.read_file_once(raw)),
        "world_sha256": v.sha256_bytes(world_bytes),
        "gzserver_log_sha256": v.sha256_bytes(v.read_file_once(log)),
        "gzserver_return_code_after_marker": code,
    }, trace


def clearance(trace, combined_radius):
    return min(
        math.hypot(
            float(item["ego_x_m"]) - float(item["actor_x_m"]),
            float(item["ego_y_m"]) - float(item["actor_y_m"]),
        )
        - combined_radius
        for item in trace
    )


def endpoint_state(v, trace, row, previous, duration, ego_speed):
    return {
        "global_time_s": float(previous["global_time_s"]) + float(duration),
        "ego_x_m": v.interpolate(trace, "ego_x_m", duration),
        "ego_y_m": v.interpolate(trace, "ego_y_m", duration),
        "ego_yaw_rad": v.interpolate(trace, "ego_yaw_rad", duration),
        "ego_speed_mps": float(ego_speed),
        "actor_coordinate_m": coordinate(v, trace, row, duration),
        "actor_speed_mps": actor_speed(v, trace, row, duration),
    }


def boundary_jump(v, trace, row, state):
    first = trace[0]
    angle = math.radians(float(row["angle_deg"]))
    actor_coordinate = (
        float(first["actor_x_m"]) * math.cos(angle)
        + float(first["actor_y_m"]) * math.sin(angle)
    )
    return {
        "ego_position_jump_m": math.hypot(
            float(first["ego_x_m"]) - float(state["ego_x_m"]),
            float(first["ego_y_m"]) - float(state["ego_y_m"]),
        ),
        "actor_position_jump_m": abs(
            actor_coordinate - float(state["actor_coordinate_m"])
        ),
    }


def trace_error(v, trace, waypoints):
    return max(
        math.hypot(
            float(item["ego_x_m"])
            - float(v.interpolate_plan(waypoints, item["sim_time_s"])[0]),
            float(item["ego_y_m"])
            - float(v.interpolate_plan(waypoints, item["sim_time_s"])[1]),
        )
        for item in trace
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", required=True)
    parser.add_argument("--release-sha", required=True)
    parser.add_argument("--authorization-sha", required=True)
    args = parser.parse_args()
    v = module()
    verify(v, args.release_sha, args.authorization_sha)
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    prereg = v.load_yaml_relative(W, v.PREREGISTRATION_RELATIVE)
    v.validate_contract(contract, prereg)
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
    execution = W / v.EXECUTION_RELATIVE
    v.require(not execution.exists(), "execution root exists; resume forbidden")
    execution.mkdir(parents=True)
    journal = W / v.JOURNAL_RELATIVE
    journal.touch(exist_ok=False)
    append(
        journal,
        {
            "event": "execution_opened",
            "release_sha256": args.release_sha,
            "authorization_sha256": args.authorization_sha,
            "authorized_units": schedule["row_count"],
            "consumed_units": 0,
        },
    )
    combined_radius = core._combined_radius(planner)
    prefix_duration = float(contract["closed_loop"]["observation_prefix_s"])
    period = float(contract["closed_loop"]["replanning_period_s"])
    maximum_decisions = int(
        contract["closed_loop"]["maximum_decisions_per_episode"]
    )
    goal_x = float(contract["closed_loop"]["goal_x_m"])
    completed = 0
    for row in schedule["rows"]:
        unit_root = execution / "episodes" / row["unit_id"]
        unit_root.mkdir(parents=True, exist_ok=False)
        append(
            journal,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "unit_id": row["unit_id"],
                "attempt": 1,
            },
        )
        angle = math.radians(float(row["angle_deg"]))
        state = {
            "global_time_s": 0.0,
            "ego_x_m": float(contract["scene"]["ego"]["initial_pose_m_rad"][0]),
            "ego_y_m": float(contract["scene"]["ego"]["initial_pose_m_rad"][1]),
            "ego_yaw_rad": float(
                contract["scene"]["ego"]["initial_pose_m_rad"][2]
            ),
            "ego_speed_mps": float(
                contract["scene"]["ego"]["initial_speed_mps"]
            ),
            "actor_coordinate_m": -float(row["actor_start_distance_m"]),
            "actor_speed_mps": float(row["actor_base_speed_mps"]),
        }
        prefix_waypoints = [
            [
                0.0,
                [state["ego_x_m"], state["ego_y_m"], state["ego_yaw_rad"]],
            ],
            [
                prefix_duration,
                [
                    state["ego_x_m"]
                    + state["ego_speed_mps"] * prefix_duration,
                    state["ego_y_m"],
                    state["ego_yaw_rad"],
                ],
            ],
        ]
        try:
            prefix_receipt, prefix_trace = run_segment(
                v,
                contract,
                bindings,
                row,
                state,
                prefix_waypoints,
                prefix_duration,
                unit_root / "prefix",
                gazebo_master_port(contract, row["sequence"]),
            )
        except Exception as exc:
            append(
                journal,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "unit_id": row["unit_id"],
                    "error": type(exc).__name__ + ":" + str(exc),
                    "consumed_units": row["sequence"],
                    "remaining_units_forfeited": schedule["row_count"]
                    - row["sequence"],
                },
            )
            raise
        prefix_boundary = boundary_jump(v, prefix_trace, row, state)
        prefix_receipt["boundary"] = prefix_boundary
        prefix_receipt["minimum_conservative_clearance_m"] = clearance(
            prefix_trace, combined_radius
        )
        prefix_receipt["maximum_ego_trace_error_m"] = trace_error(
            v, prefix_trace, prefix_waypoints
        )
        state = endpoint_state(
            v,
            prefix_trace,
            row,
            state,
            prefix_duration,
            contract["scene"]["ego"]["initial_speed_mps"],
        )
        decisions = []
        goal_reached = state["ego_x_m"] >= goal_x
        for decision_index in range(1, maximum_decisions + 1):
            if goal_reached:
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
                    "x_m": state["ego_x_m"],
                    "y_m": state["ego_y_m"],
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
                    "localization_ok": True,
                    "transform_ok": True,
                    "actor_track_ok": True,
                    "command_chain_ok": True,
                },
            }
            started = time.perf_counter_ns()
            result = apply_selection_method(
                core.plan(snapshot, model, planner), row, planner
            )
            latency_ms = (time.perf_counter_ns() - started) / 1e6
            selected_id = result["decision"]["selected_candidate_id"]
            selected = next(
                item
                for item in result["candidate_set"]["candidates"]
                if item["candidate_id"] == selected_id
            )
            selected_evaluation = next(
                item
                for item in result["evaluations"]
                if item["candidate_id"] == selected_id
            )
            v.require(
                selected_evaluation["accepted"] is True,
                "planner selected rejected candidate",
            )
            waypoints = candidate_waypoints(selected, period)
            planned_endpoint = next(
                point
                for point in selected["points"]
                if abs(float(point["time_from_start_s"]) - period) <= 1e-12
            )
            segment_root = unit_root / "decision_{:02d}".format(decision_index)
            try:
                segment_receipt, trace = run_segment(
                    v,
                    contract,
                    bindings,
                    row,
                    state,
                    waypoints,
                    period,
                    segment_root,
                    gazebo_master_port(
                        contract, row["sequence"], decision_index
                    ),
                )
            except Exception as exc:
                append(
                    journal,
                    {
                        "event": "terminal_failure",
                        "sequence": row["sequence"],
                        "unit_id": row["unit_id"],
                        "decision_index": decision_index,
                        "error": type(exc).__name__ + ":" + str(exc),
                        "consumed_units": row["sequence"],
                        "remaining_units_forfeited": schedule["row_count"]
                        - row["sequence"],
                    },
                )
                raise
            boundary = boundary_jump(v, trace, row, state)
            segment_receipt["boundary"] = boundary
            segment_receipt["minimum_conservative_clearance_m"] = clearance(
                trace, combined_radius
            )
            segment_receipt["maximum_ego_trace_error_m"] = trace_error(
                v, trace, waypoints
            )
            new_state = endpoint_state(
                v,
                trace,
                row,
                state,
                period,
                planned_endpoint["longitudinal_velocity_mps"],
            )
            decisions.append(
                {
                    "decision_index": decision_index,
                    "snapshot_runtime_input_without_policy": snapshot,
                    "planner_latency_ms": latency_ms,
                    "planner_result_digest": core.digest(result),
                    "candidate_set_digest": result["candidate_set"][
                        "candidate_set_digest"
                    ],
                    "decision_digest": result["decision"]["decision_digest"],
                    "selected_candidate_id": selected_id,
                    "selected_candidate_digest": selected["candidate_digest"],
                    "selected_candidate_accepted": selected_evaluation[
                        "accepted"
                    ],
                    "selected_minimum_predicted_clearance_m": (
                        selected_evaluation["minimum_clearance_m"]
                    ),
                    "certified_stop_available": sum(
                        item["certified_stop"]
                        for item in result["candidate_set"]["candidates"]
                    )
                    == 1,
                    "actor_cleared_certificate_active": core.actor_cleared(
                        snapshot, planner
                    ),
                    "segment": segment_receipt,
                    "state_after": new_state,
                }
            )
            state = new_state
            goal_reached = state["ego_x_m"] >= goal_x
        receipt = {
            "schema_version": "1.0",
            "architecture_generation": "v3",
            "stage": contract["stage"],
            "status": "unit_complete",
            "row": dict(row),
            "planner_contract_sha256": contract["upstream"][
                "planner_contract_sha256"
            ],
            "frozen_model_sha256": contract["frozen_model"]["sha256"],
            "prefix": prefix_receipt,
            "decisions": decisions,
            "decision_count": len(decisions),
            "goal_reached": goal_reached,
            "goal_time_s": state["global_time_s"] if goal_reached else None,
            "final_state": state,
            "evaluator_policy_not_in_any_planner_snapshot": all(
                "policy" not in json.dumps(
                    item["snapshot_runtime_input_without_policy"],
                    sort_keys=True,
                )
                for item in decisions
            ),
        }
        receipt_path = unit_root / "episode_receipt.yaml"
        receipt_path.write_bytes(v.yaml_bytes(receipt))
        completed += 1
        append(
            journal,
            {
                "event": "unit_complete",
                "sequence": row["sequence"],
                "unit_id": row["unit_id"],
                "episode_receipt_sha256": v.sha256_bytes(
                    v.read_file_once(receipt_path)
                ),
                "decision_count": len(decisions),
                "goal_reached": goal_reached,
                "completed_units": completed,
            },
        )
        print(
            "[{}] {}/{} {} goal={} time={:.2f}s decisions={}".format(
                contract["stage"],
                completed,
                schedule["row_count"],
                row["unit_id"],
                goal_reached,
                state["global_time_s"],
                len(decisions),
            ),
            flush=True,
        )
    append(
        journal,
        {
            "event": "execution_complete",
            "authorized_units": schedule["row_count"],
            "consumed_units": schedule["row_count"],
            "completed_units": schedule["row_count"],
            "forfeited_units": 0,
            "retry_units": 0,
        },
    )


if __name__ == "__main__":
    main()
