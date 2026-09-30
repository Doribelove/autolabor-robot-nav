#!/usr/bin/env python3
"""Run the one-shot V3-03-A1 probe-select-execute integration schedule."""

import argparse
import copy
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time


W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_03_a1.py"


def module():
    spec = importlib.util.spec_from_file_location("v3_03_a1_runner", M)
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
    release_bytes = v.read_file_once(W / v.RELEASE_RELATIVE)
    review_bytes = v.read_file_once(W / v.REVIEW_RELATIVE)
    freeze_bytes = v.read_file_once(W / v.READINESS_FREEZE_RELATIVE)
    auth_bytes = v.read_file_once(W / v.AUTHORIZATION_RELATIVE)
    v.require(v.sha256_bytes(release_bytes) == release_sha, "release SHA differs")
    v.require(
        v.sha256_bytes(auth_bytes) == authorization_sha,
        "authorization SHA differs",
    )
    authorization = v.load_yaml_relative(W, v.AUTHORIZATION_RELATIVE)
    v.require(
        authorization["stage"] == contract["stage"]
        and authorization["status"]
        == "explicit_bounded_simulation_authorization"
        and authorization["simulation_authorized"] is True
        and authorization["authorized_units"]
        == contract["experimental_unit"]["simulation_units"]
        and authorization["attempts_per_identity"] == 1
        and authorization["retry_authorized"] is False
        and authorization["resume_authorized"] is False
        and authorization["training_authorized"] is False
        and authorization["move_base_authorized"] is False
        and authorization["cmd_vel_publish_authorized"] is False
        and authorization["real_vehicle_authorized"] is False,
        "authorization boundary differs",
    )
    v.require(
        authorization["release_sha256"] == release_sha
        and authorization["independent_review_sha256"]
        == v.sha256_bytes(review_bytes)
        and authorization["readiness_freeze_sha256"]
        == v.sha256_bytes(freeze_bytes),
        "authorization binding differs",
    )


def candidate_override(contract, snapshot, candidate):
    initial = contract["scene"]["ego"]["initial_pose_m_rad"]
    decision_time = float(
        contract["probe_select_execute"]["decision_absolute_time_s"]
    )
    values = [
        [0.0, [float(value) for value in initial]],
        [
            decision_time,
            [
                float(snapshot["ego"]["x_m"]),
                float(snapshot["ego"]["y_m"]),
                float(snapshot["ego"]["yaw_rad"]),
            ],
        ],
    ]
    values.extend(
        [
            decision_time + float(point["time_from_start_s"]),
            [
                float(point["x_m"]),
                float(point["y_m"]),
                float(point["yaw_rad"]),
            ],
        ]
        for point in candidate["points"][1:]
    )
    return values


def run_phase(
    v,
    contract,
    bindings,
    schedule_row,
    row,
    phase_root,
    duration_s,
    journal,
    total_units,
):
    phase_root.mkdir(parents=True, exist_ok=False)
    raw = phase_root / "raw_pose_trace.csv"
    marker = phase_root / "done.marker"
    world = phase_root / "world.sdf"
    log = phase_root / "gzserver.log"
    phase_contract = copy.deepcopy(contract)
    phase_contract["scene"]["duration_s"] = float(duration_s)
    world_bytes = v.render_world(phase_contract, row, raw, marker).encode("utf-8")
    world.write_bytes(world_bytes)
    append(
        journal,
        {
            "event": "unit_consumed",
            "sequence": schedule_row["sequence"],
            "unit_id": schedule_row["unit_id"],
            "decision_id": schedule_row["decision_id"],
            "phase": schedule_row["phase"],
            "attempt": 1,
            "world_sha256": v.sha256_bytes(world_bytes),
        },
    )
    runtime_home = phase_root / "runtime_home"
    runtime_home.mkdir()
    environment = dict(bindings["environment"])
    environment.update(
        {
            "HOME": str(runtime_home),
            "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(
                13500 + int(schedule_row["sequence"])
            ),
            "GAZEBO_MODEL_DATABASE_URI": "",
        }
    )
    command = [
        bindings["gazebo_executable"]["path"],
        "--verbose",
        str(world),
    ]
    with log.open("wb") as stream:
        process = subprocess.Popen(
            command,
            cwd=str(phase_root),
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
        append(
            journal,
            {
                "event": "terminal_failure",
                "sequence": schedule_row["sequence"],
                "unit_id": schedule_row["unit_id"],
                "phase": schedule_row["phase"],
                "marker_seen": marker_seen,
                "raw_csv_present": raw.is_file(),
                "gzserver_return_code": code,
                "consumed_units": schedule_row["sequence"],
                "remaining_units_forfeited": total_units
                - schedule_row["sequence"],
            },
        )
        raise v.V301Error("terminal phase failure: {}".format(schedule_row["unit_id"]))
    raw_bytes = v.read_file_once(raw)
    result = {
        "unit_id": schedule_row["unit_id"],
        "phase": schedule_row["phase"],
        "raw_csv_sha256": v.sha256_bytes(raw_bytes),
        "raw_csv_size_bytes": len(raw_bytes),
        "world_sha256": v.sha256_bytes(world_bytes),
        "gzserver_log_sha256": v.sha256_bytes(v.read_file_once(log)),
        "gzserver_return_code_after_marker": code,
    }
    append(
        journal,
        {
            "event": "unit_complete",
            "sequence": schedule_row["sequence"],
            "unit_id": schedule_row["unit_id"],
            "decision_id": schedule_row["decision_id"],
            "phase": schedule_row["phase"],
            "raw_csv_sha256": result["raw_csv_sha256"],
        },
    )
    return result, v.load_trace(raw)


def actor_snapshot(v, trace, row):
    angle = float(row["angle_deg"])
    radians = math.radians(angle)
    dx = math.cos(radians)
    dy = math.sin(radians)
    coordinate = (
        v.interpolate(trace, "actor_x_m", 0.5) * dx
        + v.interpolate(trace, "actor_y_m", 0.5) * dy
    )
    coordinate_before = (
        v.interpolate(trace, "actor_x_m", 0.3) * dx
        + v.interpolate(trace, "actor_y_m", 0.3) * dy
    )
    return {
        "actor_coordinate_m": coordinate,
        "actor_speed_mps": (coordinate - coordinate_before) / 0.2,
        "ego_x_m": v.interpolate(trace, "ego_x_m", 0.5),
        "ego_y_m": v.interpolate(trace, "ego_y_m", 0.5),
        "ego_yaw_rad": v.interpolate(trace, "ego_yaw_rad", 0.5),
    }


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
    planner_contract = v.load_yaml_path(
        W / contract["upstream"]["planner_contract_path"]
    )
    model = v.load_yaml_path(W / contract["upstream"]["predictor_path"])

    root = W / v.EXECUTION_RELATIVE
    v.require(not root.exists(), "execution root exists; resume forbidden")
    root.mkdir(parents=True)
    journal = W / v.JOURNAL_RELATIVE
    journal.touch(exist_ok=False)
    append(
        journal,
        {
            "event": "execution_opened",
            "release_sha256": args.release_sha,
            "authorization_sha256": args.authorization_sha,
            "authorized_units": 18,
            "decision_count": 9,
            "consumed_units": 0,
        },
    )
    rows = schedule["rows"]
    completed_units = 0
    completed_decisions = 0
    for probe_schedule, execution_schedule in zip(rows[::2], rows[1::2]):
        v.require(
            probe_schedule["decision_id"] == execution_schedule["decision_id"]
            and probe_schedule["phase"] == "common_prefix_probe"
            and execution_schedule["phase"] == "selected_candidate_execution",
            "phase pairing differs",
        )
        decision_root = (
            root / "decisions" / probe_schedule["decision_id"]
        )
        decision_root.mkdir(parents=True, exist_ok=False)
        base_row = {
            key: probe_schedule[key]
            for key in (
                "block_id",
                "seed",
                "angle_deg",
                "actor_start_distance_m",
                "actor_base_speed_mps",
                "actor_policy_evaluator_only",
            )
        }
        probe_row = dict(base_row)
        probe_row.update(
            {
                "unit_id": probe_schedule["unit_id"],
                "ego_speed_mps": 0.8,
                "candidate_id_evaluator_only": contract[
                    "probe_select_execute"
                ]["probe_candidate_id"],
                "certified_stop": False,
            }
        )
        probe_receipt, probe_trace = run_phase(
            v,
            contract,
            bindings,
            probe_schedule,
            probe_row,
            decision_root / "probe",
            contract["probe_select_execute"]["probe_duration_s"],
            journal,
            18,
        )
        completed_units += 1
        measured = actor_snapshot(v, probe_trace, base_row)
        snapshot = {
            "scene_snapshot_id": probe_schedule["decision_id"],
            "world_model_seq": completed_decisions + 1,
            "snapshot_age_s": 0.0,
            "predictor_latency_ms": 0.0,
            "verifier_latency_ms": 0.0,
            "ego": {
                "x_m": measured["ego_x_m"],
                "y_m": measured["ego_y_m"],
                "yaw_rad": measured["ego_yaw_rad"],
                "speed_mps": 0.8,
            },
            "actor": {
                "coordinate_m": measured["actor_coordinate_m"],
                "speed_mps": measured["actor_speed_mps"],
                "path_angle_deg": float(base_row["angle_deg"]),
            },
            "health": {
                "localization_ok": True,
                "transform_ok": True,
                "actor_track_ok": True,
                "command_chain_ok": True,
            },
        }
        started = time.perf_counter_ns()
        planner_result = v.core.plan(snapshot, model, planner_contract)
        planner_latency_ms = (time.perf_counter_ns() - started) / 1e6
        selected_id = planner_result["decision"]["selected_candidate_id"]
        selected = next(
            item
            for item in planner_result["candidate_set"]["candidates"]
            if item["candidate_id"] == selected_id
        )
        selected_evaluation = next(
            item
            for item in planner_result["evaluations"]
            if item["candidate_id"] == selected_id
        )
        v.require(
            selected_evaluation["accepted"] is True,
            "planner selected rejected candidate",
        )
        execution_row = dict(base_row)
        execution_row.update(
            {
                "unit_id": execution_schedule["unit_id"],
                "ego_speed_mps": float(selected["target_speed_mps"]),
                "candidate_id_evaluator_only": selected_id,
                "certified_stop": bool(selected["certified_stop"]),
                "ego_waypoints_override": candidate_override(
                    contract, snapshot, selected
                ),
            }
        )
        execution_receipt, execution_trace = run_phase(
            v,
            contract,
            bindings,
            execution_schedule,
            execution_row,
            decision_root / "execution",
            contract["probe_select_execute"]["execution_duration_s"],
            journal,
            18,
        )
        completed_units += 1
        execution_measured = actor_snapshot(v, execution_trace, base_row)
        receipt = {
            "schema_version": "1.0",
            "stage": contract["stage"],
            "status": "decision_pair_complete",
            "decision_id": probe_schedule["decision_id"],
            "block_id": base_row["block_id"],
            "seed": base_row["seed"],
            "actor_policy_evaluator_only": base_row[
                "actor_policy_evaluator_only"
            ],
            "snapshot_runtime_input_without_policy": snapshot,
            "planner_latency_ms": planner_latency_ms,
            "planner_result": planner_result,
            "selected_candidate_id": selected_id,
            "selected_candidate_digest": selected["candidate_digest"],
            "probe_observation": measured,
            "execution_observation": execution_measured,
            "probe_phase": probe_receipt,
            "execution_phase": execution_receipt,
        }
        receipt_path = decision_root / "decision_receipt.yaml"
        receipt_path.write_bytes(v.yaml_bytes(receipt))
        completed_decisions += 1
        append(
            journal,
            {
                "event": "decision_complete",
                "decision_id": probe_schedule["decision_id"],
                "selected_candidate_id": selected_id,
                "planner_latency_ms": planner_latency_ms,
                "decision_receipt_sha256": v.sha256_bytes(
                    v.read_file_once(receipt_path)
                ),
                "completed_decisions": completed_decisions,
                "completed_units": completed_units,
            },
        )
        print(
            "[{}] {}/9 {} selected={} units={}/18".format(
                contract["stage"],
                completed_decisions,
                probe_schedule["decision_id"],
                selected_id,
                completed_units,
            ),
            flush=True,
        )
    append(
        journal,
        {
            "event": "execution_complete",
            "authorized_units": 18,
            "consumed_units": 18,
            "completed_units": 18,
            "completed_decisions": 9,
            "forfeited_units": 0,
            "retry_units": 0,
        },
    )


if __name__ == "__main__":
    main()
