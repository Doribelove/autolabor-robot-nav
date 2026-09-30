#!/usr/bin/env python3
"""Run bounded V3-07-A3 cloned probe/treatment safe-overlap simulation."""

import argparse
import copy
import csv
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time

W = Path("/home/robot/robot_ws_base_rl")
MODULE = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_07_a3.py"


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def append(path, value):
    payload = (
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
        + "\n"
    ).encode()
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


def verify(m, release_sha, authorization_sha):
    release = m.read_file_once(W / m.RELEASE_RELATIVE)
    review = m.read_file_once(W / m.REVIEW_RELATIVE)
    freeze = m.read_file_once(W / m.READINESS_FREEZE_RELATIVE)
    authorization_bytes = m.read_file_once(W / m.AUTHORIZATION_RELATIVE)
    m.require(m.sha256_bytes(release) == release_sha, "A3 release differs")
    m.require(
        m.sha256_bytes(authorization_bytes) == authorization_sha,
        "A3 authorization SHA differs",
    )
    authorization = m.load_yaml_relative(W, m.AUTHORIZATION_RELATIVE)
    m.require(
        authorization["stage"] == "V3-07-A3"
        and authorization["status"] == "explicit_bounded_simulation_authorization"
        and authorization["simulation_authorized"] is True
        and authorization["authorized_units"] == 48
        and authorization["attempts_per_identity"] == 1
        and authorization["retry_authorized"] is False
        and authorization["resume_authorized"] is False
        and authorization["training_authorized"] is False
        and authorization["real_vehicle_authorized"] is False,
        "A3 authorization boundary differs",
    )
    m.require(
        authorization["release_sha256"] == release_sha
        and authorization["independent_review_sha256"]
        == m.sha256_bytes(review)
        and authorization["readiness_freeze_sha256"]
        == m.sha256_bytes(freeze),
        "A3 authorization binding differs",
    )


def trace(path):
    with path.open(newline="") as stream:
        rows = [
            {key: float(value) for key, value in row.items()}
            for row in csv.DictReader(stream)
        ]
    return rows


def coordinate(m, rows, row, stamp):
    dx, dy = m.actor_direction(row)
    return (
        m.interpolate(rows, "actor_x_m", stamp) * dx
        + m.interpolate(rows, "actor_y_m", stamp) * dy
    )


def observed_speed(m, rows, row):
    return (coordinate(m, rows, row, 0.5) - coordinate(m, rows, row, 0.3)) / 0.2


def run_world(m, contract, bindings, row, root, port, duration, candidate):
    root.mkdir(parents=True, exist_ok=False)
    raw = root / "raw_pose_trace.csv"
    marker = root / "done.marker"
    world = root / "world.sdf"
    log = root / "gzserver.log"
    effective = copy.deepcopy(contract)
    effective["scene"]["duration_s"] = float(duration)
    runtime_row = dict(row)
    runtime_row["runtime_candidate_id"] = candidate
    world_bytes = m.render_world(effective, runtime_row, raw, marker).encode()
    world.write_bytes(world_bytes)
    runtime_home = root / "runtime_home"
    runtime_home.mkdir()
    environment = dict(bindings["environment"])
    environment.update({
        "HOME": str(runtime_home),
        "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(port),
        "GAZEBO_MODEL_DATABASE_URI": "",
    })
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
            "world failure marker={} raw={} code={}".format(
                marker_seen, raw.is_file(), code
            )
        )
    rows = trace(raw)
    return {
        "raw_csv_sha256": m.sha256_bytes(m.read_file_once(raw)),
        "raw_csv_size_bytes": len(m.read_file_once(raw)),
        "world_sha256": m.sha256_bytes(world_bytes),
        "gzserver_log_sha256": m.sha256_bytes(m.read_file_once(log)),
        "gzserver_return_code_after_marker": code,
        "raw_data_row_count": len(rows),
    }, rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", required=True)
    parser.add_argument("--release-sha", required=True)
    parser.add_argument("--authorization-sha", required=True)
    args = parser.parse_args()
    m = load(MODULE, "v3_07_a3_runner_module")
    verify(m, args.release_sha, args.authorization_sha)
    contract = m.load_yaml_relative(W, m.CONTRACT_RELATIVE)
    prereg = m.load_yaml_relative(W, m.PREREGISTRATION_RELATIVE)
    m.validate_contract(contract, prereg)
    schedule = m.load_yaml_relative(W, m.SCHEDULE_RELATIVE)
    m.require(schedule == m.exact_schedule(contract), "A3 schedule differs")
    closure = m.load_yaml_relative(W, m.CLOSURE_RELATIVE)
    m.check_receipts(W, closure["source_resources"])
    m.check_receipts(W, closure["binary_resources"])
    bindings = m.load_yaml_relative(W, m.BINDINGS_RELATIVE)
    execution = W / m.EXECUTION_RELATIVE
    m.require(not execution.exists(), "A3 execution exists; resume forbidden")
    execution.mkdir(parents=True)
    journal = W / m.JOURNAL_RELATIVE
    journal.touch(exist_ok=False)
    append(journal, {
        "event": "execution_opened",
        "release_sha256": args.release_sha,
        "authorization_sha256": args.authorization_sha,
        "authorized_units": 48,
        "consumed_units": 0,
    })
    probes = {}
    completed = 0
    for row in schedule["rows"]:
        append(journal, {
            "event": "unit_consumed", "sequence": row["sequence"],
            "unit_id": row["unit_id"], "attempt": 1,
        })
        key = "{}:{}".format(row["block_id"], row["actor_policy_evaluator_only"])
        try:
            if row["phase"] == "probe":
                duration = float(contract["scene"]["probe_duration_s"])
                candidate = "straight_v120"
            elif row["phase"] == "certified_stop":
                duration = float(contract["scene"]["duration_s"])
                candidate = "certified_stop_v000"
                m.require(key in probes, "A3 treatment lacks probe")
            else:
                duration = float(contract["scene"]["duration_s"])
                candidate = "spatial_left_v120"
                m.require(
                    key in probes
                    and probes[key]["inferred_mode"] in ("cautious", "hesitant"),
                    "A3 spatial gate failed closed",
                )
            root = execution / "attempts" / row["unit_id"]
            runtime, rows = run_world(
                m, contract, bindings, row, root,
                16000 + int(row["sequence"]), duration, candidate,
            )
            minimum_rows = 10 if row["phase"] == "probe" else 150
            m.require(
                len(rows) >= minimum_rows,
                "A3 trace shorter than phase minimum",
            )
            speed = observed_speed(m, rows, row)
            mode = m.infer_mode(speed, contract["observed_mode_gate"])
            if row["phase"] == "probe":
                probes[key] = {"observed_speed_mps": speed, "inferred_mode": mode}
            receipt = {
                "schema_version": "1.0", "architecture_generation": "v3",
                "stage": "V3-07-A3", "status": "unit_complete",
                "row": dict(row), "runtime_candidate_id": candidate,
                "selector_input": {
                    "actor_observed_speed_mps": (
                        speed if row["phase"] == "probe"
                        else probes[key]["observed_speed_mps"]
                    ),
                    "inferred_mode": (
                        mode if row["phase"] == "probe"
                        else probes[key]["inferred_mode"]
                    ),
                    "hidden_policy_used": False,
                },
                "treatment_prefix_observed_speed_mps": speed,
                "treatment_prefix_inferred_mode": mode,
                "runtime": runtime,
            }
            receipt_path = root / "attempt_receipt.yaml"
            receipt_path.write_bytes(m.yaml_bytes(receipt))
        except Exception as exc:
            append(journal, {
                "event": "terminal_failure", "sequence": row["sequence"],
                "unit_id": row["unit_id"],
                "error": type(exc).__name__ + ":" + str(exc),
                "consumed_units": row["sequence"],
                "remaining_units_forfeited": 48 - row["sequence"],
            })
            raise
        completed += 1
        append(journal, {
            "event": "unit_complete", "sequence": row["sequence"],
            "unit_id": row["unit_id"],
            "receipt_sha256": m.sha256_bytes(m.read_file_once(receipt_path)),
            "completed_units": completed,
        })
        print(
            "[V3-07-A3] {}/48 {} mode={} candidate={}".format(
                completed, row["unit_id"], mode, candidate
            ),
            flush=True,
        )
    append(journal, {
        "event": "execution_complete", "authorized_units": 48,
        "consumed_units": 48, "completed_units": 48,
        "forfeited_units": 0, "retry_units": 0,
    })


if __name__ == "__main__":
    main()
