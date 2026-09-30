#!/usr/bin/env python3
"""Consume the one-shot V3-01-R1 repaired 36-unit schedule."""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import time


W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_r1.py"


def module():
    spec = importlib.util.spec_from_file_location("v3_01_r1", M)
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
        and authorization["authorized_units"] == authorized_units
        and authorization["attempts_per_identity"] == 1
        and authorization["retry_authorized"] is False
        and authorization["resume_authorized"] is False
        and authorization["training_authorized"] is False
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
    unit_count = len(schedule["rows"])
    v.require(
        bindings["gazebo_arguments"] == ["--verbose", "WORLD"],
        "repaired command differs",
    )

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
            "authorized_units": unit_count,
            "consumed_units": 0,
            "stage_purpose": contract.get(
                "single_factor_repair", {}
            ).get("factor_id", contract.get("stage_purpose", "data_collection")),
        },
    )
    completed = 0
    for row in schedule["rows"]:
        unit_root = root / "attempts" / row["unit_id"]
        unit_root.mkdir(parents=True, exist_ok=False)
        raw = unit_root / "raw_pose_trace.csv"
        marker = unit_root / "done.marker"
        world = unit_root / "world.sdf"
        log = unit_root / "gzserver.log"
        world_bytes = v.render_world(contract, row, raw, marker).encode("utf-8")
        world.write_bytes(world_bytes)
        append(
            journal,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "unit_id": row["unit_id"],
                "attempt": 1,
                "world_sha256": v.sha256_bytes(world_bytes),
            },
        )
        runtime_home = unit_root / "runtime_home"
        runtime_home.mkdir()
        environment = dict(bindings["environment"])
        environment.update(
            {
                "HOME": str(runtime_home),
                "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(
                    12500 + int(row["sequence"])
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
                cwd=str(unit_root),
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
                    "sequence": row["sequence"],
                    "unit_id": row["unit_id"],
                    "marker_seen": marker_seen,
                    "raw_csv_present": raw.is_file(),
                    "gzserver_return_code": code,
                    "consumed_units": int(row["sequence"]),
                    "remaining_units_forfeited": unit_count
                    - int(row["sequence"]),
                },
            )
            raise v.V301Error("terminal unit failure: {}".format(row["unit_id"]))
        raw_bytes = v.read_file_once(raw)
        trace_rows = v.load_trace(raw)
        metrics = (
            v.trace_metrics(trace_rows, row)
            if getattr(v, "TRACE_METRICS_NEEDS_ROW", False)
            else v.trace_metrics(trace_rows)
        )
        receipt = {
            "schema_version": "1.0",
            "stage": contract["stage"],
            "status": "unit_complete",
            "row": dict(row),
            "raw_csv_sha256": v.sha256_bytes(raw_bytes),
            "raw_csv_size_bytes": len(raw_bytes),
            "world_sha256": v.sha256_bytes(world_bytes),
            "gzserver_log_sha256": v.sha256_bytes(v.read_file_once(log)),
            "gzserver_return_code_after_marker": code,
            "metrics_preview": metrics,
        }
        receipt_path = unit_root / "attempt_receipt.yaml"
        receipt_path.write_bytes(v.yaml_bytes(receipt))
        completed += 1
        append(
            journal,
            {
                "event": "unit_complete",
                "sequence": row["sequence"],
                "unit_id": row["unit_id"],
                "raw_csv_sha256": receipt["raw_csv_sha256"],
                "receipt_sha256": v.sha256_bytes(
                    v.read_file_once(receipt_path)
                ),
                "completed_units": completed,
            },
        )
        print(
            "[{}] {}/{} {}".format(
                contract["stage"], completed, unit_count, row["unit_id"]
            ),
            flush=True,
        )
    append(
        journal,
        {
            "event": "execution_complete",
            "authorized_units": unit_count,
            "consumed_units": unit_count,
            "completed_units": unit_count,
            "forfeited_units": 0,
            "retry_units": 0,
        },
    )


if __name__ == "__main__":
    main()
