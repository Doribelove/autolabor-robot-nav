#!/usr/bin/env python3
"""Consume the one-shot V3-01 36-unit Gazebo qualification schedule."""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
MODULE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_qualification.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("v3_01_qualification", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def append_journal(path: Path, value):
    payload = (
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)
        + "\n"
    ).encode("utf-8")
    with path.open("ab", buffering=0) as stream:
        stream.write(payload)
        os.fsync(stream.fileno())


def terminate_child(process):
    if process.poll() is not None:
        return process.returncode
    process.send_signal(signal.SIGINT)
    try:
        return process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.kill()
        return process.wait(timeout=5.0)


def verify_authorization(module, release_sha, authorization_sha):
    release_payload = module.read_file_once(WORKSPACE / module.RELEASE_RELATIVE)
    review_payload = module.read_file_once(WORKSPACE / module.REVIEW_RELATIVE)
    freeze_payload = module.read_file_once(
        WORKSPACE / module.READINESS_FREEZE_RELATIVE
    )
    module.require(
        module.sha256_bytes(release_payload) == release_sha,
        "command release SHA differs",
    )
    authorization_payload = module.read_file_once(
        WORKSPACE / module.AUTHORIZATION_RELATIVE
    )
    module.require(
        module.sha256_bytes(authorization_payload) == authorization_sha,
        "command authorization SHA differs",
    )
    authorization = module.load_yaml_relative(
        WORKSPACE, module.AUTHORIZATION_RELATIVE
    )
    module.require(
        authorization["stage"] == "V3-01"
        and authorization["status"]
        == "explicit_bounded_simulation_authorization"
        and authorization["simulation_authorized"] is True
        and authorization["authorized_units"] == 36
        and authorization["attempts_per_identity"] == 1
        and authorization["retry_authorized"] is False
        and authorization["resume_authorized"] is False
        and authorization["training_authorized"] is False
        and authorization["real_vehicle_authorized"] is False,
        "authorization boundary differs",
    )
    module.require(
        authorization["release_sha256"] == release_sha,
        "authorization release binding differs",
    )
    module.require(
        authorization["independent_review_sha256"]
        == module.sha256_bytes(review_payload),
        "authorization review binding differs",
    )
    module.require(
        authorization["readiness_freeze_sha256"]
        == module.sha256_bytes(freeze_payload),
        "authorization freeze binding differs",
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--execute", action="store_true", required=True)
    parser.add_argument("--release-sha", required=True)
    parser.add_argument("--authorization-sha", required=True)
    args = parser.parse_args()
    module = load_module()
    module.require(args.execute, "--execute is required")
    verify_authorization(module, args.release_sha, args.authorization_sha)

    contract = module.load_yaml_relative(WORKSPACE, module.CONTRACT_RELATIVE)
    prereg = module.load_yaml_relative(WORKSPACE, module.PREREGISTRATION_RELATIVE)
    module.validate_contract(contract, prereg)
    schedule = module.load_yaml_relative(WORKSPACE, module.SCHEDULE_RELATIVE)
    module.require(
        schedule == module.exact_schedule(contract), "schedule differs"
    )
    closure = module.load_yaml_relative(WORKSPACE, module.CLOSURE_RELATIVE)
    module.check_receipts(WORKSPACE, closure["source_resources"])
    module.check_receipts(WORKSPACE, closure["binary_resources"])

    execution_root = WORKSPACE / module.EXECUTION_RELATIVE
    module.require(
        not execution_root.exists(),
        "execution root already exists; resume and retry are forbidden",
    )
    execution_root.mkdir(parents=True)
    journal_path = WORKSPACE / module.JOURNAL_RELATIVE
    journal_path.touch(exist_ok=False)
    append_journal(
        journal_path,
        {
            "event": "execution_opened",
            "release_sha256": args.release_sha,
            "authorization_sha256": args.authorization_sha,
            "authorized_units": 36,
            "consumed_units": 0,
        },
    )

    bindings = module.load_yaml_relative(WORKSPACE, module.BINDINGS_RELATIVE)
    gzserver = bindings["gazebo_executable"]["path"]
    completed = 0
    for row in schedule["rows"]:
        unit_root = execution_root / "attempts" / row["unit_id"]
        unit_root.mkdir(parents=True, exist_ok=False)
        raw_csv = unit_root / "raw_pose_trace.csv"
        marker = unit_root / "done.marker"
        world_path = unit_root / "world.sdf"
        log_path = unit_root / "gzserver.log"
        world_payload = module.render_world(contract, row, raw_csv, marker).encode(
            "utf-8"
        )
        world_path.write_bytes(world_payload)
        append_journal(
            journal_path,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "unit_id": row["unit_id"],
                "attempt": 1,
                "world_sha256": module.sha256_bytes(world_payload),
            },
        )

        runtime_home = unit_root / "runtime_home"
        runtime_home.mkdir()
        environment = dict(bindings["environment"])
        environment["HOME"] = str(runtime_home)
        environment["GAZEBO_MASTER_URI"] = "http://127.0.0.1:{}".format(
            12400 + int(row["sequence"])
        )
        environment["GAZEBO_MODEL_DATABASE_URI"] = ""
        command = [gzserver, "--verbose", "-u", str(world_path)]
        with log_path.open("wb") as log_stream:
            process = subprocess.Popen(
                command,
                cwd=str(unit_root),
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=log_stream,
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
            return_code = terminate_child(process)
        if not marker_seen or not raw_csv.is_file():
            append_journal(
                journal_path,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "unit_id": row["unit_id"],
                    "marker_seen": marker_seen,
                    "raw_csv_present": raw_csv.is_file(),
                    "gzserver_return_code": return_code,
                    "consumed_units": int(row["sequence"]),
                    "remaining_units_forfeited": 36 - int(row["sequence"]),
                },
            )
            raise module.V301Error(
                "unit failed terminally: {}".format(row["unit_id"])
            )
        raw_payload = module.read_file_once(raw_csv)
        metrics = module.trace_metrics(module.load_trace(raw_csv))
        receipt = {
            "schema_version": "1.0",
            "stage": "V3-01",
            "status": "unit_complete",
            "row": dict(row),
            "raw_csv_sha256": module.sha256_bytes(raw_payload),
            "raw_csv_size_bytes": len(raw_payload),
            "world_sha256": module.sha256_bytes(world_payload),
            "gzserver_log_sha256": module.sha256_bytes(
                module.read_file_once(log_path)
            ),
            "gzserver_return_code_after_marker": return_code,
            "metrics_preview": metrics,
        }
        receipt_path = unit_root / "attempt_receipt.yaml"
        receipt_path.write_bytes(module.yaml_bytes(receipt))
        completed += 1
        append_journal(
            journal_path,
            {
                "event": "unit_complete",
                "sequence": row["sequence"],
                "unit_id": row["unit_id"],
                "raw_csv_sha256": receipt["raw_csv_sha256"],
                "receipt_sha256": module.sha256_bytes(
                    module.read_file_once(receipt_path)
                ),
                "completed_units": completed,
            },
        )
        print(
            "[V3-01] {}/36 complete {}".format(
                completed, row["unit_id"]
            ),
            flush=True,
        )
    append_journal(
        journal_path,
        {
            "event": "execution_complete",
            "authorized_units": 36,
            "consumed_units": 36,
            "completed_units": completed,
            "forfeited_units": 0,
            "retry_units": 0,
        },
    )
    print("V3-01 execution complete: 36/36")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("V3-01 execution failed closed: {}".format(exc), file=sys.stderr)
        raise
