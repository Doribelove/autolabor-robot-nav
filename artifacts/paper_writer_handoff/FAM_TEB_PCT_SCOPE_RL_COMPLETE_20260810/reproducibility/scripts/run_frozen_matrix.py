#!/usr/bin/env python3
"""Execute one exact pilot/formal identity from a hash-bound release."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import yaml

import run_development_matrix as development


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = "m2_pct_teb_dwa_benchmark_episode.launch"
STAGES = {
    "pilot": {
        "split": "pilot",
        "report_stage": "PCT-BENCH-PILOT-R1",
        "seed_min": 196501,
        "seed_max": 196530,
    },
    "formal_round_1": {
        "split": "formal_round_1",
        "report_stage": "PCT-BENCH-FORMAL-R1",
        "seed_min": 197001,
        "seed_max": 197200,
    },
    "formal_round_2": {
        "split": "formal_round_2",
        "report_stage": "PCT-BENCH-FORMAL-R2",
        "seed_min": 198001,
        "seed_max": 198200,
    },
}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_schedule(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    required = {"block_id", "seed", "scene_id", "method", "attempt", "order_index"}
    if not rows or not required.issubset(rows[0]):
        raise RuntimeError("FROZEN_SCHEDULE_SCHEMA_INVALID")
    identities = [(row["seed"], row["scene_id"], row["method"], row["attempt"]) for row in rows]
    if len(identities) != len(set(identities)):
        raise RuntimeError("FROZEN_SCHEDULE_IDENTITY_DUPLICATE")
    return rows


def verify_release(path, expected_sha, stage):
    observed = sha256(path)
    if observed != expected_sha:
        raise RuntimeError("RELEASE_HASH_MISMATCH")
    release = yaml.safe_load(path.read_text(encoding="utf-8"))
    if release.get("stage") != stage or release.get("execution_authorized") is not True:
        raise RuntimeError("RELEASE_STAGE_OR_AUTHORITY_INVALID")
    if release.get("simulation_only") is not True or release.get("real_vehicle_authorized") is not False:
        raise RuntimeError("RELEASE_SCOPE_INVALID")
    for item in release.get("resource_bindings", []):
        target = ROOT / item["path"]
        if not target.is_file() or sha256(target) != item["sha256"]:
            raise RuntimeError("RELEASE_RESOURCE_HASH_MISMATCH:" + item["path"])
    for item in release.get("external_resource_bindings", []):
        target = Path(item["path"])
        if not target.is_absolute() or not target.is_file() or sha256(target) != item["sha256"]:
            raise RuntimeError("RELEASE_EXTERNAL_HASH_MISMATCH:" + item["path"])
    schedule = ROOT / release["schedule_path"]
    if sha256(schedule) != release["schedule_sha256"]:
        raise RuntimeError("RELEASE_SCHEDULE_HASH_MISMATCH")
    return release, schedule, load_schedule(schedule), observed


def exact_row(rows, seed, scene, method, attempt):
    matches = [
        row
        for row in rows
        if int(row["seed"]) == seed
        and row["scene_id"] == scene
        and row["method"] == method
        and int(row["attempt"]) == attempt
    ]
    if len(matches) != 1:
        raise RuntimeError("IDENTITY_NOT_EXACTLY_ONCE_IN_RELEASE_SCHEDULE")
    return matches[0]


def stop(process):
    if process.poll() is not None:
        return process.returncode
    try:
        os.killpg(process.pid, signal.SIGINT)
        return process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        return process.wait(timeout=5.0)


def execute(arguments):
    release_path = arguments.release.resolve()
    release, schedule_path, rows, release_sha = verify_release(
        release_path, arguments.expected_release_sha256, arguments.stage
    )
    contract = STAGES[arguments.stage]
    if not contract["seed_min"] <= arguments.seed <= contract["seed_max"]:
        raise RuntimeError("SEED_OUTSIDE_FROZEN_STAGE_PARTITION")
    row = exact_row(
        rows, arguments.seed, arguments.scene, arguments.method, arguments.attempt
    )
    output = arguments.output.resolve()
    root = output / "episodes" / "{}_{}_{}".format(
        arguments.seed, arguments.scene, arguments.method
    )
    if root.exists():
        raise FileExistsError("FROZEN_IDENTITY_OUTPUT_ALREADY_EXISTS")
    root.mkdir(parents=True)
    world, manifest = development.render(
        arguments.scene, arguments.seed, split=contract["split"]
    )
    world_path = root / "world.sdf"
    manifest_path = root / "scene_manifest.yaml"
    report_path = root / "episode_report.yaml"
    log_path = root / "roslaunch.log"
    world_path.write_text(world, encoding="utf-8")
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")

    actor_path = ROOT / release["method_bindings"]["pct_scope_rl"]["path"]
    teb_path = ROOT / release["method_bindings"]["actual_teb"]["path"]
    dwa_path = ROOT / release["method_bindings"]["dwa"]["path"]
    method_path = {
        "pct_scope_rl": actor_path,
        "actual_teb": teb_path,
        "dwa": dwa_path,
    }.get(arguments.method)
    if method_path is not None:
        expected = release["method_bindings"][arguments.method]["sha256"]
        if not method_path.is_file() or sha256(method_path) != expected:
            raise RuntimeError("METHOD_BINDING_HASH_MISMATCH")

    preflight_path = root / "preflight_bindings.json"
    preflight = {
        "schema_version": "1.0",
        "stage": arguments.stage,
        "split": contract["split"],
        "block_id": row["block_id"],
        "method": arguments.method,
        "scene_id": arguments.scene,
        "seed": arguments.seed,
        "attempt": arguments.attempt,
        "order_index": int(row["order_index"]),
        "release_path": str(release_path.relative_to(ROOT)),
        "release_sha256": release_sha,
        "schedule_path": str(schedule_path.relative_to(ROOT)),
        "schedule_sha256": sha256(schedule_path),
        "world_sha256": sha256(world_path),
        "scene_manifest_sha256": sha256(manifest_path),
        "resource_bindings": release["resource_bindings"],
        "external_resource_bindings": release.get("external_resource_bindings", []),
        "method_binding": release["method_bindings"][arguments.method],
    }
    preflight_path.write_text(
        json.dumps(preflight, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if arguments.preflight_only:
        return root, None

    runtime = root / "runtime"
    (runtime / "roslog").mkdir(parents=True)
    environment = dict(os.environ)
    environment.update(
        {
            "THESIS_WS": str(ROOT),
            "ROS_HOME": str((runtime / "ros_home").resolve()),
            "ROS_LOG_DIR": str((runtime / "roslog").resolve()),
            "ROS_MASTER_URI": "http://127.0.0.1:{}".format(arguments.port),
            "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(arguments.port + 1),
            "GAZEBO_MODEL_DATABASE_URI": "",
        }
    )
    command = [
        "roslaunch", "m2_gazebo", LAUNCH,
        "world:={}".format(world_path),
        "method:={}".format(arguments.method),
        "scene_manifest:={}".format(manifest_path),
        "scene_id:={}".format(arguments.scene),
        "seed:={}".format(arguments.seed),
        "report_path:={}".format(report_path),
        "stage:={}".format(contract["report_stage"]),
        "split:={}".format(contract["split"]),
        "actor_checkpoint_path:={}".format(actor_path if arguments.method == "pct_scope_rl" else ""),
        "gui:=false",
    ]
    if arguments.method == "actual_teb":
        command.append("teb_config:={}".format(teb_path))
    elif arguments.method == "dwa":
        command.append("dwa_config:={}".format(dwa_path))
    started = time.monotonic()
    with log_path.open("wb") as log:
        process = subprocess.Popen(
            command,
            cwd=str(ROOT),
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        timed_out = False
        try:
            return_code = process.wait(timeout=150.0)
        except subprocess.TimeoutExpired:
            timed_out = True
            return_code = stop(process)
        else:
            stop(process)
    receipt = {
        "stage": arguments.stage,
        "split": contract["split"],
        "block_id": row["block_id"],
        "method": arguments.method,
        "scene_id": arguments.scene,
        "seed": arguments.seed,
        "attempt": arguments.attempt,
        "order_index": int(row["order_index"]),
        "return_code": return_code,
        "wall_elapsed_s": time.monotonic() - started,
        "timed_out": timed_out,
        "release_sha256": release_sha,
        "schedule_sha256": sha256(schedule_path),
        "world_sha256": sha256(world_path),
        "scene_manifest_sha256": sha256(manifest_path),
        "roslaunch_log_sha256": sha256(log_path),
        "episode_report_present": report_path.is_file(),
        "episode_report_sha256": sha256(report_path) if report_path.is_file() else None,
        "preflight_bindings_sha256": sha256(preflight_path),
        "thesis_workspace_path": str(ROOT),
    }
    receipt_path = root / "receipt.json"
    receipt_path.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if not report_path.is_file():
        raise RuntimeError("episode report absent; tail:\n" + log_path.read_text(errors="replace")[-8000:])
    report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
    if report.get("split") != contract["split"] or report.get("stage") != contract["report_stage"]:
        raise RuntimeError("EPISODE_REPORT_STAGE_MISMATCH")
    return root, report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--stage", choices=sorted(STAGES), required=True)
    parser.add_argument("--release", type=Path, required=True)
    parser.add_argument("--expected-release-sha256", required=True)
    parser.add_argument("--method", choices=development.ALLOWED_METHODS[:1] + development.ALLOWED_METHODS[2:], required=True)
    parser.add_argument("--scene", choices=sorted(development.SCENES), required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--attempt", type=int, default=1)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--port", type=int, default=15300)
    parser.add_argument("--preflight-only", action="store_true")
    arguments = parser.parse_args()
    if arguments.attempt != 1:
        raise RuntimeError("ONLY_ATTEMPT_ONE_AUTHORIZED")
    root, report = execute(arguments)
    print(root.resolve().relative_to(ROOT))
    if report is None:
        print("preflight_only=true")
    else:
        print(
            "goal={} integrity={} time={} clearance={}".format(
                report["result"]["goal_reached"],
                report["execution_integrity_pass"],
                report["result"]["restricted_elapsed_s"],
                report["result"]["minimum_conservative_clearance_m"],
            )
        )


if __name__ == "__main__":
    main()
