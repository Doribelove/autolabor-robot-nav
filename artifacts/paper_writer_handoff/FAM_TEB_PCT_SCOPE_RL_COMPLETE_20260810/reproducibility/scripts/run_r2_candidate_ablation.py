#!/usr/bin/env python3
"""Execute one preregistered R2 development-ablation identity."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

import yaml

import run_development_matrix as r1


ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark/development/r2_candidate_coverage_d0"
SCHEDULE = OUTPUT / "schedule.csv"
LAUNCH = "m2_pct_teb_dwa_benchmark_r2_episode.launch"
CHECKPOINT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark/development/training/pct_benchmark_r1_seed9701/actor_final.pt"
EXPECTED_ACTOR_SHA256 = "deaf8b9fce438d304bd45c261ff1fa6315ff04aaaa52a36b5974db2ca1d8c6b8"
VARIANTS = ("r1_control_v040", "r2_candidate_v060")
SOURCE_BINDINGS = (
    ROOT / "scripts/gazebo_pct_teb_dwa_benchmark/run_r2_candidate_ablation.py",
    ROOT / "scripts/gazebo_pct_teb_dwa_benchmark/run_development_matrix.py",
    ROOT / "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py",
    ROOT / "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_r2_episode.py",
    ROOT / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py",
    ROOT / "src/simulation/m2_gazebo/launch/m2_pct_teb_dwa_benchmark_r2_episode.launch",
    ROOT / "src/application/v5_e2e_scope_rl/config/pct_teb_dwa_benchmark_r2_development.yaml",
    OUTPUT / "preregistration.yaml",
    SCHEDULE,
    OUTPUT / "design_freeze.json",
    OUTPUT / "failures/196201_control_import_failure.json",
    OUTPUT / "failures/pct_teb_dwa_benchmark_r2_episode.failed_v0.py",
    OUTPUT / "integration_repair_amendment_1.json",
)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def registered_row(seed, scene, variant):
    with SCHEDULE.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    matches = [
        row for row in rows
        if int(row["seed"]) == seed
        and row["scene_id"] == scene
        and row["method"] == "pct_scope_rl"
        and row["candidate_variant"] == variant
        and int(row["attempt"]) == 1
    ]
    if len(matches) != 1:
        raise RuntimeError("IDENTITY_NOT_EXACTLY_ONCE_IN_PREREGISTERED_SCHEDULE")
    return matches[0]


def execute(seed, scene, variant, port):
    if not 196201 <= seed <= 196212:
        raise RuntimeError("SEED_OUTSIDE_R2_D0_PREREGISTRATION")
    if variant not in VARIANTS:
        raise RuntimeError("UNKNOWN_CANDIDATE_VARIANT")
    row = registered_row(seed, scene, variant)
    if sha256(CHECKPOINT) != EXPECTED_ACTOR_SHA256:
        raise RuntimeError("R1_ACTOR_HASH_MISMATCH")
    root = OUTPUT / "episodes" / "{}_{}_pct_scope_rl_{}".format(seed, scene, variant)
    root.mkdir(parents=True, exist_ok=False)
    world, manifest = r1.render(scene, seed, split="development")
    world_path = root / "world.sdf"
    manifest_path = root / "scene_manifest.yaml"
    report_path = root / "episode_report.yaml"
    log_path = root / "roslaunch.log"
    world_path.write_text(world, encoding="utf-8")
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    preflight_path = root / "preflight_bindings.json"
    preflight = {
        "schema_version": "1.0",
        "stage": "PCT-BENCH-R2-D0",
        "block_id": row["block_id"],
        "seed": seed,
        "scene_id": scene,
        "method": "pct_scope_rl",
        "candidate_variant": variant,
        "attempt": 1,
        "order_index": int(row["order_index"]),
        "actor_checkpoint_path": str(CHECKPOINT.relative_to(ROOT)),
        "actor_checkpoint_sha256": sha256(CHECKPOINT),
        "world_sha256": sha256(world_path),
        "scene_manifest_sha256": sha256(manifest_path),
        "resource_bindings": [
            {"path": str(path.relative_to(ROOT)), "sha256": sha256(path)}
            for path in SOURCE_BINDINGS
        ],
    }
    preflight_path.write_text(
        json.dumps(preflight, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    runtime = root / "runtime"
    (runtime / "roslog").mkdir(parents=True)
    environment = dict(os.environ)
    environment.update({
        "THESIS_WS": str(ROOT),
        "ROS_HOME": str((runtime / "ros_home").resolve()),
        "ROS_LOG_DIR": str((runtime / "roslog").resolve()),
        "ROS_MASTER_URI": "http://127.0.0.1:{}".format(port),
        "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(port + 1),
        "GAZEBO_MODEL_DATABASE_URI": "",
    })
    command = [
        "roslaunch", "m2_gazebo", LAUNCH,
        "world:={}".format(world_path.resolve()),
        "method:=pct_scope_rl",
        "scene_manifest:={}".format(manifest_path.resolve()),
        "scene_id:={}".format(scene),
        "seed:={}".format(seed),
        "report_path:={}".format(report_path.resolve()),
        "stage:=PCT-BENCH-R2-D0",
        "split:=development",
        "candidate_variant:={}".format(variant),
        "actor_checkpoint_path:={}".format(CHECKPOINT.resolve()),
        "gui:=false",
    ]
    started = time.monotonic()
    with log_path.open("wb") as log:
        process = subprocess.Popen(
            command, cwd=str(ROOT), env=environment, stdin=subprocess.DEVNULL,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        timed_out = False
        try:
            return_code = process.wait(timeout=150.0)
        except subprocess.TimeoutExpired:
            timed_out = True
            return_code = r1.stop(process)
        else:
            r1.stop(process)
    receipt = {
        "stage": "PCT-BENCH-R2-D0",
        "block_id": row["block_id"],
        "seed": seed,
        "scene_id": scene,
        "method": "pct_scope_rl",
        "candidate_variant": variant,
        "attempt": 1,
        "order_index": int(row["order_index"]),
        "return_code": return_code,
        "wall_elapsed_s": time.monotonic() - started,
        "timed_out": timed_out,
        "episode_report_present": report_path.is_file(),
        "episode_report_sha256": sha256(report_path) if report_path.is_file() else None,
        "roslaunch_log_sha256": sha256(log_path),
        "preflight_bindings_sha256": sha256(preflight_path),
    }
    (root / "receipt.json").write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if not report_path.is_file():
        raise RuntimeError(
            "episode report absent; tail:\n" + log_path.read_text(errors="replace")[-8000:]
        )
    report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
    if report.get("candidate_contract_variant") != variant:
        raise RuntimeError("EPISODE_CANDIDATE_VARIANT_MISMATCH")
    return root, report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--scene", choices=sorted(r1.SCENES), required=True)
    parser.add_argument("--candidate-variant", choices=VARIANTS, required=True)
    parser.add_argument("--port", type=int, default=15900)
    arguments = parser.parse_args()
    root, report = execute(
        arguments.seed, arguments.scene, arguments.candidate_variant, arguments.port
    )
    print(root.relative_to(ROOT))
    print("goal={} integrity={} time={} clearance={}".format(
        report["result"]["goal_reached"], report["execution_integrity_pass"],
        report["result"]["restricted_elapsed_s"],
        report["result"]["minimum_conservative_clearance_m"],
    ))


if __name__ == "__main__":
    main()
