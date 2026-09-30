#!/usr/bin/env python3
"""Execute the frozen V6-04 directional-verifier Gazebo stages."""

import argparse
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / "src/tools/thesis_experiment/scripts/run_v6_03_sensor_gazebo.py"
CONTRACT = W / "src/application/v5_e2e_scope_rl/config/v6_04_directional_shield_contract.yaml"
FOUNDATION = W / "artifacts/v6/gazebo/v6_04_a0_foundation"
SPEC = importlib.util.spec_from_file_location("v6_03_runner_for_v604", str(BASE_PATH))
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


def phase_rows(base_contract, new_contract, phase):
    rows = BASE.make_rows(base_contract, phase)
    start = int(new_contract["evaluation"][phase]["seed_start"])
    for row in rows:
        row["seed"] = start + row["block_index"] - 1
    return rows


def prepare():
    if FOUNDATION.exists():
        raise RuntimeError("V6-04 foundation already exists")
    base_contract = yaml.safe_load(BASE.read(BASE.CONTRACT))
    contract = yaml.safe_load(BASE.read(CONTRACT))
    FOUNDATION.mkdir(parents=True)
    schedule_hashes = {}
    for phase in ("preflight", "qualification", "formal"):
        path = FOUNDATION / "{}_exact_schedule.yaml".format(phase)
        BASE.dump(path, {"schema_version": "1.0", "stage": "V6-04", "phase": phase, "rows": phase_rows(base_contract, contract, phase)})
        schedule_hashes[phase] = BASE.sha(BASE.read(path))
    bindings = list(BASE.BINDINGS) + [
        "src/tools/thesis_experiment/scripts/run_v6_04_directional_gazebo.py",
        "src/application/v5_e2e_scope_rl/scripts/v6_sensor_gazebo_episode_r2.py",
        "src/application/v5_e2e_scope_rl/scripts/v6_sensor_gazebo_episode_v604.py",
        "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/directional_safety.py",
        "src/application/v5_e2e_scope_rl/config/v6_04_directional_shield_contract.yaml",
        "src/simulation/m2_gazebo/launch/m2_v6_04_sensor_benchmark_episode.launch",
        "artifacts/v6/gazebo/v6_03_r2_p0_preflight/matrix_summary.yaml",
    ]
    closure = []
    for relative in bindings:
        path = W / relative
        if not path.is_file():
            raise RuntimeError("V6-04 binding absent: {}".format(relative))
        closure.append({"path": relative, "sha256": BASE.sha(BASE.read(path))})
    for actor in base_contract["methods"]["frozen_actor_checkpoints"]:
        path = W / actor["path"]
        observed = BASE.sha(BASE.read(path))
        if observed != actor["sha256"]:
            raise RuntimeError("actor differs")
        closure.append({"path": actor["path"], "sha256": observed})
    release = {
        "schema_version": "1.0", "stage": "V6-04-A0",
        "status": "v6_04_directional_verifier_release_frozen",
        "base_contract_sha256": BASE.sha(BASE.read(BASE.CONTRACT)),
        "v6_04_contract_sha256": BASE.sha(BASE.read(CONTRACT)),
        "schedule_sha256": schedule_hashes, "runtime_bindings": closure,
        "training_authorized": False, "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    BASE.dump(FOUNDATION / "execution_release.yaml", release)
    print("v6_04_release_sha256={}".format(BASE.sha(BASE.read(FOUNDATION / "execution_release.yaml"))))


def validate_release(phase):
    release = yaml.safe_load(BASE.read(FOUNDATION / "execution_release.yaml"))
    if release["status"] != "v6_04_directional_verifier_release_frozen":
        raise RuntimeError("V6-04 release status differs")
    if release["base_contract_sha256"] != BASE.sha(BASE.read(BASE.CONTRACT)) or release["v6_04_contract_sha256"] != BASE.sha(BASE.read(CONTRACT)):
        raise RuntimeError("V6-04 contract differs")
    for item in release["runtime_bindings"]:
        path = W / item["path"]
        if not path.is_file() or BASE.sha(BASE.read(path)) != item["sha256"]:
            raise RuntimeError("V6-04 binding differs: {}".format(item["path"]))
    schedule_path = FOUNDATION / "{}_exact_schedule.yaml".format(phase)
    if BASE.sha(BASE.read(schedule_path)) != release["schedule_sha256"][phase]:
        raise RuntimeError("V6-04 schedule differs")
    if phase in ("qualification", "formal"):
        preflight = W / "artifacts/v6/gazebo/v6_04_p0_preflight/matrix_summary.yaml"
        summary = yaml.safe_load(BASE.read(preflight)) if preflight.is_file() else {}
        tracker_pass = summary.get("v6_04_tracker_gate_pass") is True
        if summary.get("phase_pass") is not True or not tracker_pass:
            raise RuntimeError("V6-04 qualification requires preflight and tracker pass")
    if phase == "formal":
        qualification = W / "artifacts/v6/gazebo/v6_04_q1_qualification/matrix_summary.yaml"
        if not qualification.is_file() or yaml.safe_load(BASE.read(qualification))["qualification_pass"] is not True:
            raise RuntimeError("V6-04 formal requires qualification pass")
    return release, yaml.safe_load(BASE.read(schedule_path))


def run_episode(row, scene, root, port):
    root.mkdir(parents=True, exist_ok=False)
    world_path, report_path, log_path = root / "world.sdf", root / "episode_report.yaml", root / "roslaunch.log"
    ros_home, ros_log = root / "runtime/ros_home", root / "runtime/roslog"
    ros_home.mkdir(parents=True); ros_log.mkdir()
    world, actor = BASE.OLD.render_world(scene, row["actor_policy"], row["seed"])
    world_path.write_text(world, encoding="utf-8")
    environment = dict(os.environ)
    environment.update({"ROS_HOME": str(ros_home.resolve()), "ROS_LOG_DIR": str(ros_log.resolve()), "ROS_MASTER_URI": "http://127.0.0.1:{}".format(port), "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(port + 1), "GAZEBO_MODEL_DATABASE_URI": ""})
    checkpoint = W / row["pct_actor_checkpoint_path"]
    command = [
        "roslaunch", "m2_gazebo", "m2_v6_04_sensor_benchmark_episode.launch",
        "world:={}".format(world_path.resolve()), "method:={}".format(row["method"]),
        "scene_id:={}".format(row["scene_id"]), "seed:={}".format(row["seed"]),
        "actor_policy:={}".format(row["actor_policy"]), "actor_angle_deg:={}".format(scene["actor_angle_deg"]),
        "report_path:={}".format(report_path.resolve()),
        "actor_checkpoint_path:={}".format(checkpoint.resolve() if row["method"] == "pct_scope_rl_v6" else ""),
        "lookahead_m:=0.700000", "gui:=false",
    ]
    started = time.monotonic()
    with log_path.open("wb") as log_stream:
        process = subprocess.Popen(command, cwd=str(W), env=environment, stdin=subprocess.DEVNULL, stdout=log_stream, stderr=subprocess.STDOUT, start_new_session=True)
        timed_out = False
        try:
            return_code = process.wait(timeout=120.0)
        except subprocess.TimeoutExpired:
            timed_out = True; return_code = BASE.stop(process)
        else:
            BASE.stop(process)
    if not report_path.is_file():
        raise RuntimeError("V6-04 report absent rc={} timeout={} tail={}".format(return_code, timed_out, log_path.read_text(errors="replace")[-6000:]))
    report = yaml.safe_load(BASE.read(report_path))
    if report.get("stage") != "V6-04" or report.get("method") != row["method"]:
        raise RuntimeError("V6-04 episode identity differs")
    if report.get("hardware_accessed") or report.get("real_vehicle_used"):
        raise RuntimeError("hardware boundary violated")
    if row["method"] == "pct_scope_rl_v6" and report["actor_checkpoint"]["sha256"] != row["pct_actor_checkpoint_sha256"]:
        raise RuntimeError("V6-04 actor differs")
    return {"row": row, "actor_generated_parameters": actor, "episode_report": report, "episode_report_sha256": BASE.sha(BASE.read(report_path)), "world_sha256": BASE.sha(BASE.read(world_path)), "roslaunch_log_sha256": BASE.sha(BASE.read(log_path)), "roslaunch_return_code": return_code, "wall_elapsed_s": time.monotonic() - started, "timed_out": timed_out}


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--phase", choices=("preflight", "qualification", "formal")); parser.add_argument("--output-root"); parser.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    if args.prepare: prepare(); return 0
    if not args.execute or not args.phase or not args.output_root: parser.error("execution requires --phase, --output-root and --execute")
    BASE.FOUNDATION = FOUNDATION; BASE.validate_release = validate_release; BASE.run_episode = run_episode; BASE.summarize = summarize_v604
    return execute_with_preserved_summary(args.phase, args.output_root)


def execute_with_preserved_summary(phase, output_root):
    BASE.execute(phase, output_root)
    return 0


ORIGINAL_SUMMARIZE = BASE.summarize
def summarize_v604(receipts, phase, release_sha):
    summary = ORIGINAL_SUMMARIZE(receipts, phase, release_sha)
    sensor_reports = [item["episode_report"] for item in receipts if item["row"]["method"] != "actual_teb"]
    summary["stage"] = "V6-04-{}".format({"preflight": "P0", "qualification": "Q1", "formal": "F1"}[phase])
    summary["v6_04_tracker_gate_pass"] = all(report["directional_reachable_occupancy"]["tracker"]["valid"] for report in sensor_reports)
    if phase == "qualification":
        summary["qualification_checks"]["directional_tracker_valid"] = summary["v6_04_tracker_gate_pass"]
        summary["qualification_pass"] = all(summary["qualification_checks"].values())
    if phase == "formal" and summary["formal_statistics"] is not None:
        summary["formal_statistics"]["effectiveness_checks"]["directional_tracker_valid"] = summary["v6_04_tracker_gate_pass"]
        summary["formal_statistics"]["effectiveness_demonstrated"] = all(summary["formal_statistics"]["effectiveness_checks"].values())
    return summary


if __name__ == "__main__":
    sys.exit(main())
