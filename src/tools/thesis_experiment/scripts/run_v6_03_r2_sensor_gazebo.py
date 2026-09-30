#!/usr/bin/env python3
"""Run the frozen V6-03-R2 geometry-aligned Gazebo study."""

import argparse
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_RUNNER = W / "src/tools/thesis_experiment/scripts/run_v6_03_sensor_gazebo.py"
AMENDMENT = W / "src/application/v5_e2e_scope_rl/config/v6_03_r2_safety_alignment.yaml"
FOUNDATION = W / "artifacts/v6/gazebo/v6_03_r2_a0_foundation"
SPEC = importlib.util.spec_from_file_location("v6_03_frozen_runner_for_r2", str(BASE_RUNNER))
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


def prepare():
    if FOUNDATION.exists():
        raise RuntimeError("R2 foundation already exists")
    amendment = yaml.safe_load(BASE.read(AMENDMENT))
    contract = yaml.safe_load(BASE.read(BASE.CONTRACT))
    FOUNDATION.mkdir(parents=True)
    schedule_hashes = {}
    for phase in ("preflight", "qualification", "formal"):
        rows = BASE.make_rows(contract, phase)
        if phase == "preflight":
            for row in rows:
                row["seed"] = int(amendment["fresh_preflight"]["seed_start"])
        path = FOUNDATION / "{}_exact_schedule.yaml".format(phase)
        BASE.dump(path, {"schema_version": "1.0", "stage": "V6-03-R2", "phase": phase, "rows": rows})
        schedule_hashes[phase] = BASE.sha(BASE.read(path))
    bindings = list(BASE.BINDINGS) + [
        "src/tools/thesis_experiment/scripts/run_v6_03_r2_sensor_gazebo.py",
        "src/application/v5_e2e_scope_rl/scripts/v6_sensor_gazebo_episode_r2.py",
        "src/application/v5_e2e_scope_rl/config/v6_03_r2_safety_alignment.yaml",
        "src/simulation/m2_gazebo/launch/m2_v6_sensor_benchmark_episode_r2.launch",
        "artifacts/v6/gazebo/v6_03_r1_p0_preflight/matrix_summary_recovered_offline.yaml",
        "artifacts/v6/gazebo/v6_03_r1_p0_preflight/offline_failure_attribution.yaml",
    ]
    closure = []
    for relative in bindings:
        path = W / relative
        if not path.is_file():
            raise RuntimeError("R2 binding absent: {}".format(relative))
        closure.append({"path": relative, "sha256": BASE.sha(BASE.read(path))})
    for actor in contract["methods"]["frozen_actor_checkpoints"]:
        path = W / actor["path"]
        observed = BASE.sha(BASE.read(path))
        if observed != actor["sha256"]:
            raise RuntimeError("actor differs")
        closure.append({"path": actor["path"], "sha256": observed})
    release = {
        "schema_version": "1.0",
        "stage": "V6-03-R2-A0",
        "status": "v6_03_r2_geometry_alignment_release_frozen",
        "base_contract_sha256": BASE.sha(BASE.read(BASE.CONTRACT)),
        "amendment_sha256": BASE.sha(BASE.read(AMENDMENT)),
        "schedule_sha256": schedule_hashes,
        "runtime_bindings": closure,
        "single_behavioral_factor": "hard_shield_inflation_0.12_to_0.33_m",
        "training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    BASE.dump(FOUNDATION / "execution_release.yaml", release)
    print("r2_release_sha256={}".format(BASE.sha(BASE.read(FOUNDATION / "execution_release.yaml"))))


def validate_release(phase):
    release = yaml.safe_load(BASE.read(FOUNDATION / "execution_release.yaml"))
    if release["status"] != "v6_03_r2_geometry_alignment_release_frozen":
        raise RuntimeError("R2 release status differs")
    if release["base_contract_sha256"] != BASE.sha(BASE.read(BASE.CONTRACT)) or release["amendment_sha256"] != BASE.sha(BASE.read(AMENDMENT)):
        raise RuntimeError("R2 contracts differ")
    for item in release["runtime_bindings"]:
        path = W / item["path"]
        if not path.is_file() or BASE.sha(BASE.read(path)) != item["sha256"]:
            raise RuntimeError("R2 runtime binding differs: {}".format(item["path"]))
    schedule_path = FOUNDATION / "{}_exact_schedule.yaml".format(phase)
    if BASE.sha(BASE.read(schedule_path)) != release["schedule_sha256"][phase]:
        raise RuntimeError("R2 schedule differs")
    if phase in ("qualification", "formal"):
        preflight = W / "artifacts/v6/gazebo/v6_03_r2_p0_preflight/matrix_summary.yaml"
        if not preflight.is_file() or yaml.safe_load(BASE.read(preflight))["phase_pass"] is not True:
            raise RuntimeError("R2 qualification requires R2 preflight pass")
    if phase == "formal":
        qualification = W / "artifacts/v6/gazebo/v6_03_r2_q1_qualification/matrix_summary.yaml"
        if not qualification.is_file() or yaml.safe_load(BASE.read(qualification))["qualification_pass"] is not True:
            raise RuntimeError("R2 formal requires R2 qualification pass")
    return release, yaml.safe_load(BASE.read(schedule_path))


def run_episode(row, scene, root, port):
    root.mkdir(parents=True, exist_ok=False)
    world_path = root / "world.sdf"
    report_path = root / "episode_report.yaml"
    log_path = root / "roslaunch.log"
    ros_home = root / "runtime/ros_home"
    ros_log = root / "runtime/roslog"
    ros_home.mkdir(parents=True)
    ros_log.mkdir()
    world, actor = BASE.OLD.render_world(scene, row["actor_policy"], row["seed"])
    world_path.write_text(world, encoding="utf-8")
    environment = dict(os.environ)
    environment.update({
        "ROS_HOME": str(ros_home.resolve()),
        "ROS_LOG_DIR": str(ros_log.resolve()),
        "ROS_MASTER_URI": "http://127.0.0.1:{}".format(port),
        "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(port + 1),
        "GAZEBO_MODEL_DATABASE_URI": "",
    })
    checkpoint = W / row["pct_actor_checkpoint_path"]
    command = [
        "roslaunch", "m2_gazebo", "m2_v6_sensor_benchmark_episode_r2.launch",
        "world:={}".format(world_path.resolve()), "method:={}".format(row["method"]),
        "scene_id:={}".format(row["scene_id"]), "seed:={}".format(row["seed"]),
        "actor_policy:={}".format(row["actor_policy"]),
        "actor_angle_deg:={}".format(scene["actor_angle_deg"]),
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
            timed_out = True
            return_code = BASE.stop(process)
        else:
            BASE.stop(process)
    if not report_path.is_file():
        raise RuntimeError("R2 report absent rc={} timeout={} tail={}".format(return_code, timed_out, log_path.read_text(errors="replace")[-6000:]))
    report = yaml.safe_load(BASE.read(report_path))
    if report.get("stage") != "V6-03-R2" or report.get("method") != row["method"]:
        raise RuntimeError("R2 episode identity differs")
    if report.get("hardware_accessed") or report.get("real_vehicle_used"):
        raise RuntimeError("hardware boundary violated")
    if row["method"] == "pct_scope_rl_v6" and report["actor_checkpoint"]["sha256"] != row["pct_actor_checkpoint_sha256"]:
        raise RuntimeError("R2 runtime actor differs")
    return {
        "row": row, "actor_generated_parameters": actor, "episode_report": report,
        "episode_report_sha256": BASE.sha(BASE.read(report_path)),
        "world_sha256": BASE.sha(BASE.read(world_path)),
        "roslaunch_log_sha256": BASE.sha(BASE.read(log_path)),
        "roslaunch_return_code": return_code, "wall_elapsed_s": time.monotonic() - started,
        "timed_out": timed_out,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--phase", choices=("preflight", "qualification", "formal"))
    parser.add_argument("--output-root")
    parser.add_argument("--execute", action="store_true")
    arguments = parser.parse_args()
    if arguments.prepare:
        prepare()
        return 0
    if not arguments.execute or not arguments.phase or not arguments.output_root:
        parser.error("execution requires --phase, --output-root and --execute")
    BASE.FOUNDATION = FOUNDATION
    BASE.validate_release = validate_release
    BASE.run_episode = run_episode
    BASE.execute(arguments.phase, arguments.output_root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
