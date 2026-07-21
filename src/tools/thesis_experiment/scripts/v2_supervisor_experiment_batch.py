#!/usr/bin/env python3
"""Run/resume preregistered V2-04E or V2-04F supervisor experiments."""

import argparse
import copy
import hashlib
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

from thesis_experiment.v2_evaluator import load_v2_trace, trace_sha256


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
METHODS = ("fixed_teb", "balanced_anchor", "rule_multi_anchor")
METHOD_LAUNCH = {
    "fixed_teb": dict(load_balanced_anchor="false", publish_teb_obstacles="false",
                      start_rule_supervisor="false", start_typed_transaction="false",
                      force_geometry_balanced="false"),
    "balanced_anchor": dict(load_balanced_anchor="true", publish_teb_obstacles="true",
                            start_rule_supervisor="true", start_typed_transaction="true",
                            force_geometry_balanced="true"),
    "rule_multi_anchor": dict(load_balanced_anchor="true", publish_teb_obstacles="true",
                              start_rule_supervisor="true", start_typed_transaction="true",
                              force_geometry_balanced="false"),
}


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write_yaml(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def _materialize_candidates(candidate_path, output_dir, expected_stage):
    bank = yaml.safe_load(Path(candidate_path).read_text(encoding="utf-8"))
    if not (bank["stage"] == expected_stage and bank["training_allowed"] is False):
        raise ValueError("calibration candidate bank safety boundary drifted")
    paths = {}
    for candidate in bank["candidates"]:
        config = copy.deepcopy(bank["shared_runtime_config"])
        candidate_id = candidate["candidate_id"]
        config["profile_id"] = "fam_teb_{}_{}".format(
            expected_stage.lower().replace("-", "_"), candidate_id
        )
        config["geometry"] = copy.deepcopy(candidate["geometry"])
        config["transition"] = copy.deepcopy(candidate["transition"])
        path = Path(output_dir) / (candidate_id + ".yaml")
        _write_yaml(path, config)
        paths[candidate_id] = path.resolve()
    return paths


def _load_instances(directory):
    instances = {}
    for instance_path in Path(directory).glob("*.instance.yaml"):
        instance = yaml.safe_load(instance_path.read_text(encoding="utf-8"))
        scene_id = instance["scene"]["scene_id"]
        world_path = Path(directory) / (scene_id + ".world")
        if not world_path.is_file():
            raise ValueError("compiled supervisor-experiment world is missing")
        instances[scene_id] = (instance, instance_path.resolve(), world_path.resolve())
    return instances


def build_schedule(preregistration, instances, runtime_configs):
    stage = preregistration["stage"]
    split = preregistration["split"]
    scene_ids = preregistration["scene_ids"]
    if set(scene_ids) != set(instances):
        raise ValueError("supervisor-experiment scene set drifted")
    rows = []
    if stage in ("V2-04E", "V2-04E2", "V2-04E3", "V2-04E4"):
        for candidate_id in preregistration["candidate_ids"]:
            if candidate_id not in runtime_configs:
                raise ValueError("materialized candidate set drifted")
            for scene_id in scene_ids:
                scene = instances[scene_id][0]["scene"]
                rows.append({
                    "sequence": len(rows) + 1, "stage": stage, "split": split,
                    "method": "rule_multi_anchor", "profile_id": candidate_id,
                    "runtime_config": str(runtime_configs[candidate_id]),
                    "scene_id": scene_id, "family": scene["family"], "seed": scene["seed"],
                })
    elif stage == "V2-04F":
        config = Path(preregistration["frozen_supervisor"]["path"])
        if not config.is_absolute():
            config = WORKSPACE / config
        if _sha256(config) != preregistration["frozen_supervisor"]["sha256"]:
            raise ValueError("V2-04F frozen supervisor hash drifted")
        for method in preregistration["method_ids"]:
            if method not in METHODS:
                raise ValueError("V2-04F method set drifted")
            for scene_id in scene_ids:
                scene = instances[scene_id][0]["scene"]
                rows.append({
                    "sequence": len(rows) + 1, "stage": stage, "split": split,
                    "method": method, "profile_id": "frozen_v2_04e_winner",
                    "runtime_config": str(config.resolve()), "scene_id": scene_id,
                    "family": scene["family"], "seed": scene["seed"],
                })
    else:
        raise ValueError("unknown supervisor-experiment stage")
    if len(rows) != preregistration["budget"]["planned_navigation_episode_count"]:
        raise ValueError("supervisor-experiment budget drifted")
    return rows


def _episode_output(root, row):
    return Path(root) / "episodes" / "ep_{:03d}__{}__{}".format(
        row["sequence"], row["profile_id"] if row["stage"] != "V2-04F" else row["method"],
        row["scene_id"],
    )


def _validate_completed(row, evaluation_path):
    evaluation_path = Path(evaluation_path)
    trace_path = evaluation_path.parent / "trace.csv"
    evaluation = yaml.safe_load(evaluation_path.read_text(encoding="utf-8"))
    expected = {
        "stage": row["stage"], "split": row["split"], "method": row["method"],
        "scene_id": row["scene_id"], "family": row["family"], "seed": row["seed"],
        "supervisor_profile_id": row["profile_id"], "training_used": False,
        "runtime_policy_manifest_access": False, "runtime_scene_labels_available": False,
        "typed_transaction_valid": True,
    }
    for key, value in expected.items():
        if evaluation.get(key) != value:
            raise ValueError("completed {} evaluation {} drifted".format(row["stage"], key))
    load_v2_trace(trace_path)
    if trace_sha256(trace_path) != evaluation["raw_trace_sha256"]:
        raise ValueError("completed supervisor-experiment trace hash drifted")
    if row["method"] == "fixed_teb":
        if evaluation["transaction_message_count"] != 0:
            raise ValueError("Fixed TEB unexpectedly received a transaction")
    elif evaluation["transaction_activated_count"] <= 0:
        raise ValueError("typed comparator has no activated transaction")
    return evaluation


def inventory_completed(schedule, root):
    by_identity = {(row["profile_id"], row["method"], row["scene_id"]): row
                   for row in schedule}
    completed = {}
    for path in sorted((Path(root) / "episodes").glob("*/evaluation.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        identity = (raw.get("supervisor_profile_id"), raw.get("method"), raw.get("scene_id"))
        if identity not in by_identity:
            continue
        if identity in completed:
            raise ValueError("duplicate supervisor-experiment episode identity")
        completed[identity] = (path, _validate_completed(by_identity[identity], path))
    return completed


def _terminate_group(process):
    if process.poll() is not None:
        return
    for sig, timeout_s in ((signal.SIGINT, 10.0), (signal.SIGTERM, 5.0),
                           (signal.SIGKILL, 2.0)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout_s)
            return
        except subprocess.TimeoutExpired:
            continue


def _ready(launch, environment, method, timeout_s=45.0):
    commands = [
        ["rosparam", "get", "/m2_gazebo/simulation_only"],
        ["rostopic", "echo", "-n", "1", "/clock"],
        ["rosservice", "info", "/move_base/TebLocalPlannerROS/set_parameters"],
        ["rostopic", "echo", "-n", "1", "/nav_world_model/health"],
    ]
    if method != "fixed_teb":
        commands.extend([
            ["rostopic", "echo", "-n", "1", "/teb_mode_manager/context"],
            ["rostopic", "echo", "-n", "1", "/teb_rl_v2/action_trace"],
        ])
    deadline = time.monotonic() + timeout_s
    for command in commands:
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise RuntimeError("roslaunch exited before readiness")
            try:
                result = subprocess.run(command, env=environment,
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                    timeout=3.0, check=False)
            except subprocess.TimeoutExpired:
                result = None
            if result is not None and result.returncode == 0:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("supervisor-experiment readiness timeout: {}".format(command[-1]))


def run_episode(row, args, instances, environment):
    instance, instance_path, world_path = instances[row["scene_id"]]
    scene = instance["scene"]
    output = _episode_output(args.output_root, row)
    output.mkdir(parents=True, exist_ok=True)
    launch_args = METHOD_LAUNCH[row["method"]]
    launch_command = [
        "roslaunch", "m2_gazebo", "m2_v2_04e_04f_supervisor_experiment.launch",
        "world:={}".format(world_path), "seed:={}".format(scene["seed"]),
        "x:={}".format(scene["start"]["x_m"]), "y:={}".format(scene["start"]["y_m"]),
        "yaw:={}".format(scene["start"]["yaw_rad"]), "gui:=false",
        "rule_supervisor_config:={}".format(row["runtime_config"]),
    ] + ["{}:={}".format(name, value) for name, value in launch_args.items()]
    runner_command = [
        sys.executable, str(args.episode_runner), "--instance", str(instance_path),
        "--method", row["method"], "--output-dir", str(output),
        "--stage", row["stage"], "--split", row["split"],
        "--profile-id", row["profile_id"],
    ]
    launch = None
    with (output / "launch.log").open("w", encoding="utf-8") as launch_log:
        try:
            launch = subprocess.Popen(launch_command, env=environment,
                stdout=launch_log, stderr=subprocess.STDOUT, start_new_session=True)
            _ready(launch, environment, row["method"])
            with (output / "runner.log").open("w", encoding="utf-8") as runner_log:
                result = subprocess.run(runner_command, env=environment,
                    stdout=runner_log, stderr=subprocess.STDOUT,
                    timeout=float(scene["timeout_s"]) + 75.0, check=False)
            if result.returncode != 0:
                raise RuntimeError("episode runner exited {}".format(result.returncode))
        finally:
            if launch is not None:
                _terminate_group(launch)
            time.sleep(1.0)
    path = output / "evaluation.yaml"
    return path, _validate_completed(row, path)


def write_progress(path, prereg_path, schedule, completed, failures):
    episode_rows = []
    for row in schedule:
        identity = (row["profile_id"], row["method"], row["scene_id"])
        if identity not in completed:
            continue
        evidence_path, evidence = completed[identity]
        episode_rows.append({
            **{key: row[key] for key in ("sequence", "profile_id", "method", "scene_id", "family", "seed")},
            "evaluation": str(evidence_path), "evaluation_sha256": _sha256(evidence_path),
            "trace_sha256": evidence["raw_trace_sha256"],
            "success": evidence["metrics"]["common"]["success"],
            "collision": evidence["metrics"]["common"]["collision"],
        })
    _write_yaml(path, {
        "schema_version": "2.0", "stage": schedule[0]["stage"],
        "status": "complete" if len(completed) == len(schedule) and not failures else "in_progress",
        "simulation_only": True, "runtime_ready": False, "training_started": False,
        "real_vehicle_used": False,
        "preregistration": {"path": str(prereg_path), "sha256": _sha256(prereg_path)},
        "planned_navigation_episode_count": len(schedule),
        "valid_evidence_episode_count": len(completed),
        "interface_failure_count": len(failures), "episodes": episode_rows,
        "interface_failures": list(failures),
    })


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preregistration", type=Path, required=True)
    parser.add_argument("--compiled-scenes-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--candidate-bank", type=Path)
    parser.add_argument("--episode-runner", type=Path,
        default=WORKSPACE / "src/tools/thesis_experiment/scripts/v2_supervisor_repair_episode.py")
    parser.add_argument("--max-new-episodes", type=int, default=None)
    parser.add_argument("--attempts-per-episode", type=int, default=2)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    stage, split = prereg["stage"], prereg["split"]
    expected_root = WORKSPACE / "artifacts/v2" / split / stage.lower().replace("-", "_")
    args.output_root = args.output_root.resolve()
    args.output_root.relative_to(expected_root.resolve())
    runtime_configs = {}
    if stage in ("V2-04E", "V2-04E2", "V2-04E3", "V2-04E4"):
        if args.candidate_bank is None:
            raise ValueError("V2-04E requires --candidate-bank")
        runtime_configs = _materialize_candidates(
            args.candidate_bank, args.output_root / "runtime_candidate_configs", stage
        )
    instances = _load_instances(args.compiled_scenes_dir)
    schedule = build_schedule(prereg, instances, runtime_configs)
    completed = inventory_completed(schedule, args.output_root)
    failures = []
    progress_path = args.output_root / (stage.lower().replace("-", "_") + "_progress.yaml")
    write_progress(progress_path, args.preregistration, schedule, completed, failures)
    pending = [row for row in schedule
               if (row["profile_id"], row["method"], row["scene_id"]) not in completed]
    if args.max_new_episodes is not None:
        pending = pending[:args.max_new_episodes]
    print("{} resume: {} valid evidence, {} pending selected".format(
        stage, len(completed), len(pending)), flush=True)
    if args.dry_run:
        for row in pending:
            print("{:03d} {} {}".format(row["sequence"], row["profile_id"], row["scene_id"]))
        return 0
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    for row in pending:
        identity = (row["profile_id"], row["method"], row["scene_id"])
        print("START {:03d}/{:03d} {} x {}".format(
            row["sequence"], len(schedule), row["profile_id"], row["scene_id"]), flush=True)
        last_error = None
        for attempt in range(1, args.attempts_per_episode + 1):
            try:
                path, evidence = run_episode(row, args, instances, environment)
                completed[identity] = (path, evidence)
                print("DONE {} success={} switches={}".format(
                    identity, evidence["metrics"]["common"]["success"],
                    evidence["active_anchor_switch_count"]), flush=True)
                last_error = None
                break
            except Exception as exc:
                last_error = "attempt {}: {}".format(attempt, exc)
                print("RETRY {} {}".format(identity, last_error), flush=True)
        if last_error is not None:
            failures.append({"identity": list(identity), "reason": last_error})
            write_progress(progress_path, args.preregistration, schedule, completed, failures)
            raise RuntimeError("persistent supervisor-experiment interface failure")
        write_progress(progress_path, args.preregistration, schedule, completed, failures)
    print("{} complete: {}/{} valid evidence".format(stage, len(completed), len(schedule)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
