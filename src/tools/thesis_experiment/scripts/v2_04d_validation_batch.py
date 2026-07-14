#!/usr/bin/env python3
"""Execute or resume the preregistered V2-04D paired validation matrix."""

import argparse
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
    "fixed_teb": {
        "load_balanced_anchor": "false",
        "publish_teb_obstacles": "false",
        "start_rule_supervisor": "false",
        "start_typed_transaction": "false",
        "force_geometry_balanced": "false",
    },
    "balanced_anchor": {
        "load_balanced_anchor": "true",
        "publish_teb_obstacles": "true",
        "start_rule_supervisor": "true",
        "start_typed_transaction": "true",
        "force_geometry_balanced": "true",
    },
    "rule_multi_anchor": {
        "load_balanced_anchor": "true",
        "publish_teb_obstacles": "true",
        "start_rule_supervisor": "true",
        "start_typed_transaction": "true",
        "force_geometry_balanced": "false",
    },
}


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write_yaml(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def build_schedule(preregistration, instances):
    methods = tuple(row["method_id"] for row in preregistration["methods"])
    if methods != METHODS:
        raise ValueError("V2-04D method order drifted")
    scene_order = preregistration["validation_scene_ids"]
    if set(scene_order) != set(instances) or len(scene_order) != len(instances):
        raise ValueError("V2-04D validation scene set drifted")
    rows = []
    for method in methods:
        for scene_id in scene_order:
            scene = instances[scene_id][0]["scene"]
            rows.append({
                "sequence": len(rows) + 1,
                "method": method,
                "scene_id": scene_id,
                "family": scene["family"],
                "seed": scene["seed"],
            })
    if len(rows) != preregistration["budget"]["planned_navigation_episode_count"]:
        raise ValueError("V2-04D planned episode count drifted")
    return rows


def _episode_output(root, row):
    return Path(root) / "episodes" / "ep_{:03d}__{}__{}".format(
        row["sequence"], row["method"], row["scene_id"]
    )


def _validate_completed(row, evaluation_path):
    evaluation_path = Path(evaluation_path)
    trace_path = evaluation_path.parent / "trace.csv"
    if not trace_path.is_file():
        raise ValueError("completed V2-04D evaluation is missing trace.csv")
    evaluation = yaml.safe_load(evaluation_path.read_text(encoding="utf-8"))
    expected = {
        "stage": "V2-04D",
        "method": row["method"],
        "scene_id": row["scene_id"],
        "family": row["family"],
        "split": "validation",
        "seed": row["seed"],
        "training_used": False,
        "runtime_policy_manifest_access": False,
        "runtime_scene_labels_available": False,
        "experiment_manager_validation_manifest_access": True,
        "typed_transaction_expected": row["method"] != "fixed_teb",
        "typed_transaction_valid": True,
    }
    for key, value in expected.items():
        if evaluation.get(key) != value:
            raise ValueError("completed V2-04D evaluation {} drifted".format(key))
    load_v2_trace(trace_path)
    if trace_sha256(trace_path) != evaluation["raw_trace_sha256"]:
        raise ValueError("completed V2-04D raw trace hash drifted")
    if row["method"] == "fixed_teb":
        if evaluation["transaction_message_count"] != 0:
            raise ValueError("Fixed TEB unexpectedly received typed transactions")
    else:
        if evaluation["transaction_activated_count"] <= 0:
            raise ValueError("typed comparator has no activated transaction")
        if evaluation["transaction_backends"] != ["simulation_teb_dynamic_reconfigure"]:
            raise ValueError("typed comparator backend drifted")
    if row["method"] == "balanced_anchor":
        if set(evaluation["active_anchor_sequence"]) - {"anchor_balanced"}:
            raise ValueError("Balanced comparator switched geometry Anchor")
    return evaluation


def inventory_completed(schedule, root):
    by_identity = {(row["method"], row["scene_id"]): row for row in schedule}
    completed = {}
    for path in sorted((Path(root) / "episodes").glob("*/evaluation.yaml")):
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
        identity = (raw.get("method"), raw.get("scene_id"))
        if identity not in by_identity:
            continue
        if identity in completed:
            raise ValueError("duplicate V2-04D episode identity")
        completed[identity] = (path, _validate_completed(by_identity[identity], path))
    return completed


def _method_summary(schedule, completed, method):
    rows = [row for row in schedule if row["method"] == method]
    evidence = [completed[(method, row["scene_id"])][1]
                for row in rows if (method, row["scene_id"]) in completed]
    return {
        "planned_episode_count": len(rows),
        "valid_evidence_episode_count": len(evidence),
        "success_count": sum(bool(item["metrics"]["common"]["success"])
                             for item in evidence),
        "collision_count": sum(bool(item["metrics"]["common"]["collision"])
                               for item in evidence),
        "interface_fault_episode_count": sum(
            bool(item.get("runner_fault_reason"))
            and item["runner_fault_reason"] not in ("timeout", "collision")
            for item in evidence
        ),
    }


def write_progress(path, prereg_path, schedule, completed, failures):
    summaries = {method: _method_summary(schedule, completed, method) for method in METHODS}
    complete = len(completed) == len(schedule) and not failures
    fixed = summaries["fixed_teb"]
    nondegradation = complete and all(
        summaries[method]["success_count"] >= fixed["success_count"]
        and summaries[method]["collision_count"] <= fixed["collision_count"]
        for method in ("balanced_anchor", "rule_multi_anchor")
    )
    safety = complete and all(
        summary["collision_count"] == 0 and summary["interface_fault_episode_count"] == 0
        for summary in summaries.values()
    )
    evidence_rows = []
    for row in schedule:
        identity = (row["method"], row["scene_id"])
        if identity not in completed:
            continue
        evaluation_path, evaluation = completed[identity]
        evidence_rows.append({
            **row,
            "evaluation": str(evaluation_path),
            "evaluation_sha256": _sha256(evaluation_path),
            "trace_sha256": evaluation["raw_trace_sha256"],
            "termination": evaluation["termination"],
            "success": evaluation["metrics"]["common"]["success"],
            "collision": evaluation["metrics"]["common"]["collision"],
        })
    _write_yaml(path, {
        "schema_version": "2.0",
        "stage": "V2-04D",
        "status": "paired_validation_complete" if complete else "paired_validation_in_progress",
        "formal_result": False,
        "simulation_only": True,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "preregistration": {"path": str(prereg_path), "sha256": _sha256(prereg_path)},
        "planned_navigation_episode_count": len(schedule),
        "valid_evidence_episode_count": len(completed),
        "interface_failure_count": len(failures),
        "method_summaries": summaries,
        "stage_1_success_non_degradation_pass": nondegradation,
        "stage_1_collision_and_interface_gate_pass": safety,
        "stage_2_performance_comparison_authorized": nondegradation and safety,
        "episodes": evidence_rows,
        "interface_failures": list(failures),
    })


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


def _ready(launch, environment, method, timeout_s=40.0):
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
                raise RuntimeError("roslaunch exited before V2-04D readiness")
            try:
                result = subprocess.run(
                    command, env=environment, stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL, timeout=3.0, check=False,
                )
            except subprocess.TimeoutExpired:
                result = None
            if result is not None and result.returncode == 0:
                break
            time.sleep(0.5)
        else:
            raise RuntimeError("V2-04D readiness timeout: {}".format(command[-1]))


def run_episode(row, args, instances, environment):
    instance, instance_path, world_path = instances[row["scene_id"]]
    scene = instance["scene"]
    output = _episode_output(args.validation_root, row)
    output.mkdir(parents=True, exist_ok=True)
    launch_args = METHOD_LAUNCH[row["method"]]
    launch_command = [
        "roslaunch", "m2_gazebo", "m2_v2_04d_paired_validation.launch",
        "world:={}".format(world_path), "seed:={}".format(scene["seed"]),
        "x:={}".format(scene["start"]["x_m"]),
        "y:={}".format(scene["start"]["y_m"]),
        "yaw:={}".format(scene["start"]["yaw_rad"]), "gui:=false",
    ] + ["{}:={}".format(name, value) for name, value in launch_args.items()]
    runner_command = [
        sys.executable, str(args.episode_runner), "--instance", str(instance_path),
        "--method", row["method"], "--output-dir", str(output),
    ]
    launch = None
    with (output / "launch.log").open("w", encoding="utf-8") as launch_log:
        try:
            launch = subprocess.Popen(
                launch_command, env=environment, stdout=launch_log,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            _ready(launch, environment, row["method"])
            with (output / "runner.log").open("w", encoding="utf-8") as runner_log:
                result = subprocess.run(
                    runner_command, env=environment, stdout=runner_log,
                    stderr=subprocess.STDOUT, timeout=float(scene["timeout_s"]) + 75.0,
                    check=False,
                )
            if result.returncode != 0:
                raise RuntimeError("V2-04D episode runner exited {}".format(result.returncode))
        finally:
            if launch is not None:
                _terminate_group(launch)
            time.sleep(1.0)
    evaluation_path = output / "evaluation.yaml"
    return evaluation_path, _validate_completed(row, evaluation_path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument("--preregistration", type=Path)
    parser.add_argument("--compiled-scenes-dir", type=Path)
    parser.add_argument("--validation-root", type=Path)
    parser.add_argument("--episode-runner", type=Path)
    parser.add_argument("--max-new-episodes", type=int, default=None)
    parser.add_argument("--attempts-per-episode", type=int, default=2)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    args.workspace = args.workspace.resolve()
    args.preregistration = (args.preregistration or args.workspace /
        "experiments/manifests/v2/validation/v2_04d_preregistration.yaml").resolve()
    args.compiled_scenes_dir = (args.compiled_scenes_dir or args.workspace /
        "artifacts/v2/validation/v2_04d/compiled_scenes").resolve()
    args.validation_root = (args.validation_root or args.workspace /
        "artifacts/v2/validation/v2_04d").resolve()
    args.episode_runner = (args.episode_runner or args.workspace /
        "src/tools/thesis_experiment/scripts/v2_04d_validation_episode.py").resolve()
    args.validation_root.relative_to((args.workspace / "artifacts/v2/validation/v2_04d").resolve())
    if args.max_new_episodes is not None and args.max_new_episodes < 0:
        raise ValueError("max-new-episodes must be non-negative")
    if args.attempts_per_episode < 1:
        raise ValueError("attempts-per-episode must be positive")
    preregistration = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    if not (preregistration["simulation_only"] is True
            and preregistration["training_started"] is False
            and preregistration["runtime_ready"] is False):
        raise ValueError("V2-04D preregistration safety boundary drifted")
    instances = {}
    for instance_path in args.compiled_scenes_dir.glob("*.instance.yaml"):
        instance = yaml.safe_load(instance_path.read_text(encoding="utf-8"))
        scene_id = instance["scene"]["scene_id"]
        world_path = args.compiled_scenes_dir / (scene_id + ".world")
        if not world_path.is_file():
            raise ValueError("compiled V2-04D world is missing")
        instances[scene_id] = (instance, instance_path.resolve(), world_path.resolve())
    schedule = build_schedule(preregistration, instances)
    completed = inventory_completed(schedule, args.validation_root)
    failures = []
    progress_path = args.validation_root / "v2_04d_paired_progress.yaml"
    write_progress(progress_path, args.preregistration, schedule, completed, failures)
    pending = [row for row in schedule
               if (row["method"], row["scene_id"]) not in completed]
    if args.max_new_episodes is not None:
        pending = pending[:args.max_new_episodes]
    print("V2-04D resume: {} valid evidence, {} pending selected".format(
        len(completed), len(pending)), flush=True)
    if args.dry_run:
        for row in pending:
            print("{:03d} {} {}".format(row["sequence"], row["method"], row["scene_id"]))
        return 0
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    consecutive_failures = 0
    for row in pending:
        identity = (row["method"], row["scene_id"])
        print("START {:03d}/{:03d} {} x {}".format(
            row["sequence"], len(schedule), *identity), flush=True)
        last_error = None
        for attempt in range(1, args.attempts_per_episode + 1):
            try:
                evaluation_path, evaluation = run_episode(row, args, instances, environment)
                completed[identity] = (evaluation_path, evaluation)
                consecutive_failures = 0
                print("DONE {:03d}/{:03d} {} success={} time={:.3f}s".format(
                    row["sequence"], len(schedule), evaluation["termination"],
                    evaluation["metrics"]["common"]["success"],
                    evaluation["metrics"]["common"]["navigation_time_s"]), flush=True)
                break
            except Exception as exc:
                last_error = "{}: {}".format(type(exc).__name__, exc)
                print("RETRY {:03d}/{:03d} attempt {}/{} {}".format(
                    row["sequence"], len(schedule), attempt,
                    args.attempts_per_episode, last_error), flush=True)
        else:
            consecutive_failures += 1
            failures.append({**row, "reason": last_error})
            print("INTERFACE_FAILURE {:03d}/{:03d} {}".format(
                row["sequence"], len(schedule), last_error), flush=True)
        write_progress(progress_path, args.preregistration, schedule, completed, failures)
        if consecutive_failures >= 3:
            print("STOP three consecutive interface failures", flush=True)
            return 2
    print("BATCH_DONE valid={}/{} failures={}".format(
        len(completed), len(schedule), len(failures)), flush=True)
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
