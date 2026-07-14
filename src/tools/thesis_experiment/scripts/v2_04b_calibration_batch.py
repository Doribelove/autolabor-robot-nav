#!/usr/bin/env python3
"""Resume and execute the preregistered V2-04B Gazebo calibration matrix."""

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


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def build_schedule(plan):
    """Put all center feasibility episodes before one-factor probes."""

    rows = []
    candidates = sorted(
        plan["candidates"],
        key=lambda row: (0 if row["screen_level"] == 0 else 1,
                         plan["candidates"].index(row)),
    )
    for candidate in candidates:
        for evaluation in candidate["evaluations"]:
            if evaluation["split"] != "calibration":
                raise ValueError("batch schedule refuses a non-calibration evaluation")
            rows.append({
                "sequence": len(rows) + 1,
                "candidate": candidate,
                "evaluation": evaluation,
            })
    identities = {
        (row["candidate"]["candidate_id"], row["evaluation"]["scene_id"])
        for row in rows
    }
    if len(rows) != plan["planned_episode_count"] or len(identities) != len(rows):
        raise ValueError("calibration schedule count or identity drifted")
    return rows


def _validate_completed(row, evaluation_path):
    evaluation_path = Path(evaluation_path)
    trace_path = evaluation_path.parent / "trace.csv"
    if not trace_path.is_file():
        raise ValueError("completed evaluation is missing trace.csv")
    evaluation = yaml.safe_load(evaluation_path.read_text(encoding="utf-8"))
    candidate = row["candidate"]
    expected = row["evaluation"]
    checks = {
        "scene_id": expected["scene_id"],
        "split": "calibration",
        "candidate_id": candidate["candidate_id"],
        "candidate_profile_sha256": candidate["profile_sha256"],
        "training_used": False,
        "runtime_policy_manifest_access": False,
        "experiment_manager_calibration_manifest_access": True,
        "typed_startup_snapshot_restored": True,
    }
    for key, value in checks.items():
        if evaluation.get(key) != value:
            raise ValueError("completed evaluation {} drifted".format(key))
    # The initial Cruise pilot predates these two redundant provenance fields;
    # its candidate/profile/scene hashes remain immutable and are checked above.
    if "dynamic_overlay" in evaluation:
        if evaluation["dynamic_overlay"] != expected["dynamic_overlay"]:
            raise ValueError("completed evaluation overlay drifted")
        if evaluation.get("effective_profile_sha256") != expected["effective_profile_sha256"]:
            raise ValueError("completed evaluation effective profile drifted")
    rows = load_v2_trace(trace_path)
    if trace_sha256(trace_path) != evaluation["raw_trace_sha256"]:
        raise ValueError("completed raw trace hash drifted")
    if len(rows) < 2:
        raise ValueError("completed raw trace is empty")
    return evaluation


def _hard_gate_pass(evaluation, clearance_min):
    common = evaluation["metrics"]["common"]
    return bool(
        common["success"]
        and not common["collision"]
        and common["minimum_clearance_m"] >= clearance_min
        and not evaluation.get("runner_fault_reason", "")
        and evaluation.get("typed_startup_snapshot_restored") is True
    )


def inventory_completed(schedule, calibration_root):
    by_identity = {
        (row["candidate"]["candidate_id"], row["evaluation"]["scene_id"]): row
        for row in schedule
    }
    completed = {}
    for evaluation_path in sorted(Path(calibration_root).glob("**/evaluation.yaml")):
        raw = yaml.safe_load(evaluation_path.read_text(encoding="utf-8"))
        identity = (raw.get("candidate_id"), raw.get("scene_id"))
        if identity not in by_identity:
            continue
        if identity in completed:
            raise ValueError("duplicate completed calibration identity {}".format(identity))
        evaluation = _validate_completed(by_identity[identity], evaluation_path)
        completed[identity] = (evaluation_path, evaluation)
    return completed


def _episode_output(root, row):
    return Path(root) / "episodes" / (
        "ep_{:03d}__{}__{}".format(
            row["sequence"], row["candidate"]["candidate_id"],
            row["evaluation"]["scene_id"],
        )
    )


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


def _ready(launch, environment, timeout_s=35.0):
    commands = (
        ["rosparam", "get", "/m2_gazebo/simulation_only"],
        ["rosservice", "info", "/move_base/TebLocalPlannerROS/set_parameters"],
        ["rostopic", "echo", "-n", "1", "/clock"],
    )
    deadline = time.monotonic() + timeout_s
    for command in commands:
        while time.monotonic() < deadline:
            if launch.poll() is not None:
                raise RuntimeError("roslaunch exited before calibration readiness")
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
            raise RuntimeError("calibration readiness timeout: {}".format(command[-1]))


def _write_yaml(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def write_progress(path, plan_path, schedule, completed, failures, clearance_min, stage):
    rows = []
    hard_gate_count = 0
    success_count = 0
    for row in schedule:
        identity = (row["candidate"]["candidate_id"], row["evaluation"]["scene_id"])
        if identity not in completed:
            continue
        evaluation_path, evaluation = completed[identity]
        hard_gate = _hard_gate_pass(evaluation, clearance_min)
        hard_gate_count += int(hard_gate)
        success_count += int(evaluation["metrics"]["common"]["success"])
        rows.append({
            "sequence": row["sequence"],
            "candidate_id": identity[0],
            "anchor_id": row["candidate"]["anchor_id"],
            "scene_id": identity[1],
            "family": row["evaluation"]["family"],
            "evaluation": str(evaluation_path),
            "evaluation_sha256": _sha256(evaluation_path),
            "termination": evaluation["termination"],
            "success": evaluation["metrics"]["common"]["success"],
            "hard_gate_pass": hard_gate,
        })
    valid_count = len(rows)
    _write_yaml(path, {
        "schema_version": "2.0",
        "stage": stage,
        "status": "calibration_complete" if valid_count == len(schedule)
        else "calibration_in_progress",
        "formal_result": False,
        "simulation_only": True,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "candidate_plan": {"path": str(plan_path), "sha256": _sha256(plan_path)},
        "planned_navigation_episode_count": len(schedule),
        "valid_evidence_episode_count": valid_count,
        "successful_episode_count": success_count,
        "hard_gate_pass_episode_count": hard_gate_count,
        "interface_failure_count": len(failures),
        "freeze_minimum_episode_gate_reached": valid_count >= 30,
        "anchor_values_frozen": False,
        "performance_improvement_observed": False,
        "episodes": rows,
        "interface_failures": list(failures),
    })


def run_episode(row, args, instance_by_scene, world_by_scene, environment):
    candidate = row["candidate"]
    expected = row["evaluation"]
    output = _episode_output(args.calibration_root, row)
    output.mkdir(parents=True, exist_ok=True)
    instance_path = instance_by_scene[expected["scene_id"]]
    instance = yaml.safe_load(instance_path.read_text(encoding="utf-8"))
    scene = instance["scene"]
    timeout_override_used = bool(
        scene["family"] == "DYNAMIC"
        and args.dynamic_timeout_override_s is not None
    )
    episode_timeout_s = (
        args.dynamic_timeout_override_s
        if timeout_override_used else float(scene["timeout_s"])
    )
    launch_command = [
        "roslaunch", "m2_gazebo", "m2_v2_typed_teb.launch",
        "world:={}".format(world_by_scene[expected["scene_id"]]),
        "seed:={}".format(scene["seed"]),
        "x:={}".format(scene["start"]["x_m"]),
        "y:={}".format(scene["start"]["y_m"]),
        "yaw:={}".format(scene["start"]["yaw_rad"]),
        "gui:=false",
    ]
    runner_command = [
        sys.executable, str(args.episode_runner),
        "--instance", str(instance_path),
        "--candidate-plan", str(args.candidate_plan),
        "--candidate-id", candidate["candidate_id"],
        "--anchor-bank", str(args.anchor_bank),
        "--output-dir", str(output),
        "--timeout-s", str(episode_timeout_s),
    ]
    if timeout_override_used:
        runner_command.append("--allow-timeout-override")
    launch = None
    with (output / "launch.log").open("w", encoding="utf-8") as launch_log:
        try:
            launch = subprocess.Popen(
                launch_command, env=environment, stdout=launch_log,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            _ready(launch, environment)
            with (output / "runner.log").open("w", encoding="utf-8") as runner_log:
                result = subprocess.run(
                    runner_command, env=environment, stdout=runner_log,
                    stderr=subprocess.STDOUT,
                    timeout=float(episode_timeout_s) + 75.0, check=False,
                )
            if result.returncode != 0:
                raise RuntimeError("episode runner exited {}".format(result.returncode))
        finally:
            if launch is not None:
                _terminate_group(launch)
            time.sleep(1.0)
    evaluation_path = output / "evaluation.yaml"
    evaluation = _validate_completed(row, evaluation_path)
    return evaluation_path, evaluation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=WORKSPACE)
    parser.add_argument("--candidate-plan", type=Path)
    parser.add_argument("--anchor-bank", type=Path)
    parser.add_argument("--calibration-root", type=Path)
    parser.add_argument("--episode-runner", type=Path)
    parser.add_argument("--max-new-episodes", type=int, default=None)
    parser.add_argument("--centers-only", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--attempts-per-episode", type=int, default=2)
    parser.add_argument("--progress-name", default=None)
    parser.add_argument("--compiled-scenes-dir", type=Path, default=None)
    parser.add_argument("--dynamic-timeout-override-s", type=float, default=None)
    args = parser.parse_args()
    args.workspace = args.workspace.resolve()
    args.candidate_plan = (args.candidate_plan or args.workspace /
        "artifacts/v2/calibration/v2_04b_anchor_screen_plan.yaml").resolve()
    args.anchor_bank = (args.anchor_bank or args.workspace /
        "src/application/teb_mode_manager/config/v2_04_anchor_bank_candidate.yaml").resolve()
    args.calibration_root = (args.calibration_root or args.workspace /
        "artifacts/v2/calibration").resolve()
    args.episode_runner = (args.episode_runner or args.workspace /
        "src/tools/thesis_experiment/scripts/v2_04b_calibration_episode.py").resolve()
    args.calibration_root.relative_to((args.workspace / "artifacts/v2/calibration").resolve())
    if args.max_new_episodes is not None and args.max_new_episodes < 0:
        raise ValueError("max-new-episodes must be non-negative")
    if args.attempts_per_episode < 1:
        raise ValueError("attempts-per-episode must be positive")
    if (
        args.dynamic_timeout_override_s is not None
        and args.dynamic_timeout_override_s <= 0.0
    ):
        raise ValueError("dynamic-timeout-override-s must be positive")

    plan = yaml.safe_load(args.candidate_plan.read_text(encoding="utf-8"))
    if plan.get("simulation_only") is not True or plan.get("training_started") is not False:
        raise ValueError("batch refuses a non-simulation or training-enabled plan")
    schedule = build_schedule(plan)
    compiled = (
        args.compiled_scenes_dir.resolve()
        if args.compiled_scenes_dir is not None
        else args.calibration_root / "compiled_scenes"
    )
    instance_by_scene = {
        yaml.safe_load(path.read_text(encoding="utf-8"))["scene"]["scene_id"]: path.resolve()
        for path in compiled.glob("*.instance.yaml")
    }
    world_by_scene = {
        path.name[:-len(".world")]: path.resolve() for path in compiled.glob("*.world")
    }
    expected_scenes = {row["evaluation"]["scene_id"] for row in schedule}
    if not expected_scenes.issubset(instance_by_scene) or not expected_scenes.issubset(world_by_scene):
        raise ValueError("compiled calibration scene set drifted")

    completed = inventory_completed(schedule, args.calibration_root)
    failures = []
    progress_name = args.progress_name or (
        "v2_04b_batch_progress.yaml" if plan["stage"] == "V2-04B"
        else "{}_batch_progress.yaml".format(plan["stage"].lower().replace("-", "_"))
    )
    progress_path = args.calibration_root / progress_name
    clearance_min = 0.25
    write_progress(
        progress_path, args.candidate_plan, schedule, completed, failures, clearance_min,
        plan["stage"],
    )
    pending = [
        row for row in schedule
        if (row["candidate"]["candidate_id"], row["evaluation"]["scene_id"])
        not in completed
        and (not args.centers_only or row["candidate"]["screen_level"] == 0)
    ]
    if args.max_new_episodes is not None:
        pending = pending[:args.max_new_episodes]
    print("{} resume: {} valid evidence, {} pending selected".format(
        plan["stage"], len(completed), len(pending)), flush=True)
    if args.dry_run:
        for row in pending:
            print("{:03d} {} {}".format(
                row["sequence"], row["candidate"]["candidate_id"],
                row["evaluation"]["scene_id"]), flush=True)
        return 0

    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    completed_this_run = 0
    consecutive_interface_failures = 0
    for row in pending:
        identity = (row["candidate"]["candidate_id"], row["evaluation"]["scene_id"])
        print("START {:03d}/{:03d} {} x {}".format(
            row["sequence"], len(schedule), identity[0], identity[1]), flush=True)
        last_error = None
        for attempt in range(1, args.attempts_per_episode + 1):
            try:
                evaluation_path, evaluation = run_episode(
                    row, args, instance_by_scene, world_by_scene, environment
                )
                completed[identity] = (evaluation_path, evaluation)
                completed_this_run += 1
                consecutive_interface_failures = 0
                print("DONE {:03d}/{:03d} {} hard_gate={} time={:.3f}s".format(
                    row["sequence"], len(schedule), evaluation["termination"],
                    _hard_gate_pass(evaluation, clearance_min),
                    evaluation["metrics"]["common"]["navigation_time_s"],
                ), flush=True)
                break
            except Exception as exc:
                last_error = "{}: {}".format(type(exc).__name__, exc)
                print("RETRY {:03d}/{:03d} attempt {}/{} {}".format(
                    row["sequence"], len(schedule), attempt,
                    args.attempts_per_episode, last_error
                ), flush=True)
        else:
            consecutive_interface_failures += 1
            failures.append({
                "sequence": row["sequence"], "candidate_id": identity[0],
                "scene_id": identity[1], "reason": last_error,
            })
            print("INTERFACE_FAILURE {:03d}/{:03d} {}".format(
                row["sequence"], len(schedule), last_error), flush=True)
        write_progress(
            progress_path, args.candidate_plan, schedule, completed, failures,
            clearance_min, plan["stage"],
        )
        if consecutive_interface_failures >= 3:
            print("STOP three consecutive interface failures", flush=True)
            return 2
    print("BATCH_DONE new={} total_valid={}/{} failures={}".format(
        completed_this_run, len(completed), len(schedule), len(failures)), flush=True)
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
