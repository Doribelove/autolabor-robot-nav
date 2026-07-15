#!/usr/bin/env python3
"""Run/resume the preregistered V2-04G-R3 full calibration-only round."""

import argparse
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R3"
R2_BATCH = Path(__file__).with_name("v2_04g_r2_calibration_batch.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r2_frozen_batch_r3", R2_BATCH)
_R2 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R2)
_BASE = _R2._R1


def verify_resources(preregistration):
    for group in ("resources", "frozen_readiness_boundary"):
        for name, resource in preregistration.get(group, {}).items():
            path = WORKSPACE / resource["path"]
            if not path.is_file() or _BASE._sha256(path) != resource["sha256"]:
                raise ValueError("R3 frozen resource drifted: {}.{}".format(group, name))


def require_readiness_and_ttc(preregistration, preregistration_path):
    readiness_path = WORKSPACE / preregistration[
        "activation_readiness_probe"]["summary_path"]
    ttc_path = WORKSPACE / preregistration["ttc_component_probe"]["report_path"]
    if not readiness_path.is_file() or not ttc_path.is_file():
        raise RuntimeError("R3 readiness or TTC prerequisite is missing")
    prereg_hash = _BASE._sha256(preregistration_path)
    readiness = yaml.safe_load(readiness_path.read_text(encoding="utf-8"))
    ttc = yaml.safe_load(ttc_path.read_text(encoding="utf-8"))
    planned = preregistration["budget"]["activation_readiness_probe_count"]
    if not (
        readiness.get("stage") == STAGE and readiness.get("status") == "complete"
        and readiness.get("valid_probe_count") == planned
        and readiness.get("all_probe_hard_gates_pass") is True
        and readiness.get("navigation_authorized") is True
        and readiness.get("preregistration", {}).get("sha256") == prereg_hash
    ):
        raise RuntimeError("R3 readiness gate failed")
    if not (
        ttc.get("stage") == STAGE and ttc.get("status") == "complete"
        and ttc.get("probe_count") == 3 and ttc.get("all_three_states_pass") is True
        and ttc.get("preregistration", {}).get("sha256") == prereg_hash
    ):
        raise RuntimeError("R3 TTC gate failed")


def build_schedule(preregistration, instances, runtime_configs):
    if preregistration.get("stage") != STAGE:
        raise ValueError("R3 schedule stage drifted")
    if set(preregistration["scene_ids"]) != set(instances):
        raise ValueError("R3 scene set drifted")
    rows = []
    frozen_supervisor = WORKSPACE / preregistration["resources"][
        "frozen_supervisor"]["path"]
    frozen_anchor = WORKSPACE / preregistration["resources"][
        "frozen_anchor_bank"]["path"]
    for scene_id in preregistration["scene_ids"]:
        scene = instances[scene_id][0]["scene"]
        rows.append({
            "sequence": len(rows) + 1, "stage": STAGE, "split": "calibration",
            "method": "fixed_teb", "profile_id": "fixed_reference",
            "runtime_config": str(frozen_supervisor), "anchor_bank": str(frozen_anchor),
            "mechanism_config": "", "scene_id": scene_id,
            "family": scene["family"], "seed": scene["seed"],
        })
    for candidate_id in preregistration["candidate_ids"]:
        runtime = runtime_configs[candidate_id]
        for scene_id in preregistration["scene_ids"]:
            scene = instances[scene_id][0]["scene"]
            rows.append({
                "sequence": len(rows) + 1, "stage": STAGE, "split": "calibration",
                "method": "rule_multi_anchor", "profile_id": candidate_id,
                "runtime_config": str(runtime["supervisor"]),
                "anchor_bank": str(runtime["anchor_bank"]),
                "mechanism_config": runtime["mechanism"], "scene_id": scene_id,
                "family": scene["family"], "seed": scene["seed"],
            })
    if len(rows) != preregistration["budget"]["planned_navigation_episode_count"]:
        raise ValueError("R3 navigation budget drifted")
    return rows


def _episode_output(root, row):
    return Path(root) / "episodes" / "ep_{:03d}__{}__{}".format(
        row["sequence"], row["profile_id"], row["scene_id"]
    )


def run_episode(row, args, instances, environment):
    instance, instance_path, world_path = instances[row["scene_id"]]
    scene = instance["scene"]
    output = _episode_output(args.output_root, row)
    output.mkdir(parents=True, exist_ok=True)
    launch_args = _BASE.METHOD_LAUNCH[row["method"]]
    launch_command = [
        "roslaunch", "m2_gazebo", "m2_v2_04g_r2_mechanism_calibration.launch",
        "world:={}".format(world_path), "seed:={}".format(scene["seed"]),
        "x:={}".format(scene["start"]["x_m"]),
        "y:={}".format(scene["start"]["y_m"]),
        "yaw:={}".format(scene["start"]["yaw_rad"]), "gui:=false",
        "rule_supervisor_config:={}".format(row["runtime_config"]),
        "anchor_bank:={}".format(row["anchor_bank"]),
        "mechanism_config:={}".format(row["mechanism_config"]),
    ] + ["{}:={}".format(name, value) for name, value in launch_args.items()]
    runner_command = [
        sys.executable, str(args.episode_runner), "--instance", str(instance_path),
        "--method", row["method"], "--output-dir", str(output),
        "--stage", STAGE, "--split", "calibration",
        "--profile-id", row["profile_id"],
    ]
    launch = None
    with (output / "launch.log").open("w", encoding="utf-8") as launch_log:
        try:
            launch = subprocess.Popen(
                launch_command, env=environment, stdout=launch_log,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            _BASE._ready(launch, environment, row["method"])
            with (output / "runner.log").open("w", encoding="utf-8") as runner_log:
                result = subprocess.run(
                    runner_command, env=environment, stdout=runner_log,
                    stderr=subprocess.STDOUT,
                    timeout=float(scene["timeout_s"]) + 75.0, check=False,
                )
            if result.returncode != 0:
                raise RuntimeError("R3 episode runner exited {}".format(result.returncode))
        finally:
            if launch is not None:
                _BASE._terminate_group(launch)
            time.sleep(1.0)
    path = output / "evaluation.yaml"
    return path, _BASE._validate_completed(row, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preregistration", type=Path, required=True)
    parser.add_argument("--compiled-scenes-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--candidate-bank", type=Path, required=True)
    parser.add_argument("--episode-runner", type=Path, default=WORKSPACE /
        "src/tools/thesis_experiment/scripts/v2_04g_r3_mechanism_episode.py")
    parser.add_argument("--max-new-episodes", type=int, default=None)
    parser.add_argument("--attempts-per-episode", type=int, default=2)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    if not (
        prereg.get("stage") == STAGE and prereg.get("split") == "calibration"
        and prereg.get("training_allowed") is False
    ):
        raise ValueError("R3 batch refuses non-calibration input")
    verify_resources(prereg)
    require_readiness_and_ttc(prereg, args.preregistration)
    if args.attempts_per_episode != prereg["budget"][
        "attempts_per_navigation_episode_max"]:
        raise ValueError("R3 attempt budget drifted")
    args.output_root = args.output_root.resolve()
    args.output_root.relative_to((WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r3").resolve())
    expected_bank = WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    if args.candidate_bank.resolve() != expected_bank.resolve():
        raise ValueError("R3 candidate bank path drifted")
    expected_index = WORKSPACE / prereg["resources"]["compiled_scene_index"]["path"]
    if (args.compiled_scenes_dir / "compiled_scene_index.yaml").resolve() != expected_index.resolve():
        raise ValueError("R3 compiled scene directory drifted")
    runtime = _R2.materialize_candidates(
        args.candidate_bank, args.output_root / "runtime_candidate_configs"
    )
    instances = _BASE._load_instances(args.compiled_scenes_dir)
    schedule = build_schedule(prereg, instances, runtime)
    completed = _BASE.inventory_completed(schedule, args.output_root)
    failures = []
    progress_path = args.output_root / "v2_04g_r3_progress.yaml"
    _BASE.write_progress(
        progress_path, args.preregistration, schedule, completed, failures
    )
    pending = [row for row in schedule if (
        row["profile_id"], row["method"], row["scene_id"]
    ) not in completed]
    if args.max_new_episodes is not None:
        pending = pending[:args.max_new_episodes]
    print("V2-04G-R3 resume: {} valid evidence, {} pending selected".format(
        len(completed), len(pending)), flush=True)
    if args.dry_run:
        for row in pending:
            print("{:03d} {} {}".format(
                row["sequence"], row["profile_id"], row["scene_id"]))
        return 0
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    for row in pending:
        identity = (row["profile_id"], row["method"], row["scene_id"])
        print("START {:03d}/{:03d} {} x {}".format(
            row["sequence"], len(schedule), row["profile_id"], row["scene_id"]
        ), flush=True)
        last_error = None
        for attempt in range(1, args.attempts_per_episode + 1):
            try:
                path, evidence = run_episode(row, args, instances, environment)
                completed[identity] = (path, evidence)
                print("DONE {} success={} switches={} reverse={}".format(
                    identity, evidence["metrics"]["common"]["success"],
                    evidence["active_anchor_switch_count"],
                    evidence.get("mechanism_maneuver_reverse_sample_count", 0),
                ), flush=True)
                last_error = None
                break
            except Exception as exc:
                last_error = "attempt {}: {}".format(attempt, exc)
                print("RETRY {} {}".format(identity, last_error), flush=True)
        if last_error is not None:
            failures.append({"identity": list(identity), "reason": last_error})
            _BASE.write_progress(
                progress_path, args.preregistration, schedule, completed, failures)
            raise RuntimeError("persistent R3 interface failure")
        _BASE.write_progress(
            progress_path, args.preregistration, schedule, completed, failures)
    print("V2-04G-R3 complete: {}/{} valid evidence".format(
        len(completed), len(schedule)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
