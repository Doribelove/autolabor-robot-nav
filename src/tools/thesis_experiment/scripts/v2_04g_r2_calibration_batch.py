#!/usr/bin/env python3
"""Run/resume preregistered V2-04G-R2 calibration-only mechanisms."""

import argparse
import copy
import importlib.util
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
R1_BATCH = Path(__file__).with_name("v2_04g_r1_calibration_batch.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r1_frozen_batch", R1_BATCH)
_R1 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R1)


def _deep_update(target, patch):
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _deep_update(target[key], value)
        else:
            target[key] = copy.deepcopy(value)


def materialize_candidates(candidate_path, output_dir):
    bank = yaml.safe_load(Path(candidate_path).read_text(encoding="utf-8"))
    if not (
        bank.get("stage") == "V2-04G-R2"
        and bank.get("training_allowed") is False
        and bank.get("runtime_ready") is False
    ):
        raise ValueError("R2 candidate bank safety boundary drifted")
    supervisor_source = WORKSPACE / bank["shared_supervisor"]["path"]
    anchor_source = WORKSPACE / bank["shared_anchor_bank"]["path"]
    if _R1._sha256(supervisor_source) != bank["shared_supervisor"]["sha256"]:
        raise ValueError("R2 supervisor source hash drifted")
    if _R1._sha256(anchor_source) != bank["shared_anchor_bank"]["sha256"]:
        raise ValueError("R2 Anchor Bank source hash drifted")
    join = bank["frozen_join_dependency"]
    if _R1._sha256(WORKSPACE / join["source_path"]) != join["source_sha256"]:
        raise ValueError("R1 bounded join source drifted")
    if _R1._sha256(WORKSPACE / join["r1_node_path"]) != join["r1_node_sha256"]:
        raise ValueError("R1 typed join node drifted")
    execution = bank["execution_repair_dependency"]
    if _R1._sha256(WORKSPACE / execution["source_path"]) != execution["source_sha256"]:
        raise ValueError("R2 idempotent execution source drifted")
    if _R1._sha256(WORKSPACE / execution["r2_node_path"]) != execution["r2_node_sha256"]:
        raise ValueError("R2 typed transaction node drifted")
    design = bank["design_probe_boundary"]
    if _R1._sha256(WORKSPACE / design["path"]) != design["sha256"]:
        raise ValueError("R2 excluded design-probe evidence drifted")
    if design.get("eligible_for_r2_ranking") is not False:
        raise ValueError("R2 design-probe ranking firewall drifted")
    base_supervisor = yaml.safe_load(supervisor_source.read_text(encoding="utf-8"))
    base_anchor = yaml.safe_load(anchor_source.read_text(encoding="utf-8"))
    paths = {}
    for candidate in bank["candidates"]:
        candidate_id = candidate["candidate_id"]
        candidate_dir = Path(output_dir) / candidate_id
        supervisor = copy.deepcopy(base_supervisor)
        anchor = copy.deepcopy(base_anchor)
        supervisor["status"] = "calibration_candidate"
        supervisor["profile_id"] = "fam_teb_v2_04g_r2_{}_supervisor".format(
            candidate_id
        )
        anchor["status"] = "uncalibrated_simulation_candidate"
        anchor["bank_id"] = "fam_teb_v2_04g_r2_{}_anchor_candidate".format(
            candidate_id
        )
        anchor["source_provenance"]["mode_deltas"] = (
            "v2_04g_r2_mechanism_repair_candidate"
        )
        _deep_update(supervisor, candidate["supervisor_patch"])
        _deep_update(anchor, candidate["anchor_patch"])
        supervisor_path = candidate_dir / "supervisor.yaml"
        anchor_path = candidate_dir / "anchor_bank.yaml"
        _R1._write_yaml(supervisor_path, supervisor)
        _R1._write_yaml(anchor_path, anchor)
        mechanism = copy.deepcopy(candidate["mechanism"])
        mechanism["profile_id"] = "fam_teb_v2_04g_r2_{}_mechanism".format(
            candidate_id
        )
        mechanism_path = candidate_dir / "mechanism.yaml"
        _R1._write_yaml(mechanism_path, mechanism)
        paths[candidate_id] = {
            "supervisor": supervisor_path.resolve(),
            "anchor_bank": anchor_path.resolve(),
            "mechanism": str(mechanism_path.resolve()),
        }
    return paths


def build_schedule(preregistration, instances, runtime_configs):
    if preregistration["stage"] != "V2-04G-R2":
        raise ValueError("R2 schedule stage drifted")
    if set(preregistration["scene_ids"]) != set(instances):
        raise ValueError("R2 scene set drifted")
    rows = []
    frozen_supervisor = WORKSPACE / preregistration["resources"]["frozen_supervisor"]["path"]
    frozen_anchor = WORKSPACE / preregistration["resources"]["frozen_anchor_bank"]["path"]
    for scene_id in preregistration["scene_ids"]:
        scene = instances[scene_id][0]["scene"]
        rows.append({
            "sequence": len(rows) + 1, "stage": "V2-04G-R2",
            "split": "calibration", "method": "fixed_teb",
            "profile_id": "fixed_reference", "runtime_config": str(frozen_supervisor),
            "anchor_bank": str(frozen_anchor), "mechanism_config": "",
            "scene_id": scene_id, "family": scene["family"], "seed": scene["seed"],
        })
    for candidate_id in preregistration["candidate_ids"]:
        runtime = runtime_configs[candidate_id]
        for scene_id in preregistration["scene_ids"]:
            scene = instances[scene_id][0]["scene"]
            rows.append({
                "sequence": len(rows) + 1, "stage": "V2-04G-R2",
                "split": "calibration", "method": "rule_multi_anchor",
                "profile_id": candidate_id,
                "runtime_config": str(runtime["supervisor"]),
                "anchor_bank": str(runtime["anchor_bank"]),
                "mechanism_config": runtime["mechanism"],
                "scene_id": scene_id, "family": scene["family"], "seed": scene["seed"],
            })
    if len(rows) != preregistration["budget"]["planned_navigation_episode_count"]:
        raise ValueError("R2 navigation budget drifted")
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
    launch_args = _R1.METHOD_LAUNCH[row["method"]]
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
        "--stage", "V2-04G-R2", "--split", "calibration",
        "--profile-id", row["profile_id"],
    ]
    launch = None
    with (output / "launch.log").open("w", encoding="utf-8") as launch_log:
        try:
            launch = subprocess.Popen(
                launch_command, env=environment, stdout=launch_log,
                stderr=subprocess.STDOUT, start_new_session=True,
            )
            _R1._ready(launch, environment, row["method"])
            with (output / "runner.log").open("w", encoding="utf-8") as runner_log:
                result = subprocess.run(
                    runner_command, env=environment, stdout=runner_log,
                    stderr=subprocess.STDOUT,
                    timeout=float(scene["timeout_s"]) + 75.0, check=False,
                )
            if result.returncode != 0:
                raise RuntimeError("R2 episode runner exited {}".format(result.returncode))
        finally:
            if launch is not None:
                _R1._terminate_group(launch)
            time.sleep(1.0)
    path = output / "evaluation.yaml"
    return path, _R1._validate_completed(row, path)


def _verify_resources(preregistration):
    for name, resource in preregistration["resources"].items():
        path = WORKSPACE / resource["path"]
        if not path.is_file() or _R1._sha256(path) != resource["sha256"]:
            raise ValueError("R2 resource hash drifted: {}".format(name))


def _require_activation(preregistration, preregistration_path):
    probe = preregistration["activation_readiness_probe"]
    summary_path = WORKSPACE / probe["summary_path"]
    if not summary_path.is_file():
        raise RuntimeError("R2 activation readiness summary is missing")
    summary = yaml.safe_load(summary_path.read_text(encoding="utf-8"))
    expected = probe["planned_probe_count"]
    if not (
        summary.get("stage") == "V2-04G-R2"
        and summary.get("status") == "complete"
        and summary.get("valid_probe_count") == expected
        and summary.get("all_probe_hard_gates_pass") is True
        and summary.get("navigation_authorized") is True
        and summary.get("preregistration", {}).get("sha256")
        == _R1._sha256(preregistration_path)
    ):
        raise RuntimeError("R2 activation readiness gate failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preregistration", type=Path, required=True)
    parser.add_argument("--compiled-scenes-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--candidate-bank", type=Path, required=True)
    parser.add_argument("--episode-runner", type=Path, default=WORKSPACE /
        "src/tools/thesis_experiment/scripts/v2_04g_r2_mechanism_episode.py")
    parser.add_argument("--max-new-episodes", type=int, default=None)
    parser.add_argument("--attempts-per-episode", type=int, default=2)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    if not (
        prereg.get("stage") == "V2-04G-R2"
        and prereg.get("split") == "calibration"
        and prereg.get("training_allowed") is False
    ):
        raise ValueError("R2 batch refuses non-calibration input")
    _verify_resources(prereg)
    _require_activation(prereg, args.preregistration)
    if args.attempts_per_episode != prereg["budget"]["attempts_per_navigation_episode_max"]:
        raise ValueError("R2 attempt budget drifted")
    args.output_root = args.output_root.resolve()
    args.output_root.relative_to(
        (WORKSPACE / "artifacts/v2/calibration/v2_04g_r2").resolve()
    )
    expected_bank = WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    if args.candidate_bank.resolve() != expected_bank.resolve():
        raise ValueError("R2 candidate bank path drifted")
    expected_index = WORKSPACE / prereg["resources"]["compiled_scene_index"]["path"]
    if (args.compiled_scenes_dir / "compiled_scene_index.yaml").resolve() != expected_index.resolve():
        raise ValueError("R2 compiled scene directory drifted")
    runtime = materialize_candidates(
        args.candidate_bank, args.output_root / "runtime_candidate_configs"
    )
    instances = _R1._load_instances(args.compiled_scenes_dir)
    schedule = build_schedule(prereg, instances, runtime)
    completed = _R1.inventory_completed(schedule, args.output_root)
    failures = []
    progress_path = args.output_root / "v2_04g_r2_progress.yaml"
    _R1.write_progress(
        progress_path, args.preregistration, schedule, completed, failures
    )
    pending = [row for row in schedule if (
        row["profile_id"], row["method"], row["scene_id"]
    ) not in completed]
    if args.max_new_episodes is not None:
        pending = pending[:args.max_new_episodes]
    print("V2-04G-R2 resume: {} valid evidence, {} pending selected".format(
        len(completed), len(pending)
    ), flush=True)
    if args.dry_run:
        for row in pending:
            print("{:03d} {} {}".format(
                row["sequence"], row["profile_id"], row["scene_id"]
            ))
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
            _R1.write_progress(
                progress_path, args.preregistration, schedule, completed, failures
            )
            raise RuntimeError("persistent R2 interface failure")
        _R1.write_progress(
            progress_path, args.preregistration, schedule, completed, failures
        )
    print("V2-04G-R2 complete: {}/{} valid evidence".format(
        len(completed), len(schedule)
    ))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
