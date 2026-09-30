#!/usr/bin/env python3
"""Execute one fresh, non-resumable V3-19 qualification or formal matrix."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml


W = Path("/home/robot/robot_ws_base_rl")
FOUNDATION = W / "artifacts/v3/thesis/v3_19_a0_benchmark_foundation"
RELEASE = FOUNDATION / "execution_release.yaml"
AUTHORIZATION = FOUNDATION / "simulation_authorization.yaml"
PLANNER = W / "artifacts/v3/planner/v3_08_a0/v3_08_a0_effective_planner_contract.yaml"
MODEL = W / "artifacts/v3/prediction/v3_07_b0_r1/model/v3_07_b0_r1_spatial_candidate_predictor.yaml"
ABLATION = FOUNDATION / "v3_19_no_candidate_conditioning_model.yaml"
BASELINE_PLANNER = W / "artifacts/v3/planner/v3_03_c5/v3_03_c5_effective_planner_contract.yaml"
BASELINE_MODEL = W / "artifacts/v3/prediction/v3_02_d1/model/v3_02_d1_candidate_predictor.yaml"
OLD_RUNNER_PATH = W / "src/tools/thesis_experiment/scripts/run_v3_11_actual_teb_matrix.py"
SPEC = importlib.util.spec_from_file_location("v3_19_frozen_v3_11_runner", str(OLD_RUNNER_PATH))
OLD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(OLD)


def sha(data):
    return hashlib.sha256(data).hexdigest()


def read(path):
    return path.read_bytes()


def yaml_bytes(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def atomic_yaml(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        raise RuntimeError("output exists: {}".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(yaml_bytes(value))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def append(path, value):
    payload = (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")
    with path.open("ab", buffering=0) as stream:
        stream.write(payload)
        os.fsync(stream.fileno())


def stop(process):
    if process.poll() is not None:
        return process.returncode
    try:
        os.killpg(process.pid, signal.SIGINT)
        return process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        return process.wait(timeout=5.0)


def validate_release(phase):
    release = yaml.safe_load(read(RELEASE))
    authorization = yaml.safe_load(read(AUTHORIZATION))
    if release["status"] != "v3_19_a0_execution_release_reviewed" or authorization["execution_authorized"] is not True:
        raise RuntimeError("V3-19 release or authorization is closed")
    if phase == "formal":
        qualification = FOUNDATION.parent / "v3_19_q1_execution/matrix_summary.yaml"
        if not qualification.is_file() or yaml.safe_load(read(qualification))["qualification_pass"] is not True:
            raise RuntimeError("formal execution requires the frozen Q1 pass")
    for item in release["runtime_bindings"]:
        path = W / item["path"]
        if not path.is_file() or sha(read(path)) != item["sha256"]:
            raise RuntimeError("release binding differs: {}".format(item["path"]))
    schedule = FOUNDATION / "{}_exact_schedule.yaml".format(phase)
    if sha(read(schedule)) != release["schedule_sha256"][phase]:
        raise RuntimeError("{} schedule differs".format(phase))
    return release, yaml.safe_load(read(schedule))


def run_episode(row, scene, root, port):
    root.mkdir(parents=True, exist_ok=False)
    world_path = root / "world.sdf"
    report_path = root / "episode_report.yaml"
    log_path = root / "roslaunch.log"
    runtime_root = root / "runtime"
    runtime_root.mkdir()
    ros_home = runtime_root / "ros_home"
    ros_log = runtime_root / "roslog"
    ros_home.mkdir()
    ros_log.mkdir()
    world, actor = OLD.render_world(scene, row["actor_policy"], row["seed"])
    world_path.write_text(world, encoding="utf-8")
    environment = dict(os.environ)
    environment.update({
        "ROS_HOME": str(ros_home.resolve()),
        "ROS_LOG_DIR": str(ros_log.resolve()),
        "ROS_MASTER_URI": "http://127.0.0.1:{}".format(port),
        "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(port + 1),
        "GAZEBO_MODEL_DATABASE_URI": "",
    })
    command = [
        "roslaunch", "m2_gazebo", "m2_v3_19_thesis_benchmark_episode.launch",
        "world:={}".format(world_path.resolve()), "method:={}".format(row["method"]),
        "scene_id:={}".format(row["scene_id"]), "seed:={}".format(row["seed"]),
        "actor_policy:={}".format(row["actor_policy"]),
        "actor_angle_deg:={}".format(scene["actor_angle_deg"]),
        "report_path:={}".format(report_path.resolve()),
        "planner_contract_path:={}".format(PLANNER.resolve()),
        "predictor_model_path:={}".format(MODEL.resolve()),
        "ablation_model_path:={}".format(ABLATION.resolve()),
        "baseline_planner_path:={}".format(BASELINE_PLANNER.resolve()),
        "baseline_model_path:={}".format(BASELINE_MODEL.resolve()),
        "lookahead_m:=0.700000", "gui:=false",
    ]
    started = time.monotonic()
    with log_path.open("wb") as log_stream:
        process = subprocess.Popen(
            command, cwd=str(W), env=environment, stdin=subprocess.DEVNULL,
            stdout=log_stream, stderr=subprocess.STDOUT, start_new_session=True,
        )
        timed_out = False
        try:
            return_code = process.wait(timeout=150.0)
        except subprocess.TimeoutExpired:
            timed_out = True
            return_code = stop(process)
        else:
            stop(process)
    if not report_path.is_file():
        raise RuntimeError("episode report absent rc={} timed_out={} tail={}".format(
            return_code, timed_out, log_path.read_text(errors="replace")[-6000:]
        ))
    report = yaml.safe_load(read(report_path))
    if report.get("stage") != "V3-19" or report.get("method") != row["method"]:
        raise RuntimeError("episode identity differs")
    if report.get("hardware_accessed") or report.get("real_vehicle_used"):
        raise RuntimeError("hardware boundary violated")
    return {
        "row": row, "actor_generated_parameters": actor, "episode_report": report,
        "episode_report_sha256": sha(read(report_path)), "world_sha256": sha(read(world_path)),
        "roslaunch_log_sha256": sha(read(log_path)), "roslaunch_return_code": return_code,
        "wall_elapsed_s": time.monotonic() - started, "timed_out": timed_out,
    }


def summarize(receipts, phase, release_sha):
    reports = [receipt["episode_report"] for receipt in receipts]
    methods = ["v3_spatial_safe", "actual_teb", "fixed_v120_shielded", "v3_no_candidate_conditioning"]
    by_method = {}
    for method in methods:
        rows = [report for report in reports if report["method"] == method]
        by_method[method] = {
            "episodes": len(rows),
            "execution_integrity_passes": sum(bool(row["execution_integrity_pass"]) for row in rows),
            "goal_successes": sum(bool(row["result"]["goal_reached"]) for row in rows),
            "mean_restricted_time_s": sum(float(row["result"]["restricted_elapsed_s"]) for row in rows) / len(rows),
            "minimum_clearance_m": min(float(row["result"]["minimum_conservative_clearance_m"]) for row in rows),
            "contact_count": sum(int(row["result"]["contact_state_count"]) for row in rows),
            "fallback_count": sum(bool((row.get("method_plan") or {}).get("safety_fallback_used")) for row in rows),
        }
    blocks = {}
    for receipt in receipts:
        row = receipt["row"]
        blocks.setdefault(row["block_index"], {})[row["method"]] = receipt["episode_report"]
    ranges = []
    for block_index, block in sorted(blocks.items()):
        tracks = [block[method]["warmup"]["gate_actor_track"] for method in methods]
        ranges.append({
            "block_index": block_index,
            "coordinate_range_m": max(track["coordinate_m"] for track in tracks) - min(track["coordinate_m"] for track in tracks),
            "speed_range_mps": max(track["coordinate_speed_mps"] for track in tracks) - min(track["coordinate_speed_mps"] for track in tracks),
        })
    gates = yaml.safe_load((W / "config/thesis_experiments/v3/v3_19_a0_four_method_benchmark_contract.yaml").read_bytes())["qualification_gates"]
    qualification_checks = {
        "episode_count": len(receipts) == gates["exact_episode_count"],
        "attempt1_complete": all(receipt["row"]["attempt"] == 1 for receipt in receipts),
        "execution_integrity": all(report["execution_integrity_pass"] for report in reports),
        "gate_physical_valid": all(report["gate_physical_valid"] for report in reports),
        "actor_mode_match": all(report["warmup"]["actor_mode_matches_evaluator_policy"] for report in reports),
        "gate_actor_coordinate_range": max(item["coordinate_range_m"] for item in ranges) <= gates["maximum_within_block_gate_actor_coordinate_range_m"],
        "gate_actor_speed_range": max(item["speed_range_mps"] for item in ranges) <= gates["maximum_within_block_gate_actor_speed_range_mps"],
        "main_goal_success": by_method["v3_spatial_safe"]["goal_successes"] / by_method["v3_spatial_safe"]["episodes"] >= gates["main_goal_success_rate_min"],
        "actual_teb_goal_success": by_method["actual_teb"]["goal_successes"] / by_method["actual_teb"]["episodes"] >= gates["actual_teb_goal_success_rate_min"],
        "zero_contact": sum(value["contact_count"] for value in by_method.values()) <= gates["contact_count_max"],
        "nonnegative_clearance": min(value["minimum_clearance_m"] for value in by_method.values()) >= gates["minimum_conservative_clearance_m"],
        "non_teb_selected_accepted": all((report.get("method_plan") or {}).get("selected_evaluation", {}).get("accepted") is True for report in reports if report["method"] != "actual_teb"),
    } if phase == "qualification" else {}
    return {
        "schema_version": "1.0", "stage": "V3-19-{}".format("Q1" if phase == "qualification" else "F1"),
        "phase": phase, "status": "execution_complete", "release_sha256": release_sha,
        "simulation_only": True, "episode_count": len(receipts), "method_summary": by_method,
        "gate_actor_ranges": ranges,
        "maximum_gate_actor_coordinate_range_m": max(item["coordinate_range_m"] for item in ranges),
        "maximum_gate_actor_speed_range_mps": max(item["speed_range_mps"] for item in ranges),
        "qualification_checks": qualification_checks,
        "qualification_pass": all(qualification_checks.values()) if phase == "qualification" else None,
        "training_steps": 0, "hardware_accessed": False, "real_vehicle_used": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--phase", choices=("qualification", "formal"), required=True)
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    args = parser.parse_args()
    release, schedule = validate_release(args.phase)
    root = Path(args.output_root).resolve()
    if root.exists():
        raise RuntimeError("output root exists; retry/resume is forbidden")
    root.mkdir(parents=True)
    exact = root / "exact_schedule.yaml"
    exact.write_bytes(read(FOUNDATION / "{}_exact_schedule.yaml".format(args.phase)))
    journal = root / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    release_sha = sha(read(RELEASE))
    append(journal, {"event": "execution_opened", "phase": args.phase, "release_sha256": release_sha, "authorized_units": len(schedule["rows"])})
    scenes = {item["scene_id"]: item for item in yaml.safe_load((W / "config/thesis_experiments/v3/v3_19_a0_four_method_benchmark_contract.yaml").read_bytes())["scene_population"]}
    receipts = []
    port_base = 30000 if args.phase == "qualification" else 34000
    for row in schedule["rows"]:
        episode_root = root / "episodes" / "{:03d}_{}_{}_{}_{}".format(row["sequence"], row["seed"], row["scene_id"], row["actor_policy"], row["method"])
        append(journal, {"event": "unit_consumed", "sequence": row["sequence"], "row": row})
        try:
            receipt = run_episode(row, scenes[row["scene_id"]], episode_root, port_base + 2 * row["sequence"])
        except Exception as error:
            append(journal, {"event": "terminal_failure", "sequence": row["sequence"], "row": row, "error": "{}:{}".format(type(error).__name__, error), "remaining_units_forfeited": len(schedule["rows"]) - row["sequence"]})
            raise
        receipt_path = episode_root / "matrix_receipt.yaml"
        atomic_yaml(receipt_path, receipt)
        receipts.append(receipt)
        report = receipt["episode_report"]
        append(journal, {"event": "unit_complete", "sequence": row["sequence"], "matrix_receipt_sha256": sha(read(receipt_path)), "integrity": report["execution_integrity_pass"], "goal": report["result"]["goal_reached"], "restricted_time_s": report["result"]["restricted_elapsed_s"]})
        print("[V3-19-{}] {}/{} {} integrity={} goal={} time={:.3f}s clearance={:.3f}m".format(args.phase.upper(), row["sequence"], len(schedule["rows"]), row["method"], report["execution_integrity_pass"], report["result"]["goal_reached"], report["result"]["restricted_elapsed_s"], report["result"]["minimum_conservative_clearance_m"]), flush=True)
    summary = summarize(receipts, args.phase, release_sha)
    atomic_yaml(root / "matrix_summary.yaml", summary)
    append(journal, {"event": "execution_complete", "completed_units": len(receipts), "forfeited_units": 0, "summary_sha256": sha(read(root / "matrix_summary.yaml"))})
    return 0


if __name__ == "__main__":
    sys.exit(main())

