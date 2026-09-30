#!/usr/bin/env python3
"""Prepare and execute the frozen V6 sensor-only Gazebo evaluation."""

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

import numpy as np
import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / "src/application/v5_e2e_scope_rl/config/v6_03_gazebo_contract.yaml"
FOUNDATION = W / "artifacts/v6/gazebo/v6_03_a0_foundation"
OLD_RUNNER = W / "src/tools/thesis_experiment/scripts/run_v3_11_actual_teb_matrix.py"
SPEC = importlib.util.spec_from_file_location("v6_frozen_world_renderer", str(OLD_RUNNER))
OLD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(OLD)

BINDINGS = (
    "src/tools/thesis_experiment/scripts/run_v6_03_sensor_gazebo.py",
    "src/application/v5_e2e_scope_rl/config/v6_03_gazebo_contract.yaml",
    "src/application/v5_e2e_scope_rl/scripts/v6_sensor_gazebo_episode.py",
    "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/asymmetric_ppo.py",
    "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/runtime_adapter.py",
    "src/simulation/m2_gazebo/launch/m2_v6_sensor_benchmark_episode.launch",
    "src/simulation/m2_gazebo/launch/spawn_m2.launch",
    "src/application/v3_scope_planner/scripts/v3_scope_actual_teb_fair_episode.py",
    "src/application/v3_scope_planner/src/v3_scope_planner/execution_core.py",
    "src/tools/thesis_experiment/scripts/run_v3_11_actual_teb_matrix.py",
)
METHODS = ("pct_scope_rl_v6", "sensor_analytic_prior", "actual_teb")


def read(path):
    return path.read_bytes()


def sha(data):
    return hashlib.sha256(data).hexdigest()


def dump(path, value, exclusive=True):
    path.parent.mkdir(parents=True, exist_ok=True)
    if exclusive and path.exists():
        raise RuntimeError("output exists: {}".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8"))
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def append(path, value):
    payload = (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode("utf-8")
    with path.open("ab", buffering=0) as stream:
        stream.write(payload)
        os.fsync(stream.fileno())


def make_rows(contract, phase):
    matrix = contract["evaluation"][phase]
    strata = contract["strata"]
    actors = contract["methods"]["frozen_actor_checkpoints"]
    rows = []
    sequence = 0
    for block_zero, seed in enumerate(range(int(matrix["seed_start"]), int(matrix["seed_end"]) + 1)):
        stratum = strata[block_zero % len(strata)]
        actor = actors[block_zero % len(actors)]
        order = METHODS[block_zero % 3 :] + METHODS[: block_zero % 3]
        for method in order:
            sequence += 1
            rows.append({
                "sequence": sequence,
                "block_index": block_zero + 1,
                "attempt": 1,
                "seed": seed,
                "scene_id": stratum["scene_id"],
                "actor_policy": stratum["actor_policy"],
                "method": method,
                "pct_actor_identity": actor["id"],
                "pct_actor_checkpoint_path": actor["path"],
                "pct_actor_checkpoint_sha256": actor["sha256"],
            })
    if len(rows) != int(matrix["episodes"]):
        raise RuntimeError("schedule cardinality differs")
    return rows


def prepare():
    if FOUNDATION.exists():
        raise RuntimeError("foundation already exists")
    contract = yaml.safe_load(read(CONTRACT))
    FOUNDATION.mkdir(parents=True)
    schedule_hashes = {}
    for phase in ("preflight", "qualification", "formal"):
        path = FOUNDATION / "{}_exact_schedule.yaml".format(phase)
        dump(path, {"schema_version": "1.0", "stage": "V6-03", "phase": phase, "rows": make_rows(contract, phase)})
        schedule_hashes[phase] = sha(read(path))
    bindings = []
    for relative in BINDINGS:
        path = W / relative
        if not path.is_file():
            raise RuntimeError("binding absent: {}".format(relative))
        bindings.append({"path": relative, "sha256": sha(read(path))})
    for actor in contract["methods"]["frozen_actor_checkpoints"]:
        path = W / actor["path"]
        observed = sha(read(path))
        if observed != actor["sha256"]:
            raise RuntimeError("actor checkpoint differs: {}".format(actor["id"]))
        bindings.append({"path": actor["path"], "sha256": observed})
    release = {
        "schema_version": "1.0",
        "stage": "V6-03-A0",
        "status": "v6_03_sensor_gazebo_release_frozen",
        "contract_sha256": sha(read(CONTRACT)),
        "schedule_sha256": schedule_hashes,
        "runtime_bindings": bindings,
        "preflight_authorized": True,
        "qualification_requires_preflight_pass": True,
        "formal_requires_qualification_pass": True,
        "training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    dump(FOUNDATION / "execution_release.yaml", release)
    print("release_sha256={}".format(sha(read(FOUNDATION / "execution_release.yaml"))))


def validate_release(phase):
    release_path = FOUNDATION / "execution_release.yaml"
    release = yaml.safe_load(read(release_path))
    if release["status"] != "v6_03_sensor_gazebo_release_frozen":
        raise RuntimeError("release status differs")
    if release["contract_sha256"] != sha(read(CONTRACT)):
        raise RuntimeError("contract changed after release")
    for item in release["runtime_bindings"]:
        path = W / item["path"]
        if not path.is_file() or sha(read(path)) != item["sha256"]:
            raise RuntimeError("runtime binding differs: {}".format(item["path"]))
    schedule_path = FOUNDATION / "{}_exact_schedule.yaml".format(phase)
    if sha(read(schedule_path)) != release["schedule_sha256"][phase]:
        raise RuntimeError("schedule differs")
    if phase in ("qualification", "formal"):
        preflight = W / "artifacts/v6/gazebo/v6_03_p0_preflight/matrix_summary.yaml"
        if not preflight.is_file() or yaml.safe_load(read(preflight))["phase_pass"] is not True:
            raise RuntimeError("qualification requires terminal preflight pass")
    if phase == "formal":
        qualification = W / "artifacts/v6/gazebo/v6_03_q1_qualification/matrix_summary.yaml"
        if not qualification.is_file() or yaml.safe_load(read(qualification))["qualification_pass"] is not True:
            raise RuntimeError("formal requires terminal qualification pass")
    return release, yaml.safe_load(read(schedule_path))


def stop(process):
    if process.poll() is not None:
        return process.returncode
    try:
        os.killpg(process.pid, signal.SIGINT)
        return process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        return process.wait(timeout=5.0)


def run_episode(row, scene, root, port):
    root.mkdir(parents=True, exist_ok=False)
    world_path = root / "world.sdf"
    report_path = root / "episode_report.yaml"
    log_path = root / "roslaunch.log"
    runtime = root / "runtime"
    ros_home = runtime / "ros_home"
    ros_log = runtime / "roslog"
    ros_home.mkdir(parents=True)
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
    checkpoint = W / row["pct_actor_checkpoint_path"] if row["method"] == "pct_scope_rl_v6" else Path("")
    command = [
        "roslaunch", "m2_gazebo", "m2_v6_sensor_benchmark_episode.launch",
        "world:={}".format(world_path.resolve()),
        "method:={}".format(row["method"]),
        "scene_id:={}".format(row["scene_id"]),
        "seed:={}".format(row["seed"]),
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
            return_code = stop(process)
        else:
            stop(process)
    if not report_path.is_file():
        raise RuntimeError("episode report absent rc={} timed_out={} tail={}".format(return_code, timed_out, log_path.read_text(errors="replace")[-6000:]))
    report = yaml.safe_load(read(report_path))
    if report.get("stage") != "V6-03" or report.get("method") != row["method"]:
        raise RuntimeError("episode identity differs")
    if report.get("hardware_accessed") or report.get("real_vehicle_used"):
        raise RuntimeError("hardware boundary violated")
    if row["method"] == "pct_scope_rl_v6" and report["actor_checkpoint"]["sha256"] != row["pct_actor_checkpoint_sha256"]:
        raise RuntimeError("runtime actor checkpoint differs")
    return {
        "row": row,
        "actor_generated_parameters": actor,
        "episode_report": report,
        "episode_report_sha256": sha(read(report_path)),
        "world_sha256": sha(read(world_path)),
        "roslaunch_log_sha256": sha(read(log_path)),
        "roslaunch_return_code": return_code,
        "wall_elapsed_s": time.monotonic() - started,
        "timed_out": timed_out,
    }


def bootstrap_interval(pairs, seed, count=20000):
    values = np.asarray(pairs, dtype=np.float64)
    rng = np.random.default_rng(seed)
    estimates = np.empty(count, dtype=np.float64)
    for start in range(0, count, 1000):
        size = min(1000, count - start)
        indices = rng.integers(0, len(values), size=(size, len(values)))
        sample = values[indices]
        estimates[start : start + size] = 1.0 - sample[:, :, 0].mean(axis=1) / sample[:, :, 1].mean(axis=1)
    return [float(np.quantile(estimates, 0.025)), float(np.quantile(estimates, 0.975))]


def summarize(receipts, phase, release_sha):
    contract = yaml.safe_load(read(CONTRACT))
    reports = [receipt["episode_report"] for receipt in receipts]
    by_method = {}
    for method in METHODS:
        rows = [report for report in reports if report["method"] == method]
        by_method[method] = {
            "episodes": len(rows),
            "execution_integrity_passes": sum(bool(row["execution_integrity_pass"]) for row in rows),
            "goal_successes": sum(bool(row["result"]["goal_reached"]) for row in rows),
            "mean_restricted_time_s": sum(float(row["result"]["restricted_elapsed_s"]) for row in rows) / len(rows),
            "minimum_clearance_m": min(float(row["result"]["minimum_conservative_clearance_m"]) for row in rows),
            "contact_count": sum(int(row["result"]["contact_state_count"]) for row in rows),
            "mean_policy_latency_ms": float(np.mean([row["result"].get("mean_policy_latency_ms") for row in rows if row["result"].get("mean_policy_latency_ms") is not None])) if any(row["result"].get("mean_policy_latency_ms") is not None for row in rows) else None,
        }
    blocks = {}
    for receipt in receipts:
        row = receipt["row"]
        blocks.setdefault(row["block_index"], {})[row["method"]] = receipt["episode_report"]
    gate_ranges = []
    paired = []
    for block_index, block in sorted(blocks.items()):
        tracks = [block[method]["warmup"]["gate_actor_track"] for method in METHODS]
        gate_ranges.append({
            "block_index": block_index,
            "coordinate_range_m": max(x["coordinate_m"] for x in tracks) - min(x["coordinate_m"] for x in tracks),
            "speed_range_mps": max(x["coordinate_speed_mps"] for x in tracks) - min(x["coordinate_speed_mps"] for x in tracks),
        })
        paired.append({
            "block_index": block_index,
            "scene_id": block["pct_scope_rl_v6"]["scene_id"],
            "actor_policy": block["pct_scope_rl_v6"]["actor_policy_evaluator_only"],
            "pct_time_s": float(block["pct_scope_rl_v6"]["result"]["restricted_elapsed_s"]),
            "analytic_time_s": float(block["sensor_analytic_prior"]["result"]["restricted_elapsed_s"]),
            "actual_teb_time_s": float(block["actual_teb"]["result"]["restricted_elapsed_s"]),
        })
    base_checks = {
        "exact_episode_count": len(receipts) == int(contract["evaluation"][phase]["episodes"]),
        "attempt1_complete": all(receipt["row"]["attempt"] == 1 for receipt in receipts),
        "execution_integrity": all(report["execution_integrity_pass"] for report in reports),
        "gate_physical_valid": all(report["gate_physical_valid"] for report in reports),
        "actor_mode_match": all(report["warmup"]["actor_mode_matches_evaluator_policy"] for report in reports),
        "zero_contact": sum(item["contact_count"] for item in by_method.values()) == 0,
        "nonnegative_clearance": min(item["minimum_clearance_m"] for item in by_method.values()) >= 0.0,
        "sensor_input_audit": all(not report["policy_input_audit"]["simulator_actor_world_track_used_by_policy"] and not report["policy_input_audit"]["gazebo_model_states_used_by_policy"] and not report["policy_input_audit"]["privileged_rank_target_used_by_policy"] for report in reports if report["method"] != "actual_teb"),
    }
    qualification_checks = {}
    if phase == "qualification":
        gates = contract["qualification_gates"]
        qualification_checks = dict(base_checks)
        qualification_checks.update({
            "gate_actor_coordinate_range": max(item["coordinate_range_m"] for item in gate_ranges) <= gates["maximum_within_block_gate_actor_coordinate_range_m"],
            "gate_actor_speed_range": max(item["speed_range_mps"] for item in gate_ranges) <= gates["maximum_within_block_gate_actor_speed_range_mps"],
            "pct_goal_success": by_method["pct_scope_rl_v6"]["goal_successes"] / by_method["pct_scope_rl_v6"]["episodes"] >= gates["pct_goal_success_rate_min"],
            "analytic_goal_success": by_method["sensor_analytic_prior"]["goal_successes"] / by_method["sensor_analytic_prior"]["episodes"] >= gates["analytic_goal_success_rate_min"],
            "actual_teb_goal_success": by_method["actual_teb"]["goal_successes"] / by_method["actual_teb"]["episodes"] >= gates["actual_teb_goal_success_rate_min"],
        })
    formal_statistics = None
    if phase == "formal":
        pct_analytic = [(row["pct_time_s"], row["analytic_time_s"]) for row in paired]
        pct_teb = [(row["pct_time_s"], row["actual_teb_time_s"]) for row in paired]
        pct_mean = by_method["pct_scope_rl_v6"]["mean_restricted_time_s"]
        analytic_mean = by_method["sensor_analytic_prior"]["mean_restricted_time_s"]
        teb_mean = by_method["actual_teb"]["mean_restricted_time_s"]
        primary = 1.0 - pct_mean / analytic_mean
        primary_ci = bootstrap_interval(pct_analytic, 66303)
        secondary = 1.0 - pct_mean / teb_mean
        secondary_ci = bootstrap_interval(pct_teb, 66304)
        formal_statistics = {
            "primary_relative_reduction": primary,
            "primary_two_sided_95_percent_ci": primary_ci,
            "secondary_relative_reduction_vs_actual_teb": secondary,
            "secondary_two_sided_95_percent_ci": secondary_ci,
            "effectiveness_checks": {
                "primary_reduction_at_least_0_05": primary >= 0.05,
                "primary_interval_lower_positive": primary_ci[0] > 0.0,
                "goal_success_noninferior": by_method["pct_scope_rl_v6"]["goal_successes"] >= by_method["sensor_analytic_prior"]["goal_successes"],
                "execution_and_safety_integrity": all(base_checks.values()),
            },
        }
        formal_statistics["effectiveness_demonstrated"] = all(formal_statistics["effectiveness_checks"].values())
    return {
        "schema_version": "1.0",
        "stage": "V6-03-{}".format({"preflight": "P0", "qualification": "Q1", "formal": "F1"}[phase]),
        "phase": phase,
        "status": "execution_complete",
        "release_sha256": release_sha,
        "simulation_only": True,
        "episode_count": len(receipts),
        "method_summary": by_method,
        "gate_actor_ranges": gate_ranges,
        "base_integrity_checks": base_checks,
        "qualification_checks": qualification_checks,
        "phase_pass": all(base_checks.values()),
        "qualification_pass": all(qualification_checks.values()) if phase == "qualification" else None,
        "paired_blocks": paired,
        "formal_statistics": formal_statistics,
        "training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }


def execute(phase, output_root):
    release, schedule = validate_release(phase)
    root = Path(output_root).resolve()
    if root.exists():
        raise RuntimeError("output root exists; retry/resume is forbidden")
    root.mkdir(parents=True)
    (root / "exact_schedule.yaml").write_bytes(read(FOUNDATION / "{}_exact_schedule.yaml".format(phase)))
    journal = root / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    release_sha = sha(read(FOUNDATION / "execution_release.yaml"))
    append(journal, {"event": "execution_opened", "phase": phase, "release_sha256": release_sha, "authorized_units": len(schedule["rows"])})
    contract = yaml.safe_load(read(CONTRACT))
    scenes = {item["scene_id"]: item for item in contract["scene_population"]}
    receipts = []
    port_base = {"preflight": 42000, "qualification": 43000, "formal": 45000}[phase]
    for row in schedule["rows"]:
        episode_root = root / "episodes" / "{:03d}_{}_{}_{}_{}".format(row["sequence"], row["seed"], row["scene_id"], row["actor_policy"], row["method"])
        append(journal, {"event": "unit_consumed", "sequence": row["sequence"], "row": row})
        try:
            receipt = run_episode(row, scenes[row["scene_id"]], episode_root, port_base + 2 * row["sequence"])
        except Exception as error:
            append(journal, {"event": "terminal_failure", "sequence": row["sequence"], "error": "{}:{}".format(type(error).__name__, error), "remaining_units_forfeited": len(schedule["rows"]) - row["sequence"]})
            raise
        receipt_path = episode_root / "matrix_receipt.yaml"
        dump(receipt_path, receipt)
        receipts.append(receipt)
        report = receipt["episode_report"]
        append(journal, {"event": "unit_complete", "sequence": row["sequence"], "receipt_sha256": sha(read(receipt_path)), "integrity": report["execution_integrity_pass"], "goal": report["result"]["goal_reached"], "restricted_time_s": report["result"]["restricted_elapsed_s"]})
        print("[V6-03-{}] {}/{} {} integrity={} goal={} time={:.3f}s clearance={:.3f}m".format(phase.upper(), row["sequence"], len(schedule["rows"]), row["method"], report["execution_integrity_pass"], report["result"]["goal_reached"], report["result"]["restricted_elapsed_s"], report["result"]["minimum_conservative_clearance_m"]), flush=True)
    summary = summarize(receipts, phase, release_sha)
    dump(root / "matrix_summary.yaml", summary)
    append(journal, {"event": "execution_complete", "completed_units": len(receipts), "forfeited_units": 0, "summary_sha256": sha(read(root / "matrix_summary.yaml"))})


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
    execute(arguments.phase, arguments.output_root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
