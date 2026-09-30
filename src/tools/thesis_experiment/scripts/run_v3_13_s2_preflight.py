#!/usr/bin/env python3
"""Run the one-shot four-action V3-13-S2 non-evidence preflight."""

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import yaml

import run_v3_13_c1_multiactor_clones as common
import v3_13_multiactor_design as design


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = (
    W
    / "config/thesis_experiments/v3/"
    "v3_13_s2_synchronized_sequential_clone_contract.yaml"
)
SCENE_CONTRACT = (
    W
    / "config/thesis_experiments/v3/"
    "v3_13_c1_multiactor_clone_contract.yaml"
)
PLANNER_CONTRACT = (
    W
    / "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
READINESS = (
    W
    / "artifacts/v3/multiactor/"
    "v3_13_s2_preflight_readiness"
)
RELEASE = READINESS / "preflight_release.yaml"
REVIEW = READINESS / "independent_review.yaml"
OUTPUT = W / "artifacts/v3/multiactor/v3_13_s2_preflight"


def read_bytes(path):
    with path.open("rb") as stream:
        return stream.read()


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True
    ).encode("utf-8")


def atomic_bytes(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(payload)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def atomic_yaml(path, value):
    atomic_bytes(path, yaml_bytes(value))


def append_jsonl(path, value):
    payload = (
        json.dumps(
            value, sort_keys=True, separators=(",", ":"), allow_nan=False
        )
        + "\n"
    ).encode("utf-8")
    with path.open("ab", buffering=0) as stream:
        stream.write(payload)
        os.fsync(stream.fileno())


def stop_process(process):
    if process.poll() is not None:
        return process.returncode
    try:
        os.killpg(process.pid, signal.SIGINT)
        return process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        return process.wait(timeout=5.0)


def rows(contract):
    preflight = contract["integration_preflight"]
    result = []
    for sequence, action in enumerate(preflight["actions"], start=1):
        result.append(
            {
                "sequence": sequence,
                "seed": int(preflight["seed"]),
                "scene_id": preflight["scene_id"],
                "actor_policies_evaluator_only": list(
                    preflight["actor_policies_evaluator_only"]
                ),
                "branch_time_label": preflight["branch_time_label"],
                "pre_action_wait_s": 5.0,
                "branch_target_sim_s": float(
                    preflight["branch_target_sim_s"]
                ),
                "forced_candidate_id": action,
                "behavior_propensity": 0.25,
                "attempt": 1,
            }
        )
    return result


def run_episode(row, scene, root, ros_port):
    root.mkdir(parents=True, exist_ok=False)
    world_path = root / "world.sdf"
    scene_path = root / "scene_spec.yaml"
    actors_path = root / "actor_specs.yaml"
    report_path = root / "episode_report.yaml"
    log_path = root / "roslaunch.log"
    runtime_home = root / "runtime_home"
    runtime_home.mkdir()
    (runtime_home / "roslog").mkdir()
    world, actors = design.render_world(
        scene, row["actor_policies_evaluator_only"], row["seed"]
    )
    atomic_bytes(world_path, world.encode("utf-8"))
    atomic_yaml(scene_path, scene)
    atomic_yaml(
        actors_path,
        {
            "schema_version": "1.0",
            "stage": "V3-13-S2-PREFLIGHT",
            "actors": actors,
        },
    )
    command = [
        "roslaunch",
        "m2_gazebo",
        "m2_v3_13_sequential_clone_episode_v2.launch",
        "world:={}".format(world_path.resolve()),
        "seed:={}".format(row["seed"]),
        "scene_id:={}".format(row["scene_id"]),
        "block_index:=1",
        "pre_action_wait_s:={:.1f}".format(
            row["pre_action_wait_s"]
        ),
        "branch_target_sim_s:={:.1f}".format(
            row["branch_target_sim_s"]
        ),
        "forced_candidate_id:={}".format(row["forced_candidate_id"]),
        "behavior_propensity:=0.25",
        "report_path:={}".format(report_path.resolve()),
        "scene_spec_path:={}".format(scene_path.resolve()),
        "actor_specs_path:={}".format(actors_path.resolve()),
        "planner_contract_path:={}".format(
            PLANNER_CONTRACT.resolve()
        ),
        "gui:=false",
    ]
    started = time.monotonic()
    with log_path.open("wb") as log_stream:
        process = subprocess.Popen(
            command,
            cwd=str(W),
            env=common.clean_environment(runtime_home, ros_port),
            stdin=subprocess.DEVNULL,
            stdout=log_stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        timed_out = False
        try:
            return_code = process.wait(timeout=170.0)
        except subprocess.TimeoutExpired:
            timed_out = True
            return_code = stop_process(process)
        else:
            stop_process(process)
    if not report_path.is_file():
        raise RuntimeError(
            "preflight report absent rc={} timeout={}".format(
                return_code, timed_out
            )
        )
    report = yaml.safe_load(read_bytes(report_path))
    receipt = {
        "schema_version": "1.0",
        "stage": "V3-13-S2-PREFLIGHT",
        "row": row,
        "episode_report": report,
        "episode_report_sha256": sha(read_bytes(report_path)),
        "world_sha256": sha(read_bytes(world_path)),
        "scene_spec_sha256": sha(read_bytes(scene_path)),
        "actor_specs_sha256": sha(read_bytes(actors_path)),
        "roslaunch_log_sha256": sha(read_bytes(log_path)),
        "roslaunch_return_code": return_code,
        "wall_elapsed_s": time.monotonic() - started,
        "timed_out": timed_out,
        "credential_environment_scrubbed": True,
    }
    atomic_yaml(root / "matrix_receipt.yaml", receipt)
    return receipt


def summarize(receipts, contract):
    reports = [receipt["episode_report"] for receipt in receipts]
    decision_x = [
        float(report["common_stop_prefix"]["decision_pose"]["x_m"])
        for report in reports
    ]
    actor_ranges = []
    for track_id in (1, 2):
        tracks = [
            next(
                item
                for item in report["common_stop_prefix"][
                    "decision_actor_tracks"
                ]
                if int(item["track_id"]) == track_id
            )
            for report in reports
        ]
        coordinates = [float(item["coordinate_m"]) for item in tracks]
        speeds = [
            float(item["coordinate_speed_mps"]) for item in tracks
        ]
        actor_ranges.append(
            {
                "track_id": track_id,
                "coordinate_range_m": max(coordinates)
                - min(coordinates),
                "speed_range_mps": max(speeds) - min(speeds),
                "speeds_mps": speeds,
            }
        )
    gates = contract["matching_gates"]
    checks = {
        "completed_units": len(receipts) == 4,
        "all_attempt_one": all(
            receipt["row"]["attempt"] == 1 for receipt in receipts
        ),
        "all_processes_complete": all(
            not receipt["timed_out"] for receipt in receipts
        ),
        "all_episode_collection_pass": all(
            report["episode_collection_pass"] for report in reports
        ),
        "complete_action_support": {
            report["forced_candidate_id"] for report in reports
        }
        == set(contract["population"]["branch_actions"]),
        "all_branch_modes_assertive": all(
            set(report["observable_modes_at_branch"].values())
            == {"assertive"}
            for report in reports
        ),
        "branch_clock_error": max(
            float(
                report["common_stop_prefix"][
                    "branch_clock_absolute_error_s"
                ]
            )
            for report in reports
        )
        <= float(gates["branch_clock_absolute_error_max_s"]),
        "paired_decision_ego_x": max(decision_x) - min(decision_x)
        <= float(
            gates["paired_decision_ego_x_difference_max_m"]
        ),
        "paired_actor_coordinate": max(
            item["coordinate_range_m"] for item in actor_ranges
        )
        <= float(
            gates[
                "paired_decision_actor_coordinate_difference_max_m"
            ]
        ),
        "paired_actor_speed": max(
            item["speed_range_mps"] for item in actor_ranges
        )
        <= float(
            gates["paired_decision_actor_speed_difference_max_mps"]
        ),
        "branch_trace_samples": all(
            int(report["result"]["trace_sample_count"])
            >= int(gates["branch_trace_samples_min"])
            for report in reports
        ),
        "branch_trace_duration": all(
            float(report["result"]["trace_duration_s"])
            >= float(gates["branch_trace_duration_s_min"])
            for report in reports
        ),
    }
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S2-PREFLIGHT",
        "status": (
            "synchronized_sequential_preflight_pass"
            if all(checks.values())
            else "synchronized_sequential_preflight_fail"
        ),
        "authorized_units": 4,
        "consumed_units": len(receipts),
        "completed_units": len(receipts),
        "evidence_units_created": 0,
        "maximum_branch_clock_absolute_error_s": max(
            float(
                report["common_stop_prefix"][
                    "branch_clock_absolute_error_s"
                ]
            )
            for report in reports
        ),
        "maximum_decision_ego_x_range_m": max(decision_x)
        - min(decision_x),
        "maximum_actor_coordinate_range_m": max(
            item["coordinate_range_m"] for item in actor_ranges
        ),
        "maximum_actor_speed_range_mps": max(
            item["speed_range_mps"] for item in actor_ranges
        ),
        "actor_ranges": actor_ranges,
        "checks": checks,
        "preflight_pass": all(checks.values()),
        "output_cannot_enter_predictor_fit_or_performance_analysis": True,
        "future_evidence_release_design_authorized": all(
            checks.values()
        ),
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--review-sha256", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    arguments = parser.parse_args()
    if OUTPUT.exists():
        raise RuntimeError("preflight output exists; rerun forbidden")
    release_payload = read_bytes(RELEASE)
    review_payload = read_bytes(REVIEW)
    if sha(release_payload) != arguments.release_sha256:
        raise RuntimeError("release SHA differs")
    if sha(review_payload) != arguments.review_sha256:
        raise RuntimeError("review SHA differs")
    release = yaml.safe_load(release_payload)
    review = yaml.safe_load(review_payload)
    if not release["execution_authorized"]:
        raise RuntimeError("preflight execution not authorized")
    if review["preflight_release_sha256"] != arguments.release_sha256:
        raise RuntimeError("review does not bind release")
    contract = yaml.safe_load(read_bytes(CONTRACT))
    schedule = rows(contract)
    scenes = {
        item["scene_id"]: item
        for item in yaml.safe_load(
            read_bytes(SCENE_CONTRACT)
        )["scene_population"]
    }
    OUTPUT.mkdir(parents=True)
    atomic_yaml(
        OUTPUT / "preflight_schedule.yaml",
        {
            "schema_version": "1.0",
            "stage": "V3-13-S2-PREFLIGHT",
            "rows": schedule,
        },
    )
    journal = OUTPUT / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    append_jsonl(
        journal,
        {
            "event": "preflight_opened",
            "authorized_units": 4,
            "evidence_identity": False,
        },
    )
    receipts = []
    for row in schedule:
        append_jsonl(
            journal,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "row": row,
            },
        )
        root = OUTPUT / "episodes" / "{:02d}_{}_{}".format(
            row["sequence"],
            row["seed"],
            row["forced_candidate_id"],
        )
        try:
            receipt = run_episode(
                row,
                scenes[row["scene_id"]],
                root,
                29900 + 2 * row["sequence"],
            )
        except Exception as error:
            append_jsonl(
                journal,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "error": "{}:{}".format(type(error).__name__, error),
                    "remaining_units_forfeited": 4 - row["sequence"],
                },
            )
            raise
        receipts.append(receipt)
        report = receipt["episode_report"]
        append_jsonl(
            journal,
            {
                "event": "unit_complete",
                "sequence": row["sequence"],
                "episode_report_sha256": receipt[
                    "episode_report_sha256"
                ],
                "collection_pass": report[
                    "episode_collection_pass"
                ],
            },
        )
        print(
            "[V3-13-S2-PREFLIGHT] {}/4 action={} pass={} "
            "clock_error={:.4f}s".format(
                row["sequence"],
                row["forced_candidate_id"],
                report["episode_collection_pass"],
                report["common_stop_prefix"][
                    "branch_clock_absolute_error_s"
                ],
            ),
            flush=True,
        )
    result = summarize(receipts, contract)
    atomic_yaml(OUTPUT / "preflight_result.yaml", result)
    append_jsonl(
        journal,
        {
            "event": "preflight_complete",
            "consumed_units": 4,
            "completed_units": 4,
            "evidence_units_created": 0,
            "preflight_pass": result["preflight_pass"],
            "result_sha256": sha(
                read_bytes(OUTPUT / "preflight_result.yaml")
            ),
        },
    )
    print(
        "status={} result_sha256={}".format(
            result["status"],
            sha(read_bytes(OUTPUT / "preflight_result.yaml")),
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
