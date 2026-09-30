#!/usr/bin/env python3
"""Run one released V3-13-Q1 actual-TEB fair matrix exactly once."""

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
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_actual_teb_multiactor_qualification_contract.yaml"
)
SCENE_CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
PLANNER_CONTRACT = SCENE_CONTRACT
PREDICTOR = W / (
    "artifacts/v3/multiactor/v3_13_s3_predictor/"
    "candidate_conditioned_recovery_predictor.yaml"
)
PHASES = {
    "preflight": {
        "readiness": W
        / "artifacts/v3/multiactor/v3_13_q1_preflight_readiness",
        "output": W / "artifacts/v3/multiactor/v3_13_q1_preflight",
        "units": 2,
    },
    "qualification": {
        "readiness": W
        / "artifacts/v3/multiactor/v3_13_q1_readiness",
        "output": W / "artifacts/v3/multiactor/v3_13_q1_execution",
        "units": 32,
    },
}


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


def run_episode(row, scene, root, ros_port, stage):
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
            "stage": stage,
            "actors": actors,
        },
    )
    command = [
        "roslaunch",
        "m2_gazebo",
        "m2_v3_13_actual_teb_multiactor_episode.launch",
        "world:={}".format(world_path.resolve()),
        "method:={}".format(row["method"]),
        "seed:={}".format(row["seed"]),
        "scene_id:={}".format(row["scene_id"]),
        "block_index:={}".format(row["block_index"]),
        "report_path:={}".format(report_path.resolve()),
        "scene_spec_path:={}".format(scene_path.resolve()),
        "actor_specs_path:={}".format(actors_path.resolve()),
        "planner_contract_path:={}".format(
            PLANNER_CONTRACT.resolve()
        ),
        "predictor_model_path:={}".format(PREDICTOR.resolve()),
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
            return_code = process.wait(timeout=190.0)
        except subprocess.TimeoutExpired:
            timed_out = True
            return_code = stop_process(process)
        else:
            stop_process(process)
    if not report_path.is_file():
        tail = read_bytes(log_path)[-10000:].decode(
            "utf-8", errors="replace"
        )
        raise RuntimeError(
            "episode report absent rc={} timeout={}\n{}".format(
                return_code, timed_out, tail
            )
        )
    report = yaml.safe_load(read_bytes(report_path))
    receipt = {
        "schema_version": "1.0",
        "stage": stage,
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


def finite_report(report):
    for sample in report["result"]["trace"]:
        values = [
            sample["elapsed_s"],
            sample["sim_time_s"],
            sample["ego"]["x_m"],
            sample["ego"]["y_m"],
            sample["ego"]["yaw_rad"],
            sample["ego"]["speed_mps"],
        ]
        for actor in sample["actors"]:
            values.extend(
                (
                    actor["world_x_m"],
                    actor["world_y_m"],
                    actor["coordinate_m"],
                    actor["coordinate_speed_mps"],
                    actor["age_s"],
                )
            )
        if not all(math.isfinite(float(value)) for value in values):
            return False
    return True


def summarize(receipts, phase, contract_sha):
    blocks = {}
    for receipt in receipts:
        blocks.setdefault(int(receipt["row"]["block_index"]), []).append(
            receipt
        )
    block_rows = []
    for block_index in sorted(blocks):
        selected = blocks[block_index]
        reports = {
            receipt["row"]["method"]: receipt["episode_report"]
            for receipt in selected
        }
        complete_pair = set(reports) == {
            "v3_13_analytic",
            "actual_teb",
        }
        actor_differences = []
        if complete_pair:
            v3_report = reports["v3_13_analytic"]
            teb_report = reports["actual_teb"]
            for track_id in (1, 2):
                v3_track = next(
                    item
                    for item in v3_report["warmup"][
                        "gate_actor_tracks"
                    ]
                    if int(item["track_id"]) == track_id
                )
                teb_track = next(
                    item
                    for item in teb_report["warmup"][
                        "gate_actor_tracks"
                    ]
                    if int(item["track_id"]) == track_id
                )
                actor_differences.append(
                    {
                        "track_id": track_id,
                        "coordinate_difference_m": abs(
                            float(v3_track["coordinate_m"])
                            - float(teb_track["coordinate_m"])
                        ),
                        "speed_difference_mps": abs(
                            float(v3_track["coordinate_speed_mps"])
                            - float(
                                teb_track["coordinate_speed_mps"]
                            )
                        ),
                    }
                )
            ego_x_difference = abs(
                float(v3_report["warmup"]["gate_pose"]["x_m"])
                - float(teb_report["warmup"]["gate_pose"]["x_m"])
            )
        else:
            ego_x_difference = float("inf")
        block_rows.append(
            {
                "block_index": block_index,
                "seed": selected[0]["row"]["seed"],
                "scene_id": selected[0]["row"]["scene_id"],
                "actor_policies_evaluator_only": selected[0]["row"][
                    "actor_policies_evaluator_only"
                ],
                "complete_method_pair": complete_pair,
                "paired_gate_ego_x_difference_m": ego_x_difference,
                "paired_gate_actor_differences": actor_differences,
                "method_results": {
                    method: {
                        "episode_pass": report["episode_pass"],
                        "goal_reached": report["result"][
                            "goal_reached"
                        ],
                        "restricted_elapsed_s": report["result"][
                            "restricted_elapsed_s"
                        ],
                        "minimum_conservative_clearance_m": report[
                            "result"
                        ]["minimum_conservative_clearance_m"],
                        "contact_state_count": report["result"][
                            "contact_state_count"
                        ],
                        "initial_stop": report["result"][
                            "initial_stop"
                        ],
                        "eventual_progress": report["result"][
                            "eventual_progress"
                        ],
                    }
                    for method, report in reports.items()
                },
            }
        )
    maximum_ego = max(
        row["paired_gate_ego_x_difference_m"] for row in block_rows
    )
    actor_rows = [
        actor
        for row in block_rows
        for actor in row["paired_gate_actor_differences"]
    ]
    summary = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": (
            "V3-13-Q1-PREFLIGHT"
            if phase == "preflight"
            else "V3-13-Q1"
        ),
        "phase": phase,
        "status": "collection_complete",
        "contract_sha256": contract_sha,
        "authorized_units": PHASES[phase]["units"],
        "consumed_units": len(receipts),
        "completed_units": len(receipts),
        "forfeited_units": 0,
        "complete_paired_blocks": sum(
            row["complete_method_pair"] for row in block_rows
        ),
        "episode_pass_count": sum(
            receipt["episode_report"]["episode_pass"]
            for receipt in receipts
        ),
        "goal_success_count": sum(
            receipt["episode_report"]["result"]["goal_reached"]
            for receipt in receipts
        ),
        "maximum_paired_gate_ego_x_difference_m": maximum_ego,
        "maximum_paired_gate_actor_coordinate_difference_m": max(
            actor["coordinate_difference_m"] for actor in actor_rows
        ),
        "maximum_paired_gate_actor_speed_difference_mps": max(
            actor["speed_difference_mps"] for actor in actor_rows
        ),
        "all_reports_finite": all(
            finite_report(receipt["episode_report"])
            for receipt in receipts
        ),
        "blocks": block_rows,
        "policy_or_ranker_training_steps": 0,
        "performance_claim_authorized": False,
        "perception_claim_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--phase", choices=sorted(PHASES), required=True
    )
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--contract-sha256", required=True)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--review-sha256", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    arguments = parser.parse_args()
    phase = arguments.phase
    config = PHASES[phase]
    output = config["output"]
    readiness = config["readiness"]
    release_path = readiness / "execution_release.yaml"
    review_path = readiness / "independent_review.yaml"
    schedule_path = readiness / "exact_schedule.yaml"
    if Path(arguments.output_root).resolve() != output.resolve():
        raise RuntimeError("canonical output root differs")
    if output.exists():
        raise RuntimeError("output exists; retry or resume forbidden")
    release_payload = read_bytes(release_path)
    review_payload = read_bytes(review_path)
    if sha(release_payload) != arguments.release_sha256:
        raise RuntimeError("release SHA differs")
    if sha(review_payload) != arguments.review_sha256:
        raise RuntimeError("review SHA differs")
    release = yaml.safe_load(release_payload)
    review = yaml.safe_load(review_payload)
    if not release["execution_authorized"]:
        raise RuntimeError("execution not authorized")
    if int(release["authorized_units"]) != config["units"]:
        raise RuntimeError("authorized units differ")
    if review["execution_release_sha256"] != arguments.release_sha256:
        raise RuntimeError("review does not bind release")
    contract_payload = read_bytes(CONTRACT)
    contract_sha = sha(contract_payload)
    if arguments.contract_sha256 != contract_sha:
        raise RuntimeError("contract SHA differs")
    if release["contract_sha256"] != contract_sha:
        raise RuntimeError("release does not bind contract")
    contract = yaml.safe_load(contract_payload)
    if contract["authority"]["training_authorized"]:
        raise RuntimeError("training boundary differs")
    rows = yaml.safe_load(read_bytes(schedule_path))["rows"]
    if len(rows) != config["units"]:
        raise RuntimeError("schedule row count differs")
    scenes = {
        item["scene_id"]: item
        for item in yaml.safe_load(read_bytes(SCENE_CONTRACT))[
            "scene_families"
        ]
    }
    output.mkdir(parents=True)
    atomic_yaml(
        output / "exact_schedule.yaml",
        {
            "schema_version": "1.0",
            "stage": (
                "V3-13-Q1-PREFLIGHT"
                if phase == "preflight"
                else "V3-13-Q1"
            ),
            "phase": phase,
            "contract_sha256": contract_sha,
            "row_count": len(rows),
            "rows": rows,
        },
    )
    journal = output / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    append_jsonl(
        journal,
        {
            "event": "execution_opened",
            "phase": phase,
            "authorized_units": config["units"],
            "contract_sha256": contract_sha,
        },
    )
    receipts = []
    stage = (
        "V3-13-Q1-PREFLIGHT"
        if phase == "preflight"
        else "V3-13-Q1"
    )
    port_base = 30600 if phase == "preflight" else 30700
    for row in rows:
        append_jsonl(
            journal,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "row": row,
            },
        )
        root = output / "episodes" / (
            "{:03d}_{:02d}_{}_{}".format(
                row["sequence"],
                row["block_index"],
                row["seed"],
                row["method"],
            )
        )
        try:
            receipt = run_episode(
                row,
                scenes[row["scene_id"]],
                root,
                port_base + 2 * row["sequence"],
                stage,
            )
        except Exception as error:
            append_jsonl(
                journal,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "row": row,
                    "error": "{}:{}".format(
                        type(error).__name__, error
                    ),
                    "remaining_units_forfeited": (
                        config["units"] - row["sequence"]
                    ),
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
                "row": row,
                "episode_report_sha256": receipt[
                    "episode_report_sha256"
                ],
                "episode_pass": report["episode_pass"],
                "goal_reached": report["result"]["goal_reached"],
            },
        )
        print(
            "[{}] {}/{} block={} method={} pass={} time={:.3f}s "
            "clearance={:.3f}m".format(
                stage,
                row["sequence"],
                config["units"],
                row["block_index"],
                row["method"],
                report["episode_pass"],
                report["result"]["restricted_elapsed_s"],
                report["result"][
                    "minimum_conservative_clearance_m"
                ],
            ),
            flush=True,
        )
    summary = summarize(receipts, phase, contract_sha)
    atomic_yaml(output / "collection_summary.yaml", summary)
    append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "authorized_units": config["units"],
            "consumed_units": config["units"],
            "completed_units": config["units"],
            "forfeited_units": 0,
            "summary_sha256": sha(
                read_bytes(output / "collection_summary.yaml")
            ),
        },
    )
    print(
        "status={} summary_sha256={}".format(
            summary["status"],
            sha(read_bytes(output / "collection_summary.yaml")),
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
