#!/usr/bin/env python3
"""Run the exact 16-unit Q1-R3 midpoint cloned-intervention matrix."""

import argparse
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import yaml

import v3_13_multiactor_design as design


W = Path("/home/robot/robot_ws_base_rl")
R1_RUNNER_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r1_actual_teb_matrix.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r1_environment_r3", R1_RUNNER_PATH
)
R1 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R1)
COMMON = R1.BASE
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r3_midpoint_clone_contract.yaml"
)
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r3_readiness"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/v3_13_q1_r3_execution"
)


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
    COMMON.atomic_bytes(world_path, world.encode("utf-8"))
    COMMON.atomic_yaml(scene_path, scene)
    COMMON.atomic_yaml(
        actors_path,
        {
            "schema_version": "1.0",
            "stage": "V3-13-Q1-R3",
            "actors": actors,
        },
    )
    command = [
        "roslaunch",
        "m2_gazebo",
        "m2_v3_13_midpoint_clone_episode.launch",
        "world:={}".format(world_path.resolve()),
        "seed:={}".format(row["seed"]),
        "scene_id:={}".format(row["scene_id"]),
        "block_index:={}".format(row["block_index"]),
        "forced_candidate_id:={}".format(
            row["forced_candidate_id"]
        ),
        "behavior_propensity:=0.25",
        "report_path:={}".format(report_path.resolve()),
        "scene_spec_path:={}".format(scene_path.resolve()),
        "actor_specs_path:={}".format(actors_path.resolve()),
        "planner_contract_path:={}".format(
            COMMON.PLANNER_CONTRACT.resolve()
        ),
        "gui:=false",
    ]
    started = time.monotonic()
    with log_path.open("wb") as stream:
        process = subprocess.Popen(
            command,
            cwd=str(W),
            env=R1.closed_overlay_environment(runtime_home, ros_port),
            stdin=subprocess.DEVNULL,
            stdout=stream,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        timed_out = False
        try:
            return_code = process.wait(timeout=190.0)
        except subprocess.TimeoutExpired:
            timed_out = True
            return_code = COMMON.stop_process(process)
        else:
            COMMON.stop_process(process)
    if not report_path.is_file():
        tail = COMMON.read_bytes(log_path)[-8000:].decode(
            "utf-8", errors="replace"
        )
        raise RuntimeError(
            "episode report absent rc={} timeout={}\n{}".format(
                return_code, timed_out, tail
            )
        )
    report = yaml.safe_load(COMMON.read_bytes(report_path))
    receipt = {
        "schema_version": "1.0",
        "stage": "V3-13-Q1-R3",
        "row": row,
        "episode_report": report,
        "episode_report_sha256": COMMON.sha(
            COMMON.read_bytes(report_path)
        ),
        "world_sha256": COMMON.sha(COMMON.read_bytes(world_path)),
        "scene_spec_sha256": COMMON.sha(
            COMMON.read_bytes(scene_path)
        ),
        "actor_specs_sha256": COMMON.sha(
            COMMON.read_bytes(actors_path)
        ),
        "roslaunch_log_sha256": COMMON.sha(
            COMMON.read_bytes(log_path)
        ),
        "roslaunch_return_code": return_code,
        "wall_elapsed_s": time.monotonic() - started,
        "timed_out": timed_out,
        "credential_environment_scrubbed": True,
        "workspace_overlay_explicit": True,
        "foreign_workspace_precedence_used": False,
    }
    COMMON.atomic_yaml(root / "matrix_receipt.yaml", receipt)
    return receipt


def finite_report(report):
    samples = (
        report["common_stop_prefix"]["trace"]
        + report["result"]["trace"]
    )
    for sample in samples:
        values = [
            sample["elapsed_s"],
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


def summarize(receipts, contract_sha):
    blocks = {}
    for receipt in receipts:
        blocks.setdefault(
            int(receipt["row"]["block_index"]), []
        ).append(receipt)
    block_rows = []
    for block_index in sorted(blocks):
        reports = [
            receipt["episode_report"]
            for receipt in blocks[block_index]
        ]
        ego_x = [
            float(report["common_stop_prefix"][
                "decision_pose"
            ]["x_m"])
            for report in reports
        ]
        actor_rows = []
        for track_id in (1, 2):
            tracks = [
                next(
                    actor
                    for actor in report["common_stop_prefix"][
                        "decision_actor_tracks"
                    ]
                    if int(actor["track_id"]) == track_id
                )
                for report in reports
            ]
            coordinates = [
                float(track["coordinate_m"]) for track in tracks
            ]
            speeds = [
                float(track["coordinate_speed_mps"])
                for track in tracks
            ]
            actor_rows.append(
                {
                    "track_id": track_id,
                    "coordinate_range_m": max(coordinates)
                    - min(coordinates),
                    "speed_range_mps": max(speeds) - min(speeds),
                }
            )
        block_rows.append(
            {
                "block_index": block_index,
                "seed": reports[0]["seed"],
                "scene_id": reports[0]["scene_id"],
                "actor_policies_evaluator_only": reports[0][
                    "actor_policies_evaluator_only"
                ],
                "row_count": len(reports),
                "full_action_support": {
                    report["forced_candidate_id"]
                    for report in reports
                }
                == set(design.CANDIDATE_IDS),
                "decision_ego_x_range_m": max(ego_x) - min(ego_x),
                "actor_ranges": actor_rows,
            }
        )
    actor_rows = [
        actor for block in block_rows for actor in block["actor_ranges"]
    ]
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R3",
        "status": "midpoint_clone_collection_complete",
        "contract_sha256": contract_sha,
        "authorized_units": 16,
        "consumed_units": len(receipts),
        "completed_units": len(receipts),
        "forfeited_units": 0,
        "matched_context_blocks": len(block_rows),
        "complete_action_support_rate": sum(
            block["full_action_support"] for block in block_rows
        )
        / len(block_rows),
        "maximum_branch_clock_absolute_error_s": max(
            float(receipt["episode_report"]["common_stop_prefix"][
                "branch_clock_absolute_error_s"
            ])
            for receipt in receipts
        ),
        "maximum_decision_ego_x_range_m": max(
            block["decision_ego_x_range_m"] for block in block_rows
        ),
        "maximum_decision_actor_coordinate_range_m": max(
            actor["coordinate_range_m"] for actor in actor_rows
        ),
        "maximum_decision_actor_speed_range_mps": max(
            actor["speed_range_mps"] for actor in actor_rows
        ),
        "all_episode_collection_pass": all(
            receipt["episode_report"]["episode_collection_pass"]
            for receipt in receipts
        ),
        "all_reports_finite": all(
            finite_report(receipt["episode_report"])
            for receipt in receipts
        ),
        "blocks": block_rows,
        "policy_or_ranker_training_steps": 0,
        "actual_TEB_execution_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--contract-sha256", required=True)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--review-sha256", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    arguments = parser.parse_args()
    if Path(arguments.output_root).resolve() != OUTPUT.resolve():
        raise RuntimeError("canonical output differs")
    if OUTPUT.exists():
        raise RuntimeError("output exists; retry forbidden")
    release_payload = COMMON.read_bytes(
        READINESS / "execution_release.yaml"
    )
    review_payload = COMMON.read_bytes(
        READINESS / "independent_review.yaml"
    )
    contract_payload = COMMON.read_bytes(CONTRACT)
    if COMMON.sha(release_payload) != arguments.release_sha256:
        raise RuntimeError("release SHA differs")
    if COMMON.sha(review_payload) != arguments.review_sha256:
        raise RuntimeError("review SHA differs")
    if COMMON.sha(contract_payload) != arguments.contract_sha256:
        raise RuntimeError("contract SHA differs")
    release = yaml.safe_load(release_payload)
    review = yaml.safe_load(review_payload)
    if not release["execution_authorized"]:
        raise RuntimeError("execution not authorized")
    if review["execution_release_sha256"] != arguments.release_sha256:
        raise RuntimeError("review binding differs")
    rows = yaml.safe_load(
        COMMON.read_bytes(READINESS / "exact_schedule.yaml")
    )["rows"]
    if len(rows) != 16:
        raise RuntimeError("schedule row count")
    scenes = {
        scene["scene_id"]: scene
        for scene in yaml.safe_load(
            COMMON.read_bytes(COMMON.SCENE_CONTRACT)
        )["scene_families"]
    }
    OUTPUT.mkdir(parents=True)
    COMMON.atomic_yaml(
        OUTPUT / "exact_schedule.yaml",
        {
            "schema_version": "1.0",
            "stage": "V3-13-Q1-R3",
            "contract_sha256": arguments.contract_sha256,
            "row_count": 16,
            "rows": rows,
        },
    )
    journal = OUTPUT / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    COMMON.append_jsonl(
        journal,
        {
            "event": "execution_opened",
            "authorized_units": 16,
            "contract_sha256": arguments.contract_sha256,
        },
    )
    receipts = []
    for row in rows:
        COMMON.append_jsonl(
            journal,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "row": row,
            },
        )
        root = OUTPUT / "episodes" / (
            "{:03d}_{:02d}_{}_{}".format(
                row["sequence"],
                row["block_index"],
                row["seed"],
                row["forced_candidate_id"],
            )
        )
        try:
            receipt = run_episode(
                row,
                scenes[row["scene_id"]],
                root,
                30900 + 2 * row["sequence"],
            )
        except Exception as error:
            COMMON.append_jsonl(
                journal,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "row": row,
                    "error": "{}:{}".format(
                        type(error).__name__, error
                    ),
                    "remaining_units_forfeited": (
                        16 - row["sequence"]
                    ),
                },
            )
            raise
        receipts.append(receipt)
        report = receipt["episode_report"]
        COMMON.append_jsonl(
            journal,
            {
                "event": "unit_complete",
                "sequence": row["sequence"],
                "row": row,
                "episode_report_sha256": receipt[
                    "episode_report_sha256"
                ],
                "collection_pass": report[
                    "episode_collection_pass"
                ],
            },
        )
        print(
            "[V3-13-Q1-R3] {}/16 block={} action={} pass={} "
            "clearance={:.3f}m".format(
                row["sequence"],
                row["block_index"],
                row["forced_candidate_id"],
                report["episode_collection_pass"],
                report["result"][
                    "minimum_conservative_clearance_m"
                ],
            ),
            flush=True,
        )
    summary = summarize(receipts, arguments.contract_sha256)
    COMMON.atomic_yaml(OUTPUT / "collection_summary.yaml", summary)
    COMMON.append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "consumed_units": 16,
            "completed_units": 16,
            "forfeited_units": 0,
            "summary_sha256": COMMON.sha(
                COMMON.read_bytes(OUTPUT / "collection_summary.yaml")
            ),
        },
    )
    print(
        "status={} summary_sha256={}".format(
            summary["status"],
            COMMON.sha(
                COMMON.read_bytes(OUTPUT / "collection_summary.yaml")
            ),
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
