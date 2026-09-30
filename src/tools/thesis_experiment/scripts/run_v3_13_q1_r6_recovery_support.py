#!/usr/bin/env python3
"""Run the exact 40-unit V3-13-Q1-R6 support matrix."""

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
R1_RUNNER = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r1_actual_teb_matrix.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r6_environment", R1_RUNNER
)
R1 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R1)
COMMON = R1.BASE
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r6_recovery_bank_support_contract.yaml"
)
GEOMETRY = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r4_r1_recovery_candidate_geometry_contract.yaml"
)
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r6_readiness"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/v3_13_q1_r6_recovery_support"
)


def finite_report(report):
    for sample in (
        report["common_stop_prefix"]["trace"]
        + report["result"]["trace"]
    ):
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
            "stage": "V3-13-Q1-R6",
            "actors": actors,
        },
    )
    command = [
        "roslaunch",
        "m2_gazebo",
        "m2_v3_13_recovery_bank_clone_episode.launch",
        "world:={}".format(world_path.resolve()),
        "seed:={}".format(row["seed"]),
        "scene_id:={}".format(row["scene_id"]),
        "block_index:={}".format(row["block_index"]),
        "pre_action_wait_s:=5.0",
        "branch_target_sim_s:=12.5",
        "forced_candidate_id:={}".format(
            row["forced_candidate_id"]
        ),
        "behavior_propensity:=0.20",
        "report_path:={}".format(report_path.resolve()),
        "scene_spec_path:={}".format(scene_path.resolve()),
        "actor_specs_path:={}".format(actors_path.resolve()),
        "planner_contract_path:={}".format(
            COMMON.PLANNER_CONTRACT.resolve()
        ),
        "geometry_contract_path:={}".format(GEOMETRY.resolve()),
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
        "stage": "V3-13-Q1-R6",
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


def summarize(receipts, contract_sha, contract):
    blocks = {}
    for receipt in receipts:
        blocks.setdefault(
            int(receipt["row"]["block_index"]), []
        ).append(receipt)
    expected = set(contract["population"]["actions"])
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
        actor_ranges = []
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
            actor_ranges.append(
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
                == expected,
                "minimum_behavior_propensity": min(
                    float(report["behavior_propensity"])
                    for report in reports
                ),
                "maximum_branch_clock_absolute_error_s": max(
                    float(
                        report["common_stop_prefix"][
                            "branch_clock_absolute_error_s"
                        ]
                    )
                    for report in reports
                ),
                "decision_ego_x_range_m": max(ego_x) - min(ego_x),
                "actor_ranges": actor_ranges,
                "all_assertive_at_branch": all(
                    set(report["observable_modes_at_branch"].values())
                    == {"assertive"}
                    for report in reports
                ),
                "all_collection_pass": all(
                    report["episode_collection_pass"]
                    for report in reports
                ),
                "all_finite": all(
                    finite_report(report) for report in reports
                ),
                "contact_state_count_retained": sum(
                    int(report["result"]["contact_state_count"])
                    for report in reports
                ),
            }
        )
    all_actor = [
        actor for block in block_rows for actor in block["actor_ranges"]
    ]
    gates = contract["matching_gates"]
    metrics = {
        "complete_action_support_rate": sum(
            block["full_action_support"] for block in block_rows
        )
        / len(block_rows),
        "minimum_behavior_propensity": min(
            block["minimum_behavior_propensity"]
            for block in block_rows
        ),
        "maximum_branch_clock_absolute_error_s": max(
            block["maximum_branch_clock_absolute_error_s"]
            for block in block_rows
        ),
        "maximum_decision_ego_x_range_m": max(
            block["decision_ego_x_range_m"] for block in block_rows
        ),
        "maximum_decision_actor_coordinate_range_m": max(
            actor["coordinate_range_m"] for actor in all_actor
        ),
        "maximum_decision_actor_speed_range_mps": max(
            actor["speed_range_mps"] for actor in all_actor
        ),
    }
    checks = {
        "completed_units_exact": len(receipts) == 40,
        "complete_action_support": metrics[
            "complete_action_support_rate"
        ]
        >= float(gates["complete_action_support_rate_min"]),
        "minimum_propensity": metrics["minimum_behavior_propensity"]
        >= float(gates["minimum_behavior_propensity"]),
        "all_processes_complete": all(
            not receipt["timed_out"] for receipt in receipts
        ),
        "all_clean_exit": all(
            receipt["roslaunch_return_code"] == 0
            for receipt in receipts
        ),
        "all_collection_pass": all(
            block["all_collection_pass"] for block in block_rows
        ),
        "all_finite": all(
            block["all_finite"] for block in block_rows
        ),
        "all_assertive_at_branch": all(
            block["all_assertive_at_branch"] for block in block_rows
        ),
        "branch_clock": metrics[
            "maximum_branch_clock_absolute_error_s"
        ]
        <= float(gates["branch_clock_absolute_error_max_s"]),
        "paired_ego_x": metrics["maximum_decision_ego_x_range_m"]
        <= float(
            gates["paired_decision_ego_x_difference_max_m"]
        ),
        "paired_actor_coordinate": metrics[
            "maximum_decision_actor_coordinate_range_m"
        ]
        <= float(
            gates[
                "paired_decision_actor_coordinate_difference_max_m"
            ]
        ),
        "paired_actor_speed": metrics[
            "maximum_decision_actor_speed_range_mps"
        ]
        <= float(
            gates["paired_decision_actor_speed_difference_max_mps"]
        ),
        "trace_samples": all(
            receipt["episode_report"]["result"][
                "trace_sample_count"
            ]
            >= int(gates["branch_trace_samples_min"])
            for receipt in receipts
        ),
        "trace_duration": all(
            receipt["episode_report"]["result"]["trace_duration_s"]
            >= float(gates["branch_trace_duration_s_min"])
            for receipt in receipts
        ),
    }
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R6",
        "status": (
            "fresh_five_action_support_collection_pass"
            if all(checks.values())
            else "fresh_five_action_support_collection_fail"
        ),
        "contract_sha256": contract_sha,
        "authorized_units": 40,
        "consumed_units": len(receipts),
        "completed_units": len(receipts),
        "forfeited_units": 0,
        "matched_context_blocks": len(block_rows),
        **metrics,
        "contact_state_count_retained": sum(
            block["contact_state_count_retained"]
            for block in block_rows
        ),
        "gate_checks": checks,
        "collection_gate_pass": all(checks.values()),
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
    args = parser.parse_args()
    if Path(args.output_root).resolve() != OUTPUT.resolve():
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
    if COMMON.sha(contract_payload) != args.contract_sha256:
        raise RuntimeError("contract SHA differs")
    if COMMON.sha(release_payload) != args.release_sha256:
        raise RuntimeError("release SHA differs")
    if COMMON.sha(review_payload) != args.review_sha256:
        raise RuntimeError("review SHA differs")
    release = yaml.safe_load(release_payload)
    review = yaml.safe_load(review_payload)
    if not release["execution_authorized"]:
        raise RuntimeError("execution not authorized")
    if review["execution_release_sha256"] != args.release_sha256:
        raise RuntimeError("review release differs")
    rows = yaml.safe_load(
        COMMON.read_bytes(READINESS / "exact_schedule.yaml")
    )["rows"]
    if len(rows) != 40:
        raise RuntimeError("schedule row count")
    d0 = yaml.safe_load(COMMON.read_bytes(COMMON.SCENE_CONTRACT))
    scenes = {
        scene["scene_id"]: scene for scene in d0["scene_families"]
    }
    contract = yaml.safe_load(contract_payload)
    OUTPUT.mkdir(parents=True)
    COMMON.atomic_yaml(
        OUTPUT / "exact_schedule.yaml",
        {
            "schema_version": "1.0",
            "stage": "V3-13-Q1-R6",
            "contract_sha256": args.contract_sha256,
            "row_count": 40,
            "rows": rows,
        },
    )
    journal = OUTPUT / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    COMMON.append_jsonl(
        journal,
        {
            "event": "execution_opened",
            "authorized_units": 40,
            "contract_sha256": args.contract_sha256,
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
                31000 + 2 * row["sequence"],
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
                    "remaining_units_forfeited": 40
                    - row["sequence"],
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
            "[V3-13-Q1-R6] {}/40 block={} action={} pass={} "
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
    summary = summarize(
        receipts, args.contract_sha256, contract
    )
    COMMON.atomic_yaml(OUTPUT / "collection_summary.yaml", summary)
    COMMON.append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "consumed_units": 40,
            "completed_units": 40,
            "forfeited_units": 0,
            "collection_gate_pass": summary[
                "collection_gate_pass"
            ],
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
