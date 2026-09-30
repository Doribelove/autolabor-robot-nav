#!/usr/bin/env python3
"""Run the exact two-unit R16 actual-TEB paired preflight once."""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

import yaml


W = Path("/home/robot/robot_ws_base_rl")
sys.path.insert(
    0, str(W / "src/tools/thesis_experiment/scripts")
)
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_actual_teb_matrix.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_frozen_runner", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r16_actual_teb_paired_preflight_contract.yaml"
)
SCENE_CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
GEOMETRY = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r10_progress_recovery_geometry_contract.yaml"
)
PREDICTOR = W / (
    "artifacts/v3/multiactor/v3_13_q1_r13_predictor/"
    "expanded_progress_predictor.yaml"
)
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r16_readiness"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r16_actual_teb_preflight"
)
STAGE = "V3-13-Q1-R16"
UNITS = 2


def closed_overlay_environment(runtime_home, ros_port):
    environment = BASE.common.clean_environment(runtime_home, ros_port)
    environment.update(
        {
            "ROS_PACKAGE_PATH": (
                "/home/robot/robot_ws_base_rl/src:"
                "/opt/ros/noetic/share"
            ),
            "CMAKE_PREFIX_PATH": (
                "/home/robot/robot_ws_base_rl/devel:"
                "/opt/ros/noetic"
            ),
            "PYTHONPATH": (
                "/home/robot/robot_ws_base_rl/devel/"
                "lib/python3/dist-packages:"
                "/opt/ros/noetic/lib/python3/dist-packages"
            ),
            "LD_LIBRARY_PATH": (
                "/home/robot/robot_ws_base_rl/devel/lib:"
                "/opt/ros/noetic/lib:"
                "/opt/ros/noetic/lib/x86_64-linux-gnu"
            ),
        }
    )
    return environment


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
    world, actors = BASE.design.render_world(
        scene, row["actor_policies_evaluator_only"], row["seed"]
    )
    BASE.atomic_bytes(world_path, world.encode("utf-8"))
    BASE.atomic_yaml(scene_path, scene)
    BASE.atomic_yaml(
        actors_path,
        {
            "schema_version": "1.0",
            "stage": STAGE,
            "actors": actors,
        },
    )
    command = [
        "roslaunch",
        "m2_gazebo",
        "m2_v3_13_progress_actual_teb_preflight.launch",
        "world:={}".format(world_path.resolve()),
        "method:={}".format(row["method"]),
        "seed:={}".format(row["seed"]),
        "scene_id:={}".format(row["scene_id"]),
        "block_index:={}".format(row["block_index"]),
        "report_path:={}".format(report_path.resolve()),
        "scene_spec_path:={}".format(scene_path.resolve()),
        "actor_specs_path:={}".format(actors_path.resolve()),
        "planner_contract_path:={}".format(
            SCENE_CONTRACT.resolve()
        ),
        "geometry_contract_path:={}".format(GEOMETRY.resolve()),
        "predictor_model_path:={}".format(PREDICTOR.resolve()),
        "gui:=false",
    ]
    started = time.monotonic()
    with log_path.open("wb") as stream:
        process = subprocess.Popen(
            command,
            cwd=str(W),
            env=closed_overlay_environment(runtime_home, ros_port),
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
            return_code = BASE.stop_process(process)
        else:
            BASE.stop_process(process)
    if not report_path.is_file():
        tail = BASE.read_bytes(log_path)[-10000:].decode(
            "utf-8", errors="replace"
        )
        raise RuntimeError(
            "episode report absent rc={} timeout={}\n{}".format(
                return_code, timed_out, tail
            )
        )
    report = yaml.safe_load(BASE.read_bytes(report_path))
    receipt = {
        "schema_version": "1.0",
        "stage": STAGE,
        "row": row,
        "episode_report": report,
        "episode_report_sha256": BASE.sha(
            BASE.read_bytes(report_path)
        ),
        "world_sha256": BASE.sha(BASE.read_bytes(world_path)),
        "scene_spec_sha256": BASE.sha(BASE.read_bytes(scene_path)),
        "actor_specs_sha256": BASE.sha(BASE.read_bytes(actors_path)),
        "roslaunch_log_sha256": BASE.sha(BASE.read_bytes(log_path)),
        "roslaunch_return_code": return_code,
        "wall_elapsed_s": time.monotonic() - started,
        "timed_out": timed_out,
        "credential_environment_scrubbed": True,
        "workspace_overlay_explicit": True,
        "workspace_overlay_root": str(W),
        "foreign_workspace_precedence_used": False,
    }
    BASE.atomic_yaml(root / "matrix_receipt.yaml", receipt)
    return receipt


def summarize(receipts, contract_sha, contract):
    summary = BASE.summarize(receipts, "preflight", contract_sha)
    summary["stage"] = STAGE
    summary["status"] = "paired_integration_collection_complete"
    reports = {
        receipt["row"]["method"]: receipt["episode_report"]
        for receipt in receipts
    }
    actual = reports["actual_teb"]
    v3 = reports["v3_13_analytic"]
    block = summary["blocks"][0]
    actor_differences = block["paired_gate_actor_differences"]
    gates = contract["integration_gates"]
    v3_decisions = v3["result"]["decisions"]
    checks = {
        "complete_paired_blocks": (
            summary["complete_paired_blocks"]
            == int(gates["complete_paired_blocks_exact"])
        ),
        "all_reports_finite": summary["all_reports_finite"],
        "paired_gate_ego_x": (
            summary["maximum_paired_gate_ego_x_difference_m"]
            <= float(gates["paired_gate_ego_x_difference_max_m"])
        ),
        "paired_gate_actor_coordinate": all(
            row["coordinate_difference_m"]
            <= float(
                gates[
                    "paired_gate_actor_coordinate_difference_max_m"
                ]
            )
            for row in actor_differences
        ),
        "paired_gate_actor_speed": all(
            row["speed_difference_mps"]
            <= float(
                gates["paired_gate_actor_speed_difference_max_mps"]
            )
            for row in actor_differences
        ),
        "actual_teb_local_plan": (
            actual["result"]["local_plan_count"]
            >= int(gates["actual_teb_local_plan_count_min"])
        ),
        "actual_teb_command": (
            actual["result"]["command_count"]
            >= int(gates["actual_teb_command_count_min"])
        ),
        "v3_decision_analytic_accept": (
            sum(
                decision["selected_evaluation"]["accepted"]
                for decision in v3_decisions
            )
            / len(v3_decisions)
            >= float(gates["v3_decision_analytic_accept_rate_min"])
        ),
        "v3_eventual_progress": bool(
            v3["result"]["eventual_progress"]
        ),
        "v3_goal_reached": bool(v3["result"]["goal_reached"]),
        "v3_contact_state_count": (
            v3["result"]["contact_state_count"]
            <= int(gates["v3_contact_state_count_max"])
        ),
        "v3_conservative_clearance": (
            v3["result"]["minimum_conservative_clearance_m"]
            >= float(
                gates[
                    "v3_minimum_conservative_clearance_m_min"
                ]
            )
        ),
        "v3_tracking_error": (
            v3["result"]["tracking_error_max_m"] is not None
            and v3["result"]["tracking_error_max_m"]
            <= float(gates["v3_tracking_error_max_m"])
        ),
    }
    summary["integration_gate_checks"] = checks
    summary["collection_gate_pass"] = all(checks.values())
    summary["actual_teb_outcome_retained_without_pass_requirement"] = {
        "episode_pass": actual["episode_pass"],
        "goal_reached": actual["result"]["goal_reached"],
        "timeout": actual["result"]["timeout"],
        "restricted_elapsed_s": actual["result"][
            "restricted_elapsed_s"
        ],
        "minimum_conservative_clearance_m": actual["result"][
            "minimum_conservative_clearance_m"
        ],
        "contact_state_count": actual["result"][
            "contact_state_count"
        ],
        "local_plan_count": actual["result"]["local_plan_count"],
        "command_count": actual["result"]["command_count"],
        "action_state_name": actual["result"]["action_state_name"],
    }
    summary["v3_outcome"] = {
        "episode_pass": v3["episode_pass"],
        "goal_reached": v3["result"]["goal_reached"],
        "restricted_elapsed_s": v3["result"]["restricted_elapsed_s"],
        "minimum_conservative_clearance_m": v3["result"][
            "minimum_conservative_clearance_m"
        ],
        "contact_state_count": v3["result"]["contact_state_count"],
        "initial_stop": v3["result"]["initial_stop"],
        "eventual_progress": v3["result"]["eventual_progress"],
        "selected_candidate_ids": [
            decision["selected_candidate_id"]
            for decision in v3_decisions
        ],
    }
    summary["included_in_qualification_or_performance_evidence"] = False
    summary["actual_TEB_performance_claim_authorized"] = False
    summary["training_authorized"] = False
    return summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--contract-sha256", required=True)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--review-sha256", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    args = parser.parse_args()
    if Path(args.output_root).resolve() != OUTPUT.resolve():
        raise RuntimeError("canonical output root differs")
    if OUTPUT.exists():
        raise RuntimeError("output exists; retry or resume forbidden")
    release_payload = BASE.read_bytes(
        READINESS / "execution_release.yaml"
    )
    review_payload = BASE.read_bytes(
        READINESS / "independent_review.yaml"
    )
    if BASE.sha(release_payload) != args.release_sha256:
        raise RuntimeError("release SHA differs")
    if BASE.sha(review_payload) != args.review_sha256:
        raise RuntimeError("review SHA differs")
    release = yaml.safe_load(release_payload)
    review = yaml.safe_load(review_payload)
    contract_payload = BASE.read_bytes(CONTRACT)
    contract_sha = BASE.sha(contract_payload)
    if args.contract_sha256 != contract_sha:
        raise RuntimeError("contract SHA differs")
    if release["contract_sha256"] != contract_sha:
        raise RuntimeError("release does not bind contract")
    if not release["execution_authorized"]:
        raise RuntimeError("execution not authorized")
    if int(release["authorized_units"]) != UNITS:
        raise RuntimeError("authorized units differ")
    if review["execution_release_sha256"] != args.release_sha256:
        raise RuntimeError("review does not bind release")
    contract = yaml.safe_load(contract_payload)
    rows = yaml.safe_load(
        BASE.read_bytes(READINESS / "exact_schedule.yaml")
    )["rows"]
    if len(rows) != UNITS:
        raise RuntimeError("schedule row count differs")
    scenes = {
        item["scene_id"]: item
        for item in yaml.safe_load(
            BASE.read_bytes(SCENE_CONTRACT)
        )["scene_families"]
    }
    OUTPUT.mkdir(parents=True)
    BASE.atomic_yaml(
        OUTPUT / "exact_schedule.yaml",
        {
            "schema_version": "1.0",
            "architecture_generation": "v3",
            "stage": STAGE,
            "contract_sha256": contract_sha,
            "row_count": len(rows),
            "rows": rows,
        },
    )
    journal = OUTPUT / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    BASE.append_jsonl(
        journal,
        {
            "event": "execution_opened",
            "stage": STAGE,
            "authorized_units": UNITS,
            "contract_sha256": contract_sha,
        },
    )
    receipts = []
    for row in rows:
        BASE.append_jsonl(
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
                row["method"],
            )
        )
        try:
            receipt = run_episode(
                row,
                scenes[row["scene_id"]],
                root,
                32300 + 2 * row["sequence"],
            )
        except Exception as error:
            BASE.append_jsonl(
                journal,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "row": row,
                    "error": "{}:{}".format(
                        type(error).__name__, error
                    ),
                    "remaining_units_forfeited": (
                        UNITS - row["sequence"]
                    ),
                },
            )
            raise
        receipts.append(receipt)
        BASE.append_jsonl(
            journal,
            {
                "event": "unit_complete",
                "sequence": row["sequence"],
                "row": row,
                "episode_report_sha256": receipt[
                    "episode_report_sha256"
                ],
                "episode_pass": receipt["episode_report"][
                    "episode_pass"
                ],
            },
        )
        report = receipt["episode_report"]
        print(
            "[{}] {}/{} method={} episode_pass={} goal={} "
            "clearance={:.3f}m".format(
                STAGE,
                row["sequence"],
                UNITS,
                row["method"],
                report["episode_pass"],
                report["result"]["goal_reached"],
                report["result"][
                    "minimum_conservative_clearance_m"
                ],
            ),
            flush=True,
        )
    summary = summarize(receipts, contract_sha, contract)
    BASE.atomic_yaml(OUTPUT / "collection_summary.yaml", summary)
    BASE.append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "authorized_units": UNITS,
            "consumed_units": UNITS,
            "completed_units": UNITS,
            "forfeited_units": 0,
            "summary_sha256": BASE.sha(
                BASE.read_bytes(OUTPUT / "collection_summary.yaml")
            ),
        },
    )
    print(
        "status={} collection_gate_pass={} summary_sha256={}".format(
            summary["status"],
            summary["collection_gate_pass"],
            BASE.sha(
                BASE.read_bytes(OUTPUT / "collection_summary.yaml")
            ),
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
