#!/usr/bin/env python3
"""Run exact one-unit R19-R1 boundary-atomic preflight."""

import argparse
import importlib.util
from pathlib import Path
import subprocess
import time

import yaml

import v3_13_multiactor_design as design


W = Path("/home/robot/robot_ws_base_rl")
R1_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r1_actual_teb_matrix.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r19_r1_environment", R1_PATH
)
R1 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R1)
COMMON = R1.BASE
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r1_boundary_atomic_preflight_contract.yaml"
)
GEOMETRY = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r17_goal_aligned_recovery_contract.yaml"
)
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r19_r1_readiness"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_r1_boundary_atomic_preflight"
)


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
    contract_payload = COMMON.read_bytes(CONTRACT)
    release_payload = COMMON.read_bytes(
        READINESS / "execution_release.yaml"
    )
    review_payload = COMMON.read_bytes(
        READINESS / "independent_review.yaml"
    )
    for payload, expected, label in (
        (contract_payload, args.contract_sha256, "contract"),
        (release_payload, args.release_sha256, "release"),
        (review_payload, args.review_sha256, "review"),
    ):
        if COMMON.sha(payload) != expected:
            raise RuntimeError("{} SHA differs".format(label))
    release = yaml.safe_load(release_payload)
    review = yaml.safe_load(review_payload)
    if not release["execution_authorized"]:
        raise RuntimeError("execution not authorized")
    if review["execution_release_sha256"] != args.release_sha256:
        raise RuntimeError("review release differs")
    rows = yaml.safe_load(
        COMMON.read_bytes(READINESS / "exact_schedule.yaml")
    )["rows"]
    if len(rows) != 1 or rows[0]["seed"] != 7677:
        raise RuntimeError("schedule identity differs")
    row = rows[0]
    d0 = yaml.safe_load(COMMON.read_bytes(COMMON.SCENE_CONTRACT))
    scenes = {
        scene["scene_id"]: scene for scene in d0["scene_families"]
    }
    OUTPUT.mkdir(parents=True)
    COMMON.atomic_yaml(
        OUTPUT / "exact_schedule.yaml",
        {
            "schema_version": "1.0",
            "stage": "V3-13-Q1-R19-R1",
            "contract_sha256": args.contract_sha256,
            "row_count": 1,
            "rows": rows,
        },
    )
    journal = OUTPUT / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    COMMON.append_jsonl(
        journal,
        {
            "event": "execution_opened",
            "authorized_units": 1,
            "contract_sha256": args.contract_sha256,
        },
    )
    COMMON.append_jsonl(
        journal,
        {"event": "unit_consumed", "sequence": 1, "row": row},
    )
    root = OUTPUT / (
        "episodes/001_01_7677_recovery_evasive_right_v120_r3"
    )
    root.mkdir(parents=True, exist_ok=False)
    world_path = root / "world.sdf"
    scene_path = root / "scene_spec.yaml"
    actors_path = root / "actor_specs.yaml"
    report_path = root / "episode_report.yaml"
    log_path = root / "roslaunch.log"
    runtime_home = root / "runtime_home"
    runtime_home.mkdir()
    (runtime_home / "roslog").mkdir()
    scene = scenes[row["scene_id"]]
    world, actors = design.render_world(
        scene, row["actor_policies_evaluator_only"], row["seed"]
    )
    COMMON.atomic_bytes(world_path, world.encode("utf-8"))
    COMMON.atomic_yaml(scene_path, scene)
    COMMON.atomic_yaml(
        actors_path,
        {
            "schema_version": "1.0",
            "stage": "V3-13-Q1-R19-R1",
            "actors": actors,
        },
    )
    command = [
        "roslaunch",
        "m2_gazebo",
        "m2_v3_13_goal_aligned_boundary_atomic_preflight.launch",
        "world:={}".format(world_path.resolve()),
        "seed:=7677",
        "scene_id:=crossing_and_oncoming",
        "block_index:=1",
        "pre_action_wait_s:=5.0",
        "branch_target_sim_s:=12.5",
        "forced_candidate_id:=recovery_evasive_right_v120_r3",
        "behavior_propensity:=1.0",
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
            env=R1.closed_overlay_environment(runtime_home, 31620),
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
        tail = COMMON.read_bytes(log_path)[-10000:].decode(
            "utf-8", errors="replace"
        )
        COMMON.append_jsonl(
            journal,
            {
                "event": "terminal_failure",
                "sequence": 1,
                "row": row,
                "error": (
                    "episode_report_absent rc={} timeout={}:{}"
                ).format(return_code, timed_out, tail),
                "remaining_units_forfeited": 0,
            },
        )
        raise RuntimeError("episode report absent")
    report = yaml.safe_load(COMMON.read_bytes(report_path))
    receipt = {
        "schema_version": "1.0",
        "stage": "V3-13-Q1-R19-R1",
        "row": row,
        "episode_report": report,
        "episode_report_sha256": COMMON.sha(
            COMMON.read_bytes(report_path)
        ),
        "world_sha256": COMMON.sha(COMMON.read_bytes(world_path)),
        "scene_spec_sha256": COMMON.sha(COMMON.read_bytes(scene_path)),
        "actor_specs_sha256": COMMON.sha(COMMON.read_bytes(actors_path)),
        "roslaunch_log_sha256": COMMON.sha(COMMON.read_bytes(log_path)),
        "roslaunch_return_code": return_code,
        "wall_elapsed_s": time.monotonic() - started,
        "timed_out": timed_out,
        "credential_environment_scrubbed": True,
        "workspace_overlay_explicit": True,
        "foreign_workspace_precedence_used": False,
    }
    COMMON.atomic_yaml(root / "matrix_receipt.yaml", receipt)
    COMMON.append_jsonl(
        journal,
        {
            "event": "unit_complete",
            "sequence": 1,
            "row": row,
            "episode_report_sha256": receipt["episode_report_sha256"],
            "collection_pass": report["episode_collection_pass"],
            "scientific_gate_pass": report[
                "preflight_scientific_gate_pass"
            ],
        },
    )
    result = report["result"]
    summary = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R1",
        "status": "one_unit_boundary_atomic_preflight_complete",
        "authorized_units": 1,
        "consumed_units": 1,
        "completed_units": 1,
        "forfeited_units": 0,
        "seed": 7677,
        "attempt": 1,
        "timed_out": timed_out,
        "roslaunch_return_code": return_code,
        "episode_collection_pass": report["episode_collection_pass"],
        "preflight_scientific_gate_pass": report[
            "preflight_scientific_gate_pass"
        ],
        "minimum_conservative_clearance_m": result[
            "minimum_conservative_clearance_m"
        ],
        "minimum_trace_static_clearance_m": report[
            "minimum_trace_static_clearance_m"
        ],
        "contact_state_count": result["contact_state_count"],
        "tracking_error_max_m": result["tracking_error_max_m"],
        "final_common_goal_distance_m": report[
            "final_common_goal_distance_m"
        ],
        "terminal_guard_activation_count": report[
            "terminal_guard_activation_count"
        ],
        "boundary_terminal_evaluation_count": report[
            "boundary_terminal_evaluation_count"
        ],
        "boundary_terminal_trigger_count": report[
            "boundary_terminal_trigger_count"
        ],
        "actual_TEB_execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    COMMON.atomic_yaml(OUTPUT / "collection_summary.yaml", summary)
    COMMON.append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "consumed_units": 1,
            "completed_units": 1,
            "forfeited_units": 0,
            "scientific_gate_pass": report[
                "preflight_scientific_gate_pass"
            ],
            "summary_sha256": COMMON.sha(
                COMMON.read_bytes(OUTPUT / "collection_summary.yaml")
            ),
        },
    )
    print(
        "status={} pass={} dynamic={:.6f} static={:.6f} "
        "goal={:.6f} guard={} boundary={}/{} summary_sha256={}".format(
            summary["status"],
            summary["preflight_scientific_gate_pass"],
            summary["minimum_conservative_clearance_m"],
            summary["minimum_trace_static_clearance_m"],
            summary["final_common_goal_distance_m"],
            summary["terminal_guard_activation_count"],
            summary["boundary_terminal_evaluation_count"],
            summary["boundary_terminal_trigger_count"],
            COMMON.sha(
                COMMON.read_bytes(OUTPUT / "collection_summary.yaml")
            ),
        )
    )


if __name__ == "__main__":
    main()
