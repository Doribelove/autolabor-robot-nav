#!/usr/bin/env python3
"""Run fresh R19-R5-R1 with the repaired 0.20 propensity binding."""

import argparse
import importlib.util
from pathlib import Path
import subprocess
import time

import yaml

import v3_13_multiactor_design as design


W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r19_r5_terminal_tolerance_preflight.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r19_r5_r1_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
COMMON = BASE.COMMON
R1 = BASE.R1
GEOMETRY = BASE.GEOMETRY
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r5_r1_propensity_binding_preflight_contract.yaml"
)
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r19_r5_r1_readiness"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_r5_r1_propensity_preflight"
)


def run_episode(row, scene, root, ros_port):
    root.mkdir(parents=True, exist_ok=False)
    paths = {
        name: root / name
        for name in (
            "world.sdf",
            "scene_spec.yaml",
            "actor_specs.yaml",
            "episode_report.yaml",
            "roslaunch.log",
        )
    }
    runtime_home = root / "runtime_home"
    runtime_home.mkdir()
    (runtime_home / "roslog").mkdir()
    world, actors = design.render_world(
        scene, row["actor_policies_evaluator_only"], row["seed"]
    )
    COMMON.atomic_bytes(paths["world.sdf"], world.encode("utf-8"))
    COMMON.atomic_yaml(paths["scene_spec.yaml"], scene)
    COMMON.atomic_yaml(
        paths["actor_specs.yaml"],
        {
            "schema_version": "1.0",
            "stage": "V3-13-Q1-R19-R5-R1",
            "actors": actors,
        },
    )
    command = [
        "roslaunch", "m2_gazebo",
        "m2_v3_13_tolerance_repaired_prefix_aligned_bank_clone_episode.launch",
        "world:={}".format(paths["world.sdf"].resolve()),
        "seed:={}".format(row["seed"]),
        "scene_id:={}".format(row["scene_id"]),
        "block_index:=1",
        "pre_action_wait_s:=5.0",
        "branch_target_sim_s:=12.5",
        "forced_candidate_id:=recovery_evasive_right_v120_r3",
        "behavior_propensity:=0.20",
        "horizon_s:=7.55",
        "report_path:={}".format(paths["episode_report.yaml"].resolve()),
        "scene_spec_path:={}".format(paths["scene_spec.yaml"].resolve()),
        "actor_specs_path:={}".format(paths["actor_specs.yaml"].resolve()),
        "planner_contract_path:={}".format(
            COMMON.PLANNER_CONTRACT.resolve()
        ),
        "geometry_contract_path:={}".format(GEOMETRY.resolve()),
        "gui:=false",
    ]
    started = time.monotonic()
    with paths["roslaunch.log"].open("wb") as stream:
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
    if not paths["episode_report.yaml"].is_file():
        tail = COMMON.read_bytes(paths["roslaunch.log"])[-8000:].decode(
            "utf-8", errors="replace"
        )
        raise RuntimeError(
            "episode report absent rc={} timeout={}\n{}".format(
                return_code, timed_out, tail
            )
        )
    report = yaml.safe_load(COMMON.read_bytes(paths["episode_report.yaml"]))
    receipt = {
        "schema_version": "1.0",
        "stage": "V3-13-Q1-R19-R5-R1",
        "row": row,
        "episode_report": report,
        "episode_report_sha256": COMMON.sha(
            COMMON.read_bytes(paths["episode_report.yaml"])
        ),
        "world_sha256": COMMON.sha(COMMON.read_bytes(paths["world.sdf"])),
        "scene_spec_sha256": COMMON.sha(
            COMMON.read_bytes(paths["scene_spec.yaml"])
        ),
        "actor_specs_sha256": COMMON.sha(
            COMMON.read_bytes(paths["actor_specs.yaml"])
        ),
        "roslaunch_log_sha256": COMMON.sha(
            COMMON.read_bytes(paths["roslaunch.log"])
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
    files = (
        CONTRACT,
        READINESS / "execution_release.yaml",
        READINESS / "independent_review.yaml",
    )
    expected = (
        args.contract_sha256, args.release_sha256, args.review_sha256
    )
    payloads = [COMMON.read_bytes(path) for path in files]
    if any(COMMON.sha(p) != e for p, e in zip(payloads, expected)):
        raise RuntimeError("release binding differs")
    release = yaml.safe_load(payloads[1])
    review = yaml.safe_load(payloads[2])
    if (
        not release["execution_authorized"]
        or review["execution_release_sha256"] != args.release_sha256
    ):
        raise RuntimeError("execution not independently released")
    schedule = yaml.safe_load(
        COMMON.read_bytes(READINESS / "exact_schedule.yaml")
    )
    row = schedule["rows"][0]
    scenes_doc = yaml.safe_load(COMMON.read_bytes(COMMON.SCENE_CONTRACT))
    scenes = {s["scene_id"]: s for s in scenes_doc["scene_families"]}
    OUTPUT.mkdir(parents=True)
    COMMON.atomic_yaml(
        OUTPUT / "exact_schedule.yaml",
        {
            "schema_version": "1.0",
            "stage": "V3-13-Q1-R19-R5-R1",
            "contract_sha256": args.contract_sha256,
            "row_count": 1,
            "rows": [row],
        },
    )
    journal = OUTPUT / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    COMMON.append_jsonl(journal, {
        "event": "execution_opened", "authorized_units": 1,
        "contract_sha256": args.contract_sha256,
    })
    COMMON.append_jsonl(
        journal, {"event": "unit_consumed", "sequence": 1, "row": row}
    )
    root = OUTPUT / (
        "episodes/001_01_7696_recovery_evasive_right_v120_r3"
    )
    try:
        receipt = run_episode(row, scenes[row["scene_id"]], root, 32104)
    except Exception as error:
        COMMON.append_jsonl(journal, {
            "event": "terminal_failure", "sequence": 1, "row": row,
            "error": "{}:{}".format(type(error).__name__, error),
            "remaining_units_forfeited": 0,
        })
        raise
    report = receipt["episode_report"]
    COMMON.append_jsonl(journal, {
        "event": "unit_complete", "sequence": 1, "row": row,
        "episode_report_sha256": receipt["episode_report_sha256"],
        "collection_pass": report["episode_collection_pass"],
    })
    summary = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R19-R5-R1",
        "status": "fresh_repaired_propensity_preflight_collected",
        "authorized_units": 1,
        "consumed_units": 1,
        "completed_units": 1,
        "forfeited_units": 0,
        "episode_collection_pass": report["episode_collection_pass"],
        "minimum_conservative_clearance_m": report["result"][
            "minimum_conservative_clearance_m"
        ],
        "minimum_trace_static_clearance_m": report[
            "minimum_trace_static_clearance_m"
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
    }
    COMMON.atomic_yaml(OUTPUT / "collection_summary.yaml", summary)
    COMMON.append_jsonl(journal, {
        "event": "execution_complete", "consumed_units": 1,
        "completed_units": 1, "forfeited_units": 0,
        "summary_sha256": COMMON.sha(
            COMMON.read_bytes(OUTPUT / "collection_summary.yaml")
        ),
    })
    print(
        "status={} dynamic={:.6f} guard={} boundary_trigger={} "
        "summary_sha256={}".format(
            summary["status"],
            summary["minimum_conservative_clearance_m"],
            summary["terminal_guard_activation_count"],
            summary["boundary_terminal_trigger_count"],
            COMMON.sha(COMMON.read_bytes(OUTPUT / "collection_summary.yaml")),
        )
    )


if __name__ == "__main__":
    main()
