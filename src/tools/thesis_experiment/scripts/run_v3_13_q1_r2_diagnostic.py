#!/usr/bin/env python3
"""Run the one-unit V3-13-Q1-R2 diagnostic release."""

import argparse
import importlib.util
import json
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
    "v3_13_q1_r1_environment", R1_RUNNER_PATH
)
R1 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R1)
COMMON = R1.BASE
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r2_diagnostic_contract.yaml"
)
PLANNER_CONTRACT = COMMON.PLANNER_CONTRACT
PREDICTOR = COMMON.PREDICTOR
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r2_readiness"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/v3_13_q1_r2_diagnostic"
)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--contract-sha256", required=True)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--review-sha256", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    arguments = parser.parse_args()
    if Path(arguments.output_root).resolve() != OUTPUT.resolve():
        raise RuntimeError("canonical output root differs")
    if OUTPUT.exists():
        raise RuntimeError("output exists; retry forbidden")
    release_path = READINESS / "execution_release.yaml"
    review_path = READINESS / "independent_review.yaml"
    schedule_path = READINESS / "exact_schedule.yaml"
    release_payload = COMMON.read_bytes(release_path)
    review_payload = COMMON.read_bytes(review_path)
    if COMMON.sha(release_payload) != arguments.release_sha256:
        raise RuntimeError("release SHA differs")
    if COMMON.sha(review_payload) != arguments.review_sha256:
        raise RuntimeError("review SHA differs")
    release = yaml.safe_load(release_payload)
    review = yaml.safe_load(review_payload)
    contract_payload = COMMON.read_bytes(CONTRACT)
    contract_sha = COMMON.sha(contract_payload)
    if contract_sha != arguments.contract_sha256:
        raise RuntimeError("contract SHA differs")
    if release["contract_sha256"] != contract_sha:
        raise RuntimeError("release contract differs")
    if review["execution_release_sha256"] != arguments.release_sha256:
        raise RuntimeError("review release differs")
    if not release["execution_authorized"]:
        raise RuntimeError("execution not authorized")
    rows = yaml.safe_load(COMMON.read_bytes(schedule_path))["rows"]
    if len(rows) != 1:
        raise RuntimeError("diagnostic row count")
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
            "stage": "V3-13-Q1-R2-D1",
            "contract_sha256": contract_sha,
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
            "contract_sha256": contract_sha,
        },
    )
    COMMON.append_jsonl(
        journal,
        {"event": "unit_consumed", "sequence": 1, "row": row},
    )
    root = OUTPUT / "episodes/001_01_7331_v3_13_analytic"
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
        scenes[row["scene_id"]],
        row["actor_policies_evaluator_only"],
        row["seed"],
    )
    COMMON.atomic_bytes(world_path, world.encode("utf-8"))
    COMMON.atomic_yaml(scene_path, scenes[row["scene_id"]])
    COMMON.atomic_yaml(
        actors_path,
        {
            "schema_version": "1.0",
            "stage": "V3-13-Q1-R2-D1",
            "actors": actors,
        },
    )
    command = [
        "roslaunch",
        "m2_gazebo",
        "m2_v3_13_actual_teb_diagnostic_episode.launch",
        "world:={}".format(world_path.resolve()),
        "seed:={}".format(row["seed"]),
        "scene_id:={}".format(row["scene_id"]),
        "block_index:=1",
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
    with log_path.open("wb") as stream:
        process = subprocess.Popen(
            command,
            cwd=str(W),
            env=R1.closed_overlay_environment(runtime_home, 30802),
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
        raise RuntimeError("diagnostic report absent")
    report = yaml.safe_load(COMMON.read_bytes(report_path))
    if not report.get("diagnostic_complete"):
        raise RuntimeError("diagnostic incomplete")
    receipt = {
        "schema_version": "1.0",
        "stage": "V3-13-Q1-R2-D1",
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
    COMMON.append_jsonl(
        journal,
        {
            "event": "unit_complete",
            "sequence": 1,
            "row": row,
            "episode_report_sha256": receipt[
                "episode_report_sha256"
            ],
            "diagnostic_complete": True,
        },
    )
    summary = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-Q1-R2-D1",
        "status": "one_unit_read_only_diagnostic_complete",
        "authorized_units": 1,
        "consumed_units": 1,
        "completed_units": 1,
        "forfeited_units": 0,
        "timed_out": timed_out,
        "planning_diagnostic_count": len(
            report["planning_diagnostics"]
        ),
        "planner_failure_observed": bool(
            report.get("failure")
        ),
        "action_override_or_safety_bypass_used": False,
        "policy_or_ranker_training_steps": 0,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    COMMON.atomic_yaml(OUTPUT / "diagnostic_summary.yaml", summary)
    COMMON.append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "consumed_units": 1,
            "completed_units": 1,
            "forfeited_units": 0,
            "summary_sha256": COMMON.sha(
                COMMON.read_bytes(OUTPUT / "diagnostic_summary.yaml")
            ),
        },
    )
    print(
        "status={} diagnostics={} failure={} report_sha256={}".format(
            summary["status"],
            summary["planning_diagnostic_count"],
            summary["planner_failure_observed"],
            receipt["episode_report_sha256"],
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
