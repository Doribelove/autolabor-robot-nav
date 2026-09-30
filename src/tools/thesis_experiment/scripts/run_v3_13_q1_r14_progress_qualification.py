#!/usr/bin/env python3
"""Run R14 by reusing the audited R12 matrix orchestration."""

import copy
import importlib.util
from pathlib import Path
import subprocess
import time

import yaml


W = Path("/home/robot/robot_ws_base_rl")
R12_PATH = W / (
    "src/tools/thesis_experiment/scripts/"
    "run_v3_13_q1_r12_progress_support.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r12_frozen_runner", R12_PATH
)
R12 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R12)
COMMON = R12.COMMON
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r14_progress_predictor_qualification_contract.yaml"
)
GEOMETRY = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r10_progress_recovery_geometry_contract.yaml"
)
MODEL = W / (
    "artifacts/v3/multiactor/v3_13_q1_r13_predictor/"
    "expanded_progress_predictor.yaml"
)
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r14_readiness"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/v3_13_q1_r14_progress_qualification"
)
ORIGINAL_SUMMARIZE = R12.summarize


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
    world, actors = R12.design.render_world(
        scene, row["actor_policies_evaluator_only"], row["seed"]
    )
    COMMON.atomic_bytes(world_path, world.encode("utf-8"))
    COMMON.atomic_yaml(scene_path, scene)
    COMMON.atomic_yaml(
        actors_path,
        {
            "schema_version": "1.0",
            "stage": "V3-13-Q1-R14",
            "actors": actors,
        },
    )
    command = [
        "roslaunch",
        "m2_gazebo",
        "m2_v3_13_progress_predictor_qualification.launch",
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
        "predictor_model_path:={}".format(MODEL.resolve()),
        "gui:=false",
    ]
    started = time.monotonic()
    with log_path.open("wb") as stream:
        process = subprocess.Popen(
            command,
            cwd=str(W),
            env=R12.R1.closed_overlay_environment(
                runtime_home, ros_port
            ),
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
        "stage": "V3-13-Q1-R14",
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
    return receipt


def summarize(receipts, contract_sha, contract):
    compatible = (
        {
            "population": {
                "actions": contract["qualification_population"][
                    "actions"
                ]
            },
            "matching_gates": contract["matching_gates"],
        }
        if "qualification_population" in contract
        else contract
    )
    value = R12.R6.summarize(
        receipts, contract_sha, compatible
    )
    value["stage"] = "V3-13-Q1-R14"
    value["status"] = (
        "fresh_progress_predictor_qualification_collection_pass"
        if value["collection_gate_pass"]
        else "fresh_progress_predictor_qualification_collection_fail"
    )
    value["static_outcomes_retained"] = True
    value["minimum_trace_static_clearance_m"] = min(
        float(receipt["episode_report"][
            "minimum_trace_static_clearance_m"
        ])
        for receipt in receipts
    )
    return value


class _AssessorCompatibility:
    summarize = staticmethod(summarize)


R6 = _AssessorCompatibility
R12.CONTRACT = CONTRACT
R12.GEOMETRY = GEOMETRY
R12.READINESS = READINESS
R12.OUTPUT = OUTPUT
R12.run_episode = run_episode
R12.summarize = summarize
ORIGINAL_ATOMIC_YAML = COMMON.atomic_yaml


def stage_correcting_atomic_yaml(path, value):
    if (
        Path(path).name == "exact_schedule.yaml"
        and Path(path).parent.resolve() == OUTPUT.resolve()
    ):
        value = copy.deepcopy(value)
        value["stage"] = "V3-13-Q1-R14"
    return ORIGINAL_ATOMIC_YAML(path, value)


COMMON.atomic_yaml = stage_correcting_atomic_yaml


if __name__ == "__main__":
    raise SystemExit(R12.main())
