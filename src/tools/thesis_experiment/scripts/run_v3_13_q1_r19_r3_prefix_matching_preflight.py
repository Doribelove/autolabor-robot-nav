#!/usr/bin/env python3
"""Run the exact five-unit R19-R3 prefix-matching preflight."""

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
    "run_v3_13_q1_r19_r2_boundary_atomic_support.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_q1_r19_r3_runner_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)
COMMON = BASE.COMMON
R1 = BASE.R1
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r3_prefix_matching_preflight_contract.yaml"
)
GEOMETRY = BASE.GEOMETRY
READINESS = W / (
    "artifacts/v3/multiactor/v3_13_q1_r19_r3_readiness"
)
OUTPUT = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_r3_prefix_matching_preflight"
)
R3 = BASE.R3


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
            "stage": "V3-13-Q1-R19-R3",
            "actors": actors,
        },
    )
    horizon_s = 7.55 if row["forced_candidate_id"] == R3 else 7.0
    command = [
        "roslaunch",
        "m2_gazebo",
        "m2_v3_13_prefix_aligned_boundary_atomic_bank_clone_episode.launch",
        "world:={}".format(world_path.resolve()),
        "seed:={}".format(row["seed"]),
        "scene_id:={}".format(row["scene_id"]),
        "block_index:={}".format(row["block_index"]),
        "pre_action_wait_s:=5.0",
        "branch_target_sim_s:=12.5",
        "forced_candidate_id:={}".format(row["forced_candidate_id"]),
        "behavior_propensity:=0.20",
        "horizon_s:={}".format(horizon_s),
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
        "stage": "V3-13-Q1-R19-R3",
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
    value = BASE.summarize(receipts, contract_sha, contract)
    value["stage"] = "V3-13-Q1-R19-R3"
    value["authorized_units"] = 5
    value["consumed_units"] = len(receipts)
    value["completed_units"] = len(receipts)
    value["gate_checks"]["completed_units_exact"] = len(receipts) == 5
    value["collection_gate_pass"] = all(value["gate_checks"].values())
    value["status"] = (
        "fresh_five_action_prefix_matching_preflight_pass"
        if value["collection_gate_pass"]
        else "fresh_five_action_prefix_matching_preflight_fail"
    )
    value["non_evidence_preflight"] = True
    value["predictor_fit_authorized"] = False
    return value


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
    payloads = [
        COMMON.read_bytes(CONTRACT),
        COMMON.read_bytes(READINESS / "execution_release.yaml"),
        COMMON.read_bytes(READINESS / "independent_review.yaml"),
    ]
    for payload, expected, label in zip(
        payloads,
        (
            args.contract_sha256,
            args.release_sha256,
            args.review_sha256,
        ),
        ("contract", "release", "review"),
    ):
        if COMMON.sha(payload) != expected:
            raise RuntimeError("{} SHA differs".format(label))
    release = yaml.safe_load(payloads[1])
    review = yaml.safe_load(payloads[2])
    if not release["execution_authorized"]:
        raise RuntimeError("execution not authorized")
    if review["execution_release_sha256"] != args.release_sha256:
        raise RuntimeError("review release differs")
    rows = yaml.safe_load(
        COMMON.read_bytes(READINESS / "exact_schedule.yaml")
    )["rows"]
    if len(rows) != 5:
        raise RuntimeError("schedule row count")
    d0 = yaml.safe_load(COMMON.read_bytes(COMMON.SCENE_CONTRACT))
    scenes = {
        scene["scene_id"]: scene for scene in d0["scene_families"]
    }
    contract = yaml.safe_load(payloads[0])
    OUTPUT.mkdir(parents=True)
    COMMON.atomic_yaml(
        OUTPUT / "exact_schedule.yaml",
        {
            "schema_version": "1.0",
            "stage": "V3-13-Q1-R19-R3",
            "contract_sha256": args.contract_sha256,
            "row_count": 5,
            "rows": rows,
        },
    )
    journal = OUTPUT / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    COMMON.append_jsonl(
        journal,
        {
            "event": "execution_opened",
            "authorized_units": 5,
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
            "{:03d}_01_7686_{}".format(
                row["sequence"], row["forced_candidate_id"]
            )
        )
        try:
            receipt = run_episode(
                row, scenes[row["scene_id"]], root,
                31800 + 2 * row["sequence"],
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
                    "remaining_units_forfeited": 5 - row["sequence"],
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
                "collection_pass": report["episode_collection_pass"],
            },
        )
        print(
            "[V3-13-Q1-R19-R3] {}/5 action={} pass={} "
            "ego={:.6f} dynamic={:.3f} guard={}".format(
                row["sequence"],
                row["forced_candidate_id"],
                report["episode_collection_pass"],
                report["common_stop_prefix"][
                    "decision_pose"
                ]["x_m"],
                report["result"]["minimum_conservative_clearance_m"],
                report["terminal_guard_activation_count"],
            ),
            flush=True,
        )
    summary = summarize(receipts, args.contract_sha256, contract)
    COMMON.atomic_yaml(OUTPUT / "collection_summary.yaml", summary)
    COMMON.append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "consumed_units": 5,
            "completed_units": 5,
            "forfeited_units": 0,
            "collection_gate_pass": summary["collection_gate_pass"],
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


if __name__ == "__main__":
    main()
