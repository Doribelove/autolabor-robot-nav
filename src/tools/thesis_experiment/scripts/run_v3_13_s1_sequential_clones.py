#!/usr/bin/env python3
"""Run the exact fresh V3-13-S1 sequential clone matrix once."""

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


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
CONTRACT_PATH = (
    WORKSPACE
    / "config/thesis_experiments/v3/"
    "v3_13_s1_sequential_clone_contract.yaml"
)
SCENE_CONTRACT_PATH = (
    WORKSPACE
    / "config/thesis_experiments/v3/"
    "v3_13_c1_multiactor_clone_contract.yaml"
)
PLANNER_CONTRACT_PATH = (
    WORKSPACE
    / "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
READINESS_ROOT = (
    WORKSPACE / "artifacts/v3/multiactor/v3_13_s1_readiness"
)
RELEASE_PATH = READINESS_ROOT / "execution_release.yaml"
REVIEW_PATH = READINESS_ROOT / "independent_review.yaml"
SCHEDULE_PATH = READINESS_ROOT / "exact_schedule.yaml"
CANONICAL_OUTPUT_ROOT = (
    WORKSPACE / "artifacts/v3/multiactor/v3_13_s1_execution"
)


def read_bytes(path):
    with path.open("rb") as stream:
        return stream.read()


def sha256_bytes(payload):
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


def run_episode(row, scene, episode_root, ros_port):
    episode_root.mkdir(parents=True, exist_ok=False)
    world_path = episode_root / "world.sdf"
    scene_path = episode_root / "scene_spec.yaml"
    actor_specs_path = episode_root / "actor_specs.yaml"
    report_path = episode_root / "episode_report.yaml"
    log_path = episode_root / "roslaunch.log"
    runtime_home = episode_root / "runtime_home"
    runtime_home.mkdir()
    (runtime_home / "roslog").mkdir()
    world, actor_specs = design.render_world(
        scene,
        row["actor_policies_evaluator_only"],
        row["seed"],
    )
    atomic_bytes(world_path, world.encode("utf-8"))
    atomic_yaml(scene_path, scene)
    atomic_yaml(
        actor_specs_path,
        {
            "schema_version": "1.0",
            "stage": "V3-13-S1",
            "actors": actor_specs,
        },
    )
    command = [
        "roslaunch",
        "m2_gazebo",
        "m2_v3_13_sequential_clone_episode.launch",
        "world:={}".format(world_path.resolve()),
        "seed:={}".format(row["seed"]),
        "scene_id:={}".format(row["scene_id"]),
        "block_index:={}".format(row["block_index"]),
        "pre_action_wait_s:={:.1f}".format(row["pre_action_wait_s"]),
        "forced_candidate_id:={}".format(row["forced_candidate_id"]),
        "behavior_propensity:={:.9f}".format(
            row["behavior_propensity"]
        ),
        "report_path:={}".format(report_path.resolve()),
        "scene_spec_path:={}".format(scene_path.resolve()),
        "actor_specs_path:={}".format(actor_specs_path.resolve()),
        "planner_contract_path:={}".format(
            PLANNER_CONTRACT_PATH.resolve()
        ),
        "gui:=false",
    ]
    started_s = time.monotonic()
    with log_path.open("wb") as log_stream:
        process = subprocess.Popen(
            command,
            cwd=str(WORKSPACE),
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
        tail = read_bytes(log_path)[-8000:].decode(
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
        "stage": "V3-13-S1",
        "row": row,
        "actor_generated_parameters": [
            item["generated_parameters"] for item in actor_specs
        ],
        "episode_report": report,
        "episode_report_sha256": sha256_bytes(read_bytes(report_path)),
        "world_sha256": sha256_bytes(read_bytes(world_path)),
        "scene_spec_sha256": sha256_bytes(read_bytes(scene_path)),
        "actor_specs_sha256": sha256_bytes(read_bytes(actor_specs_path)),
        "roslaunch_log_sha256": sha256_bytes(read_bytes(log_path)),
        "roslaunch_return_code": return_code,
        "wall_elapsed_s": time.monotonic() - started_s,
        "timed_out": timed_out,
        "credential_environment_scrubbed": True,
    }
    atomic_yaml(episode_root / "matrix_receipt.yaml", receipt)
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


def summarize(receipts, contract, contract_sha256):
    blocks = {}
    for receipt in receipts:
        row = receipt["row"]
        blocks.setdefault(int(row["block_index"]), []).append(receipt)
    expected_actions = set(contract["population"]["branch_actions"])
    block_rows = []
    for block_index in sorted(blocks):
        block = blocks[block_index]
        reports = [item["episode_report"] for item in block]
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
                "pre_action_wait_s": reports[0]["pre_action_wait_s"],
                "row_count": len(reports),
                "full_action_support": {
                    report["forced_candidate_id"] for report in reports
                }
                == expected_actions,
                "propensity_sum": sum(
                    float(report["behavior_propensity"])
                    for report in reports
                ),
                "minimum_propensity": min(
                    float(report["behavior_propensity"])
                    for report in reports
                ),
                "decision_ego_x_range_m": max(decision_x)
                - min(decision_x),
                "actor_decision_ranges": actor_ranges,
                "all_assertive_at_branch": all(
                    set(report["observable_modes_at_branch"].values())
                    == {"assertive"}
                    for report in reports
                ),
                "all_collection_pass": all(
                    report["episode_collection_pass"]
                    for report in reports
                ),
                "all_finite_traces": all(
                    finite_report(report) for report in reports
                ),
                "contact_state_count_retained": sum(
                    int(report["result"]["contact_state_count"])
                    for report in reports
                ),
            }
        )
    all_reports = [
        receipt["episode_report"] for receipt in receipts
    ]
    gates = contract["collection_gates"]
    common_prefix = contract["common_prefix"]
    complete_support_rate = sum(
        row["full_action_support"] for row in block_rows
    ) / len(block_rows)
    maximum_ego_x_range = max(
        row["decision_ego_x_range_m"] for row in block_rows
    )
    maximum_actor_coordinate_range = max(
        actor["coordinate_range_m"]
        for row in block_rows
        for actor in row["actor_decision_ranges"]
    )
    maximum_actor_speed_range = max(
        actor["speed_range_mps"]
        for row in block_rows
        for actor in row["actor_decision_ranges"]
    )
    checks = {
        "completed_units_exact": len(receipts)
        == int(gates["completed_units_exact"]),
        "complete_action_support_rate": complete_support_rate
        >= float(gates["complete_action_support_rate_min"]),
        "minimum_behavior_propensity": min(
            float(report["behavior_propensity"])
            for report in all_reports
        )
        >= float(gates["minimum_behavior_propensity"]),
        "branch_trace_samples": all(
            int(report["result"]["trace_sample_count"])
            >= int(gates["branch_trace_samples_min"])
            for report in all_reports
        ),
        "branch_trace_duration": all(
            float(report["result"]["trace_duration_s"])
            >= float(gates["branch_trace_duration_s_min"])
            for report in all_reports
        ),
        "finite_tracks": all(
            row["all_finite_traces"] for row in block_rows
        ),
        "gazebo_process_failures": all(
            not receipt["timed_out"]
            and receipt["episode_report"]["result"]["completed_horizon"]
            for receipt in receipts
        ),
        "common_gate_valid_rate": (
            sum(report["gate_physical_valid"] for report in all_reports)
            / len(all_reports)
        )
        >= float(gates["common_gate_valid_rate_min"]),
        "decision_physical_valid_rate": (
            sum(
                report["decision_physical_valid"]
                for report in all_reports
            )
            / len(all_reports)
        )
        >= float(gates["matched_decision_context_rate_min"]),
        "branch_modes_assertive": all(
            row["all_assertive_at_branch"] for row in block_rows
        ),
        "paired_decision_ego_x": maximum_ego_x_range
        <= float(
            common_prefix["paired_decision_ego_x_difference_max_m"]
        ),
        "paired_decision_actor_coordinate": (
            maximum_actor_coordinate_range
            <= float(
                common_prefix[
                    "paired_decision_actor_coordinate_difference_max_m"
                ]
            )
        ),
        "paired_decision_actor_speed": maximum_actor_speed_range
        <= float(
            common_prefix[
                "paired_decision_actor_speed_difference_max_mps"
            ]
        ),
    }
    return {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-13-S1",
        "status": (
            "sequential_clone_collection_pass"
            if all(checks.values())
            else "sequential_clone_collection_gate_fail"
        ),
        "contract_sha256": contract_sha256,
        "authorized_units": 32,
        "consumed_units": len(receipts),
        "completed_units": len(receipts),
        "forfeited_units": 0,
        "matched_context_blocks": len(block_rows),
        "complete_action_support_rate": complete_support_rate,
        "maximum_decision_ego_x_range_m": maximum_ego_x_range,
        "maximum_decision_actor_coordinate_range_m": (
            maximum_actor_coordinate_range
        ),
        "maximum_decision_actor_speed_range_mps": (
            maximum_actor_speed_range
        ),
        "contact_state_count_retained": sum(
            row["contact_state_count_retained"] for row in block_rows
        ),
        "gate_checks": checks,
        "collection_gate_pass": all(checks.values()),
        "blocks": block_rows,
        "policy_or_ranker_training_steps": 0,
        "assertive_recovery_predictor_fit_authorized": all(
            checks.values()
        ),
        "policy_or_ranker_training_authorized": False,
        "actual_teb_comparison_authorized": False,
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
    output_root = Path(arguments.output_root).resolve()
    if output_root != CANONICAL_OUTPUT_ROOT.resolve():
        raise RuntimeError("output root is not the released canonical path")
    if output_root.exists():
        raise RuntimeError("output root exists; retry/resume is forbidden")
    for path in (RELEASE_PATH, REVIEW_PATH, SCHEDULE_PATH):
        if not path.is_file():
            raise RuntimeError("release resource absent")
    release_payload = read_bytes(RELEASE_PATH)
    review_payload = read_bytes(REVIEW_PATH)
    if sha256_bytes(release_payload) != arguments.release_sha256:
        raise RuntimeError("release SHA does not match")
    if sha256_bytes(review_payload) != arguments.review_sha256:
        raise RuntimeError("review SHA does not match")
    release = yaml.safe_load(release_payload)
    review = yaml.safe_load(review_payload)
    if not release["execution_authorized"]:
        raise RuntimeError("release does not authorize execution")
    if int(release["authorized_units"]) != 32:
        raise RuntimeError("release unit count differs")
    if review["status"] != (
        "v3_13_s1_independent_readiness_review_pass"
    ):
        raise RuntimeError("independent readiness review did not pass")
    if review["execution_release_sha256"] != arguments.release_sha256:
        raise RuntimeError("review does not bind release")
    contract_payload = read_bytes(CONTRACT_PATH)
    contract_sha256 = sha256_bytes(contract_payload)
    if arguments.contract_sha256 != contract_sha256:
        raise RuntimeError("contract SHA does not match")
    if release["zero_authority_design_contract_sha256"] != contract_sha256:
        raise RuntimeError("release does not bind design contract")
    contract = yaml.safe_load(contract_payload)
    if contract["authority"]["training_authorized"]:
        raise RuntimeError("training boundary differs")
    if contract["authority"]["hardware_device_access_authorized"]:
        raise RuntimeError("hardware boundary differs")
    schedule = yaml.safe_load(read_bytes(SCHEDULE_PATH))
    rows = schedule["rows"]
    if len(rows) != 32:
        raise RuntimeError("exact schedule differs")
    scene_contract = yaml.safe_load(read_bytes(SCENE_CONTRACT_PATH))
    scenes = {
        item["scene_id"]: item
        for item in scene_contract["scene_population"]
    }
    output_root.mkdir(parents=True)
    atomic_yaml(
        output_root / "exact_schedule.yaml",
        {
            "schema_version": "1.0",
            "stage": "V3-13-S1",
            "contract_sha256": contract_sha256,
            "row_count": len(rows),
            "rows": rows,
        },
    )
    journal = output_root / "attempt_journal.jsonl"
    journal.touch(exist_ok=False)
    append_jsonl(
        journal,
        {
            "event": "execution_opened",
            "stage": "V3-13-S1",
            "contract_sha256": contract_sha256,
            "authorized_units": len(rows),
        },
    )
    receipts = []
    for row in rows:
        episode_root = output_root / "episodes" / (
            "{:03d}_{:02d}_{}_wait{:02d}_{}".format(
                row["sequence"],
                row["block_index"],
                row["seed"],
                int(round(10.0 * row["pre_action_wait_s"])),
                row["forced_candidate_id"],
            )
        )
        append_jsonl(
            journal,
            {
                "event": "unit_consumed",
                "sequence": row["sequence"],
                "row": row,
            },
        )
        try:
            receipt = run_episode(
                row,
                scenes[row["scene_id"]],
                episode_root,
                29600 + 2 * row["sequence"],
            )
        except Exception as error:
            append_jsonl(
                journal,
                {
                    "event": "terminal_failure",
                    "sequence": row["sequence"],
                    "row": row,
                    "error": "{}:{}".format(type(error).__name__, error),
                    "consumed_units": row["sequence"],
                    "remaining_units_forfeited": len(rows)
                    - row["sequence"],
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
                "collection_pass": report[
                    "episode_collection_pass"
                ],
                "trace_samples": report["result"][
                    "trace_sample_count"
                ],
                "contacts_retained": report["result"][
                    "contact_state_count"
                ],
            },
        )
        print(
            "[V3-13-S1] {}/32 block={} wait={:.1f}s action={} "
            "pass={} samples={} clearance={:.3f}".format(
                row["sequence"],
                row["block_index"],
                row["pre_action_wait_s"],
                row["forced_candidate_id"],
                report["episode_collection_pass"],
                report["result"]["trace_sample_count"],
                report["result"]["minimum_conservative_clearance_m"],
            ),
            flush=True,
        )
    summary = summarize(receipts, contract, contract_sha256)
    atomic_yaml(output_root / "collection_summary.yaml", summary)
    append_jsonl(
        journal,
        {
            "event": "execution_complete",
            "authorized_units": 32,
            "consumed_units": 32,
            "completed_units": 32,
            "forfeited_units": 0,
            "collection_gate_pass": summary["collection_gate_pass"],
            "summary_sha256": sha256_bytes(
                read_bytes(output_root / "collection_summary.yaml")
            ),
        },
    )
    print(
        "status={} summary_sha256={}".format(
            summary["status"],
            sha256_bytes(
                read_bytes(output_root / "collection_summary.yaml")
            ),
        )
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
