#!/usr/bin/env python3
"""Run the one-shot V3-14-S1 preflight and conditional support matrix."""

import argparse
import importlib.util
from pathlib import Path
import subprocess
import sys
import time

import yaml


W = Path("/home/robot/robot_ws_base_rl")
SCRIPTS = W / "src/tools/thesis_experiment/scripts"
sys.path.insert(0, str(SCRIPTS))
import v3_13_multiactor_design as design


def import_path(name, relative):
    spec = importlib.util.spec_from_file_location(name, W / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BINDING = import_path("v3_14_s1_binding", "src/tools/thesis_experiment/src/thesis_experiment/v3_14_action_binding.py")
R4 = import_path("v3_14_s1_r4_summary", "src/tools/thesis_experiment/scripts/run_v3_13_q1_r19_r4_prefix_aligned_support.py")
COMMON = R4.COMMON
R1 = R4.R2.R1
CONTRACT = W / "config/thesis_experiments/v3/v3_14_s1_multiaction_support_contract.yaml"
READINESS = W / "artifacts/v3/multiactor/v3_14_s1_readiness"
OUTPUT = W / "artifacts/v3/multiactor/v3_14_s1_multiaction_support"
GEOMETRY = W / "config/thesis_experiments/v3/v3_13_q1_r17_goal_aligned_recovery_contract.yaml"
LAUNCH = "m2_v3_13_tolerance_repaired_prefix_aligned_bank_clone_episode.launch"


def command_for(row, paths, allowed):
    command = ["roslaunch", "m2_gazebo", LAUNCH]
    command += BINDING.build_launch_binding_args(row, allowed)
    command += [
        "world:={}".format(paths["world"].resolve()),
        "pre_action_wait_s:={:.12g}".format(float(row["pre_action_wait_s"])),
        "branch_target_sim_s:={:.12g}".format(float(row["branch_target_sim_s"])),
        "report_path:={}".format(paths["report"].resolve()),
        "scene_spec_path:={}".format(paths["scene"].resolve()),
        "actor_specs_path:={}".format(paths["actors"].resolve()),
        "planner_contract_path:={}".format(COMMON.PLANNER_CONTRACT.resolve()),
        "geometry_contract_path:={}".format(GEOMETRY.resolve()),
        "gui:=false",
    ]
    return command


def run_episode(row, scene, root, ros_port, journal, allowed):
    root.mkdir(parents=True, exist_ok=False)
    paths = {
        "world": root / "world.sdf", "scene": root / "scene_spec.yaml",
        "actors": root / "actor_specs.yaml", "report": root / "episode_report.yaml",
        "log": root / "roslaunch.log",
    }
    runtime_home = root / "runtime_home"
    runtime_home.mkdir(); (runtime_home / "roslog").mkdir()
    world, actors = design.render_world(scene, row["actor_policies_evaluator_only"], row["seed"])
    COMMON.atomic_bytes(paths["world"], world.encode("utf-8"))
    COMMON.atomic_yaml(paths["scene"], scene)
    COMMON.atomic_yaml(paths["actors"], {"schema_version": "1.0", "stage": "V3-14-S1", "actors": actors})
    command = command_for(row, paths, allowed)
    pre = BINDING.verify_preconsumption_binding(row, command, allowed)
    COMMON.append_jsonl(journal, {"event": "unit_binding_verified_preconsumption", "sequence": row["sequence"], "binding": pre})
    COMMON.append_jsonl(journal, {"event": "unit_consumed", "sequence": row["sequence"], "row": row})
    started = time.monotonic()
    with paths["log"].open("wb") as stream:
        process = subprocess.Popen(
            command, cwd=str(W), env=R1.closed_overlay_environment(runtime_home, ros_port),
            stdin=subprocess.DEVNULL, stdout=stream, stderr=subprocess.STDOUT,
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
    if not paths["report"].is_file():
        tail = COMMON.read_bytes(paths["log"])[-8000:].decode("utf-8", errors="replace")
        raise RuntimeError("episode report absent rc={} timeout={}\n{}".format(return_code, timed_out, tail))
    report = yaml.safe_load(COMMON.read_bytes(paths["report"]))
    runtime = BINDING.verify_runtime_report_binding(row, report, allowed)
    receipt = {
        "schema_version": "1.0", "stage": "V3-14-S1", "runtime_component_stage": "V3-13-Q1-R19-R5",
        "row": row, "preconsumption_binding": pre, "runtime_binding": runtime,
        "episode_report": report, "episode_report_sha256": COMMON.sha(COMMON.read_bytes(paths["report"])),
        "world_sha256": COMMON.sha(COMMON.read_bytes(paths["world"])),
        "scene_spec_sha256": COMMON.sha(COMMON.read_bytes(paths["scene"])),
        "actor_specs_sha256": COMMON.sha(COMMON.read_bytes(paths["actors"])),
        "roslaunch_log_sha256": COMMON.sha(COMMON.read_bytes(paths["log"])),
        "roslaunch_return_code": return_code, "wall_elapsed_s": time.monotonic() - started,
        "timed_out": timed_out, "credential_environment_scrubbed": True,
        "workspace_overlay_explicit": True, "foreign_workspace_precedence_used": False,
        "scientific_inclusion_binding_pass": True,
    }
    COMMON.atomic_yaml(root / "matrix_receipt.yaml", receipt)
    COMMON.append_jsonl(journal, {
        "event": "unit_complete", "sequence": row["sequence"], "row": row,
        "episode_report_sha256": receipt["episode_report_sha256"],
        "runtime_binding": runtime, "collection_pass": report["episode_collection_pass"],
    })
    return receipt


def summarize(receipts, contract_sha, contract, phase):
    value = R4.summarize(receipts, contract_sha, contract)
    expected = 5 if phase == "preflight" else 40
    value.update({
        "stage": "V3-14-S1", "phase": phase, "authorized_units": expected,
        "consumed_units": len(receipts), "completed_units": len(receipts), "forfeited_units": 0,
        "non_scientific_preflight": phase == "preflight", "runtime_component_stage": "V3-13-Q1-R19-R5",
        "predictor_fit_authorized": False, "actual_TEB_execution_authorized": False,
        "hardware_accessed": False, "real_vehicle_used": False,
    })
    value["gate_checks"]["completed_units_exact"] = len(receipts) == expected
    value["gate_checks"]["all_action_bindings"] = all(
        r["preconsumption_binding"]["binding_pass"] and r["runtime_binding"]["binding_pass"] for r in receipts
    )
    value["collection_gate_pass"] = all(value["gate_checks"].values())
    value["status"] = "v3_14_s1_{}_pass".format(phase) if value["collection_gate_pass"] else "v3_14_s1_{}_fail".format(phase)
    return value


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-root", required=True)
    parser.add_argument("--contract-sha256", required=True)
    parser.add_argument("--release-sha256", required=True)
    parser.add_argument("--review-sha256", required=True)
    parser.add_argument("--execute", action="store_true", required=True)
    args = parser.parse_args()
    if Path(args.output_root).resolve() != OUTPUT.resolve() or OUTPUT.exists():
        raise RuntimeError("canonical output differs or already exists; retry forbidden")
    payloads = [COMMON.read_bytes(CONTRACT), COMMON.read_bytes(READINESS / "execution_release.yaml"), COMMON.read_bytes(READINESS / "independent_review.yaml")]
    for payload, expected, label in zip(payloads, (args.contract_sha256, args.release_sha256, args.review_sha256), ("contract", "release", "review")):
        if COMMON.sha(payload) != expected:
            raise RuntimeError("{} SHA differs".format(label))
    release, review = yaml.safe_load(payloads[1]), yaml.safe_load(payloads[2])
    if not release["execution_authorized"] or not review["execution_ready"] or review["execution_release_sha256"] != args.release_sha256:
        raise RuntimeError("release not independently ready")
    rows = yaml.safe_load(COMMON.read_bytes(READINESS / "exact_schedule.yaml"))["rows"]
    if len(rows) != 45:
        raise RuntimeError("schedule row count")
    scene_contract = yaml.safe_load(COMMON.read_bytes(COMMON.SCENE_CONTRACT))
    scenes = {scene["scene_id"]: scene for scene in scene_contract["scene_families"]}
    contract = yaml.safe_load(payloads[0]); allowed = tuple(contract["population"]["actions"])
    OUTPUT.mkdir(parents=True)
    COMMON.atomic_yaml(OUTPUT / "exact_schedule.yaml", {"schema_version": "1.0", "stage": "V3-14-S1", "contract_sha256": args.contract_sha256, "row_count": 45, "rows": rows})
    journal = OUTPUT / "attempt_journal.jsonl"; journal.touch(exist_ok=False)
    COMMON.append_jsonl(journal, {"event": "execution_opened", "authorized_units": 45, "contract_sha256": args.contract_sha256, "release_sha256": args.release_sha256})
    receipts = {"preflight": [], "evidence": []}
    for row in rows:
        if row["phase"] == "evidence" and not (OUTPUT / "preflight_summary.yaml").is_file():
            raise RuntimeError("evidence reached before preflight gate")
        root = OUTPUT / "episodes" / "{:03d}_{}_b{:02d}_{}_{}".format(row["sequence"], row["phase"], row["block_index"], row["seed"], row["forced_candidate_id"])
        try:
            receipt = run_episode(row, scenes[row["scene_id"]], root, 32200 + 2 * row["sequence"], journal, allowed)
        except Exception as error:
            COMMON.append_jsonl(journal, {"event": "terminal_failure", "sequence": row["sequence"], "row": row, "error": "{}:{}".format(type(error).__name__, error), "remaining_units_forfeited": 45 - row["sequence"], "scientific_inclusion": False})
            raise
        receipts[row["phase"]].append(receipt)
        report = receipt["episode_report"]
        print("[V3-14-S1] {}/45 phase={} block={} action={} bind=pass collection={} ego={:.6f}".format(row["sequence"], row["phase"], row["block_index"], row["forced_candidate_id"], report["episode_collection_pass"], report["common_stop_prefix"]["decision_pose"]["x_m"]), flush=True)
        if row["sequence"] == 5:
            preflight = summarize(receipts["preflight"], args.contract_sha256, contract, "preflight")
            COMMON.atomic_yaml(OUTPUT / "preflight_summary.yaml", preflight)
            if not preflight["collection_gate_pass"]:
                COMMON.append_jsonl(journal, {"event": "terminal_preflight_failure", "consumed_units": 5, "completed_units": 5, "remaining_units_forfeited": 40, "summary_sha256": COMMON.sha(COMMON.read_bytes(OUTPUT / "preflight_summary.yaml"))})
                COMMON.atomic_yaml(OUTPUT / "terminal_result.yaml", {"schema_version": "1.0", "stage": "V3-14-S1", "status": "terminal_negative_preflight", "consumed_units": 5, "forfeited_units": 40, "scientific_episode_count": 0, "predictor_fit_authorized": False, "hardware_accessed": False, "real_vehicle_used": False})
                print("status=terminal_negative_preflight")
                return
            COMMON.append_jsonl(journal, {"event": "preflight_gate_passed", "summary_sha256": COMMON.sha(COMMON.read_bytes(OUTPUT / "preflight_summary.yaml")), "evidence_units_released": 40})
    evidence = summarize(receipts["evidence"], args.contract_sha256, contract, "evidence")
    COMMON.atomic_yaml(OUTPUT / "evidence_summary.yaml", evidence)
    terminal = {
        "schema_version": "1.0", "stage": "V3-14-S1",
        "status": "fresh_multiaction_support_pass" if evidence["collection_gate_pass"] else "terminal_negative_multiaction_support",
        "authorized_units": 45, "consumed_units": 45, "completed_units": 45, "forfeited_units": 0,
        "scientific_episode_count": 40, "preflight_excluded_from_science": True,
        "all_action_bindings_pass": True, "evidence_collection_gate_pass": evidence["collection_gate_pass"],
        "predictor_fit_authorized": evidence["collection_gate_pass"], "actual_TEB_execution_authorized": False,
        "perception_execution_authorized": False, "training_authorized": False,
        "hardware_accessed": False, "real_vehicle_used": False,
    }
    COMMON.atomic_yaml(OUTPUT / "terminal_result.yaml", terminal)
    COMMON.append_jsonl(journal, {"event": "execution_complete", **{key: terminal[key] for key in ("consumed_units", "completed_units", "forfeited_units", "scientific_episode_count", "evidence_collection_gate_pass")}, "terminal_result_sha256": COMMON.sha(COMMON.read_bytes(OUTPUT / "terminal_result.yaml"))})
    print("status={} evidence_summary_sha256={}".format(terminal["status"], COMMON.sha(COMMON.read_bytes(OUTPUT / "evidence_summary.yaml"))))


if __name__ == "__main__":
    main()

