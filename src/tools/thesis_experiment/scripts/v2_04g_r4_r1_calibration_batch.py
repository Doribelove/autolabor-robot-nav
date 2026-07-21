#!/usr/bin/env python3
"""Run/resume the preregistered R4-R1 full calibration navigation."""

import argparse
import importlib.util
import os
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R4-R1"
R3_BATCH = Path(__file__).with_name("v2_04g_r3_calibration_batch.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r3_frozen_batch_r4_r1", R3_BATCH)
_R3 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R3)
_R3.STAGE = STAGE
_BASE = _R3._BASE
MATERIALIZER = Path(__file__).with_name("v2_04g_r4_r1_candidate_materializer.py")
_MAT_SPEC = importlib.util.spec_from_file_location("v2_04g_r4_r1_materializer_for_batch", MATERIALIZER)
_MAT = importlib.util.module_from_spec(_MAT_SPEC)
_MAT_SPEC.loader.exec_module(_MAT)


def verify_resources(preregistration):
    for group in ("resources", "frozen_r4_boundary"):
        for name, resource in preregistration.get(group, {}).items():
            path = WORKSPACE / resource["path"]
            if not path.is_file() or _BASE._sha256(path) != resource["sha256"]:
                raise ValueError("R4-R1 frozen resource drifted: {}.{}".format(group, name))


def require_readiness_and_ttc(preregistration, preregistration_path):
    readiness_path = WORKSPACE / preregistration[
        "activation_readiness_probe"]["summary_path"]
    ttc_path = WORKSPACE / preregistration["ttc_component_probe"]["report_path"]
    if not readiness_path.is_file() or not ttc_path.is_file():
        raise RuntimeError("R4-R1 readiness or TTC prerequisite is missing")
    prereg_hash = _BASE._sha256(preregistration_path)
    readiness = yaml.safe_load(readiness_path.read_text(encoding="utf-8"))
    ttc = yaml.safe_load(ttc_path.read_text(encoding="utf-8"))
    planned = preregistration["budget"]["activation_readiness_probe_count"]
    if not (
        readiness.get("stage") == STAGE and readiness.get("status") == "complete"
        and readiness.get("valid_probe_count") == planned
        and readiness.get("all_probe_hard_gates_pass") is True
        and readiness.get("navigation_authorized") is True
        and readiness.get("preregistration", {}).get("sha256") == prereg_hash
    ):
        raise RuntimeError("R4-R1 readiness gate failed")
    if not (
        ttc.get("stage") == STAGE and ttc.get("status") == "complete"
        and ttc.get("probe_count") == 3 and ttc.get("all_three_states_pass") is True
        and ttc.get("preregistration", {}).get("sha256") == prereg_hash
    ):
        raise RuntimeError("R4-R1 TTC gate failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preregistration", type=Path, required=True)
    parser.add_argument("--compiled-scenes-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--candidate-bank", type=Path, required=True)
    parser.add_argument("--episode-runner", type=Path, default=WORKSPACE /
        "src/tools/thesis_experiment/scripts/v2_04g_r4_r1_mechanism_episode.py")
    parser.add_argument("--max-new-episodes", type=int, default=None)
    parser.add_argument("--attempts-per-episode", type=int, default=2)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    if not (prereg.get("stage") == STAGE and prereg.get("split") == "calibration"
            and prereg.get("training_allowed") is False):
        raise ValueError("R4-R1 batch refuses non-calibration input")
    verify_resources(prereg)
    require_readiness_and_ttc(prereg, args.preregistration)
    if args.attempts_per_episode != prereg["budget"]["attempts_per_navigation_episode_max"]:
        raise ValueError("R4-R1 attempt budget drifted")
    args.output_root = args.output_root.resolve()
    args.output_root.relative_to((WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r4_r1").resolve())
    expected_bank = WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    if args.candidate_bank.resolve() != expected_bank.resolve():
        raise ValueError("R4-R1 candidate bank path drifted")
    expected_index = WORKSPACE / prereg["resources"]["compiled_scene_index"]["path"]
    if (args.compiled_scenes_dir / "compiled_scene_index.yaml").resolve() != expected_index.resolve():
        raise ValueError("R4-R1 compiled scene directory drifted")
    runtime = _MAT.materialize_candidates(
        args.candidate_bank, args.output_root / "runtime_candidate_configs")
    instances = _BASE._load_instances(args.compiled_scenes_dir)
    schedule = _R3.build_schedule(prereg, instances, runtime)
    completed = _BASE.inventory_completed(schedule, args.output_root)
    failures = []
    progress_path = args.output_root / "v2_04g_r4_r1_progress.yaml"
    _BASE.write_progress(progress_path, args.preregistration, schedule, completed, failures)
    pending = [row for row in schedule if (
        row["profile_id"], row["method"], row["scene_id"]) not in completed]
    if args.max_new_episodes is not None:
        pending = pending[:args.max_new_episodes]
    print("V2-04G-R4-R1 resume: {} valid evidence, {} pending selected".format(
        len(completed), len(pending)), flush=True)
    if args.dry_run:
        for row in pending:
            print("{:03d} {} {}".format(row["sequence"], row["profile_id"], row["scene_id"]))
        return 0
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    for row in pending:
        identity = (row["profile_id"], row["method"], row["scene_id"])
        print("START {:03d}/{:03d} {} x {}".format(
            row["sequence"], len(schedule), row["profile_id"], row["scene_id"]), flush=True)
        last_error = None
        for attempt in range(1, args.attempts_per_episode + 1):
            try:
                path, evidence = _R3.run_episode(row, args, instances, environment)
                completed[identity] = (path, evidence)
                print("DONE {} success={} clearance={} switches={} reverse={}".format(
                    identity, evidence["metrics"]["common"]["success"],
                    evidence["metrics"]["common"]["minimum_clearance_m"],
                    evidence["active_anchor_switch_count"],
                    evidence.get("mechanism_maneuver_reverse_sample_count", 0)), flush=True)
                last_error = None
                break
            except Exception as exc:
                last_error = "attempt {}: {}".format(attempt, exc)
                print("RETRY {} {}".format(identity, last_error), flush=True)
        if last_error is not None:
            failures.append({"identity": list(identity), "reason": last_error})
            _BASE.write_progress(progress_path, args.preregistration, schedule, completed, failures)
            raise RuntimeError("persistent R4-R1 interface failure")
        _BASE.write_progress(progress_path, args.preregistration, schedule, completed, failures)
    print("V2-04G-R4-R1 complete: {}/{} valid evidence".format(
        len(completed), len(schedule)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
