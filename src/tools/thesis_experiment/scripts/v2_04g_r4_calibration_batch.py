#!/usr/bin/env python3
"""Run/resume preregistered V2-04G-R4 full calibration navigation."""

import argparse
import importlib.util
import os
from pathlib import Path

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R4"
R3_BATCH = Path(__file__).with_name("v2_04g_r3_calibration_batch.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r3_frozen_batch_r4", R3_BATCH)
_R3 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R3)
_R3.STAGE = STAGE
_R2, _BASE = _R3._R2, _R3._BASE


def verify_resources(preregistration):
    for group in ("resources", "frozen_repair_boundary"):
        for name, resource in preregistration.get(group, {}).items():
            path = WORKSPACE / resource["path"]
            if not path.is_file() or _BASE._sha256(path) != resource["sha256"]:
                raise ValueError("R4 frozen resource drifted: {}.{}".format(group, name))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preregistration", type=Path, required=True)
    parser.add_argument("--compiled-scenes-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--candidate-bank", type=Path, required=True)
    parser.add_argument("--episode-runner", type=Path, default=WORKSPACE /
        "src/tools/thesis_experiment/scripts/v2_04g_r4_mechanism_episode.py")
    parser.add_argument("--max-new-episodes", type=int, default=None)
    parser.add_argument("--attempts-per-episode", type=int, default=2)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    if not (prereg.get("stage") == STAGE and prereg.get("split") == "calibration"
            and prereg.get("training_allowed") is False):
        raise ValueError("R4 batch refuses non-calibration input")
    verify_resources(prereg)
    _R3.require_readiness_and_ttc(prereg, args.preregistration)
    if args.attempts_per_episode != prereg["budget"]["attempts_per_navigation_episode_max"]:
        raise ValueError("R4 attempt budget drifted")
    args.output_root = args.output_root.resolve()
    args.output_root.relative_to((WORKSPACE / "artifacts/v2/calibration/v2_04g_r4").resolve())
    expected_bank = WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    if args.candidate_bank.resolve() != expected_bank.resolve():
        raise ValueError("R4 candidate bank path drifted")
    expected_index = WORKSPACE / prereg["resources"]["compiled_scene_index"]["path"]
    if (args.compiled_scenes_dir / "compiled_scene_index.yaml").resolve() != expected_index.resolve():
        raise ValueError("R4 compiled scene directory drifted")
    runtime = _R2.materialize_candidates(
        args.candidate_bank, args.output_root / "runtime_candidate_configs")
    instances = _BASE._load_instances(args.compiled_scenes_dir)
    schedule = _R3.build_schedule(prereg, instances, runtime)
    completed = _BASE.inventory_completed(schedule, args.output_root)
    failures = []
    progress_path = args.output_root / "v2_04g_r4_progress.yaml"
    _BASE.write_progress(progress_path, args.preregistration, schedule, completed, failures)
    pending = [row for row in schedule if (
        row["profile_id"], row["method"], row["scene_id"]) not in completed]
    if args.max_new_episodes is not None:
        pending = pending[:args.max_new_episodes]
    print("V2-04G-R4 resume: {} valid evidence, {} pending selected".format(
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
                print("DONE {} success={} switches={} reverse={}".format(
                    identity, evidence["metrics"]["common"]["success"],
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
            raise RuntimeError("persistent R4 interface failure")
        _BASE.write_progress(progress_path, args.preregistration, schedule, completed, failures)
    print("V2-04G-R4 complete: {}/{} valid evidence".format(len(completed), len(schedule)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
