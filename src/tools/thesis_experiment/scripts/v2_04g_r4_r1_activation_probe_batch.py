#!/usr/bin/env python3
"""Run the preregistered R4-R1 readiness gate before TTC and navigation."""

import argparse
import importlib.util
import os
from pathlib import Path
import subprocess
import time

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R4-R1"
R4_BATCH = Path(__file__).with_name("v2_04g_r4_activation_probe_batch.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r4_frozen_probe_helpers_r4_r1", R4_BATCH)
_R4 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R4)
_R4.STAGE = STAGE
_R4._R3.STAGE = STAGE
_R2 = _R4._R2
MATERIALIZER = Path(__file__).with_name("v2_04g_r4_r1_candidate_materializer.py")
_MAT_SPEC = importlib.util.spec_from_file_location("v2_04g_r4_r1_materializer_for_probe", MATERIALIZER)
_MAT = importlib.util.module_from_spec(_MAT_SPEC)
_MAT_SPEC.loader.exec_module(_MAT)


def verify_resources(preregistration):
    for group in ("resources", "frozen_r4_boundary"):
        for name, resource in preregistration.get(group, {}).items():
            path = WORKSPACE / resource["path"]
            if not path.is_file() or _R4._R3._sha256(path) != resource["sha256"]:
                raise ValueError("R4-R1 frozen resource drifted: {}.{}".format(group, name))


def _summary(preregistration_path, schedule, reports, status, failure):
    summary = _R4._summary(preregistration_path, schedule, reports, status, failure)
    summary["stage"] = STAGE
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preregistration", type=Path, required=True)
    parser.add_argument("--candidate-bank", type=Path, required=True)
    parser.add_argument("--compiled-scenes-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--listener", type=Path, default=Path(__file__).with_name(
        "v2_04g_r4_r1_activation_probe_listener.py"))
    args = parser.parse_args()
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    probe = prereg.get("activation_readiness_probe", {})
    if not (
        prereg.get("stage") == STAGE
        and prereg.get("split") == "calibration"
        and prereg.get("training_allowed") is False
        and prereg.get("simulation_only") is True
        and probe.get("required_before_ttc_and_navigation") is True
    ):
        raise ValueError("R4-R1 readiness boundary drifted")
    verify_resources(prereg)
    expected_bank = WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    if args.candidate_bank.resolve() != expected_bank.resolve():
        raise ValueError("R4-R1 candidate bank path drifted")
    expected_index = WORKSPACE / prereg["resources"]["compiled_scene_index"]["path"]
    if (args.compiled_scenes_dir / "compiled_scene_index.yaml").resolve() != expected_index.resolve():
        raise ValueError("R4-R1 compiled scene path drifted")
    output_root = args.output_root.resolve()
    output_root.relative_to((WORKSPACE /
        "artifacts/v2/calibration/v2_04g_r4_r1/activation_probe").resolve())
    runtime = _MAT.materialize_candidates(
        args.candidate_bank, output_root / "runtime_candidate_configs")
    instances = _R2._R1._load_instances(args.compiled_scenes_dir)
    instance, _, world_path = instances[probe["scene_id"]]
    schedule = probe["schedule"]
    if len(schedule) != prereg["budget"]["activation_readiness_probe_count"]:
        raise ValueError("R4-R1 readiness budget drifted")
    if {row["seed"] for row in schedule} != set(
        prereg["seed_firewall"]["readiness_probe_only_seeds"]
    ):
        raise ValueError("R4-R1 readiness seed schedule drifted")
    reports = []
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    summary_path = output_root / "activation_probe_summary.yaml"
    for row in schedule:
        profile = row["profile_id"]
        if profile not in probe["profile_ids"]:
            raise ValueError("R4-R1 readiness profile schedule drifted")
        target = output_root / "probe_{:02d}_{}_repeat_{}".format(
            row["sequence"], profile, row["repeat"])
        target.mkdir(parents=True, exist_ok=True)
        report_path = target / "report.yaml"
        if report_path.is_file():
            report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
            identity = {"stage": STAGE, "profile_id": profile,
                        "repeat": row["repeat"], "seed": row["seed"]}
            if not all(report.get(key) == value for key, value in identity.items()):
                raise RuntimeError("existing R4-R1 readiness identity drifted")
            if report.get("all_hard_gates_pass") is not True:
                raise RuntimeError("existing R4-R1 readiness probe is failed evidence")
            report["_report_path"] = str(report_path)
            report["_report_sha256"] = _R4._R3._sha256(report_path)
            reports.append(report)
            continue
        config = runtime[profile]
        scene = instance["scene"]
        launch_command = [
            "roslaunch", "m2_gazebo", "m2_v2_04g_r2_mechanism_calibration.launch",
            "world:={}".format(world_path), "seed:={}".format(row["seed"]),
            "x:={}".format(scene["start"]["x_m"]),
            "y:={}".format(scene["start"]["y_m"]),
            "yaw:={}".format(scene["start"]["yaw_rad"]), "gui:=false",
            "rule_supervisor_config:={}".format(config["supervisor"]),
            "anchor_bank:={}".format(config["anchor_bank"]),
            "mechanism_config:={}".format(config["mechanism"]),
            "load_balanced_anchor:=true", "publish_teb_obstacles:=true",
            "start_rule_supervisor:=true", "start_typed_transaction:=true",
            "force_geometry_balanced:=false",
        ]
        listener_command = [
            str(args.listener), "--output", str(report_path),
            "--profile-id", profile, "--repeat", str(row["repeat"]),
            "--seed", str(row["seed"]),
            "--warmup-timeout-s", str(probe["warmup_timeout_s"]),
            "--measurement-duration-s", str(probe["measurement_duration_s"]),
            "--minimum-message-count", str(probe["minimum_message_count"]),
            "--minimum-valid-fraction", str(probe["minimum_valid_fraction"]),
            "--required-consecutive-stable-count", str(
                probe["required_consecutive_stable_count"]),
            "--maximum-expected-context-hold-count", str(
                probe["maximum_expected_context_hold_count"]),
        ]
        launch, result, listener_error = None, None, None
        with (target / "launch.log").open("w", encoding="utf-8") as launch_log:
            try:
                launch = subprocess.Popen(
                    launch_command, env=environment, stdout=launch_log,
                    stderr=subprocess.STDOUT, start_new_session=True)
                _R2._R1._ready(launch, environment, "rule_multi_anchor")
                with (target / "listener.log").open("w", encoding="utf-8") as listener_log:
                    try:
                        result = subprocess.run(
                            listener_command, env=environment, stdout=listener_log,
                            stderr=subprocess.STDOUT,
                            timeout=(probe["warmup_timeout_s"]
                                     + probe["measurement_duration_s"] + 30.0),
                            check=False)
                    except subprocess.TimeoutExpired as exc:
                        listener_error = "listener timeout: {}".format(exc)
            finally:
                if launch is not None:
                    _R2._R1._terminate_group(launch)
                time.sleep(1.0)
        if not report_path.is_file():
            failure = {**row, "reason": listener_error or
                       "listener exited without an atomic report"}
            _R4._R3._write_yaml(summary_path, _summary(
                args.preregistration, schedule, reports, "failed", failure))
            raise RuntimeError(failure["reason"])
        report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
        report["_report_path"] = str(report_path)
        report["_report_sha256"] = _R4._R3._sha256(report_path)
        reports.append(report)
        if (result is None or result.returncode != 0
                or report.get("all_hard_gates_pass") is not True):
            failure = {**row, "reason": "R4-R1 atomic-input readiness hard gate failed"}
            _R4._R3._write_yaml(summary_path, _summary(
                args.preregistration, schedule, reports, "failed", failure))
            raise RuntimeError(failure["reason"])
        print("PASS R4-R1 readiness {}/{} {} repeat {}".format(
            len(reports), len(schedule), profile, row["repeat"]), flush=True)
        _R4._R3._write_yaml(summary_path, _summary(
            args.preregistration, schedule, reports, "in_progress", None))
    summary = _summary(args.preregistration, schedule, reports, "complete", None)
    _R4._R3._write_yaml(summary_path, summary)
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0 if summary["all_probe_hard_gates_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
