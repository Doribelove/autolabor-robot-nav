#!/usr/bin/env python3
"""Run the preregistered activation-readiness gate before R1 navigation."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import subprocess
import time

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BATCH_SOURCE = Path(__file__).with_name("v2_04g_r1_calibration_batch.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r1_batch_helpers", BATCH_SOURCE)
_BATCH = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BATCH)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _write_yaml(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preregistration", type=Path, required=True)
    parser.add_argument("--candidate-bank", type=Path, required=True)
    parser.add_argument("--compiled-scenes-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument(
        "--listener", type=Path,
        default=Path(__file__).with_name("v2_04g_r1_activation_probe_listener.py"),
    )
    args = parser.parse_args()
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    if not (
        prereg["stage"] == "V2-04G-R1"
        and prereg["split"] == "calibration"
        and prereg["activation_readiness_probe"]["required_before_navigation"] is True
    ):
        raise ValueError("activation probe preregistration boundary drifted")
    _BATCH._verify_preregistered_resources(prereg)
    if args.candidate_bank.resolve() != (
        WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    ).resolve():
        raise ValueError("activation probe candidate bank path drifted")
    compiled_index = args.compiled_scenes_dir / "compiled_scene_index.yaml"
    expected_index = WORKSPACE / prereg["resources"]["compiled_scene_index"]["path"]
    if compiled_index.resolve() != expected_index.resolve():
        raise ValueError("activation probe compiled scene directory drifted")
    output_root = args.output_root.resolve()
    expected_root = WORKSPACE / "artifacts/v2/calibration/v2_04g_r1/activation_probe"
    output_root.relative_to(expected_root.resolve())
    runtime = _BATCH._materialize_candidates(
        args.candidate_bank,
        output_root / "runtime_candidate_configs",
        "V2-04G-R1",
    )
    instances = _BATCH._load_instances(args.compiled_scenes_dir)
    scene_id = prereg["activation_readiness_probe"]["scene_id"]
    instance, instance_path, world_path = instances[scene_id]
    del instance_path
    schedule = prereg["activation_readiness_probe"]["schedule"]
    if len(schedule) != prereg["activation_readiness_probe"]["planned_probe_count"]:
        raise ValueError("activation probe budget drifted")
    reports = []
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    for row in schedule:
        profile = row["profile_id"]
        if profile not in ("g1_mechanism_balanced", "g2_mechanism_aggressive"):
            raise ValueError("activation probe profile set drifted")
        target = output_root / "probe_{:02d}_{}_repeat_{}".format(
            row["sequence"], profile, row["repeat"]
        )
        target.mkdir(parents=True, exist_ok=True)
        report_path = target / "report.yaml"
        if report_path.is_file():
            report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
            expected_identity = {
                "stage": "V2-04G-R1", "profile_id": profile,
                "repeat": row["repeat"], "seed": row["seed"],
            }
            if (
                all(report.get(key) == value for key, value in expected_identity.items())
                and report.get("all_hard_gates_pass") is True
            ):
                reports.append(report)
                continue
            raise RuntimeError("existing activation probe evidence identity or gate failed")
        config = runtime[profile]
        scene = instance["scene"]
        launch_command = [
            "roslaunch", "m2_gazebo", "m2_v2_04g_r1_mechanism_calibration.launch",
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
            "--warmup-timeout-s", str(
                prereg["activation_readiness_probe"]["warmup_timeout_s"]
            ),
            "--measurement-duration-s", str(
                prereg["activation_readiness_probe"]["measurement_duration_s"]
            ),
            "--minimum-message-count", str(
                prereg["activation_readiness_probe"]["minimum_message_count"]
            ),
            "--minimum-valid-fraction", str(
                prereg["activation_readiness_probe"]["minimum_valid_fraction"]
            ),
        ]
        launch = None
        with (target / "launch.log").open("w", encoding="utf-8") as launch_log:
            try:
                launch = subprocess.Popen(
                    launch_command, env=environment, stdout=launch_log,
                    stderr=subprocess.STDOUT, start_new_session=True,
                )
                _BATCH._ready(launch, environment, "rule_multi_anchor")
                with (target / "listener.log").open("w", encoding="utf-8") as listener_log:
                    result = subprocess.run(
                        listener_command, env=environment,
                        stdout=listener_log, stderr=subprocess.STDOUT,
                        timeout=45.0, check=False,
                    )
                if result.returncode != 0:
                    raise RuntimeError("activation probe listener failed")
            finally:
                if launch is not None:
                    _BATCH._terminate_group(launch)
                time.sleep(1.0)
        report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
        if not report["all_hard_gates_pass"]:
            raise RuntimeError("activation readiness hard gate failed")
        reports.append(report)
        print("PASS activation probe {}/{} {} repeat {}".format(
            len(reports), len(schedule), profile, row["repeat"]
        ), flush=True)
    summary = {
        "schema_version": "2.0", "stage": "V2-04G-R1",
        "status": "complete" if len(reports) == len(schedule) else "incomplete",
        "simulation_only": True, "runtime_ready": False,
        "training_started": False, "real_vehicle_used": False,
        "preregistration": {
            "path": str(args.preregistration), "sha256": _sha256(args.preregistration),
        },
        "planned_probe_count": len(schedule),
        "valid_probe_count": len(reports),
        "all_probe_hard_gates_pass": (
            len(reports) == len(schedule)
            and all(report["all_hard_gates_pass"] for report in reports)
        ),
        "reports": [
            {
                "profile_id": report["profile_id"], "repeat": report["repeat"],
                "seed": report["seed"],
                "transaction_activated_fraction": report[
                    "transaction_activated_fraction"
                ],
                "join_valid_fraction": report["join_valid_fraction"],
                "join_reason_counts": report["join_reason_counts"],
            }
            for report in reports
        ],
        "navigation_authorized": (
            len(reports) == len(schedule)
            and all(report["all_hard_gates_pass"] for report in reports)
        ),
    }
    _write_yaml(output_root / "activation_probe_summary.yaml", summary)
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0 if summary["all_probe_hard_gates_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
