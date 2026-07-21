#!/usr/bin/env python3
"""Run the preregistered V2-04G-R2 activation-readiness gate."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import subprocess
import time

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
R2_BATCH = Path(__file__).with_name("v2_04g_r2_calibration_batch.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r2_probe_helpers", R2_BATCH)
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
        default=Path(__file__).with_name("v2_04g_r2_activation_probe_listener.py"),
    )
    args = parser.parse_args()
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    if not (
        prereg.get("stage") == "V2-04G-R2"
        and prereg.get("split") == "calibration"
        and prereg["activation_readiness_probe"]["required_before_navigation"] is True
    ):
        raise ValueError("R2 activation preregistration boundary drifted")
    _BATCH._verify_resources(prereg)
    expected_bank = WORKSPACE / prereg["resources"]["candidate_bank"]["path"]
    if args.candidate_bank.resolve() != expected_bank.resolve():
        raise ValueError("R2 activation candidate bank path drifted")
    expected_index = WORKSPACE / prereg["resources"]["compiled_scene_index"]["path"]
    if (args.compiled_scenes_dir / "compiled_scene_index.yaml").resolve() != expected_index.resolve():
        raise ValueError("R2 activation compiled scene directory drifted")
    output_root = args.output_root.resolve()
    output_root.relative_to((
        WORKSPACE / "artifacts/v2/calibration/v2_04g_r2/activation_probe"
    ).resolve())
    runtime = _BATCH.materialize_candidates(
        args.candidate_bank, output_root / "runtime_candidate_configs"
    )
    instances = _BATCH._R1._load_instances(args.compiled_scenes_dir)
    probe = prereg["activation_readiness_probe"]
    instance, _, world_path = instances[probe["scene_id"]]
    schedule = probe["schedule"]
    if len(schedule) != probe["planned_probe_count"]:
        raise ValueError("R2 activation budget drifted")
    reports = []
    environment = dict(os.environ)
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11311"
    for row in schedule:
        profile = row["profile_id"]
        if profile not in probe["profile_ids"]:
            raise ValueError("R2 activation profile set drifted")
        target = output_root / "probe_{:02d}_{}_repeat_{}".format(
            row["sequence"], profile, row["repeat"]
        )
        target.mkdir(parents=True, exist_ok=True)
        report_path = target / "report.yaml"
        if report_path.is_file():
            report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
            identity = {
                "stage": "V2-04G-R2", "profile_id": profile,
                "repeat": row["repeat"], "seed": row["seed"],
            }
            if all(report.get(k) == v for k, v in identity.items()) and report.get("all_hard_gates_pass") is True:
                reports.append(report)
                continue
            raise RuntimeError("existing R2 activation evidence failed identity/gate")
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
            str(args.listener), "--output", str(report_path), "--profile-id", profile,
            "--repeat", str(row["repeat"]), "--seed", str(row["seed"]),
            "--warmup-timeout-s", str(probe["warmup_timeout_s"]),
            "--measurement-duration-s", str(probe["measurement_duration_s"]),
            "--minimum-message-count", str(probe["minimum_message_count"]),
            "--minimum-valid-fraction", str(probe["minimum_valid_fraction"]),
        ]
        launch = None
        with (target / "launch.log").open("w", encoding="utf-8") as launch_log:
            try:
                launch = subprocess.Popen(
                    launch_command, env=environment, stdout=launch_log,
                    stderr=subprocess.STDOUT, start_new_session=True,
                )
                _BATCH._R1._ready(launch, environment, "rule_multi_anchor")
                with (target / "listener.log").open("w", encoding="utf-8") as listener_log:
                    result = subprocess.run(
                        listener_command, env=environment, stdout=listener_log,
                        stderr=subprocess.STDOUT, timeout=45.0, check=False,
                    )
                if result.returncode != 0:
                    raise RuntimeError("R2 activation listener failed")
            finally:
                if launch is not None:
                    _BATCH._R1._terminate_group(launch)
                time.sleep(1.0)
        report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
        if not report["all_hard_gates_pass"]:
            raise RuntimeError("R2 activation readiness hard gate failed")
        reports.append(report)
        print("PASS R2 activation {}/{} {} repeat {}".format(
            len(reports), len(schedule), profile, row["repeat"]
        ), flush=True)
    all_pass = len(reports) == len(schedule) and all(
        row["all_hard_gates_pass"] for row in reports
    )
    summary = {
        "schema_version": "2.0", "stage": "V2-04G-R2",
        "status": "complete" if all_pass else "incomplete",
        "simulation_only": True, "runtime_ready": False,
        "training_started": False, "real_vehicle_used": False,
        "preregistration": {
            "path": str(args.preregistration), "sha256": _sha256(args.preregistration),
        },
        "planned_probe_count": len(schedule), "valid_probe_count": len(reports),
        "all_probe_hard_gates_pass": all_pass,
        "reports": [{
            "profile_id": row["profile_id"], "repeat": row["repeat"],
            "seed": row["seed"],
            "transaction_activated_fraction": row["transaction_activated_fraction"],
            "join_valid_fraction": row["join_valid_fraction"],
            "join_reason_counts": row["join_reason_counts"],
        } for row in reports],
        "navigation_authorized": all_pass,
    }
    _write_yaml(output_root / "activation_probe_summary.yaml", summary)
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
