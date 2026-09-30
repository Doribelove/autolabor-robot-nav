#!/usr/bin/env python3
"""Independent fail-closed review of the V3-14-S1 simulation release."""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/multiactor/v3_14_s1_readiness"
OUT = ROOT / "independent_review.yaml"


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def load(path):
    return yaml.safe_load(path.read_bytes())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    schedule_path = ROOT / "exact_schedule.yaml"
    dependency_path = ROOT / "dependency_freeze.yaml"
    release_path = ROOT / "execution_release.yaml"
    schedule = load(schedule_path)
    dependencies = load(dependency_path)
    release = load(release_path)
    contract = load(W / "config/thesis_experiments/v3/v3_14_s1_multiaction_support_contract.yaml")
    checks = {}
    checks["release_binds_schedule"] = release["exact_schedule_sha256"] == sha(schedule_path.read_bytes())
    checks["release_binds_dependencies"] = release["dependency_freeze_sha256"] == sha(dependency_path.read_bytes())
    checks["release_binds_contract"] = release["contract_sha256"] == sha((W / "config/thesis_experiments/v3/v3_14_s1_multiaction_support_contract.yaml").read_bytes())
    checks["all_dependencies_rehash"] = all(sha((W / item["path"]).read_bytes()) == item["sha256"] for item in dependencies["resources"])
    rows = schedule["rows"]
    checks["exact_45_rows"] = len(rows) == 45 and [r["sequence"] for r in rows] == list(range(1, 46))
    checks["phase_counts"] = sum(r["phase"] == "preflight" for r in rows) == 5 and sum(r["phase"] == "evidence" for r in rows) == 40
    checks["fresh_seed_roster"] = {r["seed"] for r in rows} == {7719, *range(7721, 7729)} and min(r["seed"] for r in rows) > 7704
    actions = set(contract["population"]["actions"])
    checks["full_support_each_block"] = all(
        {r["forced_candidate_id"] for r in rows if r["phase"] == phase and r["block_index"] == block} == actions
        for phase, blocks in (("preflight", [1]), ("evidence", range(1, 9))) for block in blocks
    )
    binding_path = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_14_action_binding.py"
    spec = importlib.util.spec_from_file_location("v3_14_s1_review_binding", binding_path)
    binding = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(binding)
    replayed = 0
    for row in rows:
        command = ["roslaunch", "m2_gazebo", contract["execution"]["launch"]]
        command += binding.build_launch_binding_args(row, tuple(actions))
        binding.verify_preconsumption_binding(row, command, tuple(actions))
        replayed += 1
    checks["all_preconsumption_bindings_replayed"] = replayed == 45
    checks["bounded_simulation_only"] = release["execution_authorized"] and not any(
        release[key] for key in ("predictor_fit_authorized", "actual_TEB_execution_authorized", "perception_execution_authorized", "training_authorized", "hardware_device_access_authorized", "real_vehicle_authorized")
    )
    report = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-14-S1",
        "status": "v3_14_s1_independent_release_review_pass" if all(checks.values()) else "v3_14_s1_independent_release_review_fail",
        "execution_release_sha256": sha(release_path.read_bytes()), "exact_schedule_sha256": sha(schedule_path.read_bytes()),
        "replayed_binding_rows": replayed, "checks": checks,
        "execution_ready": all(checks.values()), "execution_authorized": all(checks.values()),
        "hardware_device_access_authorized": False, "real_vehicle_authorized": False,
    }
    payload = yaml.safe_dump(report, sort_keys=False, allow_unicode=True).encode("utf-8")
    if args.check:
        if not OUT.is_file() or OUT.read_bytes() != payload:
            raise RuntimeError("independent review differs")
    else:
        if OUT.exists():
            raise RuntimeError("independent review exists")
        temporary = OUT.with_name(OUT.name + ".tmp")
        with temporary.open("wb") as stream:
            stream.write(payload); stream.flush(); os.fsync(stream.fileno())
        os.replace(str(temporary), str(OUT))
    print("status={} review_sha256={}".format(report["status"], sha(payload)))


if __name__ == "__main__":
    main()

