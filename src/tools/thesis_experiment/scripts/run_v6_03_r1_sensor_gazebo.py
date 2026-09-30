#!/usr/bin/env python3
"""Environment-only R1 wrapper around the frozen V6-03 Gazebo runner."""

import argparse
import importlib.util
import os
from pathlib import Path
import sys

import yaml


W = Path("/home/robot/robot_ws_base_rl")
BASE_RUNNER = W / "src/tools/thesis_experiment/scripts/run_v6_03_sensor_gazebo.py"
AMENDMENT = W / "src/application/v5_e2e_scope_rl/config/v6_03_r1_environment_repair.yaml"
FOUNDATION = W / "artifacts/v6/gazebo/v6_03_r1_a0_foundation"
SPEC = importlib.util.spec_from_file_location("v6_03_frozen_runner", str(BASE_RUNNER))
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


def prepare():
    if FOUNDATION.exists():
        raise RuntimeError("R1 foundation already exists")
    amendment = yaml.safe_load(BASE.read(AMENDMENT))
    contract = yaml.safe_load(BASE.read(BASE.CONTRACT))
    FOUNDATION.mkdir(parents=True)
    schedule_hashes = {}
    for phase in ("preflight", "qualification", "formal"):
        rows = BASE.make_rows(contract, phase)
        if phase == "preflight":
            fresh_seed = int(amendment["fresh_preflight"]["seed_start"])
            for row in rows:
                row["seed"] = fresh_seed
        path = FOUNDATION / "{}_exact_schedule.yaml".format(phase)
        BASE.dump(path, {"schema_version": "1.0", "stage": "V6-03-R1", "phase": phase, "rows": rows})
        schedule_hashes[phase] = BASE.sha(BASE.read(path))
    bindings = []
    paths = list(BASE.BINDINGS) + [
        "src/tools/thesis_experiment/scripts/run_v6_03_r1_sensor_gazebo.py",
        "src/application/v5_e2e_scope_rl/config/v6_03_r1_environment_repair.yaml",
        "artifacts/v6/gazebo/v6_03_p0_preflight/attempt_journal.jsonl",
    ]
    for relative in paths:
        path = W / relative
        if not path.is_file():
            raise RuntimeError("R1 binding absent: {}".format(relative))
        bindings.append({"path": relative, "sha256": BASE.sha(BASE.read(path))})
    for actor in contract["methods"]["frozen_actor_checkpoints"]:
        path = W / actor["path"]
        observed = BASE.sha(BASE.read(path))
        if observed != actor["sha256"]:
            raise RuntimeError("actor differs")
        bindings.append({"path": actor["path"], "sha256": observed})
    release = {
        "schema_version": "1.0",
        "stage": "V6-03-R1-A0",
        "status": "v6_03_r1_environment_repair_release_frozen",
        "base_contract_sha256": BASE.sha(BASE.read(BASE.CONTRACT)),
        "amendment_sha256": BASE.sha(BASE.read(AMENDMENT)),
        "schedule_sha256": schedule_hashes,
        "runtime_bindings": bindings,
        "only_factor": "network_namespace_execution_environment",
        "algorithm_or_threshold_change": False,
        "training_authorized": False,
        "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    BASE.dump(FOUNDATION / "execution_release.yaml", release)
    print("r1_release_sha256={}".format(BASE.sha(BASE.read(FOUNDATION / "execution_release.yaml"))))


def validate_release(phase):
    release_path = FOUNDATION / "execution_release.yaml"
    release = yaml.safe_load(BASE.read(release_path))
    if release["status"] != "v6_03_r1_environment_repair_release_frozen":
        raise RuntimeError("R1 release status differs")
    if release["base_contract_sha256"] != BASE.sha(BASE.read(BASE.CONTRACT)):
        raise RuntimeError("base contract differs")
    if release["amendment_sha256"] != BASE.sha(BASE.read(AMENDMENT)):
        raise RuntimeError("R1 amendment differs")
    for item in release["runtime_bindings"]:
        path = W / item["path"]
        if not path.is_file() or BASE.sha(BASE.read(path)) != item["sha256"]:
            raise RuntimeError("R1 runtime binding differs: {}".format(item["path"]))
    schedule_path = FOUNDATION / "{}_exact_schedule.yaml".format(phase)
    if BASE.sha(BASE.read(schedule_path)) != release["schedule_sha256"][phase]:
        raise RuntimeError("R1 schedule differs")
    if phase in ("qualification", "formal"):
        preflight = W / "artifacts/v6/gazebo/v6_03_r1_p0_preflight/matrix_summary.yaml"
        if not preflight.is_file() or yaml.safe_load(BASE.read(preflight))["phase_pass"] is not True:
            raise RuntimeError("R1 qualification requires R1 preflight pass")
    if phase == "formal":
        qualification = W / "artifacts/v6/gazebo/v6_03_r1_q1_qualification/matrix_summary.yaml"
        if not qualification.is_file() or yaml.safe_load(BASE.read(qualification))["qualification_pass"] is not True:
            raise RuntimeError("R1 formal requires R1 qualification pass")
    return release, yaml.safe_load(BASE.read(schedule_path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prepare", action="store_true")
    parser.add_argument("--phase", choices=("preflight", "qualification", "formal"))
    parser.add_argument("--output-root")
    parser.add_argument("--execute", action="store_true")
    arguments = parser.parse_args()
    if arguments.prepare:
        prepare()
        return 0
    if not arguments.execute or not arguments.phase or not arguments.output_root:
        parser.error("execution requires --phase, --output-root and --execute")
    BASE.FOUNDATION = FOUNDATION
    BASE.validate_release = validate_release
    BASE.execute(arguments.phase, arguments.output_root)
    return 0


if __name__ == "__main__":
    sys.exit(main())
