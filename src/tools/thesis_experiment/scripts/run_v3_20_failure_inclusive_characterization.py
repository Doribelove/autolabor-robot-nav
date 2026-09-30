#!/usr/bin/env python3
"""Run the fresh V3-20 failure-inclusive matrix through frozen V3-19 runtime."""
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ENGINE_PATH = W / "src/tools/thesis_experiment/scripts/run_v3_19_four_method_benchmark.py"
SPEC = importlib.util.spec_from_file_location("v3_20_frozen_v3_19_engine", str(ENGINE_PATH))
ENGINE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(ENGINE)
ORIGINAL_RUN_EPISODE = ENGINE.run_episode
ORIGINAL_SUMMARIZE = ENGINE.summarize
FOUNDATION = W / "artifacts/v3/thesis/v3_20_a0_characterization_foundation"
RELEASE = FOUNDATION / "execution_release.yaml"
AUTHORIZATION = FOUNDATION / "simulation_authorization.yaml"


def sha(data): return hashlib.sha256(data).hexdigest()


def validate_release(phase):
    if phase != "formal":
        raise RuntimeError("V3-20 has only one formal characterization matrix")
    release = yaml.safe_load(RELEASE.read_bytes())
    authorization = yaml.safe_load(AUTHORIZATION.read_bytes())
    if release["status"] != "v3_20_a0_failure_inclusive_release_reviewed" or authorization["execution_authorized"] is not True:
        raise RuntimeError("V3-20 release or authorization is closed")
    for item in release["runtime_bindings"]:
        path = W / item["path"]
        if not path.is_file() or sha(path.read_bytes()) != item["sha256"]:
            raise RuntimeError("release binding differs: {}".format(item["path"]))
    schedule_path = FOUNDATION / "formal_exact_schedule.yaml"
    if sha(schedule_path.read_bytes()) != release["formal_schedule_sha256"]:
        raise RuntimeError("formal schedule differs")
    return release, yaml.safe_load(schedule_path.read_bytes())


def run_episode(row, scene, root, port):
    receipt = ORIGINAL_RUN_EPISODE(row, scene, root, port)
    report = receipt["episode_report"]
    report["origin_execution_engine_stage"] = report["stage"]
    report["stage"] = "V3-20"
    report["study_design"] = "failure_inclusive_followup_characterization"
    report["navigation_failure_value_s"] = 22.0
    report_path = root / "episode_report.yaml"
    temporary = report_path.with_name(report_path.name + ".v3_20.tmp")
    with temporary.open("wb") as stream:
        stream.write(yaml.safe_dump(report, sort_keys=False, allow_unicode=True).encode("utf-8"))
        stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(report_path))
    receipt["episode_report"] = report
    receipt["episode_report_sha256"] = sha(report_path.read_bytes())
    return receipt


def summarize(receipts, phase, release_sha):
    value = ORIGINAL_SUMMARIZE(receipts, phase, release_sha)
    value["stage"] = "V3-20-F1"
    value["status"] = "failure_inclusive_characterization_execution_complete"
    value["qualification_checks"] = {}
    value["qualification_pass"] = None
    value["all_consumed_failures_retained_at_restricted_time"] = True
    return value


def main():
    ENGINE.FOUNDATION = FOUNDATION
    ENGINE.RELEASE = RELEASE
    ENGINE.AUTHORIZATION = AUTHORIZATION
    ENGINE.validate_release = validate_release
    ENGINE.run_episode = run_episode
    ENGINE.summarize = summarize
    return ENGINE.main()


if __name__ == "__main__":
    raise SystemExit(main())

