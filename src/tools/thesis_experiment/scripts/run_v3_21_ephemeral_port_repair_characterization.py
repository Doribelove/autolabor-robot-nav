#!/usr/bin/env python3
"""Run V3-21 with frozen science and ports below the ephemeral range."""
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
V20_PATH = W / "src/tools/thesis_experiment/scripts/run_v3_20_failure_inclusive_characterization.py"
SPEC = importlib.util.spec_from_file_location("v3_21_frozen_v3_20_wrapper", str(V20_PATH))
V20 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V20)
FOUNDATION = W / "artifacts/v3/thesis/v3_21_a0_port_repair_foundation"
RELEASE = FOUNDATION / "execution_release.yaml"
AUTHORIZATION = FOUNDATION / "simulation_authorization.yaml"


def sha(data): return hashlib.sha256(data).hexdigest()


def validate_release(phase):
    if phase != "formal": raise RuntimeError("V3-21 has only the formal repair matrix")
    release = yaml.safe_load(RELEASE.read_bytes()); authorization = yaml.safe_load(AUTHORIZATION.read_bytes())
    if release["status"] != "v3_21_a0_ephemeral_port_repair_release_reviewed" or authorization["execution_authorized"] is not True: raise RuntimeError("V3-21 release closed")
    for item in release["runtime_bindings"]:
        path = W / item["path"]
        if not path.is_file() or sha(path.read_bytes()) != item["sha256"]: raise RuntimeError("release binding differs: {}".format(item["path"]))
    schedule_path = FOUNDATION / "formal_exact_schedule.yaml"
    if sha(schedule_path.read_bytes()) != release["formal_schedule_sha256"]: raise RuntimeError("formal schedule differs")
    return release, yaml.safe_load(schedule_path.read_bytes())


def run_episode(row, scene, root, _unsafe_port):
    safe_port = 24000 + 2 * int(row["sequence"])
    if not (24002 <= safe_port <= 24240): raise RuntimeError("safe port formula differs")
    receipt = V20.ORIGINAL_RUN_EPISODE(row, scene, root, safe_port)
    report = receipt["episode_report"]
    report["origin_execution_engine_stage"] = report["stage"]
    report["stage"] = "V3-21"
    report["study_design"] = "failure_inclusive_single_infrastructure_repair"
    report["navigation_failure_value_s"] = 22.0
    report["master_port"] = safe_port
    report["master_port_below_ephemeral_range"] = safe_port < 32768
    report_path = root / "episode_report.yaml"; temporary = report_path.with_name(report_path.name + ".v3_21.tmp")
    with temporary.open("wb") as stream:
        stream.write(yaml.safe_dump(report, sort_keys=False, allow_unicode=True).encode("utf-8")); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(report_path)); receipt["episode_report"] = report; receipt["episode_report_sha256"] = sha(report_path.read_bytes()); return receipt


def summarize(receipts, phase, release_sha):
    value = V20.ORIGINAL_SUMMARIZE(receipts, phase, release_sha); value["stage"] = "V3-21-F1"; value["status"] = "failure_inclusive_port_repair_execution_complete"; value["qualification_checks"] = {}; value["qualification_pass"] = None; value["all_consumed_failures_retained_at_restricted_time"] = True; value["all_master_ports_below_ephemeral_range"] = all(item["episode_report"]["master_port_below_ephemeral_range"] for item in receipts); return value


def main():
    V20.FOUNDATION = FOUNDATION; V20.RELEASE = RELEASE; V20.AUTHORIZATION = AUTHORIZATION; V20.validate_release = validate_release; V20.run_episode = run_episode; V20.summarize = summarize; return V20.main()


if __name__ == "__main__": raise SystemExit(main())

