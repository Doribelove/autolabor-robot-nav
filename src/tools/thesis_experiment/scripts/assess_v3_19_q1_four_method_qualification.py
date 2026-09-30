#!/usr/bin/env python3
"""Deterministically assess and freeze the terminal V3-19-Q1 result."""
import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_19_q1_execution"
CONTRACT = W / "config/thesis_experiments/v3/v3_19_a0_four_method_benchmark_contract.yaml"
RELEASE = W / "artifacts/v3/thesis/v3_19_a0_benchmark_foundation/execution_release.yaml"
OUT = ROOT / "qualification_assessment.yaml"
MANIFEST = ROOT / "qualification_evidence_manifest.sha256"


def sha(data): return hashlib.sha256(data).hexdigest()


def atomic(path, data, check):
    if check:
        if not path.is_file() or path.read_bytes() != data: raise RuntimeError("{} differs".format(path))
        return
    if path.exists(): raise RuntimeError("{} exists".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream: stream.write(data); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def outputs():
    summary_path = ROOT / "matrix_summary.yaml"
    schedule_path = ROOT / "exact_schedule.yaml"
    journal_path = ROOT / "attempt_journal.jsonl"
    summary = yaml.safe_load(summary_path.read_bytes())
    schedule = yaml.safe_load(schedule_path.read_bytes())
    receipts = sorted((ROOT / "episodes").glob("*/matrix_receipt.yaml"))
    reports = []
    resources = [CONTRACT, RELEASE, summary_path, schedule_path, journal_path, Path(__file__).resolve(), W / "src/tools/thesis_experiment/scripts/review_v3_19_q1_four_method_qualification.py"]
    for path in receipts:
        receipt = yaml.safe_load(path.read_bytes())
        report_path = path.with_name("episode_report.yaml")
        if sha(report_path.read_bytes()) != receipt["episode_report_sha256"]:
            raise RuntimeError("report binding differs: {}".format(path))
        if receipt["row"] != schedule["rows"][int(receipt["row"]["sequence"]) - 1]:
            raise RuntimeError("schedule identity differs: {}".format(path))
        reports.append(receipt["episode_report"])
        resources.extend((path, report_path, path.with_name("world.sdf"), path.with_name("roslaunch.log")))
    checks = dict(summary["qualification_checks"])
    checks["all_24_receipts_replayed"] = len(receipts) == 24
    checks["all_attempt1_no_retry"] = all(receipt["attempt"] == 1 for receipt in schedule["rows"])
    checks["all_simulation_only"] = all(report["simulation_only"] for report in reports)
    checks["no_hardware_or_real_vehicle"] = all(not report["hardware_accessed"] and not report["real_vehicle_used"] for report in reports)
    value = {
        "schema_version": "1.0", "stage": "V3-19-Q1",
        "status": "v3_19_q1_valid_negative_qualification_result",
        "qualification_pass": False,
        "failed_preregistered_gates": sorted(key for key, passed in checks.items() if not passed),
        "checks": checks,
        "main_failure_identity": "seed7906_bounded_corridor_crossing_110_hesitant_v3_spatial_safe",
        "main_failure_mechanism": "spatial candidate rejected for OCCUPANCY_INTERSECTION; accepted certified stop retained",
        "method_summary": summary["method_summary"],
        "maximum_gate_actor_coordinate_range_m": summary["maximum_gate_actor_coordinate_range_m"],
        "maximum_gate_actor_speed_range_mps": summary["maximum_gate_actor_speed_range_mps"],
        "formal_execution_authorized": False,
        "reserved_formal_seed_range_retired": [8001, 8030],
        "training_steps": 0, "hardware_accessed": False, "real_vehicle_used": False,
    }
    body = yaml.safe_dump(value, sort_keys=False).encode("utf-8")
    generated = {OUT: body}
    manifest = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in sorted(set(resources)))
    manifest += "{}  {}\n".format(sha(body), OUT.relative_to(W))
    generated[MANIFEST] = manifest.encode("utf-8")
    return generated, value


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    generated, value = outputs()
    for path, data in generated.items(): atomic(path, data, args.check)
    print("status={} manifest_sha256={}".format(value["status"], sha(generated[MANIFEST])))


if __name__ == "__main__": main()

