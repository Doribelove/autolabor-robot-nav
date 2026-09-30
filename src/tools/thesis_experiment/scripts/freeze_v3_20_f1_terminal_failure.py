#!/usr/bin/env python3
"""Freeze the non-resumable V3-20 port-collision terminal failure."""
import argparse
import hashlib
import json
import os
from collections import Counter
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_20_f1_execution"
OUT = ROOT / "terminal_failure_assessment.yaml"
MANIFEST = ROOT / "terminal_evidence_manifest.sha256"


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
    schedule_path = ROOT / "exact_schedule.yaml"; journal_path = ROOT / "attempt_journal.jsonl"
    schedule = yaml.safe_load(schedule_path.read_bytes()); events = [json.loads(line) for line in journal_path.read_text().splitlines() if line]
    receipts = sorted((ROOT / "episodes").glob("*/matrix_receipt.yaml")); reports = []
    resources = [schedule_path, journal_path, W / "artifacts/v3/thesis/v3_20_a0_characterization_foundation/execution_release.yaml", Path(__file__).resolve(), W / "src/tools/thesis_experiment/scripts/review_v3_20_f1_terminal_failure.py"]
    for path in receipts:
        receipt = yaml.safe_load(path.read_bytes()); report_path = path.with_name("episode_report.yaml")
        if sha(report_path.read_bytes()) != receipt["episode_report_sha256"]: raise RuntimeError("report digest differs")
        reports.append(receipt["episode_report"]); resources.extend((path, report_path, path.with_name("world.sdf"), path.with_name("roslaunch.log")))
    failure = next(event for event in reversed(events) if event["event"] == "terminal_failure")
    failed_root = next(path for path in (ROOT / "episodes").glob("078_*") if path.is_dir())
    resources.extend((failed_root / "world.sdf", failed_root / "roslaunch.log"))
    counts = Counter(report["method"] for report in reports)
    value = {
        "schema_version": "1.0", "stage": "V3-20-F1",
        "status": "terminal_infrastructure_failure_nonresumable",
        "authorized_units": len(schedule["rows"]), "consumed_units": 78,
        "completed_units": len(receipts), "terminal_failed_units": 1,
        "forfeited_units": failure["remaining_units_forfeited"],
        "completed_method_counts": dict(sorted(counts.items())),
        "failure_identity": failure["row"],
        "failure_before_semantic_execution": True,
        "failure_cause": "ROS master port 34156 collided with a Linux ephemeral port",
        "linux_ephemeral_port_range": [32768, 60999],
        "frozen_port_formula": "34000 + 2 * sequence",
        "failed_port": 34156,
        "scientific_matrix_complete": False,
        "formal_or_inferential_result": False,
        "partial_results_descriptive_only": True,
        "retry_resume_replacement_authorized": False,
        "seed_range_8101_8130_reuse_forbidden": True,
        "next_repair_constraint": "new stage, fresh seeds, predeclared ports below 32768, independent release",
        "all_completed_execution_integrity_pass": all(report["execution_integrity_pass"] for report in reports),
        "completed_contact_count": sum(report["result"]["contact_state_count"] for report in reports),
        "training_steps": 0, "hardware_accessed": False, "real_vehicle_used": False,
    }
    body = yaml.safe_dump(value, sort_keys=False).encode("utf-8"); generated = {OUT: body}
    manifest = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in sorted(set(resources)))
    manifest += "{}  {}\n".format(sha(body), OUT.relative_to(W)); generated[MANIFEST] = manifest.encode("utf-8")
    return generated, value


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args(); generated, value = outputs()
    for path, data in generated.items(): atomic(path, data, args.check)
    print("status={} completed={} forfeited={} manifest_sha256={}".format(value["status"], value["completed_units"], value["forfeited_units"], sha(generated[MANIFEST])))


if __name__ == "__main__": main()

