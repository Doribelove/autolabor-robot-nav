#!/usr/bin/env python3
"""Independent review of the V3-19-Q1 valid negative qualification."""
import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_19_q1_execution"
OUT = ROOT / "independent_qualification_review.yaml"
FREEZE = ROOT / "terminal_freeze.sha256"


def sha(data): return hashlib.sha256(data).hexdigest()


def atomic(path, data, check):
    if check:
        if not path.is_file() or path.read_bytes() != data: raise RuntimeError("{} differs".format(path))
        return
    if path.exists(): raise RuntimeError("{} exists".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream: stream.write(data); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    manifest = ROOT / "qualification_evidence_manifest.sha256"
    assessment_path = ROOT / "qualification_assessment.yaml"
    assessment = yaml.safe_load(assessment_path.read_bytes())
    entries = [line.split("  ", 1) for line in manifest.read_text().splitlines() if line]
    checks = {
        "manifest_rehash": all((W / path).is_file() and sha((W / path).read_bytes()) == digest for digest, path in entries),
        "negative_gate_not_overridden": assessment["qualification_pass"] is False and assessment["formal_execution_authorized"] is False,
        "failure_identity_preserved": assessment["main_failure_identity"].startswith("seed7906_") and "OCCUPANCY_INTERSECTION" in assessment["main_failure_mechanism"],
        "all_execution_integrity_and_safety_gates_pass": all(assessment["checks"][key] for key in ("execution_integrity", "zero_contact", "nonnegative_clearance", "non_teb_selected_accepted")),
        "only_performance_gate_failed": assessment["failed_preregistered_gates"] == ["main_goal_success"],
        "retired_formal_seeds": assessment["reserved_formal_seed_range_retired"] == [8001, 8030],
    }
    value = {
        "schema_version": "1.0", "stage": "V3-19-Q1",
        "status": "v3_19_q1_independent_valid_negative_review_pass" if all(checks.values()) else "v3_19_q1_independent_valid_negative_review_fail",
        "checks": checks, "assessment_sha256": sha(assessment_path.read_bytes()),
        "manifest_sha256": sha(manifest.read_bytes()), "v3_19_formal_started": False,
        "training_authorized": False, "hardware_accessed": False, "real_vehicle_used": False,
    }
    body = yaml.safe_dump(value, sort_keys=False).encode("utf-8"); atomic(OUT, body, args.check)
    paths = (manifest, assessment_path, ROOT / "matrix_summary.yaml", ROOT / "exact_schedule.yaml", ROOT / "attempt_journal.jsonl", OUT)
    freeze = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in paths).encode("utf-8"); atomic(FREEZE, freeze, args.check)
    print("status={} review_sha256={} freeze_sha256={}".format(value["status"], sha(body), sha(freeze)))


if __name__ == "__main__": main()

