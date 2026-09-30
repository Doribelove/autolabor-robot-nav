#!/usr/bin/env python3
"""Freeze the valid V3-06-C4 offline gate failure."""

import argparse
import hashlib
from pathlib import Path

import yaml

W = Path("/home/robot/robot_ws_base_rl")
ROOT = Path("artifacts/v3/development/v3_06_c4")
REPORT = ROOT / "v3_06_c4_offline_equivalence_report.yaml"
REVIEW = ROOT / "v3_06_c4_failure_review.yaml"
TERMINAL = ROOT / "v3_06_c4_failure_terminal_freeze.sha256"


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload: raise RuntimeError("refusing overwrite")
    else: path.write_bytes(payload)


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    report_bytes = (W / REPORT).read_bytes()
    report = yaml.safe_load(report_bytes)
    if not (
        report["overall_pass"] is False
        and report["metrics"]["removed_candidate_selected_count"] == 2
        and report["metrics"]["selected_trajectory_exact_rate"] == 1.0
        and report["offline_gates"]["removed_candidate_unselected_pass"] is False
    ):
        raise RuntimeError("C4 failure differs")
    value = {
        "schema_version": "1.0", "architecture_generation": "v3", "stage": "V3-06-C4",
        "status": "v3_06_c4_candidate_identity_gate_failure_review_pass",
        "report_sha256": sha(report_bytes), "removed_candidate_selected_count": 2,
        "trajectory_equivalence_does_not_override_identity_gate": True,
        "fresh_qualification_authorized": False, "simulation_authorized": False,
        "training_authorized": False, "real_vehicle_authorized": False,
    }
    payload = yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, width=1000000
    ).encode("utf-8")
    freeze = (
        "{}  {}\n{}  {}\n".format(
            sha(report_bytes), REPORT.as_posix(), sha(payload), REVIEW.as_posix()
        )
    ).encode("utf-8")
    if args.check:
        if (W / REVIEW).read_bytes() != payload or (W / TERMINAL).read_bytes() != freeze:
            raise RuntimeError("C4 failure freeze differs")
    else:
        write_once(W / REVIEW, payload); write_once(W / TERMINAL, freeze)
    print("status={} review_sha256={}".format(value["status"], sha(payload)))


if __name__ == "__main__": main()
