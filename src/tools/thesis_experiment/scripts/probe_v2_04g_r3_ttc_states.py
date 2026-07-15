#!/usr/bin/env python3
"""Produce the frozen three-state TTC component probe for V2-04G-R3."""

import argparse
import hashlib
from pathlib import Path

import yaml

from nav_world_model.risk_evidence import (
    RelativeTrack,
    classify_ttc_evidence,
    earliest_relative_ttc,
)


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--preregistration", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prereg = yaml.safe_load(args.preregistration.read_text(encoding="utf-8"))
    if not (
        prereg.get("stage") == "V2-04G-R3"
        and prereg.get("split") == "calibration"
        and prereg.get("training_allowed") is False
    ):
        raise ValueError("TTC probe requires R3 calibration preregistration")
    conflict = earliest_relative_ttc((RelativeTrack(
        x=3.0, y=-0.25, vx=-0.9, vy=0.05, radius=0.30,
        confidence=0.95, motion_class="UNKNOWN",
    ),))
    clear = earliest_relative_ttc((RelativeTrack(
        x=3.0, y=3.0, vx=-0.9, vy=0.0, radius=0.30,
        confidence=0.95, motion_class="UNKNOWN",
    ),))
    probes = [
        {"probe_id": "healthy_observed_conflict", "tracker_message_count": 1,
         "healthy_tracker_sample_count": 1,
         "finite_ttc_sample_count": int(conflict is not None), "finite_ttc_s": conflict},
        {"probe_id": "healthy_no_conflict_in_horizon", "tracker_message_count": 1,
         "healthy_tracker_sample_count": 1,
         "finite_ttc_sample_count": int(clear is not None), "finite_ttc_s": clear},
        {"probe_id": "tracker_invalid_fail_closed", "tracker_message_count": 0,
         "healthy_tracker_sample_count": 0, "finite_ttc_sample_count": 0,
         "finite_ttc_s": None},
    ]
    for probe in probes:
        probe["status"] = classify_ttc_evidence(
            tracker_message_count=probe["tracker_message_count"],
            healthy_tracker_sample_count=probe["healthy_tracker_sample_count"],
            finite_ttc_sample_count=probe["finite_ttc_sample_count"],
        )
    expected = ["OBSERVED_CONFLICT", "NO_CONFLICT_IN_HORIZON", "TRACKER_INVALID"]
    observed = [probe["status"] for probe in probes]
    report = {
        "schema_version": "2.0", "stage": "V2-04G-R3",
        "status": "complete" if observed == expected else "failed",
        "simulation_only": True, "runtime_ready": False,
        "training_used": False, "real_vehicle_used": False,
        "preregistration": {
            "path": str(args.preregistration), "sha256": _sha256(args.preregistration),
        },
        "probe_count": len(probes), "expected_status_order": expected,
        "observed_status_order": observed,
        "all_three_states_pass": observed == expected, "probes": probes,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    temporary.replace(args.output)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0 if report["all_three_states_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
