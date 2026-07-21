#!/usr/bin/env python3
"""Run the exact one-attempt, ROS-free R5 TTC three-state component gate."""

import argparse
import importlib.util
from pathlib import Path

import yaml

from nav_world_model.risk_evidence import (
    RelativeTrack,
    classify_ttc_evidence,
    earliest_relative_ttc,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-R5"
ROOT = WORKSPACE / "artifacts/v2/calibration/v2_04g_r5"
READINESS_SUMMARY = (
    ROOT / "readiness/v2_04g_r5_readiness_summary.yaml"
)
OUTPUT = ROOT / "v2_04g_r5_ttc_three_state_probe.yaml"
GUARD = Path(__file__).with_name("v2_04g_r5_execution_guard.py")


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_GUARD = _load_module("v2_04g_r5_execution_guard_for_ttc_probe", GUARD)


def _validate_schedule(prereg):
    component = prereg["ttc_component_probe"]
    schedule = component["schedule"]
    if not (
        component["implementation"]
        == "deterministic_ros_free_component_fixture"
        and component["seed_consumption"] == "none"
        and component["planned_probe_count"] == 3
        and component["attempts_per_identity_max"] == 1
        and [row["sequence"] for row in schedule] == [1, 2, 3]
        and [row["expected_status"] for row in schedule]
        == [
            "OBSERVED_CONFLICT",
            "NO_CONFLICT_IN_HORIZON",
            "TRACKER_INVALID",
        ]
        and all(row["attempt_limit"] == 1 for row in schedule)
        and prereg["budget"]["ttc_component_probe_count"] == 3
        and prereg["budget"]["attempts_per_ttc_component_identity_max"] == 1
    ):
        raise RuntimeError("R5 TTC component schedule or budget drifted")
    return schedule


def _require_readiness():
    if not READINESS_SUMMARY.is_file():
        raise RuntimeError("R5 TTC component refused before readiness evidence")
    summary = yaml.safe_load(READINESS_SUMMARY.read_text(encoding="utf-8"))
    if not (
        summary.get("stage") == STAGE
        and summary.get("status") == "complete"
        and summary.get("planned_probe_count") == 6
        and summary.get("attempted_identity_count") == 6
        and summary.get("valid_probe_count") == 6
        and summary.get("evidence_unit_count") == 6
        and summary.get("all_probe_hard_gates_pass") is True
        and summary.get("readiness_pass") is True
        and summary.get("component_probe_authorized") is True
        and summary.get("terminal_failure") is None
        and summary.get("resume_forbidden") is True
        and summary.get("preregistration", {}).get("sha256")
        == _GUARD.PREREGISTRATION_SHA256
        and len(summary.get("attempt_ledger", [])) == 6
        and all(
            row.get("attempt") == 1
            and row.get("status") == "evidence_complete"
            for row in summary["attempt_ledger"]
        )
    ):
        raise RuntimeError("R5 TTC component refused: readiness gate failed")
    for record in summary["reports"]:
        for path_key, hash_key in (
            ("activation_report", "activation_report_sha256"),
            ("evaluation", "evaluation_sha256"),
        ):
            path = Path(record[path_key])
            if not path.is_file() or _GUARD.sha256(path) != record[hash_key]:
                raise RuntimeError("R5 readiness evidence hash drifted")
    return summary


def _fixture(expected_status):
    if expected_status == "OBSERVED_CONFLICT":
        finite = earliest_relative_ttc((RelativeTrack(
            x=3.0,
            y=-0.25,
            vx=-0.9,
            vy=0.05,
            radius=0.30,
            confidence=0.95,
            motion_class="UNKNOWN",
        ),))
        tracker_count, healthy_count = 1, 1
    elif expected_status == "NO_CONFLICT_IN_HORIZON":
        finite = earliest_relative_ttc((RelativeTrack(
            x=3.0,
            y=3.0,
            vx=-0.9,
            vy=0.0,
            radius=0.30,
            confidence=0.95,
            motion_class="UNKNOWN",
        ),))
        tracker_count, healthy_count = 1, 1
    elif expected_status == "TRACKER_INVALID":
        finite = None
        tracker_count, healthy_count = 0, 0
    else:
        raise RuntimeError("unknown preregistered TTC component state")
    finite_count = int(finite is not None)
    observed = classify_ttc_evidence(
        tracker_message_count=tracker_count,
        healthy_tracker_sample_count=healthy_count,
        finite_ttc_sample_count=finite_count,
    )
    return {
        "tracker_message_count": tracker_count,
        "healthy_tracker_sample_count": healthy_count,
        "finite_ttc_sample_count": finite_count,
        "finite_ttc_s": finite,
        "status": observed,
    }


def _initial_report(schedule, readiness):
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "stage": STAGE,
        "split": "calibration",
        "status": "in_progress",
        "simulation_only": True,
        "calibration_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_used": False,
        "real_vehicle_used": False,
        "seed_consumption": "none",
        "preregistration": {
            "path": str(_GUARD.PREREGISTRATION.relative_to(WORKSPACE)),
            "sha256": _GUARD.PREREGISTRATION_SHA256,
        },
        "readiness_summary": {
            "path": str(READINESS_SUMMARY),
            "sha256": _GUARD.sha256(READINESS_SUMMARY),
        },
        "planned_probe_count": len(schedule),
        "probe_count": 0,
        "attempted_identity_count": 0,
        "valid_probe_count": 0,
        "evidence_unit_count": 0,
        "attempt_limit_per_identity": 1,
        "attempts_per_identity_max": 1,
        "retry_count": 0,
        "resume_used": False,
        "attempt_ledger": [],
        "probes": [],
        "expected_status_order": [
            row["expected_status"] for row in schedule
        ],
        "observed_status_order": [],
        "all_three_states_pass": False,
        "navigation_authorized": False,
        "terminal_failure": None,
        "resume_forbidden": False,
        "held_out_seed_consumption": False,
    }


def execute():
    prereg, _, _ = _GUARD.verify_frozen_start()
    _GUARD.assert_thesis_workspace_environment()
    _GUARD.assert_no_live_runtime_processes()
    schedule = _validate_schedule(prereg)
    readiness = _require_readiness()
    if OUTPUT.exists():
        raise RuntimeError(
            "R5 TTC component has existing state; retry/resume is forbidden"
        )
    report = _initial_report(schedule, readiness)
    _GUARD.atomic_yaml(OUTPUT, report)
    for row in schedule:
        ledger = {
            "sequence": row["sequence"],
            "identity": row["identity"],
            "attempt": 1,
            "attempt_limit": 1,
            "status": "attempt_started",
            "resume_forbidden_if_interrupted": True,
        }
        report["attempt_ledger"].append(ledger)
        report["attempted_identity_count"] = len(report["attempt_ledger"])
        report["evidence_unit_count"] = len(report["attempt_ledger"])
        report["active_identity"] = row["identity"]
        _GUARD.atomic_yaml(OUTPUT, report)
        try:
            probe = {
                "sequence": row["sequence"],
                "identity": row["identity"],
                "attempt": 1,
                "expected_status": row["expected_status"],
                **_fixture(row["expected_status"]),
            }
            probe["observed_status"] = probe["status"]
            probe["pass"] = probe["status"] == row["expected_status"]
            if probe["status"] != row["expected_status"]:
                raise RuntimeError("R5 TTC component state mismatch")
            ledger.update({
                "status": "evidence_complete",
                "expected_status": row["expected_status"],
                "observed_status": probe["status"],
            })
            report["probes"].append(probe)
            report["observed_status_order"].append(probe["status"])
            report["valid_probe_count"] = len(report["probes"])
            report["probe_count"] = len(report["probes"])
            report["active_identity"] = None
            _GUARD.atomic_yaml(OUTPUT, report)
            print(
                "PASS TTC component {}/3 {}".format(
                    row["sequence"], probe["status"]
                ),
                flush=True,
            )
        except Exception as exc:
            ledger.update({
                "status": "terminal_failure",
                "reason": str(exc),
                "resume_forbidden": True,
            })
            report.update({
                "status": "terminal_failure",
                "terminal_failure": {
                    "identity": row["identity"],
                    "sequence": row["sequence"],
                    "reason": str(exc),
                },
                "resume_forbidden": True,
                "navigation_authorized": False,
            })
            _GUARD.atomic_yaml(OUTPUT, report)
            raise RuntimeError(
                "R5 TTC component terminal failure; retry/resume forbidden"
            ) from exc
    passed = (
        report["observed_status_order"] == report["expected_status_order"]
        and len(report["probes"]) == 3
        and all(
            row["status"] == "evidence_complete"
            for row in report["attempt_ledger"]
        )
    )
    report.update({
        "status": "complete" if passed else "terminal_failure",
        "all_three_states_pass": passed,
        "navigation_authorized": passed,
        "resume_forbidden": True,
    })
    if not passed:
        report["terminal_failure"] = {
            "identity": "aggregate-component-gate",
            "reason": "R5 TTC component aggregate gate failed",
        }
    _GUARD.atomic_yaml(OUTPUT, report)
    if not passed:
        raise RuntimeError("R5 TTC component gate failed; resume forbidden")
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


def dry_run():
    prereg, _, _ = _GUARD.verify_frozen_start()
    schedule = _validate_schedule(prereg)
    if OUTPUT.exists():
        raise RuntimeError("R5 TTC component state already exists")
    print("R5 TTC component dry-run: exact 3 identities; no file written")
    for row in schedule:
        print(
            "{:02d} {} expected={}".format(
                row["sequence"], row["identity"], row["expected_status"]
            )
        )
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--execute", action="store_true")
    args = parser.parse_args()
    return dry_run() if args.dry_run else execute()


if __name__ == "__main__":
    raise SystemExit(main())
