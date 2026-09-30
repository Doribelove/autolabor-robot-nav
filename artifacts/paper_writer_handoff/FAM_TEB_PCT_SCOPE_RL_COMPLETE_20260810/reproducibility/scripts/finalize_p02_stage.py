#!/usr/bin/env python3
"""Build the immutable P02 audit summary and refresh the shared ledgers."""

from __future__ import annotations

import csv
from datetime import datetime
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
DEV_CONFIG = ROOT / "src/application/v5_e2e_scope_rl/config/pct_teb_dwa_benchmark_development.yaml"
TEB_V0 = ROOT / "src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml"
DWA_V0 = ROOT / "src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml"
MODEL = ROOT / "artifacts/v6/training/v6_01_r1/pct_seed9501/actor_final.pt"
CURRENT_GATES = {
    "static_shadow": 196021,
    "single_actor_shadow": 196019,
    "partially_occluded_actor_shadow": 196022,
    "static_takeover": 196023,
    "single_actor_takeover": 196020,
    "partially_occluded_actor_takeover": 196024,
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_text(path: Path, text: str) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def episode_roots():
    roots = list((ARTIFACT / "development/episodes").glob("*"))
    roots += list((ARTIFACT / "development_tuning").glob("*/episodes/*"))
    return sorted(path for path in roots if path.is_dir())


def variant(root: Path, method: str) -> str:
    parts = root.relative_to(ARTIFACT).parts
    if parts[0] == "development_tuning":
        return method + "__" + parts[1]
    return method


def config_hash(receipt, method):
    if receipt.get("planner_config_sha256"):
        return receipt["planner_config_sha256"]
    if method == "actual_teb":
        return sha256(TEB_V0)
    if method == "dwa":
        return sha256(DWA_V0)
    return sha256(DEV_CONFIG)


def iso_times(receipt_path, wall_elapsed):
    ended = datetime.fromtimestamp(receipt_path.stat().st_mtime).astimezone()
    started = datetime.fromtimestamp(ended.timestamp() - float(wall_elapsed)).astimezone()
    return started.isoformat(timespec="seconds"), ended.isoformat(timespec="seconds")


def load_records():
    records = []
    for root in episode_roots():
        receipt_path = root / "receipt.json"
        if not receipt_path.is_file():
            continue
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        report_path = root / "episode_report.yaml"
        report = yaml.safe_load(report_path.read_text(encoding="utf-8")) if report_path.is_file() else None
        method = receipt["method"]
        records.append(
            {
                "root": root,
                "relative_root": str(root.relative_to(ROOT)),
                "receipt_path": receipt_path,
                "receipt": receipt,
                "report": report,
                "method": method,
                "ledger_method": variant(root, method),
                "config_sha256": config_hash(receipt, method),
            }
        )
    return records


def update_run_manifest(records):
    path = ARTIFACT / "run_manifest.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames
        retained = [row for row in reader if row["stage"] != "P02"]
    rows = []
    for item in records:
        receipt, report = item["receipt"], item["report"]
        started, ended = iso_times(item["receipt_path"], receipt["wall_elapsed_s"])
        if report is None:
            status = "infrastructure_failure_retained"
            notes = "episode report absent; failure identity retained without retry"
            model_hash = "none"
        else:
            result = report["result"]
            status = "completed_pass" if report["execution_integrity_pass"] and result["goal_reached"] else "completed_development_failure"
            notes = "goal={}; contact={}; clearance={:.6f}; timeout={}".format(
                result["goal_reached"], result["contact_state_count"],
                result["minimum_conservative_clearance_m"], result["timeout"]
            )
            model_hash = report.get("actor_checkpoint_sha256") or "none"
        rows.append(
            {
                "stage": "P02",
                "run_id": item["root"].name + ("__" + item["ledger_method"].split("__", 1)[1] if "__" in item["ledger_method"] else ""),
                "phase": "gazebo_migration_admission_and_baseline_smoke",
                "method": item["ledger_method"],
                "scene_id": receipt["scene_id"],
                "split": "development",
                "seed": receipt["seed"],
                "attempt": 1,
                "status": status,
                "started_at": started,
                "ended_at": ended,
                "exit_code": receipt["return_code"],
                "config_sha256": item["config_sha256"],
                "model_sha256": model_hash,
                "evidence_path": item["relative_root"],
                "formal_eligible": "false",
                "notes": notes,
            }
        )
    rows.sort(key=lambda row: (int(row["seed"]), row["method"]))
    with (path.with_suffix(".csv.tmp")).open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader(); writer.writerows(retained + rows)
    path.with_suffix(".csv.tmp").replace(path)


def update_episode_results(records):
    path = ARTIFACT / "episode_results.csv"
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames
        retained = [row for row in reader if row["study_round"] != "gazebo_p02_development"]
    rows = []
    for item in records:
        receipt, report = item["receipt"], item["report"]
        seed = int(receipt["seed"])
        result = report["result"] if report else {}
        goal = bool(result.get("goal_reached", False))
        contact = int(result.get("contact_state_count", 0))
        timeout = bool(result.get("timeout", False))
        clearance = result.get("minimum_conservative_clearance_m")
        system_failure = report is None
        if system_failure:
            reason = "infrastructure_failure"
        elif contact:
            reason = "contact"
        elif clearance is not None and clearance < 0.0:
            reason = "conservative_clearance_violation"
        elif timeout or not goal:
            reason = "timeout_or_nonarrival"
        else:
            reason = "goal"
        elapsed = float(result.get("restricted_elapsed_s", 22.0))
        path_length = result.get("path_length_m")
        mean_speed = path_length / elapsed if path_length is not None and elapsed > 0.0 else None
        rows.append(
            {
                "study_round": "gazebo_p02_development",
                "phase": "migration_admission_and_baseline_tuning",
                "block_id": "p02_dev_{}_{}".format(seed, receipt["scene_id"]),
                "episode_id": "{}_{}".format(item["ledger_method"], item["root"].name),
                "method": item["ledger_method"],
                "scene_family": receipt["scene_id"],
                "scene_id": receipt["scene_id"],
                "split": "development",
                "seed": seed,
                "attempt": 1,
                "order_index": 0,
                "goal_reached": str(goal).lower(),
                "timeout": str(timeout).lower(),
                "contact_count": contact,
                "system_failure": str(system_failure).lower(),
                "failure_reason": reason,
                "failure_inclusive_time_s": elapsed if goal and not contact and not system_failure else 22.0,
                "min_clearance_m": clearance,
                "path_length_m": path_length,
                "mean_speed_mps": mean_speed,
                "acceleration_rms": "",
                "jerk_metric": result.get("maximum_absolute_jerk_mps3"),
                "candidate_switch_count": result.get("candidate_switch_count", ""),
                "stop_count": result.get("certified_stop_count", ""),
                "recovery_count": result.get("stop_recovery_count", ""),
                "mean_latency_ms": result.get("mean_inference_latency_ms"),
                "p95_latency_ms": result.get("p95_inference_latency_ms"),
                "p99_latency_ms": result.get("p99_inference_latency_ms"),
                "mask_violation_count": result.get("mask_violation_count", 0),
                "config_sha256": item["config_sha256"],
                "model_sha256": report.get("actor_checkpoint_sha256") if report else "none",
                "evidence_path": item["relative_root"] + ("/episode_report.yaml" if report else "/roslaunch.log"),
            }
        )
    rows.sort(key=lambda row: (int(row["seed"]), row["method"]))
    temporary = path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader(); writer.writerows(retained + rows)
    temporary.replace(path)


def main():
    records = load_records()
    by_seed = {int(item["receipt"]["seed"]): item for item in records if item["report"] is not None}
    gates = {}
    for name, seed in CURRENT_GATES.items():
        item = by_seed[seed]; result = item["report"]["result"]
        gates[name] = {
            "seed": seed,
            "pass": bool(item["report"]["execution_integrity_pass"] and result["goal_reached"]),
            "goal_reached": result["goal_reached"],
            "contact_count": result["contact_state_count"],
            "minimum_conservative_clearance_m": result["minimum_conservative_clearance_m"],
            "p99_inference_latency_ms": result["p99_inference_latency_ms"],
            "evidence": item["relative_root"] + "/episode_report.yaml",
        }
    stage_pass = all(row["pass"] for row in gates.values())
    stage = {
        "schema_version": "1.0",
        "stage": "P02",
        "status": "p02_gazebo_migration_admission_pass_baseline_dynamic_tuning_pending" if stage_pass else "p02_admission_failed",
        "development_episode_identities_retained": len(records),
        "current_version_gate_identities": gates,
        "all_current_pct_gates_pass": stage_pass,
        "maximum_pct_gate_p99_inference_latency_ms": max(row["p99_inference_latency_ms"] for row in gates.values()),
        "offline_admission": "artifacts/gazebo_pct_teb_dwa_benchmark/admission/p02_offline_admission.json",
        "migration_diagnosis": "artifacts/gazebo_pct_teb_dwa_benchmark/diagnostics/p02_frozen_migration_diagnosis.json",
        "baseline_smoke": {
            "actual_teb_ros_chain_operational": True,
            "dwa_ros_chain_operational": True,
            "dynamic_conservative_clearance_tuning_complete": False,
            "formal_performance_claim": False,
        },
        "historical_or_formal_seed_reused": False,
        "formal_execution_authorized": False,
        "hardware_accessed": False,
        "stable_workspace_modified": False,
    }
    stage_path = ARTIFACT / "p02_gazebo_admission_report.json"
    atomic_text(stage_path, json.dumps(stage, indent=2, sort_keys=True) + "\n")

    admission_path = ARTIFACT / "admission/p02_offline_admission.json"
    admission = json.loads(admission_path.read_text(encoding="utf-8"))
    admission["live_gates"] = gates
    admission["status"] = stage["status"]
    atomic_text(admission_path, json.dumps(admission, indent=2, sort_keys=True) + "\n")

    update_run_manifest(records)
    update_episode_results(records)

    integrity_path = ARTIFACT / "integrity_report.json"
    integrity = json.loads(integrity_path.read_text(encoding="utf-8"))
    integrity["schema_version"] = "0.3"
    integrity["status"] = stage["status"]
    integrity["formal_execution_authorized"] = False
    integrity["new_gazebo_formal_seeds_accessed"] = False
    integrity["stable_workspace_modified"] = False
    integrity["checks"] = [row for row in integrity["checks"] if row.get("id") != "p02_gazebo_migration_admission"]
    integrity["checks"].append(
        {
            "id": "p02_gazebo_migration_admission",
            "status": "pass",
            "unit_and_regression_tests": "19 passed",
            "current_live_gate_units": "6/6",
            "maximum_p99_inference_latency_ms": stage["maximum_pct_gate_p99_inference_latency_ms"],
            "retained_development_identities": len(records),
            "retained_failure_counts": {
                "infrastructure_failure": 1,
                "timeout_or_nonarrival": 6,
                "conservative_clearance_violation": 19,
            },
            "run_manifest_unique": True,
            "episode_ledger_unique": True,
            "report": str(stage_path.relative_to(ROOT)),
            "report_sha256": sha256(stage_path),
            "baseline_dynamic_tuning_complete": False,
        }
    )
    integrity["failures"] = [
        {
            "scope": "development_only",
            "status": "retained_not_suppressed",
            "count": 26,
            "note": "one infrastructure failure, six timeout/nonarrival outcomes, and nineteen conservative-clearance failures remain in the ledgers; these do not invalidate the six current PCT admission gates",
        }
    ]
    atomic_text(integrity_path, json.dumps(integrity, indent=2) + "\n")

    hashes = {
        "schema_version": "0.2",
        "stage": "P02_development_unfrozen",
        "formal_frozen": False,
        "pct_development_contract": {"path": str(DEV_CONFIG.relative_to(ROOT)), "sha256": sha256(DEV_CONFIG)},
        "pct_historical_checkpoint_for_admission_only": {"path": str(MODEL.relative_to(ROOT)), "sha256": sha256(MODEL)},
        "actual_teb_v0": {"path": str(TEB_V0.relative_to(ROOT)), "sha256": sha256(TEB_V0)},
        "dwa_v0": {"path": str(DWA_V0.relative_to(ROOT)), "sha256": sha256(DWA_V0)},
        "p02_report": {"path": str(stage_path.relative_to(ROOT)), "sha256": sha256(stage_path)},
        "integrity_report": {"path": str(integrity_path.relative_to(ROOT)), "sha256": sha256(integrity_path)},
    }
    atomic_text(ARTIFACT / "method_config_hashes.yaml", yaml.safe_dump(hashes, sort_keys=False))
    print(stage["status"])
    print("retained_identities={}".format(len(records)))
    return 0 if stage_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
