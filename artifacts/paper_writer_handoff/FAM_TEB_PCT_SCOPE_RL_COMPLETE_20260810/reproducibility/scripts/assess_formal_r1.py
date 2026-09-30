#!/usr/bin/env python3
"""Deterministically assess the locked 160-block formal-round-1 matrix."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
FORMAL = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark/formal/round_1"
SCHEDULE = FORMAL / "formal_r1_schedule.csv"
PREREG = FORMAL / "formal_r1_preregistration.yaml"
RELEASE = FORMAL / "formal_r1_execution_release.yaml"
EXECUTION = FORMAL / "execution/episodes"
REPORT = FORMAL / "formal_r1_assessment.json"
EPISODES = FORMAL / "formal_r1_episode_results.csv"
METHODS = ("pct_scope_rl", "analytic_prior", "actual_teb", "dwa")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def failure_reason(report):
    result = report["result"]
    if int(result.get("contact_state_count", 0)) > 0:
        return "contact"
    if float(result["minimum_conservative_clearance_m"]) < 0.0:
        return "conservative_clearance_violation"
    if not bool(report["execution_integrity_pass"]):
        return "execution_integrity_failure"
    if bool(result["goal_reached"]):
        return "goal"
    return "timeout_or_nonarrival"


def load():
    schedule = list(csv.DictReader(SCHEDULE.open(encoding="utf-8")))
    prereg = yaml.safe_load(PREREG.read_text(encoding="utf-8"))
    release_sha = sha256(RELEASE)
    rows = []
    for item in schedule:
        root = EXECUTION / "{}_{}_{}".format(item["seed"], item["scene_id"], item["method"])
        report_path = root / "episode_report.yaml"
        receipt_path = root / "receipt.json"
        if not report_path.is_file() or not receipt_path.is_file():
            raise RuntimeError("FORMAL_R1_INCOMPLETE_IDENTITY:" + root.name)
        report = yaml.safe_load(report_path.read_text(encoding="utf-8"))
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        if (
            receipt.get("release_sha256") != release_sha
            or receipt.get("episode_report_sha256") != sha256(report_path)
            or report.get("stage") != "PCT-BENCH-FORMAL-R1"
            or report.get("split") != "formal_round_1"
            or int(item["attempt"]) != 1
        ):
            raise RuntimeError("FORMAL_R1_IDENTITY_INTEGRITY_FAILURE:" + root.name)
        reason = failure_reason(report)
        result = report["result"]
        rows.append(
            {
                "block_id": item["block_id"],
                "seed": int(item["seed"]),
                "scene_id": item["scene_id"],
                "method": item["method"],
                "attempt": 1,
                "order_index": int(item["order_index"]),
                "goal_reached": bool(result["goal_reached"]),
                "execution_integrity_pass": bool(report["execution_integrity_pass"]),
                "valid_goal": reason == "goal",
                "failure_reason": reason,
                "failure_inclusive_time_s": float(result["restricted_elapsed_s"]) if reason == "goal" else 22.0,
                "minimum_conservative_clearance_m": float(result["minimum_conservative_clearance_m"]),
                "contact_count": int(result.get("contact_state_count", 0)),
                "mask_violation_count": int(result.get("mask_violation_count", 0)),
                "path_length_m": result.get("path_length_m"),
                "candidate_switch_count": result.get("candidate_switch_count"),
                "certified_stop_count": result.get("certified_stop_count"),
                "stop_recovery_count": result.get("stop_recovery_count"),
                "mean_inference_latency_ms": result.get("mean_inference_latency_ms"),
                "p95_inference_latency_ms": result.get("p95_inference_latency_ms"),
                "p99_inference_latency_ms": result.get("p99_inference_latency_ms"),
                "report_path": str(report_path.relative_to(ROOT)),
                "report_sha256": sha256(report_path),
                "receipt_path": str(receipt_path.relative_to(ROOT)),
                "receipt_sha256": sha256(receipt_path),
            }
        )
    identities = {(row["seed"], row["scene_id"], row["method"], row["attempt"]) for row in rows}
    if len(schedule) != 640 or len(rows) != 640 or len(identities) != 640:
        raise RuntimeError("FORMAL_R1_MATRIX_COMPLETENESS_FAILURE")
    return prereg, rows


def bootstrap(pct, baseline, seed, samples):
    rng = np.random.default_rng(seed)
    indices = rng.integers(0, len(pct), size=(samples, len(pct)))
    pct_means = pct[indices].mean(axis=1)
    baseline_means = baseline[indices].mean(axis=1)
    effect = (baseline_means - pct_means) / baseline_means
    return {
        "samples": samples,
        "seed": seed,
        "lower_97_5_percent": float(np.quantile(effect, 0.0125)),
        "upper_97_5_percent": float(np.quantile(effect, 0.9875)),
    }


def summary(rows, method):
    subset = [row for row in rows if row["method"] == method]
    return {
        "episodes": len(subset),
        "valid_goals": sum(row["valid_goal"] for row in subset),
        "completion_rate": float(np.mean([row["valid_goal"] for row in subset])),
        "mean_failure_inclusive_time_s": float(np.mean([row["failure_inclusive_time_s"] for row in subset])),
        "minimum_conservative_clearance_m": min(row["minimum_conservative_clearance_m"] for row in subset),
        "contact_count": sum(row["contact_count"] for row in subset),
        "mask_violation_count": sum(row["mask_violation_count"] for row in subset),
        "failure_counts": {
            reason: sum(row["failure_reason"] == reason for row in subset)
            for reason in sorted({row["failure_reason"] for row in subset})
        },
    }


def compare(rows, prereg, baseline_name):
    blocks = sorted({row["block_id"] for row in rows})
    lookup = {(row["block_id"], row["method"]): row for row in rows}
    pct = np.asarray([lookup[(block, "pct_scope_rl")]["failure_inclusive_time_s"] for block in blocks])
    baseline = np.asarray([lookup[(block, baseline_name)]["failure_inclusive_time_s"] for block in blocks])
    point = float((baseline.mean() - pct.mean()) / baseline.mean())
    spec = prereg["multiplicity"]
    interval = bootstrap(
        pct,
        baseline,
        int(spec["bootstrap_seeds"][baseline_name]),
        int(spec["bootstrap_samples"]),
    )
    pct_rows = [lookup[(block, "pct_scope_rl")] for block in blocks]
    baseline_rows = [lookup[(block, baseline_name)] for block in blocks]
    completion_difference_pp = float(
        100.0
        * (
            np.mean([row["valid_goal"] for row in pct_rows])
            - np.mean([row["valid_goal"] for row in baseline_rows])
        )
    )
    gates = prereg["positive_gate_per_comparator"]
    checks = {
        "point_reduction_at_least_8_percent": bool(point >= float(gates["minimum_relative_time_reduction"])),
        "bootstrap_lower_strictly_positive": bool(interval["lower_97_5_percent"] > 0.0),
        "completion_difference_at_least_minus_5_pp": bool(completion_difference_pp >= float(gates["pct_completion_rate_difference_percentage_points_minimum"])),
        "pct_contact_not_higher": bool(sum(row["contact_count"] for row in pct_rows) <= sum(row["contact_count"] for row in baseline_rows)),
        "pct_mask_violation_zero": bool(sum(row["mask_violation_count"] for row in pct_rows) <= int(gates["pct_mask_violation_count_maximum"])),
        "finite_statistics": bool(np.isfinite(point) and all(np.isfinite(list(interval.values())))),
    }
    return {
        "baseline": baseline_name,
        "relative_reduction": point,
        "mean_paired_difference_s": float(np.mean(baseline - pct)),
        "bootstrap": interval,
        "completion_rate_difference_percentage_points": completion_difference_pp,
        "pct_contact_count": sum(row["contact_count"] for row in pct_rows),
        "baseline_contact_count": sum(row["contact_count"] for row in baseline_rows),
        "pct_mask_violation_count": sum(row["mask_violation_count"] for row in pct_rows),
        "gate_checks": checks,
        "positive_gate_pass": all(checks.values()),
    }


def strata(rows):
    result = {}
    for scene in sorted({row["scene_id"] for row in rows}):
        result[scene] = {}
        for method in METHODS:
            subset = [row for row in rows if row["scene_id"] == scene and row["method"] == method]
            result[scene][method] = {
                "episodes": len(subset),
                "valid_goals": sum(row["valid_goal"] for row in subset),
                "mean_failure_inclusive_time_s": float(np.mean([row["failure_inclusive_time_s"] for row in subset])),
                "minimum_clearance_m": min(row["minimum_conservative_clearance_m"] for row in subset),
            }
    return result


def write_episode_csv(rows):
    fields = [key for key in rows[0] if not key.endswith("_path") or key in ("report_path", "receipt_path")]
    temporary = EPISODES.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: row[key] for key in fields} for row in rows)
    temporary.replace(EPISODES)


def main():
    if REPORT.exists() or EPISODES.exists():
        raise FileExistsError("FORMAL_R1_ASSESSMENT_ALREADY_EXISTS")
    prereg, rows = load()
    summaries = {method: summary(rows, method) for method in METHODS}
    comparisons = {
        name: compare(rows, prereg, name) for name in ("actual_teb", "dwa")
    }
    write_episode_csv(rows)
    positive = [name for name, value in comparisons.items() if value["positive_gate_pass"]]
    report = {
        "schema_version": "1.0",
        "stage": "P08_FORMAL_R1_ASSESSMENT",
        "status": "formal_round_1_complete",
        "formal_result": True,
        "performance_improvement_demonstrated": bool(positive),
        "positive_comparators": positive,
        "blocks": 160,
        "episodes": 640,
        "attempt_one_complete": True,
        "release_sha256": sha256(RELEASE),
        "preregistration_sha256": sha256(PREREG),
        "schedule_sha256": sha256(SCHEDULE),
        "episode_results_path": str(EPISODES.relative_to(ROOT)),
        "episode_results_sha256": sha256(EPISODES),
        "method_summaries": summaries,
        "primary_comparisons": comparisons,
        "scene_strata": strata(rows),
        "all_failures_retained": True,
        "same_identity_retry_count": 0,
        "replacement_seed_count": 0,
        "formal_round_2_seed_accessed": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
        "claim_boundary": "fresh single-robot Gazebo population over the six frozen core templates; no perception/OOD/deployment/real-vehicle claim",
        "per_identity_evidence": rows,
    }
    REPORT.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key not in ("per_identity_evidence", "scene_strata")}, indent=2))


if __name__ == "__main__":
    main()
