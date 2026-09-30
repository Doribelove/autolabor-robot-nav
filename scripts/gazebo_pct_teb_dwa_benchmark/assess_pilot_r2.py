#!/usr/bin/env python3
"""Hash-checked, preregistered R2 pilot assessment and power analysis."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import statistics

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
PILOT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark/pilot/pilot_r2"
SCHEDULE = PILOT / "pilot_r2_schedule.csv"
RELEASE = PILOT / "pilot_r2_execution_release.yaml"
EXECUTION = PILOT / "execution/episodes"
METHODS = ("pct_scope_rl", "analytic_prior", "actual_teb", "dwa")


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def atomic_text(path, content):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    temporary.replace(path)


def load(expected_release_sha256):
    if sha256(RELEASE) != expected_release_sha256:
        raise RuntimeError("PILOT_R2_RELEASE_HASH_MISMATCH")
    release = yaml.safe_load(RELEASE.read_text(encoding="utf-8"))
    if release.get("stage") != "pilot" or release.get("execution_authorized") is not True:
        raise RuntimeError("PILOT_R2_RELEASE_AUTHORITY_INVALID")
    schedule = list(csv.DictReader(SCHEDULE.open(encoding="utf-8")))
    rows = []
    for item in schedule:
        root = EXECUTION / "{}_{}_{}".format(item["seed"], item["scene_id"], item["method"])
        report_path = root / "episode_report.yaml"; receipt_path = root / "receipt.json"
        if not report_path.is_file() or not receipt_path.is_file():
            raise RuntimeError("PILOT_R2_IDENTITY_EVIDENCE_ABSENT:" + root.name)
        report = yaml.load(report_path.read_text(encoding="utf-8"), Loader=yaml.CSafeLoader)
        receipt = json.loads(receipt_path.read_text(encoding="utf-8"))
        result = report["result"]
        valid = bool(
            report["execution_integrity_pass"] and result["goal_reached"]
            and int(result.get("contact_state_count", 0)) == 0
            and float(result["minimum_conservative_clearance_m"]) >= 0.0
        )
        failure = "goal" if valid else (
            "contact" if int(result.get("contact_state_count", 0)) > 0
            else "conservative_clearance_violation" if float(result["minimum_conservative_clearance_m"]) < 0.0
            else "execution_integrity_failure" if not report["execution_integrity_pass"]
            else "timeout_or_nonarrival"
        )
        if receipt["release_sha256"] != expected_release_sha256 or receipt["episode_report_sha256"] != sha256(report_path):
            raise RuntimeError("PILOT_R2_RECEIPT_BINDING_MISMATCH")
        if report.get("stage") != "PCT-BENCH-PILOT-R2" or report.get("candidate_contract_variant") != "r2_candidate_v060":
            raise RuntimeError("PILOT_R2_REPORT_CONTRACT_MISMATCH")
        rows.append({**item, "root": root, "report_path": report_path, "receipt_path": receipt_path,
                     "report": report, "receipt": receipt, "valid": valid, "failure_reason": failure,
                     "failure_inclusive_time_s": float(result["restricted_elapsed_s"]) if valid else 22.0})
    ids = [(row["seed"], row["scene_id"], row["method"], row["attempt"]) for row in rows]
    if len(rows) != 80 or len(ids) != len(set(ids)):
        raise RuntimeError("PILOT_R2_MATRIX_INCOMPLETE_OR_DUPLICATE")
    return rows


def method_summary(rows, method):
    subset = [row for row in rows if row["method"] == method]
    results = [row["report"]["result"] for row in subset]
    return {
        "episodes": len(subset), "valid_goals": sum(row["valid"] for row in subset),
        "mean_failure_inclusive_time_s": float(np.mean([row["failure_inclusive_time_s"] for row in subset])),
        "minimum_conservative_clearance_m": min(float(item["minimum_conservative_clearance_m"]) for item in results),
        "contact_count": sum(int(item.get("contact_state_count", 0)) for item in results),
        "mask_violation_count": sum(int(item.get("mask_violation_count", 0)) for item in results),
        "failure_counts": {reason: sum(row["failure_reason"] == reason for row in subset)
                           for reason in sorted({row["failure_reason"] for row in subset})},
    }


def comparison(pct, baseline, seed):
    rng = np.random.default_rng(seed)
    indices = rng.integers(0, len(pct), size=(100000, len(pct)))
    effects = (np.mean(baseline[indices], axis=1) - np.mean(pct[indices], axis=1)) / np.mean(baseline[indices], axis=1)
    difference = baseline - pct
    sd = float(np.std(difference, ddof=1)); target = 0.08 * float(np.mean(baseline))
    normal = statistics.NormalDist(); critical = normal.inv_cdf(0.9875); desired = normal.inv_cdf(0.80)
    required = ((critical + desired) * sd / target) ** 2 if sd > 0 else 0.0
    return {
        "relative_reduction": float((np.mean(baseline) - np.mean(pct)) / np.mean(baseline)),
        "bootstrap": {"samples": 100000, "seed": seed,
                      "lower_97_5_percent": float(np.quantile(effects, 0.0125)),
                      "upper_97_5_percent": float(np.quantile(effects, 0.9875))},
        "power_analysis": {"paired_difference_sd_s": sd, "target_8_percent_difference_s": target,
                           "required_blocks_unrounded": required, "required_blocks_ceiling": int(np.ceil(required))},
    }


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--expected-release-sha256", required=True)
    args = parser.parse_args(); rows = load(args.expected_release_sha256)
    blocks = sorted({row["block_id"] for row in rows})
    values = {(row["block_id"], row["method"]): row["failure_inclusive_time_s"] for row in rows}
    pct = np.asarray([values[(block, "pct_scope_rl")] for block in blocks])
    comparisons = {
        name: comparison(pct, np.asarray([values[(block, name)] for block in blocks]), 20261981 + index)
        for index, name in enumerate(("actual_teb", "dwa"))
    }
    required = max(item["power_analysis"]["required_blocks_ceiling"] for item in comparisons.values())
    selected = min(200, max(160, int(np.ceil(required / 20.0) * 20)))
    report = {
        "schema_version": "1.0", "stage": "P10_PILOT_R2", "status": "pilot_r2_complete",
        "release_sha256": args.expected_release_sha256, "schedule_sha256": sha256(SCHEDULE),
        "blocks": 20, "episodes": 80, "attempt_one_complete": True,
        "same_identity_retry_count": 0, "method_summaries": {m: method_summary(rows, m) for m in METHODS},
        "paired_comparisons": comparisons, "formal_round_2_selected_blocks": selected,
        "selection_rule": "ceil maximum two-comparison 80% power requirement to 20-block increment, floor 160, cap 200",
        "pilot_tuning_performed": False, "formal_round_2_seed_accessed": False,
        "per_identity_evidence": [{"block_id": row["block_id"], "seed": int(row["seed"]),
            "scene_id": row["scene_id"], "method": row["method"], "valid": row["valid"],
            "failure_reason": row["failure_reason"], "failure_inclusive_time_s": row["failure_inclusive_time_s"],
            "report": str(row["report_path"].relative_to(ROOT)), "report_sha256": sha256(row["report_path"]),
            "receipt": str(row["receipt_path"].relative_to(ROOT)), "receipt_sha256": sha256(row["receipt_path"])} for row in rows],
    }
    atomic_text(PILOT / "pilot_r2_assessment.json", json.dumps(report, indent=2, sort_keys=True) + "\n")
    fields = ["block_id", "seed", "scene_id", "method", "attempt", "order_index", "valid", "failure_reason", "failure_inclusive_time_s", "report_path", "receipt_path"]
    target = PILOT / "pilot_r2_episode_results.csv"; temporary = target.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
        for row in rows:
            writer.writerow({field: ({"report_path": str(row["report_path"].relative_to(ROOT)), "receipt_path": str(row["receipt_path"].relative_to(ROOT))}.get(field, row.get(field, ""))) for field in fields})
    temporary.replace(target)
    print(json.dumps({"assessment_sha256": sha256(PILOT / "pilot_r2_assessment.json"), "selected_blocks": selected}, indent=2))


if __name__ == "__main__":
    main()
