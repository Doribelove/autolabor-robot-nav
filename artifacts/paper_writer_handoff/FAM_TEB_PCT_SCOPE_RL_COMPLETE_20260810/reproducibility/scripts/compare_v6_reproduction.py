#!/usr/bin/env python3
"""Compare the independent V6 replay with the frozen evidence and export ledger rows."""

import argparse
import csv
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ORIGINAL = ROOT / "artifacts/v6/formal/v6_02_f1_fresh_formal"
DEFAULT_REPRODUCTION = (
    ROOT
    / "artifacts/gazebo_pct_teb_dwa_benchmark/reproduction/v6_02_f1_recomputed_20260810"
)
DEFAULT_LEDGER = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark/episode_results.csv"
NONDETERMINISTIC_FIELDS = {
    "mean_inference_latency_ms",
    "p99_inference_latency_ms",
    "p99_episode_inference_latency_ms",
}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv(path):
    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def nested_without_latency(value):
    if isinstance(value, dict):
        return {
            key: nested_without_latency(item)
            for key, item in value.items()
            if key not in NONDETERMINISTIC_FIELDS
        }
    if isinstance(value, list):
        return [nested_without_latency(item) for item in value]
    return value


def ledger_rows(rows):
    method_order = {
        "sensor_analytic_prior": 0,
        "frozen_v5_full": 1,
        "pct_scope_rl_v6": 2,
        "pct_scope_rl_v6_no_history": 3,
    }
    output = []
    for row in rows:
        goal = row["goal_reached"] == "True"
        output.append(
            {
                "study_round": "historical_v6_02_f1_reproduction",
                "phase": "reproduction",
                "block_id": "v6f1_block_{:03d}".format(int(row["block_index"])),
                "episode_id": "v6f1_repro_{:03d}_{}".format(
                    int(row["block_index"]), row["method"]
                ),
                "method": row["method"],
                "scene_family": row["family"],
                "scene_id": row["family"],
                "split": "historical_formal_deterministic_reproduction",
                "seed": row["seed"],
                "attempt": 1,
                "order_index": method_order[row["method"]],
                "goal_reached": str(goal).lower(),
                "timeout": str(not goal).lower(),
                "contact_count": row["contact_count"],
                "system_failure": "false",
                "failure_reason": "goal" if goal else "not_reached",
                "failure_inclusive_time_s": row[
                    "failure_inclusive_restricted_time_s"
                ],
                "min_clearance_m": row["minimum_physical_clearance_m"],
                "path_length_m": "",
                "mean_speed_mps": "",
                "acceleration_rms": "",
                "jerk_metric": "",
                "candidate_switch_count": "",
                "stop_count": "",
                "recovery_count": row["activation_revalidation_failure_count"],
                "mean_latency_ms": row["mean_inference_latency_ms"],
                "p95_latency_ms": "",
                "p99_latency_ms": row["p99_inference_latency_ms"],
                "mask_violation_count": row["mask_violation_count"],
                "config_sha256": "47fd816ab1e1225a7c941c1d6f562e0024beee410a51a80c7ce6bccfca6febf0",
                "model_sha256": row["actor_sha256"],
                "evidence_path": str(
                    DEFAULT_REPRODUCTION.relative_to(ROOT) / "formal_episodes.csv"
                ),
            }
        )
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--original", type=Path, default=DEFAULT_ORIGINAL)
    parser.add_argument("--reproduction", type=Path, default=DEFAULT_REPRODUCTION)
    parser.add_argument("--episode-ledger", type=Path, default=DEFAULT_LEDGER)
    arguments = parser.parse_args()
    original_rows = read_csv(arguments.original / "formal_episodes.csv")
    reproduced_rows = read_csv(arguments.reproduction / "formal_episodes.csv")
    if len(original_rows) != len(reproduced_rows):
        raise RuntimeError("V6_REPRODUCTION_ROW_COUNT_MISMATCH")
    fields = list(original_rows[0])
    if fields != list(reproduced_rows[0]):
        raise RuntimeError("V6_REPRODUCTION_SCHEMA_MISMATCH")
    differences = {field: 0 for field in fields}
    for original, reproduced in zip(original_rows, reproduced_rows):
        for field in fields:
            differences[field] += int(original[field] != reproduced[field])
    scientific_fields = [field for field in fields if field not in NONDETERMINISTIC_FIELDS]
    scientific_exact = all(differences[field] == 0 for field in scientific_fields)
    original_report = json.loads(
        (arguments.original / "formal_assessment.json").read_text(encoding="utf-8")
    )
    reproduced_report = json.loads(
        (arguments.reproduction / "formal_assessment.json").read_text(encoding="utf-8")
    )
    report_without_latency_exact = nested_without_latency(original_report) == nested_without_latency(
        reproduced_report
    )
    bootstrap_exact = (
        sha256(arguments.original / "bootstrap_distributions.csv")
        == sha256(arguments.reproduction / "bootstrap_distributions.csv")
    )
    comparison = {
        "status": (
            "v6_02_f1_scientific_reproduction_pass"
            if scientific_exact and report_without_latency_exact and bootstrap_exact
            else "v6_02_f1_scientific_reproduction_fail"
        ),
        "episode_count": len(original_rows),
        "scientific_episode_fields_exact": scientific_exact,
        "assessment_excluding_wall_clock_latency_exact": report_without_latency_exact,
        "bootstrap_distribution_byte_exact": bootstrap_exact,
        "field_difference_counts": {
            field: count for field, count in differences.items() if count
        },
        "allowed_nondeterminism": sorted(NONDETERMINISTIC_FIELDS),
        "original_sha256": {
            name: sha256(arguments.original / name)
            for name in (
                "formal_episodes.csv",
                "family_summary.csv",
                "formal_assessment.json",
                "bootstrap_distributions.csv",
            )
        },
        "reproduction_sha256": {
            name: sha256(arguments.reproduction / name)
            for name in (
                "formal_episodes.csv",
                "family_summary.csv",
                "formal_assessment.json",
                "bootstrap_distributions.csv",
            )
        },
        "formal_statistics": {
            "primary_relative_reduction_vs_analytic": reproduced_report[
                "primary_relative_reduction_vs_analytic"
            ],
            "primary_two_sided_97_5_percent_interval": reproduced_report[
                "primary_two_sided_97_5_percent_interval"
            ],
            "secondary_relative_reduction_vs_frozen_v5": reproduced_report[
                "secondary_relative_reduction_vs_frozen_v5"
            ],
            "secondary_two_sided_95_percent_interval": reproduced_report[
                "secondary_two_sided_95_percent_interval"
            ],
            "summaries": nested_without_latency(reproduced_report["summaries"]),
            "gates": reproduced_report["gates"],
        },
        "scope": "historical four-family sensor-level fast population only",
        "gazebo_or_teb_dwa_claim_created": False,
    }
    comparison_path = arguments.reproduction / "reproduction_comparison.json"
    comparison_path.write_text(
        json.dumps(comparison, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    rows = ledger_rows(reproduced_rows)
    with arguments.episode_ledger.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(comparison, indent=2, sort_keys=True))
    raise SystemExit(0 if comparison["status"].endswith("pass") else 2)


if __name__ == "__main__":
    main()
