#!/usr/bin/env python3
"""Run the frozen fresh-seed V5 fast formal matrix and assessment."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from v5_e2e_scope_rl.asymmetric_ppo import SensorActor
from v5_e2e_scope_rl.evaluation import run_episode
from v5_e2e_scope_rl.sensor_env import FAMILY_NAMES


METHODS = (
    "sensor_analytic_prior",
    "symmetric_recurrent_masked_ppo",
    "asymmetric_residual_masked_ppo",
    "full_advantage_gated_v5",
    "no_scan_history",
    "no_analytic_prior",
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_selected(selection_report):
    report = json.loads(selection_report.read_text(encoding="utf-8"))
    if not report.get("pass") or not report.get("fast_formal_seed_access_authorized"):
        raise RuntimeError("FAST_FORMAL_ENTRY_GATE_NOT_PASSED")
    grouped = {"asymmetric": [], "symmetric": [], "no_prior": []}
    for row in report["selected_checkpoints"]:
        path = Path(row["checkpoint_path"])
        if sha256(path) != row["checkpoint_sha256"]:
            raise RuntimeError("SELECTED_CHECKPOINT_HASH_MISMATCH:" + str(path))
        actor = SensorActor.from_actor_checkpoint(
            torch.load(path, map_location="cpu", weights_only=False)
        )
        actor.eval()
        grouped[row["variant"]].append((row, actor))
    if [len(grouped[key]) for key in ("asymmetric", "symmetric", "no_prior")] != [4, 2, 2]:
        raise RuntimeError("SELECTED_POLICY_ROSTER_MISMATCH")
    return report, grouped


def summarize(rows):
    return {
        "episodes": len(rows),
        "goal_count": int(sum(row["goal_reached"] for row in rows)),
        "contact_count": int(sum(row["contact_count"] for row in rows)),
        "mask_violation_count": int(sum(row["mask_violation_count"] for row in rows)),
        "mean_failure_inclusive_restricted_time_s": float(
            np.mean([row["failure_inclusive_restricted_time_s"] for row in rows])
        ),
        "minimum_physical_clearance_m": float(
            min(row["minimum_physical_clearance_m"] for row in rows)
        ),
        "mean_inference_latency_ms": float(
            np.mean([row["mean_inference_latency_ms"] for row in rows])
        ),
        "p99_episode_inference_latency_ms": float(
            np.percentile([row["p99_inference_latency_ms"] for row in rows], 99.0)
        ),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--selection-report", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output_directory.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(2)
    selection, grouped = load_selected(arguments.selection_report)
    rows = []
    for block_index, seed in enumerate(range(930100, 930160)):
        family = FAMILY_NAMES[block_index % len(FAMILY_NAMES)]
        for method in METHODS:
            actor = None
            gate_margin = None
            no_history = False
            training_identity = "none"
            checkpoint_sha = "none"
            if method == "symmetric_recurrent_masked_ppo":
                metadata, actor = grouped["symmetric"][block_index % 2]
            elif method in (
                "asymmetric_residual_masked_ppo",
                "full_advantage_gated_v5",
                "no_scan_history",
            ):
                metadata, actor = grouped["asymmetric"][block_index % 4]
                gate_margin = 0.10 if method in ("full_advantage_gated_v5", "no_scan_history") else None
                no_history = method == "no_scan_history"
            elif method == "no_analytic_prior":
                metadata, actor = grouped["no_prior"][block_index % 2]
            if actor is not None:
                training_identity = metadata["identity"]
                checkpoint_sha = metadata["checkpoint_sha256"]
            row = run_episode(
                seed=seed,
                family=family,
                method=method,
                actor=actor,
                advantage_gate_margin=gate_margin,
                no_scan_history=no_history,
            )
            row.update(
                {
                    "block_index": block_index,
                    "training_identity": training_identity,
                    "checkpoint_sha256": checkpoint_sha,
                }
            )
            rows.append(row)

    episodes_path = arguments.output_directory / "formal_episodes.csv"
    with episodes_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    by_method = {
        method: [row for row in rows if row["method"] == method] for method in METHODS
    }
    summaries = {method: summarize(values) for method, values in by_method.items()}
    baseline = np.asarray(
        [row["failure_inclusive_restricted_time_s"] for row in by_method["sensor_analytic_prior"]]
    )
    treatment = np.asarray(
        [row["failure_inclusive_restricted_time_s"] for row in by_method["full_advantage_gated_v5"]]
    )
    relative_reduction = float(1.0 - treatment.mean() / baseline.mean())
    random = np.random.default_rng(50321)
    bootstrap = np.empty(20000, dtype=np.float64)
    for index in range(len(bootstrap)):
        selected = random.integers(0, len(baseline), size=len(baseline))
        bootstrap[index] = 1.0 - treatment[selected].mean() / baseline[selected].mean()
    lower, upper = np.percentile(bootstrap, [2.5, 97.5])
    block_rows = []
    for block_index in range(60):
        row = {
            "block_index": block_index,
            "seed": 930100 + block_index,
            "family": FAMILY_NAMES[block_index % 4],
        }
        for method in METHODS:
            value = by_method[method][block_index]
            row[method + "_restricted_time_s"] = value[
                "failure_inclusive_restricted_time_s"
            ]
            row[method + "_goal"] = int(value["goal_reached"])
        row["full_vs_analytic_relative_reduction"] = (
            1.0
            - row["full_advantage_gated_v5_restricted_time_s"]
            / row["sensor_analytic_prior_restricted_time_s"]
        )
        block_rows.append(row)
    block_path = arguments.output_directory / "paired_blocks.csv"
    with block_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(block_rows[0]))
        writer.writeheader()
        writer.writerows(block_rows)
    gates = {
        "relative_reduction_minimum_0_08": relative_reduction >= 0.08,
        "bootstrap_lower_strictly_positive": float(lower) > 0.0,
        "goal_success_noninferior": summaries["full_advantage_gated_v5"]["goal_count"]
        >= summaries["sensor_analytic_prior"]["goal_count"],
        "zero_contact": summaries["full_advantage_gated_v5"]["contact_count"] == 0,
        "zero_mask_violation": summaries["full_advantage_gated_v5"]["mask_violation_count"]
        == 0,
    }
    report = {
        "status": (
            "v5_03_f1_fast_formal_positive"
            if all(gates.values())
            else "v5_03_f1_fast_formal_negative"
        ),
        "formal_result": True,
        "method_effectiveness_demonstrated_on_fast_population": all(gates.values()),
        "summaries": summaries,
        "primary_relative_time_reduction": relative_reduction,
        "paired_bootstrap_95_percent_interval": [float(lower), float(upper)],
        "positive_block_count": int(
            sum(row["full_vs_analytic_relative_reduction"] > 0.0 for row in block_rows)
        ),
        "zero_effect_block_count": int(
            sum(row["full_vs_analytic_relative_reduction"] == 0.0 for row in block_rows)
        ),
        "negative_block_count": int(
            sum(row["full_vs_analytic_relative_reduction"] < 0.0 for row in block_rows)
        ),
        "gates": gates,
        "gazebo_entry_authorized": all(gates.values()),
        "real_vehicle_authorized": False,
        "claim_boundary": "four synthetic sensor-level interaction families",
        "selection_report_sha256": sha256(arguments.selection_report),
    }
    report_path = arguments.output_directory / "formal_assessment.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (arguments.output_directory / "bootstrap_distribution.csv").write_text(
        "replicate,relative_reduction\n"
        + "".join(f"{index},{value:.12f}\n" for index, value in enumerate(bootstrap)),
        encoding="utf-8",
    )
    manifest = {
        path.name: sha256(path)
        for path in sorted(arguments.output_directory.iterdir())
        if path.is_file()
    }
    (arguments.output_directory / "sha256_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    raise SystemExit(0 if all(gates.values()) else 2)


if __name__ == "__main__":
    main()

