#!/usr/bin/env python3
"""Select V5 checkpoints using only the frozen validation population."""

import argparse
import csv
import hashlib
import json
import re
from pathlib import Path

import numpy as np
import torch

from v5_e2e_scope_rl.asymmetric_ppo import SensorActor
from v5_e2e_scope_rl.evaluation import evaluate_method, summarize_rows


SCHEDULE = (
    ("asym_seed9101", "asymmetric", 9101, 300000),
    ("asym_seed9102", "asymmetric", 9102, 300000),
    ("asym_seed9103", "asymmetric", 9103, 300000),
    ("asym_seed9104", "asymmetric", 9104, 300000),
    ("symmetric_seed9101", "symmetric", 9101, 150000),
    ("symmetric_seed9102", "symmetric", 9102, 150000),
    ("no_prior_seed9103", "no_prior", 9103, 150000),
    ("no_prior_seed9104", "no_prior", 9104, 150000),
)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def checkpoint_step(path):
    match = re.fullmatch(r"actor_step_(\d+)\.pt", path.name)
    if not match:
        raise ValueError("UNEXPECTED_CHECKPOINT_NAME:" + path.name)
    return int(match.group(1))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output_directory.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(2)
    candidate_rows = []
    selected_rows = []
    selected_episode_rows = []
    analytic_episode_rows = []

    for schedule_index, (identity, variant, training_seed, expected_steps) in enumerate(SCHEDULE):
        identity_directory = arguments.training_root / identity
        summary = json.loads(
            (identity_directory / "training_summary.json").read_text(encoding="utf-8")
        )
        if summary["status"] != "training_complete" or summary["global_steps"] != expected_steps:
            raise RuntimeError("TRAINING_IDENTITY_INCOMPLETE:" + identity)
        if summary["contact_count"] or summary["mask_violation_count"]:
            raise RuntimeError("TRAINING_SAFETY_INVARIANT_FAILED:" + identity)
        seeds = range(920100 + schedule_index * 40, 920100 + (schedule_index + 1) * 40)
        analytic_rows = evaluate_method(seeds, "sensor_analytic_prior")
        for row in analytic_rows:
            row.update({"identity": identity, "training_seed": training_seed})
        analytic_episode_rows.extend(analytic_rows)
        checkpoints = sorted(identity_directory.glob("actor_step_*.pt"), key=checkpoint_step)
        if not checkpoints or checkpoint_step(checkpoints[-1]) != expected_steps:
            raise RuntimeError("FINAL_INTERVAL_CHECKPOINT_MISSING:" + identity)
        identity_candidates = []
        for checkpoint in checkpoints:
            actor = SensorActor.from_actor_checkpoint(
                torch.load(checkpoint, map_location="cpu", weights_only=False)
            )
            actor.eval()
            gate_margin = 0.10 if variant == "asymmetric" else None
            rows = evaluate_method(
                seeds,
                method=identity,
                actor=actor,
                advantage_gate_margin=gate_margin,
            )
            summary_row = summarize_rows(rows)
            candidate = {
                "identity": identity,
                "variant": variant,
                "training_seed": training_seed,
                "checkpoint_step": checkpoint_step(checkpoint),
                "checkpoint_path": str(checkpoint),
                "checkpoint_sha256": sha256(checkpoint),
                **summary_row,
            }
            candidate_rows.append(candidate)
            identity_candidates.append((candidate, rows))
        safe_candidates = [
            item
            for item in identity_candidates
            if item[0]["contact_count"] == 0 and item[0]["mask_violation_count"] == 0
        ]
        if not safe_candidates:
            raise RuntimeError("NO_SAFE_VALIDATION_CHECKPOINT:" + identity)
        selected, rows = min(
            safe_candidates,
            key=lambda item: (
                -item[0]["goal_count"],
                item[0]["mean_failure_inclusive_restricted_time_s"],
                item[0]["checkpoint_step"],
            ),
        )
        analytic_summary = summarize_rows(analytic_rows)
        selected = dict(selected)
        selected.update(
            {
                "validation_seed_start": 920100 + schedule_index * 40,
                "validation_blocks": 40,
                "analytic_goal_count": analytic_summary["goal_count"],
                "analytic_mean_failure_inclusive_restricted_time_s": analytic_summary[
                    "mean_failure_inclusive_restricted_time_s"
                ],
                "relative_time_reduction_vs_analytic": 1.0
                - selected["mean_failure_inclusive_restricted_time_s"]
                / analytic_summary["mean_failure_inclusive_restricted_time_s"],
            }
        )
        selected_rows.append(selected)
        for row in rows:
            row.update(
                {
                    "identity": identity,
                    "variant": variant,
                    "training_seed": training_seed,
                    "checkpoint_step": selected["checkpoint_step"],
                }
            )
        selected_episode_rows.extend(rows)

    def write_csv(path, rows):
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    write_csv(arguments.output_directory / "checkpoint_candidates.csv", candidate_rows)
    write_csv(arguments.output_directory / "selected_validation_episodes.csv", selected_episode_rows)
    write_csv(arguments.output_directory / "analytic_validation_episodes.csv", analytic_episode_rows)
    asymmetric = [row for row in selected_rows if row["variant"] == "asymmetric"]
    asymmetric_time = float(
        np.mean([row["mean_failure_inclusive_restricted_time_s"] for row in asymmetric])
    )
    analytic_time = float(
        np.mean(
            [row["analytic_mean_failure_inclusive_restricted_time_s"] for row in asymmetric]
        )
    )
    positive_directions = sum(
        row["relative_time_reduction_vs_analytic"] > 0.0 for row in asymmetric
    )
    gates = {
        "asymmetric_seed_positive_directions_minimum_2": positive_directions >= 2,
        "asymmetric_goal_noninferior": sum(row["goal_count"] for row in asymmetric)
        >= sum(row["analytic_goal_count"] for row in asymmetric),
        "zero_contact": sum(row["contact_count"] for row in asymmetric) == 0,
        "zero_mask_violation": sum(row["mask_violation_count"] for row in asymmetric) == 0,
        "relative_time_reduction_minimum_0_05": 1.0 - asymmetric_time / analytic_time
        >= 0.05,
    }
    report = {
        "status": (
            "v5_02_r1_training_validation_gate_pass"
            if all(gates.values())
            else "v5_02_r1_training_validation_gate_fail"
        ),
        "selected_checkpoints": selected_rows,
        "asymmetric_positive_seed_directions": int(positive_directions),
        "asymmetric_mean_failure_inclusive_restricted_time_s": asymmetric_time,
        "analytic_mean_failure_inclusive_restricted_time_s": analytic_time,
        "asymmetric_relative_time_reduction": float(1.0 - asymmetric_time / analytic_time),
        "gates": gates,
        "pass": all(gates.values()),
        "fast_formal_seed_access_authorized": all(gates.values()),
        "gazebo_authorized": False,
        "real_vehicle_authorized": False,
    }
    report_path = arguments.output_directory / "selection_report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    manifest = {
        path.name: sha256(path)
        for path in sorted(arguments.output_directory.iterdir())
        if path.is_file()
    }
    (arguments.output_directory / "sha256_manifest.json").write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    raise SystemExit(0 if report["pass"] else 2)


if __name__ == "__main__":
    main()

