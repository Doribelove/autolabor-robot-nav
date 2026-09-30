#!/usr/bin/env python3
"""Fresh validation selection and entry gate for V6 PCT checkpoints."""

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


IDENTITIES = ("pct_seed9501", "pct_seed9502", "pct_seed9503", "pct_seed9504")


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def checkpoint_step(path):
    match = re.fullmatch(r"actor_step_(\d+)\.pt", path.name)
    if not match:
        raise ValueError("UNEXPECTED_CHECKPOINT_NAME")
    return int(match.group(1))


def write_csv(path, rows):
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--training-root", type=Path, required=True)
    parser.add_argument("--training-release", type=Path, required=True)
    parser.add_argument("--output-directory", type=Path, required=True)
    arguments = parser.parse_args()
    arguments.output_directory.mkdir(parents=True, exist_ok=False)
    torch.set_num_threads(2)
    release = json.loads(arguments.training_release.read_text(encoding="utf-8"))
    initialization_by_identity = {
        item["v6_identity"]: item for item in release["initializations"]
    }
    candidates = []
    selected = []
    selected_episodes = []
    analytic_episodes = []
    frozen_v5_episodes = []
    for identity_index, identity in enumerate(IDENTITIES):
        directory = arguments.training_root / identity
        training_summary = json.loads(
            (directory / "training_summary.json").read_text(encoding="utf-8")
        )
        if (
            training_summary["status"] != "training_complete"
            or training_summary["global_steps"] != 120000
            or training_summary["contact_count"] != 0
            or training_summary["mask_violation_count"] != 0
        ):
            raise RuntimeError("V6_TRAINING_IDENTITY_INVALID:" + identity)
        seed_start = 952100 + identity_index * 40
        seeds = range(seed_start, seed_start + 40)
        analytic_rows = evaluate_method(seeds, "sensor_analytic_prior")
        for row in analytic_rows:
            row["identity"] = identity
        analytic_episodes.extend(analytic_rows)
        initialization = initialization_by_identity[identity]
        initial_actor_path = Path(initialization["actor_path"])
        if sha256(initial_actor_path) != initialization["actor_sha256"]:
            raise RuntimeError("V5_INITIAL_ACTOR_HASH_MISMATCH")
        v5_actor = SensorActor.from_actor_checkpoint(
            torch.load(initial_actor_path, map_location="cpu", weights_only=False)
        )
        v5_actor.eval()
        v5_rows = evaluate_method(
            seeds,
            method="frozen_" + initialization["v5_identity"],
            actor=v5_actor,
            advantage_gate_margin=0.10,
        )
        for row in v5_rows:
            row["identity"] = identity
        frozen_v5_episodes.extend(v5_rows)
        identity_candidates = []
        checkpoint_paths = sorted(directory.glob("actor_step_*.pt"), key=checkpoint_step)
        if [checkpoint_step(path) for path in checkpoint_paths] != [40704, 80640, 120000]:
            raise RuntimeError("V6_CHECKPOINT_SCHEDULE_MISMATCH:" + identity)
        for checkpoint_path in checkpoint_paths:
            actor = SensorActor.from_actor_checkpoint(
                torch.load(checkpoint_path, map_location="cpu", weights_only=False)
            )
            actor.eval()
            rows = evaluate_method(
                seeds,
                method=identity,
                actor=actor,
                advantage_gate_margin=0.10,
            )
            summary = summarize_rows(rows)
            candidate = {
                "identity": identity,
                "training_seed": 9501 + identity_index,
                "checkpoint_step": checkpoint_step(checkpoint_path),
                "checkpoint_path": str(checkpoint_path),
                "checkpoint_sha256": sha256(checkpoint_path),
                **summary,
            }
            candidates.append(candidate)
            identity_candidates.append((candidate, rows))
        safe_candidates = [
            pair
            for pair in identity_candidates
            if pair[0]["contact_count"] == 0 and pair[0]["mask_violation_count"] == 0
        ]
        winner, winner_rows = min(
            safe_candidates,
            key=lambda pair: (
                -pair[0]["goal_count"],
                pair[0]["mean_failure_inclusive_restricted_time_s"],
                pair[0]["checkpoint_step"],
            ),
        )
        analytic_summary = summarize_rows(analytic_rows)
        v5_summary = summarize_rows(v5_rows)
        winner = dict(winner)
        winner.update(
            {
                "validation_seed_start": seed_start,
                "validation_blocks": 40,
                "analytic_goal_count": analytic_summary["goal_count"],
                "analytic_mean_time_s": analytic_summary[
                    "mean_failure_inclusive_restricted_time_s"
                ],
                "frozen_v5_goal_count": v5_summary["goal_count"],
                "frozen_v5_mean_time_s": v5_summary[
                    "mean_failure_inclusive_restricted_time_s"
                ],
                "relative_reduction_vs_analytic": 1.0
                - winner["mean_failure_inclusive_restricted_time_s"]
                / analytic_summary["mean_failure_inclusive_restricted_time_s"],
                "relative_reduction_vs_frozen_v5": 1.0
                - winner["mean_failure_inclusive_restricted_time_s"]
                / v5_summary["mean_failure_inclusive_restricted_time_s"],
            }
        )
        selected.append(winner)
        for row in winner_rows:
            row.update(
                {
                    "identity": identity,
                    "checkpoint_step": winner["checkpoint_step"],
                }
            )
        selected_episodes.extend(winner_rows)
    write_csv(arguments.output_directory / "checkpoint_candidates.csv", candidates)
    write_csv(arguments.output_directory / "selected_validation_episodes.csv", selected_episodes)
    write_csv(arguments.output_directory / "analytic_validation_episodes.csv", analytic_episodes)
    write_csv(arguments.output_directory / "frozen_v5_validation_episodes.csv", frozen_v5_episodes)
    pct_time = float(
        np.mean([row["mean_failure_inclusive_restricted_time_s"] for row in selected])
    )
    analytic_time = float(np.mean([row["analytic_mean_time_s"] for row in selected]))
    v5_time = float(np.mean([row["frozen_v5_mean_time_s"] for row in selected]))
    positive = sum(row["relative_reduction_vs_analytic"] > 0.0 for row in selected)
    gates = {
        "positive_seed_directions_minimum_3": positive >= 3,
        "aggregate_relative_reduction_minimum_0_08": 1.0 - pct_time / analytic_time
        >= 0.08,
        "goal_noninferior": sum(row["goal_count"] for row in selected)
        >= sum(row["analytic_goal_count"] for row in selected),
        "zero_contact": sum(row["contact_count"] for row in selected) == 0,
        "zero_mask_violation": sum(row["mask_violation_count"] for row in selected) == 0,
    }
    report = {
        "status": (
            "v6_01_r1_training_validation_gate_pass"
            if all(gates.values())
            else "v6_01_r1_training_validation_gate_fail"
        ),
        "selected_checkpoints": selected,
        "positive_seed_directions": positive,
        "pct_mean_time_s": pct_time,
        "analytic_mean_time_s": analytic_time,
        "frozen_v5_mean_time_s": v5_time,
        "relative_reduction_vs_analytic": float(1.0 - pct_time / analytic_time),
        "relative_reduction_vs_frozen_v5": float(1.0 - pct_time / v5_time),
        "gates": gates,
        "pass": all(gates.values()),
        "fresh_formal_seed_access_authorized": all(gates.values()),
        "gazebo_authorized": False,
        "training_release_sha256": sha256(arguments.training_release),
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

