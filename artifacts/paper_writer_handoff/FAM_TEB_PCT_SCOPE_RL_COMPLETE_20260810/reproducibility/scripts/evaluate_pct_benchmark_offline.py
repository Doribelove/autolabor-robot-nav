#!/usr/bin/env python3
"""Failure-inclusive batched offline validation for benchmark PCT actors."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/application/v5_e2e_scope_rl/src"))

from v5_e2e_scope_rl.asymmetric_ppo import (  # noqa: E402
    SensorActor,
    observations_to_torch,
    stack_observations,
)
from v5_e2e_scope_rl.benchmark_runtime import BenchmarkSensorCandidateEnv  # noqa: E402
from v5_e2e_scope_rl.sensor_env import FAMILY_NAMES  # noqa: E402


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def actor_spec(value):
    if "=" not in value:
        raise argparse.ArgumentTypeError("actor must be LABEL=PATH")
    label, raw_path = value.split("=", 1)
    if not label:
        raise argparse.ArgumentTypeError("actor label is empty")
    return label, Path(raw_path)


def evaluate(label, actor, seed_start, seeds_per_family):
    environments = []
    rows = []
    for family_index, family in enumerate(FAMILY_NAMES):
        for offset in range(seeds_per_family):
            seed = seed_start + family_index * seeds_per_family + offset
            environment = BenchmarkSensorCandidateEnv({"family": family})
            observation, _ = environment.reset(seed=seed)
            environments.append(environment)
            rows.append(
                {
                    "method": label,
                    "family": family,
                    "seed": seed,
                    "observation": observation,
                    "intervention_count": 0,
                    "learned_difference_count": 0,
                }
            )
    active = list(range(len(rows)))
    while active:
        if actor is None:
            actions = [environments[index].analytic_action() for index in active]
            metadata = None
        else:
            batch = stack_observations([rows[index]["observation"] for index in active])
            with torch.no_grad():
                action_tensor, metadata = actor.choose_actions(
                    observations_to_torch(batch, torch.device("cpu")),
                    deterministic=True,
                    advantage_gate_margin=0.10,
                )
            actions = action_tensor.cpu().numpy().astype(int).tolist()
        next_active = []
        for batch_index, (index, action) in enumerate(zip(active, actions)):
            if metadata is not None:
                rows[index]["intervention_count"] += int(
                    metadata["intervened"][batch_index].cpu()
                )
                rows[index]["learned_difference_count"] += int(
                    metadata["learned_action"][batch_index].cpu()
                    != metadata["analytic_action"][batch_index].cpu()
                )
            observation, _, terminated, truncated, info = environments[index].step(action)
            rows[index]["observation"] = observation
            if terminated or truncated:
                rows[index].update(
                    {
                        "goal_reached": bool(info["goal_reached"]),
                        "contact_count": int(info["contact_count"]),
                        "mask_violation_count": int(info["mask_violation_count"]),
                        "restricted_elapsed_s": float(info["restricted_elapsed_s"]),
                        "failure_inclusive_restricted_time_s": float(
                            info["restricted_elapsed_s"] if info["goal_reached"] else 22.0
                        ),
                        "minimum_physical_clearance_m": float(
                            info["minimum_physical_clearance_m"]
                        ),
                    }
                )
                rows[index].pop("observation")
            else:
                next_active.append(index)
        active = next_active
    return rows


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
        "intervention_count": int(sum(row["intervention_count"] for row in rows)),
        "by_family": {
            family: {
                "episodes": len(subset),
                "goal_count": int(sum(row["goal_reached"] for row in subset)),
                "mean_failure_inclusive_restricted_time_s": float(
                    np.mean([row["failure_inclusive_restricted_time_s"] for row in subset])
                ),
                "intervention_count": int(
                    sum(row["intervention_count"] for row in subset)
                ),
            }
            for family in FAMILY_NAMES
            for subset in [[row for row in rows if row["family"] == family]]
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--actor", action="append", type=actor_spec, default=[])
    parser.add_argument("--seed-start", type=int, required=True)
    parser.add_argument("--seeds-per-family", type=int, default=100)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.output.exists():
        raise FileExistsError("OFFLINE_VALIDATION_OUTPUT_ALREADY_EXISTS")
    if arguments.seeds_per_family <= 0:
        raise ValueError("NONPOSITIVE_SEEDS_PER_FAMILY")
    torch.set_num_threads(2)

    methods = [("analytic_prior", None, None)]
    for label, path in arguments.actor:
        absolute = path if path.is_absolute() else ROOT / path
        checkpoint = torch.load(absolute, map_location="cpu", weights_only=False)
        actor = SensorActor.from_actor_checkpoint(checkpoint)
        actor.eval()
        methods.append((label, actor, absolute))

    all_rows = []
    bindings = []
    for label, actor, path in methods:
        all_rows.extend(
            evaluate(label, actor, arguments.seed_start, arguments.seeds_per_family)
        )
        if path is not None:
            bindings.append({"label": label, "path": str(path), "sha256": sha256(path)})
    summaries = {
        label: summarize([row for row in all_rows if row["method"] == label])
        for label, _, _ in methods
    }
    result = {
        "schema_version": "1.0",
        "stage": "PCT-BENCH-DEVELOPMENT-OFFLINE-VALIDATION",
        "scientific_split": "development_offline",
        "formal_seed_accessed": False,
        "seed_start": arguments.seed_start,
        "seeds_per_family": arguments.seeds_per_family,
        "families": list(FAMILY_NAMES),
        "actor_bindings": bindings,
        "summaries": summaries,
    }
    arguments.output.mkdir(parents=True)
    (arguments.output / "report.json").write_text(
        json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    with (arguments.output / "episode_results.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(all_rows[0]))
        writer.writeheader()
        writer.writerows(all_rows)
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
