#!/usr/bin/env python3
"""Evaluate an actor-only V5 checkpoint without loading privileged modules."""

import argparse
import csv
import json
from pathlib import Path

import torch

from v5_e2e_scope_rl.asymmetric_ppo import SensorActor
from v5_e2e_scope_rl.evaluation import evaluate_method, summarize_rows


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--actor", type=Path)
    parser.add_argument("--method", required=True)
    parser.add_argument("--seed-start", type=int, required=True)
    parser.add_argument("--blocks", type=int, required=True)
    parser.add_argument("--advantage-gate-margin", type=float)
    parser.add_argument("--no-scan-history", action="store_true")
    parser.add_argument("--output-directory", type=Path, required=True)
    parser.add_argument("--device", default="cpu")
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    actor = None
    if arguments.method != "sensor_analytic_prior":
        if arguments.actor is None:
            raise ValueError("ACTOR_PATH_REQUIRED")
        checkpoint = torch.load(arguments.actor, map_location="cpu", weights_only=False)
        actor = SensorActor.from_actor_checkpoint(checkpoint, map_location=arguments.device)
        actor.eval()
    rows = evaluate_method(
        seeds=range(arguments.seed_start, arguments.seed_start + arguments.blocks),
        method=arguments.method,
        actor=actor,
        advantage_gate_margin=arguments.advantage_gate_margin,
        no_scan_history=arguments.no_scan_history,
        device=arguments.device,
    )
    arguments.output_directory.mkdir(parents=True, exist_ok=False)
    with (arguments.output_directory / "episodes.csv").open(
        "w", encoding="utf-8", newline=""
    ) as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    summary = summarize_rows(rows)
    summary.update(
        {
            "method": arguments.method,
            "seed_start": arguments.seed_start,
            "blocks": arguments.blocks,
            "actor_path": str(arguments.actor) if arguments.actor else None,
            "advantage_gate_margin": arguments.advantage_gate_margin,
            "no_scan_history": arguments.no_scan_history,
        }
    )
    with (arguments.output_directory / "summary.json").open(
        "w", encoding="utf-8"
    ) as handle:
        json.dump(summary, handle, indent=2, sort_keys=True)
        handle.write("\n")
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

