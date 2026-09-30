#!/usr/bin/env python3
"""Evaluate one saved V4 policy on an explicit seed interval."""

import argparse
import sys

from v4_scope_rl.evaluation import evaluate_paired
from v4_scope_rl.io_utils import atomic_yaml, sha256_file
from v4_scope_rl.masked_ppo import MaskedPPO


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--seed-first", type=int, required=True)
    parser.add_argument("--seed-last", type=int, required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--cautious-memory-s", type=float, default=2.5)
    parser.add_argument("--hesitant-memory-s", type=float, default=1.5)
    parser.add_argument("--elapsed-reward-per-s", type=float, default=-0.4)
    parser.add_argument("--maximum-actor-deceleration-mps2", type=float, default=1.35)
    parser.add_argument("--traces", action="store_true")
    args = parser.parse_args()
    if args.seed_last < args.seed_first:
        raise RuntimeError("INVALID_SEED_INTERVAL")
    model, metadata = MaskedPPO.load(args.model)
    result = evaluate_paired(
        range(args.seed_first, args.seed_last + 1),
        model,
        include_traces=args.traces,
        env_config={
            "cautious_yield_memory_s": args.cautious_memory_s,
            "hesitant_yield_memory_s": args.hesitant_memory_s,
            "elapsed_reward_per_s": args.elapsed_reward_per_s,
            "maximum_actor_deceleration_mps2": args.maximum_actor_deceleration_mps2,
        },
    )
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v4",
        "stage": "V4-EVAL",
        "simulation_only": True,
        "hardware_accessed": False,
        "model": {"path": args.model, "sha256": sha256_file(args.model), "metadata": metadata},
        "seed_range_inclusive": [args.seed_first, args.seed_last],
        **result,
    }
    report_sha256 = atomic_yaml(args.output, report)
    print("{} {}".format(args.output, report_sha256))
    return 0


if __name__ == "__main__":
    sys.exit(main())
