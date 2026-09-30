#!/usr/bin/env python3
"""Bounded multi-seed Masked-PPO trainer for V4-SCOPE-RL."""

import argparse
import sys
from pathlib import Path

import torch

from v4_scope_rl.contracts import load_contract
from v4_scope_rl.evaluation import evaluate_paired
from v4_scope_rl.fast_env import CounterfactualCrossingEnv
from v4_scope_rl.io_utils import atomic_yaml, sha256_file
from v4_scope_rl.masked_ppo import MaskedPPO, PPOConfig


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--contract", required=True)
    parser.add_argument("--factor", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--steps", type=int, required=True)
    parser.add_argument("--device", default="cpu")
    return parser.parse_args()


def main():
    args = parse_args()
    contract = load_contract(args.contract)
    import yaml

    with open(args.factor, "r", encoding="utf-8") as stream:
        factor = yaml.safe_load(stream)
    factor_stage = factor.get("stage")
    if factor_stage == "V4-02-F1":
        allowed_seeds = [4102]
        maximum_per_seed = int(factor["budget"]["maximum_training_environment_steps"])
        candidate = factor["single_factor"]["candidate"]
        env_config = {
            "cautious_yield_memory_s": float(candidate["cautious_yield_memory_s"]),
            "hesitant_yield_memory_s": float(candidate["hesitant_yield_memory_s"]),
        }
        validation_first, validation_last = 421200, 421399
        result_stage = "V4-03-F1"
    elif factor_stage == "V4-03-F2":
        allowed_seeds = [int(value) for value in factor["training"]["seeds"]]
        maximum_per_seed = int(factor["training"]["maximum_steps_per_seed"])
        response = factor["frozen_nonfactors"]["actor_response"]
        env_config = {
            "cautious_yield_memory_s": float(response["cautious_yield_memory_s"]),
            "hesitant_yield_memory_s": float(response["hesitant_yield_memory_s"]),
            "elapsed_reward_per_s": float(factor["single_factor"]["candidate_per_s"]),
        }
        validation_first, validation_last = factor["training"][
            "checkpoint_validation_seed_range_inclusive"
        ]
        result_stage = "V4-03-F2"
    elif factor_stage == "V4-03-F3":
        allowed_seeds = [int(factor["training"]["seed"])]
        maximum_per_seed = int(factor["training"]["maximum_steps"])
        response = factor["frozen_nonfactors"]["actor_response_persistence"]
        env_config = {
            "cautious_yield_memory_s": float(response["cautious_yield_memory_s"]),
            "hesitant_yield_memory_s": float(response["hesitant_yield_memory_s"]),
            "elapsed_reward_per_s": float(
                factor["frozen_nonfactors"]["elapsed_time_reward_per_s"]
            ),
            "maximum_actor_deceleration_mps2": float(
                factor["single_factor"]["candidate_mps2"]
            ),
        }
        validation_first, validation_last = factor["training"][
            "checkpoint_validation_seed_range_inclusive"
        ]
        result_stage = "V4-03-F3"
    else:
        raise RuntimeError("FACTOR_STAGE_MISMATCH")
    if args.seed not in allowed_seeds:
        raise RuntimeError("TRAINING_SEED_MISMATCH")
    if args.steps > maximum_per_seed or args.steps <= 0:
        raise RuntimeError("TRAINING_BUDGET_EXCEEDED")
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    torch.set_num_threads(2)
    probe = CounterfactualCrossingEnv(env_config)
    configuration = PPOConfig(
        environment_count=8,
        rollout_steps=256,
        update_epochs=6,
        minibatch_size=256,
        hidden_size=128,
    )
    model = MaskedPPO(
        probe.observation_space.shape[0],
        probe.action_space.n,
        configuration,
        seed=args.seed,
        device=args.device,
    )
    validation_seeds = list(range(int(validation_first), int(validation_last) + 1))
    validations = []
    best_return = float("-inf")
    best_steps = None
    best_path = output_dir / "best_model.pt"

    def callback(current_model, update):
        nonlocal best_return, best_steps
        if update["update"] % 5 != 0 and update["environment_steps"] < args.steps:
            return
        result = evaluate_paired(validation_seeds, current_model, env_config=env_config)
        summary = result["summary"]
        gate = (
            summary["learned_goal_rate"] >= summary["analytic_goal_rate"]
            and summary["contact_count"] == 0
            and summary["mask_violation_count"] == 0
        )
        item = {
            "environment_steps": int(update["environment_steps"]),
            "checkpoint_gate_pass": bool(gate),
            "summary": summary,
            "evidence_sha256": result["evidence_sha256"],
        }
        validations.append(item)
        if gate and summary["learned_mean_return"] > best_return:
            best_return = summary["learned_mean_return"]
            best_steps = int(update["environment_steps"])
            current_model.save(
                best_path,
                metadata={
                    "stage": result_stage,
                    "selected_on": "fresh_validation_return",
                    "validation_seed_range_inclusive": [
                        int(validation_first),
                        int(validation_last),
                    ],
                    "environment_steps": best_steps,
                },
            )
        print(
            "steps={} validation_reduction={:.6f} learned_return={:.6f} gate={}".format(
                update["environment_steps"],
                summary["paired_relative_time_reduction_mean"],
                summary["learned_mean_return"],
                gate,
            ),
            flush=True,
        )

    training = model.train(
        lambda: CounterfactualCrossingEnv(env_config),
        args.steps,
        callback=callback,
    )
    latest_path = output_dir / "latest_model.pt"
    model.save(latest_path, metadata={"stage": result_stage, "selected": False})
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v4",
        "stage": result_stage,
        "status": "bounded_training_complete",
        "simulation_only": True,
        "hardware_accessed": False,
        "contract": {"path": str(Path(args.contract).resolve()), "sha256": sha256_file(args.contract)},
        "factor": {"path": str(Path(args.factor).resolve()), "sha256": sha256_file(args.factor)},
        "training_seed": args.seed,
        "environment_steps": int(training["environment_steps"]),
        "completed_episode_count": int(training["completed_episode_count"]),
        "best_checkpoint_environment_steps": best_steps,
        "best_validation_return": best_return if best_steps is not None else None,
        "best_checkpoint": str(best_path) if best_steps is not None else None,
        "latest_checkpoint": str(latest_path),
        "validation_seed_range_inclusive": [
            int(validation_first),
            int(validation_last),
        ],
        "validations": validations,
        "training_updates": training["updates"],
    }
    report_path = output_dir / "training_report.yaml"
    atomic_yaml(report_path, report)
    print(str(report_path))
    return 0


if __name__ == "__main__":
    sys.exit(main())
