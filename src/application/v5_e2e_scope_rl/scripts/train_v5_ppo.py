#!/usr/bin/env python3
"""Train one preregistered V5 PPO identity within an explicit step budget."""

import argparse
import json
from pathlib import Path

from v5_e2e_scope_rl.asymmetric_ppo import AsymmetricActorCritic
from v5_e2e_scope_rl.ppo_trainer import PPOConfig, PPOTrainer


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--variant", choices=("asymmetric", "symmetric", "no_prior"), required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--steps", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--vector-envs", type=int, default=12)
    parser.add_argument("--rollout-steps", type=int, default=64)
    parser.add_argument("--checkpoint-interval", type=int, default=50000)
    parser.add_argument("--device", default="cpu")
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    if arguments.output.exists():
        raise FileExistsError("TRAINING_IDENTITY_OUTPUT_ALREADY_EXISTS:" + str(arguments.output))
    asymmetric = arguments.variant != "symmetric"
    use_prior = arguments.variant != "no_prior"
    model = AsymmetricActorCritic(
        asymmetric_critic=asymmetric,
        use_analytic_prior=use_prior,
    )
    config = PPOConfig(
        total_steps=arguments.steps,
        vector_envs=arguments.vector_envs,
        rollout_steps=arguments.rollout_steps,
        checkpoint_interval_steps=arguments.checkpoint_interval,
    )
    trainer = PPOTrainer(
        model=model,
        seed=arguments.seed,
        output_directory=arguments.output,
        config=config,
        device=arguments.device,
    )
    print(json.dumps(trainer.train(), indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
