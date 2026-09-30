#!/usr/bin/env python3
"""Fine-tune one V6 PCT identity from a frozen selected V5 checkpoint."""

import argparse
import hashlib
import json
from pathlib import Path

import torch

from v5_e2e_scope_rl.asymmetric_ppo import ActorConfig, AsymmetricActorCritic
from v5_e2e_scope_rl.ppo_trainer import PPOConfig
from v5_e2e_scope_rl.privileged_distillation import PrivilegedRankPPOTrainer


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--initial-trainer", type=Path, required=True)
    parser.add_argument("--initial-actor", type=Path, required=True)
    parser.add_argument("--expected-initial-actor-sha256", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--steps", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    if arguments.output.exists():
        raise FileExistsError("TRAINING_IDENTITY_OUTPUT_ALREADY_EXISTS")
    checkpoint = torch.load(arguments.initial_trainer, map_location="cpu", weights_only=False)
    model = AsymmetricActorCritic(
        privileged_feature_count=int(checkpoint["privileged_feature_count"]),
        asymmetric_critic=bool(checkpoint["asymmetric_critic"]),
        use_analytic_prior=bool(checkpoint["use_analytic_prior"]),
        actor_config=ActorConfig(**dict(checkpoint["actor_config"])),
    )
    model.load_state_dict(checkpoint["state_dict"])
    observed_actor_sha = sha256(arguments.initial_actor)
    if observed_actor_sha != arguments.expected_initial_actor_sha256:
        raise RuntimeError(
            "INITIAL_ACTOR_HASH_MISMATCH expected={} observed={}".format(
                arguments.expected_initial_actor_sha256, observed_actor_sha
            )
        )
    actor_checkpoint = torch.load(arguments.initial_actor, map_location="cpu", weights_only=False)
    actor_state = actor_checkpoint["state_dict"]
    model_state = model.actor.state_dict()
    if actor_state.keys() != model_state.keys() or any(
        not torch.equal(actor_state[name], model_state[name]) for name in actor_state
    ):
        raise RuntimeError("INITIAL_TRAINER_ACTOR_TENSOR_MISMATCH")
    trainer = PrivilegedRankPPOTrainer(
        model=model,
        seed=arguments.seed,
        output_directory=arguments.output,
        config=PPOConfig(
            total_steps=arguments.steps,
            vector_envs=12,
            rollout_steps=64,
            checkpoint_interval_steps=40000,
        ),
        auxiliary_loss_coefficient=0.35,
    )
    summary = trainer.train()
    summary["initial_trainer_path"] = str(arguments.initial_trainer)
    summary["initial_trainer_sha256"] = sha256(arguments.initial_trainer)
    summary["initial_actor_path"] = str(arguments.initial_actor)
    summary["initial_actor_sha256"] = observed_actor_sha
    (arguments.output / "training_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
