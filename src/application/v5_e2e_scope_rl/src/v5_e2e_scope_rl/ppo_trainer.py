"""Bounded local PPO implementation for the V5 sensor interaction backend."""

import json
import random
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Dict, List, Mapping, Optional

import numpy as np
import torch
from torch import nn

from .asymmetric_ppo import (
    POLICY_OBSERVATION_KEYS,
    AsymmetricActorCritic,
    observations_to_torch,
    stack_observations,
)
from .sensor_env import SensorCandidateEnv


@dataclass
class PPOConfig:
    total_steps: int = 250000
    vector_envs: int = 12
    rollout_steps: int = 64
    update_epochs: int = 4
    minibatch_size: int = 256
    learning_rate: float = 3.0e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.20
    entropy_coefficient: float = 0.012
    value_coefficient: float = 0.50
    maximum_gradient_norm: float = 0.50
    target_kl: float = 0.025
    checkpoint_interval_steps: int = 50000
    torch_threads: int = 2


class PPOTrainer:
    def __init__(
        self,
        model: AsymmetricActorCritic,
        seed: int,
        output_directory: Path,
        config: Optional[PPOConfig] = None,
        environment_config: Optional[Mapping[str, object]] = None,
        device: str = "cpu",
    ):
        self.model = model
        self.seed = int(seed)
        self.output_directory = Path(output_directory)
        self.output_directory.mkdir(parents=True, exist_ok=True)
        self.config = config or PPOConfig()
        self.environment_config = dict(environment_config or {})
        self.device = torch.device(device)
        random.seed(self.seed)
        np.random.seed(self.seed)
        torch.manual_seed(self.seed)
        torch.set_num_threads(self.config.torch_threads)
        torch.use_deterministic_algorithms(True, warn_only=True)
        self.model.to(self.device)
        self.optimizer = torch.optim.Adam(
            self.model.parameters(), lr=self.config.learning_rate, eps=1.0e-5
        )
        self.environments = [
            SensorCandidateEnv(self.environment_config)
            for _ in range(self.config.vector_envs)
        ]
        self.episode_counts = np.zeros(self.config.vector_envs, dtype=np.int64)
        self.observations = []
        for index, environment in enumerate(self.environments):
            observation, _ = environment.reset(seed=self._episode_seed(index))
            self.observations.append(observation)
        self.global_steps = 0
        self.next_checkpoint_step = self.config.checkpoint_interval_steps
        self.episode_rows: List[Dict[str, object]] = []
        self.update_rows: List[Dict[str, object]] = []

    def _episode_seed(self, environment_index: int) -> int:
        # Each training identity owns a deterministic, disjoint episode stream.
        return (
            self.seed * 1_000_000
            + int(environment_index) * 10_000
            + int(self.episode_counts[environment_index])
        ) % (2**31 - 1)

    def _privileged_batch(self) -> np.ndarray:
        return np.stack(
            [environment.privileged_observation() for environment in self.environments]
        ).astype(np.float32)

    def _record_episode(self, environment_index: int, info: Mapping[str, object]) -> None:
        self.episode_rows.append(
            {
                "training_seed": self.seed,
                "environment_index": int(environment_index),
                "episode_index": int(self.episode_counts[environment_index]),
                "episode_seed": int(self._episode_seed(environment_index)),
                "global_steps": int(self.global_steps),
                "family": str(info["family_evaluator_only"]),
                "goal_reached": bool(info["goal_reached"]),
                "contact_count": int(info["contact_count"]),
                "mask_violation_count": int(info["mask_violation_count"]),
                "activation_revalidation_failure_count": int(
                    info["activation_revalidation_failure_count"]
                ),
                "restricted_elapsed_s": float(info["restricted_elapsed_s"]),
                "minimum_physical_clearance_m": float(
                    info["minimum_physical_clearance_m"]
                ),
            }
        )

    def _collect_rollout(self, rollout_steps: int) -> Dict[str, object]:
        observation_rows = {key: [] for key in POLICY_OBSERVATION_KEYS}
        privileged_rows = []
        action_rows = []
        log_probability_rows = []
        reward_rows = []
        done_rows = []
        value_rows = []
        entropy_rows = []

        for _ in range(rollout_steps):
            observation_numpy = stack_observations(self.observations)
            privileged_numpy = self._privileged_batch()
            observation_torch = observations_to_torch(observation_numpy, self.device)
            privileged_torch = torch.as_tensor(
                privileged_numpy, dtype=torch.float32, device=self.device
            )
            with torch.no_grad():
                distribution, values, _ = self.model.distribution_and_value(
                    observation_torch,
                    privileged_torch if self.model.asymmetric_critic else None,
                )
                actions = distribution.sample()
                log_probabilities = distribution.log_prob(actions)
                entropies = distribution.entropy()
            for key in POLICY_OBSERVATION_KEYS:
                observation_rows[key].append(observation_numpy[key])
            privileged_rows.append(privileged_numpy)
            action_rows.append(actions.cpu().numpy())
            log_probability_rows.append(log_probabilities.cpu().numpy())
            value_rows.append(values.cpu().numpy())
            entropy_rows.append(entropies.cpu().numpy())

            next_observations = []
            rewards = []
            dones = []
            for index, (environment, action) in enumerate(
                zip(self.environments, actions.cpu().numpy())
            ):
                observation, reward, terminated, truncated, info = environment.step(int(action))
                done = bool(terminated or truncated)
                rewards.append(float(reward))
                dones.append(done)
                if done:
                    self._record_episode(index, info)
                    self.episode_counts[index] += 1
                    observation, _ = environment.reset(seed=self._episode_seed(index))
                next_observations.append(observation)
            reward_rows.append(np.asarray(rewards, dtype=np.float32))
            done_rows.append(np.asarray(dones, dtype=np.float32))
            self.observations = next_observations
            self.global_steps += self.config.vector_envs

        final_observation_numpy = stack_observations(self.observations)
        final_privileged_numpy = self._privileged_batch()
        with torch.no_grad():
            final_values = self.model.value(
                observations_to_torch(final_observation_numpy, self.device),
                torch.as_tensor(
                    final_privileged_numpy, dtype=torch.float32, device=self.device
                )
                if self.model.asymmetric_critic
                else None,
            ).cpu().numpy()

        values = np.stack(value_rows)
        rewards = np.stack(reward_rows)
        dones = np.stack(done_rows)
        advantages = np.zeros_like(rewards)
        last_advantage = np.zeros(self.config.vector_envs, dtype=np.float32)
        for time_index in reversed(range(rollout_steps)):
            next_values = final_values if time_index == rollout_steps - 1 else values[time_index + 1]
            nonterminal = 1.0 - dones[time_index]
            delta = (
                rewards[time_index]
                + self.config.gamma * next_values * nonterminal
                - values[time_index]
            )
            last_advantage = (
                delta
                + self.config.gamma
                * self.config.gae_lambda
                * nonterminal
                * last_advantage
            )
            advantages[time_index] = last_advantage
        returns = advantages + values
        return {
            "observations": {
                key: np.stack(rows) for key, rows in observation_rows.items()
            },
            "privileged": np.stack(privileged_rows),
            "actions": np.stack(action_rows),
            "old_log_probabilities": np.stack(log_probability_rows),
            "old_values": values,
            "advantages": advantages,
            "returns": returns,
            "mean_rollout_entropy": float(np.mean(entropy_rows)),
            "mean_rollout_reward": float(np.mean(rewards)),
        }

    def _flatten(self, value: np.ndarray) -> np.ndarray:
        return value.reshape((-1,) + value.shape[2:])

    def _update(self, rollout: Mapping[str, object]) -> Dict[str, float]:
        observations = {
            key: self._flatten(value)
            for key, value in rollout["observations"].items()
        }
        privileged = self._flatten(rollout["privileged"])
        actions = self._flatten(rollout["actions"])
        old_log_probabilities = self._flatten(rollout["old_log_probabilities"])
        old_values = self._flatten(rollout["old_values"])
        advantages = self._flatten(rollout["advantages"])
        returns = self._flatten(rollout["returns"])
        advantages = (advantages - advantages.mean()) / (advantages.std() + 1.0e-8)
        batch_size = len(actions)
        metrics = {
            "policy_loss": [],
            "value_loss": [],
            "entropy": [],
            "approximate_kl": [],
            "clip_fraction": [],
        }
        stop_early = False
        for _ in range(self.config.update_epochs):
            order = np.random.permutation(batch_size)
            for start in range(0, batch_size, self.config.minibatch_size):
                indices = order[start : start + self.config.minibatch_size]
                observation_torch = observations_to_torch(
                    {key: value[indices] for key, value in observations.items()},
                    self.device,
                )
                privileged_torch = torch.as_tensor(
                    privileged[indices], dtype=torch.float32, device=self.device
                )
                action_torch = torch.as_tensor(
                    actions[indices], dtype=torch.int64, device=self.device
                )
                old_log_probability_torch = torch.as_tensor(
                    old_log_probabilities[indices], dtype=torch.float32, device=self.device
                )
                old_value_torch = torch.as_tensor(
                    old_values[indices], dtype=torch.float32, device=self.device
                )
                advantage_torch = torch.as_tensor(
                    advantages[indices], dtype=torch.float32, device=self.device
                )
                return_torch = torch.as_tensor(
                    returns[indices], dtype=torch.float32, device=self.device
                )
                distribution, values, _ = self.model.distribution_and_value(
                    observation_torch,
                    privileged_torch if self.model.asymmetric_critic else None,
                )
                log_probabilities = distribution.log_prob(action_torch)
                log_ratio = log_probabilities - old_log_probability_torch
                ratio = log_ratio.exp()
                unclipped = -advantage_torch * ratio
                clipped = -advantage_torch * torch.clamp(
                    ratio,
                    1.0 - self.config.clip_range,
                    1.0 + self.config.clip_range,
                )
                policy_loss = torch.maximum(unclipped, clipped).mean()
                clipped_values = old_value_torch + torch.clamp(
                    values - old_value_torch,
                    -self.config.clip_range,
                    self.config.clip_range,
                )
                value_loss = 0.5 * torch.maximum(
                    (values - return_torch).square(),
                    (clipped_values - return_torch).square(),
                ).mean()
                entropy = distribution.entropy().mean()
                loss = (
                    policy_loss
                    + self.config.value_coefficient * value_loss
                    - self.config.entropy_coefficient * entropy
                )
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                nn.utils.clip_grad_norm_(
                    self.model.parameters(), self.config.maximum_gradient_norm
                )
                self.optimizer.step()
                with torch.no_grad():
                    approximate_kl = ((ratio - 1.0) - log_ratio).mean()
                    clip_fraction = (
                        (torch.abs(ratio - 1.0) > self.config.clip_range)
                        .to(dtype=torch.float32)
                        .mean()
                    )
                metrics["policy_loss"].append(float(policy_loss.detach().cpu()))
                metrics["value_loss"].append(float(value_loss.detach().cpu()))
                metrics["entropy"].append(float(entropy.detach().cpu()))
                metrics["approximate_kl"].append(float(approximate_kl.cpu()))
                metrics["clip_fraction"].append(float(clip_fraction.cpu()))
                if float(approximate_kl) > self.config.target_kl:
                    stop_early = True
                    break
            if stop_early:
                break
        return {key: float(np.mean(value)) for key, value in metrics.items()}

    def _save_checkpoint(self, label: str) -> None:
        actor_path = self.output_directory / ("actor_" + label + ".pt")
        full_path = self.output_directory / ("trainer_" + label + ".pt")
        torch.save(self.model.actor.actor_checkpoint(), actor_path)
        torch.save(
            {
                **self.model.full_checkpoint(),
                "optimizer_state_dict": self.optimizer.state_dict(),
                "global_steps": self.global_steps,
                "training_seed": self.seed,
                "ppo_config": asdict(self.config),
                "environment_config": self.environment_config,
            },
            full_path,
        )

    def train(self) -> Dict[str, object]:
        started = time.time()
        updates_path = self.output_directory / "training_updates.jsonl"
        while self.global_steps < self.config.total_steps:
            remaining = self.config.total_steps - self.global_steps
            rollout_steps = min(
                self.config.rollout_steps,
                max(1, int(np.ceil(remaining / self.config.vector_envs))),
            )
            rollout = self._collect_rollout(rollout_steps)
            update_metrics = self._update(rollout)
            row = {
                "global_steps": int(self.global_steps),
                "completed_episodes": int(len(self.episode_rows)),
                "mean_rollout_reward": rollout["mean_rollout_reward"],
                "mean_rollout_entropy": rollout["mean_rollout_entropy"],
                **update_metrics,
            }
            self.update_rows.append(row)
            with updates_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(row, sort_keys=True) + "\n")
            if self.global_steps >= self.next_checkpoint_step:
                self._save_checkpoint("step_%07d" % self.global_steps)
                while self.next_checkpoint_step <= self.global_steps:
                    self.next_checkpoint_step += self.config.checkpoint_interval_steps
        self._save_checkpoint("final")
        summary = {
            "status": "training_complete",
            "training_seed": self.seed,
            "global_steps": int(self.global_steps),
            "wall_time_s": float(time.time() - started),
            "completed_episodes": int(len(self.episode_rows)),
            "goal_count": int(sum(row["goal_reached"] for row in self.episode_rows)),
            "contact_count": int(sum(row["contact_count"] for row in self.episode_rows)),
            "mask_violation_count": int(
                sum(row["mask_violation_count"] for row in self.episode_rows)
            ),
            "minimum_physical_clearance_m": float(
                min(
                    (row["minimum_physical_clearance_m"] for row in self.episode_rows),
                    default=float("nan"),
                )
            ),
            "ppo_config": asdict(self.config),
            "asymmetric_critic": self.model.asymmetric_critic,
            "use_analytic_prior": self.model.actor.use_analytic_prior,
        }
        with (self.output_directory / "training_summary.json").open(
            "w", encoding="utf-8"
        ) as handle:
            json.dump(summary, handle, indent=2, sort_keys=True)
            handle.write("\n")
        with (self.output_directory / "training_episodes.jsonl").open(
            "w", encoding="utf-8"
        ) as handle:
            for row in self.episode_rows:
                handle.write(json.dumps(row, sort_keys=True) + "\n")
        return summary

