"""Small, auditable PyTorch implementation of PPO with invalid-action masks."""

import json
import random
from dataclasses import asdict, dataclass
from pathlib import Path

import numpy as np
import torch
from torch import nn
from torch.distributions import Categorical


def masked_logits(logits, masks):
    if logits.shape != masks.shape:
        raise ValueError("LOGIT_MASK_SHAPE_MISMATCH")
    if not bool(torch.all(torch.any(masks, dim=-1))):
        raise ValueError("EMPTY_SAFE_ACTION_SET")
    return logits.masked_fill(~masks, torch.finfo(logits.dtype).min)


class MaskedActorCritic(nn.Module):
    def __init__(self, observation_size, action_count, hidden_size=128):
        super().__init__()
        self.observation_size = int(observation_size)
        self.action_count = int(action_count)
        self.hidden_size = int(hidden_size)
        self.backbone = nn.Sequential(
            nn.Linear(self.observation_size, self.hidden_size),
            nn.Tanh(),
            nn.Linear(self.hidden_size, self.hidden_size),
            nn.Tanh(),
        )
        self.policy_head = nn.Linear(self.hidden_size, self.action_count)
        self.value_head = nn.Linear(self.hidden_size, 1)
        for module in self.modules():
            if isinstance(module, nn.Linear):
                nn.init.orthogonal_(module.weight, gain=np.sqrt(2.0))
                nn.init.constant_(module.bias, 0.0)
        nn.init.orthogonal_(self.policy_head.weight, gain=0.01)
        nn.init.orthogonal_(self.value_head.weight, gain=1.0)

    def forward(self, observations):
        hidden = self.backbone(observations)
        return self.policy_head(hidden), self.value_head(hidden).squeeze(-1)

    def distribution(self, observations, masks):
        logits, values = self(observations)
        return Categorical(logits=masked_logits(logits, masks)), values


@dataclass
class PPOConfig:
    learning_rate: float = 3.0e-4
    gamma: float = 0.99
    gae_lambda: float = 0.95
    clip_range: float = 0.20
    entropy_coefficient: float = 0.015
    value_coefficient: float = 0.50
    maximum_gradient_norm: float = 0.50
    rollout_steps: int = 256
    environment_count: int = 8
    update_epochs: int = 8
    minibatch_size: int = 256
    hidden_size: int = 128


class MaskedPPO:
    def __init__(self, observation_size, action_count, config=None, seed=0, device="cpu"):
        self.config = config or PPOConfig()
        self.seed = int(seed)
        self.device = torch.device(device)
        random.seed(self.seed)
        np.random.seed(self.seed)
        torch.manual_seed(self.seed)
        self.network = MaskedActorCritic(
            observation_size,
            action_count,
            hidden_size=self.config.hidden_size,
        ).to(self.device)
        self.optimizer = torch.optim.Adam(
            self.network.parameters(), lr=self.config.learning_rate, eps=1.0e-5
        )
        self.environment_steps = 0
        self.update_count = 0

    def predict(self, observation, action_mask, deterministic=True):
        observations = torch.as_tensor(
            np.asarray(observation, dtype=np.float32)[None, :], device=self.device
        )
        masks = torch.as_tensor(
            np.asarray(action_mask, dtype=bool)[None, :], device=self.device
        )
        with torch.no_grad():
            distribution, _ = self.network.distribution(observations, masks)
            if deterministic:
                action = torch.argmax(distribution.logits, dim=-1)
            else:
                action = distribution.sample()
        value = int(action.item())
        if not bool(np.asarray(action_mask, dtype=bool)[value]):
            raise RuntimeError("POLICY_RETURNED_MASKED_ACTION")
        return value

    def _initial_batch(self, envs):
        observations = []
        for index, env in enumerate(envs):
            observation, _ = env.reset(seed=self.seed * 1000 + index)
            observations.append(observation)
        return np.asarray(observations, dtype=np.float32)

    def train(self, env_factory, total_environment_steps, callback=None):
        config = self.config
        envs = [env_factory() for _ in range(config.environment_count)]
        observations = self._initial_batch(envs)
        episode_returns = np.zeros(config.environment_count, dtype=np.float64)
        episode_lengths = np.zeros(config.environment_count, dtype=np.int64)
        completed = []
        update_reports = []

        batch_capacity = config.rollout_steps * config.environment_count
        while self.environment_steps < int(total_environment_steps):
            obs_buffer = np.zeros((config.rollout_steps, config.environment_count, observations.shape[1]), dtype=np.float32)
            mask_buffer = np.zeros((config.rollout_steps, config.environment_count, envs[0].action_space.n), dtype=bool)
            action_buffer = np.zeros((config.rollout_steps, config.environment_count), dtype=np.int64)
            logprob_buffer = np.zeros((config.rollout_steps, config.environment_count), dtype=np.float32)
            reward_buffer = np.zeros((config.rollout_steps, config.environment_count), dtype=np.float32)
            done_buffer = np.zeros((config.rollout_steps, config.environment_count), dtype=np.float32)
            value_buffer = np.zeros((config.rollout_steps, config.environment_count), dtype=np.float32)

            collected_steps = config.rollout_steps
            for step_index in range(config.rollout_steps):
                masks_np = np.stack([env.action_masks() for env in envs])
                obs_tensor = torch.as_tensor(observations, device=self.device)
                mask_tensor = torch.as_tensor(masks_np, device=self.device)
                with torch.no_grad():
                    distribution, values = self.network.distribution(obs_tensor, mask_tensor)
                    actions = distribution.sample()
                    logprobs = distribution.log_prob(actions)
                actions_np = actions.cpu().numpy()

                obs_buffer[step_index] = observations
                mask_buffer[step_index] = masks_np
                action_buffer[step_index] = actions_np
                logprob_buffer[step_index] = logprobs.cpu().numpy()
                value_buffer[step_index] = values.cpu().numpy()

                next_observations = []
                for env_index, env in enumerate(envs):
                    next_observation, reward, terminated, truncated, info = env.step(actions_np[env_index])
                    done = bool(terminated or truncated)
                    reward_buffer[step_index, env_index] = reward
                    done_buffer[step_index, env_index] = float(done)
                    episode_returns[env_index] += reward
                    episode_lengths[env_index] += 1
                    if done:
                        completed.append(
                            {
                                "return": float(episode_returns[env_index]),
                                "length": int(episode_lengths[env_index]),
                                "goal_reached": bool(info["goal_reached"]),
                                "contact": bool(info["contact"]),
                                "minimum_clearance_m": float(info["minimum_clearance_m"]),
                            }
                        )
                        episode_returns[env_index] = 0.0
                        episode_lengths[env_index] = 0
                        next_observation, _ = env.reset()
                    next_observations.append(next_observation)
                observations = np.asarray(next_observations, dtype=np.float32)
                self.environment_steps += config.environment_count
                if self.environment_steps >= int(total_environment_steps):
                    collected_steps = step_index + 1
                    break

            obs_buffer = obs_buffer[:collected_steps]
            mask_buffer = mask_buffer[:collected_steps]
            action_buffer = action_buffer[:collected_steps]
            logprob_buffer = logprob_buffer[:collected_steps]
            reward_buffer = reward_buffer[:collected_steps]
            done_buffer = done_buffer[:collected_steps]
            value_buffer = value_buffer[:collected_steps]

            next_masks = np.stack([env.action_masks() for env in envs])
            with torch.no_grad():
                _, next_values_tensor = self.network.distribution(
                    torch.as_tensor(observations, device=self.device),
                    torch.as_tensor(next_masks, device=self.device),
                )
            next_values = next_values_tensor.cpu().numpy()
            advantages = np.zeros_like(reward_buffer)
            last_gae = np.zeros(config.environment_count, dtype=np.float32)
            for step_index in reversed(range(collected_steps)):
                next_nonterminal = 1.0 - done_buffer[step_index]
                following_values = next_values if step_index == collected_steps - 1 else value_buffer[step_index + 1]
                delta = (
                    reward_buffer[step_index]
                    + config.gamma * following_values * next_nonterminal
                    - value_buffer[step_index]
                )
                last_gae = delta + config.gamma * config.gae_lambda * next_nonterminal * last_gae
                advantages[step_index] = last_gae
            returns = advantages + value_buffer

            flat_observations = obs_buffer.reshape(-1, obs_buffer.shape[-1])
            flat_masks = mask_buffer.reshape(-1, mask_buffer.shape[-1])
            flat_actions = action_buffer.reshape(-1)
            flat_logprobs = logprob_buffer.reshape(-1)
            flat_advantages = advantages.reshape(-1)
            flat_returns = returns.reshape(-1)
            flat_advantages = (flat_advantages - flat_advantages.mean()) / (flat_advantages.std() + 1.0e-8)
            indices = np.arange(len(flat_actions))
            losses = []
            for _ in range(config.update_epochs):
                np.random.shuffle(indices)
                for start in range(0, len(indices), config.minibatch_size):
                    batch = indices[start : start + config.minibatch_size]
                    distribution, values = self.network.distribution(
                        torch.as_tensor(flat_observations[batch], device=self.device),
                        torch.as_tensor(flat_masks[batch], device=self.device),
                    )
                    new_logprobs = distribution.log_prob(torch.as_tensor(flat_actions[batch], device=self.device))
                    entropy = distribution.entropy().mean()
                    ratio = torch.exp(new_logprobs - torch.as_tensor(flat_logprobs[batch], device=self.device))
                    batch_advantages = torch.as_tensor(flat_advantages[batch], device=self.device)
                    policy_loss = -torch.min(
                        ratio * batch_advantages,
                        torch.clamp(ratio, 1.0 - config.clip_range, 1.0 + config.clip_range) * batch_advantages,
                    ).mean()
                    value_loss = 0.5 * (
                        values - torch.as_tensor(flat_returns[batch], device=self.device)
                    ).pow(2).mean()
                    loss = policy_loss + config.value_coefficient * value_loss - config.entropy_coefficient * entropy
                    self.optimizer.zero_grad()
                    loss.backward()
                    nn.utils.clip_grad_norm_(self.network.parameters(), config.maximum_gradient_norm)
                    self.optimizer.step()
                    losses.append((float(policy_loss.item()), float(value_loss.item()), float(entropy.item())))

            self.update_count += 1
            recent = completed[-100:]
            update_report = {
                "update": self.update_count,
                "environment_steps": self.environment_steps,
                "batch_capacity": batch_capacity,
                "completed_episode_count": len(completed),
                "recent_mean_return": float(np.mean([item["return"] for item in recent])) if recent else None,
                "recent_goal_rate": float(np.mean([item["goal_reached"] for item in recent])) if recent else None,
                "recent_contact_count": int(sum(item["contact"] for item in recent)),
                "policy_loss": float(np.mean([item[0] for item in losses])),
                "value_loss": float(np.mean([item[1] for item in losses])),
                "entropy": float(np.mean([item[2] for item in losses])),
            }
            update_reports.append(update_report)
            if callback is not None:
                callback(self, update_report)

        return {
            "environment_steps": self.environment_steps,
            "update_count": self.update_count,
            "completed_episode_count": len(completed),
            "completed_episodes": completed,
            "updates": update_reports,
        }

    def save(self, path, metadata=None):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "format": "v4_scope_rl_masked_ppo_v1",
            "observation_size": self.network.observation_size,
            "action_count": self.network.action_count,
            "config": asdict(self.config),
            "seed": self.seed,
            "environment_steps": self.environment_steps,
            "update_count": self.update_count,
            "network_state_dict": self.network.state_dict(),
            "optimizer_state_dict": self.optimizer.state_dict(),
            "metadata": dict(metadata or {}),
        }
        temporary = path.with_suffix(path.suffix + ".tmp")
        torch.save(payload, temporary)
        temporary.replace(path)
        metadata_path = path.with_suffix(path.suffix + ".json")
        metadata_path.write_text(
            json.dumps(
                {key: value for key, value in payload.items() if not key.endswith("state_dict")},
                indent=2,
                sort_keys=True,
            )
            + "\n",
            encoding="utf-8",
        )

    @classmethod
    def load(cls, path, device="cpu"):
        payload = torch.load(path, map_location=device, weights_only=False)
        if payload.get("format") != "v4_scope_rl_masked_ppo_v1":
            raise ValueError("UNKNOWN_CHECKPOINT_FORMAT")
        model = cls(
            payload["observation_size"],
            payload["action_count"],
            config=PPOConfig(**payload["config"]),
            seed=payload["seed"],
            device=device,
        )
        model.network.load_state_dict(payload["network_state_dict"])
        model.optimizer.load_state_dict(payload["optimizer_state_dict"])
        model.environment_steps = int(payload["environment_steps"])
        model.update_count = int(payload["update_count"])
        return model, payload.get("metadata", {})
