"""Training-only privileged candidate ranking for the V6 PCT factor."""

import json
from pathlib import Path

import numpy as np
import torch
from torch.nn import functional as torch_functional

from .asymmetric_ppo import observations_to_torch
from .ppo_trainer import PPOTrainer
from .sensor_env import FAMILY_NAMES, SensorCandidateEnv


class PrivilegedRankSensorEnv(SensorCandidateEnv):
    """V5 transition model plus a training-only rank target method."""

    def reset(self, *, seed=None, options=None):
        observation, info = super().reset(seed=seed, options=options)
        if self.family == FAMILY_NAMES[2]:
            self._privileged_hazard_side = float(np.sign(self.static_circles[0]["y"]))
        elif self.family == FAMILY_NAMES[3]:
            self._privileged_hazard_side = float(np.sign(self.actors[0]["y"]))
        else:
            self._privileged_hazard_side = 0.0
        return observation, info

    def privileged_rank_target(self):
        """Return evaluator-only target ID, or -1 when PCT is not applied."""
        if self.family not in (FAMILY_NAMES[2], FAMILY_NAMES[3]) or self.ego_x >= 4.0:
            return -1
        if self._privileged_hazard_side > 0.0:
            spatial_order = (7, 5)
        else:
            spatial_order = (6, 4)
        for action in spatial_order:
            if self._action_mask[action]:
                return action
        for action in (3, 2, 1, 0):
            if self._action_mask[action]:
                return action
        raise RuntimeError("PRIVILEGED_RANK_TARGET_EMPTY_SAFE_SET")


class PrivilegedRankPPOTrainer(PPOTrainer):
    def __init__(self, *args, auxiliary_loss_coefficient=0.35, **kwargs):
        super().__init__(*args, **kwargs)
        self.auxiliary_loss_coefficient = float(auxiliary_loss_coefficient)
        self.environments = [
            PrivilegedRankSensorEnv(self.environment_config)
            for _ in range(self.config.vector_envs)
        ]
        self.episode_counts[:] = 0
        self.observations = []
        for index, environment in enumerate(self.environments):
            observation, _ = environment.reset(seed=self._episode_seed(index))
            self.observations.append(observation)
        self._capture_privileged_targets = False
        self._captured_target_rows = []

    def _privileged_batch(self):
        result = super()._privileged_batch()
        if self._capture_privileged_targets:
            self._captured_target_rows.append(
                np.asarray(
                    [environment.privileged_rank_target() for environment in self.environments],
                    dtype=np.int64,
                )
            )
        return result

    def _collect_rollout(self, rollout_steps):
        self._captured_target_rows = []
        self._capture_privileged_targets = True
        try:
            rollout = super()._collect_rollout(rollout_steps)
        finally:
            self._capture_privileged_targets = False
        # The base collector requests one final privileged batch for bootstrap;
        # the last captured target is therefore not a transition label.
        targets = np.stack(self._captured_target_rows[:rollout_steps])
        if targets.shape != (rollout_steps, self.config.vector_envs):
            raise RuntimeError("PRIVILEGED_TARGET_CAPTURE_SHAPE_MISMATCH")
        rollout["privileged_rank_targets"] = targets
        return rollout

    def _update(self, rollout):
        metrics = super()._update(rollout)
        observations = {
            key: self._flatten(value) for key, value in rollout["observations"].items()
        }
        targets = self._flatten(rollout["privileged_rank_targets"])
        eligible = np.flatnonzero(targets >= 0)
        losses = []
        accuracies = []
        if len(eligible):
            order = np.random.permutation(eligible)
            for start in range(0, len(order), self.config.minibatch_size):
                indices = order[start : start + self.config.minibatch_size]
                observation_torch = observations_to_torch(
                    {key: value[indices] for key, value in observations.items()},
                    self.device,
                )
                target_torch = torch.as_tensor(
                    targets[indices], dtype=torch.int64, device=self.device
                )
                logits, _, _ = self.model.actor(observation_torch)
                cross_entropy = torch_functional.cross_entropy(logits, target_torch)
                loss = self.auxiliary_loss_coefficient * cross_entropy
                self.optimizer.zero_grad(set_to_none=True)
                loss.backward()
                torch.nn.utils.clip_grad_norm_(
                    self.model.parameters(), self.config.maximum_gradient_norm
                )
                self.optimizer.step()
                losses.append(float(cross_entropy.detach().cpu()))
                accuracies.append(
                    float((logits.argmax(dim=-1) == target_torch).float().mean().cpu())
                )
        metrics.update(
            {
                "privileged_rank_eligible_samples": float(len(eligible)),
                "privileged_rank_cross_entropy": float(np.mean(losses)) if losses else 0.0,
                "privileged_rank_accuracy": float(np.mean(accuracies)) if accuracies else 0.0,
            }
        )
        return metrics

    def train(self):
        summary = super().train()
        summary.update(
            {
                "architecture_generation": "v6",
                "single_factor": "privileged_counterfactual_candidate_rank_distillation",
                "auxiliary_loss_coefficient": self.auxiliary_loss_coefficient,
            }
        )
        path = self.output_directory / "training_summary.json"
        path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return summary

