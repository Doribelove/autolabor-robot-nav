"""Masked recurrent actor-critic used by V5-E2E-SCOPE-RL.

The actor API accepts only deployable policy observations.  Privileged
simulation state enters through the value path of ``AsymmetricActorCritic``;
it is intentionally absent from ``SensorActor.forward`` and actor-only
checkpoints.
"""

from dataclasses import asdict, dataclass
from typing import Dict, Iterable, Mapping, Optional, Tuple

import numpy as np
import torch
from torch import nn
from torch.distributions import Categorical


POLICY_OBSERVATION_KEYS = (
    "scans",
    "robot",
    "candidates",
    "analytic_prior",
    "action_mask",
)


@dataclass(frozen=True)
class ActorConfig:
    history_frames: int = 6
    beam_count: int = 72
    robot_features: int = 10
    candidate_count: int = 8
    candidate_features: int = 7
    scan_embedding: int = 48
    temporal_embedding: int = 64
    robot_embedding: int = 32
    candidate_embedding: int = 32
    residual_scale: float = 3.0
    analytic_prior_scale: float = 1.0


def stack_observations(observations: Iterable[Mapping[str, np.ndarray]]) -> Dict[str, np.ndarray]:
    observations = list(observations)
    return {
        key: np.stack([observation[key] for observation in observations])
        for key in POLICY_OBSERVATION_KEYS
    }


def observations_to_torch(
    observations: Mapping[str, np.ndarray],
    device: torch.device,
) -> Dict[str, torch.Tensor]:
    result = {}
    for key in POLICY_OBSERVATION_KEYS:
        tensor = torch.as_tensor(observations[key], device=device)
        if key == "action_mask":
            tensor = tensor.to(dtype=torch.bool)
        else:
            tensor = tensor.to(dtype=torch.float32)
        result[key] = tensor
    return result


class SensorActor(nn.Module):
    """Laser/odometry/candidate actor with a GRU over scan frames."""

    def __init__(self, config: Optional[ActorConfig] = None, use_analytic_prior: bool = True):
        super().__init__()
        self.config = config or ActorConfig()
        self.use_analytic_prior = bool(use_analytic_prior)
        config = self.config
        self.scan_cnn = nn.Sequential(
            nn.Conv1d(1, 12, kernel_size=7, stride=3, padding=3),
            nn.ReLU(),
            nn.Conv1d(12, 24, kernel_size=5, stride=2, padding=2),
            nn.ReLU(),
            nn.AdaptiveAvgPool1d(6),
            nn.Flatten(),
            nn.Linear(24 * 6, config.scan_embedding),
            nn.ReLU(),
        )
        self.temporal_gru = nn.GRU(
            input_size=config.scan_embedding,
            hidden_size=config.temporal_embedding,
            batch_first=True,
        )
        self.robot_mlp = nn.Sequential(
            nn.Linear(config.robot_features, 48),
            nn.ReLU(),
            nn.Linear(48, config.robot_embedding),
            nn.ReLU(),
        )
        self.candidate_mlp = nn.Sequential(
            nn.Linear(config.candidate_features, 48),
            nn.ReLU(),
            nn.Linear(48, config.candidate_embedding),
            nn.ReLU(),
        )
        fusion_features = (
            config.temporal_embedding
            + config.robot_embedding
            + config.candidate_embedding
            + 2
        )
        self.residual_head = nn.Sequential(
            nn.Linear(fusion_features, 96),
            nn.Tanh(),
            nn.Linear(96, 48),
            nn.Tanh(),
            nn.Linear(48, 1),
        )
        nn.init.zeros_(self.residual_head[-1].weight)
        nn.init.zeros_(self.residual_head[-1].bias)

    @property
    def value_feature_count(self) -> int:
        return (
            self.config.temporal_embedding
            + self.config.robot_embedding
            + self.config.candidate_embedding
        )

    def _validate_observation(self, observation: Mapping[str, torch.Tensor]) -> None:
        if set(observation) != set(POLICY_OBSERVATION_KEYS):
            raise ValueError("ACTOR_OBSERVATION_SCHEMA_VIOLATION")

    def forward(
        self,
        observation: Mapping[str, torch.Tensor],
    ) -> Tuple[torch.Tensor, torch.Tensor, torch.Tensor]:
        self._validate_observation(observation)
        scans = observation["scans"]
        batch_size, frame_count, beam_count = scans.shape
        if (frame_count, beam_count) != (
            self.config.history_frames,
            self.config.beam_count,
        ):
            raise ValueError("ACTOR_SCAN_SHAPE_VIOLATION")
        frame_embeddings = self.scan_cnn(scans.reshape(batch_size * frame_count, 1, beam_count))
        frame_embeddings = frame_embeddings.reshape(batch_size, frame_count, -1)
        _, temporal_hidden = self.temporal_gru(frame_embeddings)
        temporal = temporal_hidden[-1]
        robot = self.robot_mlp(observation["robot"])
        candidates = self.candidate_mlp(observation["candidates"])
        context = torch.cat((temporal, robot), dim=-1)
        expanded_context = context[:, None, :].expand(-1, self.config.candidate_count, -1)
        prior = observation["analytic_prior"]
        prior_centered = prior - prior.mean(dim=-1, keepdim=True)
        prior_scale = prior.std(dim=-1, keepdim=True).clamp_min(1.0e-4)
        normalized_prior = prior_centered / prior_scale
        fusion = torch.cat(
            (
                expanded_context,
                candidates,
                normalized_prior[..., None],
                observation["action_mask"].to(dtype=torch.float32)[..., None],
            ),
            dim=-1,
        )
        residual = self.config.residual_scale * torch.tanh(self.residual_head(fusion).squeeze(-1))
        if self.use_analytic_prior:
            logits = self.config.analytic_prior_scale * normalized_prior + residual
        else:
            logits = residual
        logits = logits.masked_fill(~observation["action_mask"], -1.0e9)
        candidate_pool = candidates.max(dim=1).values
        value_features = torch.cat((context, candidate_pool), dim=-1)
        return logits, residual, value_features

    def choose_actions(
        self,
        observation: Mapping[str, torch.Tensor],
        deterministic: bool = True,
        advantage_gate_margin: Optional[float] = None,
    ) -> Tuple[torch.Tensor, Dict[str, torch.Tensor]]:
        logits, residual, _ = self.forward(observation)
        distribution = Categorical(logits=logits)
        learned = torch.argmax(logits, dim=-1) if deterministic else distribution.sample()
        masked_prior = observation["analytic_prior"].masked_fill(
            ~observation["action_mask"], -1.0e9
        )
        analytic = torch.argmax(masked_prior, dim=-1)
        final = learned
        intervened = learned != analytic
        residual_margin = torch.zeros_like(logits[:, 0])
        if advantage_gate_margin is not None:
            row = torch.arange(logits.shape[0], device=logits.device)
            residual_margin = residual[row, learned] - residual[row, analytic]
            intervened = (learned != analytic) & (
                residual_margin >= float(advantage_gate_margin)
            )
            final = torch.where(intervened, learned, analytic)
        return final, {
            "learned_action": learned,
            "analytic_action": analytic,
            "intervened": intervened,
            "residual_margin": residual_margin,
            "logits": logits,
            "residual": residual,
        }

    def actor_checkpoint(self) -> Dict[str, object]:
        return {
            "format": "v5_sensor_actor_only_v1",
            "actor_config": asdict(self.config),
            "use_analytic_prior": self.use_analytic_prior,
            "state_dict": self.state_dict(),
            "policy_observation_keys": POLICY_OBSERVATION_KEYS,
            "contains_privileged_modules": False,
        }

    @classmethod
    def from_actor_checkpoint(
        cls,
        checkpoint: Mapping[str, object],
        map_location: str = "cpu",
    ) -> "SensorActor":
        if checkpoint.get("format") != "v5_sensor_actor_only_v1":
            raise ValueError("UNKNOWN_ACTOR_CHECKPOINT_FORMAT")
        if checkpoint.get("contains_privileged_modules") is not False:
            raise ValueError("ACTOR_CHECKPOINT_PRIVILEGE_LEAK")
        actor = cls(
            ActorConfig(**dict(checkpoint["actor_config"])),
            use_analytic_prior=bool(checkpoint["use_analytic_prior"]),
        )
        actor.load_state_dict(checkpoint["state_dict"])
        return actor.to(map_location)


class AsymmetricActorCritic(nn.Module):
    """PPO model whose value function optionally receives simulator truth."""

    def __init__(
        self,
        privileged_feature_count: int = 32,
        asymmetric_critic: bool = True,
        use_analytic_prior: bool = True,
        actor_config: Optional[ActorConfig] = None,
    ):
        super().__init__()
        self.asymmetric_critic = bool(asymmetric_critic)
        self.privileged_feature_count = int(privileged_feature_count)
        self.actor = SensorActor(actor_config, use_analytic_prior=use_analytic_prior)
        privileged_embedding = 0
        if self.asymmetric_critic:
            privileged_embedding = 64
            self.privileged_encoder = nn.Sequential(
                nn.Linear(self.privileged_feature_count, 96),
                nn.Tanh(),
                nn.Linear(96, privileged_embedding),
                nn.Tanh(),
            )
        else:
            self.privileged_encoder = None
        self.value_head = nn.Sequential(
            nn.Linear(self.actor.value_feature_count + privileged_embedding, 128),
            nn.Tanh(),
            nn.Linear(128, 64),
            nn.Tanh(),
            nn.Linear(64, 1),
        )

    def distribution_and_value(
        self,
        observation: Mapping[str, torch.Tensor],
        privileged: Optional[torch.Tensor],
    ) -> Tuple[Categorical, torch.Tensor, torch.Tensor]:
        logits, residual, value_features = self.actor(observation)
        if self.asymmetric_critic:
            if privileged is None:
                raise ValueError("ASYMMETRIC_CRITIC_REQUIRES_PRIVILEGED_STATE")
            if privileged.shape[-1] != self.privileged_feature_count:
                raise ValueError("PRIVILEGED_VALUE_SCHEMA_VIOLATION")
            value_features = torch.cat(
                (value_features, self.privileged_encoder(privileged)), dim=-1
            )
        value = self.value_head(value_features).squeeze(-1)
        return Categorical(logits=logits), value, residual

    def value(
        self,
        observation: Mapping[str, torch.Tensor],
        privileged: Optional[torch.Tensor],
    ) -> torch.Tensor:
        return self.distribution_and_value(observation, privileged)[1]

    def full_checkpoint(self) -> Dict[str, object]:
        return {
            "format": "v5_asymmetric_actor_critic_v1",
            "asymmetric_critic": self.asymmetric_critic,
            "privileged_feature_count": self.privileged_feature_count,
            "actor_config": asdict(self.actor.config),
            "use_analytic_prior": self.actor.use_analytic_prior,
            "state_dict": self.state_dict(),
        }

