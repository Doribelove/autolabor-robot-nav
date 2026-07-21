"""Fair Gymnasium adapters for Semantic-Eta and Direct-Theta SAC."""

import math
from typing import Any, Dict, Optional, Sequence, Tuple

import gymnasium as gym
import numpy as np

from .config import EXPECTED_ETA_ORDER, EXPECTED_THETA_ORDER
from .direct_theta_action import DirectThetaActionError, DirectThetaMapping
from .semantic_action import (
    FrozenSemanticMapping, ResidualSemanticMapping, SemanticActionError,
)


ACTION_CONTEXT_DIMENSION = len(EXPECTED_THETA_ORDER) + 1


def _shared_action_context(delta_normalized_theta: Sequence[Any]) -> Tuple[float, ...]:
    values = tuple(float(value) for value in delta_normalized_theta)
    if len(values) != len(EXPECTED_THETA_ORDER) or not all(math.isfinite(v) for v in values):
        raise SemanticActionError("applied normalized theta delta must be a finite nine-vector")
    return values + (sum(abs(value) for value in values),)


def _applied_delta(mapping: Any, before: Sequence[float], info: Dict[str, Any]) -> Tuple[float, ...]:
    applied = info.get("applied_theta")
    if not isinstance(applied, dict):
        return (0.0,) * len(EXPECTED_THETA_ORDER)
    after = mapping.normalize_theta(applied)
    return tuple(right - left for left, right in zip(before, after))


class _SharedThetaObservationEnv(gym.Env):
    """Keep observation information and dimension identical across the SAC pair."""

    metadata = {"render_modes": []}

    def _initialize_observation(self, core_environment: Any) -> None:
        self.core = core_environment
        history = core_environment.config.history_length
        builder = core_environment.state_builder
        self._base_dimension = history * (builder.sector_count + len(builder.feature_order))
        self.previous_applied_delta = (0.0,) * len(EXPECTED_THETA_ORDER)
        self.observation_space = gym.spaces.Box(
            low=-np.inf, high=np.inf,
            shape=(self._base_dimension + ACTION_CONTEXT_DIMENSION,), dtype=np.float32,
        )

    def _observation(self, base: Sequence[Any]) -> np.ndarray:
        if len(base) != self._base_dimension:
            raise SemanticActionError("core observation dimension changed")
        values = tuple(float(value) for value in base) + _shared_action_context(
            self.previous_applied_delta
        )
        if not all(math.isfinite(value) for value in values):
            raise SemanticActionError("SAC observation contains NaN/Inf")
        return np.asarray(values, dtype=np.float32)

    def _reset_context(self) -> None:
        self.previous_applied_delta = (0.0,) * len(EXPECTED_THETA_ORDER)

    def close(self) -> None:
        if hasattr(self.core.adapter, "close"):
            self.core.adapter.close()


class SemanticEtaGymEnv(_SharedThetaObservationEnv):
    """Expose delta-eta actions while preserving the core fail-closed lifecycle."""

    def __init__(self, core_environment: Any, mapping: FrozenSemanticMapping) -> None:
        super().__init__()
        self._initialize_observation(core_environment)
        self.mapping = mapping
        self.eta = (0.0,) * len(EXPECTED_ETA_ORDER)
        self.previous_action = (0.0,) * len(EXPECTED_ETA_ORDER)
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(len(EXPECTED_ETA_ORDER),), dtype=np.float32
        )

    def reset(
        self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        del options
        super().reset(seed=seed)
        self.eta = (0.0,) * len(EXPECTED_ETA_ORDER)
        self.previous_action = (0.0,) * len(EXPECTED_ETA_ORDER)
        self._reset_context()
        if hasattr(self.core.adapter, "set_semantic_state"):
            self.core.adapter.set_semantic_state(self.eta, self.previous_action)
        observation, info = self.core.reset(seed=seed)
        result = dict(info)
        result.update({"eta": self.eta, "previous_delta_eta": self.previous_action,
                       "previous_applied_delta_normalized_theta": self.previous_applied_delta,
                       "mapping_version": self.mapping.mapping_version,
                       "mapping_sha256": self.mapping.mapping_sha256})
        return self._observation(observation), result

    def step(self, action: Any):
        values = np.asarray(action, dtype=np.float64)
        if values.shape != (len(EXPECTED_ETA_ORDER),) or not np.isfinite(values).all():
            raise SemanticActionError("SAC action must be a finite five-vector")
        delta_eta = tuple(float(value) for value in values)
        current = self.core.adapter.current_theta()
        mapped = self.mapping.map_action(current, self.eta, delta_eta)
        self.eta = mapped.eta_after
        self.previous_action = mapped.delta_eta
        if hasattr(self.core.adapter, "set_semantic_state"):
            self.core.adapter.set_semantic_state(self.eta, self.previous_action)
        observation, reward, terminated, truncated, info = self.core.step(mapped.theta_candidate)
        self.previous_applied_delta = _applied_delta(
            self.mapping, mapped.normalized_theta_before, info
        )
        result = dict(info)
        result.update({
            "eta_before": mapped.eta_before, "delta_eta": mapped.delta_eta,
            "eta_after": mapped.eta_after,
            "delta_normalized_theta": mapped.delta_normalized_theta,
            "semantic_theta_candidate": dict(mapped.theta_candidate),
            "semantic_eta_clipped": mapped.clipped_eta,
            "semantic_theta_clipped": mapped.clipped_theta,
            "mapping_version": self.mapping.mapping_version,
            "mapping_sha256": self.mapping.mapping_sha256,
            "previous_applied_delta_normalized_theta": self.previous_applied_delta,
        })
        return self._observation(observation), float(reward), bool(terminated), bool(truncated), result


class DirectThetaGymEnv(_SharedThetaObservationEnv):
    """Expose nine direct normalized-theta deltas through the same T05 lifecycle."""

    def __init__(self, core_environment: Any, mapping: DirectThetaMapping) -> None:
        super().__init__()
        self._initialize_observation(core_environment)
        self.mapping = mapping
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(len(EXPECTED_THETA_ORDER),), dtype=np.float32
        )

    def reset(
        self, *, seed: Optional[int] = None, options: Optional[Dict[str, Any]] = None
    ) -> Tuple[np.ndarray, Dict[str, Any]]:
        del options
        super().reset(seed=seed)
        self._reset_context()
        observation, info = self.core.reset(seed=seed)
        result = dict(info)
        result.update({
            "previous_applied_delta_normalized_theta": self.previous_applied_delta,
            "direct_theta_contract_version": self.mapping.contract_version,
            "direct_theta_contract_sha256": self.mapping.contract_sha256,
        })
        return self._observation(observation), result

    def step(self, action: Any):
        values = np.asarray(action, dtype=np.float64)
        if (values.shape != (len(EXPECTED_THETA_ORDER),) or
                not np.isfinite(values).all()):
            raise DirectThetaActionError("SAC action must be a finite nine-vector")
        current = self.core.adapter.current_theta()
        mapped = self.mapping.map_action(current, tuple(float(value) for value in values))
        observation, reward, terminated, truncated, info = self.core.step(mapped.theta_candidate)
        self.previous_applied_delta = _applied_delta(
            self.mapping, mapped.normalized_theta_before, info
        )
        result = dict(info)
        result.update({
            "delta_normalized_theta": mapped.delta_normalized_theta,
            "direct_theta_candidate": dict(mapped.theta_candidate),
            "direct_theta_clipped": mapped.clipped_theta,
            "previous_applied_delta_normalized_theta": self.previous_applied_delta,
            "direct_theta_contract_version": self.mapping.contract_version,
            "direct_theta_contract_sha256": self.mapping.contract_sha256,
        })
        return self._observation(observation), float(reward), bool(terminated), bool(truncated), result


class ResidualSemanticEtaGymEnv(_SharedThetaObservationEnv):
    """Expose an absolute bounded eta target around a fixed tuned theta anchor."""

    def __init__(self, core_environment: Any, mapping: ResidualSemanticMapping) -> None:
        super().__init__()
        self._initialize_observation(core_environment)
        self.mapping = mapping
        self.previous_action = (0.0,) * len(EXPECTED_ETA_ORDER)
        self._held_action = self.previous_action
        self._held_risk_scale = mapping.minimum_risk_scale
        self._step_index = 0
        self.action_space = gym.spaces.Box(
            low=-1.0, high=1.0, shape=(len(EXPECTED_ETA_ORDER),), dtype=np.float32)

    def _risk_scale(self) -> float:
        frame = self.core.history.frames[-1]
        features = frame.named_features
        clearance = float(features.get("footprint_clearance", 0.0))
        ttc = float(features.get("approximate_ttc", 0.0))
        clearance_scale = min(1.0, max(0.0, (clearance - 0.20) / 0.80))
        ttc_scale = min(1.0, max(0.0, ttc / 2.0))
        return min(clearance_scale, ttc_scale)

    def reset(self, *, seed: Optional[int] = None,
              options: Optional[Dict[str, Any]] = None):
        del options
        super().reset(seed=seed)
        self.previous_action = (0.0,) * len(EXPECTED_ETA_ORDER)
        self._held_action = self.previous_action
        self._held_risk_scale = self.mapping.minimum_risk_scale
        self._step_index = 0
        self._reset_context()
        observation, info = self.core.reset(seed=seed)
        result = dict(info)
        result.update({
            "residual_eta_target": self.previous_action,
            "residual_anchor_theta": dict(self.mapping.anchor_theta),
            "mapping_version": self.mapping.mapping_version,
            "mapping_sha256": self.mapping.mapping_sha256,
        })
        return self._observation(observation), result

    def step(self, action: Any):
        values = np.asarray(action, dtype=np.float64)
        if values.shape != (len(EXPECTED_ETA_ORDER),) or not np.isfinite(values).all():
            raise SemanticActionError("residual SAC action must be a finite five-vector")
        raw_action = tuple(float(value) for value in values)
        if self._step_index % self.mapping.decision_hold_steps == 0:
            alpha = self.mapping.action_ema_alpha
            self._held_action = tuple(
                previous + alpha * (target - previous)
                for previous, target in zip(self._held_action, raw_action))
            self._held_risk_scale = self._risk_scale()
        self._step_index += 1
        mapped = self.mapping.map_action(self._held_action, self._held_risk_scale)
        current_before = self.mapping.normalize_theta(self.core.adapter.current_theta())
        self.previous_action = mapped.eta_target
        observation, reward, terminated, truncated, info = self.core.step(mapped.theta_candidate)
        self.previous_applied_delta = _applied_delta(self.mapping, current_before, info)
        result = dict(info)
        result.update({
            "residual_eta_target": mapped.eta_target,
            "residual_risk_scale": mapped.risk_scale,
            "residual_raw_action": raw_action,
            "residual_decision_held": (
                (self._step_index - 1) % self.mapping.decision_hold_steps != 0),
            "residual_normalized_theta": mapped.residual_normalized_theta,
            "residual_theta_candidate": dict(mapped.theta_candidate),
            "residual_eta_clipped": mapped.clipped_eta,
            "residual_theta_clipped": mapped.clipped_theta,
            "mapping_version": self.mapping.mapping_version,
            "mapping_sha256": self.mapping.mapping_sha256,
            "previous_applied_delta_normalized_theta": self.previous_applied_delta,
        })
        return self._observation(observation), float(reward), bool(terminated), bool(truncated), result
