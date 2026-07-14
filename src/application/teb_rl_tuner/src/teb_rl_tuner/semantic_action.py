"""Frozen Semantic-Eta action mapping for T09.

The policy action is a five-dimensional delta-eta in [-1, 1].  The frozen
matrix maps it to a delta in normalized theta space.  Physical candidates are
still passed through T05 projection and safety before any ROS write.
"""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

import yaml

from .config import EXPECTED_ETA_ORDER, EXPECTED_THETA_ORDER
from .parameter_projection import CandidateRejected, validate_theta


class SemanticActionError(ValueError):
    pass


def _vector(values: Sequence[Any], length: int, label: str) -> Tuple[float, ...]:
    if not isinstance(values, Sequence) or isinstance(values, (str, bytes)) or len(values) != length:
        raise SemanticActionError("{} must contain {} values".format(label, length))
    result = []
    for index, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise SemanticActionError("{}[{}] must be numeric".format(label, index))
        number = float(value)
        if not math.isfinite(number):
            raise SemanticActionError("{}[{}] must be finite".format(label, index))
        result.append(number)
    return tuple(result)


@dataclass(frozen=True)
class SemanticActionResult:
    eta_before: Tuple[float, ...]
    delta_eta: Tuple[float, ...]
    eta_after: Tuple[float, ...]
    normalized_theta_before: Tuple[float, ...]
    delta_normalized_theta: Tuple[float, ...]
    normalized_theta_candidate: Tuple[float, ...]
    theta_candidate: Mapping[str, float]
    clipped_eta: bool
    clipped_theta: bool


class FrozenSemanticMapping:
    def __init__(
        self,
        matrix: Sequence[Sequence[Any]],
        bounds: Mapping[str, Sequence[Any]],
        mapping_version: str,
        mapping_sha256: str,
        normalized_delta_limits: Sequence[Any] = None,
    ) -> None:
        if len(matrix) != len(EXPECTED_THETA_ORDER):
            raise SemanticActionError("mapping must have nine theta rows")
        checked_matrix = []
        for row in matrix:
            checked_matrix.append(_vector(row, len(EXPECTED_ETA_ORDER), "mapping row"))
        if set(bounds) != set(EXPECTED_THETA_ORDER):
            raise SemanticActionError("bounds must contain exactly the frozen theta order")
        checked_bounds = {}
        for name in EXPECTED_THETA_ORDER:
            pair = _vector(bounds[name], 2, "bounds.{}".format(name))
            if pair[1] <= pair[0]:
                raise SemanticActionError("bounds.{} must have positive width".format(name))
            checked_bounds[name] = pair
        if not mapping_version or not isinstance(mapping_sha256, str) or len(mapping_sha256) != 64:
            raise SemanticActionError("frozen mapping identity is invalid")
        self.matrix = tuple(checked_matrix)
        self.bounds = checked_bounds
        self.mapping_version = str(mapping_version)
        self.mapping_sha256 = mapping_sha256
        self.normalized_delta_limits = (
            None if normalized_delta_limits is None else
            _vector(normalized_delta_limits, len(EXPECTED_THETA_ORDER),
                    "normalized_delta_limits")
        )
        if (self.normalized_delta_limits is not None and
                any(value <= 0.0 or value > 2.0 for value in self.normalized_delta_limits)):
            raise SemanticActionError("normalized delta limits must be within (0, 2]")

    @classmethod
    def from_files(
        cls, mapping_path: Any, safety_path: Any, executable_rate_scaling: bool = False
    ) -> "FrozenSemanticMapping":
        mapping = yaml.safe_load(Path(mapping_path).read_text(encoding="utf-8"))
        safety = yaml.safe_load(Path(safety_path).read_text(encoding="utf-8"))
        if not isinstance(mapping, dict) or mapping.get("status") != "frozen":
            raise SemanticActionError("A_TEB must be frozen")
        if tuple(mapping.get("eta_order", ())) != EXPECTED_ETA_ORDER:
            raise SemanticActionError("A_TEB eta order mismatch")
        if tuple(mapping.get("theta_order", ())) != EXPECTED_THETA_ORDER:
            raise SemanticActionError("A_TEB theta order mismatch")
        if mapping.get("normalization") != "normalized_theta_to_minus_one_plus_one":
            raise SemanticActionError("unsupported A_TEB normalization")
        if not isinstance(safety, dict) or safety.get("real_vehicle_use_forbidden") is not True:
            raise SemanticActionError("T09 mapping requires the simulation-only safety contract")
        limits = None
        version = mapping["mapping_version"]
        if executable_rate_scaling:
            limits = tuple(
                2.0 * float(safety["max_delta_per_step"][name]) /
                (float(safety["theta_bounds"][name][1]) -
                 float(safety["theta_bounds"][name][0]))
                for name in EXPECTED_THETA_ORDER
            )
            version = version + "+executable_rate_v1"
        return cls(mapping["matrix"], safety["theta_bounds"], version,
                   mapping["sha256"], limits)

    def normalize_theta(self, theta: Mapping[str, Any]) -> Tuple[float, ...]:
        values = validate_theta(theta, "theta")
        return tuple(
            2.0 * (values[name] - self.bounds[name][0]) /
            (self.bounds[name][1] - self.bounds[name][0]) - 1.0
            for name in EXPECTED_THETA_ORDER
        )

    def denormalize_theta(self, normalized: Sequence[Any]) -> Dict[str, float]:
        values = _vector(normalized, len(EXPECTED_THETA_ORDER), "normalized_theta")
        return {
            name: self.bounds[name][0] + 0.5 * (values[index] + 1.0) *
                  (self.bounds[name][1] - self.bounds[name][0])
            for index, name in enumerate(EXPECTED_THETA_ORDER)
        }

    def map_action(
        self,
        current_theta: Mapping[str, Any],
        eta_before: Sequence[Any],
        delta_eta: Sequence[Any],
    ) -> SemanticActionResult:
        try:
            current = validate_theta(current_theta, "current_theta")
        except CandidateRejected as exc:
            raise SemanticActionError(str(exc))
        before = _vector(eta_before, len(EXPECTED_ETA_ORDER), "eta_before")
        action = _vector(delta_eta, len(EXPECTED_ETA_ORDER), "delta_eta")
        if any(value < -1.0 or value > 1.0 for value in action):
            raise SemanticActionError("delta_eta must be within [-1, 1]")
        eta_unclipped = tuple(left + right for left, right in zip(before, action))
        eta_after = tuple(min(1.0, max(-1.0, value)) for value in eta_unclipped)
        delta_z = tuple(sum(row[column] * action[column]
                            for column in range(len(EXPECTED_ETA_ORDER)))
                        for row in self.matrix)
        if self.normalized_delta_limits is not None:
            delta_z = tuple(
                value * limit / max(sum(abs(item) for item in row), 1.0)
                for value, limit, row in zip(
                    delta_z, self.normalized_delta_limits, self.matrix
                )
            )
        z_before = self.normalize_theta(current)
        z_unclipped = tuple(left + right for left, right in zip(z_before, delta_z))
        z_candidate = tuple(min(1.0, max(-1.0, value)) for value in z_unclipped)
        return SemanticActionResult(
            before, action, eta_after, z_before, delta_z, z_candidate,
            self.denormalize_theta(z_candidate),
            eta_after != eta_unclipped, z_candidate != z_unclipped,
        )


@dataclass(frozen=True)
class ResidualSemanticActionResult:
    eta_target: Tuple[float, ...]
    risk_scale: float
    normalized_anchor: Tuple[float, ...]
    residual_normalized_theta: Tuple[float, ...]
    normalized_theta_candidate: Tuple[float, ...]
    theta_candidate: Mapping[str, float]
    clipped_eta: bool
    clipped_theta: bool


class ResidualSemanticMapping:
    """Bounded, non-accumulating Semantic-Eta residual around a tuned anchor."""

    def __init__(self, mapping: FrozenSemanticMapping,
                 anchor_theta: Mapping[str, Any],
                 normalized_radius: Sequence[Any],
                 minimum_risk_scale: float = 0.2,
                 action_ema_alpha: float = 1.0,
                 decision_hold_steps: int = 1) -> None:
        self.mapping = mapping
        try:
            self.anchor_theta = validate_theta(anchor_theta, "residual_anchor")
        except CandidateRejected as exc:
            raise SemanticActionError(str(exc))
        for name in EXPECTED_THETA_ORDER:
            low, high = mapping.bounds[name]
            if not low <= self.anchor_theta[name] <= high:
                raise SemanticActionError("residual anchor is outside bounds: {}".format(name))
        self.normalized_radius = _vector(
            normalized_radius, len(EXPECTED_THETA_ORDER), "normalized_radius")
        if any(value <= 0.0 or value > 1.0 for value in self.normalized_radius):
            raise SemanticActionError("normalized residual radii must be within (0, 1]")
        if (isinstance(minimum_risk_scale, bool) or
                not isinstance(minimum_risk_scale, (int, float)) or
                not math.isfinite(float(minimum_risk_scale)) or
                not 0.0 <= float(minimum_risk_scale) <= 1.0):
            raise SemanticActionError("minimum risk scale must be within [0, 1]")
        self.minimum_risk_scale = float(minimum_risk_scale)
        if not 0.0 < float(action_ema_alpha) <= 1.0:
            raise SemanticActionError("action EMA alpha must be within (0, 1]")
        if (isinstance(decision_hold_steps, bool) or
                not isinstance(decision_hold_steps, int) or decision_hold_steps <= 0):
            raise SemanticActionError("decision hold steps must be positive")
        self.action_ema_alpha = float(action_ema_alpha)
        self.decision_hold_steps = int(decision_hold_steps)
        self.bounds = mapping.bounds
        self.mapping_version = mapping.mapping_version + "+residual_anchor_v1"
        self.mapping_sha256 = mapping.mapping_sha256

    @classmethod
    def from_files(cls, mapping_path: Any, safety_path: Any, residual_path: Any):
        mapping = FrozenSemanticMapping.from_files(mapping_path, safety_path)
        state = yaml.safe_load(Path(residual_path).read_text(encoding="utf-8"))
        if (state.get("mode") != "residual_semantic_eta" or
                not isinstance(state.get("training_enabled"), bool)):
            raise SemanticActionError("residual config identity/training flag is invalid")
        return cls(mapping, state["anchor_theta"], state["normalized_residual_radius"],
                   state["minimum_risk_scale"], state.get("action_ema_alpha", 1.0),
                   state.get("decision_hold_steps", 1))

    def normalize_theta(self, theta: Mapping[str, Any]) -> Tuple[float, ...]:
        return self.mapping.normalize_theta(theta)

    def map_action(self, eta_target: Sequence[Any], risk_scale: Any = 1.0
                   ) -> ResidualSemanticActionResult:
        eta = _vector(eta_target, len(EXPECTED_ETA_ORDER), "eta_target")
        clipped_eta = tuple(min(1.0, max(-1.0, value)) for value in eta)
        if (isinstance(risk_scale, bool) or not isinstance(risk_scale, (int, float)) or
                not math.isfinite(float(risk_scale))):
            raise SemanticActionError("risk scale must be finite")
        scale = min(1.0, max(self.minimum_risk_scale, float(risk_scale)))
        anchor = self.mapping.normalize_theta(self.anchor_theta)
        residual = []
        for row, radius in zip(self.mapping.matrix, self.normalized_radius):
            direction = sum(value * weight for value, weight in zip(clipped_eta, row))
            direction /= max(1.0, sum(abs(weight) for weight in row))
            residual.append(scale * radius * direction)
        candidate_raw = tuple(left + right for left, right in zip(anchor, residual))
        candidate = tuple(min(1.0, max(-1.0, value)) for value in candidate_raw)
        return ResidualSemanticActionResult(
            clipped_eta, scale, anchor, tuple(residual), candidate,
            self.mapping.denormalize_theta(candidate), clipped_eta != eta,
            candidate != candidate_raw,
        )
