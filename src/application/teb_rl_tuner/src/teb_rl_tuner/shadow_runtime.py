"""Read-only T12 policy supervision for replay and real-time shadow mode."""

from dataclasses import dataclass
import math
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from .config import EXPECTED_THETA_ORDER
from .fallback_policy import ConservativeFallbackPolicy
from .parameter_projection import ParameterProjector, ProjectionResult, validate_theta
from .safety_margin_filter import SafetyDecision, SafetyMarginFilter, SafetyMode


class ShadowRuntimeError(ValueError):
    pass


@dataclass(frozen=True)
class ShadowRuntimeConfig:
    ema_alpha: float
    ood_warning_score: float
    ood_fallback_score: float

    def __post_init__(self) -> None:
        values = (self.ema_alpha, self.ood_warning_score, self.ood_fallback_score)
        if any(isinstance(value, bool) or not isinstance(value, (int, float))
               or not math.isfinite(float(value)) for value in values):
            raise ShadowRuntimeError("shadow runtime thresholds must be finite numbers")
        if not 0.0 < self.ema_alpha <= 1.0:
            raise ShadowRuntimeError("ema_alpha must be in (0, 1]")
        if self.ood_warning_score < 0.0:
            raise ShadowRuntimeError("ood_warning_score must be non-negative")
        if self.ood_fallback_score <= self.ood_warning_score:
            raise ShadowRuntimeError("ood_fallback_score must exceed warning score")


@dataclass(frozen=True)
class ShadowDecision:
    candidate_theta: Dict[str, float]
    smoothed_theta: Dict[str, float]
    projected_theta: Dict[str, float]
    recommended_theta: Dict[str, float]
    safety: SafetyDecision
    ood_score: float
    projection_reasons: Tuple[str, ...]
    reasons: Tuple[str, ...]
    write_allowed: bool = False
    motion_allowed: bool = False


class FeatureEnvelope:
    """Simple explainable training-distribution envelope.

    A score of 0 is inside all reference ranges. A score of 1 means one feature
    lies one full reference-span beyond its range. Missing/non-finite features
    fail closed with an infinite score.
    """

    def __init__(self, ranges: Mapping[str, Sequence[float]]) -> None:
        checked = {}
        for name, bounds in ranges.items():
            if not isinstance(name, str) or not name:
                raise ShadowRuntimeError("feature envelope names must be non-empty")
            if not isinstance(bounds, Sequence) or len(bounds) != 2:
                raise ShadowRuntimeError("feature envelope bounds must have length two")
            low, high = float(bounds[0]), float(bounds[1])
            if not math.isfinite(low) or not math.isfinite(high) or low > high:
                raise ShadowRuntimeError("invalid feature envelope for {}".format(name))
            checked[name] = (low, high)
        if not checked:
            raise ShadowRuntimeError("feature envelope cannot be empty")
        self.ranges = checked

    def score(self, features: Mapping[str, Any]) -> Tuple[float, Tuple[str, ...]]:
        scores, reasons = [], []
        for name, (low, high) in self.ranges.items():
            value = features.get(name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(float(value)):
                return float("inf"), ("ood:{}:missing_or_invalid".format(name),)
            number = float(value)
            span = max(high - low, 1e-9)
            if number < low:
                scores.append((low - number) / span)
                reasons.append("ood:{}:below".format(name))
            elif number > high:
                scores.append((number - high) / span)
                reasons.append("ood:{}:above".format(name))
        return (max(scores) if scores else 0.0), tuple(reasons)


class ThetaEmaSmoother:
    """EMA prefilter used before the non-negotiable box/rate projector."""

    def __init__(self, alpha: float) -> None:
        if isinstance(alpha, bool) or not isinstance(alpha, (int, float)) \
                or not math.isfinite(float(alpha)) or not 0.0 < float(alpha) <= 1.0:
            raise ShadowRuntimeError("EMA alpha must be in (0, 1]")
        self.alpha = float(alpha)
        self._previous = None  # type: Optional[Dict[str, float]]

    def reset(self, theta: Optional[Mapping[str, Any]] = None) -> None:
        self._previous = None if theta is None else validate_theta(theta, "ema_reset")

    def update(
        self, candidate: Mapping[str, Any], current: Mapping[str, Any]
    ) -> Dict[str, float]:
        target = validate_theta(candidate, "ema_candidate")
        baseline = (
            validate_theta(current, "ema_current")
            if self._previous is None else self._previous
        )
        result = {
            name: baseline[name] + self.alpha * (target[name] - baseline[name])
            for name in EXPECTED_THETA_ORDER
        }
        self._previous = dict(result)
        return result


class ShadowRuntime:
    """Compose smoothing, projection, OOD gating and safety without side effects."""

    def __init__(
        self,
        config: ShadowRuntimeConfig,
        projector: ParameterProjector,
        safety_filter: SafetyMarginFilter,
        fallback_policy: ConservativeFallbackPolicy,
        feature_envelope: FeatureEnvelope,
    ) -> None:
        self.config = config
        self.projector = projector
        self.safety_filter = safety_filter
        self.fallback_policy = fallback_policy
        self.feature_envelope = feature_envelope
        self.smoother = ThetaEmaSmoother(config.ema_alpha)

    def reset(self, theta: Optional[Mapping[str, Any]] = None) -> None:
        self.safety_filter.reset()
        self.smoother.reset(theta)

    def evaluate(
        self,
        candidate_theta: Mapping[str, Any],
        current_theta: Mapping[str, Any],
        features: Mapping[str, Any],
        health: Mapping[str, Any],
        now: float,
    ) -> ShadowDecision:
        candidate = validate_theta(candidate_theta, "shadow_candidate")
        current = validate_theta(current_theta, "shadow_current")
        smoothed = self.smoother.update(candidate, current)
        projected = self.projector.project(smoothed, current)
        ood_score, ood_reasons = self.feature_envelope.score(features)
        decision = self.safety_filter.update(
            features.get("footprint_clearance"),
            abs(float(features.get("linear_velocity", 0.0))),
            now,
            health,
        )
        self.fallback_policy.confirm_applied_safe(current)
        fallback_mode = decision.mode
        reasons = list(ood_reasons) + list(decision.reasons)
        if ood_score >= self.config.ood_fallback_score:
            fallback_mode = SafetyMode.FAULT
            reasons.append("ood:fallback")
        elif ood_score >= self.config.ood_warning_score \
                and fallback_mode == SafetyMode.NORMAL:
            fallback_mode = SafetyMode.WARNING
            reasons.append("ood:warning")
        fallback = self.fallback_policy.decide(fallback_mode, projected.projected, True)
        reasons.extend(fallback.reasons)
        # T12 is intentionally incapable of authorizing motion or writes. The
        # recommendation can only be logged and compared with the active TEB.
        return ShadowDecision(
            candidate_theta=dict(candidate),
            smoothed_theta=dict(smoothed),
            projected_theta=dict(projected.projected),
            recommended_theta=dict(fallback.theta),
            safety=decision,
            ood_score=ood_score,
            projection_reasons=tuple(projected.reasons),
            reasons=tuple(reasons),
        )

