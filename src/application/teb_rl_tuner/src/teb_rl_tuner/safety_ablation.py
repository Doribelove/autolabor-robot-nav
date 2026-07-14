"""Gazebo-only safety ablation adapters for the frozen T11 experiment matrix."""

from typing import Any, Mapping, Optional

from .fallback_policy import ConservativeFallbackPolicy, FallbackDecision
from .safety_margin_filter import SafetyMarginFilter, SafetyMode
from .training_environment import SafeParameterDecision


SAFETY_ABLATIONS = ("FullSafety", "ProjectionOnly", "NoSafety", "NoFallback")


class NoFallbackSafetyAdapter:
    """Keep the margin machine and WARNING filter, but omit emergency theta fallback.

    EMERGENCY/FAULT still requests a stop. This simulation-only ablation removes
    the conservative parameter substitution, not the independent stop boundary.
    """

    def __init__(
        self, safety_filter: SafetyMarginFilter,
        warning_policy: ConservativeFallbackPolicy,
    ) -> None:
        self.safety_filter = safety_filter
        self.warning_policy = warning_policy
        self.last_decision = None
        self.last_fallback: Optional[FallbackDecision] = None

    def reset(self, seed: Optional[int] = None) -> None:
        del seed
        self.safety_filter.reset()
        self.last_decision = None
        self.last_fallback = None

    def filter(
        self, projected: Mapping[str, float], current: Mapping[str, float],
        frame: Any, now: float,
    ) -> SafeParameterDecision:
        features = frame.named_features
        health = {name: True for name in
                  ("sensor", "tf", "localization", "parameter_interface", "planner")}
        decision = self.safety_filter.update(
            features["footprint_clearance"], abs(features["linear_velocity"]), now, health
        )
        self.warning_policy.confirm_applied_safe(current)
        if decision.mode == SafetyMode.WARNING:
            filtered = self.warning_policy.decide(decision.mode, projected, True)
            theta = filtered.theta
            reasons = tuple(decision.reasons) + tuple(filtered.reasons)
        else:
            theta = dict(projected)
            reasons = tuple(decision.reasons)
        request_stop = decision.mode in (SafetyMode.EMERGENCY, SafetyMode.FAULT)
        self.last_fallback = FallbackDecision(
            dict(theta), False, request_stop, request_stop,
            ("ablation:no_conservative_parameter_fallback",) if request_stop else (),
        )
        return SafeParameterDecision(theta, request_stop, reasons + self.last_fallback.reasons)
