"""Atomic conservative fallback selection for the complete TEB theta vector."""

from dataclasses import dataclass
from typing import Any, Dict, Mapping, Optional, Tuple

from .parameter_projection import CandidateRejected, validate_theta
from .safety_margin_filter import SafetyMode


class FallbackUnavailable(RuntimeError):
    """No complete confirmed-safe vector exists; caller must keep writes stopped."""


@dataclass(frozen=True)
class FallbackDecision:
    theta: Dict[str, float]
    use_fallback: bool
    stop_learning_writes: bool
    request_stop: bool
    reasons: Tuple[str, ...]


class ConservativeFallbackPolicy:
    """Return only complete nine-parameter snapshots suitable for atomic writing."""

    def __init__(self, conservative_theta: Mapping[str, Any]) -> None:
        try:
            self._conservative = validate_theta(conservative_theta, "conservative")
        except CandidateRejected as exc:
            raise FallbackUnavailable("invalid conservative configuration: {}".format(exc))
        self._last_confirmed_safe = None  # type: Optional[Dict[str, float]]

    def confirm_applied_safe(self, theta: Mapping[str, Any]) -> None:
        """Record a full vector only after external writer ack/readback confirmation."""

        try:
            self._last_confirmed_safe = validate_theta(theta, "confirmed_safe")
        except CandidateRejected as exc:
            raise FallbackUnavailable("cannot confirm incomplete/unsafe snapshot: {}".format(exc))

    @property
    def last_confirmed_safe(self) -> Optional[Dict[str, float]]:
        return None if self._last_confirmed_safe is None else dict(self._last_confirmed_safe)

    def decide(
        self,
        mode: SafetyMode,
        projected_theta: Optional[Mapping[str, Any]],
        parameter_interface_healthy: bool,
    ) -> FallbackDecision:
        try:
            mode_value = mode if isinstance(mode, SafetyMode) else SafetyMode(mode)
        except (TypeError, ValueError):
            raise FallbackUnavailable("unknown safety mode")
        if parameter_interface_healthy is not True:
            if self._last_confirmed_safe is None:
                raise FallbackUnavailable("parameter interface failed before any confirmed-safe snapshot")
            return FallbackDecision(dict(self._last_confirmed_safe), True, True, True,
                                    ("parameter_interface:use_last_confirmed_safe",))
        if mode_value in (SafetyMode.EMERGENCY, SafetyMode.FAULT):
            return FallbackDecision(dict(self._conservative), True, True, True,
                                    ("safety_mode:{}".format(mode_value.value),))
        if projected_theta is None:
            raise FallbackUnavailable("projected theta is required outside emergency/fault")
        try:
            theta = validate_theta(projected_theta, "projected")
        except CandidateRejected as exc:
            raise FallbackUnavailable("invalid projected theta: {}".format(exc))
        if mode_value == SafetyMode.WARNING:
            if self._last_confirmed_safe is None:
                return FallbackDecision(dict(self._conservative), True, False, False,
                                        ("safety_mode:WARNING:no_confirmed_baseline",))
            previous = self._last_confirmed_safe
            # Known TEB monotonic directions: do not increase speed and do not
            # reduce the requested obstacle clearance/conservatism in WARNING.
            theta["max_vel_x"] = min(theta["max_vel_x"], previous["max_vel_x"])
            theta["max_vel_theta"] = min(theta["max_vel_theta"], previous["max_vel_theta"])
            theta["min_obstacle_dist"] = max(
                theta["min_obstacle_dist"], previous["min_obstacle_dist"])
            theta["inflation_dist"] = max(theta["inflation_dist"], previous["inflation_dist"])
            theta["weight_obstacle"] = max(theta["weight_obstacle"], previous["weight_obstacle"])
            return FallbackDecision(theta, False, False, False,
                                    ("safety_mode:WARNING:conservative_filter",))
        return FallbackDecision(theta, False, False, False, ())
