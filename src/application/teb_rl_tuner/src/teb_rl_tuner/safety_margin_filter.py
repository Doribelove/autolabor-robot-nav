"""Calibrated engineering safety-margin filter and auditable mode machine."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Mapping, Optional, Tuple


class SafetyConfigurationError(ValueError):
    pass


class SafetyMode(str, Enum):
    NORMAL = "NORMAL"
    WARNING = "WARNING"
    EMERGENCY = "EMERGENCY"
    FAULT = "FAULT"


@dataclass(frozen=True)
class SafetyMarginConfig:
    a_brake_lower: float
    tau_total_upper: float
    d_margin: float
    warning_margin: float
    emergency_margin: float
    hysteresis_margin: float
    recovery_healthy_s: float
    emergency_distance_cap: Optional[float] = None
    emergency_confirmation_s: float = 0.0


@dataclass(frozen=True)
class SafetyDecision:
    mode: SafetyMode
    previous_mode: SafetyMode
    d_safe: Optional[float]
    margin: Optional[float]
    risk_score: float
    reasons: Tuple[str, ...]
    changed: bool


def _calibrated_nonnegative(value: Any, name: str, strictly_positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise SafetyConfigurationError("{} must be a calibrated finite number".format(name))
    result = float(value)
    if result < 0 or (strictly_positive and result <= 0):
        raise SafetyConfigurationError("{} has an invalid calibrated value".format(name))
    return result


class SafetyMarginFilter:
    """Four-state safety machine with hysteresis and continuous health recovery."""

    def __init__(self, config: SafetyMarginConfig) -> None:
        if not isinstance(config, SafetyMarginConfig):
            raise SafetyConfigurationError("complete SafetyMarginConfig is required")
        self.a_brake_lower = _calibrated_nonnegative(config.a_brake_lower, "a_brake_lower", True)
        self.tau_total_upper = _calibrated_nonnegative(config.tau_total_upper, "tau_total_upper")
        self.d_margin = _calibrated_nonnegative(config.d_margin, "d_margin")
        self.warning_margin = _calibrated_nonnegative(config.warning_margin, "warning_margin")
        self.emergency_margin = _calibrated_nonnegative(config.emergency_margin, "emergency_margin")
        self.hysteresis_margin = _calibrated_nonnegative(config.hysteresis_margin, "hysteresis_margin", True)
        self.recovery_healthy_s = _calibrated_nonnegative(config.recovery_healthy_s, "recovery_healthy_s", True)
        self.emergency_distance_cap = (
            None if config.emergency_distance_cap is None else
            _calibrated_nonnegative(
                config.emergency_distance_cap, "emergency_distance_cap", True)
        )
        self.emergency_confirmation_s = _calibrated_nonnegative(
            config.emergency_confirmation_s, "emergency_confirmation_s"
        )
        if self.emergency_margin > self.warning_margin:
            raise SafetyConfigurationError("emergency_margin must not exceed warning_margin")
        self.mode = SafetyMode.NORMAL
        self._healthy_since = None  # type: Optional[float]
        self._hazard_since = None  # type: Optional[float]

    def reset(self) -> None:
        """Clear episode-local hysteresis without changing frozen calibration."""

        self.mode = SafetyMode.NORMAL
        self._healthy_since = None
        self._hazard_since = None

    def safe_distance(self, speed: float) -> float:
        speed_value = _calibrated_nonnegative(abs(speed), "speed")
        return (speed_value ** 2 / (2.0 * self.a_brake_lower) +
                speed_value * self.tau_total_upper + self.d_margin)

    @staticmethod
    def _invalid_health(health: Mapping[str, Any]) -> Tuple[str, ...]:
        required = ("sensor", "tf", "localization", "parameter_interface", "planner")
        if not isinstance(health, Mapping):
            return ("health:not_mapping",)
        reasons = []
        for name in required:
            if health.get(name) is not True:
                reasons.append("health:{}:invalid".format(name))
        return tuple(reasons)

    def update(
        self,
        obstacle_distance: Any,
        speed: Any,
        now: Any,
        health: Mapping[str, Any],
        fault_reset_requested: bool = False,
        emergency_obstacle_distance: Any = None,
    ) -> SafetyDecision:
        previous = self.mode
        reasons = list(self._invalid_health(health))
        numeric_valid = True
        for name, value in (("obstacle_distance", obstacle_distance), ("speed", speed), ("now", now)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
                reasons.append("{}:invalid".format(name))
                numeric_valid = False
        if numeric_valid and (obstacle_distance < 0 or now < 0):
            reasons.append("measurement:negative")
            numeric_valid = False
        if reasons:
            self.mode = SafetyMode.FAULT
            self._healthy_since = None
            return SafetyDecision(self.mode, previous, None, None, 1.0, tuple(reasons), self.mode != previous)

        now_value = float(now)
        d_safe = self.safe_distance(float(speed))
        margin = float(obstacle_distance) - d_safe
        # Bounded engineering risk indicator, not a formal CBF quantity.
        scale = max(self.warning_margin + self.hysteresis_margin, 1e-12)
        risk_score = min(1.0, max(0.0, (self.warning_margin - margin) / scale))

        emergency_distance = (
            float(obstacle_distance) if emergency_obstacle_distance is None else
            emergency_obstacle_distance
        )
        if (isinstance(emergency_distance, bool) or
                not isinstance(emergency_distance, (int, float)) or
                not math.isfinite(float(emergency_distance)) or
                float(emergency_distance) < 0.0):
            self.mode = SafetyMode.FAULT
            self._healthy_since = None
            return SafetyDecision(
                self.mode, previous, d_safe, margin, 1.0,
                ("emergency_obstacle_distance:invalid",), self.mode != previous)
        emergency_candidate = margin <= self.emergency_margin
        if self.emergency_distance_cap is not None:
            emergency_candidate = (
                emergency_candidate and
                float(emergency_distance) <= self.emergency_distance_cap
            )
        if emergency_candidate:
            if self._hazard_since is None:
                self._hazard_since = now_value
            hazard_duration = now_value - self._hazard_since
        else:
            self._hazard_since = None
            hazard_duration = 0.0
        hazardous = (
            emergency_candidate and
            hazard_duration >= self.emergency_confirmation_s
        )
        warning = margin <= self.warning_margin
        if hazardous:
            target = SafetyMode.EMERGENCY
            reasons.append("margin:emergency")
            self._healthy_since = None
        elif warning:
            target = SafetyMode.WARNING
            reasons.append("margin:warning")
            if emergency_candidate:
                reasons.append("emergency:confirmation_pending")
            self._healthy_since = None
        else:
            target = SafetyMode.NORMAL

        recovering = self.mode != SafetyMode.NORMAL
        recovery_threshold = self.warning_margin + self.hysteresis_margin
        recovery_safe = margin > recovery_threshold
        if recovering:
            if not recovery_safe:
                self._healthy_since = None
            elif self._healthy_since is None:
                self._healthy_since = now_value
                reasons.append("recovery:healthy_timer_started")
            healthy_duration = 0.0 if self._healthy_since is None else now_value - self._healthy_since
            can_recover = recovery_safe and healthy_duration >= self.recovery_healthy_s
            if self.mode == SafetyMode.FAULT and not fault_reset_requested:
                target = SafetyMode.FAULT
                reasons.append("fault:manual_reset_required")
            elif not can_recover and target == SafetyMode.NORMAL:
                target = self.mode
                reasons.append("recovery:hysteresis_or_health_time")

        # Escalation is immediate; recovery never skips more than one safety level.
        rank = {SafetyMode.NORMAL: 0, SafetyMode.WARNING: 1,
                SafetyMode.EMERGENCY: 2, SafetyMode.FAULT: 3}
        if rank[target] < rank[self.mode] - 1:
            target = SafetyMode(rank_to_name(rank[self.mode] - 1))
        self.mode = target
        return SafetyDecision(self.mode, previous, d_safe, margin, risk_score,
                              tuple(reasons), self.mode != previous)


def rank_to_name(rank: int) -> str:
    return (SafetyMode.NORMAL.value, SafetyMode.WARNING.value,
            SafetyMode.EMERGENCY.value, SafetyMode.FAULT.value)[rank]
