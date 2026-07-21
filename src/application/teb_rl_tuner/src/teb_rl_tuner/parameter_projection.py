"""Fail-closed projection of the nine online TEB tuning parameters.

The module intentionally contains no ROS dependencies.  Calibration values are
supplied by the caller; absent bounds or rate limits are an error, rather than a
reason to silently invent defaults.
"""

from dataclasses import dataclass
import math
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from .config import EXPECTED_THETA_ORDER


class ProjectionConfigurationError(ValueError):
    """Raised when projection cannot be configured without guessed values."""


class CandidateRejected(ValueError):
    """Raised when a candidate is malformed or requests an offline-only change."""

    def __init__(self, reasons: Sequence[str]):
        self.reasons = tuple(reasons)
        super().__init__("; ".join(self.reasons))


@dataclass(frozen=True)
class ParameterLimit:
    physical_min: float
    physical_max: float
    max_delta_per_rl_step: float
    online_support: bool = True


@dataclass(frozen=True)
class ProjectionResult:
    candidate: Dict[str, float]
    projected: Dict[str, float]
    reasons: Tuple[str, ...]

    @property
    def intervened(self) -> bool:
        return bool(self.reasons)


def _finite_number(value: Any, context: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CandidateRejected(("{}:not_numeric".format(context),))
    number = float(value)
    if not math.isfinite(number):
        raise CandidateRejected(("{}:non_finite".format(context),))
    return number


def validate_theta(theta: Mapping[str, Any], context: str = "theta") -> Dict[str, float]:
    """Require an exact, finite mapping in the stable nine-parameter order."""

    if not isinstance(theta, Mapping):
        raise CandidateRejected(("{}:not_mapping".format(context),))
    expected = set(EXPECTED_THETA_ORDER)
    actual = set(theta.keys())
    reasons = []
    if actual - expected:
        reasons.append("{}:unexpected_keys:{}".format(context, ",".join(sorted(actual - expected))))
    if expected - actual:
        reasons.append("{}:missing_keys:{}".format(context, ",".join(sorted(expected - actual))))
    if reasons:
        raise CandidateRejected(reasons)
    result = {}
    errors = []
    for name in EXPECTED_THETA_ORDER:
        try:
            result[name] = _finite_number(theta[name], "{}:{}".format(context, name))
        except CandidateRejected as exc:
            errors.extend(exc.reasons)
    if errors:
        raise CandidateRejected(errors)
    return result


class ParameterProjector:
    """Apply calibrated box, rate and Ackermann/TEB coupling constraints."""

    def __init__(
        self,
        limits: Mapping[str, ParameterLimit],
        min_turning_radius: Optional[float] = None,
    ) -> None:
        if not isinstance(limits, Mapping) or set(limits.keys()) != set(EXPECTED_THETA_ORDER):
            raise ProjectionConfigurationError("limits must contain exactly the nine theta parameters")
        checked = {}
        for name in EXPECTED_THETA_ORDER:
            limit = limits[name]
            if not isinstance(limit, ParameterLimit):
                raise ProjectionConfigurationError("{} limit is missing or invalid".format(name))
            values = (limit.physical_min, limit.physical_max, limit.max_delta_per_rl_step)
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in values):
                raise ProjectionConfigurationError("{} limits must be calibrated numbers".format(name))
            if not all(math.isfinite(float(v)) for v in values):
                raise ProjectionConfigurationError("{} limits must be finite".format(name))
            if limit.physical_min > limit.physical_max or limit.max_delta_per_rl_step < 0:
                raise ProjectionConfigurationError("{} limits are inconsistent".format(name))
            if not isinstance(limit.online_support, bool):
                raise ProjectionConfigurationError("{} online_support must be boolean".format(name))
            checked[name] = limit
        if min_turning_radius is not None:
            if (isinstance(min_turning_radius, bool) or
                    not isinstance(min_turning_radius, (int, float)) or
                    not math.isfinite(float(min_turning_radius)) or min_turning_radius <= 0):
                raise ProjectionConfigurationError("min_turning_radius must be a calibrated positive value")
        self._limits = checked
        self._min_turning_radius = None if min_turning_radius is None else float(min_turning_radius)

    @classmethod
    def from_contract_candidates(
        cls, candidates: Sequence[Mapping[str, Any]], min_turning_radius: Optional[float] = None
    ) -> "ParameterProjector":
        """Build from contract-style rows, failing when any calibration is null."""

        if not isinstance(candidates, Sequence) or len(candidates) != len(EXPECTED_THETA_ORDER):
            raise ProjectionConfigurationError("theta_candidates must contain exactly nine rows")
        limits = {}
        for expected, row in zip(EXPECTED_THETA_ORDER, candidates):
            if not isinstance(row, Mapping) or row.get("name") != expected:
                raise ProjectionConfigurationError("theta_candidates order/name mismatch")
            try:
                limits[expected] = ParameterLimit(
                    physical_min=row["physical_min"],
                    physical_max=row["physical_max"],
                    max_delta_per_rl_step=row["max_delta_per_rl_step"],
                    online_support=row["online_support"],
                )
            except KeyError as exc:
                raise ProjectionConfigurationError("missing {} for {}".format(exc.args[0], expected))
        return cls(limits, min_turning_radius=min_turning_radius)

    def project(self, candidate: Mapping[str, Any], current: Mapping[str, Any]) -> ProjectionResult:
        candidate_values = validate_theta(candidate, "candidate")
        current_values = validate_theta(current, "current")
        projected = {}
        reasons = []
        rejected = []
        for name in EXPECTED_THETA_ORDER:
            requested = candidate_values[name]
            previous = current_values[name]
            limit = self._limits[name]
            if not limit.online_support and requested != previous:
                rejected.append("{}:online_update_unsupported".format(name))
                continue
            bounded = min(limit.physical_max, max(limit.physical_min, requested))
            if bounded != requested:
                reasons.append("{}:physical_bound".format(name))
            low = previous - limit.max_delta_per_rl_step
            high = previous + limit.max_delta_per_rl_step
            rate_limited = min(high, max(low, bounded))
            if rate_limited != bounded:
                reasons.append("{}:rate_limit".format(name))
            projected[name] = rate_limited
        if rejected:
            raise CandidateRejected(rejected)

        # TEB requires the inflation distance to cover its minimum obstacle distance.
        if projected["inflation_dist"] < projected["min_obstacle_dist"]:
            projected["inflation_dist"] = projected["min_obstacle_dist"]
            reasons.append("inflation_dist:below_min_obstacle_dist")
        if self._min_turning_radius is not None:
            yaw_limit = abs(projected["max_vel_x"]) / self._min_turning_radius
            if projected["max_vel_theta"] > yaw_limit:
                projected["max_vel_theta"] = yaw_limit
                reasons.append("max_vel_theta:ackermann_turning_radius")

        # A coupling correction must itself remain inside calibrated bounds and rate.
        for name in EXPECTED_THETA_ORDER:
            limit = self._limits[name]
            if not (limit.physical_min <= projected[name] <= limit.physical_max):
                raise CandidateRejected(("{}:coupling_outside_physical_bound".format(name),))
            if (abs(projected[name] - current_values[name]) >
                    limit.max_delta_per_rl_step + 1e-12):
                raise CandidateRejected(("{}:coupling_outside_rate_limit".format(name),))
        return ProjectionResult(candidate_values, projected, tuple(reasons))
