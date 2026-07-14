"""Direct normalized-theta action mapping for the T10 SAC control group."""

from dataclasses import dataclass
import hashlib
import math
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence, Tuple

import yaml

from .config import EXPECTED_THETA_ORDER
from .parameter_projection import CandidateRejected, validate_theta


class DirectThetaActionError(ValueError):
    pass


def _vector(values: Sequence[Any], length: int, label: str) -> Tuple[float, ...]:
    if (not isinstance(values, Sequence) or isinstance(values, (str, bytes)) or
            len(values) != length):
        raise DirectThetaActionError("{} must contain {} values".format(label, length))
    result = []
    for index, value in enumerate(values):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise DirectThetaActionError("{}[{}] must be numeric".format(label, index))
        number = float(value)
        if not math.isfinite(number):
            raise DirectThetaActionError("{}[{}] must be finite".format(label, index))
        result.append(number)
    return tuple(result)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


@dataclass(frozen=True)
class DirectThetaActionResult:
    delta_normalized_theta: Tuple[float, ...]
    normalized_theta_before: Tuple[float, ...]
    normalized_theta_candidate: Tuple[float, ...]
    theta_candidate: Mapping[str, float]
    clipped_theta: bool


class DirectThetaMapping:
    """Map a nine-dimensional SAC action directly into normalized theta space."""

    def __init__(
        self, bounds: Mapping[str, Sequence[Any]], contract_version: str,
        contract_sha256: str, normalized_delta_limits: Sequence[Any] = None,
    ) -> None:
        if set(bounds) != set(EXPECTED_THETA_ORDER):
            raise DirectThetaActionError("bounds must contain exactly the frozen theta order")
        checked = {}
        for name in EXPECTED_THETA_ORDER:
            pair = _vector(bounds[name], 2, "bounds.{}".format(name))
            if pair[1] <= pair[0]:
                raise DirectThetaActionError("bounds.{} must have positive width".format(name))
            checked[name] = pair
        if not contract_version or len(str(contract_sha256)) != 64:
            raise DirectThetaActionError("direct-theta contract identity is invalid")
        self.bounds = checked
        self.contract_version = str(contract_version)
        self.contract_sha256 = str(contract_sha256)
        self.normalized_delta_limits = (
            None if normalized_delta_limits is None else
            _vector(normalized_delta_limits, len(EXPECTED_THETA_ORDER),
                    "normalized_delta_limits")
        )
        if (self.normalized_delta_limits is not None and
                any(value <= 0.0 or value > 2.0 for value in self.normalized_delta_limits)):
            raise DirectThetaActionError("normalized delta limits must be within (0, 2]")

    @classmethod
    def from_file(cls, safety_path: Any, executable_rate_scaling: bool = False) -> "DirectThetaMapping":
        path = Path(safety_path)
        safety = yaml.safe_load(path.read_text(encoding="utf-8"))
        if not isinstance(safety, dict) or safety.get("real_vehicle_use_forbidden") is not True:
            raise DirectThetaActionError("T10 mapping requires simulation-only safety bounds")
        limits = None
        version = "direct_theta_v1"
        if executable_rate_scaling:
            limits = tuple(
                2.0 * float(safety["max_delta_per_step"][name]) /
                (float(safety["theta_bounds"][name][1]) -
                 float(safety["theta_bounds"][name][0]))
                for name in EXPECTED_THETA_ORDER
            )
            version = "direct_theta_executable_rate_v1"
        return cls(safety["theta_bounds"], version, _sha256(path), limits)

    def normalize_theta(self, theta: Mapping[str, Any]) -> Tuple[float, ...]:
        try:
            values = validate_theta(theta, "theta")
        except CandidateRejected as exc:
            raise DirectThetaActionError(str(exc))
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
        self, current_theta: Mapping[str, Any], delta_normalized_theta: Sequence[Any]
    ) -> DirectThetaActionResult:
        action = _vector(
            delta_normalized_theta, len(EXPECTED_THETA_ORDER), "delta_normalized_theta"
        )
        if any(value < -1.0 or value > 1.0 for value in action):
            raise DirectThetaActionError("delta_normalized_theta must be within [-1, 1]")
        scaled = action if self.normalized_delta_limits is None else tuple(
            value * limit for value, limit in zip(action, self.normalized_delta_limits)
        )
        before = self.normalize_theta(current_theta)
        unclipped = tuple(left + right for left, right in zip(before, scaled))
        candidate = tuple(min(1.0, max(-1.0, value)) for value in unclipped)
        return DirectThetaActionResult(
            scaled, before, candidate, self.denormalize_theta(candidate),
            candidate != unclipped,
        )
