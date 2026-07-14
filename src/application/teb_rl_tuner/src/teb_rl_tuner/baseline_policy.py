"""Frozen non-learning baselines for T08.

The rule policy is deliberately causal: it consumes only the latest frame in
the observation history and its own previous mode.  It never receives a goal
result, future trajectory sample, episode summary, or evaluator statistic.
"""

from dataclasses import dataclass
import math
from typing import Dict, Mapping, Optional, Sequence, Tuple

from .config import EXPECTED_THETA_ORDER
from .training_environment import TrainingEnvironmentError


RULE_MODES = ("efficient", "tracking", "cautious")


def _theta(values: Mapping[str, object], label: str) -> Dict[str, float]:
    if set(values) != set(EXPECTED_THETA_ORDER):
        raise TrainingEnvironmentError("{} must contain exactly the frozen theta keys".format(label))
    result = {}
    for name in EXPECTED_THETA_ORDER:
        value = values[name]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TrainingEnvironmentError("{}.{} must be numeric".format(label, name))
        number = float(value)
        if not math.isfinite(number):
            raise TrainingEnvironmentError("{}.{} must be finite".format(label, name))
        result[name] = number
    return result


@dataclass(frozen=True)
class ObservationLayout:
    sector_count: int
    feature_order: Tuple[str, ...]
    history_length: int

    def __post_init__(self) -> None:
        if self.sector_count <= 0 or self.history_length <= 0:
            raise TrainingEnvironmentError("observation dimensions must be positive")
        if len(set(self.feature_order)) != len(self.feature_order):
            raise TrainingEnvironmentError("feature_order contains duplicates")

    @property
    def frame_size(self) -> int:
        return self.sector_count + len(self.feature_order)

    def latest_features(self, observation: Sequence[object]) -> Dict[str, float]:
        if len(observation) != self.frame_size * self.history_length:
            raise TrainingEnvironmentError("observation does not match the frozen layout")
        start = len(observation) - self.frame_size + self.sector_count
        result = {}
        for index, name in enumerate(self.feature_order):
            value = observation[start + index]
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise TrainingEnvironmentError("observation feature {} is not numeric".format(name))
            number = float(value)
            if not math.isfinite(number):
                raise TrainingEnvironmentError("observation feature {} is not finite".format(name))
            result[name] = number
        return result


@dataclass(frozen=True)
class RuleThresholds:
    cautious_clearance_enter_m: float
    cautious_clearance_exit_m: float
    cautious_ttc_enter_s: float
    cautious_ttc_exit_s: float
    tracking_error_enter_m: float
    tracking_error_exit_m: float

    def __post_init__(self) -> None:
        values = tuple(float(getattr(self, name)) for name in self.__dataclass_fields__)
        if any(not math.isfinite(value) or value <= 0.0 for value in values):
            raise TrainingEnvironmentError("rule thresholds must be finite and positive")
        if self.cautious_clearance_exit_m <= self.cautious_clearance_enter_m:
            raise TrainingEnvironmentError("clearance exit threshold must exceed enter threshold")
        if self.cautious_ttc_exit_s <= self.cautious_ttc_enter_s:
            raise TrainingEnvironmentError("TTC exit threshold must exceed enter threshold")
        if self.tracking_error_exit_m >= self.tracking_error_enter_m:
            raise TrainingEnvironmentError("tracking exit threshold must be below enter threshold")


class CausalRuleTebPolicy:
    """Three-mode, hysteretic Rule-TEB baseline using present observations only."""

    def __init__(
        self,
        profiles: Mapping[str, Mapping[str, object]],
        thresholds: RuleThresholds,
        layout: ObservationLayout,
    ) -> None:
        if set(profiles) != set(RULE_MODES):
            raise TrainingEnvironmentError("rule profiles must be efficient/tracking/cautious")
        self.profiles = {name: _theta(profiles[name], "profiles.{}".format(name))
                         for name in RULE_MODES}
        self.thresholds = thresholds
        self.layout = layout
        required = {"footprint_clearance", "approximate_ttc", "path_cross_track_error"}
        if not required.issubset(layout.feature_order):
            raise TrainingEnvironmentError("rule observation layout lacks required causal features")
        self.mode = "efficient"
        self.decision_count = 0
        self.mode_transitions = 0
        self.last_features: Dict[str, float] = {}

    def reset(self, seed: Optional[int] = None) -> None:
        del seed
        self.mode = "efficient"
        self.decision_count = 0
        self.mode_transitions = 0
        self.last_features = {}

    def _next_mode(self, features: Mapping[str, float]) -> str:
        clearance = features["footprint_clearance"]
        ttc = features["approximate_ttc"]
        error = features["path_cross_track_error"]
        if self.mode == "cautious":
            if (clearance < self.thresholds.cautious_clearance_exit_m or
                    ttc < self.thresholds.cautious_ttc_exit_s):
                return "cautious"
        elif (clearance <= self.thresholds.cautious_clearance_enter_m or
              ttc <= self.thresholds.cautious_ttc_enter_s):
            return "cautious"
        if self.mode == "tracking":
            if error > self.thresholds.tracking_error_exit_m:
                return "tracking"
        elif error >= self.thresholds.tracking_error_enter_m:
            return "tracking"
        return "efficient"

    def act(self, observation: Sequence[object]) -> Dict[str, float]:
        features = self.layout.latest_features(observation)
        next_mode = self._next_mode(features)
        if next_mode != self.mode:
            self.mode_transitions += 1
        self.mode = next_mode
        self.decision_count += 1
        self.last_features = dict(features)
        return dict(self.profiles[self.mode])

