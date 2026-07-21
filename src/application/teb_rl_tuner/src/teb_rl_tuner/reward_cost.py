"""Physical-time reward integration and independently logged CMDP costs."""

import math
from dataclasses import dataclass
from typing import Dict, Iterable, List, Mapping, Sequence


class RewardWindowError(ValueError):
    """Raised when a feedback window cannot be attributed safely."""


def _number(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise RewardWindowError("{} must be numeric".format(name))
    result = float(value)
    if not math.isfinite(result):
        raise RewardWindowError("{} must be finite".format(name))
    return result


@dataclass(frozen=True)
class RewardWeights:
    progress: float = 1.0
    elapsed_time: float = 1.0
    near_obstacle: float = 1.0
    path_error: float = 1.0
    smoothness: float = 1.0
    angular_acceleration: float = 1.0
    planner_failure: float = 1.0
    parameter_adjustment: float = 1.0
    goal_terminal: float = 1.0
    collision_terminal: float = 1.0

    def __post_init__(self) -> None:
        for name, value in self.__dict__.items():
            number = _number(value, "weights.{}".format(name))
            if number < 0.0:
                raise RewardWindowError("reward weights must be non-negative")


@dataclass(frozen=True)
class FeedbackSample:
    stamp: float
    goal_distance: float
    path_error: float
    clearance: float
    linear_acceleration: float = 0.0
    angular_acceleration: float = 0.0
    near_collision: bool = False
    fallback_active: bool = False
    emergency_active: bool = False

    def __post_init__(self) -> None:
        for name in (
            "stamp",
            "goal_distance",
            "path_error",
            "clearance",
            "linear_acceleration",
            "angular_acceleration",
        ):
            _number(getattr(self, name), name)
        if self.stamp < 0.0 or self.goal_distance < 0.0 or self.clearance < 0.0:
            raise RewardWindowError("stamp, goal_distance and clearance must be non-negative")


@dataclass(frozen=True)
class WindowEvents:
    planner_failure_count: int = 0
    parameter_violation_count: int = 0
    collision: bool = False
    goal: bool = False

    def __post_init__(self) -> None:
        if (
            isinstance(self.planner_failure_count, bool)
            or not isinstance(self.planner_failure_count, int)
            or self.planner_failure_count < 0
        ):
            raise RewardWindowError("planner_failure_count must be a non-negative integer")
        if (
            isinstance(self.parameter_violation_count, bool)
            or not isinstance(self.parameter_violation_count, int)
            or self.parameter_violation_count < 0
        ):
            raise RewardWindowError("parameter_violation_count must be a non-negative integer")
        if self.collision and self.goal:
            raise RewardWindowError("goal and collision cannot be simultaneous terminal events")


@dataclass(frozen=True)
class RewardCostResult:
    components: Mapping[str, float]
    costs: Mapping[str, float]
    valid_feedback_duration: float
    goal_distance_start: float
    goal_distance_end: float

    @property
    def reward_total(self) -> float:
        return self.components["reward_total"]

    def step_fields(self) -> Dict[str, float]:
        values = dict(self.components)
        values.update(self.costs)
        values["valid_feedback_duration"] = self.valid_feedback_duration
        values["goal_distance_start"] = self.goal_distance_start
        values["goal_distance_end"] = self.goal_distance_end
        return values


def _trapezoid(samples: Sequence[FeedbackSample], value_fn: object) -> float:
    total = 0.0
    for left, right in zip(samples, samples[1:]):
        duration = right.stamp - left.stamp
        total += 0.5 * duration * (value_fn(left) + value_fn(right))
    return total


def calculate_reward_and_cost(
    samples: Iterable[FeedbackSample],
    t_active: float,
    t_window_end: float,
    theta_delta_normalized: Sequence[object],
    events: WindowEvents,
    weights: RewardWeights,
    warning_distance: float,
) -> RewardCostResult:
    """Calculate every schema component from feedback after activation only."""

    start = _number(t_active, "t_active")
    end = _number(t_window_end, "t_window_end")
    warning = _number(warning_distance, "warning_distance")
    if end <= start:
        raise RewardWindowError("t_window_end must be later than t_active")
    if warning <= 0.0:
        raise RewardWindowError("warning_distance must be positive")
    selected = [sample for sample in samples if start <= sample.stamp <= end]
    if len(selected) < 2:
        raise RewardWindowError("at least two post-activation samples are required")
    if any(right.stamp <= left.stamp for left, right in zip(selected, selected[1:])):
        raise RewardWindowError("feedback timestamps must be strictly increasing")
    duration = selected[-1].stamp - selected[0].stamp
    near_risk = lambda sample: max(0.0, (warning - sample.clearance) / warning) ** 2
    near_integral = _trapezoid(selected, near_risk)
    path_integral = _trapezoid(selected, lambda sample: sample.path_error ** 2)
    smooth_integral = _trapezoid(
        selected,
        lambda sample: sample.linear_acceleration ** 2
        + weights.angular_acceleration * sample.angular_acceleration ** 2,
    )
    near_collision_duration = _trapezoid(
        selected, lambda sample: 1.0 if sample.near_collision else 0.0
    )
    emergency_fallback_duration = _trapezoid(
        selected,
        lambda sample: 1.0 if sample.emergency_active or sample.fallback_active else 0.0,
    )
    adjustment = sum(abs(_number(value, "theta_delta")) for value in theta_delta_normalized)
    terminal = weights.goal_terminal if events.goal else 0.0
    if events.collision:
        terminal -= weights.collision_terminal
    components: Dict[str, float] = {
        "reward_progress": weights.progress
        * (selected[0].goal_distance - selected[-1].goal_distance),
        "reward_time": -weights.elapsed_time * duration,
        "reward_near_obstacle": -weights.near_obstacle * near_integral,
        "reward_path_error": -weights.path_error * path_integral,
        "reward_smoothness": -weights.smoothness * smooth_integral,
        "reward_planner_failure": -weights.planner_failure * events.planner_failure_count,
        "reward_parameter_adjustment": -weights.parameter_adjustment * adjustment,
        "reward_terminal": terminal,
    }
    components["reward_total"] = sum(components.values())
    costs = {
        "cost_collision": 1.0 if events.collision else 0.0,
        "cost_near_collision": near_collision_duration,
        "cost_parameter_violation": float(events.parameter_violation_count),
        "cost_planner_failure": float(events.planner_failure_count),
        "cost_emergency_or_fallback": emergency_fallback_duration,
    }
    return RewardCostResult(
        components,
        costs,
        duration,
        selected[0].goal_distance,
        selected[-1].goal_distance,
    )
