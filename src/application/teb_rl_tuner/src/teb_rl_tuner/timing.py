"""ROS-time synchronization and parameter-activation attribution primitives.

This module deliberately has no rospy dependency.  Callers pass ROS timestamps as
seconds, which makes simulation resets and bag replay behavior straightforward to
unit test.
"""

import math
from dataclasses import dataclass
from typing import Dict, Mapping, Optional, Sequence, Tuple


class TimingError(ValueError):
    """Base error for invalid ROS-time or synchronization state."""


class NonMonotonicTimeError(TimingError):
    """Raised when a stream moves backwards or repeats a timestamp."""


class SynchronizationError(TimingError):
    """Raised when observation timestamps cannot form one synchronized frame."""


class ActivationTimeoutError(TimingError):
    """Raised when no complete planner cycle activates an acknowledged config."""

    code = "parameter_activation_timeout"


def ros_time(value: object, name: str = "timestamp") -> float:
    """Return a finite, non-negative ROS timestamp in seconds."""

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TimingError("{} must be a numeric ROS timestamp".format(name))
    stamp = float(value)
    if not math.isfinite(stamp) or stamp < 0.0:
        raise TimingError("{} must be finite and non-negative".format(name))
    return stamp


class MonotonicRosTime:
    """Validate strictly increasing timestamps independently for named streams."""

    def __init__(self) -> None:
        self._latest: Dict[str, float] = {}

    def observe(self, stream: str, stamp: object) -> float:
        if not isinstance(stream, str) or not stream:
            raise TimingError("stream must be a non-empty string")
        current = ros_time(stamp, "{}.stamp".format(stream))
        previous = self._latest.get(stream)
        if previous is not None and current <= previous:
            raise NonMonotonicTimeError(
                "{} timestamp {} is not later than {}".format(stream, current, previous)
            )
        self._latest[stream] = current
        return current

    def latest(self, stream: str) -> Optional[float]:
        return self._latest.get(stream)

    def reset(self) -> None:
        """Start a new epoch after an explicit simulator/bag reset."""

        self._latest.clear()


@dataclass(frozen=True)
class SynchronizedStamps:
    """A validated observation timestamp set."""

    observation_time: float
    minimum_time: float
    maximum_time: float
    skew_s: float
    stamps: Mapping[str, float]


def synchronize_stamps(
    stamps: Mapping[str, object], required_streams: Sequence[str], max_skew_s: float
) -> SynchronizedStamps:
    """Validate presence and bounded skew; use the newest stamp as frame time."""

    if isinstance(max_skew_s, bool) or not isinstance(max_skew_s, (int, float)):
        raise SynchronizationError("max_skew_s must be numeric")
    tolerance = float(max_skew_s)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise SynchronizationError("max_skew_s must be finite and non-negative")
    if not required_streams:
        raise SynchronizationError("at least one required stream is needed")
    missing = [name for name in required_streams if name not in stamps]
    if missing:
        raise SynchronizationError("missing streams: {}".format(", ".join(missing)))
    checked = {name: ros_time(stamps[name], "{}.stamp".format(name)) for name in required_streams}
    minimum = min(checked.values())
    maximum = max(checked.values())
    skew = maximum - minimum
    if skew > tolerance:
        raise SynchronizationError(
            "observation skew {:.9f}s exceeds {:.9f}s".format(skew, tolerance)
        )
    return SynchronizedStamps(maximum, minimum, maximum, skew, checked)


@dataclass(frozen=True)
class ActivationWindow:
    config_seq: int
    t_decision: float
    t_request: float
    t_ack: float
    t_active: float

    def close(self, t_window_end: object) -> Tuple[float, float]:
        end = ros_time(t_window_end, "t_window_end")
        if end <= self.t_active:
            raise TimingError("t_window_end must be later than t_active")
        return self.t_active, end


class ActivationTracker:
    """Track ``decision -> request -> ack -> complete plan`` in ROS time.

    Only a *complete* local plan strictly newer than the acknowledgement activates
    the configuration.  Timeout uses ROS time as well, so paused simulation cannot
    consume the attribution deadline.
    """

    def __init__(
        self,
        config_seq: int,
        t_decision: object,
        t_request: object,
        t_ack: object,
        timeout_s: float,
    ) -> None:
        if isinstance(config_seq, bool) or not isinstance(config_seq, int) or config_seq < 0:
            raise TimingError("config_seq must be a non-negative integer")
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)):
            raise TimingError("timeout_s must be numeric")
        self.config_seq = config_seq
        self.t_decision = ros_time(t_decision, "t_decision")
        self.t_request = ros_time(t_request, "t_request")
        self.t_ack = ros_time(t_ack, "t_ack")
        self.timeout_s = float(timeout_s)
        if not math.isfinite(self.timeout_s) or self.timeout_s <= 0.0:
            raise TimingError("timeout_s must be finite and positive")
        if not self.t_decision <= self.t_request <= self.t_ack:
            raise TimingError("timestamps must satisfy t_decision <= t_request <= t_ack")
        self._active: Optional[ActivationWindow] = None

    @property
    def active(self) -> Optional[ActivationWindow]:
        return self._active

    def observe_local_plan(self, stamp: object, complete: bool) -> Optional[ActivationWindow]:
        plan_time = ros_time(stamp, "local_plan.stamp")
        if self._active is not None:
            return self._active
        if complete and plan_time > self.t_ack:
            self._active = ActivationWindow(
                self.config_seq, self.t_decision, self.t_request, self.t_ack, plan_time
            )
        return self._active

    def check_timeout(self, now: object) -> None:
        current = ros_time(now, "now")
        if current < self.t_ack:
            raise NonMonotonicTimeError("now precedes t_ack")
        if self._active is None and current - self.t_ack >= self.timeout_s:
            raise ActivationTimeoutError(
                "config_seq {} did not activate within {:.3f}s".format(
                    self.config_seq, self.timeout_s
                )
            )
