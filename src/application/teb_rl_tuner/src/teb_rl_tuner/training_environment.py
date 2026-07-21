"""Long-running, dependency-free training environment orchestration.

The environment deliberately keeps ROS and Gazebo behind a duck-typed adapter.
This makes the lifecycle stress-testable without starting ROS, while preserving
Gymnasium's ``reset``/``step`` return convention.
"""

from dataclasses import dataclass, field
import math
import random
from typing import Any, Dict, Iterable, Mapping, Optional, Sequence, Tuple

from .episode_state_machine import EpisodePhase, EpisodeStateMachine
from .parameter_projection import ParameterProjector, ProjectionResult
from .reward_cost import (
    FeedbackSample,
    RewardCostResult,
    RewardWeights,
    WindowEvents,
    calculate_reward_and_cost,
)
from .state_builder import HistoryWindow, ScanAngularMetadata, StateBuilder, StateFrame
from .timing import ActivationTimeoutError, ActivationTracker, ros_time


class TrainingEnvironmentError(RuntimeError):
    """Configuration or lifecycle error in the training environment."""


@dataclass(frozen=True)
class EnvironmentConfig:
    history_length: int = 4
    activation_timeout_s: float = 1.0
    reward_window_s: float = 1.0
    max_steps: int = 1000
    max_ros_duration_s: float = 1000.0
    warning_distance: float = 0.5

    def __post_init__(self) -> None:
        if isinstance(self.history_length, bool) or not isinstance(self.history_length, int):
            raise TrainingEnvironmentError("history_length must be an integer")
        if isinstance(self.max_steps, bool) or not isinstance(self.max_steps, int):
            raise TrainingEnvironmentError("max_steps must be an integer")
        if self.history_length <= 0 or self.max_steps <= 0:
            raise TrainingEnvironmentError("history_length and max_steps must be positive")
        for name in (
            "activation_timeout_s", "reward_window_s", "max_ros_duration_s",
            "warning_distance",
        ):
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(float(value)) or value <= 0.0):
                raise TrainingEnvironmentError("{} must be finite and positive".format(name))


@dataclass(frozen=True)
class ObservationInput:
    """Raw synchronized inputs consumed by :class:`StateBuilder`."""

    stamps: Mapping[str, object]
    ranges: Sequence[object]
    range_min: float
    range_max: float
    features: Mapping[str, object]
    validity: Mapping[str, bool] = field(default_factory=dict)
    scan_metadata: Optional[ScanAngularMetadata] = None


@dataclass(frozen=True)
class ParameterWriteReceipt:
    t_request: float
    t_ack: float
    applied_theta: Mapping[str, float]


@dataclass(frozen=True)
class ActivationPoll:
    """Planner observations made while waiting for a new config to activate."""

    plans: Sequence[Tuple[float, bool]]
    now: float


@dataclass(frozen=True)
class FeedbackWindow:
    samples: Iterable[FeedbackSample]
    events: WindowEvents
    theta_delta_normalized: Sequence[object]
    terminal_reason: str = ""


@dataclass(frozen=True)
class SafeParameterDecision:
    """Output expected from an optional safety/fallback adapter."""

    theta: Mapping[str, float]
    request_stop: bool = False
    reasons: Tuple[str, ...] = ()


class FixedPolicy:
    """Return the same full theta vector for every observation."""

    def __init__(self, theta: Mapping[str, float]) -> None:
        self._theta = dict(theta)

    def act(self, observation: Sequence[float]) -> Dict[str, float]:
        del observation
        return dict(self._theta)


class RandomSmallPolicy:
    """Uniform, deterministic-with-seed perturbations around a baseline."""

    def __init__(
        self, baseline: Mapping[str, float], maximum_delta: Mapping[str, float], seed: int = 0
    ) -> None:
        if set(baseline) != set(maximum_delta):
            raise TrainingEnvironmentError("baseline and maximum_delta keys must match")
        self._baseline = dict(baseline)
        self._delta = dict(maximum_delta)
        if any(not math.isfinite(float(v)) or float(v) < 0.0 for v in self._delta.values()):
            raise TrainingEnvironmentError("maximum_delta values must be finite and non-negative")
        self._random = random.Random(seed)

    def reset(self, seed: Optional[int] = None) -> None:
        if seed is not None:
            self._random.seed(seed)

    def act(self, observation: Sequence[float]) -> Dict[str, float]:
        del observation
        return {
            name: float(value) + self._random.uniform(-self._delta[name], self._delta[name])
            for name, value in self._baseline.items()
        }


class TrainingEnvironment:
    """Coordinate state, safe parameter writes, attribution and episodes.

    ``adapter`` must implement ``reset(seed)``, ``read_observation()``,
    ``current_theta()``, ``capture_parameter_snapshot()``,
    ``write_parameters(theta, config_seq)``, ``poll_activation(config_seq,
    t_ack, timeout_s)``, and ``collect_feedback(t_active, t_window_end)``.
    Fail-closed recovery uses optional ``request_stop(reason)`` and
    ``restore_parameter_snapshot(snapshot)`` hooks.

    An optional ``safety_adapter`` implements ``filter(projected_theta,
    current_theta, frame, now)`` and returns :class:`SafeParameterDecision`.
    """

    def __init__(
        self,
        adapter: Any,
        state_builder: StateBuilder,
        projector: ParameterProjector,
        reward_weights: RewardWeights,
        config: EnvironmentConfig = EnvironmentConfig(),
        safety_adapter: Optional[Any] = None,
    ) -> None:
        self.adapter = adapter
        self.state_builder = state_builder
        self.projector = projector
        self.reward_weights = reward_weights
        self.config = config
        self.safety_adapter = safety_adapter
        self.history = HistoryWindow(config.history_length)
        self.episode = EpisodeStateMachine()
        self._next_episode = 0
        self._config_seq = 0
        self._attempted_steps = 0
        self._stored_steps = 0
        self._episode_start_ros: Optional[float] = None
        self._snapshot: Any = None
        self._last_observation: Tuple[float, ...] = ()

    @property
    def config_seq(self) -> int:
        return self._config_seq

    @property
    def current_outcome(self) -> Any:
        """Immutable current outcome, or ``None`` while the episode is active."""

        return self.episode.outcome

    @property
    def episode_statistics(self) -> Dict[str, Any]:
        outcome = self.episode.outcome
        return {
            "episode_id": self.episode.episode_id,
            "attempted_steps": self._attempted_steps,
            "stored_steps": self._stored_steps,
            "dropped_steps": self._attempted_steps - self._stored_steps,
            "config_seq_last": self._config_seq,
            "termination_reason": "" if outcome is None else outcome.termination_reason,
            "terminated": False if outcome is None else outcome.terminated,
            "truncated": False if outcome is None else outcome.truncated,
        }

    def _frame(self) -> StateFrame:
        raw = self.adapter.read_observation()
        if not isinstance(raw, ObservationInput):
            raise TrainingEnvironmentError("adapter returned an invalid observation")
        return self.state_builder.build(
            raw.stamps,
            raw.ranges,
            raw.range_min,
            raw.range_max,
            raw.features,
            raw.validity,
            scan_metadata=raw.scan_metadata,
        )

    def _restore_snapshot(self) -> None:
        if self._snapshot is not None and hasattr(self.adapter, "restore_parameter_snapshot"):
            self.adapter.restore_parameter_snapshot(self._snapshot)
        self._snapshot = None

    def _reset_episode_boundary(self, seed: Optional[int]) -> None:
        """Restore parameters and reset the simulator as one ordered boundary.

        ROS adapters may need to keep the previous navigation goal quiescent
        across both the parameter restore and the scene reset.  The atomic
        adapter hook prevents the generic environment from restoring parameters
        in one call and dispatching the next goal in an unrelated call.
        """

        if (self._snapshot is not None and
                hasattr(self.adapter, "reset_with_parameter_snapshot")):
            self.adapter.reset_with_parameter_snapshot(self._snapshot, seed)
            self._snapshot = None
            return
        self._restore_snapshot()
        self.adapter.reset(seed)

    def reset(self, seed: Optional[int] = None) -> Tuple[Tuple[float, ...], Dict[str, Any]]:
        """Reset simulator epoch and completely isolate episode-local state."""

        if self.episode.phase == EpisodePhase.RUNNING:
            self.episode.finish("operator_stop")
        self._reset_episode_boundary(seed)
        if self.safety_adapter is not None and hasattr(self.safety_adapter, "reset"):
            self.safety_adapter.reset(seed)
        self.state_builder.reset_time_epoch()
        self.history.clear()
        self._attempted_steps = 0
        self._stored_steps = 0
        self._config_seq = 0
        self._snapshot = self.adapter.capture_parameter_snapshot()
        for _ in range(self.config.history_length):
            self.history.append(self._frame())
        observation = self.history.stacked()
        self._last_observation = observation
        # Priming belongs to reset, not to the agent's episode time budget.
        self._episode_start_ros = self.history.frames[-1].timestamp
        episode_id = "episode-{:06d}".format(self._next_episode)
        self._next_episode += 1
        self.episode.start(episode_id)
        return observation, {
            "episode_id": episode_id,
            "config_seq": self._config_seq,
            "history_ready": True,
        }

    @staticmethod
    def action_from(policy: Any, observation: Sequence[float]) -> Mapping[str, float]:
        """Accept either a callable policy or an object exposing ``act``."""

        if hasattr(policy, "act"):
            return policy.act(observation)
        if callable(policy):
            return policy(observation)
        raise TrainingEnvironmentError("policy must be callable or expose act")

    def _finish(self, reason: str) -> Tuple[bool, bool]:
        outcome = self.episode.finish(reason)
        return outcome.terminated, outcome.truncated

    def _adapter_terminal_reason(self) -> str:
        probe = getattr(self.adapter, "terminal_reason", None)
        if not callable(probe):
            return ""
        reason = str(probe() or "")
        return reason if reason in ("goal", "planner_failure", "collision") else ""

    def _fail_closed(self, exc: BaseException) -> Tuple[Tuple[float, ...], float, bool, bool, Dict[str, Any]]:
        reason = "{}: {}".format(type(exc).__name__, exc)
        stop_error = ""
        restore_error = ""
        try:
            if hasattr(self.adapter, "request_stop"):
                self.adapter.request_stop(reason)
        except Exception as stop_exc:  # recovery must continue even if stop transport fails
            stop_error = "{}: {}".format(type(stop_exc).__name__, stop_exc)
        try:
            self._restore_snapshot()
        except Exception as restore_exc:
            restore_error = "{}: {}".format(type(restore_exc).__name__, restore_exc)
        if self.episode.phase == EpisodePhase.RUNNING:
            terminated, truncated = self._finish("interface_fault")
        else:
            terminated, truncated = True, False
        return self._last_observation, 0.0, terminated, truncated, {
            "config_seq": self._config_seq,
            "transition_stored": False,
            "transition_drop_reason": "fail_closed",
            "error": reason,
            "stop_error": stop_error,
            "restore_error": restore_error,
            "termination_reason": "interface_fault",
        }

    def step(
        self, action: Mapping[str, object]
    ) -> Tuple[Tuple[float, ...], float, bool, bool, Dict[str, Any]]:
        if self.episode.phase != EpisodePhase.RUNNING:
            raise TrainingEnvironmentError("step requires reset and a running episode")
        self._attempted_steps += 1
        self._config_seq += 1
        seq = self._config_seq
        try:
            current = self.adapter.current_theta()
            projected = self.projector.project(action, current)
            frame = self.history.frames[-1]
            safe = SafeParameterDecision(projected.projected)
            if self.safety_adapter is not None:
                safe = self.safety_adapter.filter(
                    projected.projected, current, frame, frame.timestamp
                )
                if not isinstance(safe, SafeParameterDecision):
                    raise TrainingEnvironmentError("safety adapter returned an invalid decision")
            if hasattr(self.adapter, "set_safety_state"):
                decision = getattr(self.safety_adapter, "last_decision", None)
                fallback = getattr(self.safety_adapter, "last_fallback", None)
                mode = getattr(getattr(decision, "mode", None), "value", "NORMAL")
                self.adapter.set_safety_state(
                    mode,
                    bool(getattr(fallback, "use_fallback", False)),
                    mode in ("EMERGENCY", "FAULT"),
                )
            if safe.request_stop:
                if hasattr(self.adapter, "request_stop"):
                    self.adapter.request_stop("safety adapter requested emergency stop")
                self._restore_snapshot()
                terminated, truncated = self._finish("emergency_stop")
                return self._last_observation, 0.0, terminated, truncated, {
                    "episode_id": self.episode.episode_id,
                    "config_seq": seq,
                    "attempted_steps": self._attempted_steps,
                    "stored_steps": self._stored_steps,
                    "candidate_theta": dict(projected.candidate),
                    "projected_theta": dict(projected.projected),
                    "safe_theta": dict(safe.theta),
                    "applied_theta": dict(current),
                    "projection_modified": projected.intervened,
                    "projection_reasons": projected.reasons,
                    "safety_reasons": safe.reasons,
                    "transition_stored": False,
                    "transition_drop_reason": "safety_emergency_stop",
                    "termination_reason": "emergency_stop",
                    "reward_fields": {},
                }
            t_decision = frame.timestamp
            receipt = self.adapter.write_parameters(safe.theta, seq)
            if not isinstance(receipt, ParameterWriteReceipt):
                raise TrainingEnvironmentError("adapter returned an invalid write receipt")
            tracker = ActivationTracker(
                seq, t_decision, receipt.t_request, receipt.t_ack,
                self.config.activation_timeout_s,
            )
            poll = self.adapter.poll_activation(seq, receipt.t_ack, self.config.activation_timeout_s)
            if not isinstance(poll, ActivationPoll):
                raise TrainingEnvironmentError("adapter returned an invalid activation poll")
            for stamp, complete in poll.plans:
                tracker.observe_local_plan(stamp, complete)
            tracker.check_timeout(poll.now)
            if tracker.active is None:
                raise ActivationTimeoutError("activation poll ended without a complete plan")
            t_active = tracker.active.t_active
            t_end = t_active + self.config.reward_window_s
            feedback = self.adapter.collect_feedback(t_active, t_end)
            if not isinstance(feedback, FeedbackWindow):
                raise TrainingEnvironmentError("adapter returned an invalid feedback window")
            events = WindowEvents(
                planner_failure_count=feedback.events.planner_failure_count,
                parameter_violation_count=(
                    feedback.events.parameter_violation_count + int(projected.intervened)
                ),
                collision=feedback.events.collision,
                goal=feedback.events.goal,
            )
            reward_result = calculate_reward_and_cost(
                feedback.samples, t_active, t_end, feedback.theta_delta_normalized,
                events, self.reward_weights, self.config.warning_distance,
            )
            next_frame = self._frame()
            self.history.append(next_frame)
            observation = self.history.stacked()
            self._last_observation = observation
            self._stored_steps += 1

            reason = feedback.terminal_reason
            if not reason and self._attempted_steps >= self.config.max_steps:
                reason = "timeout"
            if (not reason and self._episode_start_ros is not None and
                    next_frame.timestamp - self._episode_start_ros >=
                    self.config.max_ros_duration_s):
                reason = "timeout"
            terminated = truncated = False
            if reason:
                terminated, truncated = self._finish(reason)
            info = self._info(
                projected, safe, receipt, reward_result, t_decision, t_active, t_end
            )
            info.update({
                "terminated_reason": reason,
                "termination_reason": reason,
                "transition_stored": True,
                "transition_drop_reason": "",
            })
            return observation, reward_result.reward_total, terminated, truncated, info
        except ActivationTimeoutError as exc:
            if hasattr(self.adapter, "mark_activation_timeout"):
                self.adapter.mark_activation_timeout(seq)
            # move_base stops publishing local plans immediately after a goal.
            # If the action server has already confirmed a terminal state, that
            # terminal outcome takes precedence over the missing post-write plan.
            # Snapshot restoration is deliberately deferred to the next atomic
            # reset boundary. Restoring inside this exception handler can race a
            # planner oscillation/recovery thread that has not gone quiet yet.
            terminal_reason = self._adapter_terminal_reason()
            if terminal_reason:
                terminated, truncated = self._finish(terminal_reason)
                return self._last_observation, 0.0, terminated, truncated, {
                    "config_seq": seq,
                    "transition_stored": False,
                    "transition_drop_reason": "terminal_before_parameter_activation",
                    "termination_reason": terminal_reason,
                    "error": str(exc),
                    "snapshot_restore_deferred": True,
                }
            try:
                if hasattr(self.adapter, "request_stop"):
                    self.adapter.request_stop(str(exc))
            except Exception:
                pass
            terminated, truncated = self._finish("interface_fault")
            return self._last_observation, 0.0, terminated, truncated, {
                "config_seq": seq,
                "transition_stored": False,
                "transition_drop_reason": ActivationTimeoutError.code,
                "termination_reason": "interface_fault",
                "error": str(exc),
                "snapshot_restore_deferred": True,
            }
        except Exception as exc:
            return self._fail_closed(exc)

    def _info(
        self,
        projection: ProjectionResult,
        safe: SafeParameterDecision,
        receipt: ParameterWriteReceipt,
        reward: RewardCostResult,
        t_decision: float,
        t_active: float,
        t_end: float,
    ) -> Dict[str, Any]:
        result = {
            "episode_id": self.episode.episode_id,
            "config_seq": self._config_seq,
            "attempted_steps": self._attempted_steps,
            "stored_steps": self._stored_steps,
            "episode_statistics": self.episode_statistics,
            "candidate_theta": dict(projection.candidate),
            "projected_theta": dict(projection.projected),
            "safe_theta": dict(safe.theta),
            "applied_theta": dict(receipt.applied_theta),
            "projection_modified": projection.intervened,
            "projection_reasons": projection.reasons,
            "safety_reasons": safe.reasons,
            "safety_mode": getattr(self.adapter, "safety_mode", "NORMAL"),
            "fallback_active": bool(getattr(self.adapter, "fallback_active", False)),
            "t_decision": t_decision,
            "t_request": receipt.t_request,
            "t_ack": receipt.t_ack,
            "t_active": t_active,
            "t_window_end": t_end,
            "reward_components": dict(reward.components),
            "costs": dict(reward.costs),
            "reward_fields": reward.step_fields(),
        }
        result.update(reward.step_fields())
        return result
