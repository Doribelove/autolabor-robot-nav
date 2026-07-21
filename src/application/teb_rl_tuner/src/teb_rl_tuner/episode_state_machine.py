"""Episode lifecycle with mutually exclusive termination and truncation."""

from dataclasses import dataclass
from enum import Enum
from typing import Optional

from .timing import ActivationTimeoutError, ActivationTracker


class EpisodeError(RuntimeError):
    """Raised for invalid episode lifecycle transitions."""


class EpisodePhase(Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    FINISHED = "FINISHED"


TERMINATION_REASONS = frozenset(
    (
        "goal",
        "collision",
        "planner_failure",
        "sensor_fault",
        "tf_fault",
        "interface_fault",
        "emergency_stop",
    )
)
TRUNCATION_REASONS = frozenset(("timeout", "operator_stop", "infrastructure_fault"))
ALL_REASONS = TERMINATION_REASONS | TRUNCATION_REASONS


@dataclass(frozen=True)
class EpisodeOutcome:
    terminated: bool
    truncated: bool
    termination_reason: str
    success: bool
    collision: bool

    def __post_init__(self) -> None:
        if self.terminated == self.truncated:
            raise EpisodeError("exactly one of terminated/truncated must be true")
        if self.termination_reason not in ALL_REASONS:
            raise EpisodeError("unknown termination_reason {}".format(self.termination_reason))
        if self.terminated != (self.termination_reason in TERMINATION_REASONS):
            raise EpisodeError("reason classification disagrees with terminated/truncated")
        if self.success != (self.termination_reason == "goal"):
            raise EpisodeError("success is only valid for goal")
        if self.collision != (self.termination_reason == "collision"):
            raise EpisodeError("collision flag must match collision reason")


@dataclass(frozen=True)
class TransitionDisposition:
    transition_stored: bool
    transition_drop_reason: str = ""


class EpisodeStateMachine:
    """First terminal event wins, guaranteeing one immutable end reason."""

    def __init__(self) -> None:
        self.phase = EpisodePhase.IDLE
        self.episode_id: Optional[str] = None
        self.outcome: Optional[EpisodeOutcome] = None

    def start(self, episode_id: str) -> None:
        if self.phase == EpisodePhase.RUNNING:
            raise EpisodeError("cannot start while an episode is running")
        if not isinstance(episode_id, str) or not episode_id:
            raise EpisodeError("episode_id must be a non-empty string")
        self.phase = EpisodePhase.RUNNING
        self.episode_id = episode_id
        self.outcome = None

    def finish(self, reason: str) -> EpisodeOutcome:
        if self.phase != EpisodePhase.RUNNING:
            raise EpisodeError("finish requires a running episode")
        if reason not in ALL_REASONS:
            raise EpisodeError("unknown termination_reason {}".format(reason))
        outcome = EpisodeOutcome(
            terminated=reason in TERMINATION_REASONS,
            truncated=reason in TRUNCATION_REASONS,
            termination_reason=reason,
            success=reason == "goal",
            collision=reason == "collision",
        )
        self.outcome = outcome
        self.phase = EpisodePhase.FINISHED
        return outcome

    def observe_activation_deadline(
        self, tracker: ActivationTracker, now: float
    ) -> TransitionDisposition:
        """Fail the episode and discard attribution on activation timeout."""

        if self.phase != EpisodePhase.RUNNING:
            raise EpisodeError("activation checks require a running episode")
        try:
            tracker.check_timeout(now)
        except ActivationTimeoutError:
            self.finish("interface_fault")
            return TransitionDisposition(False, ActivationTimeoutError.code)
        return TransitionDisposition(True, "")
