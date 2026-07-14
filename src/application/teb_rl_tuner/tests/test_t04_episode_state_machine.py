import pytest

from teb_rl_tuner.episode_state_machine import EpisodeError, EpisodeStateMachine
from teb_rl_tuner.timing import ActivationTracker


def test_each_reason_has_exactly_one_classification():
    cases = [
        ("goal", True, False),
        ("collision", True, False),
        ("planner_failure", True, False),
        ("sensor_fault", True, False),
        ("tf_fault", True, False),
        ("interface_fault", True, False),
        ("emergency_stop", True, False),
        ("timeout", False, True),
        ("operator_stop", False, True),
        ("infrastructure_fault", False, True),
    ]
    for reason, terminated, truncated in cases:
        manager = EpisodeStateMachine()
        manager.start("episode-1")
        outcome = manager.finish(reason)
        assert outcome.terminated is terminated
        assert outcome.truncated is truncated
        assert outcome.terminated != outcome.truncated
        assert outcome.termination_reason == reason


def test_first_terminal_event_is_immutable():
    manager = EpisodeStateMachine()
    manager.start("episode-1")
    manager.finish("collision")
    with pytest.raises(EpisodeError):
        manager.finish("timeout")
    assert manager.outcome.termination_reason == "collision"


def test_activation_timeout_discards_transition_and_faults_episode():
    manager = EpisodeStateMachine()
    manager.start("episode-activation")
    tracker = ActivationTracker(3, 1.0, 1.1, 1.2, timeout_s=0.5)
    disposition = manager.observe_activation_deadline(tracker, 1.7)
    assert disposition.transition_stored is False
    assert disposition.transition_drop_reason == "parameter_activation_timeout"
    assert manager.outcome.termination_reason == "interface_fault"
    assert manager.outcome.terminated is True
