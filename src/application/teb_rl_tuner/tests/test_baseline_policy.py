import pytest

from teb_rl_tuner.baseline_policy import (
    CausalRuleTebPolicy,
    ObservationLayout,
    RuleThresholds,
)
from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.training_environment import TrainingEnvironmentError


FEATURES = ("footprint_clearance", "approximate_ttc", "path_cross_track_error")


def profile(value):
    return {name: float(value) for name in EXPECTED_THETA_ORDER}


def policy():
    return CausalRuleTebPolicy(
        {"efficient": profile(1), "tracking": profile(2), "cautious": profile(3)},
        RuleThresholds(0.6, 0.8, 1.0, 1.5, 0.4, 0.2),
        ObservationLayout(1, FEATURES, 2),
    )


def observation(clearance, ttc, error, old=(9.0, 9.0, 9.0)):
    return (5.0,) + old + (5.0, clearance, ttc, error)


def test_rule_policy_uses_latest_frame_and_switches_causally():
    rule = policy()
    assert rule.act(observation(2.0, 5.0, 0.0)) == profile(1)
    assert rule.mode == "efficient"
    assert rule.act(observation(2.0, 5.0, 0.5)) == profile(2)
    assert rule.mode == "tracking"
    assert rule.act(observation(0.5, 5.0, 0.5)) == profile(3)
    assert rule.mode == "cautious"
    assert rule.decision_count == 3 and rule.mode_transitions == 2


def test_rule_hysteresis_prevents_threshold_chatter():
    rule = policy()
    rule.act(observation(0.5, 5.0, 0.0))
    assert rule.mode == "cautious"
    rule.act(observation(0.7, 5.0, 0.0))
    assert rule.mode == "cautious"
    rule.act(observation(0.9, 2.0, 0.0))
    assert rule.mode == "efficient"
    rule.act(observation(2.0, 5.0, 0.5))
    assert rule.mode == "tracking"
    rule.act(observation(2.0, 5.0, 0.3))
    assert rule.mode == "tracking"
    rule.act(observation(2.0, 5.0, 0.1))
    assert rule.mode == "efficient"


def test_reset_removes_episode_local_rule_state():
    rule = policy()
    rule.act(observation(0.5, 5.0, 0.0))
    rule.reset(seed=42)
    assert rule.mode == "efficient"
    assert rule.decision_count == 0 and rule.mode_transitions == 0
    assert rule.last_features == {}


def test_invalid_profile_and_observation_fail_closed():
    bad = profile(1)
    bad.pop(EXPECTED_THETA_ORDER[-1])
    with pytest.raises(TrainingEnvironmentError):
        CausalRuleTebPolicy(
            {"efficient": bad, "tracking": profile(2), "cautious": profile(3)},
            RuleThresholds(0.6, 0.8, 1.0, 1.5, 0.4, 0.2),
            ObservationLayout(1, FEATURES, 2),
        )
    with pytest.raises(TrainingEnvironmentError):
        policy().act((1.0,))
