import pytest

from teb_rl_tuner.reward_cost import (
    FeedbackSample,
    RewardWeights,
    RewardWindowError,
    WindowEvents,
    calculate_reward_and_cost,
)


def sample(stamp, distance, clearance=1.0, path_error=0.0, near=False, fallback=False):
    return FeedbackSample(
        stamp=stamp,
        goal_distance=distance,
        path_error=path_error,
        clearance=clearance,
        linear_acceleration=1.0,
        angular_acceleration=2.0,
        near_collision=near,
        fallback_active=fallback,
    )


def test_reward_uses_only_post_activation_physical_time():
    result = calculate_reward_and_cost(
        samples=[
            sample(0.5, 9.0),  # write-overlap feedback must not be attributed
            sample(1.0, 8.0, clearance=0.5, path_error=1.0, near=True),
            sample(1.5, 7.5, clearance=0.5, path_error=1.0, near=True),
            sample(2.0, 7.0, clearance=0.5, path_error=1.0, near=True, fallback=True),
            sample(2.5, 6.0),  # outside reward window
        ],
        t_active=1.0,
        t_window_end=2.0,
        theta_delta_normalized=[0.1, -0.2],
        events=WindowEvents(planner_failure_count=2, parameter_violation_count=1),
        weights=RewardWeights(),
        warning_distance=1.0,
    )
    fields = result.step_fields()
    assert fields["valid_feedback_duration"] == pytest.approx(1.0)
    assert fields["goal_distance_start"] == pytest.approx(8.0)
    assert fields["goal_distance_end"] == pytest.approx(7.0)
    assert fields["reward_progress"] == pytest.approx(1.0)
    assert fields["reward_time"] == pytest.approx(-1.0)
    assert fields["reward_near_obstacle"] == pytest.approx(-0.25)
    assert fields["reward_path_error"] == pytest.approx(-1.0)
    assert fields["reward_smoothness"] == pytest.approx(-5.0)
    assert fields["reward_planner_failure"] == pytest.approx(-2.0)
    assert fields["reward_parameter_adjustment"] == pytest.approx(-0.3)
    assert fields["cost_near_collision"] == pytest.approx(1.0)
    assert fields["cost_parameter_violation"] == pytest.approx(1.0)
    assert fields["cost_planner_failure"] == pytest.approx(2.0)
    assert fields["reward_total"] == pytest.approx(
        sum(value for key, value in result.components.items() if key != "reward_total")
    )


def test_reward_is_sampling_rate_independent_for_constant_signals():
    common = dict(
        t_active=0.0,
        t_window_end=1.0,
        theta_delta_normalized=[],
        events=WindowEvents(),
        weights=RewardWeights(),
        warning_distance=1.0,
    )
    sparse = calculate_reward_and_cost(
        [sample(0.0, 2.0, 0.5, 1.0), sample(1.0, 1.0, 0.5, 1.0)], **common
    )
    dense = calculate_reward_and_cost(
        [
            sample(0.0, 2.0, 0.5, 1.0),
            sample(0.25, 1.75, 0.5, 1.0),
            sample(0.5, 1.5, 0.5, 1.0),
            sample(0.75, 1.25, 0.5, 1.0),
            sample(1.0, 1.0, 0.5, 1.0),
        ],
        **common
    )
    assert dense.reward_total == pytest.approx(sparse.reward_total)


def test_window_requires_two_post_activation_samples():
    with pytest.raises(RewardWindowError):
        calculate_reward_and_cost(
            [sample(0.9, 2.0), sample(1.1, 1.9)],
            1.0,
            2.0,
            [],
            WindowEvents(),
            RewardWeights(),
            1.0,
        )
