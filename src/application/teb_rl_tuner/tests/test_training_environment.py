import math

from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.parameter_projection import ParameterLimit, ParameterProjector
from teb_rl_tuner.reward_cost import FeedbackSample, RewardWeights, WindowEvents
from teb_rl_tuner.state_builder import StateBuilder
from teb_rl_tuner.training_environment import (
    ActivationPoll,
    EnvironmentConfig,
    FeedbackWindow,
    FixedPolicy,
    ObservationInput,
    ParameterWriteReceipt,
    RandomSmallPolicy,
    TrainingEnvironment,
)


def theta(value=1.0):
    return {name: float(value) for name in EXPECTED_THETA_ORDER}


def projector():
    return ParameterProjector({
        name: ParameterLimit(0.0, 10.0, 1.0) for name in EXPECTED_THETA_ORDER
    })


class FakeAdapter:
    def __init__(self):
        self.clock = 0.0
        self.theta = theta()
        self.restore_calls = []
        self.stop_calls = []
        self.writes = []
        self.timeout_activation = False
        self.fail_write = False
        self.reset_calls = []
        self.activation_timeout_marks = []

    def reset(self, seed):
        self.clock = 0.0
        self.reset_calls.append(seed)

    def read_observation(self):
        self.clock += 0.1
        return ObservationInput(
            stamps={"scan": self.clock},
            ranges=(1.0,),
            range_min=0.0,
            range_max=5.0,
            features={"speed": 0.2},
            validity={"scan": True},
        )

    def current_theta(self):
        return dict(self.theta)

    def capture_parameter_snapshot(self):
        return dict(self.theta)

    def restore_parameter_snapshot(self, snapshot):
        self.theta = dict(snapshot)
        self.restore_calls.append(dict(snapshot))

    def mark_activation_timeout(self, config_seq):
        self.activation_timeout_marks.append(config_seq)

    def request_stop(self, reason):
        self.stop_calls.append(reason)

    def write_parameters(self, requested, config_seq):
        if self.fail_write:
            raise RuntimeError("writer unavailable")
        request = self.clock + 0.01
        ack = request + 0.01
        self.theta = dict(requested)
        self.writes.append((config_seq, dict(requested)))
        return ParameterWriteReceipt(request, ack, dict(requested))

    def poll_activation(self, config_seq, t_ack, timeout_s):
        del config_seq
        if self.timeout_activation:
            return ActivationPoll((), t_ack + timeout_s)
        active = t_ack + 0.01
        return ActivationPoll(((active, True),), active)

    def collect_feedback(self, t_active, t_window_end):
        self.clock = max(self.clock, t_window_end)
        return FeedbackWindow(
            samples=(
                FeedbackSample(t_active, 2.0, 0.1, 1.0),
                FeedbackSample(t_window_end, 1.9, 0.1, 1.0),
            ),
            events=WindowEvents(),
            theta_delta_normalized=(0.0,) * 9,
        )


def make_env(adapter, max_steps=1000, max_ros_duration_s=10000.0):
    return TrainingEnvironment(
        adapter,
        StateBuilder(("speed",), ("scan",), sector_count=1),
        projector(),
        RewardWeights(),
        EnvironmentConfig(
            history_length=4,
            activation_timeout_s=0.5,
            reward_window_s=0.2,
            max_steps=max_steps,
            max_ros_duration_s=max_ros_duration_s,
            warning_distance=0.5,
        ),
    )


def test_reset_primes_history_and_clears_all_episode_local_state():
    adapter = FakeAdapter()
    env = make_env(adapter, max_steps=1)
    observation, info = env.reset(seed=7)
    assert len(observation) == 8
    assert info["history_ready"] is True
    assert info["config_seq"] == 0
    _, _, _, truncated, _ = env.step(theta())
    assert truncated is True
    second, second_info = env.reset(seed=8)
    assert len(second) == 8
    assert second_info["config_seq"] == 0
    assert second_info["episode_id"] != info["episode_id"]
    assert adapter.reset_calls == [7, 8]
    assert len(adapter.restore_calls) == 1


def test_atomic_episode_boundary_hook_owns_restore_and_reset_order():
    class AtomicAdapter(FakeAdapter):
        def __init__(self):
            super().__init__()
            self.atomic_calls = []

        def reset_with_parameter_snapshot(self, snapshot, seed):
            self.atomic_calls.append((dict(snapshot), seed))
            self.theta = dict(snapshot)
            self.clock = 0.0

    adapter = AtomicAdapter()
    env = make_env(adapter)
    env.reset(seed=11)
    env.step(theta(1.1))
    env.reset(seed=12)

    assert adapter.reset_calls == [11]
    assert adapter.restore_calls == []
    assert adapter.atomic_calls == [(theta(), 12)]
    assert adapter.theta == theta()


def test_hundreds_of_steps_keep_config_sequence_and_history_bounded():
    adapter = FakeAdapter()
    env = make_env(adapter, max_steps=300)
    observation, _ = env.reset(seed=1)
    policy = FixedPolicy(theta())
    for expected in range(1, 301):
        action = env.action_from(policy, observation)
        observation, reward, terminated, truncated, info = env.step(action)
        assert len(observation) == 8
        assert math.isfinite(reward)
        assert info["config_seq"] == expected
        assert info["transition_stored"] is True
        assert info["t_decision"] < info["t_request"] < info["t_ack"] < info["t_active"]
        assert info["candidate_theta"] == theta()
        assert info["projected_theta"] == theta()
        assert info["safe_theta"] == theta()
        assert info["applied_theta"] == theta()
        assert info["projection_modified"] is False
        assert "reward_total" in info["reward_fields"]
        assert len(env.history.frames) == 4
        assert terminated is False
        assert truncated is (expected == 300)
    assert [seq for seq, _ in adapter.writes] == list(range(1, 301))
    assert info["stored_steps"] == 300
    assert info["termination_reason"] == "timeout"
    assert env.current_outcome.truncated is True
    assert env.episode_statistics["stored_steps"] == 300
    assert env.episode_statistics["dropped_steps"] == 0


def test_activation_timeout_discards_transition_and_restores_snapshot():
    adapter = FakeAdapter()
    env = make_env(adapter)
    old_observation, _ = env.reset(seed=2)
    adapter.timeout_activation = True
    observation, reward, terminated, truncated, info = env.step(theta(1.1))
    assert observation == old_observation
    assert reward == 0.0
    assert terminated is True and truncated is False
    assert info["transition_stored"] is False
    assert info["transition_drop_reason"] == "parameter_activation_timeout"
    assert info["termination_reason"] == "interface_fault"
    assert info["snapshot_restore_deferred"] is True
    assert adapter.activation_timeout_marks == [1]
    assert adapter.restore_calls == []
    env.reset(seed=3)
    assert adapter.restore_calls == [theta()]


def test_confirmed_goal_takes_precedence_over_post_goal_activation_timeout():
    adapter = FakeAdapter()
    env = make_env(adapter)
    env.reset(seed=9)
    adapter.timeout_activation = True
    adapter.terminal_reason = lambda: "goal"
    _, reward, terminated, truncated, info = env.step(theta())
    assert reward == 0.0
    assert terminated is True and truncated is False
    assert info["termination_reason"] == "goal"
    assert info["transition_drop_reason"] == "terminal_before_parameter_activation"
    assert adapter.stop_calls == []
    assert info["snapshot_restore_deferred"] is True
    assert adapter.activation_timeout_marks == [1]
    assert adapter.restore_calls == []
    env.reset(seed=10)
    assert adapter.restore_calls == [theta()]


def test_unexpected_writer_exception_fails_closed_without_escaping_step():
    adapter = FakeAdapter()
    env = make_env(adapter)
    original, _ = env.reset(seed=3)
    adapter.fail_write = True
    observation, reward, terminated, truncated, info = env.step(theta())
    assert observation == original
    assert reward == 0.0
    assert terminated is True and truncated is False
    assert info["transition_drop_reason"] == "fail_closed"
    assert "writer unavailable" in info["error"]
    assert len(adapter.stop_calls) == 1
    assert len(adapter.restore_calls) == 1


def test_random_small_policy_is_seeded_and_never_uses_numpy():
    baseline = theta(2.0)
    delta = theta(0.05)
    left = RandomSmallPolicy(baseline, delta, seed=42)
    right = RandomSmallPolicy(baseline, delta, seed=42)
    action_left = left.act(())
    action_right = right.act(())
    assert action_left == action_right
    assert all(abs(action_left[name] - 2.0) <= 0.05 for name in EXPECTED_THETA_ORDER)


def test_ros_time_limit_truncates_even_below_step_limit():
    adapter = FakeAdapter()
    env = make_env(adapter, max_steps=100, max_ros_duration_s=0.25)
    env.reset(seed=4)
    _, _, terminated, truncated, info = env.step(theta())
    assert terminated is False and truncated is True
    assert info["termination_reason"] == "timeout"


class EmergencySafety:
    def __init__(self):
        self.reset_calls = []

    def reset(self, seed=None):
        self.reset_calls.append(seed)

    def filter(self, projected, current, frame, now):
        del current, frame, now
        from teb_rl_tuner.training_environment import SafeParameterDecision
        return SafeParameterDecision(projected, request_stop=True, reasons=("margin:emergency",))


def test_safety_stop_is_an_explicit_emergency_and_safety_resets_per_episode():
    adapter = FakeAdapter()
    safety = EmergencySafety()
    env = make_env(adapter)
    env.safety_adapter = safety
    old, _ = env.reset(seed=9)
    observation, reward, terminated, truncated, info = env.step(theta())
    assert observation == old and reward == 0.0
    assert terminated is True and truncated is False
    assert info["termination_reason"] == "emergency_stop"
    assert info["transition_stored"] is False
    assert info["transition_drop_reason"] == "safety_emergency_stop"
    assert safety.reset_calls == [9]
