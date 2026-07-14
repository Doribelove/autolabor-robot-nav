import threading

import pytest
from actionlib_msgs.msg import GoalStatus

from thesis_experiment.gazebo_training_adapter import (
    GazeboTrainingAdapter,
    TrainingSafetyAdapter,
)


class _MoveBase:
    def __init__(self):
        self.cancel_count = 0

    def cancel_all_goals(self):
        self.cancel_count += 1


class _Publisher:
    def __init__(self):
        self.publish_count = 0

    def publish(self, _message):
        self.publish_count += 1


class _StateMoveBase(_MoveBase):
    def __init__(self, state):
        super().__init__()
        self.state = state

    def get_state(self):
        return self.state

    def wait_for_server(self, _duration):
        return True


class _ResettableSafety:
    def __init__(self):
        self.reset_count = 0

    def reset(self):
        self.reset_count += 1


def test_unchanged_curriculum_preserves_scenario_rotation_index():
    adapter = GazeboTrainingAdapter.__new__(GazeboTrainingAdapter)
    adapter.scenarios = ({"scene_id": "a"}, {"scene_id": "b"})
    adapter._scenario_index = 1
    adapter.move_base = _MoveBase()
    adapter.cmd_pub = _Publisher()

    adapter.set_scenarios(({"scene_id": "a"}, {"scene_id": "b"}))

    assert adapter._scenario_index == 1
    assert adapter.move_base.cancel_count == 0
    assert adapter.cmd_pub.publish_count == 0


def test_training_safety_reset_clears_episode_local_corridor_latch():
    safety = _ResettableSafety()
    adapter = TrainingSafetyAdapter(safety, object(), directional_emergency=True)
    adapter._corridor_active = True
    adapter.last_decision = object()
    adapter.last_fallback = object()

    adapter.reset(seed=101)

    assert safety.reset_count == 1
    assert adapter._corridor_active is False
    assert adapter.last_decision is None
    assert adapter.last_fallback is None


def test_changed_curriculum_restarts_rotation_once():
    adapter = GazeboTrainingAdapter.__new__(GazeboTrainingAdapter)
    adapter.scenarios = ({"scene_id": "a"},)
    adapter._scenario_index = 0
    adapter.move_base = _MoveBase()
    adapter.cmd_pub = _Publisher()

    adapter.set_scenarios(({"scene_id": "a"}, {"scene_id": "b"}))

    assert adapter._scenario_index == -1
    assert tuple(item["scene_id"] for item in adapter.scenarios) == ("a", "b")
    assert adapter.move_base.cancel_count == 1
    assert adapter.cmd_pub.publish_count == 1


def test_atomic_boundary_orders_quiesce_restore_reset_then_dispatch():
    adapter = GazeboTrainingAdapter.__new__(GazeboTrainingAdapter)
    events = []
    snapshot = {"anchor": 1.0}
    adapter._quiesce_navigation = lambda: events.append("quiesce")
    adapter._apply_parameter_snapshot = lambda value: events.append(
        ("restore", dict(value)))
    adapter._reset_scene_and_dispatch = lambda seed: events.append(
        ("reset_and_dispatch", seed))

    adapter.reset_with_parameter_snapshot(snapshot, 101)

    assert events == [
        "quiesce", ("restore", snapshot), ("reset_and_dispatch", 101)]


def test_quiesce_failure_prevents_parameter_restore_and_goal_dispatch():
    adapter = GazeboTrainingAdapter.__new__(GazeboTrainingAdapter)
    events = []

    def fail_quiesce():
        events.append("quiesce_failed")
        raise RuntimeError("not quiet")

    adapter._quiesce_navigation = fail_quiesce
    adapter._apply_parameter_snapshot = lambda value: events.append("restore")
    adapter._reset_scene_and_dispatch = lambda seed: events.append("dispatch")

    try:
        adapter.reset_with_parameter_snapshot({"anchor": 1.0}, 101)
    except RuntimeError as exc:
        assert str(exc) == "not quiet"
    else:
        raise AssertionError("quiescence failure must fail closed")

    assert events == ["quiesce_failed"]


def test_quiesce_requires_terminal_action_state_before_write_boundary():
    adapter = GazeboTrainingAdapter.__new__(GazeboTrainingAdapter)
    adapter.move_base = _StateMoveBase(GoalStatus.PREEMPTED)
    adapter.cmd_pub = _Publisher()
    adapter._condition = threading.Condition()
    adapter.local_plan_generation = 4
    adapter._boundary_quiet_period_s = 0.0
    adapter._boundary_recovery_quiet_period_s = 0.0
    adapter._boundary_quiesce_timeout_s = 0.01
    adapter._boundary_quiesce_count = 0
    adapter._boundary_quiesce_failure_count = 0
    adapter._boundary_last_terminal_state = None
    adapter._boundary_last_quiet_period_s = 0.0
    adapter._activation_timeout_barrier_count = 0
    adapter._activation_timeout_last_config_seq = None
    adapter._require_recovery_barrier = False

    adapter._quiesce_navigation()

    assert adapter.move_base.cancel_count == 1
    assert adapter._boundary_quiesce_count == 1
    assert adapter._boundary_quiesce_failure_count == 0
    assert adapter._boundary_last_terminal_state == GoalStatus.PREEMPTED


def test_quiesce_times_out_while_action_is_active():
    adapter = GazeboTrainingAdapter.__new__(GazeboTrainingAdapter)
    adapter.move_base = _StateMoveBase(GoalStatus.ACTIVE)
    adapter.cmd_pub = _Publisher()
    adapter._condition = threading.Condition()
    adapter.local_plan_generation = 9
    adapter._boundary_quiet_period_s = 0.0
    adapter._boundary_recovery_quiet_period_s = 0.0
    adapter._boundary_quiesce_timeout_s = 0.0
    adapter._boundary_quiesce_count = 0
    adapter._boundary_quiesce_failure_count = 0
    adapter._boundary_last_terminal_state = None
    adapter._boundary_last_quiet_period_s = 0.0
    adapter._activation_timeout_barrier_count = 0
    adapter._activation_timeout_last_config_seq = None
    adapter._require_recovery_barrier = False

    with pytest.raises(RuntimeError, match="did not quiesce"):
        adapter._quiesce_navigation()

    assert adapter._boundary_quiesce_count == 0
    assert adapter._boundary_quiesce_failure_count == 1


def test_activation_timeout_marks_one_extended_recovery_barrier():
    adapter = GazeboTrainingAdapter.__new__(GazeboTrainingAdapter)
    adapter._activation_timeout_barrier_count = 0
    adapter._activation_timeout_last_config_seq = None
    adapter._require_recovery_barrier = False

    adapter.mark_activation_timeout(7)

    assert adapter._activation_timeout_barrier_count == 1
    assert adapter._activation_timeout_last_config_seq == 7
    assert adapter._require_recovery_barrier is True
