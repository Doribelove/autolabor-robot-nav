import pytest

from teb_rl_tuner.timing import (
    ActivationTimeoutError,
    ActivationTracker,
    MonotonicRosTime,
    NonMonotonicTimeError,
    SynchronizationError,
    synchronize_stamps,
)


def test_ros_streams_are_monotonic_and_synchronized():
    clock = MonotonicRosTime()
    assert clock.observe("scan", 10.0) == 10.0
    assert clock.observe("scan", 10.1) == 10.1
    with pytest.raises(NonMonotonicTimeError):
        clock.observe("scan", 10.1)

    synced = synchronize_stamps(
        {"scan": 20.00, "odom": 20.03, "plan": 20.02},
        ("scan", "odom", "plan"),
        0.05,
    )
    assert synced.observation_time == pytest.approx(20.03)
    assert synced.skew_s == pytest.approx(0.03)
    with pytest.raises(SynchronizationError):
        synchronize_stamps({"scan": 20.0, "odom": 20.2}, ("scan", "odom"), 0.05)


def test_activation_is_first_complete_plan_strictly_after_ack():
    tracker = ActivationTracker(7, t_decision=1.0, t_request=1.1, t_ack=1.2, timeout_s=0.5)
    assert tracker.observe_local_plan(1.19, complete=True) is None
    assert tracker.observe_local_plan(1.3, complete=False) is None
    active = tracker.observe_local_plan(1.31, complete=True)
    assert active is not None
    assert active.t_active == pytest.approx(1.31)
    assert active.close(2.0) == pytest.approx((1.31, 2.0))
    tracker.check_timeout(3.0)  # Activated configurations cannot time out later.


def test_activation_timeout_uses_ros_time():
    tracker = ActivationTracker(8, 1.0, 1.1, 1.2, timeout_s=0.5)
    tracker.check_timeout(1.69)
    with pytest.raises(ActivationTimeoutError) as error:
        tracker.check_timeout(1.7)
    assert error.value.code == "parameter_activation_timeout"
