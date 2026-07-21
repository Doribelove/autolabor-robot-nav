import math

import pytest

from teb_rl_tuner.fallback_policy import ConservativeFallbackPolicy, FallbackUnavailable
from teb_rl_tuner.safety_margin_filter import (
    SafetyConfigurationError,
    SafetyMarginConfig,
    SafetyMarginFilter,
    SafetyMode,
)


HEALTHY = dict(sensor=True, tf=True, localization=True,
               parameter_interface=True, planner=True)


def config():
    return SafetyMarginConfig(
        a_brake_lower=1.0,
        tau_total_upper=0.2,
        d_margin=0.1,
        warning_margin=0.4,
        emergency_margin=0.0,
        hysteresis_margin=0.2,
        recovery_healthy_s=1.0,
    )


def theta(value=1.0):
    return {
        "max_vel_x": value, "max_vel_theta": value,
        "acc_lim_x": value, "acc_lim_theta": value,
        "min_obstacle_dist": value, "inflation_dist": value,
        "weight_obstacle": value, "weight_viapoint": value,
        "weight_optimaltime": value,
    }


def test_calibrated_braking_model_and_missing_values_fail_closed():
    filt = SafetyMarginFilter(config())
    assert filt.safe_distance(2.0) == pytest.approx(2.5)
    with pytest.raises(SafetyConfigurationError):
        SafetyMarginFilter(SafetyMarginConfig(None, 0.2, 0.1, 0.4, 0.0, 0.2, 1.0))


def test_immediate_escalation_hysteresis_and_continuous_recovery():
    filt = SafetyMarginFilter(config())
    assert filt.update(0.3, 0.0, 0.0, HEALTHY).mode == SafetyMode.WARNING
    assert filt.update(0.05, 0.0, 0.1, HEALTHY).mode == SafetyMode.EMERGENCY
    # Safe distance is 0.1; recovery requires margin > 0.6 continuously.
    assert filt.update(1.0, 0.0, 0.2, HEALTHY).mode == SafetyMode.EMERGENCY
    assert filt.update(1.0, 0.0, 1.3, HEALTHY).mode == SafetyMode.WARNING
    assert filt.update(1.0, 0.0, 1.4, HEALTHY).mode == SafetyMode.NORMAL


def test_emergency_cap_and_confirmation_prevent_single_frame_false_stop():
    cfg = config()
    cfg = SafetyMarginConfig(
        cfg.a_brake_lower, cfg.tau_total_upper, cfg.d_margin,
        cfg.warning_margin, cfg.emergency_margin, cfg.hysteresis_margin,
        cfg.recovery_healthy_s, emergency_distance_cap=0.2,
        emergency_confirmation_s=0.25,
    )
    filt = SafetyMarginFilter(cfg)
    # Braking margin is negative, but 0.3 m remains outside the hard near field.
    assert filt.update(0.3, 1.0, 0.0, HEALTHY).mode == SafetyMode.WARNING
    # A near-field sample starts confirmation instead of immediately stopping.
    pending = filt.update(0.1, 1.0, 0.1, HEALTHY)
    assert pending.mode == SafetyMode.WARNING
    assert "emergency:confirmation_pending" in pending.reasons
    assert filt.update(0.1, 1.0, 0.4, HEALTHY).mode == SafetyMode.EMERGENCY


def test_side_clearance_warns_without_forward_emergency():
    cfg = config()
    filt = SafetyMarginFilter(SafetyMarginConfig(
        cfg.a_brake_lower, cfg.tau_total_upper, cfg.d_margin,
        cfg.warning_margin, cfg.emergency_margin, cfg.hysteresis_margin,
        cfg.recovery_healthy_s, emergency_distance_cap=0.35,
        emergency_confirmation_s=0.25,
    ))
    first = filt.update(0.30, 0.8, 1.0, HEALTHY,
                        emergency_obstacle_distance=2.0)
    second = filt.update(0.30, 0.8, 2.0, HEALTHY,
                         emergency_obstacle_distance=2.0)
    assert first.mode == second.mode == SafetyMode.WARNING
    assert all("confirmation_pending" not in reason for reason in second.reasons)


def test_health_fault_requires_reset_and_audits_reason():
    filt = SafetyMarginFilter(config())
    failed = dict(HEALTHY, tf=False)
    decision = filt.update(1.0, 0.0, 0.0, failed)
    assert decision.mode == SafetyMode.FAULT
    assert "health:tf:invalid" in decision.reasons
    assert filt.update(1.0, 0.0, 0.1, HEALTHY).mode == SafetyMode.FAULT
    # Reset still observes continuous-health timer and descends conservatively.
    assert filt.update(1.0, 0.0, 1.2, HEALTHY, True).mode == SafetyMode.EMERGENCY


def test_invalid_measurements_enter_fault():
    filt = SafetyMarginFilter(config())
    assert filt.update(math.nan, 0.0, 0.0, HEALTHY).mode == SafetyMode.FAULT


def test_fallback_is_complete_atomic_and_interface_failure_uses_last_ack():
    conservative = theta(0.5)
    policy = ConservativeFallbackPolicy(conservative)
    emergency = policy.decide(SafetyMode.EMERGENCY, None, True)
    assert emergency.theta == conservative
    assert emergency.stop_learning_writes and emergency.request_stop
    with pytest.raises(FallbackUnavailable, match="before any confirmed-safe"):
        policy.decide(SafetyMode.FAULT, None, False)
    confirmed = theta(0.75)
    policy.confirm_applied_safe(confirmed)
    failed = policy.decide(SafetyMode.FAULT, None, False)
    assert failed.theta == confirmed
    assert "last_confirmed_safe" in failed.reasons[0]


def test_warning_never_increases_speed_or_reduces_clearance_from_last_safe():
    policy = ConservativeFallbackPolicy(theta(0.5))
    policy.confirm_applied_safe(theta(1.0))
    requested = theta(2.0)
    requested["min_obstacle_dist"] = 0.5
    requested["inflation_dist"] = 0.6
    requested["weight_obstacle"] = 0.7
    warning = policy.decide(SafetyMode.WARNING, requested, True)
    assert warning.theta["max_vel_x"] == 1.0
    assert warning.theta["max_vel_theta"] == 1.0
    assert warning.theta["min_obstacle_dist"] == 1.0
    assert warning.theta["inflation_dist"] == 1.0
    assert warning.theta["weight_obstacle"] == 1.0


def test_incomplete_conservative_or_runtime_vector_is_rejected():
    incomplete = theta()
    incomplete.pop("max_vel_x")
    with pytest.raises(FallbackUnavailable):
        ConservativeFallbackPolicy(incomplete)
    policy = ConservativeFallbackPolicy(theta(0.5))
    with pytest.raises(FallbackUnavailable):
        policy.decide(SafetyMode.NORMAL, incomplete, True)
