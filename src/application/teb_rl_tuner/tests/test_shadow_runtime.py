import pytest

from teb_rl_tuner.fallback_policy import ConservativeFallbackPolicy
from teb_rl_tuner.parameter_projection import ParameterLimit, ParameterProjector
from teb_rl_tuner.safety_margin_filter import SafetyMarginConfig, SafetyMarginFilter, SafetyMode
from teb_rl_tuner.shadow_runtime import (
    FeatureEnvelope, ShadowRuntime, ShadowRuntimeConfig, ThetaEmaSmoother,
)
from teb_rl_tuner.config import EXPECTED_THETA_ORDER


def theta(value=1.0):
    return {name: value for name in EXPECTED_THETA_ORDER}


def runtime():
    limits = {name: ParameterLimit(0.0, 2.0, 0.2) for name in EXPECTED_THETA_ORDER}
    safety = SafetyMarginFilter(SafetyMarginConfig(
        0.5, 0.5, 0.2, 0.2, 0.0, 0.3, 1.0,
        emergency_distance_cap=0.35, emergency_confirmation_s=0.25,
    ))
    return ShadowRuntime(
        ShadowRuntimeConfig(0.5, 0.25, 1.0),
        ParameterProjector(limits), safety,
        ConservativeFallbackPolicy(theta(0.5)),
        FeatureEnvelope({"footprint_clearance": (0.2, 30.0),
                         "linear_velocity": (-1.2, 1.2),
                         "approximate_ttc": (0.0, 30.0)}),
    )


def test_ema_and_projection_limit_aggressive_candidate():
    smooth = ThetaEmaSmoother(0.5)
    result = smooth.update(theta(2.0), theta(1.0))
    assert result["max_vel_x"] == pytest.approx(1.5)
    decision = runtime().evaluate(
        theta(2.0), theta(1.0),
        {"footprint_clearance": 2.0, "linear_velocity": 1.0,
         "approximate_ttc": 2.0},
        dict(sensor=True, tf=True, localization=True,
             parameter_interface=True, planner=True), 0.0)
    assert decision.projected_theta["max_vel_x"] == pytest.approx(1.2)
    assert decision.write_allowed is False and decision.motion_allowed is False


def test_ood_falls_back_but_never_authorizes_write():
    decision = runtime().evaluate(
        theta(1.1), theta(1.0),
        {"footprint_clearance": 100.0, "linear_velocity": 1.0,
         "approximate_ttc": 100.0},
        dict(sensor=True, tf=True, localization=True,
             parameter_interface=True, planner=True), 0.0)
    assert decision.recommended_theta == theta(0.5)
    assert "ood:fallback" in decision.reasons
    assert decision.write_allowed is False


def test_obstacle_at_t11_distance_is_warning_not_emergency():
    core = runtime()
    decision = core.evaluate(
        theta(1.0), theta(1.0),
        {"footprint_clearance": 1.2, "linear_velocity": 1.2,
         "approximate_ttc": 1.0},
        dict(sensor=True, tf=True, localization=True,
             parameter_interface=True, planner=True), 0.0)
    assert decision.safety.mode == SafetyMode.WARNING
    assert decision.safety.mode != SafetyMode.EMERGENCY
