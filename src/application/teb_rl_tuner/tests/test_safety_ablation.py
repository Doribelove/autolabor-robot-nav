from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.fallback_policy import ConservativeFallbackPolicy
from teb_rl_tuner.safety_ablation import NoFallbackSafetyAdapter, SAFETY_ABLATIONS
from teb_rl_tuner.safety_margin_filter import SafetyMarginConfig, SafetyMarginFilter


class Frame:
    named_features = {"footprint_clearance": 0.0, "linear_velocity": 1.0}


def theta(value):
    return {name: float(value) for name in EXPECTED_THETA_ORDER}


def test_ablation_order_is_frozen():
    assert SAFETY_ABLATIONS == ("FullSafety", "ProjectionOnly", "NoSafety", "NoFallback")


def test_no_fallback_requests_stop_without_substituting_conservative_theta():
    filter_core = SafetyMarginFilter(SafetyMarginConfig(
        a_brake_lower=0.5, tau_total_upper=0.5, d_margin=0.2,
        warning_margin=0.2, emergency_margin=0.0,
        hysteresis_margin=0.3, recovery_healthy_s=1.0,
    ))
    adapter = NoFallbackSafetyAdapter(
        filter_core, ConservativeFallbackPolicy(theta(0.5))
    )
    projected = theta(1.0)
    result = adapter.filter(projected, theta(0.8), Frame(), 1.0)
    assert result.request_stop is True
    assert result.theta == projected
    assert adapter.last_fallback.use_fallback is False
    assert "ablation:no_conservative_parameter_fallback" in result.reasons
