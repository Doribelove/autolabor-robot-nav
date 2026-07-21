import math

import pytest

from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.parameter_projection import (
    CandidateRejected,
    ParameterLimit,
    ParameterProjector,
    ProjectionConfigurationError,
)


def theta(**updates):
    values = {
        "max_vel_x": 1.0, "max_vel_theta": 0.5,
        "acc_lim_x": 1.0, "acc_lim_theta": 1.0,
        "min_obstacle_dist": 0.4, "inflation_dist": 0.6,
        "weight_obstacle": 10.0, "weight_viapoint": 2.0,
        "weight_optimaltime": 1.0,
    }
    values.update(updates)
    return values


def limits(online=True):
    result = {}
    for name, value in theta().items():
        result[name] = ParameterLimit(0.0, max(20.0, value * 2.0), 0.25, online)
    return result


def test_requires_exact_nine_and_rejects_non_finite_and_bool():
    projector = ParameterProjector(limits())
    for bad in (theta(max_vel_x=math.nan), theta(max_vel_x=math.inf), theta(max_vel_x=True)):
        with pytest.raises(CandidateRejected):
            projector.project(bad, theta())
    missing = theta()
    del missing[EXPECTED_THETA_ORDER[-1]]
    with pytest.raises(CandidateRejected):
        projector.project(missing, theta())
    with pytest.raises(CandidateRejected):
        projector.project(dict(theta(), surprise=1.0), theta())


def test_box_rate_and_coupling_projection_are_audited():
    configured = limits()
    configured["max_vel_x"] = ParameterLimit(0.2, 1.5, 0.1)
    configured["inflation_dist"] = ParameterLimit(0.0, 20.0, 1.0)
    configured["min_obstacle_dist"] = ParameterLimit(0.0, 20.0, 1.0)
    projector = ParameterProjector(configured, min_turning_radius=2.0)
    result = projector.project(
        theta(max_vel_x=4.0, max_vel_theta=2.0,
              min_obstacle_dist=0.8, inflation_dist=0.3), theta())
    assert result.projected["max_vel_x"] == pytest.approx(1.1)
    assert result.projected["max_vel_theta"] == pytest.approx(0.55)
    assert result.projected["inflation_dist"] == pytest.approx(0.8)
    assert result.intervened
    assert "max_vel_x:physical_bound" in result.reasons
    assert "max_vel_x:rate_limit" in result.reasons
    assert "inflation_dist:below_min_obstacle_dist" in result.reasons
    assert "max_vel_theta:ackermann_turning_radius" in result.reasons


def test_offline_change_is_atomic_rejection():
    configured = limits()
    configured["weight_viapoint"] = ParameterLimit(0.0, 20.0, 1.0, False)
    with pytest.raises(CandidateRejected, match="online_update_unsupported"):
        ParameterProjector(configured).project(theta(weight_viapoint=3.0), theta())


def test_contract_null_calibration_fails_closed():
    rows = [dict(name=name, online_support=True, physical_min=None,
                 physical_max=None, max_delta_per_rl_step=None)
            for name in EXPECTED_THETA_ORDER]
    with pytest.raises(ProjectionConfigurationError):
        ParameterProjector.from_contract_candidates(rows)
