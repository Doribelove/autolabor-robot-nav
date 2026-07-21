import math
from pathlib import Path

import pytest

from teb_rl_tuner.config import EXPECTED_ETA_ORDER, EXPECTED_THETA_ORDER
from teb_rl_tuner.semantic_action import (
    FrozenSemanticMapping, ResidualSemanticMapping, SemanticActionError,
)


ROOT = Path(__file__).resolve().parents[4]


def bounds():
    return {name: (0.0, 10.0) for name in EXPECTED_THETA_ORDER}


def mapping(matrix=None):
    matrix = matrix or [[float(row == column) for column in range(5)] for row in range(9)]
    return FrozenSemanticMapping(matrix, bounds(), "test-v1", "a" * 64)


def theta(value=5.0):
    return {name: float(value) for name in EXPECTED_THETA_ORDER}


def test_delta_eta_maps_in_frozen_order_and_denormalizes():
    result = mapping().map_action(theta(), (0.0,) * 5, (0.2, -0.4, 0.0, 0.5, -0.1))
    assert result.eta_after == (0.2, -0.4, 0.0, 0.5, -0.1)
    assert result.delta_normalized_theta[:5] == result.delta_eta
    assert result.delta_normalized_theta[5:] == (0.0,) * 4
    assert result.theta_candidate[EXPECTED_THETA_ORDER[0]] == pytest.approx(6.0)
    assert result.theta_candidate[EXPECTED_THETA_ORDER[1]] == pytest.approx(3.0)
    assert all(math.isfinite(value) for value in result.theta_candidate.values())


def test_eta_and_theta_clipping_are_audited_separately():
    matrix = [[2.0] + [0.0] * 4 for _ in range(9)]
    result = mapping(matrix).map_action(theta(9.5), (0.8,) * 5, (0.5, 0.0, 0.0, 0.0, 0.0))
    assert result.clipped_eta is True and result.clipped_theta is True
    assert result.eta_after[0] == 1.0
    assert all(value == pytest.approx(10.0) for value in result.theta_candidate.values())


@pytest.mark.parametrize("action", [(0.0,) * 4, (float("nan"),) * 5, (1.1,) * 5])
def test_invalid_actions_fail_before_theta_candidate(action):
    with pytest.raises(SemanticActionError):
        mapping().map_action(theta(), (0.0,) * len(EXPECTED_ETA_ORDER), action)


def test_checked_in_mapping_loads_with_simulation_boundary():
    result = FrozenSemanticMapping.from_files(
        ROOT / "config/thesis_experiments/A_TEB_v1.yaml",
        ROOT / "src/application/teb_rl_tuner/config/t05_simulation_safety.yaml",
    )
    assert result.mapping_sha256 == "1ca660f8d4f1863a93d75686bc0cafe8259942aaac60c3e2817c31162fcb1000"


def test_t11_semantic_scaling_cannot_exceed_any_physical_rate_limit():
    result = FrozenSemanticMapping.from_files(
        ROOT / "config/thesis_experiments/A_TEB_v1.yaml",
        ROOT / "src/application/teb_rl_tuner/config/t05_simulation_safety.yaml",
        executable_rate_scaling=True,
    )
    current = {name: sum(result.bounds[name]) / 2.0 for name in EXPECTED_THETA_ORDER}
    mapped = result.map_action(current, (0.0,) * 5, (1.0,) * 5)
    for index, name in enumerate(EXPECTED_THETA_ORDER):
        physical_delta = abs(mapped.theta_candidate[name] - current[name])
        normalized_limit = result.normalized_delta_limits[index]
        physical_limit = 0.5 * normalized_limit * (
            result.bounds[name][1] - result.bounds[name][0]
        )
        assert physical_delta <= physical_limit + 1e-12


def test_residual_mapping_is_anchor_centered_non_accumulating_and_risk_scaled():
    base = mapping()
    residual = ResidualSemanticMapping(base, theta(5.0), (0.4,) * 9, 0.2)
    first = residual.map_action((1.0, 0.0, 0.0, 0.0, 0.0), 1.0)
    repeated = residual.map_action((1.0, 0.0, 0.0, 0.0, 0.0), 1.0)
    risky = residual.map_action((1.0, 0.0, 0.0, 0.0, 0.0), 0.0)
    assert first.theta_candidate == repeated.theta_candidate
    assert first.residual_normalized_theta[0] == pytest.approx(0.4)
    assert risky.residual_normalized_theta[0] == pytest.approx(0.08)
    assert risky.theta_candidate[EXPECTED_THETA_ORDER[0]] < first.theta_candidate[
        EXPECTED_THETA_ORDER[0]]


def test_checked_in_residual_pilot_contract_loads_and_is_bounded():
    residual = ResidualSemanticMapping.from_files(
        ROOT / "config/thesis_experiments/A_TEB_v1.yaml",
        ROOT / "src/application/teb_rl_tuner/config/t05_simulation_safety.yaml",
        ROOT / "config/thesis_experiments/t12_residual_semantic_eta.yaml",
    )
    result = residual.map_action((1.0,) * 5, 1.0)
    for name, value in result.theta_candidate.items():
        assert residual.bounds[name][0] <= value <= residual.bounds[name][1]
