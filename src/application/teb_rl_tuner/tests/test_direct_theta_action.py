from pathlib import Path

import pytest

from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.direct_theta_action import DirectThetaActionError, DirectThetaMapping


ROOT = Path(__file__).resolve().parents[4]


def bounds():
    return {name: (0.0, 10.0) for name in EXPECTED_THETA_ORDER}


def theta(value=5.0):
    return {name: float(value) for name in EXPECTED_THETA_ORDER}


def test_direct_action_is_normalized_theta_delta_in_frozen_order():
    mapping = DirectThetaMapping(bounds(), "direct-test", "a" * 64)
    action = (0.2, -0.4) + (0.0,) * 7
    result = mapping.map_action(theta(), action)
    assert result.delta_normalized_theta == action
    assert result.theta_candidate[EXPECTED_THETA_ORDER[0]] == pytest.approx(6.0)
    assert result.theta_candidate[EXPECTED_THETA_ORDER[1]] == pytest.approx(3.0)
    assert result.clipped_theta is False


def test_direct_action_clips_normalized_candidate_and_audits_it():
    mapping = DirectThetaMapping(bounds(), "direct-test", "b" * 64)
    result = mapping.map_action(theta(9.5), (0.5,) * 9)
    assert result.clipped_theta is True
    assert all(value == pytest.approx(10.0) for value in result.theta_candidate.values())


@pytest.mark.parametrize("action", [(0.0,) * 8, (float("nan"),) * 9, (1.1,) * 9])
def test_direct_action_rejects_wrong_nonfinite_or_out_of_range_values(action):
    with pytest.raises(DirectThetaActionError):
        DirectThetaMapping(bounds(), "direct-test", "c" * 64).map_action(theta(), action)


def test_checked_in_safety_contract_is_simulation_only_and_hash_pinned():
    mapping = DirectThetaMapping.from_file(
        ROOT / "src/application/teb_rl_tuner/config/t05_simulation_safety.yaml"
    )
    assert mapping.contract_version == "direct_theta_v1"
    assert mapping.contract_sha256 == (
        "f7c258a538a0e0baa7946c0c6b39dd26c51a43eb26d2e0fa78832978ece9e797"
    )


def test_t11_executable_rate_scaling_maps_unit_action_to_rate_limit():
    mapping = DirectThetaMapping.from_file(
        ROOT / "src/application/teb_rl_tuner/config/t05_simulation_safety.yaml",
        executable_rate_scaling=True,
    )
    current = {
        name: sum(mapping.bounds[name]) / 2.0 for name in EXPECTED_THETA_ORDER
    }
    result = mapping.map_action(current, (1.0,) * 9)
    for index, name in enumerate(EXPECTED_THETA_ORDER):
        expected_physical_delta = (
            0.5 * result.delta_normalized_theta[index] *
            (mapping.bounds[name][1] - mapping.bounds[name][0])
        )
        safety = __import__("yaml").safe_load(
            (ROOT / "src/application/teb_rl_tuner/config/t05_simulation_safety.yaml")
            .read_text()
        )
        assert expected_physical_delta == pytest.approx(
            safety["max_delta_per_step"][name]
        )
