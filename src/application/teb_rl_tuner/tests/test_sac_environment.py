import numpy as np
import pytest
from stable_baselines3.common.env_checker import check_env

from teb_rl_tuner.config import EXPECTED_THETA_ORDER
from teb_rl_tuner.direct_theta_action import DirectThetaMapping
from teb_rl_tuner.semantic_action import FrozenSemanticMapping, ResidualSemanticMapping
from teb_rl_tuner.sac_environment import (
    DirectThetaGymEnv, ResidualSemanticEtaGymEnv, SemanticEtaGymEnv,
)
from test_training_environment import FakeAdapter, make_env


def mapping():
    matrix = [[float(row == column) for column in range(5)] for row in range(9)]
    bounds = {name: (0.0, 10.0) for name in EXPECTED_THETA_ORDER}
    return FrozenSemanticMapping(matrix, bounds, "test-v1", "b" * 64)


def test_gymnasium_contract_and_semantic_audit_fields():
    env = SemanticEtaGymEnv(make_env(FakeAdapter(), max_steps=2), mapping())
    check_env(env, warn=True)
    observation, info = env.reset(seed=42)
    assert observation.shape == env.observation_space.shape
    assert info["eta"] == (0.0,) * 5
    observation, reward, terminated, truncated, info = env.step(
        np.asarray([0.1, -0.1, 0.0, 0.0, 0.0], dtype=np.float32)
    )
    assert observation.dtype == np.float32 and np.isfinite(observation).all()
    assert np.isfinite(reward)
    assert not terminated and not truncated
    assert info["delta_eta"][0] > 0.09
    assert info["mapping_sha256"] == "b" * 64
    assert len(info["previous_applied_delta_normalized_theta"]) == 9


def test_reset_clears_eta_and_previous_action():
    env = SemanticEtaGymEnv(make_env(FakeAdapter(), max_steps=1), mapping())
    env.reset(seed=1)
    _, _, _, truncated, info = env.step(np.ones(5, dtype=np.float32) * 0.2)
    assert truncated
    observation, info = env.reset(seed=2)
    assert tuple(observation[-10:]) == (0.0,) * 10
    assert info["eta"] == (0.0,) * 5


def test_direct_theta_uses_identical_observation_shape_and_nine_actions():
    core = make_env(FakeAdapter(), max_steps=2)
    direct = DirectThetaGymEnv(
        core, DirectThetaMapping(bounds={name: (0.0, 10.0) for name in EXPECTED_THETA_ORDER},
                                 contract_version="direct-test", contract_sha256="c" * 64)
    )
    semantic = SemanticEtaGymEnv(make_env(FakeAdapter(), max_steps=2), mapping())
    check_env(direct, warn=True)
    assert direct.observation_space.shape == semantic.observation_space.shape == (18,)
    assert direct.action_space.shape == (9,)
    observation, _ = direct.reset(seed=42)
    observation, reward, terminated, truncated, info = direct.step(
        np.asarray([0.1, -0.1] + [0.0] * 7, dtype=np.float32)
    )
    assert observation.shape == direct.observation_space.shape and np.isfinite(observation).all()
    assert np.isfinite(reward) and not terminated and not truncated
    assert info["delta_normalized_theta"][0] > 0.09
    assert info["direct_theta_contract_sha256"] == "c" * 64


def test_both_policies_observe_same_applied_theta_delta_context():
    semantic = SemanticEtaGymEnv(make_env(FakeAdapter(), max_steps=2), mapping())
    direct = DirectThetaGymEnv(
        make_env(FakeAdapter(), max_steps=2),
        DirectThetaMapping({name: (0.0, 10.0) for name in EXPECTED_THETA_ORDER},
                           "direct-test", "d" * 64),
    )
    semantic.reset(seed=1)
    direct.reset(seed=1)
    semantic_obs, *_ = semantic.step(np.asarray([0.1, 0.0, 0.0, 0.0, 0.0], dtype=np.float32))
    direct_obs, *_ = direct.step(np.asarray([0.1] + [0.0] * 8, dtype=np.float32))
    assert tuple(semantic_obs[-10:]) == tuple(direct_obs[-10:])


def test_residual_semantic_env_preserves_shape_and_does_not_accumulate_action():
    base = mapping()
    residual = ResidualSemanticMapping(
        base, {name: 5.0 for name in EXPECTED_THETA_ORDER}, (0.4,) * 9, 0.2)
    env = ResidualSemanticEtaGymEnv(make_env(FakeAdapter(), max_steps=3), residual)
    check_env(env, warn=True)
    env.reset(seed=3)
    action = np.asarray([0.5, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    _, _, _, _, first = env.step(action)
    _, _, _, _, second = env.step(action)
    assert first["residual_theta_candidate"] == second["residual_theta_candidate"]
    assert first["residual_risk_scale"] == pytest.approx(0.2)
    assert env.observation_space.shape == (18,)
