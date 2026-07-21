from pathlib import Path

import gymnasium as gym
import numpy as np
import pytest
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from teb_rl_tuner.sac_training import (
    SacCheckpointManager, SacTrainingConfig, SacTrainingError, build_sac,
)


class TinyContinuousEnv(gym.Env):
    def __init__(self):
        self.observation_space = gym.spaces.Box(-10, 10, shape=(3,), dtype=np.float32)
        self.action_space = gym.spaces.Box(-1, 1, shape=(5,), dtype=np.float32)
        self.steps = 0

    def reset(self, *, seed=None, options=None):
        super().reset(seed=seed)
        self.steps = 0
        return np.zeros(3, dtype=np.float32), {}

    def step(self, action):
        self.steps += 1
        observation = np.asarray([self.steps, action[0], action[1]], dtype=np.float32)
        reward = -float(np.square(action).sum())
        return observation, reward, False, self.steps >= 8, {}


def vec_env():
    return DummyVecEnv([TinyContinuousEnv])


def test_cpu_sac_checkpoint_resume_and_hash_rejection(tmp_path):
    vec = VecNormalize(vec_env(), norm_obs=True, norm_reward=False)
    config = SacTrainingConfig(
        seed=7, buffer_size=128, learning_starts=4, batch_size=8,
        net_arch=(16, 16), device="cpu",
    )
    model = build_sac(vec, config)
    model.learn(total_timesteps=24)
    manager = SacCheckpointManager(tmp_path / "checkpoint")
    manager.save(model, vec, config, {"test": True})
    loaded, loaded_vec, state = manager.load(vec_env())
    assert state["num_timesteps"] == 24
    assert loaded.replay_buffer.size() > 0
    loaded.learn(total_timesteps=8, reset_num_timesteps=False)
    assert loaded.num_timesteps == 32
    observation = loaded_vec.reset()
    action, _ = loaded.predict(observation, deterministic=True)
    assert action.shape == (1, 5) and np.isfinite(action).all()

    model_path = tmp_path / "checkpoint/model.zip"
    with model_path.open("ab") as handle:
        handle.write(b"tampered")
    with pytest.raises(SacTrainingError, match="hash mismatch"):
        manager.validate()


def test_non_cpu_training_config_is_rejected():
    with pytest.raises(SacTrainingError, match="cpu"):
        SacTrainingConfig(device="cuda")


def test_checkpoint_algorithm_identity_is_enforced(tmp_path):
    vec = VecNormalize(vec_env(), norm_obs=True, norm_reward=False)
    config = SacTrainingConfig(
        seed=3, buffer_size=32, learning_starts=1, batch_size=2,
        net_arch=(8,), device="cpu",
    )
    model = build_sac(vec, config)
    model.learn(total_timesteps=2)
    semantic = SacCheckpointManager(tmp_path / "checkpoint")
    semantic.save(model, vec, config)
    direct = SacCheckpointManager(
        tmp_path / "checkpoint", algorithm="RL-TEB-Direct-Theta"
    )
    with pytest.raises(SacTrainingError, match="algorithm mismatch"):
        direct.validate()
