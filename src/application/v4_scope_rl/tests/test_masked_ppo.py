import numpy as np
import torch

from v4_scope_rl.fast_env import CounterfactualCrossingEnv
from v4_scope_rl.masked_ppo import MaskedPPO, PPOConfig, masked_logits


def test_masked_logits_assign_zero_probability_to_rejected_action():
    logits = torch.tensor([[1.0, 2.0, 3.0]])
    mask = torch.tensor([[True, False, True]])
    probabilities = torch.softmax(masked_logits(logits, mask), dim=-1)
    assert probabilities[0, 1].item() == 0.0
    assert torch.isclose(probabilities.sum(), torch.tensor(1.0))


def test_smoke_training_and_checkpoint_round_trip(tmp_path):
    env = CounterfactualCrossingEnv()
    config = PPOConfig(
        environment_count=2,
        rollout_steps=16,
        update_epochs=1,
        minibatch_size=16,
        hidden_size=16,
    )
    model = MaskedPPO(
        env.observation_space.shape[0], env.action_space.n, config, seed=8
    )
    result = model.train(CounterfactualCrossingEnv, 64)
    assert result["environment_steps"] == 64
    observation, _ = env.reset(seed=19)
    action = model.predict(observation, env.action_masks())
    assert env.action_masks()[action]
    path = tmp_path / "model.pt"
    model.save(path, metadata={"test": True})
    restored, metadata = MaskedPPO.load(path)
    assert metadata == {"test": True}
    assert restored.predict(observation, env.action_masks()) == action
