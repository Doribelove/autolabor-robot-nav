import numpy as np
import pytest

from v4_scope_rl.fast_env import CounterfactualCrossingEnv


def test_observation_and_mask_contract():
    env = CounterfactualCrossingEnv()
    observation, info = env.reset(seed=17)
    assert env.observation_space.contains(observation)
    assert info["action_mask"].shape == (env.action_space.n,)
    assert bool(info["action_mask"][0])


def test_masked_action_is_rejected_without_projection():
    env = CounterfactualCrossingEnv()
    env.reset(seed=3)
    rejected = np.flatnonzero(~env.action_masks())
    assert len(rejected) > 0
    with pytest.raises(ValueError, match="ACTION_REJECTED_BY_HARD_MASK"):
        env.step(int(rejected[0]))


def test_analytic_rollouts_are_contact_free_and_complete():
    for seed in range(100, 160):
        env = CounterfactualCrossingEnv()
        env.reset(seed=seed)
        while True:
            _, _, terminated, truncated, info = env.step(env.analytic_action())
            if terminated or truncated:
                break
        assert info["contact"] is False
        assert info["mask_violation_count"] == 0
        assert info["goal_reached"] is True


def test_clone_preserves_counterfactual_state():
    env = CounterfactualCrossingEnv()
    env.reset(seed=55)
    clone = env.clone()
    action = env.analytic_action()
    left = env.step(action)
    right = clone.step(action)
    np.testing.assert_allclose(left[0], right[0], atol=0.0, rtol=0.0)
    assert left[1:4] == right[1:4]
    np.testing.assert_array_equal(left[4]["action_mask"], right[4]["action_mask"])
    left_info = dict(left[4])
    right_info = dict(right[4])
    left_info.pop("action_mask")
    right_info.pop("action_mask")
    assert left_info == right_info
