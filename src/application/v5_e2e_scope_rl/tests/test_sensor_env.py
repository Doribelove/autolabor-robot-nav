import numpy as np
import torch

from v5_e2e_scope_rl.asymmetric_ppo import (
    POLICY_OBSERVATION_KEYS,
    AsymmetricActorCritic,
    SensorActor,
    observations_to_torch,
    stack_observations,
)
from v5_e2e_scope_rl.sensor_env import FAMILY_NAMES, SensorCandidateEnv


def test_policy_and_privileged_schemas_are_separate():
    environment = SensorCandidateEnv()
    observation, _ = environment.reset(seed=71)
    assert set(observation) == set(POLICY_OBSERVATION_KEYS)
    assert environment.observation_space.contains(observation)
    assert environment.privileged_observation().shape == (32,)


def test_certified_stop_is_always_available():
    for family in FAMILY_NAMES:
        environment = SensorCandidateEnv({"family": family})
        observation, _ = environment.reset(seed=72)
        for _ in range(20):
            assert observation["action_mask"][0] == 1
            action = environment.analytic_action()
            observation, _, terminated, truncated, _ = environment.step(action)
            if terminated or truncated:
                break


def test_actor_only_checkpoint_has_no_privileged_module(tmp_path):
    environment = SensorCandidateEnv()
    observation, _ = environment.reset(seed=73)
    model = AsymmetricActorCritic(asymmetric_critic=True)
    path = tmp_path / "actor.pt"
    torch.save(model.actor.actor_checkpoint(), path)
    checkpoint = torch.load(path, weights_only=False)
    assert checkpoint["contains_privileged_modules"] is False
    assert not any("privileg" in name for name in checkpoint["state_dict"])
    actor = SensorActor.from_actor_checkpoint(checkpoint)
    batch = observations_to_torch(
        stack_observations([observation]), torch.device("cpu")
    )
    actor.choose_actions(batch, advantage_gate_margin=0.10)
    bad = dict(batch)
    bad["privileged"] = torch.zeros(1, 32)
    try:
        actor(bad)
    except ValueError as error:
        assert str(error) == "ACTOR_OBSERVATION_SCHEMA_VIOLATION"
    else:
        raise AssertionError("privileged key was not rejected")


def test_seeded_clone_replays_exactly():
    environment = SensorCandidateEnv({"family": FAMILY_NAMES[1]})
    observation, _ = environment.reset(seed=74)
    clone = environment.clone()
    for _ in range(8):
        action = environment.analytic_action()
        left = environment.step(action)
        right = clone.step(action)
        for key in POLICY_OBSERVATION_KEYS:
            np.testing.assert_array_equal(left[0][key], right[0][key])
        assert left[1] == right[1]
        assert left[2] == right[2]
        assert left[3] == right[3]
        assert left[4].keys() == right[4].keys()
        for key in left[4]:
            if isinstance(left[4][key], np.ndarray):
                np.testing.assert_array_equal(left[4][key], right[4][key])
            else:
                assert left[4][key] == right[4][key]
        observation = left[0]
