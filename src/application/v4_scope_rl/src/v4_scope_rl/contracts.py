"""Contract loading and fail-closed validation for V4-SCOPE-RL."""

import hashlib
from pathlib import Path

import yaml


class ContractError(ValueError):
    """Raised when a V4 contract violates a frozen invariant."""


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_contract(path):
    path = Path(path).resolve()
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    validate_contract(value)
    return value


def validate_contract(value):
    if value.get("architecture_generation") != "v4":
        raise ContractError("ARCHITECTURE_GENERATION_MISMATCH")
    if value.get("stage") != "V4-00":
        raise ContractError("FOUNDATION_STAGE_MISMATCH")
    scope = value["scope"]
    if not scope.get("historical_v2_v3_evidence_read_only"):
        raise ContractError("HISTORICAL_EVIDENCE_MUST_BE_READ_ONLY")
    if not scope.get("simulation_and_offline_only"):
        raise ContractError("SIMULATION_ONLY_REQUIRED")
    if not scope.get("real_vehicle_forbidden"):
        raise ContractError("REAL_VEHICLE_MUST_BE_FORBIDDEN")
    if scope.get("command_boundary_if_later_integrated") != "/cmd_vel_navigation":
        raise ContractError("COMMAND_BOUNDARY_MISMATCH")

    actions = value["decision_process"]["actions"]
    if [item["id"] for item in actions] != list(range(len(actions))):
        raise ContractError("ACTION_IDS_MUST_BE_DENSE_AND_ORDERED")
    if actions[0].get("name") != "certified_stop":
        raise ContractError("ACTION_ZERO_MUST_BE_CERTIFIED_STOP")
    if float(actions[0].get("target_speed_mps", -1.0)) != 0.0:
        raise ContractError("CERTIFIED_STOP_SPEED_MUST_BE_ZERO")

    reward = value["reward"]
    if not reward.get("safety_is_not_a_reward_tradeoff"):
        raise ContractError("SAFETY_MUST_NOT_BE_A_REWARD_TRADEOFF")
    budget = value["global_budget"]
    if int(budget["factor_rounds_maximum"]) < 1:
        raise ContractError("INVALID_FACTOR_ROUND_BUDGET")
    if int(budget["training_environment_steps_maximum"]) <= 0:
        raise ContractError("INVALID_TRAINING_BUDGET")
    if int(budget["gazebo_episode_units_maximum"]) <= 0:
        raise ContractError("INVALID_GAZEBO_BUDGET")
    return True
