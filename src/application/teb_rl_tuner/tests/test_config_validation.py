from pathlib import Path

import pytest

from teb_rl_tuner import (
    ConfigValidationError,
    load_yaml_mapping,
    validate_a_teb,
    validate_experiment_contract,
    validate_runtime_config,
)


ROOT = Path(__file__).resolve().parents[4]


def test_repository_contract_is_valid():
    data = load_yaml_mapping(ROOT / "docs/thesis_experiment/experiment_contract.yaml")
    validate_experiment_contract(data)


def test_runtime_defaults_are_shadow_only():
    data = load_yaml_mapping(ROOT / "src/application/teb_rl_tuner/config/runtime_defaults.yaml")
    validate_runtime_config(data)
    assert data["safety"]["allow_motion"] is False
    assert data["safety"]["allow_parameter_write"] is False


def test_a_teb_template_allows_null_before_freeze():
    data = load_yaml_mapping(ROOT / "docs/thesis_experiment/templates/A_TEB.template.yaml")
    validate_a_teb(data)
    with pytest.raises(ConfigValidationError):
        validate_a_teb(data, require_frozen=True)


def test_contract_rejects_real_workspace_selection():
    data = load_yaml_mapping(ROOT / "docs/thesis_experiment/experiment_contract.yaml")
    data["environment"]["workspace"] = "/home/robot/robot_ws"
    with pytest.raises(ConfigValidationError):
        validate_experiment_contract(data)
