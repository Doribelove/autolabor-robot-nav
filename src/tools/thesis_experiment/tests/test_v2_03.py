import copy
from pathlib import Path

import pytest
import yaml

from thesis_experiment import (
    V2ContractError,
    load_v2_yaml,
    validate_mode_thresholds,
    validate_world_model_contract,
)
from thesis_experiment.v2_03_acceptance import evaluate_v2_03_synthetic


WORKSPACE = Path(__file__).resolve().parents[4]
CONTRACT = WORKSPACE / "config/thesis_experiments/v2/world_model_contract.yaml"
MODE_THRESHOLDS = WORKSPACE / "config/thesis_experiments/v2/mode_thresholds.yaml"
WORLD_CONFIG = WORKSPACE / "src/perception/nav_world_model/config/v2_03_candidate.yaml"
SUPERVISOR_CONFIG = WORKSPACE / "src/application/teb_mode_manager/config/v2_03_rule_candidate.yaml"


def test_v2_03_contract_and_profiles_are_valid_but_not_runtime_ready():
    contract = load_v2_yaml(CONTRACT)
    assert validate_world_model_contract(
        contract, workspace=WORKSPACE, verify_profiles=True
    ) is contract
    assert contract["runtime_ready"] is False
    thresholds = load_v2_yaml(MODE_THRESHOLDS)
    validate_mode_thresholds(thresholds)
    assert thresholds["runtime_ready"] is False


def test_v2_03_contract_rejects_truth_leakage_and_output_authority():
    contract = load_v2_yaml(CONTRACT)
    leaked = copy.deepcopy(contract)
    leaked["runtime_inputs"]["truth_used_by_policy"] = True
    with pytest.raises(V2ContractError):
        validate_world_model_contract(leaked)
    authority = copy.deepcopy(contract)
    authority["rule_supervisor"]["publishes_velocity_commands"] = True
    with pytest.raises(V2ContractError):
        validate_world_model_contract(authority)


def test_runtime_nodes_do_not_subscribe_to_truth_or_read_scene_labels():
    sources = [
        WORKSPACE / "src/perception/nav_world_model/scripts/nav_world_model_node.py",
        WORKSPACE / "src/application/teb_mode_manager/scripts/rule_context_supervisor_node.py",
    ]
    forbidden = ("/gazebo/model_states", "/pedsim_simulator/simulated_agents",
                 "scene_id", "['family']", '["family"]')
    for source in sources:
        text = source.read_text(encoding="utf-8")
        for token in forbidden:
            assert token not in text


def test_unfrozen_candidate_launches_are_opt_in_and_fail_closed_by_default():
    launches = [
        WORKSPACE / "src/perception/nav_world_model/launch/v2_03_world_model.launch",
        WORKSPACE / "src/application/teb_mode_manager/launch/v2_03_rule_supervisor.launch",
    ]
    for launch in launches:
        text = launch.read_text(encoding="utf-8")
        assert 'name="allow_unfrozen_simulation_candidate" default="false"' in text
        assert "parameter_write" not in text
        assert "cmd_vel" not in text


def test_synthetic_acceptance_meets_frozen_component_gates():
    report = evaluate_v2_03_synthetic(
        load_v2_yaml(WORLD_CONFIG),
        load_v2_yaml(SUPERVISOR_CONFIG),
        load_v2_yaml(CONTRACT),
    )
    assert report["passed"] is True
    assert report["tracking"]["id_switches"] == 0
    assert report["mode_supervisor"]["macro_recall"] == 1.0
    assert report["health"]["fault_cases_passed"] == 4
    assert report["runtime_ready"] is False
