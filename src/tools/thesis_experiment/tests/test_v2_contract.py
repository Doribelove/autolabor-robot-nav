import copy
import re
import tempfile
from pathlib import Path

import pytest
import yaml

from thesis_experiment.v2_contract import (
    V2ContractError,
    require_v1_resource,
    validate_architecture_contract,
    validate_mode_thresholds,
    validate_parameter_registry,
    validate_state_contract,
    validate_v1_baseline_snapshot,
)


ROOT = Path(__file__).resolve().parents[4]
CONFIG_ROOT = ROOT / "config/thesis_experiments/v2"


def _load(name):
    return yaml.safe_load((CONFIG_ROOT / name).read_text(encoding="utf-8"))


def test_checked_in_v2_foundation_contracts_are_valid_but_not_runtime_ready():
    architecture = _load("architecture_contract.yaml")
    registry = _load("parameter_registry.yaml")
    thresholds = _load("mode_thresholds.yaml")
    state = _load("state_contract.yaml")

    validate_architecture_contract(architecture)
    validate_parameter_registry(registry)
    validate_mode_thresholds(thresholds)
    validate_state_contract(state)

    assert registry["runtime_ready"] is False
    assert thresholds["runtime_ready"] is False
    assert state["runtime_ready"] is False
    assert architecture["runtime"]["allow_training"] is False
    assert architecture["runtime"]["allow_motion"] is False
    assert architecture["runtime"]["allow_parameter_write"] is False


def test_unfrozen_v2_design_contracts_fail_runtime_ready_gate():
    cases = (
        ("parameter_registry.yaml", validate_parameter_registry),
        ("mode_thresholds.yaml", validate_mode_thresholds),
        ("state_contract.yaml", validate_state_contract),
    )
    for name, validator in cases:
        with pytest.raises(V2ContractError, match="not runtime-ready"):
            validator(_load(name), require_runtime_ready=True)


def test_v2_contract_rejects_enum_action_and_extra_key_drift():
    architecture = _load("architecture_contract.yaml")

    changed = copy.deepcopy(architecture)
    changed["enums"]["geometry_mode"]["CRUISE"] = 9
    with pytest.raises(V2ContractError, match="GeometryMode"):
        validate_architecture_contract(changed)

    changed = copy.deepcopy(architecture)
    changed["action_execution"]["stages"] = [
        "commanded", "safe", "feasible", "executed"]
    with pytest.raises(V2ContractError, match="four-stage"):
        validate_architecture_contract(changed)

    changed = copy.deepcopy(architecture)
    changed["unexpected"] = True
    with pytest.raises(V2ContractError, match="extra"):
        validate_architecture_contract(changed)


def test_parameter_registry_rejects_duplicates_and_lifecycle_drift():
    registry = _load("parameter_registry.yaml")
    changed = copy.deepcopy(registry)
    changed["fast_continuous"][1]["name"] = "max_vel_x"
    with pytest.raises(V2ContractError, match="names/order|duplicate"):
        validate_parameter_registry(changed)

    changed = copy.deepcopy(registry)
    changed["startup_structural"][0]["online_support"] = True
    with pytest.raises(V2ContractError, match="startup parameter"):
        validate_parameter_registry(changed)


def test_state_contract_requires_scan_angles_rear_coverage_and_four_actions():
    state = _load("state_contract.yaml")
    scan = state["laser_scan"]
    assert scan["metadata_required"] == [
        "stamp", "frame_id", "angle_min", "angle_max", "angle_increment",
        "range_min", "range_max", "ray_count",
    ]
    assert scan["directional_coverage"]["rear_required_for_reverse"] is True
    assert state["action_context"]["stages"] == [
        "commanded", "feasible", "safe", "executed"]

    changed = copy.deepcopy(state)
    changed["laser_scan"]["metadata_required"].remove("angle_increment")
    with pytest.raises(V2ContractError, match="metadata"):
        validate_state_contract(changed)


def test_v1_snapshot_preserves_t11_t12_hashes_and_runner_resources():
    snapshot = _load("v1_frozen_baseline.yaml")
    validate_v1_baseline_snapshot(snapshot, ROOT, verify_evidence=True)
    assert len(snapshot["submodules"]) == 13
    assert len(snapshot["frozen_evidence"]) >= 16


def test_v1_resource_guard_rejects_v2_path_or_marker():
    architecture_path = CONFIG_ROOT / "architecture_contract.yaml"
    architecture = _load("architecture_contract.yaml")
    with pytest.raises(V2ContractError, match="V2 namespace"):
        require_v1_resource(architecture_path, architecture, "test")

    with tempfile.TemporaryDirectory() as directory:
        disguised = Path(directory) / "legacy_name.yaml"
        disguised.write_text(
            "schema_version: '2.0'\narchitecture_generation: v2\n",
            encoding="utf-8",
        )
        with pytest.raises(V2ContractError, match="V2 marker"):
            require_v1_resource(disguised, architecture, "test")


def test_t11_t12_runner_loads_yaml_only_through_v1_guard():
    source = (
        ROOT / "src/tools/thesis_experiment/src/thesis_experiment/t11_training.py"
    ).read_text(encoding="utf-8")
    assert source.count("load_v1_yaml(") >= 6
    assert "yaml.safe_load(" not in source


def test_ros_message_enums_and_action_fields_match_machine_contract():
    architecture = _load("architecture_contract.yaml")
    context = (
        ROOT / "src/application/teb_mode_manager/msg/ContextState.msg"
    ).read_text(encoding="utf-8")
    constants = {
        name: int(value)
        for name, value in re.findall(r"^uint8 ([A-Z0-9_]+)=(\d+)$", context, re.MULTILINE)
    }
    for name, value in architecture["enums"]["geometry_mode"].items():
        assert constants["GEOMETRY_" + name] == value
    for name, value in architecture["enums"]["dynamic_overlay"].items():
        assert constants["DYNAMIC_" + name] == value
    for name, value in architecture["enums"]["transition_state"].items():
        assert constants["TRANSITION_" + name] == value

    transaction = (
        ROOT / "src/application/teb_mode_manager/msg/ParameterTransaction.msg"
    ).read_text(encoding="utf-8")
    for stage in architecture["action_execution"]["stages"]:
        assert "float64[] {}".format(stage) in transaction


def test_skeleton_nodes_are_fail_closed_and_do_not_publish_motion():
    world = (
        ROOT / "src/perception/nav_world_model/scripts/nav_world_model_skeleton_node.py"
    ).read_text(encoding="utf-8")
    manager = (
        ROOT
        / "src/application/teb_mode_manager/scripts/teb_mode_manager_skeleton_node.py"
    ).read_text(encoding="utf-8")
    assert "message.valid = False" in world
    assert "message.valid = False" in manager
    assert "TRANSITION_FAULTED" in manager
    assert "/cmd_vel" not in world + manager
    assert "set_parameters" not in world + manager
