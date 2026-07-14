import copy
from pathlib import Path

import pytest
import yaml

from thesis_experiment.v2_04_acceptance import run_v2_04_acceptance
from thesis_experiment.v2_contract import V2ContractError, validate_action_pipeline_contract


WORKSPACE = Path(__file__).resolve().parents[4]
CONTRACT_PATH = WORKSPACE / "config/thesis_experiments/v2/action_pipeline_contract.yaml"
ANCHOR_BANK = (
    WORKSPACE
    / "src/application/teb_mode_manager/config/v2_04_anchor_bank_candidate.yaml"
)


def load_contract():
    return yaml.safe_load(CONTRACT_PATH.read_text(encoding="utf-8"))


def test_v2_04_contract_and_profiles_validate_but_remain_not_runtime_ready():
    contract = load_contract()
    assert validate_action_pipeline_contract(
        contract, workspace=WORKSPACE, verify_profiles=True
    ) is contract
    assert contract["runtime_ready"] is False
    assert contract["training_allowed"] is False
    assert contract["parameter_transaction"]["real_parameter_write_enabled"] is False


def test_v2_04_contract_fails_closed_on_scope_or_semantic_drift():
    mutations = (
        lambda data: data.update(runtime_ready=True),
        lambda data: data.update(training_allowed=True),
        lambda data: data["parameter_transaction"].update(transition_origin="new_anchor"),
        lambda data: data["parameter_transaction"].update(dynamic_reconfigure_backend_enabled=True),
        lambda data: data["rule_closed_loop"].update(learned_policy_loaded=True),
        lambda data: data["rule_closed_loop"].update(runtime_scene_labels_allowed=True),
        lambda data: data["acceptance_gates"].update(normal_projection_rate_max=0.11),
        lambda data: data["claims"].update(navigation_performance_improved=True),
    )
    for mutation in mutations:
        changed = copy.deepcopy(load_contract())
        mutation(changed)
        with pytest.raises(V2ContractError):
            validate_action_pipeline_contract(changed)


def test_v2_04_offline_rule_loop_meets_projection_continuity_and_trace_gates():
    report = run_v2_04_acceptance(ANCHOR_BANK)
    assert report["status"] == "passed"
    normal = report["normal_rule_loop"]
    assert normal["transaction_count"] == 800
    assert normal["projection_rate"] < 0.10
    assert normal["continuous_jump_count"] == 0
    assert normal["maximum_continuous_rate_ratio"] <= 1.0 + 1.0e-9
    assert normal["complete_trace_reconstruction"] is True
    assert normal["all_training_used_false"] is True
    assert all(item["held_previous_executed"] for item in report["fault_atomicity"])
    assert report["profile_type_counts"] == {"double": 18, "int": 1, "bool": 1}


def test_v2_04_ros_message_and_node_preserve_full_action_semantics():
    message = (
        WORKSPACE / "src/application/teb_mode_manager/msg/ParameterTransaction.msg"
    ).read_text(encoding="utf-8")
    for field in (
        "uint64 world_model_seq", "string[] parameter_types", "float64[] commanded",
        "float64[] feasible", "float64[] safe", "float64[] executed",
        "string execution_backend", "bool slow_profile_committed", "bool training_used",
    ):
        assert field in message
    node = (
        WORKSPACE
        / "src/application/teb_mode_manager/scripts/rule_anchor_transaction_node.py"
    ).read_text(encoding="utf-8")
    assert "RuleAnchorTransactionLoop" in node
    assert "allow_dynamic_reconfigure" in node
    assert "dynamic_reconfigure.client" not in node
    assert "/cmd_vel" not in node
    assert "/gazebo/model_states" not in node
    assert "scene" not in node.lower()
