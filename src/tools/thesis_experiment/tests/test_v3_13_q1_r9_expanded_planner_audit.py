#!/usr/bin/env python3
"""Static tests for the pure-offline R9 integration audit."""

from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r9_expanded_planner_integration_contract.yaml"
)


def test_audit_uses_all_fresh_r8_r2_blocks():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    population = contract["audit_population"]
    assert population["source_blocks"] == list(range(1, 9))
    assert population["candidate_count"] == 5
    assert population["hidden_policy_runtime_input"] is False


def test_static_and_dynamic_safety_are_both_required():
    gates = yaml.safe_load(CONTRACT.read_bytes())[
        "integration_gates"
    ]
    assert gates["all_new_candidate_dynamic_verdicts_accepted"] is True
    assert gates["all_new_candidate_static_verdicts_accepted"] is True
    assert gates[
        "all_selected_clone_outcomes_static_containment"
    ] is True
    assert gates["no_safe_candidate_exception_count_max"] == 0


def test_failure_cannot_lower_static_gate_or_release_actual_teb():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    forbidden = contract["failure_routing"]["forbidden_repairs"]
    assert "lower_or_remove_static_containment_gate" in forbidden
    assert contract["downstream"][
        "direct_actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False


def test_audit_is_pure_offline_and_hardware_closed():
    authority = yaml.safe_load(CONTRACT.read_bytes())["authority"]
    assert authority["pure_offline_audit_authorized"] is True
    assert authority["ROS_or_Gazebo_execution_authorized"] is False
    assert authority["hardware_device_access_authorized"] is False
    assert authority["real_vehicle_authorized"] is False
