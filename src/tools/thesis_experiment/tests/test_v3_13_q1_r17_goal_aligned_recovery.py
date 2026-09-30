#!/usr/bin/env python3
"""Tests for the R17 goal-aligned recovery closure."""

import importlib.util
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r17_goal_aligned_recovery_contract.yaml"
)
BUILDER = W / (
    "src/tools/thesis_experiment/scripts/"
    "build_v3_13_q1_r17_goal_aligned_recovery.py"
)


def load_builder():
    spec = importlib.util.spec_from_file_location(
        "v3_13_q1_r17_builder_test", BUILDER
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_single_factor_keeps_frozen_gates():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    factor = contract["single_architectural_factor"]
    assert factor["factor"] == "end_to_end_recovery_completion"
    assert factor["analytic_ranker_weights_unchanged"] is True
    assert factor["hard_verifier_thresholds_unchanged"] is True
    assert factor[
        "common_goal_timeout_or_safety_thresholds_changed"
    ] is False


def test_goal_aligned_candidate_parameters_are_exact():
    candidate = yaml.safe_load(CONTRACT.read_bytes())[
        "candidate_extension"
    ]
    assert candidate["candidate_id"] == (
        "recovery_evasive_right_v120_r3"
    )
    assert candidate["steering_peak_rad"] == 0.434
    assert candidate["outbound_progress_range_m"] == [0.0, 3.04]
    assert candidate["return_progress_range_m"] == [3.04, 6.58]
    assert candidate["horizon_s"] == 7.55


def test_offline_audit_passes_goal_safety_and_terminal_guard():
    builder = load_builder()
    audit = yaml.safe_load(
        builder.build_outputs()[builder.AUDIT]
    )
    assert audit["offline_design_pass"] is True
    assert audit["design_state_count"] == 9
    assert audit["maximum_terminal_common_goal_distance_m"] <= 0.05
    assert audit["minimum_surrogate_dynamic_clearance_m"] >= 0.05
    assert audit["minimum_static_clearance_m"] >= 0.05
    assert audit["gate_checks"][
        "terminal_guard_zero_at_terminal"
    ] is True
    assert audit["gate_checks"][
        "terminal_guard_zero_beyond_terminal"
    ] is True
    assert audit["gate_checks"][
        "terminal_guard_preserves_approach"
    ] is True


def test_simulation_training_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["authority"]["ROS_or_Gazebo_execution_authorized"] is False
    assert contract["downstream"][
        "candidate_specific_support_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
