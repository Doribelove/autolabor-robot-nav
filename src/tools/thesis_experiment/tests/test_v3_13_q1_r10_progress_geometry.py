#!/usr/bin/env python3
"""Static tests for R10 progress-relative recovery geometry."""

from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r10_progress_recovery_geometry_contract.yaml"
)


def test_single_factor_is_progress_phase():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    factor = contract["single_architectural_factor"]
    assert (
        factor["factor"]
        == "replace_R4_R1_absolute_world_x_phase_with_recovery_relative_progress_phase"
    )
    assert factor["legacy_candidate_bytes_and_order_unchanged"] is True
    assert factor["analytic_ranker_weights_unchanged"] is True
    assert factor["hard_verifier_thresholds_unchanged"] is True


def test_geometry_is_inside_m2_simulation_envelope():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    candidate = contract["candidate_extension"]
    envelope = contract["hardware_envelope"]
    assert candidate["phase_coordinate"] == (
        "longitudinal_progress_from_recovery_pose_m"
    )
    assert candidate["steering_peak_rad"] <= envelope[
        "maximum_steering_angle_rad"
    ]
    assert candidate["steering_rate_limit_radps"] <= envelope[
        "maximum_steering_rate_radps"
    ]
    assert candidate["longitudinal_acceleration_limit_mps2"] <= envelope[
        "maximum_acceleration_mps2"
    ]


def test_static_and_surrogate_dynamic_margins_remain_positive():
    gates = yaml.safe_load(CONTRACT.read_bytes())[
        "offline_geometry_gates"
    ]
    assert gates[
        "every_R8_state_surrogate_dynamic_clearance_m_min"
    ] == 0.05
    assert gates["every_R8_state_static_clearance_m_min"] == 0.10
    assert gates["translation_normalized_geometry_residual_max"] == 1.0e-9


def test_execution_training_and_hardware_are_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["authority"]["ROS_or_Gazebo_execution_authorized"] is False
    assert contract["downstream"][
        "candidate_specific_support_authorized"
    ] is False
    assert contract["downstream"][
        "direct_actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
