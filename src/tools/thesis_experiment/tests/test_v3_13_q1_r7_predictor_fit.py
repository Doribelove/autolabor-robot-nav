#!/usr/bin/env python3
"""Static checks for the V3-13-Q1-R7 fit contract."""

from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r7_recovery_candidate_predictor_fit_contract.yaml"
)


def test_fit_scope_is_exactly_one_new_head():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    model = contract["model"]
    assert model["retained_S3_heads_byte_equivalent"] is True
    assert model["newly_fitted_mode"] == "assertive"
    assert model["newly_fitted_candidate_id"] == (
        "recovery_evasive_right_v120_r1"
    )
    assert contract["data_scope"]["selected_candidate_episodes"] == 8
    assert contract["data_scope"]["actor_trajectories"] == 16


def test_calibration_and_gates_are_not_relaxed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["calibration"]["coordinate_radius_m"] == 0.30
    assert contract["calibration"]["coordinate_growth_mps"] == 0.08
    gates = contract["internal_cross_validation"]
    assert gates["global_simultaneous_trajectory_coverage_min"] == 0.90
    assert gates["mean_trajectory_rmse_m_max"] == 0.25
    assert gates["maximum_trajectory_rmse_m_max"] == 0.65


def test_internal_fit_does_not_authorize_qualification_or_training():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["authority"][
        "deterministic_offline_supervised_fit_authorized"
    ] is True
    assert contract["authority"]["simulation_execution_authorized"] is False
    assert contract["authority"][
        "policy_or_ranker_training_authorized"
    ] is False
    assert contract["future_qualification"][
        "internal_validation_is_not_fresh_qualification"
    ] is True
    assert contract["authority"]["real_vehicle_authorized"] is False
