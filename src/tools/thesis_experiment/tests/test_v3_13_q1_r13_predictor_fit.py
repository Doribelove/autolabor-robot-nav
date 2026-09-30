#!/usr/bin/env python3
"""Static checks for the R13 progress-candidate head fit."""

from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r13_progress_predictor_fit_contract.yaml"
)


def test_only_progress_candidate_assertive_head_is_fitted():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["data_scope"]["source_stage"] == "V3-13-Q1-R12"
    assert contract["model"]["newly_fitted_mode"] == "assertive"
    assert contract["model"]["newly_fitted_candidate_id"] == (
        "recovery_evasive_right_v120_r2"
    )
    assert contract["model"]["retained_S3_heads_byte_equivalent"] is True


def test_internal_cv_is_block_held_out_and_fixed():
    cv = yaml.safe_load(CONTRACT.read_bytes())[
        "internal_cross_validation"
    ]
    assert cv["split"] == "leave_one_matched_context_block_out"
    assert cv["folds"] == 8
    assert cv["global_simultaneous_trajectory_coverage_min"] == 0.90
    assert cv["maximum_trajectory_rmse_m_max"] == 0.65
    assert cv["fixed_before_fit"] is True


def test_fresh_qualification_is_still_required():
    future = yaml.safe_load(CONTRACT.read_bytes())[
        "future_qualification"
    ]
    assert future["internal_validation_is_not_fresh_qualification"] is True
    assert future["fresh_seed_range_reserved_not_authorized"] == [
        7650,
        7657,
    ]
    assert future["exact_future_units"] == 40


def test_simulation_training_and_hardware_are_closed():
    authority = yaml.safe_load(CONTRACT.read_bytes())["authority"]
    assert authority["simulation_execution_authorized"] is False
    assert authority["policy_or_ranker_training_authorized"] is False
    assert authority["hardware_device_access_authorized"] is False
    assert authority["real_vehicle_authorized"] is False
