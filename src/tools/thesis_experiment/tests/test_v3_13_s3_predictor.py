#!/usr/bin/env python3

from pathlib import Path

import yaml


ROOT = Path("/home/robot/robot_ws_base_rl")
CONTRACT = yaml.safe_load(
    (
        ROOT
        / "config/thesis_experiments/v3/"
        "v3_13_s3_recovery_predictor_fit_contract.yaml"
    ).read_text(encoding="utf-8")
)


def test_only_assertive_heads_are_new():
    model = CONTRACT["model"]
    assert model["retained_frozen_modes"] == ["cautious", "hesitant"]
    assert model["newly_fitted_modes"] == ["assertive"]
    assert len(model["candidate_ids"]) == 4


def test_calibration_is_not_widened():
    calibration = CONTRACT["calibration"]
    assert calibration["coordinate_radius_m"] == 0.30
    assert calibration["coordinate_growth_mps"] == 0.08
    assert calibration["terminal_full_width_at_7s_m"] == 1.72
    assert calibration["retained_exactly_from_C2"]


def test_internal_validation_is_not_fresh_qualification():
    assert CONTRACT["internal_cross_validation"]["folds"] == 8
    assert CONTRACT["future_qualification"][
        "S3_internal_validation_is_not_fresh_qualification"
    ]


def test_fit_is_not_policy_training():
    authority = CONTRACT["authority"]
    assert authority["deterministic_offline_supervised_fit_authorized"]
    assert not authority["simulation_execution_authorized"]
    assert not authority["policy_or_ranker_training_authorized"]
    assert not authority["hardware_device_access_authorized"]
    assert not authority["real_vehicle_authorized"]
