#!/usr/bin/env python3

from pathlib import Path

import yaml


ROOT = Path("/home/robot/robot_ws_base_rl")
CONTRACT = yaml.safe_load(
    (
        ROOT
        / "config/thesis_experiments/v3/"
        "v3_13_s2_synchronized_sequential_clone_contract.yaml"
    ).read_text(encoding="utf-8")
)


def test_repair_is_execution_integrity_only():
    repair = CONTRACT["repair"]
    assert not repair["behavioral_factor_changed"]
    assert repair["fixed_absolute_sim_clock_branch_barrier"]
    assert repair["actor_speed_estimator"] == (
        "causal_coordinate_slope_over_0_50_s"
    )
    assert repair["original_matching_thresholds_retained"]


def test_preflight_is_non_evidence_and_exact():
    preflight = CONTRACT["integration_preflight"]
    assert not preflight["evidence_identity"]
    assert preflight["seed"] == 7239
    assert preflight["exact_units"] == 4
    assert len(preflight["actions"]) == 4
    assert preflight[
        "output_cannot_enter_predictor_fit_or_performance_analysis"
    ]


def test_future_evidence_roster_is_fresh_and_bounded():
    population = CONTRACT["population"]
    assert population["future_evidence_seeds"] == list(
        range(7241, 7249)
    )
    assert population["future_matched_context_blocks"] == 8
    assert population["future_exact_evidence_units"] == 32


def test_authority_boundary():
    authority = CONTRACT["authority"]
    assert authority["exact_non_evidence_preflight_authorized"]
    assert authority["authorized_preflight_units"] == 4
    assert not authority["evidence_execution_authorized"]
    assert authority["authorized_evidence_units"] == 0
    assert not authority["training_authorized"]
    assert not authority["hardware_device_access_authorized"]
    assert not authority["real_vehicle_authorized"]
