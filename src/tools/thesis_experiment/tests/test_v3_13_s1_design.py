#!/usr/bin/env python3

from pathlib import Path

import yaml


ROOT = Path("/home/robot/robot_ws_base_rl")
CONTRACT = yaml.safe_load(
    (
        ROOT
        / "config/thesis_experiments/v3/"
        "v3_13_s1_sequential_clone_contract.yaml"
    ).read_text(encoding="utf-8")
)


def test_sequential_schedule_scope_is_exact():
    population = CONTRACT["population"]
    assert population["matched_context_blocks"] == 8
    assert population["actions_per_block"] == 4
    assert population["exact_units"] == 32
    assert len(population["seeds"]) == 8
    assert len(population["branch_actions"]) == 4


def test_wait_depths_cover_early_and_late_recovery():
    assert CONTRACT["population"]["pre_action_wait_depths_s"] == [1.0, 5.0]


def test_support_gap_is_not_misrepresented_as_closed():
    gap = CONTRACT["support_gap"]
    assert gap["initial_actions_fully_supported"]
    assert not gap["matched_post_stop_alternative_actions_supported"]
    assert not gap["current_predictor_supports_assertive"]


def test_design_has_zero_execution_or_training_authority():
    authority = CONTRACT["authority"]
    assert authority["offline_design_authorized"]
    assert not authority["simulation_execution_authorized"]
    assert authority["authorized_units"] == 0
    assert not authority["training_authorized"]
    assert not authority["hardware_device_access_authorized"]
    assert not authority["real_vehicle_authorized"]

