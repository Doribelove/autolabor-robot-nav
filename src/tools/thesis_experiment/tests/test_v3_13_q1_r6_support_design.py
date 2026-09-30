#!/usr/bin/env python3
"""Static checks for V3-13-Q1-R6 support collection."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r6_recovery_bank_support_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_recovery_bank_clone_episode.launch"
)


def test_population_is_fresh_complete_and_bounded():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    population = contract["population"]
    assert population["seeds"] == list(range(7617, 7625))
    assert population["matched_context_blocks"] == 8
    assert population["actions_per_block"] == 5
    assert population["exact_units"] == 40
    assert population["behavior_propensity_each"] == 0.20
    assert len(population["actions"]) == len(set(population["actions"])) == 5
    assert population[
        "retry_resume_replacement_or_selective_exclusion_authorized"
    ] is False


def test_only_new_head_may_be_fit_after_result():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["data_use"]["fit_scope_if_gates_pass"] == (
        "assertive_head_for_recovery_evasive_right_v120_r1_only"
    )
    assert contract["data_use"][
        "retain_all_S3_heads_byte_equivalent"
    ] is True
    assert contract["downstream"][
        "predictor_fit_authorized_before_result"
    ] is False
    assert contract["authority"]["training_authorized"] is False


def test_launch_keeps_frozen_dynamics_radius_and_boundary():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'max_acceleration" value="1.20"' in text
    assert 'max_steering_rate" value="0.80"' in text
    assert 'combined_radius_m" value="1.15"' in text
    assert "/cmd_vel_navigation /cmd_vel" in text
    assert "v3_scope_recovery_bank_clone_episode.py" in text


def test_actual_teb_perception_training_and_hardware_stay_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["authority"][
        "exact_bounded_simulation_support_authorized"
    ] is True
    assert contract["downstream"]["actual_TEB_execution_authorized"] is False
    assert contract["downstream"][
        "perception_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
