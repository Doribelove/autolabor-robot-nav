#!/usr/bin/env python3
"""Static checks for R12 progress-candidate support."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r12_progress_recovery_support_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_progress_recovery_bank_clone_episode.launch"
)


def test_fresh_complete_support_population():
    population = yaml.safe_load(CONTRACT.read_bytes())["population"]
    assert population["seeds"] == list(range(7642, 7650))
    assert population["matched_context_blocks"] == 8
    assert population["actions_per_block"] == 5
    assert population["exact_units"] == 40
    assert population["behavior_propensity_each"] == 0.20
    assert population["actions"][-1] == (
        "recovery_evasive_right_v120_r2"
    )


def test_matching_gates_remain_strict():
    gates = yaml.safe_load(CONTRACT.read_bytes())["matching_gates"]
    assert gates["complete_action_support_rate_min"] == 1.0
    assert gates["paired_decision_ego_x_difference_max_m"] == 0.04
    assert gates[
        "paired_decision_actor_coordinate_difference_max_m"
    ] == 0.05
    assert gates["paired_decision_actor_speed_difference_max_mps"] == 0.05


def test_launch_binds_progress_bank_and_frozen_physics():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = LAUNCH.read_text(encoding="utf-8")
    assert "v3_scope_progress_recovery_bank_clone_episode.py" in text
    assert 'max_acceleration" value="1.20"' in text
    assert "/cmd_vel_navigation /cmd_vel" in text


def test_actual_teb_training_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"][
        "actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
