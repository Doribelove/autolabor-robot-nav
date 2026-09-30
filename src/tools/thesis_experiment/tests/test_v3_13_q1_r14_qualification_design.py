#!/usr/bin/env python3
"""Static tests for R14 fresh qualification."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r14_progress_predictor_qualification_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_progress_predictor_qualification.launch"
)


def test_fresh_population_and_complete_support():
    population = yaml.safe_load(CONTRACT.read_bytes())[
        "qualification_population"
    ]
    assert population["seeds"] == list(range(7650, 7658))
    assert population["exact_units"] == 40
    assert population["actions_per_block"] == 5
    assert population["behavior_propensity_each"] == 0.20


def test_predictor_and_static_gates_are_fixed():
    gates = yaml.safe_load(CONTRACT.read_bytes())["predictor_gates"]
    assert gates["simultaneous_whole_trajectory_coverage_min"] == 0.90
    assert gates["mean_trajectory_rmse_m_max"] == 0.25
    assert gates["maximum_trajectory_rmse_m_max"] == 0.65
    assert gates[
        "progress_candidate_every_block_static_clearance_m_min"
    ] == 0.05


def test_launch_binds_progress_model_and_runtime():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = LAUNCH.read_text(encoding="utf-8")
    assert "v3_scope_progress_predictor_qualification_episode.py" in text
    assert "predictor_model_path" in text
    assert "/cmd_vel_navigation /cmd_vel" in text


def test_actual_teb_training_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"][
        "direct_actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
