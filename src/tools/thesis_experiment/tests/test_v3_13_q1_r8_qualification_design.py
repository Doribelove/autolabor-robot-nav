#!/usr/bin/env python3
"""Static checks for V3-13-Q1-R8 qualification."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r8_recovery_predictor_qualification_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_recovery_predictor_qualification.launch"
)


def test_fresh_population_and_complete_support():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    population = contract["qualification_population"]
    assert population["seeds"] == list(range(7625, 7633))
    assert population["matched_context_blocks"] == 8
    assert population["actions_per_block"] == 5
    assert population["exact_units"] == 40
    assert population["behavior_propensity_each"] == 0.20
    assert population[
        "retry_resume_replacement_or_selective_exclusion_authorized"
    ] is False


def test_predictor_gates_are_frozen_before_execution():
    gates = yaml.safe_load(CONTRACT.read_bytes())["predictor_gates"]
    assert gates["simultaneous_whole_trajectory_coverage_min"] == 0.90
    assert gates[
        "every_scene_simultaneous_trajectory_coverage_min"
    ] == 0.85
    assert gates[
        "every_candidate_simultaneous_trajectory_coverage_min"
    ] == 0.75
    assert gates["mean_trajectory_rmse_m_max"] == 0.25
    assert gates["maximum_trajectory_rmse_m_max"] == 0.65


def test_launch_binds_model_and_frozen_physics():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = LAUNCH.read_text(encoding="utf-8")
    assert "predictor_model_path" in text
    assert 'max_acceleration" value="1.20"' in text
    assert 'combined_radius_m" value="1.15"' in text
    assert "/cmd_vel_navigation /cmd_vel" in text


def test_actual_teb_training_perception_and_hardware_stay_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["authority"][
        "exact_bounded_simulation_qualification_authorized"
    ] is True
    assert contract["downstream"][
        "direct_actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"][
        "perception_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
