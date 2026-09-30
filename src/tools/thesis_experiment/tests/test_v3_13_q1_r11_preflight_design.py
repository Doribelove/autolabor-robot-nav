#!/usr/bin/env python3
"""Static checks for R11 one-unit progress preflight."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r11_progress_recovery_preflight_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_progress_recovery_preflight.launch"
)


def test_unique_fresh_identity_and_no_retry():
    population = yaml.safe_load(CONTRACT.read_bytes())["population"]
    assert population["units"] == 1
    assert population["seed"] == 7641
    assert population["attempt"] == 1
    assert population[
        "retry_resume_replacement_or_selective_exclusion_authorized"
    ] is False


def test_dynamic_and_static_gates_are_both_required():
    gates = yaml.safe_load(CONTRACT.read_bytes())["scientific_gates"]
    assert gates["minimum_conservative_clearance_m_min"] == 0.0
    assert gates["minimum_trace_static_clearance_m_min"] == 0.05
    assert gates["contact_state_count_max"] == 0


def test_launch_binds_progress_runtime_and_frozen_physics():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = LAUNCH.read_text(encoding="utf-8")
    assert "v3_scope_progress_recovery_preflight_episode.py" in text
    assert 'max_acceleration" value="1.20"' in text
    assert 'max_steering_rate" value="0.80"' in text
    assert "/cmd_vel_navigation /cmd_vel" in text


def test_actual_teb_training_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"][
        "direct_actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
