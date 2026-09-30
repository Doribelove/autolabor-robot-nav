#!/usr/bin/env python3
"""Static checks for R18 one-unit goal-aligned preflight."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r18_goal_aligned_recovery_preflight_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_goal_aligned_recovery_preflight.launch"
)
RUNTIME = W / (
    "src/application/v3_scope_planner/scripts/"
    "v3_scope_goal_aligned_recovery_preflight_episode.py"
)


def test_unique_fresh_identity_and_no_retry():
    population = yaml.safe_load(CONTRACT.read_bytes())["population"]
    assert population["units"] == 1
    assert population["seed"] == 7667
    assert population["attempt"] == 1
    assert population["forced_candidate_id"].endswith("_r3")
    assert population[
        "retry_resume_replacement_or_selective_exclusion_authorized"
    ] is False


def test_safety_goal_and_terminal_guard_gates_are_all_required():
    gates = yaml.safe_load(CONTRACT.read_bytes())["scientific_gates"]
    assert gates["minimum_conservative_clearance_m_min"] == 0.0
    assert gates["minimum_trace_static_clearance_m_min"] == 0.05
    assert gates["contact_state_count_max"] == 0
    assert gates["final_common_goal_distance_m_max"] == 0.25
    assert gates["terminal_guard_activation_count_min"] == 1


def test_launch_binds_goal_aligned_runtime_and_frozen_physics():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = LAUNCH.read_text(encoding="utf-8")
    assert "v3_scope_goal_aligned_recovery_preflight_episode.py" in text
    assert 'max_acceleration" value="1.20"' in text
    assert 'max_steering_rate" value="0.80"' in text
    assert 'horizon_s" value="7.55"' in text
    assert "/cmd_vel_navigation /cmd_vel" in text
    runtime = RUNTIME.read_text(encoding="utf-8")
    assert "terminal_guard_activation_count" in runtime
    assert "final_common_goal_distance_m" in runtime


def test_actual_teb_training_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"][
        "direct_actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
