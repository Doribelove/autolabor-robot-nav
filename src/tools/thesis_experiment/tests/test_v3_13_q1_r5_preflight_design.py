#!/usr/bin/env python3
"""Static checks for the one-unit V3-13-Q1-R5 preflight."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r5_recovery_candidate_preflight_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_recovery_candidate_preflight.launch"
)


def test_exact_single_fresh_identity_and_no_retry():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    population = contract["population"]
    assert population["units"] == 1
    assert population["seed"] == 7336
    assert population["attempt"] == 1
    assert population["scene_id"] == "crossing_and_oncoming"
    assert population["actor_policies_evaluator_only"] == [
        "hesitant",
        "hesitant",
    ]
    assert population[
        "retry_resume_replacement_or_selective_exclusion_authorized"
    ] is False


def test_clearance_conventions_and_candidate_are_frozen():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    factor = contract["single_factor"]
    assert factor["candidate_id"] == (
        "recovery_evasive_right_v120_r1"
    )
    assert factor["branch_time_unchanged_from_R2_late_recovery_s"] == 12.5
    assert factor["clone_combined_radius_m_unchanged"] == 1.15
    assert contract["scientific_gates"][
        "minimum_conservative_clearance_m_min"
    ] == 0.0


def test_launch_keeps_vehicle_dynamics_and_command_boundary():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = LAUNCH.read_text(encoding="utf-8")
    assert 'max_acceleration" value="1.20"' in text
    assert 'max_steering_rate" value="0.80"' in text
    assert "/cmd_vel_navigation /cmd_vel" in text
    assert 'combined_radius_m" value="1.15"' in text
    assert "v3_scope_recovery_candidate_preflight_episode.py" in text


def test_downstream_claims_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["authority"][
        "exact_bounded_simulation_preflight_authorized"
    ] is True
    assert contract["authority"]["authorized_units"] == 1
    assert contract["authority"]["training_authorized"] is False
    assert contract["downstream"]["actual_TEB_execution_authorized"] is False
    assert contract["downstream"][
        "perception_execution_authorized"
    ] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
