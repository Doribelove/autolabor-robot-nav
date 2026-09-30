#!/usr/bin/env python3
"""Static checks for R19 goal-aligned support."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_goal_aligned_recovery_support_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_goal_aligned_recovery_bank_clone_episode.launch"
)
RUNTIME = W / (
    "src/application/v3_scope_planner/scripts/"
    "v3_scope_goal_aligned_recovery_bank_clone_episode.py"
)


def test_fresh_complete_support_population():
    population = yaml.safe_load(CONTRACT.read_bytes())["population"]
    assert population["seeds"] == list(range(7669, 7677))
    assert population["matched_context_blocks"] == 8
    assert population["actions_per_block"] == 5
    assert population["exact_units"] == 40
    assert population["behavior_propensity_each"] == 0.20
    assert population["actions"][-1].endswith("_r3")


def test_matching_and_guard_gates_remain_strict():
    gates = yaml.safe_load(CONTRACT.read_bytes())["matching_gates"]
    assert gates["complete_action_support_rate_min"] == 1.0
    assert gates["paired_decision_ego_x_difference_max_m"] == 0.04
    assert gates[
        "paired_decision_actor_coordinate_difference_max_m"
    ] == 0.05
    assert gates["paired_decision_actor_speed_difference_max_mps"] == 0.05
    assert gates["r3_terminal_guard_execution_rate_min"] == 1.0


def test_launch_and_runtime_preserve_legacy_horizon():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = LAUNCH.read_text(encoding="utf-8")
    assert "v3_scope_goal_aligned_recovery_bank_clone_episode.py" in text
    assert 'horizon_s" value="$(arg horizon_s)"' in text
    runtime = RUNTIME.read_text(encoding="utf-8")
    assert "legacy action horizon changed" in runtime
    assert "terminal_guard_activation_count" in runtime


def test_actual_teb_training_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"][
        "actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
