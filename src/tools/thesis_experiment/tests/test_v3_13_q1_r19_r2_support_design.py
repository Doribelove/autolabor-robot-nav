#!/usr/bin/env python3
"""Static checks for R19-R2 support."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r2_boundary_atomic_support_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_boundary_atomic_recovery_bank_clone_episode.launch"
)
RUNTIME = W / (
    "src/application/v3_scope_planner/scripts/"
    "v3_scope_boundary_atomic_recovery_bank_clone_episode.py"
)


def test_fresh_complete_support_population():
    population = yaml.safe_load(CONTRACT.read_bytes())["population"]
    assert population["seeds"] == list(range(7678, 7686))
    assert population["exact_units"] == 40
    assert population["actions_per_block"] == 5
    assert population["behavior_propensity_each"] == 0.20


def test_guard_and_boundary_rates_are_strict():
    gates = yaml.safe_load(CONTRACT.read_bytes())["matching_gates"]
    assert gates["r3_terminal_guard_execution_rate_min"] == 1.0
    assert gates["r3_boundary_atomic_execution_rate_min"] == 1.0
    assert gates["complete_action_support_rate_min"] == 1.0


def test_runtime_applies_atomic_path_only_to_r3():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = RUNTIME.read_text(encoding="utf-8")
    assert "if self.forced_candidate_id == R3" in text
    assert "return super().execute(candidate)" in text


def test_training_actual_teb_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"]["actual_TEB_execution_authorized"] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
