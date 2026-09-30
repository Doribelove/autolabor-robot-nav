#!/usr/bin/env python3
"""Static checks for R19-R1 boundary-atomic preflight."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r1_boundary_atomic_preflight_contract.yaml"
)
RUNTIME = W / (
    "src/application/v3_scope_planner/scripts/"
    "v3_scope_goal_aligned_boundary_atomic_preflight_episode.py"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_goal_aligned_boundary_atomic_preflight.launch"
)


def test_fresh_identity_and_single_factor():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["population"]["seed"] == 7677
    assert contract["population"]["attempt"] == 1
    factor = contract["single_factor"]
    assert factor["factor"] == (
        "terminal_guard_evaluation_before_horizon_exit"
    )
    assert factor[
        "geometry_actor_model_threshold_and_horizon_change_count"
    ] == 0


def test_runtime_evaluates_before_horizon_exit():
    text = RUNTIME.read_text(encoding="utf-8")
    command = text.index("command = execution_core.pure_pursuit_command(")
    boundary = text.index("if elapsed_s >= self.horizon_s:")
    assert command < boundary
    assert "self.publish_command(0.0, 0.0)" in text


def test_launch_and_gates_are_exact():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = LAUNCH.read_text(encoding="utf-8")
    assert "v3_scope_goal_aligned_boundary_atomic_preflight_episode.py" in text
    gates = yaml.safe_load(CONTRACT.read_bytes())["scientific_gates"]
    assert gates["boundary_terminal_evaluation_count_exact"] == 1
    assert gates["boundary_terminal_trigger_count_exact"] == 1


def test_training_actual_teb_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"]["actual_TEB_execution_authorized"] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
