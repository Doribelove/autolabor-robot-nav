#!/usr/bin/env python3
"""Static contract checks for V3-13-Q1 actual-TEB qualification."""

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_actual_teb_multiactor_qualification_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_actual_teb_multiactor_episode.launch"
)


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_contract_population_and_authority():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["methods"]["identifiers"] == [
        "v3_13_analytic",
        "actual_teb",
    ]
    assert len(contract["scene_population"]) == 4
    assert sum(
        len(scene["actor_policy_pairs"])
        for scene in contract["scene_population"]
    ) == 16
    assert contract["execution_design"]["qualification"]["seeds"] == list(
        range(7301, 7317)
    )
    assert contract["execution_design"]["qualification"]["units"] == 32
    assert contract["authority"]["training_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
    assert contract["authority"]["stable_workspace_write_authorized"] is False


def test_exact_actual_teb_identity_is_frozen():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    baseline = contract["methods"]["primary_baseline"]
    assert baseline["planner_plugin"] == (
        "teb_local_planner/TebLocalPlannerROS"
    )
    assert baseline["teb_version"] == "0.8.4"
    assert baseline["max_vel_x_mps"] == 1.2
    assert sha(W / baseline["config_path"]) == baseline["config_sha256"]
    assert sha(W / baseline["built_plugin_path"]) == (
        baseline["built_plugin_sha256"]
    )
    assert sha(W / baseline["source_path"]) == baseline["source_sha256"]


def test_launch_keeps_common_boundary_and_two_obstacle_capability():
    root = ET.parse(str(LAUNCH)).getroot()
    text = LAUNCH.read_text(encoding="utf-8")
    assert root.tag == "launch"
    assert "/cmd_vel_navigation /cmd_vel" in text
    assert "fixed_teb.yaml" in text
    assert "include_dynamic_obstacles" in text
    assert "v3_scope_multiactor_actual_teb_fair_episode.py" in text


def test_recovery_and_claim_boundaries_are_frozen():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    treatment = contract["methods"]["treatment"]
    assert treatment["recovery_replanning"][
        "fixed_absolute_sim_time_barriers_s"
    ] == [8.5, 12.5]
    assert treatment["recovery_replanning"][
        "no_hidden_actor_policy_input"
    ] is True
    assert contract["downstream"]["direct_formal_execution_authorized"] is False
    assert contract["downstream"][
        "perception_stage_execution_authorized"
    ] is False
    assert "actual_teb_performance_improvement" in contract["claims"][
        "forbidden"
    ]
