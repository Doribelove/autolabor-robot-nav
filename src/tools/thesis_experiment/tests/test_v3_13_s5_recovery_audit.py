#!/usr/bin/env python3

from pathlib import Path

import yaml


ROOT = Path("/home/robot/robot_ws_base_rl")
CONTRACT = yaml.safe_load(
    (
        ROOT
        / "config/thesis_experiments/v3/"
        "v3_13_s5_recovery_planner_audit_contract.yaml"
    ).read_text(encoding="utf-8")
)


def test_audit_has_three_state_chain():
    population = CONTRACT["audit_population"]
    assert population["high_risk_strata"] == 4
    assert population["states_per_stratum"] == [
        "initial_interaction",
        "early_recovery",
        "late_recovery",
    ]


def test_progress_requires_supported_safe_clone():
    gates = CONTRACT["gates"]
    assert gates["all_late_recovery_decisions_nonstop_progress"]
    assert gates[
        "all_selected_clone_outcomes_nonnegative_conservative_clearance"
    ]
    assert gates["all_states_have_four_action_clone_support"]


def test_audit_has_no_execution_or_training_authority():
    authority = CONTRACT["authority"]
    assert authority["pure_offline_audit_authorized"]
    assert not authority["ROS_or_Gazebo_execution_authorized"]
    assert not authority["training_authorized"]
    assert not authority["hardware_device_access_authorized"]
    assert not authority["real_vehicle_authorized"]
