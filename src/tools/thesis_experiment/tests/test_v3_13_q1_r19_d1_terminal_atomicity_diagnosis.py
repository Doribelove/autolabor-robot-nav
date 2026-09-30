#!/usr/bin/env python3
"""Checks for R19-D1 terminal atomicity diagnosis."""

from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_d1_terminal_atomicity_diagnosis_contract.yaml"
)
DIAGNOSIS = W / (
    "artifacts/v3/multiactor/"
    "v3_13_q1_r19_d1_terminal_atomicity_diagnosis/"
    "terminal_atomicity_diagnosis.yaml"
)


def test_diagnosis_replays_all_r3_final_poses():
    value = yaml.safe_load(DIAGNOSIS.read_bytes())
    assert len(value["rows"]) == 8
    assert sum(
        row["observed_guard_activation_count"] >= 1
        for row in value["rows"]
    ) == 6
    assert all(
        row["final_pose_would_trigger_frozen_guard"]
        for row in value["rows"]
    )


def test_only_guard_runtime_ordering_failed():
    value = yaml.safe_load(DIAGNOSIS.read_bytes())
    assert all(value["checks"].values())
    assert value["aggregate"]["minimum_dynamic_clearance_m"] > 0.0
    assert value["aggregate"]["minimum_static_clearance_m"] > 0.05
    assert value["R19_remains_failed"] is True
    assert value["R19_predictor_fit_authorized"] is False


def test_execution_training_and_hardware_remain_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["authority"]["ROS_or_Gazebo_execution_authorized"] is False
    assert contract["authority"]["predictor_fit_authorized"] is False
    assert contract["authority"]["training_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
