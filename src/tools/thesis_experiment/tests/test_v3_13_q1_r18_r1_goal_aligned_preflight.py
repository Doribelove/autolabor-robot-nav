#!/usr/bin/env python3
"""Static and semantic-equivalence checks for R18-R1."""

import copy
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
sys.path.insert(0, str(W / "src/application/v3_scope_planner/src"))
from v3_scope_planner import goal_aligned_execution_adapter_r1 as adapter  # noqa: E402
from v3_scope_planner import goal_aligned_execution_core as frozen  # noqa: E402

CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r18_r1_goal_aligned_recovery_preflight_contract.yaml"
)
R18 = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r18_goal_aligned_recovery_preflight_contract.yaml"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_goal_aligned_recovery_preflight_r1.launch"
)


def points():
    return [
        {
            "x_m": 0.1 * index,
            "y_m": -0.02 * index,
            "longitudinal_velocity_mps": 1.2,
        }
        for index in range(20)
    ]


def test_adapter_is_exactly_equivalent_to_frozen_guard():
    path = points()
    probes = [
        (0.2, -0.04, -0.1, 0),
        (1.4, -0.28, -0.1, 10),
        (1.9, -0.38, -0.1, 16),
        (2.0, -0.40, -0.1, 16),
    ]
    for x_m, y_m, yaw_rad, minimum_index in probes:
        expected = frozen.pure_pursuit_command(
            copy.deepcopy(path),
            x_m,
            y_m,
            yaw_rad,
            minimum_index=minimum_index,
        )
        observed = adapter.pure_pursuit_command(
            copy.deepcopy(path),
            x_m,
            y_m,
            yaw_rad,
            minimum_index=minimum_index,
        )
        assert observed == expected


def test_fresh_identity_and_R18_is_not_retried():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["population"]["seed"] == 7668
    assert contract["population"]["attempt"] == 1
    assert contract["failure_inheritance"]["R18_seed7667_reused"] is False
    assert contract["failure_inheritance"]["retry_or_resume_of_R18"] is False


def test_semantic_population_and_gates_equal_R18():
    repaired = yaml.safe_load(CONTRACT.read_bytes())
    original = yaml.safe_load(R18.read_bytes())
    for key in (
        "scene_id",
        "actor_policies_evaluator_only",
        "nominal_wait_depth_s",
        "branch_target_sim_s",
        "forced_candidate_id",
        "behavior_propensity",
        "non_evidence_preflight",
    ):
        assert repaired["population"][key] == original["population"][key]
    assert repaired["scientific_gates"] == original["scientific_gates"]


def test_launch_and_authority_are_closed():
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"
    text = LAUNCH.read_text(encoding="utf-8")
    assert (
        "v3_scope_goal_aligned_recovery_preflight_episode_r1.py"
        in text
    )
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"][
        "direct_actual_TEB_execution_authorized"
    ] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False
