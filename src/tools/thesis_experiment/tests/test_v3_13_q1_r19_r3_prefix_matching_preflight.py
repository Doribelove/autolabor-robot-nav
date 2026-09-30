#!/usr/bin/env python3
"""Static tests for the R19-R3 prefix-matching preflight."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r3_prefix_matching_preflight_contract.yaml"
)
RUNTIME = W / (
    "src/application/v3_scope_planner/scripts/"
    "v3_scope_prefix_aligned_boundary_atomic_bank_clone_episode.py"
)
LAUNCH = W / (
    "src/simulation/m2_gazebo/launch/"
    "m2_v3_13_prefix_aligned_boundary_atomic_bank_clone_episode.launch"
)


def test_single_factor_and_fresh_identity():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["population"]["seed"] == 7686
    assert contract["population"]["exact_units"] == 5
    assert contract["single_factor"]["control"] == 20
    assert contract["single_factor"]["treatment"] == 100
    assert contract["single_factor"]["actor_plugin_unchanged"] is True


def test_runtime_changes_only_common_prefix_rate():
    text = RUNTIME.read_text(encoding="utf-8")
    assert "rate = rospy.Rate(100)" in text
    assert "return super().execute" not in text
    assert "postbranch_execution_changed" in text
    assert ET.parse(str(LAUNCH)).getroot().tag == "launch"


def test_all_downstream_authority_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"]["predictor_fit_authorized"] is False
    assert contract["downstream"]["actual_TEB_execution_authorized"] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
