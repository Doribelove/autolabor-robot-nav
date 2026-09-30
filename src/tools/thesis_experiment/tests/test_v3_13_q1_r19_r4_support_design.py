#!/usr/bin/env python3
"""Static checks for R19-R4 support."""

from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r19_r4_prefix_aligned_support_contract.yaml"
)


def test_fresh_complete_support_population():
    population = yaml.safe_load(CONTRACT.read_bytes())["population"]
    assert population["seeds"] == list(range(7687, 7695))
    assert population["exact_units"] == 40
    assert population["actions_per_block"] == 5
    assert population["behavior_propensity_each"] == 0.20


def test_prefix_and_boundary_execution_are_frozen():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert (
        contract["frozen_execution"][
            "prebranch_warmup_command_sampling_rate_hz"
        ]
        == 100
    )
    assert contract["frozen_execution"][
        "boundary_atomic_terminal_evaluation_for_r3_only"
    ] is True
    assert contract["matching_gates"][
        "r3_terminal_guard_execution_rate_min"
    ] == 1.0


def test_downstream_authority_remains_closed():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    assert contract["downstream"]["actual_TEB_execution_authorized"] is False
    assert contract["downstream"]["training_authorized"] is False
    assert contract["authority"]["hardware_device_access_authorized"] is False
