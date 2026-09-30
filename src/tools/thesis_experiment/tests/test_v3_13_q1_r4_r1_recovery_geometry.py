#!/usr/bin/env python3
"""Tests for runtime-radius-aligned recovery geometry."""

import copy
import math
from pathlib import Path

import yaml

from v3_scope_planner import multi_actor_core
from v3_scope_planner import multi_actor_core_v1
from v3_scope_planner import recovery_candidate_core


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = W / (
    "config/thesis_experiments/v3/"
    "v3_13_q1_r4_r1_recovery_candidate_geometry_contract.yaml"
)
D0 = W / (
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
MODEL = W / (
    "artifacts/v3/multiactor/v3_13_s3_predictor/"
    "candidate_conditioned_recovery_predictor.yaml"
)
R2 = W / (
    "artifacts/v3/multiactor/v3_13_q1_r2_diagnostic/episodes/"
    "001_01_7331_v3_13_analytic/episode_report.yaml"
)


def load(path):
    return yaml.safe_load(path.read_bytes())


def snapshot():
    report = load(R2)
    return next(
        row["snapshot"]
        for row in report["planning_diagnostics"]
        if row["label"] == "late_recovery"
    )


def surrogate_evaluation(combined_radius):
    extension = load(CONTRACT)
    d0 = load(D0)
    model = load(MODEL)
    candidate, local = recovery_candidate_core.generate_recovery_candidate(
        snapshot(), d0, extension, True
    )
    local["hard_verifier"]["combined_actor_ego_radius_m"] = combined_radius
    candidate_id = extension["candidate_extension"]["candidate_id"]
    for heads in model["heads"].values():
        heads[candidate_id] = copy.deepcopy(
            heads["spatial_right_v120"]
        )
    return multi_actor_core_v1.evaluate_candidate(
        snapshot(), candidate, model, local
    )


def test_superseded_r4_cross_layer_gap_is_frozen():
    contract = load(CONTRACT)
    diagnosis = contract["superseded_design_diagnosis"]
    assert math.isclose(
        diagnosis["r4_planner_combined_radius_m"], 1.02
    )
    assert math.isclose(
        diagnosis["clone_measurement_combined_radius_m"], 1.15
    )
    assert diagnosis[
        "r4_clone_radius_equivalent_surrogate_clearance_m"
    ] < 0.0
    assert diagnosis["r4_candidate_not_executed"] is True


def test_revised_geometry_preserves_legacy_and_is_recovery_only():
    contract = load(CONTRACT)
    d0 = load(D0)
    legacy = multi_actor_core.generate_candidate_set(snapshot(), d0)
    initial = recovery_candidate_core.generate_candidate_set(
        snapshot(), d0, contract, False
    )
    recovery = recovery_candidate_core.generate_candidate_set(
        snapshot(), d0, contract, True
    )
    assert initial["candidates"] == legacy["candidates"]
    assert recovery["candidates"][:4] == legacy["candidates"]
    assert recovery["extension_count"] == 1
    assert recovery["candidates"][-1]["candidate_id"] == (
        "recovery_evasive_right_v120_r1"
    )


def test_revised_candidate_is_inside_M2_simulation_envelope():
    contract = load(CONTRACT)
    candidate, local = recovery_candidate_core.generate_recovery_candidate(
        snapshot(), load(D0), contract, True
    )
    points = candidate["points"]
    envelope = contract["hardware_envelope"]
    assert not multi_actor_core.validate_candidate(candidate, local)
    assert max(
        point["longitudinal_acceleration_mps2"] for point in points
    ) <= envelope["maximum_acceleration_mps2"] + 1.0e-12
    assert max(abs(point["steering_angle_rad"]) for point in points) <= (
        envelope["maximum_steering_angle_rad"] + 1.0e-12
    )
    assert max(abs(point["steering_rate_radps"]) for point in points) <= (
        envelope["maximum_steering_rate_radps"] + 1.0e-12
    )
    assert max(abs(point["curvature_inv_m"]) for point in points) <= (
        math.tan(envelope["maximum_steering_angle_rad"])
        / envelope["wheelbase_m"]
        + 1.0e-12
    )


def test_revised_geometry_clears_both_frozen_radius_conventions():
    contract = load(CONTRACT)
    gates = contract["offline_geometry_gates"]
    planner = surrogate_evaluation(1.02)
    runtime = surrogate_evaluation(1.15)
    assert planner["minimum_dynamic_clearance_m"] >= gates[
        "R2_surrogate_dynamic_clearance_m_min"
    ]
    assert runtime["minimum_dynamic_clearance_m"] >= gates[
        "R2_runtime_radius_surrogate_dynamic_clearance_m_min"
    ]
    assert runtime["minimum_static_clearance_m"] >= gates[
        "R2_corridor_static_clearance_m_min"
    ]
    assert load(CONTRACT)["authority"][
        "ros_or_gazebo_execution_authorized"
    ] is False
