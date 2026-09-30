#!/usr/bin/env python3
"""Static tests for the V3-13-Q1-R4 recovery geometry closure."""

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
    "v3_13_q1_r4_recovery_candidate_geometry_contract.yaml"
)
D0 = W / (
    "config/thesis_experiments/v3/"
    "v3_13_d0_multiactor_foundation_contract.yaml"
)
MODEL = W / (
    "artifacts/v3/multiactor/v3_13_s3_predictor/"
    "candidate_conditioned_recovery_predictor.yaml"
)
R2_EPISODE = W / (
    "artifacts/v3/multiactor/v3_13_q1_r2_diagnostic/episodes/"
    "001_01_7331_v3_13_analytic/episode_report.yaml"
)


def load(path):
    return yaml.safe_load(path.read_bytes())


def late_snapshot():
    report = load(R2_EPISODE)
    return next(
        row["snapshot"]
        for row in report["planning_diagnostics"]
        if row["label"] == "late_recovery"
    )


def test_exactly_one_recovery_only_candidate_and_authority():
    contract = load(CONTRACT)
    spec = contract["candidate_extension"]
    assert spec["candidate_id"] == "recovery_evasive_right_v120"
    assert spec["recovery_only"] is True
    assert contract["single_architectural_factor"][
        "legacy_candidate_count"
    ] == 4
    assert contract["single_architectural_factor"][
        "prospective_candidate_count"
    ] == 5
    assert contract["authority"]["ros_or_gazebo_execution_authorized"] is False
    assert contract["authority"]["training_authorized"] is False
    assert contract["authority"]["real_vehicle_authorized"] is False


def test_legacy_bank_is_identical_and_extension_is_recovery_only():
    extension = load(CONTRACT)
    d0 = load(D0)
    snapshot = late_snapshot()
    legacy = multi_actor_core.generate_candidate_set(snapshot, d0)
    initial = recovery_candidate_core.generate_candidate_set(
        snapshot, d0, extension, False
    )
    recovery = recovery_candidate_core.generate_candidate_set(
        snapshot, d0, extension, True
    )
    assert initial["extension_count"] == 0
    assert initial["candidates"] == legacy["candidates"]
    assert recovery["extension_count"] == 1
    assert recovery["candidates"][:4] == legacy["candidates"]
    assert len(recovery["candidates"]) == 5


def test_candidate_is_inside_frozen_m2_simulation_envelope():
    extension = load(CONTRACT)
    d0 = load(D0)
    candidate, local = recovery_candidate_core.generate_recovery_candidate(
        late_snapshot(), d0, extension, True
    )
    points = candidate["points"]
    envelope = extension["hardware_envelope"]
    assert not multi_actor_core.validate_candidate(candidate, local)
    assert max(
        point["longitudinal_acceleration_mps2"] for point in points
    ) <= extension["offline_geometry_gates"][
        "maximum_acceleration_mps2_max"
    ] + 1.0e-12
    assert max(abs(point["steering_angle_rad"]) for point in points) <= (
        extension["offline_geometry_gates"][
            "maximum_steering_angle_rad_max"
        ]
        + 1.0e-12
    )
    assert max(abs(point["steering_rate_radps"]) for point in points) <= (
        extension["offline_geometry_gates"][
            "maximum_steering_rate_radps_max"
        ]
        + 1.0e-12
    )
    maximum_curvature = max(
        abs(point["curvature_inv_m"]) for point in points
    )
    assert maximum_curvature <= extension["offline_geometry_gates"][
        "maximum_curvature_inv_m_max"
    ]
    assert maximum_curvature <= math.tan(
        envelope["maximum_steering_angle_rad"]
    ) / envelope["wheelbase_m"]


def test_design_surrogate_clears_frozen_r2_snapshot_without_gate_changes():
    extension = load(CONTRACT)
    d0 = load(D0)
    model = load(MODEL)
    candidate, local = recovery_candidate_core.generate_recovery_candidate(
        late_snapshot(), d0, extension, True
    )
    surrogate = copy.deepcopy(model)
    candidate_id = extension["candidate_extension"]["candidate_id"]
    for mode, heads in surrogate["heads"].items():
        heads[candidate_id] = copy.deepcopy(heads["spatial_right_v120"])
    evaluation = multi_actor_core_v1.evaluate_candidate(
        late_snapshot(), candidate, surrogate, local
    )
    gates = extension["offline_geometry_gates"]
    assert evaluation["minimum_dynamic_clearance_m"] >= gates[
        "R2_surrogate_dynamic_clearance_m_min"
    ]
    assert evaluation["minimum_static_clearance_m"] >= gates[
        "R2_corridor_static_clearance_m_min"
    ]
    assert local["hard_verifier"] == d0["hard_verifier"]
    assert local["analytic_ranker"] == d0["analytic_ranker"]
