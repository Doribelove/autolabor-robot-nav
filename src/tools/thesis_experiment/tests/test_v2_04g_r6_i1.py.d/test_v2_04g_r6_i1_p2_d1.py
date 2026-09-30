import importlib.util
import math
from pathlib import Path

import pytest


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SOURCE = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p2_d1.py"
)


def _module():
    spec = importlib.util.spec_from_file_location("p2_d1_under_test", SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


P2 = _module()


def test_minimum_time_trapezoid_and_triangle():
    trapezoid = P2.minimum_time(18.2, 0.0, 0.0, 1.2, 0.8)
    assert trapezoid == pytest.approx(16.666666666666668)
    triangle = P2.minimum_time(0.4, 0.0, 0.0, 1.2, 0.8)
    assert triangle == pytest.approx(2.0 * math.sqrt(0.4 / 0.8))
    assert P2.time_at_distance(9.1, 18.2, 1.2, 0.8) == pytest.approx(trapezoid / 2.0)


def test_invalid_kinematic_boundary_fails_closed():
    with pytest.raises(P2.AttributionError):
        P2.minimum_time(-1.0, 0.0, 0.0, 1.2, 0.8)
    with pytest.raises(P2.AttributionError):
        P2.minimum_time(1.0, 1.3, 0.0, 1.2, 0.8)


def test_frozen_p2_d1_rebuild_and_decision_boundary():
    report, blocks, closure = P2.build_outputs(WORKSPACE)
    assert report["status"] == "offline_trace_attribution_physical_room_pass"
    assert len(blocks) == 30
    assert [row["seed"] for row in blocks] == list(range(5261, 5291))
    assert len(closure) == 108
    assert report["frozen_p1_result"]["observed_reduction_fraction"] == pytest.approx(
        0.03762364346211233
    )
    primary = report["same_path_longitudinal_envelopes"][
        "frozen_prefix_post_cruise_primary"
    ]
    assert primary["reduction_fraction_vs_paired_legacy"] == pytest.approx(
        0.12014432991289692
    )
    assert primary["paired_bootstrap_one_sided_95_percent_lower"] > 0.11
    assert primary["physical_room_gate_pass"] is True
    assert report["decision"] == {
        "aggregate_physical_room_for_8_to_10_percent": True,
        "factor_hypothesis_retained_for_design": True,
        "specific_executable_runtime_factor_qualified": False,
        "counterfactual_gain_achieved": False,
        "causal_performance_improvement_demonstrated": False,
        "new_speed_bound_safety_qualified": False,
        "simulation_authorized": False,
        "decision_reason": "post_cruise_same_path_point_and_one_sided_lower_exceed_ten_percent_but_runtime_mapping_and_causal_gain_remain_unqualified",
    }


def test_target_recovery_and_historical_mapping_are_not_overclaimed():
    report, _, _ = P2.build_outputs(WORKSPACE)
    eight = report["target_closure"]["8percent"]
    ten = report["target_closure"]["10percent"]
    assert eight["required_fraction_of_post_cruise_recoverable_time"] == pytest.approx(
        0.5139795085583981
    )
    assert ten["required_fraction_of_post_cruise_recoverable_time"] == pytest.approx(
        0.7562315434904949
    )
    assert eight["inside_current_anchor_bank_speed_bound"] is False
    assert ten["inside_current_anchor_bank_speed_bound"] is False
    historical = report["historical_single_coordinate_check"]
    assert historical["interpretation"][
        "speed_ceiling_has_target_scale_directional_sensitivity"
    ] is True
    assert historical["interpretation"][
        "weight_optimaltime_alone_has_target_scale_evidence"
    ] is False
    assert historical["interpretation"]["new_above_1_2_speed_bound_qualified"] is False


def test_serialization_is_deterministic():
    first_report, first_blocks, _ = P2.build_outputs(WORKSPACE)
    second_report, second_blocks, _ = P2.build_outputs(WORKSPACE)
    assert P2.report_bytes(first_report) == P2.report_bytes(second_report)
    assert P2.canonical_blocks_bytes(first_blocks) == P2.canonical_blocks_bytes(second_blocks)

