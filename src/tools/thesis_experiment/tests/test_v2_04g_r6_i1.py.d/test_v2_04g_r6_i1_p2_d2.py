import copy
import importlib.util
from pathlib import Path

import pytest
import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
SOURCE = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v2_04g_r6_i1_p2_d2.py"
)


def _module():
    spec = importlib.util.spec_from_file_location("p2_d2_under_test", SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


P2 = _module()


def test_stopping_envelope_and_power_are_frozen():
    normal = P2.stopping_envelope(1.4, 0.99, 1.6, 0.25)
    fault = P2.stopping_envelope(1.4, 1.49, 1.6, 0.25)
    assert normal["latency_distance_m"] == pytest.approx(1.386)
    assert normal["service_braking_distance_m"] == pytest.approx(0.6125)
    assert normal["required_forward_free_path_m"] == pytest.approx(2.2485)
    assert fault["required_forward_free_path_m"] == pytest.approx(2.9485)
    assert P2.approximate_required_pairs(0.025, 0.90, 0.15, 0.10) == 22


def test_frozen_p2_d2_rebuild_and_permission_boundary():
    report, amendment, closure = P2.build_outputs(WORKSPACE)
    assert report["status"] == "offline_single_factor_design_closure_pass"
    assert len(closure) == 10
    assert len({row["path"] for row in closure}) == 10
    assert report["selected_design_factor"] == {
        "factor_id": "p2_cruise_forward_velocity_ceiling_v1",
        "runtime_field": "TebLocalPlannerROS.max_vel_x",
        "behavioral_field_count": 1,
        "control_value_mps": 1.2,
        "candidate_grid_mps": [1.25, 1.3, 1.35, 1.4],
        "design_upper_bound_mps": 1.4,
        "all_other_behavioral_fields_frozen": True,
        "runtime_label_access": False,
        "selection_uses_calibration_only": True,
        "held_out_test_used_for_selection": False,
    }
    assert amendment["behavioral_diff_contract"]["behavioral_field_count"] == 1
    assert [
        row["kind"] for row in amendment["materialization_diff_allowlist"]
    ] == ["registry_metadata", "sole_behavioral_field"]
    assert amendment["parameter_write_enabled"] is False
    assert amendment["execution_authorized"] is False
    assert report["decision"]["simulation_release_exists"] is False
    assert report["decision"]["simulation_authorized"] is False
    assert report["decision"]["performance_improvement_demonstrated"] is False


def test_fair_baseline_and_global_budget_are_not_weak_or_open_ended():
    report, _, _ = P2.build_outputs(WORKSPACE)
    fairness = report["fair_baseline_firewall"]
    assert fairness["transparent_reference"]["method_id"] == "TEB-Default"
    assert fairness["transparent_reference"]["primary_baseline_eligible"] is False
    assert (
        fairness["required_fixed_baseline_candidates"]["TEB-Tuned-V1"]["theta"]
        == P2.EXPECTED_TEB_TUNED_V1
    )
    assert (
        fairness["required_fixed_baseline_candidates"][
            "independently_tuned_fixed_teb"
        ]["grid_mps"]
        == [1.2, 1.25, 1.3, 1.35, 1.4]
    )
    assert fairness["baseline_weakening_or_test_selection_allowed"] is False
    program = report["finite_program"]
    assert program["maximum_factor_rounds"] == 3
    assert program["global_simulation_evidence_units_max"] == 2200
    assert program["global_training_environment_steps_max"] == 120000
    assert program["global_one_sided_alpha_spend"] == pytest.approx(0.05)
    assert program["round_1"]["total_evidence_units_max"] == 792
    assert program["round_1"]["test_attempts_per_identity"] == 1
    assert program["unused_budget_or_alpha_recycled"] is False


def test_contract_mutations_fail_closed():
    contract = yaml.safe_load(
        P2.stable_read(WORKSPACE, P2.CONTRACT_RELATIVE).decode("utf-8")
    )
    unsafe = copy.deepcopy(contract)
    unsafe["execution_authorized"] = True
    with pytest.raises(P2.DesignError):
        P2.validate_contract(unsafe)
    multi_factor = copy.deepcopy(contract)
    multi_factor["single_executable_factor"]["behavioral_field_count"] = 2
    with pytest.raises(P2.DesignError):
        P2.validate_contract(multi_factor)
    open_budget = copy.deepcopy(contract)
    open_budget["global_research_budget"]["rounds"][0][
        "simulation_evidence_units_max"
    ] += 1
    with pytest.raises(P2.DesignError):
        P2.validate_contract(open_budget)
    weaker_gate = copy.deepcopy(contract)
    weaker_gate["held_out_statistics"]["minimum_effect"][
        "point_reduction_fraction_at_least"
    ] = 0.05
    with pytest.raises(P2.DesignError):
        P2.validate_contract(weaker_gate)


def test_serialization_is_deterministic():
    first_report, first_amendment, _ = P2.build_outputs(WORKSPACE)
    second_report, second_amendment, _ = P2.build_outputs(WORKSPACE)
    assert P2.report_bytes(first_report) == P2.report_bytes(second_report)
    assert P2.amendment_bytes(first_amendment) == P2.amendment_bytes(
        second_amendment
    )
