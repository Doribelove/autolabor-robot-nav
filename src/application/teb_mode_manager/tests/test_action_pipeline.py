from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from teb_mode_manager.action_pipeline import (
    ActionPipelineError,
    AnchorBank,
    DeterministicShadowBackend,
    FeasibleActionDecoder,
    PROJECTION_SPEED_ENVELOPE,
    RuleAnchorTransactionLoop,
)


CONFIG = Path(__file__).parents[1] / "config/v2_04_anchor_bank_candidate.yaml"


def load_bank():
    return AnchorBank.from_file(CONFIG)


def test_anchor_bank_has_exact_typed_profiles_and_safety_boundary():
    bank = load_bank()
    assert len(bank.parameter_names) == 20
    assert bank.parameter_types.count("double") == 18
    assert bank.parameter_types.count("int") == 1
    assert bank.parameter_types.count("bool") == 1
    assert set(bank.anchors) == {
        "anchor_balanced", "anchor_cruise", "anchor_static_dense",
        "anchor_corridor", "anchor_maneuver_forward", "anchor_maneuver_reverse",
    }
    assert bank.transaction["backend"] == "deterministic_shadow"
    assert bank.transaction["dynamic_reconfigure_enabled"] is False


def test_all_zero_residual_anchor_overlay_pairs_are_intrinsically_feasible():
    bank = load_bank()
    decoder = FeasibleActionDecoder(bank)
    results = []
    for anchor_id in bank.anchors:
        for overlay in bank.overlays:
            result = decoder.decode_anchor(anchor_id, overlay)
            results.append(result)
            assert result.projection_reason_mask == 0
            assert result.commanded.values == result.feasible.values
            values = result.feasible.values
            assert values["max_vel_theta"] <= (
                values["max_vel_x"] / bank.minimum_turning_radius_m + 1.0e-12
            )
            assert values["inflation_dist"] >= (
                values["min_obstacle_dist"] + bank.minimum_inflation_gap_m
            )
            assert values["dynamic_obstacle_inflation_dist"] >= (
                values["min_obstacle_dist"] + bank.minimum_dynamic_inflation_gap_m
            )
    assert len(results) == 30


def test_decoder_rejects_invalid_residual_and_audits_speed_envelope():
    decoder = FeasibleActionDecoder(load_bank())
    with pytest.raises(ActionPipelineError):
        decoder.decode("CRUISE", "NONE", {"max_vel_x": 1.01})
    with pytest.raises(ActionPipelineError):
        decoder.decode("CRUISE", "NONE", {"max_number_classes": 0.0})
    result = decoder.decode("CRUISE", "NONE", speed_envelope_mps=0.75)
    assert result.projection_reason_mask & PROJECTION_SPEED_ENVELOPE
    assert result.feasible.values["max_vel_x"] == pytest.approx(0.75)
    assert result.feasible.values["max_vel_theta"] <= 0.625 + 1.0e-12


def test_extreme_normalized_residuals_decode_natively_inside_coupled_domain():
    bank = load_bank()
    decoder = FeasibleActionDecoder(bank)
    residuals = {
        name: (-1.0 if index % 2 == 0 else 1.0)
        for index, (name, definition) in enumerate(bank.definitions.items())
        if definition.lifecycle == "fast_continuous"
    }
    result = decoder.decode("CRUISE", "HEAD_ON", residuals=residuals)
    assert result.projection_reason_mask == 0
    values = result.feasible.values
    assert values["max_vel_theta"] <= values["max_vel_x"] / bank.minimum_turning_radius_m
    assert values["inflation_dist"] >= values["min_obstacle_dist"] + 0.20
    assert values["dynamic_obstacle_inflation_dist"] >= values["min_obstacle_dist"] + 0.20
    assert all(values[name] > 0.0 for name in decoder.WEIGHT_NAMES)


def test_transition_origin_is_previous_executed_and_rate_limited():
    bank = load_bank()
    loop = RuleAnchorTransactionLoop(bank)
    first = loop.update(0.2, 1, 1, "CRUISE", "NONE", "ENTERING", True)
    max_vel_index = first.parameter_names.index("max_vel_x")
    assert first.commanded[max_vel_index] == pytest.approx(1.20)
    assert first.executed[max_vel_index] == pytest.approx(1.00)
    second = loop.update(0.4, 2, 2, "CORRIDOR", "NONE", "ENTERING", True)
    assert second.commanded[max_vel_index] == pytest.approx(0.50)
    assert second.executed[max_vel_index] == pytest.approx(0.90)
    assert abs(second.executed[max_vel_index] - first.executed[max_vel_index]) <= 0.10 + 1e-12
    assert first.training_used is False and second.training_used is False


def test_discrete_profile_commits_only_after_continuous_convergence():
    bank = load_bank()
    loop = RuleAnchorTransactionLoop(bank)
    class_index = bank.parameter_names.index("max_number_classes")
    first = loop.update(0.2, 1, 1, "STATIC_DENSE", "NONE", "ENTERING", True)
    assert first.executed[class_index] == 2.0
    assert first.slow_profile_committed is False
    last = first
    for index in range(1, 200):
        last = loop.update(
            0.2 * (index + 1), index + 1, 1,
            "STATIC_DENSE", "NONE", "STABLE", True,
        )
        if last.slow_profile_committed:
            break
    assert last.slow_profile_committed is True
    assert last.executed[class_index] == 4.0


def test_transaction_fault_holds_previous_executed_atomically():
    for fault in ("timeout", "ack_mismatch", "readback_mismatch"):
        bank = load_bank()
        initial = bank.anchors["anchor_balanced"]
        backend = DeterministicShadowBackend(bank, initial)
        loop = RuleAnchorTransactionLoop(bank, backend=backend)
        before = loop.numeric_vector(loop.executed)
        backend.inject_next_fault(fault)
        trace = loop.update(0.2, 1, 1, "CRUISE", "NONE", "ENTERING", True)
        assert trace.valid is False
        assert trace.activated is False
        assert trace.executed == before
        assert loop.numeric_vector(loop.executed) == before
        assert loop.numeric_vector(backend.current) == before


def test_action_trace_is_complete_reconstructible_and_timestamp_ordered():
    bank = load_bank()
    trace = RuleAnchorTransactionLoop(bank).update(
        0.2, 7, 3, "CRUISE", "CROSSING", "ENTERING", True
    )
    assert len(trace.parameter_names) == len(set(trace.parameter_names)) == 20
    assert len(trace.parameter_types) == 20
    for stage in ("commanded", "feasible", "safe", "executed"):
        mapping = trace.stage_mapping(stage)
        assert tuple(mapping) == trace.parameter_names
        assert len(mapping) == 20
        typed = trace.typed_stage_mapping(stage)
        assert isinstance(typed["include_dynamic_obstacles"], bool)
        assert isinstance(typed["max_number_classes"], int)
        assert isinstance(typed["max_vel_x"], float)
    assert trace.t_request_s <= trace.t_ack_s <= trace.t_readback_s <= trace.t_active_s
    assert trace.world_model_seq == 7
    assert trace.mode_seq == 3
    assert trace.execution_backend == "deterministic_shadow"


def test_invalid_context_publishes_invalid_hold_without_activation():
    loop = RuleAnchorTransactionLoop(load_bank())
    before = loop.numeric_vector(loop.executed)
    trace = loop.update(0.2, 0, 0, "BALANCED", "NONE", "FAULTED", False)
    assert trace.valid is False
    assert trace.activated is False
    assert trace.executed == before
    assert trace.commanded == trace.feasible == trace.safe == trace.executed


def test_anchor_bank_fail_closed_on_boundary_or_type_drift():
    mutations = (
        lambda data: data.update(runtime_ready=True),
        lambda data: data["transaction"].update(dynamic_reconfigure_enabled=True),
        lambda data: data["policy_boundary"].update(learned_policy_loaded=True),
        lambda data: data["source_provenance"].update(formal_test_scenes_used=True),
        lambda data: data["anchors"]["anchor_cruise"]["values"].update(max_number_classes=1.5),
    )
    for mutation in mutations:
        data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))
        changed = deepcopy(data)
        mutation(changed)
        with pytest.raises(ActionPipelineError):
            AnchorBank(changed)
