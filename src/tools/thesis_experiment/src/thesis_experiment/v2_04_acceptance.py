"""Deterministic component acceptance for the V2-04 no-training rule loop."""

from pathlib import Path
from typing import Any, Dict, List, Mapping, Tuple

import yaml

from teb_mode_manager.action_pipeline import (
    AnchorBank,
    DeterministicShadowBackend,
    RuleAnchorTransactionLoop,
)


RULE_SCHEDULE = (
    ("BALANCED", "NONE", False),
    ("CRUISE", "NONE", False),
    ("CRUISE", "CROSSING", False),
    ("STATIC_DENSE", "NONE", False),
    ("STATIC_DENSE", "HEAD_ON", False),
    ("CORRIDOR", "NONE", False),
    ("CORRIDOR", "FOLLOW", False),
    ("MANEUVER", "NONE", False),
    ("MANEUVER", "OVERTAKE_OR_YIELD", True),
    ("BALANCED", "NONE", False),
)


def run_v2_04_acceptance(anchor_bank_path: Any) -> Dict[str, Any]:
    bank = AnchorBank.from_file(anchor_bank_path)
    loop = RuleAnchorTransactionLoop(bank)
    dt = 1.0 / loop.frequency_hz
    traces = []
    projection_count = 0
    activated_count = 0
    slow_commit_count = 0
    continuous_jump_count = 0
    maximum_rate_ratio = 0.0
    previous = loop.numeric_vector(loop.executed)
    previous_by_name = dict(zip(bank.parameter_names, previous))
    tick = 0
    mode_seq = 0
    switch_first_steps = []
    for geometry_mode, overlay, reverse in RULE_SCHEDULE:
        mode_seq += 1
        for local_tick in range(80):
            tick += 1
            trace = loop.update(
                tick * dt, tick, mode_seq, geometry_mode, overlay,
                "ENTERING" if local_tick < 3 else "STABLE", True,
                maneuver_reverse=reverse,
            )
            traces.append(trace)
            projection_count += int(trace.projection_reason_mask != 0)
            activated_count += int(trace.activated)
            slow_commit_count += int(trace.slow_profile_committed)
            executed_by_name = trace.stage_mapping("executed")
            for name, definition in bank.definitions.items():
                if not definition.continuous:
                    continue
                delta = abs(executed_by_name[name] - previous_by_name[name])
                allowed = definition.max_rate_per_s * dt
                ratio = delta / allowed if allowed > 0.0 else float("inf")
                maximum_rate_ratio = max(maximum_rate_ratio, ratio)
                if delta > allowed + 1.0e-12:
                    continuous_jump_count += 1
            if local_tick == 0:
                switch_first_steps.append({
                    "mode_seq": mode_seq,
                    "geometry_mode": geometry_mode,
                    "dynamic_overlay": overlay,
                    "anchor_id": trace.anchor_id,
                    "max_vel_x_commanded": trace.stage_mapping("commanded")["max_vel_x"],
                    "max_vel_x_previous_executed": previous_by_name["max_vel_x"],
                    "max_vel_x_executed": executed_by_name["max_vel_x"],
                })
            previous_by_name = executed_by_name

    reconstructible = all(
        len(trace.parameter_names) == len(bank.parameter_names)
        and len(set(trace.parameter_names)) == len(bank.parameter_names)
        and trace.parameter_names == bank.parameter_names
        and trace.parameter_types == bank.parameter_types
        and all(len(getattr(trace, stage)) == len(bank.parameter_names)
                for stage in ("commanded", "feasible", "safe", "executed"))
        and all(len(trace.typed_stage_mapping(stage)) == len(bank.parameter_names)
                for stage in ("commanded", "feasible", "safe", "executed"))
        and trace.t_request_s <= trace.t_ack_s <= trace.t_readback_s <= trace.t_active_s
        for trace in traces
    )
    zero_training = all(trace.training_used is False for trace in traces)
    fault_results = _fault_atomicity(bank, tick * dt)
    projection_rate = projection_count / float(len(traces))
    passed = (
        projection_rate < 0.10
        and continuous_jump_count == 0
        and maximum_rate_ratio <= 1.0 + 1.0e-9
        and reconstructible
        and zero_training
        and all(item["held_previous_executed"] for item in fault_results)
    )
    return {
        "schema_version": "2.0",
        "stage": "V2-04",
        "status": "passed" if passed else "failed",
        "formal_result": False,
        "simulation_only": True,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "real_teb_parameter_write_used": False,
        "execution_backend": "deterministic_shadow",
        "anchor_bank_id": "fam_teb_v2_04_anchor_bank_sim_candidate_1",
        "parameter_count": len(bank.parameter_names),
        "profile_type_counts": {
            parameter_type: bank.parameter_types.count(parameter_type)
            for parameter_type in ("double", "int", "bool")
        },
        "anchor_count": len(bank.anchors),
        "overlay_count": len(bank.overlays),
        "normal_rule_loop": {
            "schedule_segments": len(RULE_SCHEDULE),
            "transaction_count": len(traces),
            "activated_count": activated_count,
            "projection_count": projection_count,
            "projection_rate": projection_rate,
            "projection_gate_max": 0.10,
            "continuous_jump_count": continuous_jump_count,
            "maximum_continuous_rate_ratio": maximum_rate_ratio,
            "slow_profile_commit_count": slow_commit_count,
            "complete_trace_reconstruction": reconstructible,
            "all_training_used_false": zero_training,
        },
        "fault_atomicity": fault_results,
        "switch_first_steps": switch_first_steps,
        "claims": {
            "anchor_values_formally_calibrated": False,
            "navigation_performance_improved": False,
            "gazebo_navigation_closed_loop_completed": False,
            "real_vehicle_validated": False,
        },
    }


def _fault_atomicity(bank: AnchorBank, start_s: float) -> List[Dict[str, Any]]:
    results = []
    for index, fault in enumerate(("timeout", "ack_mismatch", "readback_mismatch")):
        initial = bank.anchors["anchor_balanced"]
        backend = DeterministicShadowBackend(bank, initial)
        loop = RuleAnchorTransactionLoop(bank, backend=backend)
        before = loop.numeric_vector(loop.executed)
        backend.inject_next_fault(fault)
        trace = loop.update(
            start_s + (index + 1) / loop.frequency_hz,
            index + 1, index + 1, "CRUISE", "NONE", "ENTERING", True,
        )
        held = (
            trace.executed == before
            and loop.numeric_vector(loop.executed) == before
            and loop.numeric_vector(backend.current) == before
            and not trace.activated
            and not trace.valid
        )
        results.append({
            "fault": fault,
            "held_previous_executed": held,
            "fault_reason": trace.fault_reason,
        })
    return results


def write_v2_04_acceptance(report: Mapping[str, Any], output_path: Any) -> None:
    destination = Path(output_path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        yaml.safe_dump(dict(report), sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
