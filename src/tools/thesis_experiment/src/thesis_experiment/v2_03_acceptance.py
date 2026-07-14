"""Evaluator-only synthetic acceptance for V2-03 tracking and rule supervision."""

import math
from typing import Dict, Mapping

from nav_world_model import Detection, MultiObjectTracker, RobotState
from teb_mode_manager import FeatureSnapshot, RuleContextSupervisor, SupervisorHealth


def _tracker(config: Mapping) -> MultiObjectTracker:
    tracker, prediction = config["tracker"], config["prediction"]
    return MultiObjectTracker(
        association_gate_m=tracker["association_gate_m"],
        alpha=tracker["alpha"],
        beta=tracker["beta"],
        minimum_confirmed_hits=tracker["minimum_confirmed_hits"],
        maximum_misses=tracker["maximum_misses"],
        maximum_dt_s=tracker["maximum_dt_s"],
        stationary_speed_max_mps=tracker["stationary_speed_max_mps"],
        dynamic_speed_min_mps=tracker["dynamic_speed_min_mps"],
        prediction_horizon_s=prediction["horizon_s"],
        prediction_step_s=prediction["step_s"],
        confidence_decay_per_s=prediction["confidence_decay_per_s"],
        crossing_lateral_speed_min_mps=tracker["crossing_lateral_speed_min_mps"],
        crossing_path_half_width_m=tracker["crossing_path_half_width_m"],
    )


def _features(stamp: float, **changes) -> FeatureSnapshot:
    values = dict(
        world_model_seq=int(stamp * 10) + 1,
        stamp_s=stamp,
        front_clearance_m=4.0,
        rear_clearance_m=4.0,
        obstacle_density=0.02,
        static_persistence=0.0,
        corridor_width_m=0.0,
        corridor_parallel_confidence=0.0,
        dead_end_score=0.0,
        path_curvature=0.15,
        goal_direction_stability=0.5,
        rear_covered=True,
    )
    values.update(changes)
    return FeatureSnapshot(**values)


def _settled_mode(config: Mapping, changes: Dict) -> str:
    supervisor = RuleContextSupervisor(dict(config))
    decision = None
    for index in range(25):
        decision = supervisor.update(
            _features(index * 0.1, **changes), (), SupervisorHealth(True, False)
        )
    return decision.geometry_mode


def evaluate_v2_03_synthetic(
    world_config: Mapping, supervisor_config: Mapping, contract: Mapping
) -> Dict:
    """Evaluate against synthetic truth kept outside both runtime algorithms."""

    tracker = _tracker(world_config)
    position_errors, prediction_errors, track_ids = [], [], []
    for index in range(50):
        stamp = index * 0.1
        truth_x, truth_y = 10.0, -2.0 + stamp
        measurement_y = truth_y + 0.03 * math.sin(index * 0.7)
        estimates = tracker.update(
            [Detection(truth_x, measurement_y, 0.30, 6)], stamp, RobotState()
        )
        if not estimates:
            continue
        estimate = estimates[0]
        track_ids.append(estimate.track_id)
        position_errors.append(math.hypot(estimate.x - truth_x, estimate.y - truth_y))
        if index >= 12:
            prediction = min(
                estimate.predictions, key=lambda item: abs(item.time_from_start_s - 1.0)
            )
            prediction_errors.append(
                math.hypot(prediction.x - truth_x, prediction.y - (truth_y + 1.0))
            )
    position_rmse = math.sqrt(sum(value * value for value in position_errors) / len(position_errors))
    prediction_rmse = math.sqrt(
        sum(value * value for value in prediction_errors) / len(prediction_errors)
    )
    id_switches = sum(first != second for first, second in zip(track_ids, track_ids[1:]))

    mode_cases = {
        "BALANCED": {},
        "CRUISE": dict(front_clearance_m=20.0, obstacle_density=0.0,
                       path_curvature=0.0, goal_direction_stability=1.0),
        "STATIC_DENSE": dict(obstacle_density=0.16, static_persistence=0.8),
        "CORRIDOR": dict(front_clearance_m=8.0, corridor_width_m=2.0,
                         corridor_parallel_confidence=0.90),
        "MANEUVER": dict(front_clearance_m=0.5, dead_end_score=0.90,
                         rear_covered=True),
    }
    mode_predictions = {
        expected: _settled_mode(supervisor_config, features)
        for expected, features in mode_cases.items()
    }
    correct = sum(expected == predicted for expected, predicted in mode_predictions.items())
    macro_recall = correct / len(mode_cases)
    confusion = {
        expected: {mode: int(mode_predictions[expected] == mode) for mode in mode_cases}
        for expected in mode_cases
    }

    fault_reasons = ("scan_metadata", "scan_stale", "tf_timeout", "localization_stale")
    fault_passed = 0
    for reason in fault_reasons:
        supervisor = RuleContextSupervisor(dict(supervisor_config))
        decision = supervisor.update(
            _features(0.0), (), SupervisorHealth(False, True, reason)
        )
        fault_passed += int(
            not decision.valid
            and decision.geometry_mode == "BALANCED"
            and decision.dynamic_overlay == "NONE"
            and decision.transition_state == "FAULTED"
        )

    gates = contract["acceptance_gates"]
    checks = {
        "tracking_position_rmse": position_rmse <= gates["synthetic_tracking_position_rmse_max_m"],
        "prediction_rmse": prediction_rmse <= gates["synthetic_prediction_rmse_max_m"],
        "id_switches": id_switches <= gates["synthetic_id_switches_max"],
        "mode_macro_recall": macro_recall >= gates["synthetic_mode_macro_recall_min"],
        "health_fault_cases": fault_passed >= gates["health_fault_cases_required"],
    }
    return {
        "schema_version": "2.0",
        "architecture_generation": "v2",
        "suite": "v2_03_synthetic_world_model_rule_acceptance",
        "simulation_only": True,
        "formal_result": False,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_started": False,
        "truth_scope": "evaluator_only_synthetic",
        "passed": all(checks.values()),
        "checks": checks,
        "tracking": {
            "sample_count": len(position_errors),
            "position_rmse_m": position_rmse,
            "prediction_1s_rmse_m": prediction_rmse,
            "id_switches": id_switches,
            "unique_track_ids": sorted(set(track_ids)),
        },
        "mode_supervisor": {
            "macro_recall": macro_recall,
            "predictions": mode_predictions,
            "confusion_matrix": confusion,
        },
        "health": {
            "fault_cases_required": list(fault_reasons),
            "fault_cases_passed": fault_passed,
        },
    }
