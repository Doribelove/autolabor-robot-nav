import copy

import pytest

from thesis_experiment.baseline_evaluator import (
    ALGORITHMS,
    BaselineEvaluationError,
    evaluate_baselines,
    validate_baseline_contract,
)


def contract():
    return {
        "schema_version": "1.0", "status": "frozen_gazebo_t08",
        "simulation_only": True, "real_vehicle_use_forbidden": True,
        "reference_algorithm": "TEB-Default",
        "algorithms": [{"name": name, "config_version": name + "-v1"} for name in ALGORITHMS],
        "evaluation_matrix": {
            "seeds": [42, 43],
            "scenes": [
                {"scene_id": "a", "split": "validation", "layout": "clear",
                 "goal": [1, 0, 0], "timeout_s": 10},
                {"scene_id": "b", "split": "validation", "layout": "obstacle",
                 "goal": [2, 0, 0], "timeout_s": 10},
            ],
        },
    }


def rows():
    result = []
    for a_index, algorithm in enumerate(ALGORITHMS):
        for scene in ("a", "b"):
            for seed in (42, 43):
                success = not (algorithm == "Rule-TEB" and scene == "b" and seed == 43)
                result.append({
                    "algorithm": algorithm, "scene_id": scene, "seed": seed,
                    "success": success, "collision": False,
                    "termination_reason": "goal" if success else "timeout",
                    "navigation_time": 5.0 + a_index, "path_length": 2.0 + a_index,
                    "smoothness": 0.1, "min_obstacle_distance": 0.5,
                    "near_collision_time_ratio": 0.0,
                    "parameter_total_variation": float(a_index),
                })
    return result


def test_complete_matrix_reports_failures_and_paired_deltas():
    report = evaluate_baselines(rows(), contract())
    assert report["status"] == "valid" and report["complete_matrix"] is True
    assert report["episode_count"] == 16
    assert report["algorithms"]["Rule-TEB"]["failure_count"] == 1
    assert report["algorithms"]["TEB-Default"]["success_rate"] == 1.0
    assert report["paired_against_reference"]["Fixed-DWA"]["pair_count"] == 4


def test_missing_duplicate_and_unknown_rows_are_rejected():
    data = rows()
    with pytest.raises(BaselineEvaluationError, match="incomplete"):
        evaluate_baselines(data[:-1], contract())
    with pytest.raises(BaselineEvaluationError, match="duplicate"):
        evaluate_baselines(data + [dict(data[0])], contract())
    bad = copy.deepcopy(data)
    bad[0]["algorithm"] = "Unknown"
    with pytest.raises(BaselineEvaluationError, match="unexpected"):
        evaluate_baselines(bad, contract())


def test_nonfinite_and_success_semantics_fail_closed():
    bad = rows()
    bad[0]["navigation_time"] = float("nan")
    with pytest.raises(BaselineEvaluationError, match="finite"):
        evaluate_baselines(bad, contract())
    bad = rows()
    bad[0]["success"] = False
    with pytest.raises(BaselineEvaluationError, match="mismatch"):
        evaluate_baselines(bad, contract())


def test_contract_freezes_algorithm_order_and_simulation_boundary():
    validate_baseline_contract(contract())
    bad = contract()
    bad["algorithms"] = list(reversed(bad["algorithms"]))
    with pytest.raises(BaselineEvaluationError):
        validate_baseline_contract(bad)
    bad = contract()
    bad["real_vehicle_use_forbidden"] = False
    with pytest.raises(BaselineEvaluationError):
        validate_baseline_contract(bad)
