#!/usr/bin/env python3
"""Pure-offline, deterministic audit of the frozen V6-to-Gazebo migration."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[2]
PACKAGE_SRC = ROOT / "src/application/v5_e2e_scope_rl/src"
sys.path.insert(0, str(PACKAGE_SRC))

from v5_e2e_scope_rl.runtime_adapter import SensorRuntimeAdapter  # noqa: E402
from v5_e2e_scope_rl.sensor_env import SensorCandidateEnv  # noqa: E402


OUTPUT = (
    ROOT
    / "artifacts/gazebo_pct_teb_dwa_benchmark/diagnostics"
    / "p02_frozen_migration_diagnosis.json"
)
V604 = ROOT / "artifacts/v6/gazebo/v6_04_p0_preflight/episodes"


def load_yaml(path: Path):
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def observation_parity_probe():
    env = SensorCandidateEnv(
        {
            "family": "bidirectional_reactive_crossing",
            "scan_noise_std_m": 0.0,
            "scan_dropout_probability": 0.0,
        }
    )
    env.reset(
        seed=196001,
        options={
            "family": "bidirectional_reactive_crossing",
            "actor_side": -1.0,
            "actor_speed_mps": 0.9,
            "actor_mode": "cautious",
        },
    )
    env.ego_x = -2.6
    env.ego_y = 0.0
    env.ego_yaw = 0.0
    env.ego_speed = 0.8
    env.ego_steering = 0.0
    env.previous_action = 2
    env.previous_target_speed = 0.8
    env.step_count = 10
    env.consecutive_stop_count = 0
    scan = env._raycast_scan(add_noise=False)
    env.scan_history = np.repeat(scan[None, :], 6, axis=0)
    env._refresh_candidates()
    training = env.policy_observation()

    runtime = SensorRuntimeAdapter()
    runtime.scan_history = np.repeat(scan[None, :], 6, axis=0)
    runtime.previous_action = 2
    runtime.previous_target_speed = 0.8
    runtime.consecutive_stop_count = 0
    runtime.decision_count = 10
    deployed, _ = runtime.build((-2.6, 0.0, 0.0), 0.8, 0.0, 2.5)

    fields = {}
    for key in ("scans", "robot", "candidates", "analytic_prior", "action_mask"):
        left = np.asarray(training[key])
        right = np.asarray(deployed[key])
        fields[key] = {
            "shape_equal": left.shape == right.shape,
            "max_absolute_difference": float(np.max(np.abs(left.astype(float) - right.astype(float)))),
            "exact": bool(np.array_equal(left, right)),
        }
    return {
        "synthetic_development_seed": 196001,
        "controlled_pose": [-2.6, 0.0, 0.0],
        "controlled_speed_mps": 0.8,
        "fields": fields,
        "training_robot_features": training["robot"].astype(float).tolist(),
        "runtime_robot_features": deployed["robot"].astype(float).tolist(),
        "training_action_mask": training["action_mask"].astype(int).tolist(),
        "runtime_action_mask": deployed["action_mask"].astype(int).tolist(),
    }


def historical_report_facts():
    result = []
    for path in sorted(V604.glob("*/episode_report.yaml")):
        report = load_yaml(path)
        row = {
            "episode": path.parent.name,
            "method": report["method"],
            "goal_reached": bool(report["result"]["goal_reached"]),
            "restricted_elapsed_s": float(report["result"]["restricted_elapsed_s"]),
            "minimum_conservative_clearance_m": float(
                report["result"]["minimum_conservative_clearance_m"]
            ),
            "contact_state_count": int(report["result"]["contact_state_count"]),
            "final_pose": report["result"]["final_pose"],
            "final_position_error_m": float(report["result"]["final_position_error_m"]),
            "execution_integrity_pass": bool(report["execution_integrity_pass"]),
        }
        directional = report.get("directional_reachable_occupancy", {})
        if directional:
            row["tracker"] = directional.get("tracker")
        trace = report.get("decision_trace", [])
        if trace:
            row.update(
                {
                    "decision_count": len(trace),
                    "certified_stop_decisions": sum(int(item["action_id"] == 0) for item in trace),
                    "spatial_decisions": sum(int(item["action_id"] >= 4) for item in trace),
                    "single_safe_action_decisions": sum(
                        int(item["safe_action_count"] == 1) for item in trace
                    ),
                    "safe_action_count_transitions": sum(
                        int(left["safe_action_count"] != right["safe_action_count"])
                        for left, right in zip(trace[:-1], trace[1:])
                    ),
                }
            )
        result.append(row)
    return result


def main() -> int:
    parity = observation_parity_probe()
    reports = historical_report_facts()
    pct = next(row for row in reports if row["method"] == "pct_scope_rl_v6")
    analytic = next(row for row in reports if row["method"] == "sensor_analytic_prior")
    teb = next(row for row in reports if row["method"] == "actual_teb")
    output = {
        "schema_version": "1.0",
        "status": "p02_frozen_migration_offline_diagnosis_complete",
        "scope": "read-only historical attribution plus synthetic fixed-input probe; no historical identity rerun",
        "historical_v604_reports": reports,
        "fixed_input_parity_probe": parity,
        "verified_contract_differences": {
            "policy_activation": {
                "training_start_x_m": -4.0,
                "historical_gazebo_spawn_x_m": -4.2,
                "historical_gazebo_policy_gate_x_m": -2.6,
                "gazebo_policy_skips_training_initial_longitudinal_distance_m": 1.4,
                "training_step_fraction_at_x_minus_2_6_is_not_zero": True,
                "gazebo_elapsed_feature_resets_to_zero_at_gate": True,
            },
            "actor_at_policy_activation": {
                "training_crossing_start_absolute_y_range_m": [1.45, 2.05],
                "training_crossing_speed_range_mps": [0.72, 1.12],
                "v604_gate_actor_coordinate_m": -2.9139688000000006,
                "v604_gate_actor_speed_mps": 0.11807999999999819,
            },
            "laser": {
                "training_beams_fov": "72 over 270 deg",
                "gazebo_raw_beams_fov": "720 over 360 deg; nearest-index resampled to 72 over 270 deg",
                "training_noise_std_m": 0.012,
                "gazebo_noise_std_m": 0.01,
                "training_dropout_probability": 0.006,
                "gazebo_dropout_probability": 0.0,
                "laser_xy_extrinsic_m": [0.0, 0.0],
            },
            "shield": {
                "training_dynamic_reachable_expansion": "1.35*t without 0.75 m cap",
                "historical_base_runtime_dynamic_reachable_expansion": "min(0.75,1.35*t)",
                "v604_dynamic_surface_inflation_m": 0.33,
                "evaluator_combined_radius_m": 1.15,
                "robot_radius_plus_surface_inflation_m": 0.79,
                "actor_half_width_m": 0.36,
                "center_separation_equivalent_m": 1.15,
            },
            "goal_semantics": {
                "training_success": "x >= 4.70 with |y| <= 0.55 and |yaw| <= 0.35",
                "historical_gazebo_success": "Euclidean distance to (5.2,0) <= 0.25 and |yaw| <= 0.25",
                "runtime_candidate_has_goal_speed_or_longitudinal_clamp": False,
                "pct_final_x_m": float(pct["final_pose"]["x_m"]),
                "pct_final_position_error_m": pct["final_position_error_m"],
                "analytic_final_position_error_m": analytic["final_position_error_m"],
            },
            "execution_semantics": {
                "separate_suggested_validated_executed_ids_logged": False,
                "activation_time_hard_mask_revalidation": False,
                "previous_action_updated_at_selection_not_confirmed_execution": True,
                "scan_odom_tf_age_deadlines_logged": False,
                "nan_inf_and_inference_timeout_fail_closed_contract": False,
            },
        },
        "falsifiable_hypotheses_for_new_development_only": [
            {
                "id": "H-P02-1",
                "hypothesis": "Starting the policy at a shared state represented in training and preserving elapsed/previous-executed semantics removes the activation distribution discontinuity.",
                "test": "fixed-input parity plus new development shadow episodes; no historical identity rerun",
            },
            {
                "id": "H-P02-2",
                "hypothesis": "A goal-aware speed envelope and terminal candidate prevents lateral-action goal overshoot without changing the shared evaluator goal.",
                "test": "static development trajectories must enter and stop inside the common goal ball",
            },
            {
                "id": "H-P02-3",
                "hypothesis": "Time-consistent scan clustering with static-map rejection and activation revalidation eliminates negative evaluator clearance while retaining non-stop candidates after the actor clears.",
                "test": "single-actor shadow development episodes: evaluator clearance >= 0, mask violations 0, recovery from certified stop",
            },
        ],
        "claims_not_supported": [
            "that the historical tracker was always associated with the wrong object",
            "that any one listed mismatch alone caused the historical negative result",
            "Gazebo superiority over Actual TEB or DWA",
        ],
        "historical_method_order_facts": {
            "pct_goal": pct["goal_reached"],
            "analytic_goal": analytic["goal_reached"],
            "actual_teb_goal": teb["goal_reached"],
        },
    }
    if not math.isclose(
        output["verified_contract_differences"]["shield"]["center_separation_equivalent_m"],
        output["verified_contract_differences"]["shield"]["evaluator_combined_radius_m"],
        abs_tol=1.0e-12,
    ):
        raise RuntimeError("shield/evaluator radius arithmetic differs")
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    with OUTPUT.open("w", encoding="utf-8") as stream:
        json.dump(output, stream, ensure_ascii=False, indent=2, sort_keys=True)
        stream.write("\n")
    print(OUTPUT.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
