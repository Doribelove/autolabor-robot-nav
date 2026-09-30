#!/usr/bin/env python3
"""Run deterministic non-ROS admission gates before any new Gazebo episode."""

from __future__ import annotations

import hashlib
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch
import yaml


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "src/application/v5_e2e_scope_rl/src"))

from v5_e2e_scope_rl.asymmetric_ppo import (  # noqa: E402
    SensorActor,
    observations_to_torch,
    stack_observations,
)
from v5_e2e_scope_rl.benchmark_runtime import (  # noqa: E402
    BenchmarkSensorCandidateEnv,
    BenchmarkRuntimeAdapter,
    RuntimeInputHealth,
)


OUTPUT = (
    ROOT
    / "artifacts/gazebo_pct_teb_dwa_benchmark/admission"
    / "p02_offline_admission.json"
)
SELECTION = ROOT / "artifacts/v6/training/v6_01_r1_validation/selection_report.json"
CONTRACT = ROOT / "src/application/v5_e2e_scope_rl/config/pct_teb_dwa_benchmark_development.yaml"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def controlled_observation():
    env = BenchmarkSensorCandidateEnv(
        {
            "family": "bidirectional_reactive_crossing",
            "scan_noise_std_m": 0.0,
            "scan_dropout_probability": 0.0,
            "shield_inflation_m": 0.33,
            "safety_response_delay_s": 0.35,
            "certified_stop_dwell_s": 1.50,
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
    runtime = BenchmarkRuntimeAdapter(
        goal_speed_envelope=False,
        safety_response_delay_s=0.35,
        certified_stop_dwell_s=1.50,
    )
    runtime.scan_history = np.repeat(scan[None, :], 6, axis=0)
    runtime.previous_action = 2
    runtime.previous_target_speed = 0.8
    runtime.consecutive_stop_count = 0
    runtime.decision_count = 10
    deployed, previews = runtime.build((-2.6, 0.0, 0.0), 0.8, 0.0, 2.5)
    return env.policy_observation(), deployed, previews, runtime


def main() -> int:
    contract = yaml.safe_load(CONTRACT.read_text(encoding="utf-8"))
    training, deployed, previews, runtime = controlled_observation()
    parity = {}
    for key in ("scans", "robot", "candidates", "analytic_prior", "action_mask"):
        left = np.asarray(training[key])
        right = np.asarray(deployed[key])
        parity[key] = {
            "exact": bool(np.array_equal(left, right)),
            "maximum_absolute_difference": float(
                np.max(np.abs(left.astype(float) - right.astype(float)))
            ),
        }

    selection = json.loads(SELECTION.read_text(encoding="utf-8"))
    batch = observations_to_torch(stack_observations([deployed]), torch.device("cpu"))
    export_rows = []
    for item in selection["selected_checkpoints"]:
        path = ROOT / item["checkpoint_path"]
        actor = SensorActor.from_actor_checkpoint(
            torch.load(path, map_location="cpu", weights_only=False)
        )
        actor.eval()
        with torch.no_grad():
            framework_action, framework = actor.choose_actions(
                batch, deterministic=True, advantage_gate_margin=0.10
            )
        deployed_runtime = BenchmarkRuntimeAdapter(
            actor=actor,
            goal_speed_envelope=False,
            inference_deadline_ms=1000.0,
            safety_response_delay_s=0.35,
            certified_stop_dwell_s=1.50,
        )
        deployed_runtime.scan_history = runtime.scan_history.copy()
        deployed_runtime.previous_action = runtime.previous_action
        deployed_runtime.previous_target_speed = runtime.previous_target_speed
        deployed_runtime.decision_count = runtime.decision_count
        suggestion = deployed_runtime.suggest(
            (-2.6, 0.0, 0.0),
            0.8,
            0.0,
            2.5,
            RuntimeInputHealth(0.01, 0.01, 0.01),
        )
        framework_logits = framework["logits"][0].detach().cpu().numpy()
        with torch.no_grad():
            runtime_logits = actor(batch)[0][0].detach().cpu().numpy()
        export_rows.append(
            {
                "identity": item["identity"],
                "checkpoint_path": item["checkpoint_path"],
                "checkpoint_sha256_expected": item["checkpoint_sha256"],
                "checkpoint_sha256_observed": digest(path),
                "checkpoint_hash_exact": digest(path) == item["checkpoint_sha256"],
                "framework_action": int(framework_action[0]),
                "runtime_suggested_action": int(suggestion["suggested_action"]),
                "action_exact": int(framework_action[0]) == int(suggestion["suggested_action"]),
                "maximum_logit_difference": float(np.max(np.abs(framework_logits - runtime_logits))),
                "runtime_latency_ms": float(suggestion["latency_ms"]),
            }
        )

    candidate_rows = []
    for action, preview in enumerate(previews):
        accelerations = []
        steering = []
        curvatures = []
        for left, right in zip(preview[:-1], preview[1:]):
            accelerations.append((float(right[3]) - float(left[3])) / 0.25)
            steering.append(abs(float(right[4])))
            if abs(float(right[3])) > 1.0e-9:
                curvatures.append(abs(math.tan(float(right[4])) / 0.65))
        candidate_rows.append(
            {
                "action_id": action,
                "point_count": len(preview),
                "maximum_acceleration_mps2": max(accelerations),
                "maximum_deceleration_mps2": max([-value for value in accelerations]),
                "maximum_steering_rad": max(steering),
                "maximum_curvature_per_m": max(curvatures) if curvatures else 0.0,
                "finite": bool(np.all(np.isfinite(np.asarray(preview)))),
            }
        )

    healthy = RuntimeInputHealth(0.01, 0.01, 0.01)
    suggestion = runtime.suggest((-2.6, 0.0, 0.0), 0.8, 0.0, 2.5, healthy)
    before = runtime.previous_action
    validation = runtime.revalidate(
        suggestion, (-2.6, 0.0, 0.0), 0.8, 0.0, 2.5, healthy
    )
    before_commit = runtime.previous_action
    commit = runtime.commit_executed(validation["validated_action"])
    stale_runtime = BenchmarkRuntimeAdapter(goal_speed_envelope=False)
    stale_runtime.scan_history = runtime.scan_history.copy()
    stale = stale_runtime.suggest(
        (-2.6, 0.0, 0.0),
        0.8,
        0.0,
        2.5,
        RuntimeInputHealth(0.21, 0.01, 0.01),
    )

    offline_gates = {
        "fixed_input_model_regression": all(row["exact"] for row in parity.values()),
        "model_export_regression": all(
            row["checkpoint_hash_exact"]
            and row["action_exact"]
            and row["maximum_logit_difference"] <= 1.0e-6
            for row in export_rows
        ),
        "candidate_count_and_finiteness": len(previews) == 8
        and all(row["point_count"] == 13 and row["finite"] for row in candidate_rows),
        "candidate_acceleration_bound": all(
            row["maximum_acceleration_mps2"] <= 0.6000001 for row in candidate_rows
        ),
        "candidate_deceleration_bound": all(
            row["maximum_deceleration_mps2"] <= 0.9000001 for row in candidate_rows
        ),
        "candidate_steering_bound": all(
            row["maximum_steering_rad"] <= 0.4000001 for row in candidate_rows
        ),
        "stale_input_certified_stop": stale["suggested_action"] == 0,
        "selection_does_not_mutate_previous_execution": before == before_commit,
        "commit_updates_previous_execution": runtime.previous_action == commit["executed_action"],
    }
    report = {
        "schema_version": "1.0",
        "stage": "P02",
        "status": "p02_offline_admission_pass_live_gates_pending"
        if all(offline_gates.values())
        else "p02_offline_admission_fail",
        "development_contract_path": str(CONTRACT.relative_to(ROOT)),
        "development_contract_sha256": digest(CONTRACT),
        "formal_execution_authorized": bool(contract["formal_execution_authorized"]),
        "synthetic_development_seed": 196001,
        "fixed_input_parity": parity,
        "model_export_regression": export_rows,
        "candidate_contract": candidate_rows,
        "permission_and_execution_transaction": {
            "suggested_action": suggestion["suggested_action"],
            "validated_action": validation["validated_action"],
            "executed_action": commit["executed_action"],
            "previous_before_suggestion": before,
            "previous_before_commit": before_commit,
            "previous_after_commit": runtime.previous_action,
            "stale_fallback_action": stale["suggested_action"],
            "stale_fallback_reason": stale["fallback_reason"],
        },
        "offline_gates": offline_gates,
        "live_gates": {
            "ros_scan_odometry_tf_timestamp_and_age": "pending",
            "static_shadow": "pending",
            "single_actor_shadow": "pending",
            "partially_occluded_actor_shadow": "pending",
            "topic_frequency_latency_deadline_and_contact": "pending",
        },
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(report["status"])
    print(OUTPUT.relative_to(ROOT))
    return 0 if all(offline_gates.values()) else 2


if __name__ == "__main__":
    raise SystemExit(main())
