#!/usr/bin/env python3
"""Create the immutable 20-block pilot schedule, preregistration and release."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
PILOT = ARTIFACT / "pilot/pilot_r1"
SCHEDULE = PILOT / "pilot_schedule.csv"
PREREG = PILOT / "pilot_preregistration.yaml"
RELEASE = PILOT / "pilot_execution_release.yaml"
REVIEW = PILOT / "pilot_release_review.json"
SCENES = (
    "core_static_avoidance",
    "core_single_crossing",
    "core_head_on_yield",
    "core_sequential_interaction",
    "core_partially_occluded",
    "core_width_change",
)
METHODS = ("pct_scope_rl", "analytic_prior", "actual_teb", "dwa")
LOCAL_RESOURCES = (
    "scripts/gazebo_pct_teb_dwa_benchmark/run_frozen_matrix.py",
    "scripts/gazebo_pct_teb_dwa_benchmark/run_development_matrix.py",
    "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py",
    "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py",
    "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/asymmetric_ppo.py",
    "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/runtime_adapter.py",
    "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/sensor_env.py",
    "src/application/v3_scope_planner/src/v3_scope_planner/execution_core.py",
    "src/simulation/m2_gazebo/launch/m2_pct_teb_dwa_benchmark_episode.launch",
    "src/simulation/m2_gazebo/launch/spawn_m2.launch",
    "src/simulation/m2_gazebo/urdf/m2.urdf.xacro",
    "src/application/v5_e2e_scope_rl/config/pct_teb_dwa_benchmark_development.yaml",
    "src/simulation/m2_gazebo/config/benchmark_move_base.yaml",
    "src/simulation/m2_gazebo/config/costmap_common.yaml",
    "src/simulation/m2_gazebo/config/local_costmap.yaml",
    "src/simulation/m2_gazebo/config/global_costmap.yaml",
    "src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml",
    "src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml",
    "src/navigation_arena/forks/navigation/local_planner/teb/teb_local_planner_plugin.xml",
)


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_once(path, content):
    if path.exists():
        raise FileExistsError("PILOT_RELEASE_RESOURCE_ALREADY_EXISTS:" + str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def schedule_rows():
    rows = []
    for block_index in range(20):
        seed = 196501 + block_index
        scene = SCENES[block_index % len(SCENES)]
        order = METHODS[block_index % 4 :] + METHODS[: block_index % 4]
        for order_index, method in enumerate(order):
            rows.append(
                {
                    "block_id": "pilot_block_{:03d}".format(block_index + 1),
                    "seed": seed,
                    "scene_id": scene,
                    "method": method,
                    "attempt": 1,
                    "order_index": order_index,
                }
            )
    return rows


def main():
    rows = schedule_rows()
    if len(rows) != 80 or len({(row["seed"], row["scene_id"], row["method"]) for row in rows}) != 80:
        raise RuntimeError("PILOT_SCHEDULE_IDENTITY_INVALID")
    PILOT.mkdir(parents=True, exist_ok=True)
    if SCHEDULE.exists():
        raise FileExistsError("PILOT_SCHEDULE_ALREADY_EXISTS")
    with SCHEDULE.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    prereg = {
        "schema_version": "1.0",
        "stage": "pilot",
        "status": "preregistered_before_outcome_access",
        "simulation_only": True,
        "purpose": "estimate paired variance, failure rates, system feasibility and formal sample size; no tuning or claims",
        "blocks": 20,
        "episodes": 80,
        "seed_range": [196501, 196520],
        "scene_allocation": {
            scene: sum(row["scene_id"] == scene for row in rows) // 4 for scene in SCENES
        },
        "methods": list(METHODS),
        "primary_metric": "failure_inclusive_restricted_time_s",
        "failure_accounting": {
            "success": "measured_gate_to_goal_time",
            "timeout_contact_nonarrival_method_system_failure": 22.0,
            "infrastructure_failure": "retained_and_reported_separately_never_deleted",
            "retry": "none_for_same_scientific_identity",
        },
        "safety_metrics": ["contact_count", "minimum_conservative_clearance_m", "mask_violation_count"],
        "pilot_decision_rule": {
            "tuning_from_pilot": "forbidden",
            "formal_sample_size_minimum_blocks": 100,
            "formal_sample_size_maximum_blocks": 200,
            "power_target": 0.80,
            "two_primary_comparisons": ["actual_teb", "dwa"],
            "formal_effect_threshold_relative_reduction": 0.08,
        },
        "schedule_path": str(SCHEDULE.relative_to(ROOT)),
        "schedule_sha256": sha256(SCHEDULE),
        "pilot_formal_independence": "pilot seeds excluded from all formal rounds",
        "hardware_authorized": False,
        "real_vehicle_authorized": False,
    }
    write_once(PREREG, yaml.safe_dump(prereg, sort_keys=False, allow_unicode=True))

    actor = ARTIFACT / "development/training/pct_benchmark_r1_seed9701/actor_final.pt"
    teb = ROOT / "src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml"
    dwa = ROOT / "src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml"
    resource_paths = [ROOT / value for value in LOCAL_RESOURCES]
    resource_paths.append(PREREG)
    missing = [str(path) for path in resource_paths if not path.is_file()]
    if missing:
        raise FileNotFoundError("PILOT_RESOURCE_ABSENT:" + repr(missing))
    external = (
        ROOT / "devel/lib/libteb_local_planner.so",
        Path("/opt/ros/noetic/lib/libdwa_local_planner.so"),
    )
    release = {
        "schema_version": "1.0",
        "stage": "pilot",
        "status": "unique_pilot_execution_release",
        "execution_authorized": True,
        "simulation_only": True,
        "real_vehicle_authorized": False,
        "training_authorized": False,
        "schedule_path": str(SCHEDULE.relative_to(ROOT)),
        "schedule_sha256": sha256(SCHEDULE),
        "preregistration_path": str(PREREG.relative_to(ROOT)),
        "preregistration_sha256": sha256(PREREG),
        "resource_bindings": [
            {"path": str(path.relative_to(ROOT)), "sha256": sha256(path)}
            for path in resource_paths
        ],
        "external_resource_bindings": [
            {"path": str(path), "sha256": sha256(path)} for path in external
        ],
        "method_bindings": {
            "pct_scope_rl": {"path": str(actor.relative_to(ROOT)), "sha256": sha256(actor)},
            "analytic_prior": {"path": None, "sha256": "none"},
            "actual_teb": {"path": str(teb.relative_to(ROOT)), "sha256": sha256(teb)},
            "dwa": {"path": str(dwa.relative_to(ROOT)), "sha256": sha256(dwa)},
        },
        "budget": {"authorized_blocks": 20, "authorized_episodes": 80, "attempts_per_identity": 1},
        "formal_seed_access_authorized": False,
        "stable_workspace_authorized": False,
    }
    write_once(RELEASE, yaml.safe_dump(release, sort_keys=False, allow_unicode=True))
    review = {
        "schema_version": "1.0",
        "status": "pilot_release_offline_review_pass",
        "schedule_rows": len(rows),
        "unique_identities": len({(row["seed"], row["scene_id"], row["method"], row["attempt"]) for row in rows}),
        "balanced_method_order_counts": {
            method: {
                str(index): sum(row["method"] == method and row["order_index"] == index for row in rows)
                for index in range(4)
            }
            for method in METHODS
        },
        "release_path": str(RELEASE.relative_to(ROOT)),
        "release_sha256": sha256(RELEASE),
        "preregistration_sha256": sha256(PREREG),
        "schedule_sha256": sha256(SCHEDULE),
        "formal_seed_accessed": False,
        "hardware_accessed": False,
    }
    write_once(REVIEW, json.dumps(review, indent=2, sort_keys=True) + "\n")
    print(json.dumps(review, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
