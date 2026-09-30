#!/usr/bin/env python3
"""Create the immutable 160-block formal-round-1 preregistration and release."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
ARTIFACT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark"
FORMAL = ARTIFACT / "formal/round_1"
SCHEDULE = FORMAL / "formal_r1_schedule.csv"
PREREG = FORMAL / "formal_r1_preregistration.yaml"
RELEASE = FORMAL / "formal_r1_execution_release.yaml"
REVIEW = FORMAL / "formal_r1_release_review.json"
PILOT_ASSESSMENT = ARTIFACT / "pilot/pilot_r1/pilot_assessment.json"
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
        raise FileExistsError("FORMAL_R1_RESOURCE_ALREADY_EXISTS:" + str(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def rows():
    result = []
    for block_index in range(160):
        seed = 197001 + block_index
        scene = SCENES[block_index % len(SCENES)]
        order = METHODS[block_index % 4 :] + METHODS[: block_index % 4]
        for order_index, method in enumerate(order):
            result.append(
                {
                    "block_id": "formal_r1_block_{:03d}".format(block_index + 1),
                    "seed": seed,
                    "scene_id": scene,
                    "method": method,
                    "attempt": 1,
                    "order_index": order_index,
                }
            )
    return result


def main():
    schedule_rows = rows()
    if len(schedule_rows) != 640 or len({(row["seed"], row["scene_id"], row["method"], row["attempt"]) for row in schedule_rows}) != 640:
        raise RuntimeError("FORMAL_R1_SCHEDULE_INVALID")
    FORMAL.mkdir(parents=True, exist_ok=True)
    if SCHEDULE.exists():
        raise FileExistsError("FORMAL_R1_SCHEDULE_ALREADY_EXISTS")
    with SCHEDULE.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(schedule_rows[0]))
        writer.writeheader()
        writer.writerows(schedule_rows)

    prereg = {
        "schema_version": "1.0",
        "stage": "formal_round_1",
        "status": "preregistered_before_formal_outcome_access",
        "simulation_only": True,
        "blocks": 160,
        "episodes": 640,
        "seed_range": [197001, 197160],
        "scene_allocation": {
            scene: sum(row["scene_id"] == scene for row in schedule_rows) // 4
            for scene in SCENES
        },
        "methods": list(METHODS),
        "primary_unit": "paired_seed_scene_block",
        "primary_metric": "failure_inclusive_restricted_time_s",
        "primary_comparisons": ["pct_scope_rl_vs_actual_teb", "pct_scope_rl_vs_dwa"],
        "primary_estimator": "(mean_baseline_minus_mean_pct)/mean_baseline",
        "multiplicity": {
            "method": "two_sided_97_5_percent_paired_block_bootstrap_for_each_of_two_locked_primary_comparisons",
            "bootstrap_samples": 200000,
            "bootstrap_seeds": {"actual_teb": 20261971, "dwa": 20261972},
            "confidence_quantiles": [0.0125, 0.9875],
        },
        "positive_gate_per_comparator": {
            "minimum_relative_time_reduction": 0.08,
            "bootstrap_lower_bound_strictly_greater_than": 0.0,
            "pct_completion_rate_difference_percentage_points_minimum": -5.0,
            "pct_contact_count_not_higher_than_comparator": True,
            "pct_mask_violation_count_maximum": 0,
            "finite_statistics_required": True,
        },
        "program_success_rule": "all integrity gates plus positive_gate for at least one locked traditional comparator",
        "failure_accounting": {
            "success": "measured_gate_to_goal_time",
            "timeout_contact_nonarrival_method_system_failure": 22.0,
            "infrastructure_failure": "retained_and_reported_separately_with_22_second_primary_penalty",
            "retry": "none_for_same_scientific_identity",
            "replacement_seed": "forbidden",
        },
        "secondary_analyses": [
            "analytic_prior_comparison",
            "scene_strata",
            "completion_contact_clearance",
            "candidate_switch_stop_recovery_tracking_latency",
            "failure_case_inventory",
        ],
        "secondary_claim_status": "descriptive_unless_separately_gated",
        "sample_size_basis": {
            "pilot_assessment_path": str(PILOT_ASSESSMENT.relative_to(ROOT)),
            "pilot_assessment_sha256": sha256(PILOT_ASSESSMENT),
            "selected_blocks": 160,
            "dwa_target_power": 0.8204883971202288,
            "teb_target_power": 0.5845471832926094,
            "teb_underpowered_within_200_block_cap": True,
        },
        "schedule_path": str(SCHEDULE.relative_to(ROOT)),
        "schedule_sha256": sha256(SCHEDULE),
        "pilot_seeds_excluded": True,
        "development_seeds_excluded": True,
        "formal_round_2_seeds_not_accessed": True,
        "tuning_from_formal_outcomes": "forbidden",
        "hardware_authorized": False,
        "real_vehicle_authorized": False,
    }
    write_once(PREREG, yaml.safe_dump(prereg, sort_keys=False, allow_unicode=True))

    actor = ARTIFACT / "development/training/pct_benchmark_r1_seed9701/actor_final.pt"
    teb = ROOT / "src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml"
    dwa = ROOT / "src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml"
    local = [ROOT / value for value in LOCAL_RESOURCES] + [PREREG]
    external = (
        ROOT / "devel/lib/libteb_local_planner.so",
        Path("/opt/ros/noetic/lib/libdwa_local_planner.so"),
    )
    missing = [str(path) for path in (*local, *external, actor) if not path.is_file()]
    if missing:
        raise FileNotFoundError("FORMAL_R1_RESOURCE_ABSENT:" + repr(missing))
    release = {
        "schema_version": "1.0",
        "stage": "formal_round_1",
        "status": "unique_formal_round_1_execution_release",
        "execution_authorized": True,
        "simulation_only": True,
        "real_vehicle_authorized": False,
        "training_authorized": False,
        "schedule_path": str(SCHEDULE.relative_to(ROOT)),
        "schedule_sha256": sha256(SCHEDULE),
        "preregistration_path": str(PREREG.relative_to(ROOT)),
        "preregistration_sha256": sha256(PREREG),
        "resource_bindings": [
            {"path": str(path.relative_to(ROOT)), "sha256": sha256(path)} for path in local
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
        "budget": {"authorized_blocks": 160, "authorized_episodes": 640, "attempts_per_identity": 1},
        "pilot_seed_access_authorized": False,
        "formal_round_2_seed_access_authorized": False,
        "stable_workspace_authorized": False,
    }
    write_once(RELEASE, yaml.safe_dump(release, sort_keys=False, allow_unicode=True))
    review = {
        "schema_version": "1.0",
        "status": "formal_r1_release_offline_review_pass",
        "schedule_rows": len(schedule_rows),
        "unique_identities": len({(row["seed"], row["scene_id"], row["method"], row["attempt"]) for row in schedule_rows}),
        "blocks": 160,
        "scene_counts": prereg["scene_allocation"],
        "balanced_method_order_counts": {
            method: {
                str(index): sum(row["method"] == method and row["order_index"] == index for row in schedule_rows)
                for index in range(4)
            }
            for method in METHODS
        },
        "release_path": str(RELEASE.relative_to(ROOT)),
        "release_sha256": sha256(RELEASE),
        "preregistration_sha256": sha256(PREREG),
        "schedule_sha256": sha256(SCHEDULE),
        "formal_outcome_accessed": False,
        "hardware_accessed": False,
    }
    write_once(REVIEW, json.dumps(review, indent=2, sort_keys=True) + "\n")
    print(json.dumps(review, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
