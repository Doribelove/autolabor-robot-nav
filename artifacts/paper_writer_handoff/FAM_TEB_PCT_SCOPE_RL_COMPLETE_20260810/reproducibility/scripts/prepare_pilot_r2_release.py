#!/usr/bin/env python3
"""Create the immutable R2 pilot schedule, preregistration and release."""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
PILOT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark/pilot/pilot_r2"
SCHEDULE = PILOT / "pilot_r2_schedule.csv"; PREREG = PILOT / "pilot_r2_preregistration.yaml"
RELEASE = PILOT / "pilot_r2_execution_release.yaml"; REVIEW = PILOT / "pilot_r2_release_review.json"
SCENES = ("core_static_avoidance", "core_single_crossing", "core_head_on_yield",
          "core_sequential_interaction", "core_partially_occluded", "core_width_change")
METHODS = ("pct_scope_rl", "analytic_prior", "actual_teb", "dwa")
RESOURCES = (
    "scripts/gazebo_pct_teb_dwa_benchmark/run_frozen_matrix.py",
    "scripts/gazebo_pct_teb_dwa_benchmark/run_frozen_matrix_r2.py",
    "scripts/gazebo_pct_teb_dwa_benchmark/run_development_matrix.py",
    "scripts/gazebo_pct_teb_dwa_benchmark/assess_pilot_r2.py",
    "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py",
    "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_r2_episode.py",
    "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py",
    "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/asymmetric_ppo.py",
    "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/runtime_adapter.py",
    "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/sensor_env.py",
    "src/application/v3_scope_planner/src/v3_scope_planner/execution_core.py",
    "src/simulation/m2_gazebo/launch/m2_pct_teb_dwa_benchmark_r2_episode.launch",
    "src/simulation/m2_gazebo/launch/spawn_m2.launch",
    "src/simulation/m2_gazebo/urdf/m2.urdf.xacro",
    "src/application/v5_e2e_scope_rl/config/pct_teb_dwa_benchmark_r2_development.yaml",
    "src/simulation/m2_gazebo/config/benchmark_move_base.yaml",
    "src/simulation/m2_gazebo/config/costmap_common.yaml",
    "src/simulation/m2_gazebo/config/local_costmap.yaml",
    "src/simulation/m2_gazebo/config/global_costmap.yaml",
    "src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml",
    "src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml",
    "src/navigation_arena/forks/navigation/local_planner/teb/teb_local_planner_plugin.xml",
)


def sha256(path): return hashlib.sha256(path.read_bytes()).hexdigest()
def write_once(path, text):
    if path.exists(): raise FileExistsError("R2_PILOT_RESOURCE_ALREADY_EXISTS:" + str(path))
    path.parent.mkdir(parents=True, exist_ok=True); path.write_text(text, encoding="utf-8")


def main():
    rows=[]
    for index in range(20):
        order=METHODS[index % 4:]+METHODS[:index % 4]
        for position, method in enumerate(order):
            rows.append({"block_id":"pilot_r2_block_{:03d}".format(index+1), "seed":196601+index,
                         "scene_id":SCENES[index % 6], "method":method, "attempt":1, "order_index":position})
    PILOT.mkdir(parents=True, exist_ok=True)
    with SCHEDULE.open("x", newline="", encoding="utf-8") as stream:
        writer=csv.DictWriter(stream, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    prereg={"schema_version":"1.0", "stage":"pilot", "research_round":"R2", "status":"preregistered_before_outcome_access",
      "simulation_only":True, "purpose":"variance and power only; tuning forbidden", "blocks":20, "episodes":80,
      "seed_range":[196601,196620], "methods":list(METHODS), "candidate_variant":"r2_candidate_v060",
      "single_change_from_formal_r1":"actions_4_5_target_speed_0.4_to_0.6_mps",
      "primary_metric":"failure_inclusive_restricted_time_s", "episode_cap_s":22.0,
      "failure_accounting":"all timeout, contact, nonarrival, method-system failure score 22 s; no retry or replacement",
      "comparisons":["actual_teb","dwa"], "effect_threshold":0.08, "interval":"two-sided 97.5% paired bootstrap",
      "bootstrap_samples":100000, "bootstrap_seeds":{"actual_teb":20261981,"dwa":20261982},
      "power_target":0.80, "formal_block_selection":"ceil max requirement to multiple of 20; floor 160; cap 200",
      "pilot_may_inform_tuning":False, "formal_round_2_seed_range":[198001,198200],
      "schedule_path":str(SCHEDULE.relative_to(ROOT)), "schedule_sha256":sha256(SCHEDULE),
      "hardware_authorized":False, "real_vehicle_authorized":False}
    write_once(PREREG, yaml.safe_dump(prereg, sort_keys=False, allow_unicode=True))
    paths=[ROOT/x for x in RESOURCES]+[PREREG]
    actor=ROOT/"artifacts/gazebo_pct_teb_dwa_benchmark/development/training/pct_benchmark_r1_seed9701/actor_final.pt"
    teb=ROOT/"src/simulation/m2_gazebo/config/benchmark_teb_v0.yaml"; dwa=ROOT/"src/simulation/m2_gazebo/config/benchmark_dwa_v0.yaml"
    external=[ROOT/"devel/lib/libteb_local_planner.so",Path("/opt/ros/noetic/lib/libdwa_local_planner.so")]
    release={"schema_version":"1.0", "stage":"pilot", "research_round":"R2", "status":"unique_pilot_r2_execution_release",
      "execution_authorized":True, "simulation_only":True, "real_vehicle_authorized":False, "training_authorized":False,
      "schedule_path":str(SCHEDULE.relative_to(ROOT)), "schedule_sha256":sha256(SCHEDULE),
      "preregistration_path":str(PREREG.relative_to(ROOT)), "preregistration_sha256":sha256(PREREG),
      "resource_bindings":[{"path":str(p.relative_to(ROOT)),"sha256":sha256(p)} for p in paths],
      "external_resource_bindings":[{"path":str(p),"sha256":sha256(p)} for p in external],
      "method_bindings":{"pct_scope_rl":{"path":str(actor.relative_to(ROOT)),"sha256":sha256(actor)},
        "analytic_prior":{"path":None,"sha256":"none"}, "actual_teb":{"path":str(teb.relative_to(ROOT)),"sha256":sha256(teb)},
        "dwa":{"path":str(dwa.relative_to(ROOT)),"sha256":sha256(dwa)}},
      "candidate_contract_variant":"r2_candidate_v060", "budget":{"authorized_blocks":20,"authorized_episodes":80,"attempts_per_identity":1},
      "formal_seed_access_authorized":False, "stable_workspace_authorized":False}
    write_once(RELEASE, yaml.safe_dump(release, sort_keys=False, allow_unicode=True))
    review={"schema_version":"1.0","status":"pilot_r2_release_offline_review_pass","schedule_rows":80,
      "unique_identities":len({(r['seed'],r['scene_id'],r['method'],r['attempt']) for r in rows}),
      "balanced_method_order_counts":{m:{str(i):sum(r['method']==m and r['order_index']==i for r in rows) for i in range(4)} for m in METHODS},
      "release_path":str(RELEASE.relative_to(ROOT)),"release_sha256":sha256(RELEASE),
      "preregistration_sha256":sha256(PREREG),"schedule_sha256":sha256(SCHEDULE),"formal_seed_accessed":False}
    write_once(REVIEW,json.dumps(review,indent=2,sort_keys=True)+"\n"); print(json.dumps(review,indent=2,sort_keys=True))


if __name__ == "__main__": main()
