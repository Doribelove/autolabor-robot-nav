#!/usr/bin/env python3
"""Create the independently released V3-20 failure-inclusive schedule."""
import argparse
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_20_a0_characterization_foundation"
CONTRACT = W / "config/thesis_experiments/v3/v3_20_a0_failure_inclusive_characterization_contract.yaml"
RELEASE = ROOT / "execution_release.yaml"
AUTH = ROOT / "simulation_authorization.yaml"
SCHEDULE = ROOT / "formal_exact_schedule.yaml"
MANIFEST = ROOT / "foundation_manifest.sha256"


def sha(data): return hashlib.sha256(data).hexdigest()
def ybytes(value): return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def atomic(path, data, check):
    if check:
        if not path.is_file() or path.read_bytes() != data: raise RuntimeError("{} differs".format(path))
        return
    if path.exists(): raise RuntimeError("{} exists".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream: stream.write(data); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def make_schedule(contract):
    spec = contract["formal_matrix"]
    methods = contract["frozen_methods"]
    rows = []
    sequence = 0
    for block_index, seed in enumerate(range(spec["seed_start"], spec["seed_end"] + 1)):
        stratum = contract["strata"][block_index % len(contract["strata"])]
        rotation = block_index % len(methods)
        order = methods[rotation:] + methods[:rotation]
        if (block_index // len(methods)) % 2: order = list(reversed(order))
        for method in order:
            sequence += 1
            rows.append({"sequence": sequence, "block_index": block_index + 1, "seed": seed, "scene_id": stratum["scene_id"], "actor_policy": stratum["actor_policy"], "method": method, "attempt": 1})
    if len(rows) != 120: raise RuntimeError("formal schedule length differs")
    return {"schema_version": "1.0", "stage": "V3-20-A0", "phase": "formal", "row_count": len(rows), "rows": rows}


def outputs():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    schedule_bytes = ybytes(make_schedule(contract))
    bindings = [
        "config/thesis_experiments/v3/v3_20_a0_failure_inclusive_characterization_contract.yaml",
        "config/thesis_experiments/v3/v3_19_a0_four_method_benchmark_contract.yaml",
        "artifacts/v3/thesis/v3_19_a0_benchmark_foundation/execution_release.yaml",
        "artifacts/v3/thesis/v3_19_a0_benchmark_foundation/terminal_freeze.sha256",
        "artifacts/v3/thesis/v3_19_a0_benchmark_foundation/v3_19_no_candidate_conditioning_model.yaml",
        "artifacts/v3/thesis/v3_19_q1_execution/qualification_assessment.yaml",
        "artifacts/v3/thesis/v3_19_q1_execution/independent_qualification_review.yaml",
        "artifacts/v3/thesis/v3_19_q1_execution/terminal_freeze.sha256",
        "artifacts/v3/planner/v3_08_a0/v3_08_a0_effective_planner_contract.yaml",
        "artifacts/v3/prediction/v3_07_b0_r1/model/v3_07_b0_r1_spatial_candidate_predictor.yaml",
        "artifacts/v3/planner/v3_03_c5/v3_03_c5_effective_planner_contract.yaml",
        "artifacts/v3/prediction/v3_02_d1/model/v3_02_d1_candidate_predictor.yaml",
        "src/application/v3_scope_planner/src/v3_scope_planner/thesis_benchmark_methods.py",
        "src/application/v3_scope_planner/scripts/v3_scope_thesis_benchmark_episode.py",
        "src/application/v3_scope_planner/scripts/v3_scope_actual_teb_fair_episode.py",
        "src/simulation/m2_gazebo/launch/m2_v3_19_thesis_benchmark_episode.launch",
        "src/simulation/m2_gazebo/launch/spawn_m2.launch",
        "src/simulation/m2_gazebo/config/fixed_teb.yaml",
        "src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp",
        "src/tools/thesis_experiment/scripts/run_v3_19_four_method_benchmark.py",
        "src/tools/thesis_experiment/scripts/run_v3_20_failure_inclusive_characterization.py",
    ]
    runtime = [{"path": path, "sha256": sha((W / path).read_bytes())} for path in bindings]
    release_value = {
        "schema_version": "1.0", "stage": "V3-20-A0",
        "status": "v3_20_a0_failure_inclusive_release_reviewed",
        "contract_sha256": sha(CONTRACT.read_bytes()), "formal_schedule_sha256": sha(schedule_bytes),
        "runtime_bindings": runtime, "formal_units": 120,
        "minimum_task_success_execution_gate": False,
        "failure_value_s": 22.0,
        "retry_resume_replacement_forbidden": True,
        "training_steps": 0, "hardware_access_authorized": False, "real_vehicle_authorized": False,
    }
    auth_value = {
        "schema_version": "1.0", "stage": "V3-20-A0",
        "basis": "explicit user simulation instruction 2026-08-06; V3-19 Q1 performance failure retained",
        "execution_authorized": True, "authorized_formal_units": 120,
        "retry_resume_replacement_authorized": False,
        "training_authorized": False, "hardware_access_authorized": False, "real_vehicle_authorized": False,
    }
    generated = {SCHEDULE: schedule_bytes, RELEASE: ybytes(release_value), AUTH: ybytes(auth_value)}
    resources = [CONTRACT, Path(__file__).resolve(), W / "src/tools/thesis_experiment/scripts/review_v3_20_a0_characterization_foundation.py"] + [W / path for path in bindings]
    manifest = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in sorted(set(resources)))
    manifest += "".join("{}  {}\n".format(sha(data), path.relative_to(W)) for path, data in generated.items())
    generated[MANIFEST] = manifest.encode("utf-8")
    return generated


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    generated = outputs()
    for path, data in generated.items(): atomic(path, data, args.check)
    print("status=v3_20_a0_foundation_pass release_sha256={} manifest_sha256={}".format(sha(generated[RELEASE]), sha(generated[MANIFEST])))


if __name__ == "__main__": main()

