#!/usr/bin/env python3
"""Materialize and freeze the V3-19 four-method benchmark foundation."""
import argparse
import copy
import hashlib
import importlib.util
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_19_a0_benchmark_foundation"
CONTRACT = W / "config/thesis_experiments/v3/v3_19_a0_four_method_benchmark_contract.yaml"
MODEL = W / "artifacts/v3/prediction/v3_07_b0_r1/model/v3_07_b0_r1_spatial_candidate_predictor.yaml"
PACKAGE = W / "src/application/v3_scope_planner/src"
METHOD_MODULE = PACKAGE / "v3_scope_planner/thesis_benchmark_methods.py"
OUT_MODEL = ROOT / "v3_19_no_candidate_conditioning_model.yaml"
RELEASE = ROOT / "execution_release.yaml"
AUTH = ROOT / "simulation_authorization.yaml"
MANIFEST = ROOT / "foundation_manifest.sha256"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def ybytes(value):
    return yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")


def atomic(path, data, check):
    if check:
        if not path.is_file() or path.read_bytes() != data:
            raise RuntimeError("{} differs".format(path))
        return
    if path.exists():
        raise RuntimeError("{} exists".format(path))
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(data); stream.flush(); os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def load_methods():
    spec = importlib.util.spec_from_file_location("v3_19_foundation_methods", METHOD_MODULE)
    module = importlib.util.module_from_spec(spec)
    module.__package__ = "v3_scope_planner"
    import sys
    if str(PACKAGE) not in sys.path:
        sys.path.insert(0, str(PACKAGE))
    spec.loader.exec_module(module)
    return module


def schedule(contract, phase):
    spec = contract["evaluation"][phase]
    methods = contract["methods"]["identifiers"]
    rows = []
    sequence = 0
    for block_index, seed in enumerate(range(spec["seed_start"], spec["seed_end"] + 1)):
        stratum = contract["strata"][block_index % len(contract["strata"])]
        rotation = block_index % len(methods)
        order = methods[rotation:] + methods[:rotation]
        if (block_index // len(methods)) % 2:
            order = list(reversed(order))
        for method in order:
            sequence += 1
            rows.append({"sequence": sequence, "block_index": block_index + 1, "seed": seed, "scene_id": stratum["scene_id"], "actor_policy": stratum["actor_policy"], "method": method, "attempt": 1})
    if len(rows) != spec["episodes"]:
        raise RuntimeError("{} schedule length differs".format(phase))
    return {"schema_version": "1.0", "stage": "V3-19-A0", "phase": phase, "row_count": len(rows), "rows": rows}


def outputs():
    contract = yaml.safe_load(CONTRACT.read_bytes())
    base_model = yaml.safe_load(MODEL.read_bytes())
    derived = load_methods().derive_no_candidate_conditioning_model(base_model)
    model_bytes = ybytes(derived)
    schedules = {phase: ybytes(schedule(contract, phase)) for phase in ("qualification", "formal")}
    bindings = [
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
    ]
    runtime = [{"path": path, "sha256": sha((W / path).read_bytes())} for path in bindings]
    runtime.append({"path": str(OUT_MODEL.relative_to(W)), "sha256": sha(model_bytes)})
    release_value = {
        "schema_version": "1.0", "stage": "V3-19-A0",
        "status": "v3_19_a0_execution_release_reviewed",
        "contract_sha256": sha(CONTRACT.read_bytes()),
        "schedule_sha256": {phase: sha(data) for phase, data in schedules.items()},
        "runtime_bindings": runtime,
        "qualification_required_before_formal": True,
        "retry_resume_replacement_forbidden": True,
        "training_steps": 0, "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    authorization_value = {
        "schema_version": "1.0", "stage": "V3-19-A0",
        "basis": "explicit user instruction on 2026-08-06 to continue simulation experiments and thesis export",
        "execution_authorized": True,
        "scope": "qualification then, only on pass, the exact fresh formal simulation schedule",
        "authorized_qualification_units": contract["evaluation"]["qualification"]["episodes"],
        "authorized_formal_units": contract["evaluation"]["formal"]["episodes"],
        "retry_resume_replacement_authorized": False,
        "training_authorized": False, "hardware_access_authorized": False,
        "real_vehicle_authorized": False,
    }
    generated = {
        OUT_MODEL: model_bytes,
        ROOT / "qualification_exact_schedule.yaml": schedules["qualification"],
        ROOT / "formal_exact_schedule.yaml": schedules["formal"],
        RELEASE: ybytes(release_value), AUTH: ybytes(authorization_value),
    }
    resources = [CONTRACT, MODEL, Path(__file__).resolve(), W / "src/tools/thesis_experiment/scripts/review_v3_19_a0_benchmark_foundation.py"] + [W / item for item in bindings]
    manifest = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in sorted(set(resources)))
    manifest += "".join("{}  {}\n".format(sha(data), path.relative_to(W)) for path, data in generated.items())
    generated[MANIFEST] = manifest.encode("utf-8")
    return generated


def main():
    parser = argparse.ArgumentParser(); parser.add_argument("--check", action="store_true"); args = parser.parse_args()
    generated = outputs()
    for path, data in generated.items(): atomic(path, data, args.check)
    print("status=v3_19_a0_benchmark_foundation_pass release_sha256={} manifest_sha256={}".format(sha(generated[RELEASE]), sha(generated[MANIFEST])))


if __name__ == "__main__": main()

