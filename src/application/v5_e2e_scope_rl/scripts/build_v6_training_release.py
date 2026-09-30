#!/usr/bin/env python3
"""Bind V6 PCT code, component evidence and four exact V5 initializations."""

import argparse
import hashlib
import json
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    arguments = parser.parse_args()
    workspace = arguments.workspace.resolve()
    resources = [
        "src/application/v5_e2e_scope_rl/config/v6_00_pct_scope_rl_contract.yaml",
        "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/privileged_distillation.py",
        "src/application/v5_e2e_scope_rl/scripts/train_v6_pct.py",
        "artifacts/v5/formal/v5_03_f1_fast_formal/formal_assessment.json",
        "artifacts/v5/formal/v5_03_f1_fast_formal/sha256_manifest.json",
        "artifacts/v5/diagnosis/v5_04_d1_failure_attribution/failure_attribution.json",
        "artifacts/v5/diagnosis/v5_04_d1_failure_attribution/sha256_manifest.json",
        "artifacts/v6/foundation/v6_00_pct_component_audit/component_audit.json",
        "artifacts/v6/foundation/v6_00_pct_component_audit/sha256_manifest.json",
        "artifacts/v5/training/v5_02_r1_validation/selection_report.json",
    ]
    bindings = {}
    for relative in resources:
        path = workspace / relative
        if not path.is_file():
            raise FileNotFoundError(relative)
        bindings[relative] = sha256(path)
    component = json.loads((workspace / resources[-3]).read_text(encoding="utf-8"))
    if not component.get("pass"):
        raise RuntimeError("V6_COMPONENT_GATE_NOT_PASSED")
    selection = json.loads((workspace / resources[-1]).read_text(encoding="utf-8"))
    initializations = []
    new_seeds = [9501, 9502, 9503, 9504]
    asymmetric = [
        row for row in selection["selected_checkpoints"] if row["variant"] == "asymmetric"
    ]
    if len(asymmetric) != 4:
        raise RuntimeError("V5_ASYMMETRIC_INITIALIZATION_ROSTER_MISMATCH")
    for new_seed, row in zip(new_seeds, asymmetric):
        actor_path = workspace / row["checkpoint_path"]
        trainer_path = actor_path.with_name(actor_path.name.replace("actor_", "trainer_"))
        if sha256(actor_path) != row["checkpoint_sha256"]:
            raise RuntimeError("V5_ACTOR_HASH_MISMATCH")
        initialization = {
            "v6_identity": "pct_seed{}".format(new_seed),
            "training_seed": new_seed,
            "steps": 120000,
            "v5_identity": row["identity"],
            "actor_path": str(actor_path.relative_to(workspace)),
            "actor_sha256": sha256(actor_path),
            "trainer_path": str(trainer_path.relative_to(workspace)),
            "trainer_sha256": sha256(trainer_path),
        }
        initializations.append(initialization)
    release = {
        "schema_version": "1.0",
        "stage": "V6-01-R1",
        "status": "pct_bounded_training_release_pass",
        "resource_bindings": bindings,
        "initializations": initializations,
        "total_environment_steps": 480000,
        "formal_seed_access_authorized": False,
        "gazebo_authorized": False,
        "real_vehicle_authorized": False,
    }
    encoded = json.dumps(release, indent=2, sort_keys=True).encode("utf-8") + b"\n"
    release["preseal_sha256"] = hashlib.sha256(encoded).hexdigest()
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    if arguments.output.exists():
        raise FileExistsError("V6_TRAINING_RELEASE_ALREADY_EXISTS")
    arguments.output.write_text(
        json.dumps(release, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(release, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

