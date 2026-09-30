#!/usr/bin/env python3
"""Bind the exact V5-02 training code and passed component evidence."""

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
        "src/application/v5_e2e_scope_rl/config/v5_00_architecture_contract.yaml",
        "src/application/v5_e2e_scope_rl/config/v5_01_r1_probe_repair_contract.yaml",
        "src/application/v5_e2e_scope_rl/config/v5_02_r1_training_contract.yaml",
        "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/sensor_env.py",
        "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/asymmetric_ppo.py",
        "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/ppo_trainer.py",
        "src/application/v5_e2e_scope_rl/scripts/train_v5_ppo.py",
        "artifacts/v5/foundation/v5_01_sensor_backend_audit/component_audit.json",
        "artifacts/v5/foundation/v5_01_sensor_backend_audit/sha256_manifest.json",
        "artifacts/v5/foundation/v5_01_r1_sensor_backend_audit/component_audit_r1.json",
        "artifacts/v5/foundation/v5_01_r1_sensor_backend_audit/sha256_manifest.json",
    ]
    bindings = {}
    for relative in resources:
        path = workspace / relative
        if not path.is_file():
            raise FileNotFoundError(relative)
        bindings[relative] = sha256(path)
    r1_report = json.loads(
        (workspace / resources[-2]).read_text(encoding="utf-8")
    )
    if not r1_report.get("pass"):
        raise RuntimeError("V5_01_R1_ENTRY_GATE_NOT_PASSED")
    release = {
        "schema_version": "1.0",
        "stage": "V5-02-R1",
        "status": "bounded_training_release_pass",
        "release_authority": "fast_backend_training_and_validation_only",
        "resource_bindings": bindings,
        "formal_seed_access_authorized": False,
        "gazebo_authorized": False,
        "real_vehicle_authorized": False,
    }
    encoded = json.dumps(release, indent=2, sort_keys=True).encode("utf-8") + b"\n"
    release["preseal_sha256"] = hashlib.sha256(encoded).hexdigest()
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    if arguments.output.exists():
        raise FileExistsError("TRAINING_RELEASE_ALREADY_EXISTS")
    arguments.output.write_text(
        json.dumps(release, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps(release, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()

