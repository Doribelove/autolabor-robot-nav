#!/usr/bin/env python3
"""Independent review for the V3-18-A0 thesis evidence audit."""
import argparse
import csv
import hashlib
import os
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_18_a0_project_audit"
OUT = ROOT / "independent_review.yaml"
FREEZE = ROOT / "terminal_freeze.sha256"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def atomic(path, data, check):
    if check:
        if not path.is_file() or path.read_bytes() != data:
            raise RuntimeError("{} differs".format(path))
        return
    if path.exists():
        raise RuntimeError("{} exists".format(path))
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("wb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    manifest_path = ROOT / "evidence_manifest.sha256"
    architecture_path = ROOT / "system_architecture.yaml"
    inventory_path = ROOT / "experiment_inventory.csv"
    matrix_path = ROOT / "method_evidence_matrix.csv"
    schema_path = ROOT / "standard_collection_schema.yaml"
    gaps_path = ROOT / "gap_analysis.yaml"
    entries = [line.split("  ", 1) for line in manifest_path.read_text().splitlines() if line]
    architecture = yaml.safe_load(architecture_path.read_bytes())
    schema = yaml.safe_load(schema_path.read_bytes())
    gaps = yaml.safe_load(gaps_path.read_bytes())
    with inventory_path.open(newline="") as stream:
        inventory = list(csv.DictReader(stream))
    with matrix_path.open(newline="") as stream:
        matrix = list(csv.DictReader(stream))
    stages = {row["stage"] for row in inventory}
    methods = {row["method"] for row in matrix}
    checks = {
        "manifest_rehash": all(sha((W / path).read_bytes()) == digest for digest, path in entries),
        "architecture_layers_present": len(architecture["v3_runtime_dataflow"]) >= 8 and len(architecture["experiment_integrity_layers"]) >= 8,
        "workspace_isolation_explicit": architecture["workspace_boundaries"]["stable_workspace_modified"] is False,
        "terminal_studies_inventory_complete": {"T11/T12", "V2-04G-P1", "V3-05-B0", "V3-09-F1", "V3-11-F1", "V3-14-S1", "V3-16-S1"}.issubset(stages),
        "method_roles_cover_thesis_need": {"v3_spatial_safe", "actual_teb_0_8_4", "fixed_v120_shielded", "response_gate_12_55"}.issubset(methods),
        "schema_has_provenance_and_safety": "provenance" in schema["field_groups"] and "safety" in schema["field_groups"],
        "cross_stage_pooling_closed": gaps["cross_stage_pooling_forbidden"] is True,
        "new_benchmark_is_fresh_and_qualification_gated": gaps["next_preregistered_study"]["fresh_seeds_required"] and gaps["next_preregistered_study"]["qualification_required"],
        "no_execution_authority": gaps["execution_authorized"] is False,
    }
    value = {
        "schema_version": "1.0",
        "stage": "V3-18-A0",
        "status": "v3_18_a0_independent_project_audit_review_pass" if all(checks.values()) else "v3_18_a0_independent_project_audit_review_fail",
        "checks": checks,
        "inventory_rows": len(inventory),
        "method_matrix_rows": len(matrix),
        "manifest_sha256": sha(manifest_path.read_bytes()),
        "execution_authorized": False,
        "training_authorized": False,
        "hardware_accessed": False,
        "real_vehicle_used": False,
    }
    body = yaml.safe_dump(value, sort_keys=False).encode("utf-8")
    atomic(OUT, body, args.check)
    freeze_paths = (manifest_path, architecture_path, inventory_path, matrix_path, schema_path, gaps_path, OUT)
    freeze = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in freeze_paths).encode("utf-8")
    atomic(FREEZE, freeze, args.check)
    print("status={} review_sha256={} freeze_sha256={}".format(value["status"], sha(body), sha(freeze)))


if __name__ == "__main__":
    main()

