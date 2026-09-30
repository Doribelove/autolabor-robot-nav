#!/usr/bin/env python3
"""Independent deterministic review of the V3-18-B0 canonical dataset."""
import argparse
import csv
import hashlib
import importlib.util
import json
import os
from collections import Counter
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_18_b0_canonical_dataset"
SCHEMA_SOURCE = W / "src/tools/thesis_experiment/src/thesis_experiment/thesis_dataset.py"
OUT = ROOT / "independent_review.yaml"
FREEZE = ROOT / "terminal_freeze.sha256"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def schema_module():
    spec = importlib.util.spec_from_file_location("v3_18_b0_review_schema", SCHEMA_SOURCE)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


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
    manifest_path = ROOT / "source_and_output_manifest.sha256"
    jsonl_path = ROOT / "canonical_episodes.jsonl"
    csv_path = ROOT / "canonical_episodes.csv"
    summary_path = ROOT / "dataset_summary.yaml"
    entries = [line.split("  ", 1) for line in manifest_path.read_text().splitlines() if line]
    rows = [json.loads(line) for line in jsonl_path.read_text().splitlines() if line]
    schema = schema_module()
    validation_errors = []
    for index, row in enumerate(rows):
        try:
            schema.validate_row(row)
            expected = schema.digest({key: value for key, value in row.items() if key not in ("source_path", "row_digest")})
            if row["row_digest"] != expected:
                raise ValueError("row digest differs")
        except Exception as error:
            validation_errors.append("{}:{}".format(index + 1, error))
    with csv_path.open(newline="") as stream:
        csv_rows = list(csv.DictReader(stream))
    counts = Counter(row["stage"] for row in rows)
    identities = [(row["stage"], row["sequence"]) for row in rows]
    checks = {
        "all_manifest_entries_rehash": all((W / path).is_file() and sha((W / path).read_bytes()) == digest for digest, path in entries),
        "canonical_rows_validate": not validation_errors,
        "row_count_exact_825": len(rows) == 825,
        "stage_counts_exact": counts == Counter({"V2-04G-P1-E1": 270, "V3-05-B0": 270, "V3-09-F1": 180, "V3-11-F1": 60, "V3-16-S1": 45}),
        "identity_unique_within_stage": len(identities) == len(set(identities)),
        "csv_jsonl_row_count_match": len(csv_rows) == len(rows),
        "main_baseline_comparator_ablation_roles_present": {"main_method", "primary_baseline", "comparison_method", "forced_action_mechanism_ablation"}.issubset({row["method_role"] for row in rows}),
        "no_hardware_or_real_vehicle": all(not row["hardware_accessed"] and not row["real_vehicle_used"] for row in rows),
        "cross_stage_pooling_closed": yaml.safe_load(summary_path.read_bytes())["cross_stage_pooling_forbidden"] is True,
    }
    value = {
        "schema_version": "1.0", "stage": "V3-18-B0",
        "status": "v3_18_b0_canonical_dataset_independent_review_pass" if all(checks.values()) else "v3_18_b0_canonical_dataset_independent_review_fail",
        "checks": checks, "validation_errors": validation_errors,
        "row_count": len(rows), "stage_counts": dict(sorted(counts.items())),
        "dataset_sha256": sha(jsonl_path.read_bytes()), "manifest_sha256": sha(manifest_path.read_bytes()),
        "execution_authorized": False, "training_authorized": False,
        "hardware_accessed": False, "real_vehicle_used": False,
    }
    body = yaml.safe_dump(value, sort_keys=False).encode("utf-8")
    atomic(OUT, body, args.check)
    paths = [manifest_path, jsonl_path, csv_path, ROOT / "data_dictionary.yaml", summary_path, OUT]
    freeze = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in paths).encode("utf-8")
    atomic(FREEZE, freeze, args.check)
    print("status={} dataset_sha256={} review_sha256={}".format(value["status"], value["dataset_sha256"], sha(body)))


if __name__ == "__main__":
    main()

