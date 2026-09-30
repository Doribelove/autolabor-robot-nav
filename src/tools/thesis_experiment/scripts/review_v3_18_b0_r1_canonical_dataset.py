#!/usr/bin/env python3
"""Independent R1 review preserving B0 while validating identical dataset bytes."""
import argparse
import hashlib
import importlib.util
import json
import os
from collections import Counter
from pathlib import Path

import yaml


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_18_b0_canonical_dataset"
MANIFEST = ROOT / "source_and_output_manifest_r1.sha256"
JSONL = ROOT / "canonical_episodes.jsonl"
OLD_REVIEW = ROOT / "independent_review.yaml"
OUT = ROOT / "independent_review_r1.yaml"
FREEZE = ROOT / "terminal_freeze_r1.sha256"
SCHEMA_SOURCE = W / "src/tools/thesis_experiment/src/thesis_experiment/thesis_dataset.py"


def sha(data):
    return hashlib.sha256(data).hexdigest()


def schema_module():
    spec = importlib.util.spec_from_file_location("v3_18_b0_r1_schema", SCHEMA_SOURCE)
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
    entries = [line.split("  ", 1) for line in MANIFEST.read_text().splitlines() if line]
    rows = [json.loads(line) for line in JSONL.read_text().splitlines() if line]
    schema = schema_module()
    validation = []
    for index, row in enumerate(rows):
        try:
            schema.validate_row(row)
            expected = schema.digest({key: value for key, value in row.items() if key not in ("source_path", "row_digest")})
            if expected != row["row_digest"]:
                raise ValueError("row digest differs")
        except Exception as error:
            validation.append("{}:{}".format(index + 1, error))
    counts = Counter(row["stage"] for row in rows)
    old_review = yaml.safe_load(OLD_REVIEW.read_bytes())
    checks = {
        "r1_manifest_rehash": all((W / path).is_file() and sha((W / path).read_bytes()) == digest for digest, path in entries),
        "old_failure_preserved_and_attributed": old_review["status"].endswith("_fail") and old_review["checks"]["all_manifest_entries_rehash"] is False,
        "canonical_rows_unchanged_and_valid": not validation and sha(JSONL.read_bytes()) == old_review["dataset_sha256"],
        "row_count_and_stages_exact": len(rows) == 825 and counts == Counter({"V2-04G-P1-E1": 270, "V3-05-B0": 270, "V3-09-F1": 180, "V3-11-F1": 60, "V3-16-S1": 45}),
        "no_hardware_or_real_vehicle": all(not row["hardware_accessed"] and not row["real_vehicle_used"] for row in rows),
    }
    value = {
        "schema_version": "1.0", "stage": "V3-18-B0-R1",
        "status": "v3_18_b0_r1_canonical_dataset_independent_review_pass" if all(checks.values()) else "v3_18_b0_r1_canonical_dataset_independent_review_fail",
        "checks": checks, "validation_errors": validation,
        "row_count": len(rows), "dataset_sha256": sha(JSONL.read_bytes()),
        "manifest_r1_sha256": sha(MANIFEST.read_bytes()),
        "repair_scope": "source hash closure only; canonical dataset bytes unchanged",
        "execution_authorized": False, "training_authorized": False,
        "hardware_accessed": False, "real_vehicle_used": False,
    }
    body = yaml.safe_dump(value, sort_keys=False).encode("utf-8")
    atomic(OUT, body, args.check)
    paths = (MANIFEST, JSONL, ROOT / "canonical_episodes.csv", ROOT / "data_dictionary.yaml", ROOT / "dataset_summary.yaml", OLD_REVIEW, OUT)
    freeze = "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in paths).encode("utf-8")
    atomic(FREEZE, freeze, args.check)
    print("status={} review_sha256={} freeze_sha256={}".format(value["status"], sha(body), sha(freeze)))


if __name__ == "__main__":
    main()

