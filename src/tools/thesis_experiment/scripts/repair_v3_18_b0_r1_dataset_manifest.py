#!/usr/bin/env python3
"""Rebind V3-18-B0 source bytes after a preserved post-build source-hash drift."""
import argparse
import hashlib
import os
from pathlib import Path


W = Path("/home/robot/robot_ws_base_rl")
ROOT = W / "artifacts/v3/thesis/v3_18_b0_canonical_dataset"
OLD = ROOT / "source_and_output_manifest.sha256"
OUT = ROOT / "source_and_output_manifest_r1.sha256"
DATA = (
    ROOT / "canonical_episodes.jsonl",
    ROOT / "canonical_episodes.csv",
    ROOT / "data_dictionary.yaml",
    ROOT / "dataset_summary.yaml",
)
EXTRA = (
    OLD,
    ROOT / "independent_review.yaml",
    ROOT / "terminal_freeze.sha256",
    W / "src/tools/thesis_experiment/scripts/repair_v3_18_b0_r1_dataset_manifest.py",
    W / "src/tools/thesis_experiment/scripts/review_v3_18_b0_r1_canonical_dataset.py",
)


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


def content():
    original_paths = [W / line.split("  ", 1)[1] for line in OLD.read_text().splitlines() if line]
    data_set = set(DATA)
    resources = sorted((set(original_paths) - data_set) | set(DATA) | set(EXTRA))
    if not all(path.is_file() and path.is_absolute() and W in path.parents for path in resources):
        raise RuntimeError("R1 resource roster is incomplete")
    return "".join("{}  {}\n".format(sha(path.read_bytes()), path.relative_to(W)) for path in resources).encode("utf-8")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    body = content()
    atomic(OUT, body, args.check)
    print("status=v3_18_b0_r1_dataset_manifest_repair_pass manifest_sha256={}".format(sha(body)))


if __name__ == "__main__":
    main()

