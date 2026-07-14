#!/usr/bin/env python3
"""Materialize a new-seed V2 calibration manifest from a frozen layout source."""

import argparse
import copy
import hashlib
from pathlib import Path

import yaml


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("spec", type=Path)
    parser.add_argument("--workspace", type=Path,
                        default=Path("/home/robot/robot_ws_base_rl"))
    args = parser.parse_args()
    spec = yaml.safe_load(args.spec.read_text(encoding="utf-8"))
    source_path = args.workspace / spec["source_manifest"]["path"]
    if _sha256(source_path) != spec["source_manifest"]["sha256"]:
        raise ValueError("source calibration scene manifest hash drifted")
    source = yaml.safe_load(source_path.read_text(encoding="utf-8"))
    by_id = {scene["scene_id"]: scene for scene in source["scenes"]}
    forbidden = set().union(*(values for values in spec["forbidden_seed_sets"].values()))
    replacements = spec["scene_replacements"]
    seeds = [row["seed"] for row in replacements]
    if len(seeds) != len(set(seeds)) or set(seeds) & forbidden:
        raise ValueError("derived calibration seed firewall violated")
    target = copy.deepcopy(source)
    target["manifest_id"] = spec["target_manifest_id"]
    target["scenes"] = []
    for replacement in replacements:
        scene = copy.deepcopy(by_id[replacement["source_scene_id"]])
        scene["scene_id"] = replacement["target_scene_id"]
        scene["seed"] = replacement["seed"]
        scene["split"] = spec["split"]
        scene["layout"]["variant"] = replacement["layout_variant"]
        scene["evaluator_only"]["reason"] = spec.get(
            "evaluator_reason", "new_derived_calibration_seed"
        )
        target["scenes"].append(scene)
    output = args.workspace / spec["target_path"]
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(target, sort_keys=False), encoding="utf-8")
    temporary.replace(output)
    print(yaml.safe_dump({"output": str(output), "sha256": _sha256(output),
                          "scene_count": len(target["scenes"])}, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
