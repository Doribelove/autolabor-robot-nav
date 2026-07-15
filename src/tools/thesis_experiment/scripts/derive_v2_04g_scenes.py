#!/usr/bin/env python3
"""Derive preregistered V2-04G calibration scenes from a frozen source."""

import argparse
import copy
import hashlib
from pathlib import Path

import yaml


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _deep_update(target, patch):
    for key, value in patch.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _deep_update(target[key], value)
        else:
            target[key] = copy.deepcopy(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("spec", type=Path)
    parser.add_argument("--workspace", type=Path,
                        default=Path("/home/robot/robot_ws_base_rl"))
    args = parser.parse_args()
    spec = yaml.safe_load(args.spec.read_text(encoding="utf-8"))
    source_path = args.workspace / spec["source_manifest"]["path"]
    if _sha256(source_path) != spec["source_manifest"]["sha256"]:
        raise ValueError("V2-04G source scene manifest hash drifted")
    source = yaml.safe_load(source_path.read_text(encoding="utf-8"))
    by_id = {scene["scene_id"]: scene for scene in source["scenes"]}
    forbidden = set().union(*spec["forbidden_seed_sets"].values())
    seeds = [row["seed"] for row in spec["scene_derivations"]]
    if len(seeds) != len(set(seeds)) or set(seeds) & forbidden:
        raise ValueError("V2-04G calibration seed firewall violated")
    target = copy.deepcopy(source)
    target["manifest_id"] = spec["target_manifest_id"]
    target["scenes"] = []
    for row in spec["scene_derivations"]:
        scene = copy.deepcopy(by_id[row["source_scene_id"]])
        scene["scene_id"] = row["target_scene_id"]
        scene["seed"] = row["seed"]
        scene["split"] = "calibration"
        scene["layout"]["variant"] = row["layout_variant"]
        scene["evaluator_only"]["reason"] = row["evaluator_reason"]
        _deep_update(scene, row.get("scene_patch", {}))
        target["scenes"].append(scene)
    output = args.workspace / spec["target_path"]
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(target, sort_keys=False), encoding="utf-8")
    temporary.replace(output)
    print(yaml.safe_dump({
        "output": str(output), "sha256": _sha256(output),
        "scene_count": len(target["scenes"]), "seeds": seeds,
    }, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
