#!/usr/bin/env python3
"""Compile a validated V2 scene manifest into deterministic YAML/SDF artifacts."""

import argparse
import hashlib
import json
import os
from pathlib import Path

import yaml

from thesis_experiment.v2_scene import (
    V2SceneError,
    compile_v2_manifest,
    load_v2_scene_manifest,
    render_v2_scene_sdf,
)


def _atomic_write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(content, encoding="utf-8")
    os.replace(str(temporary), str(path))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest")
    parser.add_argument("output_dir")
    parser.add_argument("--workspace", default="/home/robot/robot_ws_base_rl")
    args = parser.parse_args()
    root = Path(args.workspace).resolve()
    output = Path(args.output_dir).resolve()
    allowed = (root / "artifacts" / "v2").resolve()
    try:
        output.relative_to(allowed)
    except ValueError:
        parser.error("output_dir must remain under artifacts/v2")

    try:
        manifest = load_v2_scene_manifest(args.manifest, root)
        instances = compile_v2_manifest(manifest, root)
    except V2SceneError as exc:
        print(json.dumps({"status": "invalid", "error": str(exc)}, ensure_ascii=False))
        return 2

    checksums = []
    for instance in instances:
        scene_id = instance["scene"]["scene_id"]
        instance_path = output / (scene_id + ".instance.yaml")
        world_path = output / (scene_id + ".world")
        instance_text = yaml.safe_dump(
            instance, sort_keys=False, allow_unicode=True
        )
        world_text = render_v2_scene_sdf(instance)
        _atomic_write(instance_path, instance_text)
        _atomic_write(world_path, world_text)
        for path, content in ((instance_path, instance_text), (world_path, world_text)):
            checksums.append({
                "path": path.relative_to(root).as_posix(),
                "sha256": hashlib.sha256(content.encode("utf-8")).hexdigest(),
            })
    index = {
        "schema_version": "2.0",
        "manifest_id": manifest["manifest_id"],
        "formal_result": False,
        "runtime_ready": False,
        "scene_count": len(instances),
        "families": [instance["scene"]["family"] for instance in instances],
        "files": checksums,
    }
    _atomic_write(
        output / "compiled_scene_index.yaml",
        yaml.safe_dump(index, sort_keys=False, allow_unicode=True),
    )
    print(json.dumps({"status": "valid", "scene_count": len(instances),
                      "output_dir": str(output)}, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
