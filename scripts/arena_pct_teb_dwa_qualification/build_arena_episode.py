#!/usr/bin/env python3
"""Build an immutable Arena-world snapshot for one qualification block."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]
ARENA_ROOT = ROOT / "src/navigation_arena/arena-rosnav-3D"
SOURCE_WORLD = ARENA_ROOT / (
    "simulator_setup/worlds/hospital/worlds/hospital.world"
)
MAP_YAML = ARENA_ROOT / "simulator_setup/maps/hospital/map.yaml"
MAP_IMAGE = ARENA_ROOT / "simulator_setup/maps/hospital/map.pgm"
EXPECTED_SOURCE_WORLD_SHA256 = (
    "818a5eab5a9eedc43050d2fb3a2345d267dd32843c4139f29d58d06b396e2c95"
)
EXPECTED_MAP_YAML_SHA256 = (
    "35868469109a420641f28750c5cbfa7306705520997b094d3c71574c33e8efef"
)
EXPECTED_MAP_IMAGE_SHA256 = (
    "c5eb8d4d802a4a954166bb2068c2623455be3312ddd9ccb6af9ca8335bd2c147"
)

ROUTE = {
    "origin_world_x_m": 0.0,
    "origin_world_y_m": 11.2,
    "yaw_world_rad": 0.0,
    "spawn_route_x_m": -5.0,
    "policy_start_route_x_m": -4.0,
    "goal_route_x_m": 4.2,
}

SCENES = {
    "hospital_static": {"actor": None},
    "hospital_single_crossing": {
        "actor": {
            "model_name": "arena_q1_actor_1",
            "start_route_m": [0.0, -2.7],
            "end_route_m": [0.0, 2.7],
            "conflict_route_m": [0.0, 0.0],
            "radius_m": 0.36,
            "base_speed_mps": 0.72,
            "policy": "cautious",
        }
    },
}


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def workspace_path_or_absolute(path):
    path = Path(path).resolve()
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def pose_text(x_m, y_m, z_m, yaw_rad=0.0):
    return "{:.9f} {:.9f} {:.9f} 0 0 {:.9f}".format(
        x_m, y_m, z_m, yaw_rad
    )


def route_to_world(point):
    x_m, y_m = (float(value) for value in point)
    yaw = ROUTE["yaw_world_rad"]
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (
        ROUTE["origin_world_x_m"] + cosine * x_m - sine * y_m,
        ROUTE["origin_world_y_m"] + sine * x_m + cosine * y_m,
    )


def add_controlled_actor(world, specification, seed):
    start = route_to_world(specification["start_route_m"])
    end = route_to_world(specification["end_route_m"])
    conflict = route_to_world(specification["conflict_route_m"])
    actor_yaw = math.atan2(end[1] - start[1], end[0] - start[0])
    model = ET.SubElement(world, "model", {"name": specification["model_name"]})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = pose_text(start[0], start[1], 0.5, actor_yaw)
    link = ET.SubElement(model, "link", {"name": "body"})
    for tag in ("collision", "visual"):
        item = ET.SubElement(link, tag, {"name": tag})
        geometry = ET.SubElement(item, "geometry")
        cylinder = ET.SubElement(geometry, "cylinder")
        ET.SubElement(cylinder, "radius").text = str(specification["radius_m"])
        ET.SubElement(cylinder, "length").text = "1.00"
        if tag == "visual":
            material = ET.SubElement(item, "material")
            ET.SubElement(material, "ambient").text = "0.95 0.28 0.08 1"
            ET.SubElement(material, "diffuse").text = "0.95 0.28 0.08 1"
    plugin = ET.SubElement(
        model,
        "plugin",
        {
            "name": specification["model_name"] + "_controller",
            "filename": "libv3_reactive_actor_plugin.so",
        },
    )
    values = {
        "ego_model": "autolabor_m2",
        "policy": specification["policy"],
        "seed": int(seed),
        "start": "{:.9f} {:.9f}".format(*start),
        "end": "{:.9f} {:.9f}".format(*end),
        "conflict": "{:.9f} {:.9f}".format(*conflict),
        "base_speed": specification["base_speed_mps"],
        "initial_speed": specification["base_speed_mps"],
        "time_offset": 0.0,
        "start_delay": 5.0,
        "ego_observation_period": 0.20,
        "ego_observation_hold": 0.30,
        "response_distance": 4.0,
        "response_time_margin": 1.35,
        # Keep the actor at the exact start pose through the common warmup
        # gate.  Method-dependent response begins only after the branch.
        "activation_ego_x": -3.90,
        "z": 0.5,
    }
    for name, value in values.items():
        ET.SubElement(plugin, name).text = str(value)
    return {
        **specification,
        "start_world_m": list(start),
        "end_world_m": list(end),
        "conflict_world_m": list(conflict),
        "plugin": "libv3_reactive_actor_plugin.so",
        "planner_truth_access": False,
    }


def build(
    scene_id,
    seed,
    output_directory,
    stage,
    release_sha256=None,
    release_review_sha256=None,
):
    if scene_id not in SCENES:
        raise ValueError("UNKNOWN_ARENA_Q1_SCENE:" + str(scene_id))
    for path, expected in (
        (SOURCE_WORLD, EXPECTED_SOURCE_WORLD_SHA256),
        (MAP_YAML, EXPECTED_MAP_YAML_SHA256),
        (MAP_IMAGE, EXPECTED_MAP_IMAGE_SHA256),
    ):
        observed = sha256(path)
        if observed != expected:
            raise RuntimeError(
                "FROZEN_ARENA_SOURCE_HASH_MISMATCH:{}:{}".format(path, observed)
            )

    source_hash_before = sha256(SOURCE_WORLD)
    tree = ET.parse(SOURCE_WORLD)
    sdf = tree.getroot()
    world = sdf.find("world")
    if world is None:
        raise RuntimeError("ARENA_SOURCE_WORLD_ELEMENT_ABSENT")
    world.set("name", "arena_pct_q1_{}_{}".format(scene_id, int(seed)))

    removed_actors = []
    for actor in list(world.findall("actor")):
        removed_actors.append(actor.get("name", ""))
        world.remove(actor)
    removed_plugins = []
    for plugin in list(world.findall("plugin")):
        removed_plugins.append(
            {"name": plugin.get("name"), "filename": plugin.get("filename")}
        )
        world.remove(plugin)
    removed_robot_models = []
    for model in list(world.findall("model")):
        name = str(model.get("name", ""))
        if name in {"autolabor_m2", "turtlebot3", "turtlebot3_waffle_pi"}:
            removed_robot_models.append(name)
            world.remove(model)

    actors = []
    actor_specification = SCENES[scene_id]["actor"]
    if actor_specification is not None:
        actors.append(add_controlled_actor(world, actor_specification, seed))

    output_directory = Path(output_directory).resolve()
    try:
        output_directory.relative_to(ARENA_ROOT.resolve())
    except ValueError:
        pass
    else:
        raise RuntimeError("ARENA_SOURCE_TREE_OUTPUT_FORBIDDEN")
    output_directory.mkdir(parents=True, exist_ok=False)
    world_path = output_directory / "world.sdf"
    manifest_path = output_directory / "scene_manifest.yaml"
    receipt_path = output_directory / "snapshot_receipt.json"
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    tree.write(world_path, encoding="utf-8", xml_declaration=True)

    manifest = {
        "schema_version": "1.0",
        "stage": str(stage),
        "release_sha256": release_sha256,
        "release_review_sha256": release_review_sha256,
        "split": "arena_qualification",
        "scene_id": scene_id,
        "seed": int(seed),
        "arena_world_family": "hospital",
        "task_type": "static" if actor_specification is None else "single_crossing",
        "source_world_path": str(SOURCE_WORLD.relative_to(ROOT)),
        "source_world_sha256": source_hash_before,
        "snapshot_world_path": workspace_path_or_absolute(world_path),
        "map_yaml_path": str(MAP_YAML.relative_to(ROOT)),
        "map_yaml_sha256": sha256(MAP_YAML),
        "map_image_path": str(MAP_IMAGE.relative_to(ROOT)),
        "map_image_sha256": sha256(MAP_IMAGE),
        "route": dict(ROUTE),
        "corridor_half_width_m": None,
        "static_boxes": [],
        "static_circles": [],
        "actors": actors,
        "removed_source_actors": removed_actors,
        "removed_source_world_plugins": removed_plugins,
        "removed_source_robot_models": removed_robot_models,
        "actor_policy_input_to_planners": False,
        "actor_seed_parameter_jitter_enabled": False,
        "actor_activation_ego_route_x_m": (
            -3.90 if actor_specification is not None else None
        ),
        "gazebo_model_states_evaluator_only": True,
        "source_world_mutated": False,
    }
    with manifest_path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(manifest, stream, sort_keys=False, allow_unicode=True)
        stream.flush()
        os.fsync(stream.fileno())
    source_hash_after = sha256(SOURCE_WORLD)
    if source_hash_after != source_hash_before:
        raise RuntimeError("ARENA_SOURCE_WORLD_CHANGED_DURING_SNAPSHOT")
    receipt = {
        "schema_version": "1.0",
        "scene_id": scene_id,
        "seed": int(seed),
        "release_sha256": release_sha256,
        "release_review_sha256": release_review_sha256,
        "source_world_sha256_before": source_hash_before,
        "source_world_sha256_after": source_hash_after,
        "snapshot_world_sha256": sha256(world_path),
        "scene_manifest_sha256": sha256(manifest_path),
        "removed_actor_count": len(removed_actors),
        "injected_actor_count": len(actors),
    }
    receipt_path.write_text(
        json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return world_path, manifest_path, receipt_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", choices=sorted(SCENES), required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--stage", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    paths = build(args.scene, args.seed, args.output, args.stage)
    for path in paths:
        print(workspace_path_or_absolute(path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
