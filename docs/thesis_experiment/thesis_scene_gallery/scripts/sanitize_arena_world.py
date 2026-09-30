#!/usr/bin/env python3
"""Create a temporary, presentation-only copy of an Arena Gazebo world.

The source world is never modified. Runtime plugins, sensors, actor scripts,
and the original GUI block are removed before a paused screenshot session is
started. The copy is only a visualisation input, not an experiment world.
"""

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path


def remove_children_by_tag(root, tags):
    removed = {tag: 0 for tag in tags}
    for parent in root.iter():
        for child in list(parent):
            if child.tag in tags:
                parent.remove(child)
                removed[child.tag] += 1
    return removed


def sanitize(source, output, camera_pose, drop_models):
    tree = ET.parse(source)
    sdf = tree.getroot()
    world = sdf.find("world")
    if world is None:
        raise ValueError(f"no <world> element in {source}")

    removed = remove_children_by_tag(world, {"plugin", "sensor", "script", "gui"})

    removed_models = []
    for model in list(world.findall("model")):
        model_name = model.get("name", "")
        uris = [node.text or "" for node in model.findall(".//uri")]
        if model_name in drop_models or any(
            (uri[len("model://") :] if uri.startswith("model://") else uri)
            in drop_models
            for uri in uris
        ):
            world.remove(model)
            removed_models.append(model_name)

    scene = world.find("scene")
    if scene is None:
        scene = ET.Element("scene")
        world.insert(0, scene)
    for tag in ("grid", "origin_visual"):
        node = scene.find(tag)
        if node is None:
            node = ET.SubElement(scene, tag)
        node.text = "false"

    gui = ET.Element("gui", {"fullscreen": "0"})
    camera = ET.SubElement(gui, "camera", {"name": "thesis_arena_camera"})
    ET.SubElement(camera, "pose").text = camera_pose
    ET.SubElement(camera, "view_controller").text = "orbit"
    ET.SubElement(camera, "projection_type").text = "perspective"
    world.insert(0, gui)

    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return removed, removed_models


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--camera-pose", required=True)
    parser.add_argument("--drop-model", action="append", default=[])
    args = parser.parse_args()

    removed, removed_models = sanitize(
        args.source, args.output, args.camera_pose, set(args.drop_model)
    )
    print(
        "sanitized",
        args.source,
        "plugins={plugin} sensors={sensor} scripts={script} gui={gui}".format(
            **removed
        ),
        f"models={len(removed_models)}",
    )


if __name__ == "__main__":
    main()
