#!/usr/bin/env python3
"""Render and execute fresh development-only benchmark episodes."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT = ROOT / "artifacts/gazebo_pct_teb_dwa_benchmark/development"
LAUNCH = "m2_pct_teb_dwa_benchmark_episode.launch"
SOURCE_BINDINGS = (
    ROOT / "scripts/gazebo_pct_teb_dwa_benchmark/run_development_matrix.py",
    ROOT / "src/application/v5_e2e_scope_rl/src/v5_e2e_scope_rl/benchmark_runtime.py",
    ROOT / "src/application/v5_e2e_scope_rl/scripts/pct_teb_dwa_benchmark_episode.py",
    ROOT / "src/simulation/m2_gazebo/launch/m2_pct_teb_dwa_benchmark_episode.launch",
    ROOT / "src/application/v5_e2e_scope_rl/config/pct_teb_dwa_benchmark_development.yaml",
)
ALLOWED_METHODS = ("pct_scope_rl", "pct_shadow", "analytic_prior", "actual_teb", "dwa")
SCENES = {
    "p02_static_corridor": {
        "corridor_half_width_m": 2.35,
        "static_boxes": [],
        "actors": [],
    },
    "p02_single_crossing": {
        "corridor_half_width_m": None,
        "static_boxes": [],
        "actors": [{"x_m": 0.0, "start_coordinate_m": -3.60, "end_coordinate_m": 4.50, "angle_deg": 90.0, "policy": "cautious"}],
    },
    "p02_occluded_crossing": {
        "corridor_half_width_m": None,
        "static_boxes": [{"name": "occluder", "x": -0.10, "y": -1.65, "length": 0.80, "width": 0.60, "height": 1.20}],
        "actors": [{"x_m": 0.0, "start_coordinate_m": -3.60, "end_coordinate_m": 4.50, "angle_deg": 90.0, "policy": "hesitant"}],
    },
    "core_static_avoidance": {
        "corridor_half_width_m": 2.55,
        "static_boxes": [{"name": "center_obstacle", "x": -0.15, "y": 0.0, "length": 0.80, "width": 0.90, "height": 1.20}],
        "actors": [],
    },
    "core_single_crossing": {
        "corridor_half_width_m": None,
        "static_boxes": [],
        "actors": [{"start": [0.0, -3.60], "end": [0.0, 4.50], "conflict": [0.0, 0.0], "policy": "cautious"}],
    },
    "core_head_on_yield": {
        "corridor_half_width_m": 2.55,
        "static_boxes": [],
        "actors": [{"start": [3.60, 1.40], "end": [-4.50, 1.40], "conflict": [0.0, 1.40], "policy": "cautious"}],
    },
    "core_sequential_interaction": {
        "corridor_half_width_m": None,
        "static_boxes": [],
        "actors": [
            {"start": [-1.0, -3.60], "end": [-1.0, 4.50], "conflict": [-1.0, 0.0], "policy": "cautious"},
            {"start": [1.35, 3.60], "end": [1.35, -4.50], "conflict": [1.35, 0.0], "policy": "hesitant"},
        ],
    },
    "core_partially_occluded": {
        "corridor_half_width_m": None,
        "static_boxes": [{"name": "occluder", "x": -0.10, "y": -1.65, "length": 0.80, "width": 0.60, "height": 1.20}],
        "actors": [{"start": [0.0, -3.60], "end": [0.0, 4.50], "conflict": [0.0, 0.0], "policy": "hesitant"}],
    },
    "core_width_change": {
        "corridor_half_width_m": None,
        "static_boxes": [
            {"name": "wide_upper_entry", "x": -2.50, "y": 2.45, "length": 3.00, "width": 0.20, "height": 1.00},
            {"name": "wide_lower_entry", "x": -2.50, "y": -2.45, "length": 3.00, "width": 0.20, "height": 1.00},
            {"name": "narrow_upper", "x": 1.00, "y": 1.70, "length": 4.00, "width": 0.20, "height": 1.00},
            {"name": "narrow_lower", "x": 1.00, "y": -1.70, "length": 4.00, "width": 0.20, "height": 1.00},
            {"name": "wide_upper_exit", "x": 4.00, "y": 2.45, "length": 2.00, "width": 0.20, "height": 1.00},
            {"name": "wide_lower_exit", "x": 4.00, "y": -2.45, "length": 2.00, "width": 0.20, "height": 1.00},
        ],
        "actors": [],
    },
}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def pose_text(x, y, z, yaw=0.0):
    return "{:.9f} {:.9f} {:.9f} 0 0 {:.9f}".format(x, y, z, yaw)


def add_box(world, name, x, y, length, width, height, static=True):
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true" if static else "false"
    ET.SubElement(model, "pose").text = pose_text(x, y, 0.5 * height)
    link = ET.SubElement(model, "link", {"name": "body"})
    for tag in ("collision", "visual"):
        item = ET.SubElement(link, tag, {"name": tag})
        geometry = ET.SubElement(item, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = "{:.6f} {:.6f} {:.6f}".format(length, width, height)
    return model


def indent(element, level=0):
    indentation = "\n" + level * "  "
    child_indentation = "\n" + (level + 1) * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_indentation
        for child in element:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = indentation
    if level and (not element.tail or not element.tail.strip()):
        element.tail = indentation


def actor_speed(seed, index):
    return 0.84 + ((int(seed) * 7 + index * 3) % 7) * 0.025


def render(scene_id, seed, split="development"):
    template = SCENES[scene_id]
    sdf = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(sdf, "world", {"name": "pct_teb_dwa_benchmark"})
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    physics = ET.SubElement(world, "physics", {"name": "benchmark_physics", "type": "ode"})
    ET.SubElement(physics, "max_step_size").text = "0.002"
    ET.SubElement(physics, "real_time_update_rate").text = "500"
    light = ET.SubElement(world, "light", {"name": "sun", "type": "directional"})
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    ET.SubElement(light, "direction").text = "-0.5 0.1 -0.9"
    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", {"name": "link"})
    collision = ET.SubElement(link, "collision", {"name": "collision"})
    geometry = ET.SubElement(collision, "geometry")
    plane = ET.SubElement(geometry, "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "40 30"

    half_width = template.get("corridor_half_width_m")
    if half_width is not None:
        for side, y in (("upper", half_width + 0.10), ("lower", -half_width - 0.10)):
            add_box(world, "benchmark_{}_wall".format(side), 0.0, y, 16.0, 0.20, 1.0)
    for box in template.get("static_boxes", []):
        add_box(world, "benchmark_" + box["name"], box["x"], box["y"], box["length"], box["width"], box["height"])

    actors = []
    for index, actor in enumerate(template.get("actors", [])):
        name = "benchmark_actor_{}".format(index + 1)
        if "start" in actor:
            start = tuple(float(value) for value in actor["start"])
            end = tuple(float(value) for value in actor["end"])
            conflict = tuple(float(value) for value in actor["conflict"])
            angle = math.atan2(end[1] - start[1], end[0] - start[0])
        else:
            angle = math.radians(float(actor["angle_deg"]))
            direction = (math.cos(angle), math.sin(angle))
            start = (
                float(actor["x_m"]) + float(actor["start_coordinate_m"]) * direction[0],
                float(actor["start_coordinate_m"]) * direction[1],
            )
            end = (
                float(actor["x_m"]) + float(actor["end_coordinate_m"]) * direction[0],
                float(actor["end_coordinate_m"]) * direction[1],
            )
            conflict = (float(actor["x_m"]), 0.0)
        model = add_box(world, name, start[0], start[1], 0.72, 0.72, 1.0, static=True)
        ET.SubElement(model, "pose").text = pose_text(start[0], start[1], 0.5, angle)
        plugin = ET.SubElement(model, "plugin", {"name": name + "_controller", "filename": "libv3_reactive_actor_plugin.so"})
        speed = actor_speed(seed, index)
        fields = {
            "ego_model": "autolabor_m2",
            "policy": actor["policy"],
            "seed": int(seed) + index * 1009,
            "start": "{:.9f} {:.9f}".format(*start),
            "end": "{:.9f} {:.9f}".format(*end),
            "conflict": "{:.9f} {:.9f}".format(*conflict),
            "base_speed": "{:.9f}".format(speed),
            "initial_speed": "{:.9f}".format(speed),
            "time_offset": "0.0",
            "start_delay": "5.0",
            "activation_ego_x": "-4.02",
            "ego_observation_period": "0.20",
            "ego_observation_hold": "0.30",
            "response_distance": "4.0",
            "response_time_margin": "1.35",
            "z": "0.5",
        }
        for key, value in fields.items():
            ET.SubElement(plugin, key).text = str(value)
        actors.append({
            "model_name": name, "radius_m": 0.36, "policy": actor["policy"],
            "base_speed_mps": speed, "start": list(start), "end": list(end),
            "conflict": list(conflict),
        })
    indent(sdf)
    world_text = '<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding="unicode") + "\n"
    manifest = {
        "schema_version": "1.0",
        "split": str(split),
        "scene_id": scene_id,
        "seed": int(seed),
        "corridor_half_width_m": half_width,
        "static_boxes": template.get("static_boxes", []),
        "static_circles": [],
        "actors": actors,
        "actor_policy_input_to_planners": False,
        "gazebo_model_states_evaluator_only": True,
    }
    return world_text, manifest


def stop(process):
    if process.poll() is not None:
        return process.returncode
    try:
        os.killpg(process.pid, signal.SIGINT)
        return process.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        return process.wait(timeout=5.0)


def execute(method, scene_id, seed, output, checkpoint, port, planner_config=None):
    if not 196001 <= seed <= 196120:
        raise RuntimeError("only registered development seeds 196001--196120 are executable")
    output = output.resolve()
    root = output / "episodes" / "{}_{}_{}".format(seed, scene_id, method)
    root.mkdir(parents=True, exist_ok=False)
    world, manifest = render(scene_id, seed)
    world_path = root / "world.sdf"
    manifest_path = root / "scene_manifest.yaml"
    report_path = root / "episode_report.yaml"
    log_path = root / "roslaunch.log"
    world_path.write_text(world, encoding="utf-8")
    manifest_path.write_text(yaml.safe_dump(manifest, sort_keys=False), encoding="utf-8")
    preflight_bindings_path = root / "preflight_bindings.json"
    preflight_bindings = {
        "schema_version": "1.0",
        "stage": "development",
        "method": method,
        "scene_id": scene_id,
        "seed": int(seed),
        "resources": [
            {"path": str(path.relative_to(ROOT)), "sha256": sha256(path)}
            for path in SOURCE_BINDINGS
        ],
        "world_sha256": sha256(world_path),
        "scene_manifest_sha256": sha256(manifest_path),
        "planner_config_path": str(planner_config.resolve().relative_to(ROOT)) if planner_config else None,
        "planner_config_sha256": sha256(planner_config.resolve()) if planner_config else None,
    }
    preflight_bindings_path.write_text(
        json.dumps(preflight_bindings, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    runtime = root / "runtime"
    (runtime / "roslog").mkdir(parents=True)
    environment = dict(os.environ)
    environment.update(
        {
            "THESIS_WS": str(ROOT),
            "ROS_HOME": str((runtime / "ros_home").resolve()),
            "ROS_LOG_DIR": str((runtime / "roslog").resolve()),
            "ROS_MASTER_URI": "http://127.0.0.1:{}".format(port),
            "GAZEBO_MASTER_URI": "http://127.0.0.1:{}".format(port + 1),
            "GAZEBO_MODEL_DATABASE_URI": "",
        }
    )
    command = [
        "roslaunch",
        "m2_gazebo",
        LAUNCH,
        "world:={}".format(world_path.resolve()),
        "method:={}".format(method),
        "scene_manifest:={}".format(manifest_path.resolve()),
        "scene_id:={}".format(scene_id),
        "seed:={}".format(seed),
        "report_path:={}".format(report_path.resolve()),
        "actor_checkpoint_path:={}".format(checkpoint.resolve() if checkpoint else ""),
        "gui:=false",
    ]
    if planner_config is not None:
        planner_config = planner_config.resolve()
        if not planner_config.is_file():
            raise RuntimeError("planner config absent: {}".format(planner_config))
        if method == "actual_teb":
            command.append("teb_config:={}".format(planner_config))
        elif method == "dwa":
            command.append("dwa_config:={}".format(planner_config))
        else:
            raise RuntimeError("planner config is valid only for actual_teb or dwa")
    started = time.monotonic()
    with log_path.open("wb") as log:
        process = subprocess.Popen(
            command,
            cwd=str(ROOT),
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        timed_out = False
        try:
            return_code = process.wait(timeout=150.0)
        except subprocess.TimeoutExpired:
            timed_out = True
            return_code = stop(process)
        else:
            stop(process)
    receipt = {
        "method": method,
        "scene_id": scene_id,
        "seed": seed,
        "return_code": return_code,
        "wall_elapsed_s": time.monotonic() - started,
        "timed_out": timed_out,
        "world_sha256": sha256(world_path),
        "scene_manifest_sha256": sha256(manifest_path),
        "roslaunch_log_sha256": sha256(log_path),
        "episode_report_present": report_path.is_file(),
        "episode_report_sha256": sha256(report_path) if report_path.is_file() else None,
        "planner_config_path": str(planner_config) if planner_config is not None else None,
        "planner_config_sha256": sha256(planner_config) if planner_config is not None else None,
        "preflight_bindings_path": str(preflight_bindings_path.relative_to(ROOT)),
        "preflight_bindings_sha256": sha256(preflight_bindings_path),
        "thesis_workspace_path": str(ROOT),
    }
    (root / "receipt.json").write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if not report_path.is_file():
        raise RuntimeError("episode report absent; tail:\n" + log_path.read_text(errors="replace")[-8000:])
    return root, yaml.safe_load(report_path.read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--method", choices=ALLOWED_METHODS, required=True)
    parser.add_argument("--scene", choices=sorted(SCENES), required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--checkpoint", type=Path)
    parser.add_argument("--planner-config", type=Path)
    parser.add_argument("--port", type=int, default=14700)
    args = parser.parse_args()
    if args.method in ("pct_scope_rl", "pct_shadow") and args.checkpoint is None:
        parser.error("PCT and shadow require --checkpoint")
    root, report = execute(
        args.method, args.scene, args.seed, args.output, args.checkpoint, args.port,
        args.planner_config,
    )
    print(root.resolve().relative_to(ROOT))
    print(
        "goal={} integrity={} time={} clearance={}".format(
            report["result"]["goal_reached"],
            report["execution_integrity_pass"],
            report["result"]["restricted_elapsed_s"],
            report["result"]["minimum_conservative_clearance_m"],
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
