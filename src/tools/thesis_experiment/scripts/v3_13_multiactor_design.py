#!/usr/bin/env python3
"""Pure helpers for the V3-13 multi-actor cloned-intervention study."""

import hashlib
import math
import xml.etree.ElementTree as ET


CANDIDATE_IDS = (
    "spatial_left_v120",
    "spatial_right_v120",
    "straight_v120",
    "certified_stop_v000",
)
POLICY_PAIRS = (
    ("cautious", "cautious"),
    ("cautious", "hesitant"),
    ("hesitant", "cautious"),
    ("hesitant", "hesitant"),
)


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def pose_text(x_m, y_m, z_m, yaw_rad=0.0):
    return "{:.9f} {:.9f} {:.9f} 0 0 {:.9f}".format(
        float(x_m), float(y_m), float(z_m), float(yaw_rad)
    )


def add_box(world, name, x_m, y_m, length_m, width_m, height_m):
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = pose_text(
        x_m, y_m, 0.5 * height_m
    )
    link = ET.SubElement(model, "link", {"name": "body"})
    for tag in ("collision", "visual"):
        item = ET.SubElement(link, tag, {"name": tag})
        geometry = ET.SubElement(item, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = "{:.6f} {:.6f} {:.6f}".format(
            length_m, width_m, height_m
        )
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


def deterministic_fraction(seed, actor_index, salt):
    payload = "{}:{}:{}".format(
        int(seed), int(actor_index), salt
    ).encode("ascii")
    integer = int(hashlib.sha256(payload).hexdigest()[:8], 16)
    return integer / float(0xFFFFFFFF)


def actor_parameters(seed, actor_index, actor_path):
    base_speed_mps = 0.90 + 0.20 * deterministic_fraction(
        seed, actor_index, "base_speed"
    )
    conflict_x_m = float(actor_path["conflict_x_m"])
    conflict_y_m = float(actor_path["conflict_y_m"])
    activation_ego_x_m = -3.0
    ego_distance_m = math.hypot(
        conflict_x_m - activation_ego_x_m, conflict_y_m
    )
    nominal_ttc_s = ego_distance_m / 0.8
    start_distance_m = (
        base_speed_mps * nominal_ttc_s
        + 0.08 * (
            deterministic_fraction(seed, actor_index, "start_distance")
            - 0.5
        )
    )
    return {
        "base_speed_mps": base_speed_mps,
        "start_coordinate_m": -start_distance_m,
        "nominal_activation_ttc_s": nominal_ttc_s,
    }


def build_schedule(contract):
    seeds = contract["cloned_intervention"]["seeds"]
    scenes = list(contract["scene_population"])
    require(len(seeds) == len(scenes) * len(POLICY_PAIRS), "SEED_COUNT")
    rows = []
    sequence = 0
    block = 0
    for scene in scenes:
        for policies in POLICY_PAIRS:
            seed = int(seeds[block])
            block += 1
            action_order = list(CANDIDATE_IDS)
            if block % 2 == 0:
                action_order.reverse()
            for candidate_id in action_order:
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "block_index": block,
                        "seed": seed,
                        "scene_id": scene["scene_id"],
                        "actor_policies_evaluator_only": list(policies),
                        "method": "cloned_intervention",
                        "forced_candidate_id": candidate_id,
                        "behavior_propensity": 0.25,
                        "attempt": 1,
                    }
                )
    return rows


def render_world(scene, actor_policies, seed):
    require(len(actor_policies) == len(scene["actor_paths"]), "ACTOR_POLICY_COUNT")
    sdf = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(
        sdf, "world", {"name": "v3_13_multiactor_world"}
    )
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    physics = ET.SubElement(
        world, "physics", {"name": "v3_13_physics", "type": "ode"}
    )
    ET.SubElement(physics, "max_step_size").text = "0.002"
    ET.SubElement(physics, "real_time_update_rate").text = "500"
    light = ET.SubElement(
        world, "light", {"name": "sun", "type": "directional"}
    )
    ET.SubElement(light, "pose").text = "0 0 10 0 0 0"
    ET.SubElement(light, "diffuse").text = "0.8 0.8 0.8 1"
    ET.SubElement(light, "specular").text = "0.2 0.2 0.2 1"
    ET.SubElement(light, "direction").text = "-0.5 0.1 -0.9"

    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    ground_link = ET.SubElement(ground, "link", {"name": "link"})
    collision = ET.SubElement(
        ground_link, "collision", {"name": "collision"}
    )
    geometry = ET.SubElement(collision, "geometry")
    plane = ET.SubElement(geometry, "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "50 40"

    half_width = scene.get("corridor_half_width_m")
    if half_width is not None:
        for side, y_m in (
            ("upper", float(half_width) + 0.10),
            ("lower", -float(half_width) - 0.10),
        ):
            add_box(
                world,
                "v3_13_{}_wall".format(side),
                0.5,
                y_m,
                15.0,
                0.20,
                1.0,
            )
    for index, rectangle in enumerate(
        scene.get("static_rectangles", [])
    ):
        add_box(
            world,
            "v3_13_static_{:02d}".format(index + 1),
            0.5
            * (
                float(rectangle["x_min_m"])
                + float(rectangle["x_max_m"])
            ),
            0.5
            * (
                float(rectangle["y_min_m"])
                + float(rectangle["y_max_m"])
            ),
            float(rectangle["x_max_m"])
            - float(rectangle["x_min_m"]),
            float(rectangle["y_max_m"])
            - float(rectangle["y_min_m"]),
            1.0,
        )

    actor_specs = []
    for index, (path, policy) in enumerate(
        zip(scene["actor_paths"], actor_policies), start=1
    ):
        parameters = actor_parameters(seed, index, path)
        angle_deg = float(path["angle_deg"])
        angle_rad = math.radians(angle_deg)
        direction_x = math.cos(angle_rad)
        direction_y = math.sin(angle_rad)
        origin_x = float(path["conflict_x_m"])
        origin_y = float(path["conflict_y_m"])
        start_coordinate = parameters["start_coordinate_m"]
        end_coordinate = 9.0
        start_x = origin_x + start_coordinate * direction_x
        start_y = origin_y + start_coordinate * direction_y
        end_x = origin_x + end_coordinate * direction_x
        end_y = origin_y + end_coordinate * direction_y
        model_name = "v3_13_actor_{:02d}".format(index)
        actor_model = add_box(
            world, model_name, start_x, start_y, 0.72, 0.72, 1.0
        )
        ET.SubElement(actor_model, "pose").text = pose_text(
            start_x, start_y, 0.5, angle_rad
        )
        plugin = ET.SubElement(
            actor_model,
            "plugin",
            {
                "name": "{}_controller".format(model_name),
                "filename": "libv3_reactive_actor_plugin.so",
            },
        )
        actor_seed = int(seed) * 10 + index
        fields = {
            "ego_model": "autolabor_m2",
            "policy": policy,
            "seed": actor_seed,
            "start": "{:.9f} {:.9f}".format(start_x, start_y),
            "end": "{:.9f} {:.9f}".format(end_x, end_y),
            "conflict": "{:.9f} {:.9f}".format(origin_x, origin_y),
            "base_speed": "{:.9f}".format(
                parameters["base_speed_mps"]
            ),
            "initial_speed": "{:.9f}".format(
                parameters["base_speed_mps"]
            ),
            "time_offset": "0.0",
            "start_delay": "5.0",
            "activation_ego_x": "-3.0",
            "ego_observation_period": "0.20",
            "ego_observation_hold": "0.30",
            "response_distance": "8.0",
            "response_time_margin": "1.35",
            "z": "0.5",
        }
        for key, value in fields.items():
            ET.SubElement(plugin, key).text = str(value)
        actor_specs.append(
            {
                "track_id": index,
                "model_name": model_name,
                "path_angle_deg": angle_deg,
                "path_origin_x_m": origin_x,
                "path_origin_y_m": origin_y,
                "actor_radius_m": 0.36,
                "evaluator_policy": policy,
                "plugin_seed": actor_seed,
                "generated_parameters": parameters,
            }
        )
    indent(sdf)
    return (
        '<?xml version="1.0"?>\n'
        + ET.tostring(sdf, encoding="unicode")
        + "\n",
        actor_specs,
    )

