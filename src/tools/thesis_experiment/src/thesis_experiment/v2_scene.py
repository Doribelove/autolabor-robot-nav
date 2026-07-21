"""Fail-closed V2 scene manifest validation and deterministic SDF compilation."""

import copy
import hashlib
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional, Sequence

import yaml


SCENE_FAMILIES = ("CRUISE", "DYNAMIC", "STATIC_DENSE", "CORRIDOR", "MANEUVER")
SCENE_SPLITS = ("calibration", "validation", "test_id", "test_ood")
POLICY_FORBIDDEN_FIELDS = ("family", "split", "evaluator_only")
IDENTIFIER = re.compile(r"^[a-z0-9][a-z0-9_-]*$")


class V2SceneError(ValueError):
    """Raised when a V2 scene can leak labels or compile ambiguously."""


def canonical_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def file_sha256(path: Any) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _exact(data: Mapping[str, Any], keys: Sequence[str], context: str) -> None:
    if not isinstance(data, dict):
        raise V2SceneError("{} must be a mapping".format(context))
    missing = sorted(set(keys) - set(data))
    extra = sorted(set(data) - set(keys))
    if missing or extra:
        raise V2SceneError(
            "{} keys differ; missing={}, extra={}".format(context, missing, extra)
        )


def _finite(value: Any, context: str, positive: bool = False,
            non_negative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise V2SceneError("{} must be numeric".format(context))
    result = float(value)
    if not math.isfinite(result):
        raise V2SceneError("{} must be finite".format(context))
    if positive and result <= 0.0:
        raise V2SceneError("{} must be positive".format(context))
    if non_negative and result < 0.0:
        raise V2SceneError("{} must be non-negative".format(context))
    return result


def _identifier(value: Any, context: str) -> str:
    if not isinstance(value, str) or not IDENTIFIER.fullmatch(value):
        raise V2SceneError("{} must match {}".format(context, IDENTIFIER.pattern))
    return value


def _pose(value: Any, context: str) -> Dict[str, float]:
    _exact(value, ("x_m", "y_m", "yaw_rad"), context)
    return {key: _finite(value[key], "{}.{}".format(context, key))
            for key in ("x_m", "y_m", "yaw_rad")}


def _size(value: Any, context: str) -> List[float]:
    if not isinstance(value, list) or len(value) != 3:
        raise V2SceneError("{} must be [x, y, z]".format(context))
    return [_finite(item, "{}[{}]".format(context, index), positive=True)
            for index, item in enumerate(value)]


def _workspace_file(root: Path, relative: Any, context: str) -> Path:
    if not isinstance(relative, str) or not relative or Path(relative).is_absolute():
        raise V2SceneError("{} must be a workspace-relative path".format(context))
    if ".." in Path(relative).parts:
        raise V2SceneError("{} cannot escape the workspace".format(context))
    resolved_root = root.resolve()
    resolved = (resolved_root / relative).resolve()
    try:
        resolved.relative_to(resolved_root)
    except ValueError as exc:
        raise V2SceneError("{} escapes the workspace".format(context)) from exc
    if not resolved.is_file():
        raise V2SceneError("{} does not exist: {}".format(context, relative))
    return resolved


def _validate_obstacle(raw: Any, context: str) -> Dict[str, Any]:
    _exact(raw, ("obstacle_id", "shape", "pose", "size_m"), context)
    if raw["shape"] != "box":
        raise V2SceneError("{}.shape currently supports only box".format(context))
    return {
        "obstacle_id": _identifier(raw["obstacle_id"], context + ".obstacle_id"),
        "shape": "box",
        "pose": _pose(raw["pose"], context + ".pose"),
        "size_m": _size(raw["size_m"], context + ".size_m"),
    }


def _validate_agent(raw: Any, context: str) -> Dict[str, Any]:
    _exact(raw, ("agent_id", "shape", "size_m", "loop", "trajectory"), context)
    if raw["shape"] != "box" or not isinstance(raw["loop"], bool):
        raise V2SceneError("{}.shape/loop is invalid".format(context))
    trajectory = raw["trajectory"]
    if not isinstance(trajectory, list) or len(trajectory) < 2:
        raise V2SceneError("{}.trajectory needs at least two waypoints".format(context))
    points = []
    last_time = -1.0
    for index, point in enumerate(trajectory):
        label = "{}.trajectory[{}]".format(context, index)
        _exact(point, ("time_s", "x_m", "y_m", "yaw_rad"), label)
        time_s = _finite(point["time_s"], label + ".time_s", non_negative=True)
        if time_s <= last_time:
            raise V2SceneError("{} times must be strictly increasing".format(context))
        last_time = time_s
        points.append({
            "time_s": time_s,
            "x_m": _finite(point["x_m"], label + ".x_m"),
            "y_m": _finite(point["y_m"], label + ".y_m"),
            "yaw_rad": _finite(point["yaw_rad"], label + ".yaw_rad"),
        })
    return {
        "agent_id": _identifier(raw["agent_id"], context + ".agent_id"),
        "shape": "box",
        "size_m": _size(raw["size_m"], context + ".size_m"),
        "loop": raw["loop"],
        "trajectory": points,
    }


def validate_v2_scene_manifest(
    manifest: Mapping[str, Any], workspace_root: Any
) -> Dict[str, Any]:
    """Validate all labels, provenance, family coverage, and policy boundaries."""

    root = Path(workspace_root)
    _exact(
        manifest,
        (
            "schema_version", "architecture_generation", "manifest_id", "status",
            "simulation_only", "formal_experiment", "runtime_ready",
            "real_vehicle_use_forbidden", "generator", "simulation_contract",
            "robot", "policy_boundary", "scenes",
        ),
        "v2_scene_manifest",
    )
    if str(manifest["schema_version"]) != "2.0" or manifest["architecture_generation"] != "v2":
        raise V2SceneError("scene manifest must be V2 schema 2.0")
    if manifest["status"] != "component_candidate_frozen":
        raise V2SceneError("scene manifest status drifted")
    if not (
        manifest["simulation_only"] is True
        and manifest["formal_experiment"] is False
        and manifest["runtime_ready"] is False
        and manifest["real_vehicle_use_forbidden"] is True
    ):
        raise V2SceneError("V2-02 scenes must stay non-formal and simulation-only")

    generator = manifest["generator"]
    _exact(generator, ("name", "version", "source", "sha256"), "generator")
    if generator["name"] != "v2_scene_compiler" or generator["version"] != "2.0.0":
        raise V2SceneError("generator identity drifted")
    generator_path = _workspace_file(root, generator["source"], "generator.source")
    if file_sha256(generator_path) != generator["sha256"]:
        raise V2SceneError("generator source hash mismatch")

    simulation = manifest["simulation_contract"]
    _exact(simulation, ("path", "sha256", "profile"), "simulation_contract")
    simulation_path = _workspace_file(root, simulation["path"], "simulation_contract.path")
    if file_sha256(simulation_path) != simulation["sha256"]:
        raise V2SceneError("simulation contract hash mismatch")
    if simulation["profile"] != "v2_02_dynamics":
        raise V2SceneError("scene simulation profile drifted")

    robot = manifest["robot"]
    _exact(
        robot,
        ("model", "candidate_config", "candidate_sha256", "footprint", "footprint_sha256"),
        "robot",
    )
    if robot["model"] != "autolabor_m2":
        raise V2SceneError("robot model drifted")
    candidate_path = _workspace_file(root, robot["candidate_config"], "robot.candidate_config")
    if file_sha256(candidate_path) != robot["candidate_sha256"]:
        raise V2SceneError("robot candidate hash mismatch")
    footprint = robot["footprint"]
    _exact(footprint, ("length_m", "width_m", "height_m"), "robot.footprint")
    normalized_footprint = {
        key: _finite(footprint[key], "robot.footprint." + key, positive=True)
        for key in ("length_m", "width_m", "height_m")
    }
    if canonical_sha256(normalized_footprint) != robot["footprint_sha256"]:
        raise V2SceneError("robot footprint hash mismatch")

    boundary = manifest["policy_boundary"]
    _exact(
        boundary,
        ("runtime_manifest_access", "runtime_label_topics", "evaluator_only_fields"),
        "policy_boundary",
    )
    if boundary["runtime_manifest_access"] is not False or boundary["runtime_label_topics"] != []:
        raise V2SceneError("runtime must not access manifest labels")
    if tuple(boundary["evaluator_only_fields"]) != (
        "family", "split", "evaluator_only.feasible", "evaluator_only.reason"
    ):
        raise V2SceneError("evaluator-only field list drifted")

    scenes = manifest["scenes"]
    if not isinstance(scenes, list) or not scenes:
        raise V2SceneError("scenes must be a non-empty list")
    normalized = []
    ids = set()
    for index, raw in enumerate(scenes):
        label = "scenes[{}]".format(index)
        _exact(
            raw,
            (
                "scene_id", "family", "split", "seed", "world_template", "start",
                "goal", "timeout_s", "layout", "static_obstacles", "dynamic_agents",
                "randomization", "sensor_profile", "actuator_profile", "collision",
                "success", "evaluator_only", "policy_forbidden_fields",
            ),
            label,
        )
        scene_id = _identifier(raw["scene_id"], label + ".scene_id")
        if scene_id in ids:
            raise V2SceneError("duplicate scene_id {}".format(scene_id))
        ids.add(scene_id)
        if raw["family"] not in SCENE_FAMILIES or raw["split"] not in SCENE_SPLITS:
            raise V2SceneError("{}.family/split is invalid".format(label))
        if isinstance(raw["seed"], bool) or not isinstance(raw["seed"], int) or raw["seed"] < 0:
            raise V2SceneError("{}.seed must be a non-negative integer".format(label))
        if raw["world_template"] != "generated_v2_sdf":
            raise V2SceneError("{}.world_template drifted".format(label))
        layout = raw["layout"]
        _exact(layout, ("variant", "reference_centerline"), label + ".layout")
        _identifier(layout["variant"], label + ".layout.variant")
        centerline = layout["reference_centerline"]
        if not isinstance(centerline, list) or len(centerline) < 2:
            raise V2SceneError("{}.reference_centerline needs at least two points".format(label))
        normalized_centerline = []
        for point_index, point in enumerate(centerline):
            if not isinstance(point, list) or len(point) != 2:
                raise V2SceneError("{}.reference_centerline point is invalid".format(label))
            normalized_centerline.append([
                _finite(point[0], "{}.centerline[{}].x".format(label, point_index)),
                _finite(point[1], "{}.centerline[{}].y".format(label, point_index)),
            ])
        obstacles = [_validate_obstacle(item, "{}.static_obstacles[{}]".format(label, item_index))
                     for item_index, item in enumerate(raw["static_obstacles"])]
        agents = [_validate_agent(item, "{}.dynamic_agents[{}]".format(label, item_index))
                  for item_index, item in enumerate(raw["dynamic_agents"])]
        obstacle_ids = [item["obstacle_id"] for item in obstacles]
        agent_ids = [item["agent_id"] for item in agents]
        if len(obstacle_ids) != len(set(obstacle_ids)) or len(agent_ids) != len(set(agent_ids)):
            raise V2SceneError("{} obstacle/agent identifiers must be unique".format(label))
        if set(obstacle_ids) & set(agent_ids):
            raise V2SceneError("{} obstacle and agent identifiers overlap".format(label))

        randomization = raw["randomization"]
        _exact(
            randomization,
            ("position_jitter_m", "yaw_jitter_rad", "agent_time_jitter_s"),
            label + ".randomization",
        )
        normalized_randomization = {
            key: _finite(randomization[key], "{}.randomization.{}".format(label, key),
                         non_negative=True)
            for key in ("position_jitter_m", "yaw_jitter_rad", "agent_time_jitter_s")
        }
        if raw["sensor_profile"] != "v2_02_laser" or raw["actuator_profile"] != "v2_02_dynamics":
            raise V2SceneError("{} simulation profiles drifted".format(label))
        collision = raw["collision"]
        _exact(collision, ("source", "topic", "terminate"), label + ".collision")
        if collision != {
            "source": "gazebo_contacts", "topic": "/m2_gazebo/contacts", "terminate": True
        }:
            raise V2SceneError("{} collision contract drifted".format(label))
        success = raw["success"]
        _exact(
            success,
            ("goal_tolerance_m", "yaw_tolerance_rad", "stopped_speed_max_mps"),
            label + ".success",
        )
        normalized_success = {
            key: _finite(success[key], "{}.success.{}".format(label, key), positive=True)
            for key in ("goal_tolerance_m", "yaw_tolerance_rad", "stopped_speed_max_mps")
        }
        evaluator = raw["evaluator_only"]
        _exact(evaluator, ("feasible", "reason"), label + ".evaluator_only")
        if not isinstance(evaluator["feasible"], bool) or not isinstance(evaluator["reason"], str):
            raise V2SceneError("{}.evaluator_only is invalid".format(label))
        if tuple(raw["policy_forbidden_fields"]) != POLICY_FORBIDDEN_FIELDS:
            raise V2SceneError("{}.policy_forbidden_fields drifted".format(label))

        start = _pose(raw["start"], label + ".start")
        goal = _pose(raw["goal"], label + ".goal")
        timeout_s = _finite(raw["timeout_s"], label + ".timeout_s", positive=True)
        distance = math.hypot(goal["x_m"] - start["x_m"], goal["y_m"] - start["y_m"])
        if raw["family"] == "CRUISE" and not 30.0 <= distance <= 60.0:
            raise V2SceneError("CRUISE start-goal distance must be 30--60 m")
        if raw["family"] == "DYNAMIC" and not agents:
            raise V2SceneError("DYNAMIC requires at least one moving agent")
        if raw["family"] != "DYNAMIC" and agents:
            raise V2SceneError("only the DYNAMIC foundation scene may contain moving agents")
        if raw["family"] == "STATIC_DENSE" and not 3 <= len(obstacles) <= 8:
            raise V2SceneError("STATIC_DENSE requires 3--8 obstacles")
        if raw["family"] in ("CORRIDOR", "MANEUVER") and len(obstacles) < 2:
            raise V2SceneError("{} requires enclosing obstacles".format(raw["family"]))

        scene = copy.deepcopy(dict(raw))
        scene.update(
            start=start, goal=goal, timeout_s=timeout_s,
            static_obstacles=obstacles, dynamic_agents=agents,
            randomization=normalized_randomization, success=normalized_success,
        )
        scene["layout"]["reference_centerline"] = normalized_centerline
        normalized.append(scene)
    families = tuple(scene["family"] for scene in normalized)
    if set(families) != set(SCENE_FAMILIES):
        raise V2SceneError("manifest must cover exactly all five foundation families")
    result = copy.deepcopy(dict(manifest))
    result["robot"]["footprint"] = normalized_footprint
    result["scenes"] = normalized
    return result


def _signed_sample(scene_id: str, seed: int, token: str) -> float:
    digest = hashlib.sha256(
        "{}:{}:{}".format(scene_id, seed, token).encode("utf-8")
    ).digest()
    unit = (int.from_bytes(digest[:8], "big") + 0.5) / (2 ** 64)
    return 2.0 * unit - 1.0


def compile_v2_scene(scene: Mapping[str, Any], generator: Mapping[str, Any]) -> Dict[str, Any]:
    """Materialize deterministic randomization without exposing evaluator labels."""

    result = copy.deepcopy(dict(scene))
    scene_id, seed = result["scene_id"], result["seed"]
    randomization = result["randomization"]
    for index, obstacle in enumerate(result["static_obstacles"]):
        obstacle["pose"]["x_m"] += randomization["position_jitter_m"] * _signed_sample(
            scene_id, seed, "obstacle:{}:x".format(index)
        )
        obstacle["pose"]["y_m"] += randomization["position_jitter_m"] * _signed_sample(
            scene_id, seed, "obstacle:{}:y".format(index)
        )
        obstacle["pose"]["yaw_rad"] += randomization["yaw_jitter_rad"] * _signed_sample(
            scene_id, seed, "obstacle:{}:yaw".format(index)
        )
    for index, agent in enumerate(result["dynamic_agents"]):
        offset = randomization["agent_time_jitter_s"] * _signed_sample(
            scene_id, seed, "agent:{}:time".format(index)
        )
        minimum = min(point["time_s"] + offset for point in agent["trajectory"])
        shift = -minimum if minimum < 0.0 else 0.0
        for point in agent["trajectory"]:
            point["time_s"] += offset + shift
    instance = {
        "schema_version": "2.0",
        "generator": dict(generator),
        "scene": result,
    }
    instance["instance_sha256"] = canonical_sha256(instance)
    return instance


def _pose_text(pose: Mapping[str, float], z_m: float) -> str:
    return "{:.9f} {:.9f} {:.9f} 0 0 {:.9f}".format(
        pose["x_m"], pose["y_m"], z_m, pose["yaw_rad"]
    )


def _box_model(world: ET.Element, item: Mapping[str, Any], static: bool) -> ET.Element:
    name = item.get("obstacle_id", item.get("agent_id"))
    size = item["size_m"]
    model = ET.SubElement(world, "model", {"name": name})
    # Trajectory actors are pose-driven static collision objects.  Gazebo's
    # dynamic/kinematic body path can race model insertion when another model
    # is spawned while SetWorldPose runs; a static body remains collision- and
    # Laser-visible while avoiding that unsafe physics mutation path.
    ET.SubElement(model, "static").text = "true"
    pose = item.get("pose") or item["trajectory"][0]
    ET.SubElement(model, "pose").text = _pose_text(pose, size[2] / 2.0)
    link = ET.SubElement(model, "link", {"name": "link"})
    for tag in ("collision", "visual"):
        element = ET.SubElement(link, tag, {"name": tag})
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = "{:.6f} {:.6f} {:.6f}".format(*size)
    return model


def _indent_xml(element: ET.Element, level: int = 0) -> None:
    """Python 3.8-compatible deterministic XML indentation."""

    indentation = "\n" + level * "  "
    child_indentation = "\n" + (level + 1) * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_indentation
        for child in element:
            _indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = indentation
    if level and (not element.tail or not element.tail.strip()):
        element.tail = indentation


def render_v2_scene_sdf(instance: Mapping[str, Any]) -> str:
    """Render a deterministic Gazebo Classic SDF world from one instance."""

    scene = instance["scene"]
    sdf = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(sdf, "world", {"name": scene["scene_id"]})
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    physics = ET.SubElement(world, "physics", {"name": "v2_deterministic", "type": "ode"})
    ET.SubElement(physics, "max_step_size").text = "0.002"
    ET.SubElement(physics, "real_time_update_rate").text = "500"
    for uri in ("model://sun", "model://ground_plane"):
        include = ET.SubElement(world, "include")
        ET.SubElement(include, "uri").text = uri
    for obstacle in scene["static_obstacles"]:
        _box_model(world, obstacle, True)
    for agent in scene["dynamic_agents"]:
        model = _box_model(world, agent, False)
        plugin = ET.SubElement(
            model, "plugin",
            {"name": "{}_trajectory".format(agent["agent_id"]),
             "filename": "libv2_trajectory_actor_plugin.so"},
        )
        ET.SubElement(plugin, "loop").text = "true" if agent["loop"] else "false"
        ET.SubElement(plugin, "z").text = "{:.6f}".format(agent["size_m"][2] / 2.0)
        for point in agent["trajectory"]:
            waypoint = ET.SubElement(plugin, "waypoint")
            ET.SubElement(waypoint, "time").text = "{:.9f}".format(point["time_s"])
            ET.SubElement(waypoint, "x").text = "{:.9f}".format(point["x_m"])
            ET.SubElement(waypoint, "y").text = "{:.9f}".format(point["y_m"])
            ET.SubElement(waypoint, "yaw").text = "{:.9f}".format(point["yaw_rad"])
    _indent_xml(sdf)
    return '<?xml version="1.0"?>\n' + ET.tostring(sdf, encoding="unicode") + "\n"


def compile_v2_manifest(
    manifest: Mapping[str, Any], workspace_root: Any
) -> List[Dict[str, Any]]:
    validated = validate_v2_scene_manifest(manifest, workspace_root)
    return [compile_v2_scene(scene, validated["generator"])
            for scene in validated["scenes"]]


def load_v2_scene_manifest(path: Any, workspace_root: Optional[Any] = None) -> Dict[str, Any]:
    source = Path(path)
    try:
        data = yaml.safe_load(source.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise V2SceneError("cannot load V2 scene manifest: {}".format(exc))
    root = Path(workspace_root) if workspace_root is not None else source.resolve().parents[4]
    return validate_v2_scene_manifest(data, root)
