"""V3-01 crossing reactive-actor qualification primitives.

The module is ROS-free.  It deterministically materializes the preregistered
schedule and Gazebo worlds, validates frozen bindings, and replays raw CSV
traces without consulting runtime-only labels.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import stat
from typing import Any, Dict, Iterable, List, Mapping, Sequence, Tuple
import xml.etree.ElementTree as ET

import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/"
    "v3_01_reactive_actor_qualification_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_01_reactive_actor_qualification_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/"
    "v3_01_reactive_actor_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/data_qualification/v3_01")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_01_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_01_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_01_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_01_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_01_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_01_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_01_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_01_qualification_assessment.yaml"
EXECUTION_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_01_execution_freeze.sha256"

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path("docs/thesis_experiment/CURRENT_V3_00_FOUNDATION_HANDOFF.md"),
    Path("artifacts/v3/foundation/v3_00/v3_00_terminal_freeze.sha256"),
    Path("src/simulation/m2_gazebo/package.xml"),
    Path("src/simulation/m2_gazebo/CMakeLists.txt"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/trajectory_interpolation.h"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path("src/simulation/m2_gazebo/tests/test_ackermann_kinematics.cpp"),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v3_01_qualification.py"
    ),
    Path("src/tools/thesis_experiment/CMakeLists.txt"),
    Path("src/tools/thesis_experiment/scripts/build_v3_01_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_01_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_01_qualification.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_01_qualification.py"),
)

BINARY_RESOURCES = (
    Path("devel/lib/libv2_trajectory_actor_plugin.so"),
    Path("devel/lib/libv3_reactive_actor_plugin.so"),
    Path("devel/lib/libv3_episode_recorder_plugin.so"),
)


class V301Error(RuntimeError):
    """Raised when a V3-01 invariant fails closed."""


class _UniqueLoader(yaml.SafeLoader):
    pass


def _construct_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise V301Error("duplicate YAML key: {!r}".format(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


_UniqueLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _construct_mapping
)


def require(condition: bool, message: str) -> None:
    if not condition:
        raise V301Error(message)


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value: Any) -> bytes:
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, default_flow_style=False
    ).encode("utf-8")


def canonical_sha(value: Any) -> str:
    return sha256_bytes(
        json.dumps(
            value,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
            allow_nan=False,
        ).encode("utf-8")
    )


def read_file_once(path: Path, *, allow_hard_links: bool = False) -> bytes:
    resolved = Path(path).resolve(strict=True)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    flags |= getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(str(resolved), flags)
    try:
        before = os.fstat(descriptor)
        require(stat.S_ISREG(before.st_mode), "not regular: {}".format(path))
        if not allow_hard_links:
            require(before.st_nlink == 1, "hard link forbidden: {}".format(path))
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        after = os.fstat(descriptor)
        fields = (
            "st_dev",
            "st_ino",
            "st_mode",
            "st_nlink",
            "st_size",
            "st_mtime_ns",
            "st_ctime_ns",
        )
        require(
            tuple(getattr(before, key) for key in fields)
            == tuple(getattr(after, key) for key in fields),
            "file changed during read: {}".format(path),
        )
        payload = b"".join(chunks)
        require(len(payload) == before.st_size, "short read: {}".format(path))
        return payload
    finally:
        os.close(descriptor)


def load_yaml_path(path: Path) -> Mapping[str, Any]:
    payload = read_file_once(path)
    try:
        result = yaml.load(payload.decode("utf-8"), Loader=_UniqueLoader)
    except (UnicodeError, yaml.YAMLError) as exc:
        raise V301Error("cannot parse {}: {}".format(path, exc)) from exc
    require(type(result) is dict, "mapping required: {}".format(path))
    return result


def load_yaml_relative(workspace: Path, relative: Path) -> Mapping[str, Any]:
    return load_yaml_path(workspace / relative)


def receipt(workspace: Path, relative: Path) -> Dict[str, Any]:
    payload = read_file_once(workspace / relative)
    return {
        "path": relative.as_posix(),
        "sha256": sha256_bytes(payload),
        "size_bytes": len(payload),
    }


def absolute_receipt(path: Path) -> Dict[str, Any]:
    payload = read_file_once(path, allow_hard_links=True)
    return {
        "path": str(path.resolve()),
        "sha256": sha256_bytes(payload),
        "size_bytes": len(payload),
    }


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["schema_version"] == "1.0", "contract schema differs")
    require(contract["stage"] == "V3-01", "contract stage differs")
    require(
        contract["status"] == "preregistered_before_simulation",
        "contract status differs",
    )
    scope = contract["scope"]
    require(
        scope == {
            "scene_family": "crossing",
            "qualification_only": True,
            "formal_performance_result": False,
            "planner_evaluation": False,
            "training": False,
        },
        "scope differs",
    )
    authority = contract["authority_boundary"]
    require(
        authority["offline_implementation_authorized"] is True
        and authority[
            "simulation_requires_separate_release_and_authorization"
        ] is True
        and authority["ros_required"] is False
        and authority["move_base_authorized"] is False
        and authority["training_authorized"] is False
        and authority["real_vehicle_authorized"] is False
        and authority["stable_workspace_modification_allowed"] is False,
        "authority boundary differs",
    )
    unit = contract["experimental_unit"]
    require(
        unit["policies"] == ["cautious", "assertive", "hesitant"],
        "policy roster differs",
    )
    require(
        unit["fresh_seeds"] == [6101, 6102, 6103],
        "seed roster differs",
    )
    require(
        unit["schedule_rows"] == 36
        and unit["evidence_budget_units"] == 36
        and unit["attempts_per_identity"] == 1,
        "budget differs",
    )
    require(
        unit["retry_allowed"] is False
        and unit["resume_allowed"] is False
        and unit["failed_unit_recyclable"] is False,
        "retry boundary differs",
    )
    speeds = [
        float(item["speed_mps"]) for item in unit["ego_interventions"]
    ]
    require(speeds == [0.6, 0.8, 1.0, 1.2], "intervention grid differs")
    require(
        preregistration["status"]
        == "frozen_before_any_v3_01_gazebo_execution",
        "preregistration status differs",
    )
    require(
        preregistration["design"]["exact_row_count"] == 36
        and preregistration["budget"]["authorized_units_after_separate_release"]
        == 36
        and preregistration["budget"]["consumed_before_release"] == 0,
        "preregistered budget differs",
    )


def exact_schedule(contract: Mapping[str, Any]) -> Dict[str, Any]:
    unit = contract["experimental_unit"]
    rows: List[Dict[str, Any]] = []
    sequence = 0
    for seed in unit["fresh_seeds"]:
        for policy in unit["policies"]:
            for candidate in unit["ego_interventions"]:
                sequence += 1
                candidate_id = candidate["candidate_id"]
                rows.append(
                    {
                        "sequence": sequence,
                        "unit_id": "v3_01_s{}_{}_{}".format(
                            seed, policy, candidate_id
                        ),
                        "seed": seed,
                        "actor_policy_evaluator_only": policy,
                        "candidate_id_evaluator_only": candidate_id,
                        "ego_speed_mps": float(candidate["speed_mps"]),
                        "attempt": 1,
                        "propensity": 0.25,
                    }
                )
    require(len(rows) == 36, "schedule row count differs")
    require(
        len({row["unit_id"] for row in rows}) == len(rows),
        "unit IDs are not unique",
    )
    return {
        "schema_version": "1.0",
        "stage": "V3-01",
        "schedule_id": "v3_scope_v3_01_crossing_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "order": "seed_then_policy_then_increasing_candidate_speed",
        "row_count": len(rows),
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }


def _pose_text(x: float, y: float, z: float, yaw: float) -> str:
    return "{:.9f} {:.9f} {:.9f} 0 0 {:.9f}".format(x, y, z, yaw)


def _box_model(
    world: ET.Element,
    name: str,
    size: Sequence[float],
    pose: Sequence[float],
) -> ET.Element:
    model = ET.SubElement(world, "model", {"name": name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = _pose_text(
        pose[0], pose[1], size[2] / 2.0, pose[2]
    )
    link = ET.SubElement(model, "link", {"name": "body"})
    for tag in ("collision", "visual"):
        item = ET.SubElement(link, tag, {"name": tag})
        geometry = ET.SubElement(item, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = "{:.6f} {:.6f} {:.6f}".format(
            *size
        )
    return model


def _indent(element: ET.Element, level: int = 0) -> None:
    indentation = "\n" + level * "  "
    child_indent = "\n" + (level + 1) * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_indent
        for child in element:
            _indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = indentation
    if level and (not element.tail or not element.tail.strip()):
        element.tail = indentation


def render_world(
    contract: Mapping[str, Any],
    row: Mapping[str, Any],
    output_csv: Path,
    done_marker: Path,
) -> str:
    scene = contract["scene"]
    ego = scene["ego"]
    actor = scene["actor"]
    sdf = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(sdf, "world", {"name": row["unit_id"]})
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    physics = ET.SubElement(
        world, "physics", {"name": "v3_01_deterministic", "type": "ode"}
    )
    ET.SubElement(physics, "max_step_size").text = "{:.9f}".format(
        float(scene["max_step_size_s"])
    )
    ET.SubElement(physics, "real_time_update_rate").text = str(
        int(scene["real_time_update_rate_hz"])
    )
    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    ground_link = ET.SubElement(ground, "link", {"name": "link"})
    collision = ET.SubElement(ground_link, "collision", {"name": "collision"})
    geometry = ET.SubElement(collision, "geometry")
    plane = ET.SubElement(geometry, "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "30 30"

    ego_model = _box_model(
        world,
        ego["model_name"],
        ego["footprint_m"],
        ego["initial_pose_m_rad"],
    )
    ego_plugin = ET.SubElement(
        ego_model,
        "plugin",
        {
            "name": "ego_open_loop_intervention",
            "filename": "libv2_trajectory_actor_plugin.so",
        },
    )
    ET.SubElement(ego_plugin, "loop").text = "false"
    ET.SubElement(ego_plugin, "z").text = "{:.6f}".format(
        float(ego["footprint_m"][2]) / 2.0
    )
    duration = 6.0 / float(row["ego_speed_mps"])
    for time_s, pose in (
        (0.0, ego["initial_pose_m_rad"]),
        (duration, ego["endpoint_m_rad"]),
    ):
        waypoint = ET.SubElement(ego_plugin, "waypoint")
        ET.SubElement(waypoint, "time").text = "{:.9f}".format(time_s)
        ET.SubElement(waypoint, "x").text = "{:.9f}".format(float(pose[0]))
        ET.SubElement(waypoint, "y").text = "{:.9f}".format(float(pose[1]))
        ET.SubElement(waypoint, "yaw").text = "{:.9f}".format(float(pose[2]))

    actor_model = _box_model(
        world,
        actor["model_name"],
        actor["size_m"],
        actor["initial_pose_m_rad"],
    )
    actor_plugin = ET.SubElement(
        actor_model,
        "plugin",
        {
            "name": "candidate_blind_reactive_actor",
            "filename": "libv3_reactive_actor_plugin.so",
        },
    )
    ET.SubElement(actor_plugin, "ego_model").text = ego["model_name"]
    ET.SubElement(actor_plugin, "policy").text = row[
        "actor_policy_evaluator_only"
    ]
    ET.SubElement(actor_plugin, "seed").text = str(row["seed"])
    ET.SubElement(actor_plugin, "start").text = "0 -3"
    ET.SubElement(actor_plugin, "end").text = "0 3"
    ET.SubElement(actor_plugin, "conflict").text = "0 0"
    ET.SubElement(actor_plugin, "base_speed").text = "{:.9f}".format(
        float(actor["base_speed_mps"])
    )
    ET.SubElement(actor_plugin, "response_distance").text = "{:.9f}".format(
        float(actor["response_distance_m"])
    )
    ET.SubElement(
        actor_plugin, "response_time_margin"
    ).text = "{:.9f}".format(float(actor["response_time_margin_s"]))
    ET.SubElement(actor_plugin, "z").text = "{:.6f}".format(
        float(actor["size_m"][2]) / 2.0
    )

    recorder = ET.SubElement(
        world,
        "plugin",
        {
            "name": "v3_01_pose_recorder",
            "filename": "libv3_episode_recorder_plugin.so",
        },
    )
    ET.SubElement(recorder, "ego_model").text = ego["model_name"]
    ET.SubElement(recorder, "actor_model").text = actor["model_name"]
    ET.SubElement(recorder, "output_csv").text = str(output_csv.resolve())
    ET.SubElement(recorder, "done_marker").text = str(done_marker.resolve())
    ET.SubElement(recorder, "duration").text = "{:.9f}".format(
        float(scene["duration_s"])
    )
    ET.SubElement(recorder, "sample_period").text = "{:.9f}".format(
        float(scene["sample_period_s"])
    )
    _indent(sdf)
    return '<?xml version="1.0"?>\n' + ET.tostring(
        sdf, encoding="unicode"
    ) + "\n"


def _finite_float(value: str, label: str) -> float:
    try:
        result = float(value)
    except ValueError as exc:
        raise V301Error("{} is not numeric".format(label)) from exc
    require(math.isfinite(result), "{} is not finite".format(label))
    return result


def load_trace(path: Path) -> List[Dict[str, float]]:
    expected = [
        "sim_time_s",
        "ego_x_m",
        "ego_y_m",
        "ego_yaw_rad",
        "actor_x_m",
        "actor_y_m",
        "actor_yaw_rad",
    ]
    rows: List[Dict[str, float]] = []
    with Path(path).open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        require(reader.fieldnames == expected, "CSV header differs")
        for index, raw in enumerate(reader):
            rows.append(
                {
                    key: _finite_float(raw[key], "{}[{}].{}".format(
                        path, index, key
                    ))
                    for key in expected
                }
            )
    require(len(rows) >= 150, "trace has too few samples: {}".format(path))
    times = [row["sim_time_s"] for row in rows]
    require(
        all(after > before for before, after in zip(times, times[1:])),
        "trace time is not strictly increasing",
    )
    require(times[-1] >= 7.95, "trace ended early")
    return rows


def interpolate(rows: Sequence[Mapping[str, float]], field: str, time_s: float) -> float:
    require(rows[0]["sim_time_s"] <= time_s <= rows[-1]["sim_time_s"], "query outside trace")
    for before, after in zip(rows, rows[1:]):
        if after["sim_time_s"] >= time_s:
            span = after["sim_time_s"] - before["sim_time_s"]
            ratio = (time_s - before["sim_time_s"]) / span
            return before[field] + ratio * (after[field] - before[field])
    return rows[-1][field]


def crossing_time(
    rows: Sequence[Mapping[str, float]], field: str, crossing: float = 0.0
) -> float:
    for before, after in zip(rows, rows[1:]):
        a = before[field] - crossing
        b = after[field] - crossing
        if a <= 0.0 <= b and after[field] > before[field]:
            ratio = -a / (b - a)
            return before["sim_time_s"] + ratio * (
                after["sim_time_s"] - before["sim_time_s"]
            )
    return math.inf


def trace_metrics(rows: Sequence[Mapping[str, float]]) -> Dict[str, Any]:
    actor_4s = interpolate(rows, "actor_y_m", 4.0)
    actor_005s = interpolate(rows, "actor_y_m", 0.05)
    ego_time = crossing_time(rows, "ego_x_m")
    actor_time = crossing_time(rows, "actor_y_m")
    samples = [
        round(interpolate(rows, "actor_y_m", stamp), 3)
        for stamp in (1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0)
    ]
    return {
        "initial_ego_xy_m": [rows[0]["ego_x_m"], rows[0]["ego_y_m"]],
        "initial_actor_xy_m": [rows[0]["actor_x_m"], rows[0]["actor_y_m"]],
        "actor_y_at_0_05s_m": actor_005s,
        "actor_y_at_4s_m": actor_4s,
        "ego_crossing_time_s": ego_time,
        "actor_crossing_time_s": actor_time,
        "response_signature": canonical_sha(samples),
        "response_signature_samples_m": samples,
    }


def check_receipts(workspace: Path, receipts: Iterable[Mapping[str, Any]]) -> None:
    for item in receipts:
        path = workspace / item["path"]
        payload = read_file_once(path)
        require(
            sha256_bytes(payload) == item["sha256"]
            and len(payload) == item["size_bytes"],
            "receipt differs: {}".format(item["path"]),
        )
