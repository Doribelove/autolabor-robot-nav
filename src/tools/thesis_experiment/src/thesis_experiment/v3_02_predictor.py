"""V3-02 candidate-conditioned predictor data and model primitives."""

import importlib.util
import math
from pathlib import Path
from typing import Any, Dict, Mapping, Sequence
import xml.etree.ElementTree as ET


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
R2_PATH = (
    WORKSPACE / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_r2.py"
)
_SPEC = importlib.util.spec_from_file_location("v3_01_r2_base", R2_PATH)
R2 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(R2)
BASE = R2.R1.BASE

V301Error = R2.V301Error
require = R2.require
sha256_bytes = R2.sha256_bytes
yaml_bytes = R2.yaml_bytes
canonical_sha = R2.canonical_sha
read_file_once = R2.read_file_once
load_yaml_path = R2.load_yaml_path
load_yaml_relative = R2.load_yaml_relative
receipt = R2.receipt
absolute_receipt = R2.absolute_receipt
load_trace = R2.load_trace
interpolate = BASE.interpolate
crossing_time = BASE.crossing_time
check_receipts = R2.check_receipts

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_02_a1_predictor_data_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_02_a1_predictor_data_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_02_a1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/prediction/v3_02_a1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_02_a1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_02_a1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_02_a1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_02_a1_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_02_a1_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_02_a1_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_02_a1_attempt_journal.jsonl"
MODEL_RELATIVE = ROOT_RELATIVE / "model/v3_02_candidate_predictor.yaml"
CALIBRATION_RELATIVE = ROOT_RELATIVE / "model/v3_02_calibration_report.yaml"
MODEL_FREEZE_RELATIVE = ROOT_RELATIVE / "model/v3_02_model_freeze.sha256"
INDEPENDENT_MODEL_REVIEW_RELATIVE = (
    ROOT_RELATIVE / "model/v3_02_independent_model_review.yaml"
)
TRACE_METRICS_NEEDS_ROW = True

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path("docs/thesis_experiment/CURRENT_V3_01_REACTIVE_ACTOR_HANDOFF.md"),
    Path(
        "artifacts/v3/data_qualification/v3_01_r2/"
        "v3_01_r2_terminal_freeze.sha256"
    ),
    Path("src/simulation/m2_gazebo/package.xml"),
    Path("src/simulation/m2_gazebo/CMakeLists.txt"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path(
        "src/tools/thesis_experiment/src/thesis_experiment/"
        "v3_01_qualification.py"
    ),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_01_r1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_01_r2.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_02_a1_data.py"),
    Path("src/tools/thesis_experiment/scripts/fit_v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_model.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_02_predictor.py"),
)
BINARY_RESOURCES = R2.BINARY_RESOURCES


def validate_contract(
    contract: Mapping[str, Any], preregistration: Mapping[str, Any]
) -> None:
    require(contract["stage"] == "V3-02-A1", "stage differs")
    require(
        contract["status"] == "preregistered_before_simulation",
        "status differs",
    )
    unit = contract["experimental_unit"]
    require(
        unit["policies"] == ["cautious", "assertive", "hesitant"],
        "policies differ",
    )
    require(
        len(unit["scenario_blocks"]) == 9
        and sum(row["role"] == "training" for row in unit["scenario_blocks"])
        == 6
        and sum(row["role"] == "calibration" for row in unit["scenario_blocks"])
        == 3,
        "scenario roster differs",
    )
    require(
        unit["schedule_rows"] == 108
        and unit["evidence_budget_units"] == 108
        and unit["attempts_per_identity"] == 1
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "budget differs",
    )
    require(
        [float(row["speed_mps"]) for row in unit["ego_interventions"]]
        == [0.6, 0.8, 1.0, 1.2],
        "candidate roster differs",
    )
    design = contract["predictor_design"]
    require(
        design["mode_heads"] == ["cautious", "assertive", "hesitant"]
        and design["model"] == "per_mode_per_horizon_ridge_regression"
        and float(design["conformal_alpha"]) == 0.10,
        "predictor design differs",
    )
    authority = contract["authority_boundary"]
    require(
        authority["held_out_test_access_authorized"] is False
        and authority["held_out_test_seeds_reserved"] == [6221, 6222, 6223]
        and authority["move_base_authorized"] is False
        and authority["real_vehicle_authorized"] is False,
        "authority differs",
    )
    require(
        preregistration["stage"] == "V3-02-A1"
        and preregistration["status"]
        == "frozen_before_any_v3_02_gazebo_execution"
        and preregistration["schedule"]["exact_row_count"] == 108
        and preregistration["held_out_firewall"]["test_outcome_access_allowed"]
        is False,
        "preregistration differs",
    )


def exact_schedule(contract: Mapping[str, Any]) -> Dict[str, Any]:
    unit = contract["experimental_unit"]
    rows = []
    sequence = 0
    for block in unit["scenario_blocks"]:
        for policy in unit["policies"]:
            for candidate in unit["ego_interventions"]:
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "unit_id": "v3_02_a1_{}_{}_{}".format(
                            block["block_id"], policy, candidate["candidate_id"]
                        ),
                        "block_id": block["block_id"],
                        "role_evaluator_only": block["role"],
                        "seed": block["seed"],
                        "angle_deg": float(block["angle_deg"]),
                        "actor_start_distance_m": float(
                            block["actor_start_distance_m"]
                        ),
                        "actor_base_speed_mps": float(
                            block["actor_base_speed_mps"]
                        ),
                        "actor_policy_evaluator_only": policy,
                        "candidate_id_evaluator_only": candidate["candidate_id"],
                        "ego_speed_mps": float(candidate["speed_mps"]),
                        "attempt": 1,
                        "propensity": 0.25,
                    }
                )
    require(len(rows) == 108, "schedule count differs")
    return {
        "schema_version": "1.0",
        "stage": "V3-02-A1",
        "schedule_id": "v3_scope_v3_02_a1_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 108,
        "training_rows": 72,
        "calibration_rows": 36,
        "held_out_test_rows": 0,
        "reserved_held_out_seeds_not_materialized": [6221, 6222, 6223],
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }


def actor_direction(row: Mapping[str, Any]) -> Sequence[float]:
    angle = math.radians(float(row["angle_deg"]))
    return (math.cos(angle), math.sin(angle))


def ego_waypoints(
    contract: Mapping[str, Any], row: Mapping[str, Any]
) -> Sequence[Sequence[Any]]:
    """Return the immutable ego intervention as (time, pose) waypoints."""
    if row.get("ego_waypoints_override"):
        return tuple(
            (
                float(item[0]),
                [float(value) for value in item[1]],
            )
            for item in row["ego_waypoints_override"]
        )
    scene = contract["scene"]
    ego = scene["ego"]
    common_prefix = ego.get("common_prefix")
    if not common_prefix:
        ego_duration = 6.0 / float(row["ego_speed_mps"])
        return (
            (0.0, ego["initial_pose_m_rad"]),
            (ego_duration, ego["endpoint_pose_m_rad"]),
        )

    prefix_time = float(common_prefix["duration_s"])
    prefix_speed = float(common_prefix["speed_mps"])
    prefix_pose = [
        float(ego["initial_pose_m_rad"][0]) + prefix_speed * prefix_time,
        float(ego["initial_pose_m_rad"][1]),
        float(ego["initial_pose_m_rad"][2]),
    ]
    feasible = ego.get("feasible_branch")
    if not feasible:
        branch_distance = float(ego["endpoint_pose_m_rad"][0]) - prefix_pose[0]
        return (
            (0.0, ego["initial_pose_m_rad"]),
            (prefix_time, prefix_pose),
            (
                prefix_time + branch_distance / float(row["ego_speed_mps"]),
                ego["endpoint_pose_m_rad"],
            ),
        )

    target_speed = float(row["ego_speed_mps"])
    initial_speed = float(feasible["initial_speed_mps"])
    require(
        abs(initial_speed - prefix_speed) <= 1e-12,
        "feasible branch initial speed differs from common prefix",
    )
    require(target_speed >= 0.0, "reverse intervention is not allowed")
    if target_speed > initial_speed:
        acceleration = float(feasible["maximum_acceleration_mps2"])
    elif target_speed < initial_speed:
        acceleration = -float(feasible["maximum_deceleration_mps2"])
    else:
        acceleration = 0.0
    ramp_duration = (
        abs(target_speed - initial_speed) / abs(acceleration)
        if acceleration
        else 0.0
    )
    step = float(feasible["ramp_waypoint_period_s"])
    require(step > 0.0, "ramp waypoint period must be positive")
    points = [(0.0, ego["initial_pose_m_rad"]), (prefix_time, prefix_pose)]
    elapsed = step
    while elapsed < ramp_duration - 1e-12:
        x = (
            prefix_pose[0]
            + initial_speed * elapsed
            + 0.5 * acceleration * elapsed * elapsed
        )
        points.append(
            (
                prefix_time + elapsed,
                [x, prefix_pose[1], prefix_pose[2]],
            )
        )
        elapsed += step
    ramp_x = (
        prefix_pose[0]
        + initial_speed * ramp_duration
        + 0.5 * acceleration * ramp_duration * ramp_duration
    )
    if ramp_duration > 0.0:
        points.append(
            (
                prefix_time + ramp_duration,
                [ramp_x, prefix_pose[1], prefix_pose[2]],
            )
        )
    if target_speed > 0.0:
        remaining = float(ego["endpoint_pose_m_rad"][0]) - ramp_x
        require(remaining > 0.0, "feasible branch overshoots endpoint")
        points.append(
            (
                prefix_time + ramp_duration + remaining / target_speed,
                ego["endpoint_pose_m_rad"],
            )
        )
    else:
        terminal_time = float(scene["duration_s"])
        require(
            terminal_time > prefix_time + ramp_duration,
            "stop terminal hold is not positive",
        )
        points.append(
            (
                terminal_time,
                [ramp_x, prefix_pose[1], prefix_pose[2]],
            )
        )
    return tuple(points)


def render_world(
    contract: Mapping[str, Any],
    row: Mapping[str, Any],
    output_csv: Path,
    done_marker: Path,
) -> str:
    scene = contract["scene"]
    ego = scene["ego"]
    actor = scene["actor"]
    dx, dy = actor_direction(row)
    start_distance = float(row["actor_start_distance_m"])
    end_distance = float(actor["endpoint_after_conflict_m"])
    actor_start = (-dx * start_distance, -dy * start_distance)
    actor_end = (dx * end_distance, dy * end_distance)
    yaw = math.atan2(dy, dx)

    sdf = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(sdf, "world", {"name": row["unit_id"]})
    ET.SubElement(world, "gravity").text = "0 0 -9.81"
    physics = ET.SubElement(
        world, "physics", {"name": "v3_02_deterministic", "type": "ode"}
    )
    ET.SubElement(physics, "max_step_size").text = "{:.9f}".format(
        float(scene["max_step_size_s"])
    )
    ET.SubElement(physics, "real_time_update_rate").text = str(
        int(scene["real_time_update_rate_hz"])
    )
    ground = ET.SubElement(world, "model", {"name": "ground_plane"})
    ET.SubElement(ground, "static").text = "true"
    link = ET.SubElement(ground, "link", {"name": "link"})
    collision = ET.SubElement(link, "collision", {"name": "collision"})
    geometry = ET.SubElement(collision, "geometry")
    plane = ET.SubElement(geometry, "plane")
    ET.SubElement(plane, "normal").text = "0 0 1"
    ET.SubElement(plane, "size").text = "30 30"

    ego_model = BASE._box_model(
        world, ego["model_name"], ego["footprint_m"], ego["initial_pose_m_rad"]
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
    for time_s, pose in ego_waypoints(contract, row):
        waypoint = ET.SubElement(ego_plugin, "waypoint")
        ET.SubElement(waypoint, "time").text = "{:.9f}".format(time_s)
        ET.SubElement(waypoint, "x").text = "{:.9f}".format(float(pose[0]))
        ET.SubElement(waypoint, "y").text = "{:.9f}".format(float(pose[1]))
        ET.SubElement(waypoint, "yaw").text = "{:.9f}".format(float(pose[2]))

    actor_model = BASE._box_model(
        world,
        actor["model_name"],
        actor["size_m"],
        (actor_start[0], actor_start[1], yaw),
    )
    plugin = ET.SubElement(
        actor_model,
        "plugin",
        {
            "name": "candidate_blind_reactive_actor",
            "filename": "libv3_reactive_actor_plugin.so",
        },
    )
    ET.SubElement(plugin, "ego_model").text = ego["model_name"]
    ET.SubElement(plugin, "policy").text = row["actor_policy_evaluator_only"]
    ET.SubElement(plugin, "seed").text = str(row["seed"])
    ET.SubElement(plugin, "start").text = "{:.9f} {:.9f}".format(*actor_start)
    ET.SubElement(plugin, "end").text = "{:.9f} {:.9f}".format(*actor_end)
    ET.SubElement(plugin, "conflict").text = "0 0"
    ET.SubElement(plugin, "base_speed").text = "{:.9f}".format(
        float(row["actor_base_speed_mps"])
    )
    ET.SubElement(plugin, "response_distance").text = "{:.9f}".format(
        float(actor["response_distance_m"])
    )
    ET.SubElement(plugin, "response_time_margin").text = "{:.9f}".format(
        float(actor["response_time_margin_s"])
    )
    ET.SubElement(plugin, "z").text = "{:.6f}".format(
        float(actor["size_m"][2]) / 2.0
    )
    recorder = ET.SubElement(
        world,
        "plugin",
        {
            "name": "v3_02_pose_recorder",
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
    BASE._indent(sdf)
    return '<?xml version="1.0"?>\n' + ET.tostring(
        sdf, encoding="unicode"
    ) + "\n"


def _actor_coordinate(
    rows: Sequence[Mapping[str, float]], row: Mapping[str, Any], time_s: float
) -> float:
    dx, dy = actor_direction(row)
    return (
        interpolate(rows, "actor_x_m", time_s) * dx
        + interpolate(rows, "actor_y_m", time_s) * dy
    )


def feature_vector(actor_coordinate: float, actor_speed: float, ego_speed: float):
    return [
        1.0,
        actor_coordinate,
        actor_speed,
        ego_speed,
        ego_speed * ego_speed,
        actor_coordinate * ego_speed,
        actor_speed * ego_speed,
    ]


def trace_metrics(
    rows: Sequence[Mapping[str, float]],
    schedule_row: Mapping[str, Any] = None,
) -> Dict[str, Any]:
    require(schedule_row is not None, "V3-02 trace metrics need schedule row")
    observation_time = 0.5
    coordinate = _actor_coordinate(rows, schedule_row, observation_time)
    before = _actor_coordinate(rows, schedule_row, 0.3)
    observed_speed = (coordinate - before) / 0.2
    targets = [
        _actor_coordinate(rows, schedule_row, stamp)
        for stamp in (1.5, 2.5, 3.5, 4.5)
    ]
    return {
        "initial_ego_xy_m": [rows[0]["ego_x_m"], rows[0]["ego_y_m"]],
        "actor_conflict_coordinate_at_observation_m": coordinate,
        "actor_observed_speed_mps": observed_speed,
        "candidate_ego_speed_mps": float(schedule_row["ego_speed_mps"]),
        "feature_vector": feature_vector(
            coordinate, observed_speed, float(schedule_row["ego_speed_mps"])
        ),
        "target_actor_conflict_coordinates_m": targets,
        "ego_crossing_time_s": crossing_time(rows, "ego_x_m"),
    }
