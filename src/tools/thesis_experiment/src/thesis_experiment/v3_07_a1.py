"""V3-07-A1 cloned spatial-intervention qualification bindings."""

import importlib.util
from pathlib import Path
from typing import Any, Mapping

W = Path("/home/robot/robot_ws_base_rl")
BASE_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"
SPATIAL_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_07_spatial_candidates.py"


def _load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


BASE = _load(BASE_PATH, "v3_02_d1_base_for_v3_07_a1")
SPATIAL = _load(SPATIAL_PATH, "v3_07_spatial_for_a1")
for _name in (
    "V301Error",
    "require",
    "sha256_bytes",
    "yaml_bytes",
    "canonical_sha",
    "read_file_once",
    "load_yaml_path",
    "load_yaml_relative",
    "receipt",
    "absolute_receipt",
    "load_trace",
    "interpolate",
    "crossing_time",
    "check_receipts",
    "actor_direction",
    "trace_metrics",
):
    globals()[_name] = getattr(BASE, _name)

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_07_a1_spatial_identifiability_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_07_a1_spatial_identifiability_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_07_a1_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/spatial/v3_07_a1")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_07_a1_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_07_a1_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_07_a1_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_07_a1_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_07_a1_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_07_a1_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_07_a1_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_07_a1_identifiability_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_07_a1_evidence_freeze.sha256"
RESULT_REVIEW_RELATIVE = EXECUTION_RELATIVE / "v3_07_a1_independent_result_review.yaml"
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_07_a1_terminal_freeze.sha256"
TRACE_METRICS_NEEDS_ROW = True

SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path("artifacts/v3/spatial/v3_07_a0/v3_07_a0_terminal_freeze.sha256"),
    Path("config/thesis_experiments/v3/v3_07_a0_spatial_candidate_foundation_contract.yaml"),
    SPATIAL_PATH.relative_to(W),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_a1.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_07_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_07_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_07_a1_identifiability.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_07_a1_identifiability.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_07_a1_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_07_a1.py"),
)
BINARY_RESOURCES = BASE.BINARY_RESOURCES


def validate_contract(contract: Mapping[str, Any], preregistration: Mapping[str, Any]):
    require(
        contract["stage"] == "V3-07-A1"
        and contract["status"] == "preregistered_before_simulation",
        "A1 contract differs",
    )
    unit = contract["experimental_unit"]
    require(
        len(unit["scenario_blocks"]) == 4
        and unit["schedule_rows"] == 36
        and unit["training_rows"] == 0
        and unit["calibration_rows"] == 36
        and unit["branches"]
        == ["straight_v120", "spatial_left_v120", "spatial_right_v120"]
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "A1 population differs",
    )
    require(
        preregistration["status"]
        == "frozen_before_any_v3_07_a1_gazebo_execution"
        and preregistration["schedule"]["exact_rows"] == 36
        and preregistration["failure_policy"][
            "no_predictor_data_stage_if_identifiability_fails"
        ]
        is True,
        "A1 preregistration differs",
    )
    authority = contract["authority_boundary"]
    require(
        authority["held_out_test_seeds_reserved"] == [6551, 6552, 6553]
        and authority["held_out_test_access_authorized"] is False
        and authority["simulation_requires_separate_release_and_authorization"]
        is True
        and authority["training_authorized"] is False
        and authority["real_vehicle_authorized"] is False,
        "A1 authority differs",
    )


def exact_schedule(contract):
    rows = []
    sequence = 0
    branches = contract["experimental_unit"]["branches"]
    for block in contract["experimental_unit"]["scenario_blocks"]:
        for policy_index, policy in enumerate(
            contract["experimental_unit"]["policies"]
        ):
            offset = (int(block["seed"]) + policy_index) % len(branches)
            ordered = branches[offset:] + branches[:offset]
            for branch in ordered:
                sequence += 1
                rows.append(
                    {
                        "sequence": sequence,
                        "unit_id": "v3_07_a1_{}_{}_{}".format(
                            block["block_id"], policy, branch
                        ),
                        "block_id": block["block_id"],
                        "role_evaluator_only": "qualification",
                        "seed": int(block["seed"]),
                        "angle_deg": float(block["angle_deg"]),
                        "actor_start_distance_m": float(
                            block["actor_start_distance_m"]
                        ),
                        "actor_base_speed_mps": float(
                            block["actor_base_speed_mps"]
                        ),
                        "actor_policy_evaluator_only": policy,
                        "candidate_id_evaluator_only": branch,
                        "ego_speed_mps": 1.2,
                        "attempt": 1,
                        "propensity": 1.0 / 3.0,
                    }
                )
    require(len(rows) == 36, "A1 schedule differs")
    return {
        "schema_version": "1.0",
        "stage": "V3-07-A1",
        "schedule_id": "v3_scope_v3_07_a1_exact_schedule_1",
        "status": "frozen_before_execution_release",
        "row_count": 36,
        "training_rows": 0,
        "calibration_rows": 36,
        "held_out_test_rows": 0,
        "reserved_held_out_seeds_not_materialized": [6551, 6552, 6553],
        "authorized_attempts_per_identity": 1,
        "retry_allowed": False,
        "rows": rows,
        "schedule_digest": canonical_sha(rows),
    }


def _straight_branch(contract):
    generator = contract["generator"]
    vehicle = contract["vehicle"]
    dt = float(generator["integration_period_s"])
    count = int(round(4.5 / dt))
    x = float(generator["initial_pose_m_rad"][0])
    speed = float(generator["initial_speed_mps"])
    target = float(generator["target_speed_mps"])
    points = [(0.0, [x, 0.0, 0.0])]
    for index in range(1, count + 1):
        new_speed = min(
            target,
            speed + float(vehicle["maximum_acceleration_mps2"]) * dt,
        )
        x += 0.5 * (speed + new_speed) * dt
        speed = new_speed
        points.append((index * dt, [x, 0.0, 0.0]))
    return points


def ego_waypoints(contract, row):
    spatial_contract = load_yaml_path(
        W
        / "config/thesis_experiments/v3/"
        "v3_07_a0_spatial_candidate_foundation_contract.yaml"
    )
    prefix = [(0.0, [-3.0, 0.0, 0.0]), (0.5, [-2.6, 0.0, 0.0])]
    branch = row["candidate_id_evaluator_only"]
    if branch == "straight_v120":
        relative = _straight_branch(spatial_contract)
    else:
        side = 1.0 if branch == "spatial_left_v120" else -1.0
        candidate = SPATIAL.generate(branch, side, spatial_contract)
        relative = [
            (
                float(point["time_from_start_s"]),
                [
                    float(point["x_m"]),
                    float(point["y_m"]),
                    float(point["yaw_rad"]),
                ],
            )
            for point in candidate["points"]
            if float(point["time_from_start_s"]) <= 4.5 + 1e-12
        ]
    return tuple(prefix + [(0.5 + time_s, pose) for time_s, pose in relative[1:]])


def render_world(contract, row, output_csv, done_marker):
    original = BASE.BASE.ego_waypoints
    BASE.BASE.ego_waypoints = ego_waypoints
    try:
        return BASE.render_world(contract, row, output_csv, done_marker)
    finally:
        BASE.BASE.ego_waypoints = original
