"""V3-07-A3 observed-mode-gated safe-overlap bindings."""

import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_07_a2.py"
spec = importlib.util.spec_from_file_location("v3_07_a2_parent_for_a3", PARENT_PATH)
PARENT = importlib.util.module_from_spec(spec)
spec.loader.exec_module(PARENT)
for name in (
    "V301Error", "require", "sha256_bytes", "yaml_bytes", "canonical_sha",
    "read_file_once", "load_yaml_path", "load_yaml_relative", "receipt",
    "absolute_receipt", "load_trace", "interpolate", "check_receipts",
    "actor_direction",
):
    globals()[name] = getattr(PARENT, name)

CONTRACT_RELATIVE = Path("config/thesis_experiments/v3/v3_07_a3_mode_gated_safe_overlap_contract.yaml")
PREREGISTRATION_RELATIVE = Path("experiments/manifests/v3/v3_07_a3_mode_gated_safe_overlap_preregistration.yaml")
AUTHORIZATION_RELATIVE = Path("experiments/manifests/v3/v3_07_a3_simulation_authorization.yaml")
ROOT_RELATIVE = Path("artifacts/v3/spatial/v3_07_a3")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_07_a3_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_07_a3_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_07_a3_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_07_a3_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_07_a3_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_07_a3_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_07_a3_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_07_a3_safe_overlap_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_07_a3_evidence_freeze.sha256"
RESULT_REVIEW_RELATIVE = EXECUTION_RELATIVE / "v3_07_a3_independent_result_review.yaml"
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_07_a3_terminal_freeze.sha256"
BINARY_RESOURCES = PARENT.BINARY_RESOURCES
SOURCE_RESOURCES = (
    CONTRACT_RELATIVE, PREREGISTRATION_RELATIVE,
    Path("artifacts/v3/spatial/v3_07_a2/v3_07_a2_terminal_freeze.sha256"),
    Path("artifacts/v3/spatial/v3_07_a2/execution/v3_07_a2_identifiability_assessment.yaml"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_spatial_candidates.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_a1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_a2.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_a3.py"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_07_a3_mode_gate.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_07_a3_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_07_a3_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_07_a3_safe_overlap.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_07_a3_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_07_a3.py"),
)


def validate_contract(contract, preregistration):
    require(
        contract["stage"] == "V3-07-A3"
        and contract["status"] == "preregistered_before_simulation"
        and contract["single_factor"]["hidden_policy_runtime_input"] is False,
        "A3 contract differs",
    )
    unit = contract["experimental_unit"]
    require(
        len(unit["scenario_blocks"]) == 6
        and unit["schedule_rows"] == 48
        and unit["exact_probe_units"] == 18
        and unit["exact_stop_units"] == 18
        and unit["exact_conditional_spatial_units"] == 12
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "A3 population differs",
    )
    require(
        preregistration["status"] == "frozen_before_any_v3_07_a3_gazebo_execution"
        and preregistration["runtime_selection"]["hidden_policy_forbidden"] is True
        and preregistration["failure_policy"]["no_predictor_data_stage_if_any_gate_fails"] is True,
        "A3 preregistration differs",
    )
    authority = contract["authority_boundary"]
    require(
        authority["held_out_test_seeds_reserved"] == [6601, 6602, 6603]
        and authority["held_out_test_access_authorized"] is False
        and authority["training_authorized"] is False
        and authority["real_vehicle_authorized"] is False,
        "A3 authority differs",
    )


def exact_schedule(contract):
    rows = []
    sequence = 0
    for block in contract["experimental_unit"]["scenario_blocks"]:
        for policy in contract["experimental_unit"]["policies_evaluator_only"]:
            phases = ["probe", "certified_stop"]
            if policy in ("cautious", "hesitant"):
                phases.append("conditional_spatial_left")
            for phase in phases:
                sequence += 1
                rows.append({
                    "sequence": sequence,
                    "unit_id": "v3_07_a3_{}_{}_{}".format(block["block_id"], policy, phase),
                    "block_id": block["block_id"], "seed": int(block["seed"]),
                    "angle_deg": float(block["angle_deg"]),
                    "actor_start_distance_m": float(block["actor_start_distance_m"]),
                    "actor_base_speed_mps": float(block["actor_base_speed_mps"]),
                    "actor_policy_evaluator_only": policy,
                    "phase": phase, "attempt": 1,
                })
    require(len(rows) == 48, "A3 schedule differs")
    return {
        "schema_version": "1.0", "stage": "V3-07-A3",
        "schedule_id": "v3_scope_v3_07_a3_exact_schedule_1",
        "status": "frozen_before_execution_release", "row_count": 48,
        "training_rows": 0, "calibration_rows": 48,
        "held_out_test_rows": 0,
        "reserved_held_out_seeds_not_materialized": [6601, 6602, 6603],
        "authorized_attempts_per_identity": 1, "retry_allowed": False,
        "rows": rows, "schedule_digest": canonical_sha(rows),
    }


def infer_mode(speed, gate):
    lower = float(gate["cautious_hesitant_threshold_mps"])
    upper = float(gate["hesitant_assertive_threshold_mps"])
    ambiguity = float(gate["ambiguity_half_width_mps"])
    if speed < lower - ambiguity:
        return "cautious"
    if lower + ambiguity < speed < upper - ambiguity:
        return "hesitant"
    if speed > upper + ambiguity:
        return "assertive"
    return "ambiguous"


def _stop_branch(spatial_contract):
    g = spatial_contract["generator"]; v = spatial_contract["vehicle"]
    dt = float(g["integration_period_s"]); count = int(round(7.5 / dt))
    x = float(g["initial_pose_m_rad"][0]); speed = float(g["initial_speed_mps"])
    points = [(0.0, [x, 0.0, 0.0])]
    for index in range(1, count + 1):
        new_speed = max(0.0, speed - float(v["maximum_deceleration_mps2"]) * dt)
        x += 0.5 * (speed + new_speed) * dt; speed = new_speed
        points.append((index * dt, [x, 0.0, 0.0]))
    return points


def ego_waypoints(contract, row):
    spatial_contract = load_yaml_path(
        W / "config/thesis_experiments/v3/v3_07_a0_spatial_candidate_foundation_contract.yaml"
    )
    prefix = [(0.0, [-3.0, 0.0, 0.0]), (0.5, [-2.6, 0.0, 0.0])]
    candidate = row.get("runtime_candidate_id", "straight_v120")
    if candidate == "spatial_left_v120":
        value = PARENT.PARENT.SPATIAL.generate(candidate, 1.0, spatial_contract)
        relative = [
            (float(p["time_from_start_s"]), [float(p["x_m"]), float(p["y_m"]), float(p["yaw_rad"])])
            for p in value["points"]
        ]
    elif candidate == "certified_stop_v000":
        relative = _stop_branch(spatial_contract)
    else:
        relative = PARENT.PARENT._straight_branch(spatial_contract)
    return tuple(prefix + [(0.5 + t, pose) for t, pose in relative[1:]])


def render_world(contract, row, output_csv, done_marker):
    base_d1 = PARENT.PARENT.BASE
    base_predictor = base_d1.BASE
    original = base_predictor.ego_waypoints
    base_predictor.ego_waypoints = ego_waypoints
    try:
        return base_d1.render_world(contract, row, output_csv, done_marker)
    finally:
        base_predictor.ego_waypoints = original
