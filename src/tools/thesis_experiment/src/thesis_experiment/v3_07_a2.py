"""V3-07-A2 trace-window repair bindings."""

import importlib.util
from pathlib import Path

W = Path("/home/robot/robot_ws_base_rl")
PARENT_PATH = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_07_a1.py"
spec = importlib.util.spec_from_file_location("v3_07_a1_parent_for_a2", PARENT_PATH)
PARENT = importlib.util.module_from_spec(spec)
spec.loader.exec_module(PARENT)
for name in (
    "V301Error", "require", "sha256_bytes", "yaml_bytes", "canonical_sha",
    "read_file_once", "load_yaml_path", "load_yaml_relative", "receipt",
    "absolute_receipt", "load_trace", "interpolate", "crossing_time",
    "check_receipts", "actor_direction", "trace_metrics", "ego_waypoints",
    "render_world",
):
    globals()[name] = getattr(PARENT, name)

CONTRACT_RELATIVE = Path(
    "config/thesis_experiments/v3/v3_07_a2_trace_window_repair_contract.yaml"
)
PREREGISTRATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_07_a2_trace_window_repair_preregistration.yaml"
)
AUTHORIZATION_RELATIVE = Path(
    "experiments/manifests/v3/v3_07_a2_simulation_authorization.yaml"
)
ROOT_RELATIVE = Path("artifacts/v3/spatial/v3_07_a2")
READINESS_RELATIVE = ROOT_RELATIVE / "offline_readiness"
SCHEDULE_RELATIVE = READINESS_RELATIVE / "v3_07_a2_exact_schedule.yaml"
BINDINGS_RELATIVE = READINESS_RELATIVE / "v3_07_a2_runtime_bindings.yaml"
CLOSURE_RELATIVE = READINESS_RELATIVE / "v3_07_a2_dependency_closure.yaml"
RELEASE_RELATIVE = READINESS_RELATIVE / "v3_07_a2_execution_release.yaml"
REVIEW_RELATIVE = READINESS_RELATIVE / "v3_07_a2_independent_release_review.yaml"
READINESS_FREEZE_RELATIVE = READINESS_RELATIVE / "v3_07_a2_readiness_freeze.sha256"
EXECUTION_RELATIVE = ROOT_RELATIVE / "execution"
JOURNAL_RELATIVE = EXECUTION_RELATIVE / "v3_07_a2_attempt_journal.jsonl"
ASSESSMENT_RELATIVE = EXECUTION_RELATIVE / "v3_07_a2_identifiability_assessment.yaml"
EVIDENCE_FREEZE_RELATIVE = EXECUTION_RELATIVE / "v3_07_a2_evidence_freeze.sha256"
RESULT_REVIEW_RELATIVE = EXECUTION_RELATIVE / "v3_07_a2_independent_result_review.yaml"
TERMINAL_FREEZE_RELATIVE = ROOT_RELATIVE / "v3_07_a2_terminal_freeze.sha256"
TRACE_METRICS_NEEDS_ROW = True
BINARY_RESOURCES = PARENT.BINARY_RESOURCES
SOURCE_RESOURCES = (
    CONTRACT_RELATIVE,
    PREREGISTRATION_RELATIVE,
    Path("artifacts/v3/spatial/v3_07_a1/v3_07_a1_terminal_freeze.sha256"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_spatial_candidates.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_a1.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_07_a2.py"),
    Path("src/simulation/m2_gazebo/include/m2_gazebo/v3_reactive_actor.h"),
    Path("src/simulation/m2_gazebo/src/v2_trajectory_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_reactive_actor_plugin.cpp"),
    Path("src/simulation/m2_gazebo/src/v3_episode_recorder_plugin.cpp"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_predictor.py"),
    Path("src/tools/thesis_experiment/src/thesis_experiment/v3_02_d1.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_02_a1_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_01_r1_qualification.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_07_a1_identifiability.py"),
    Path("src/tools/thesis_experiment/scripts/build_v3_07_a2_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_07_a2_readiness.py"),
    Path("src/tools/thesis_experiment/scripts/run_v3_07_a2_identifiability.py"),
    Path("src/tools/thesis_experiment/scripts/assess_v3_07_a2_identifiability.py"),
    Path("src/tools/thesis_experiment/scripts/review_v3_07_a2_result.py"),
    Path("src/tools/thesis_experiment/tests/test_v3_07_a2.py"),
)


def validate_contract(contract, preregistration):
    require(
        contract["stage"] == "V3-07-A2"
        and contract["status"]
        == "single_factor_repair_preregistered_before_simulation",
        "A2 contract differs",
    )
    repair = contract["single_factor_repair"]
    require(
        repair["factor_id"] == "evidence_recording_duration_only"
        and float(repair["old_duration_s"]) == 5.0
        and float(repair["new_duration_s"]) == 8.0
        and all(
            repair[key] is False
            for key in (
                "candidate_geometry_changed",
                "common_prefix_changed",
                "actor_semantics_changed",
                "qualification_gates_changed",
                "safety_radius_changed",
            )
        ),
        "A2 repair differs",
    )
    unit = contract["experimental_unit"]
    require(
        len(unit["scenario_blocks"]) == 4
        and unit["schedule_rows"] == 36
        and unit["training_rows"] == 0
        and unit["calibration_rows"] == 36
        and unit["retry_allowed"] is False
        and unit["resume_allowed"] is False,
        "A2 population differs",
    )
    require(
        preregistration["status"]
        == "frozen_before_any_v3_07_a2_gazebo_execution"
        and preregistration["repair"]["only_changed_field"]
        == "scene.duration_s"
        and preregistration["failure_policy"][
            "no_predictor_data_stage_if_identifiability_fails"
        ]
        is True,
        "A2 preregistration differs",
    )
    authority = contract["authority_boundary"]
    require(
        authority["held_out_test_seeds_reserved"] == [6571, 6572, 6573]
        and authority["held_out_test_access_authorized"] is False
        and authority["simulation_requires_separate_release_and_authorization"]
        is True
        and authority["training_authorized"] is False
        and authority["real_vehicle_authorized"] is False,
        "A2 authority differs",
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
            for branch in branches[offset:] + branches[:offset]:
                sequence += 1
                rows.append({
                    "sequence": sequence,
                    "unit_id": "v3_07_a2_{}_{}_{}".format(
                        block["block_id"], policy, branch
                    ),
                    "block_id": block["block_id"],
                    "role_evaluator_only": "qualification",
                    "seed": int(block["seed"]),
                    "angle_deg": float(block["angle_deg"]),
                    "actor_start_distance_m": float(block["actor_start_distance_m"]),
                    "actor_base_speed_mps": float(block["actor_base_speed_mps"]),
                    "actor_policy_evaluator_only": policy,
                    "candidate_id_evaluator_only": branch,
                    "ego_speed_mps": 1.2,
                    "attempt": 1,
                    "propensity": 1.0 / 3.0,
                })
    require(len(rows) == 36, "A2 schedule differs")
    return {
        "schema_version": "1.0", "stage": "V3-07-A2",
        "schedule_id": "v3_scope_v3_07_a2_exact_schedule_1",
        "status": "frozen_before_execution_release", "row_count": 36,
        "training_rows": 0, "calibration_rows": 36,
        "held_out_test_rows": 0,
        "reserved_held_out_seeds_not_materialized": [6571, 6572, 6573],
        "authorized_attempts_per_identity": 1, "retry_allowed": False,
        "rows": rows, "schedule_digest": canonical_sha(rows),
    }
