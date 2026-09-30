import importlib.util
from pathlib import Path

import pytest
import yaml


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
BUILDER_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/scripts/"
    "v2_04g_r6_i1_p2_q1_offline_builder.py"
)
REPLAY_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/"
    "v2_04g_r6_i1_p2_q1_replay.py"
)


def _module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BUILDER = _module("p2_q1_builder_under_test", BUILDER_PATH)
REPLAY = _module("p2_q1_replay_under_test", REPLAY_PATH)


def _yaml(files, relative):
    return yaml.safe_load(files[relative].decode("utf-8"))


def _qualification_sample(
    stamp,
    x,
    speed,
    yaw=0.0,
    overlay="NONE",
    trigger=False,
    collision=False,
    emergency=False,
    projection=False,
    transaction=True,
):
    return {
        "stamp_s": stamp,
        "x_m": x,
        "y_m": 0.0,
        "linear_velocity_mps": speed,
        "yaw_rate_radps": yaw,
        "clearance_m": 0.5,
        "collision": collision,
        "dynamic_overlay": overlay,
        "conflict_triggered": trigger,
        "emergency": emergency,
        "projection_applied": projection,
        "transaction_valid": transaction,
    }


def _planned_probe(probe, speed=1.3, seed=5334):
    return {
        "sequence": 331,
        "row_type": "qualification_probe",
        "probe_type": probe,
        "candidate_speed_mps": speed,
        "scene_id": "fixture",
        "seed": seed,
        "attempt": 1,
    }


def _thresholds():
    contract = yaml.safe_load(
        (WORKSPACE / BUILDER.CONTRACT_RELATIVE).read_text(encoding="utf-8")
    )
    return contract["qualification_thresholds"]


def test_q1_bundle_has_exact_single_factor_method_and_seed_firewalls():
    files = BUILDER.build_bundle(WORKSPACE)
    methods = _yaml(files, BUILDER.METHOD_BINDINGS_RELATIVE)
    scenes = _yaml(files, BUILDER.SCENES_RELATIVE)
    schedule = _yaml(files, BUILDER.SCHEDULE_RELATIVE)
    plan = _yaml(files, BUILDER.COMMAND_PLAN_RELATIVE)
    assert len(methods["methods"]) == 11
    assert len(scenes["scenes"]) == 48
    assert len(schedule["rows"]) == 342
    assert len(plan["rows"]) == 342
    assert schedule["calibration_row_count"] == 330
    assert schedule["qualification_row_count"] == 12
    assert schedule["held_out_test_row_count"] == 0
    assert not set(range(5401, 5491)) & {
        row["seed"] for row in schedule["rows"]
    }
    assert methods["fair_baseline_rule"]["intentional_weakening_or_hidden_advantage_allowed"] is False
    assert methods["fair_baseline_rule"]["transparent_reference_is_automatically_primary"] is False


def test_q1_candidate_diffs_and_schedule_balance_are_exact():
    files = BUILDER.build_bundle(WORKSPACE)
    audit = _yaml(files, BUILDER.AUDIT_RELATIVE)
    schedule = _yaml(files, BUILDER.SCHEDULE_RELATIVE)
    candidate_rows = {
        row["method_id"]: row for row in audit["candidate_diff_audit"]
    }
    for speed in ("v125", "v130", "v135", "v140"):
        circle = candidate_rows["circle_contact_{}".format(speed)]
        assert circle["changed_behavioral_field_count"] == 1
        assert circle["changed_fields"].count(
            "anchors.anchor_cruise.values.max_vel_x"
        ) == 1
        fixed = candidate_rows["fixed_teb_{}".format(speed)]
        assert fixed["changed_fields"] == ["TebLocalPlannerROS.max_vel_x"]
    assert candidate_rows["TEB-Tuned-V1"]["changed_behavioral_field_count"] == 9
    assert audit["schedule_audit"]["maximum_method_position_count_difference"] <= 1
    assert audit["schedule_audit"]["every_method_runs_every_calibration_seed_once"] is True
    assert schedule["canonical_row_digest"] == BUILDER._canonical_sha(
        schedule["rows"]
    )


def test_navigation_raw_replay_recomputes_metrics_and_rejects_tracker_failure():
    planned = {
        "sequence": 1,
        "row_type": "calibration_navigation",
        "method_id": "circle_contact_v130",
        "scene_id": "fixture",
        "scene_role": "single_true_conflict",
        "seed": 5301,
        "attempt": 1,
        "block_id": "p2-q1-cal-block-s5301",
        "timeout_s": 80.0,
    }
    rows = []
    for stamp, x, speed, goal, ttc in (
        (0.0, 0.0, 1.0, False, None),
        (1.0, 1.0, 1.0, False, 1.2),
        (2.0, 2.0, 0.0, True, 0.8),
    ):
        rows.append(
            {
                "stamp_s": stamp,
                "x_m": x,
                "y_m": 0.0,
                "linear_velocity_mps": speed,
                "yaw_rate_radps": 0.0,
                "clearance_m": 0.5,
                "predicted_ttc_s": ttc,
                "collision": False,
                "goal_reached": goal,
                "emergency": False,
                "projection_applied": False,
                "transaction_valid": True,
                "tracker_valid": True,
            }
        )
    normalized = REPLAY.normalize_navigation_trace(planned, rows)
    assert normalized["success"] is True
    assert normalized["observed_ttc_status"] == "OBSERVED_CONFLICT"
    assert normalized["restricted_completion_time_s"] == pytest.approx(2.0)
    assert normalized["path_length_m"] == pytest.approx(2.0)
    assert normalized["normal_projection_rate"] == 0.0
    invalid = [dict(row, tracker_valid=False, predicted_ttc_s=None) for row in rows]
    with pytest.raises(REPLAY.Q1ReplayError):
        REPLAY.normalize_navigation_trace(planned, invalid)


def test_qualification_replay_passes_three_positive_fixtures_and_fails_invalid_interface():
    thresholds = _thresholds()
    braking = [
        _qualification_sample(0.0, 0.0, 1.30),
        _qualification_sample(0.5, 0.5, 0.70),
        _qualification_sample(1.0, 0.8, 0.0),
    ]
    assert REPLAY.assess_qualification_trace(
        _planned_probe("braking"), braking, thresholds
    )["pass"] is True

    overlay = [
        _qualification_sample(0.0, 0.0, 1.30),
        _qualification_sample(0.5, 0.6, 1.28, trigger=True),
        _qualification_sample(
            0.9, 1.0, 1.20, overlay="CROSSING", trigger=True
        ),
    ]
    overlay_result = REPLAY.assess_qualification_trace(
        _planned_probe("high_speed_to_conflict_overlay_transition"),
        overlay,
        thresholds,
    )
    assert overlay_result["pass"] is True
    assert overlay_result["activation_latency_s"] == pytest.approx(0.4)

    circle = [
        _qualification_sample(
            index * 0.1, index * 0.1, 1.30, yaw=1.30 / 1.2
        )
        for index in range(8)
    ]
    circle_result = REPLAY.assess_qualification_trace(
        _planned_probe("ackermann_circle"), circle, thresholds
    )
    assert circle_result["pass"] is True
    assert circle_result["radius_m"] == pytest.approx(1.2)

    broken = [dict(row) for row in overlay]
    broken[-1]["transaction_valid"] = False
    assert REPLAY.assess_qualification_trace(
        _planned_probe("high_speed_to_conflict_overlay_transition"),
        broken,
        thresholds,
    )["pass"] is False


def test_q1_bundle_serialization_is_deterministic_and_zero_authority():
    first = BUILDER.build_bundle(WORKSPACE)
    second = BUILDER.build_bundle(WORKSPACE)
    assert first == second
    report = _yaml(first, BUILDER.REPORT_RELATIVE)
    failure_index = _yaml(first, BUILDER.FAILURE_INDEX_RELATIVE)
    assert report["status"] == "p2_q1_offline_implementation_readiness_pass_release_absent"
    assert report["execution_ready"] is False
    assert report["execution_authorized"] is False
    assert report["authorized_evidence_units"] == 0
    assert report["execution_gap"]["runner_execute_entry_hard_disabled"] is True
    assert report["claim_state"]["performance_improvement_demonstrated"] is False
    assert failure_index["evidence_units_consumed"] == 0
    assert failure_index["failures"] == []
