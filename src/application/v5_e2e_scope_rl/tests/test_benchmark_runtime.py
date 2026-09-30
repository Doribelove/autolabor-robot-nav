import math

import numpy as np
import pytest

from v5_e2e_scope_rl.benchmark_runtime import (
    BenchmarkSensorCandidateEnv,
    BenchmarkRuntimeAdapter,
    DirectionalBenchmarkRuntimeAdapter,
    RuntimeInputHealth,
)
from v5_e2e_scope_rl.benchmark_training import BenchmarkPrivilegedRankSensorEnv
from v5_e2e_scope_rl.sensor_env import SensorCandidateEnv


def healthy():
    return RuntimeInputHealth(0.01, 0.01, 0.01)


def controlled_pair():
    env = BenchmarkSensorCandidateEnv(
        {
            "family": "bidirectional_reactive_crossing",
            "scan_noise_std_m": 0.0,
            "scan_dropout_probability": 0.0,
            "shield_inflation_m": 0.33,
            "safety_response_delay_s": 0.35,
            "certified_stop_dwell_s": 1.50,
        }
    )
    env.reset(
        seed=196001,
        options={
            "family": "bidirectional_reactive_crossing",
            "actor_side": -1.0,
            "actor_speed_mps": 0.9,
            "actor_mode": "cautious",
        },
    )
    env.ego_x = -2.6
    env.ego_y = 0.0
    env.ego_yaw = 0.0
    env.ego_speed = 0.8
    env.ego_steering = 0.0
    env.previous_action = 2
    env.previous_target_speed = 0.8
    env.step_count = 10
    env.consecutive_stop_count = 0
    scan = env._raycast_scan(add_noise=False)
    env.scan_history = np.repeat(scan[None, :], 6, axis=0)
    env._refresh_candidates()

    runtime = BenchmarkRuntimeAdapter(
        goal_speed_envelope=False,
        wall_half_width_m=20.0,
        shield_inflation_m=0.33,
        safety_response_delay_s=0.35,
        certified_stop_dwell_s=1.50,
    )
    runtime.scan_history = np.repeat(scan[None, :], 6, axis=0)
    runtime.previous_action = 2
    runtime.previous_target_speed = 0.8
    runtime.consecutive_stop_count = 0
    runtime.decision_count = 10
    return env.policy_observation(), runtime


def test_fixed_input_training_runtime_parity():
    training, runtime = controlled_pair()
    deployed, previews = runtime.build((-2.6, 0.0, 0.0), 0.8, 0.0, 2.5)
    assert len(previews) == 8
    for key in ("scans", "robot", "candidates", "analytic_prior", "action_mask"):
        np.testing.assert_array_equal(deployed[key], training[key], err_msg=key)


def test_certified_stop_penalty_has_training_runtime_parity_and_preserves_fallback():
    training, runtime = controlled_pair()
    deployed, _ = runtime.build((-2.6, 0.0, 0.0), 0.8, 0.0, 2.5)
    assert deployed["analytic_prior"][0] == pytest.approx(training["analytic_prior"][0])
    assert deployed["analytic_prior"][0] < deployed["analytic_prior"][1]
    only_stop = {key: np.array(value, copy=True) for key, value in deployed.items()}
    only_stop["action_mask"][:] = 0
    only_stop["action_mask"][0] = 1
    assert runtime._analytic_action(only_stop) == 0


def test_spatial_commitment_bonus_is_training_runtime_aligned():
    env = BenchmarkSensorCandidateEnv(
        {"spatial_commitment_bonus": 1.0, "shield_inflation_m": 0.33}
    )
    runtime = BenchmarkRuntimeAdapter(spatial_commitment_bonus=1.0)
    env.reset(seed=196049, options={"family": "narrow_corridor_yield"})
    runtime.wall_half_width_m = env.wall_half_width_m
    runtime.static_circles = tuple(dict(item) for item in env.static_circles)
    pose = (env.ego_x, env.ego_y, env.ego_yaw)
    runtime.scan_history = env.scan_history.copy()
    runtime.previous_action = env.previous_action
    runtime.previous_target_speed = env.previous_target_speed
    deployed, _ = runtime.build(pose, env.ego_speed, 0.0, 0.0)
    training = env.policy_observation()
    np.testing.assert_array_equal(deployed["analytic_prior"], training["analytic_prior"])


def test_candidate_switch_hysteresis_is_training_runtime_aligned():
    training, runtime = controlled_pair()
    deployed, _ = runtime.build((-2.6, 0.0, 0.0), 0.8, 0.0, 2.5)
    np.testing.assert_array_equal(deployed["analytic_prior"], training["analytic_prior"])
    assert runtime.candidate_switch_penalty == pytest.approx(0.15)


def test_benchmark_spatial_candidate_coverage_is_training_runtime_aligned():
    env = BenchmarkSensorCandidateEnv()
    runtime = BenchmarkRuntimeAdapter()
    np.testing.assert_array_equal(runtime.lateral_offsets, env.lateral_offsets)
    np.testing.assert_array_equal(runtime.target_speeds, env.target_speeds)
    np.testing.assert_array_equal(runtime.lateral_offsets[4:], [1.35, -1.35, 1.35, -1.35])
    np.testing.assert_array_equal(runtime.target_speeds[4:], [0.4, 0.4, 1.2, 1.2])
    assert runtime.maximum_steering_rad == pytest.approx(env.maximum_steering_rad)
    assert runtime.maximum_steering_rad == pytest.approx(0.48)
    assert runtime.maximum_steering_rate_radps == pytest.approx(0.80)


@pytest.mark.parametrize(
    "family,wall_half_width,actor_xy",
    [
        ("bidirectional_reactive_crossing", 20.0, (0.0, 3.60)),
        ("partially_occluded_crossing", 20.0, (0.0, 3.60)),
        ("narrow_corridor_yield", 2.55, (3.60, 1.40)),
    ],
)
def test_benchmark_training_scene_geometry_matches_gazebo_contract(
    family, wall_half_width, actor_xy
):
    env = BenchmarkSensorCandidateEnv({"family": family})
    env.reset(seed=196200, options={"actor_side": 1.0, "actor_speed_mps": 0.90})
    assert env.max_steps == 88
    assert env.wall_half_width_m == pytest.approx(wall_half_width)
    assert env.actors[0]["x"] == pytest.approx(actor_xy[0])
    assert env.actors[0]["y"] == pytest.approx(actor_xy[1])
    if family == "partially_occluded_crossing":
        assert env.static_circles == [
            {"x": -0.10, "y": 1.65, "radius": 0.50}
        ]


def test_privileged_rank_label_returns_to_straight_after_head_on_actor_passes():
    env = BenchmarkPrivilegedRankSensorEnv(
        {
            "family": "narrow_corridor_yield",
            "scan_noise_std_m": 0.0,
            "scan_dropout_probability": 0.0,
        }
    )
    env.reset(
        seed=196201,
        options={"actor_side": 1.0, "actor_speed_mps": 0.90, "actor_mode": "cautious"},
    )
    env.ego_x = -1.0
    env.ego_speed = 0.8
    env.actors[0]["x"] = -3.0
    env.actors[0]["vx"] = -0.9
    env.actors[0]["base_vx"] = -0.9
    env._initialize_scan_history()
    env._refresh_candidates()
    assert env.analytic_action() == 3
    assert env.privileged_rank_target() == 3
    assert env.action_masks()[env.privileged_rank_target()]


def test_privileged_rank_label_has_no_scene_specific_head_on_override():
    env = BenchmarkPrivilegedRankSensorEnv(
        {
            "family": "narrow_corridor_yield",
            "scan_noise_std_m": 0.0,
            "scan_dropout_probability": 0.0,
        }
    )
    env.reset(
        seed=196202,
        options={"actor_side": 1.0, "actor_speed_mps": 0.90, "actor_mode": "cautious"},
    )
    # The rejected R2 experiment added a scene-specific action-7 bonus here.
    # The retained label must remain the generic candidate-clearance
    # substitution used for every family.
    for _ in range(20):
        target = env.privileged_rank_target()
        scores = env.privileged_rank_scores()
        if env.action_masks()[7] and env._oracle_clearance(7) >= 0.20:
            break
        env.step(env.analytic_action())
    assert target == int(np.argmax(scores))
    assert env.action_masks()[7]
    assert target != 7


def test_opposite_spatial_side_penalty_is_training_runtime_aligned():
    _, runtime = controlled_pair()
    runtime.previous_action = 4
    runtime.previous_target_speed = 0.4
    runtime.committed_spatial_side = 1
    deployed, _ = runtime.build((-2.6, 0.0, 0.0), 0.8, 0.0, 2.5)

    env = BenchmarkSensorCandidateEnv(
        {
            "family": "bidirectional_reactive_crossing",
            "scan_noise_std_m": 0.0,
            "scan_dropout_probability": 0.0,
            "shield_inflation_m": 0.33,
            "safety_response_delay_s": 0.35,
            "certified_stop_dwell_s": 1.50,
        }
    )
    env.reset(
        seed=196001,
        options={
            "family": "bidirectional_reactive_crossing",
            "actor_side": -1.0,
            "actor_speed_mps": 0.9,
            "actor_mode": "cautious",
        },
    )
    env.ego_x, env.ego_y, env.ego_yaw = -2.6, 0.0, 0.0
    env.ego_speed, env.ego_steering = 0.8, 0.0
    env.previous_action, env.previous_target_speed = 4, 0.4
    env.committed_spatial_side = 1
    env.step_count, env.consecutive_stop_count = 10, 0
    env.scan_history = runtime.scan_history.copy()
    env._refresh_candidates()
    np.testing.assert_array_equal(deployed["analytic_prior"], env.policy_observation()["analytic_prior"])
    assert deployed["analytic_prior"][5] < deployed["analytic_prior"][4]


def test_certified_stop_preserves_spatial_side_commitment_until_straight_commit():
    runtime = BenchmarkRuntimeAdapter()
    runtime.last_validated_action = 4
    runtime.commit_executed(4)
    assert runtime.committed_spatial_side == 1
    assert runtime.committed_spatial_speed_mps == pytest.approx(0.4)
    runtime.last_validated_action = 0
    runtime.commit_executed(0)
    assert runtime.committed_spatial_side == 1
    assert runtime.committed_spatial_speed_mps == pytest.approx(0.4)
    for _ in range(3):
        runtime.last_validated_action = 2
        runtime.commit_executed(2)
        assert runtime.committed_spatial_side == 1
    runtime.last_validated_action = 2
    runtime.commit_executed(2)
    assert runtime.committed_spatial_side == 0
    assert runtime.committed_spatial_speed_mps == 0.0


def test_low_speed_spatial_commitment_penalizes_all_non_stop_escalation():
    _, runtime = controlled_pair()
    runtime.previous_action = 5
    runtime.previous_target_speed = 0.4
    runtime.committed_spatial_side = -1
    runtime.committed_spatial_speed_mps = 0.4
    deployed, _ = runtime.build((-2.6, 0.0, 0.0), 0.4, 0.0, 2.5)
    assert deployed["analytic_prior"][7] < deployed["analytic_prior"][5]
    assert deployed["analytic_prior"][3] < deployed["analytic_prior"][1]


def test_same_side_spatial_speed_upgrade_requires_full_clearance_release():
    env = BenchmarkSensorCandidateEnv()
    runtime = BenchmarkRuntimeAdapter()
    for subject in (env, runtime):
        subject.committed_spatial_side = -1
        subject.committed_spatial_speed_mps = 0.4
        assert not subject._spatial_speed_upgrade_allowed(7, 0.219)
        assert subject._spatial_speed_upgrade_allowed(7, 0.220)
        assert not subject._spatial_speed_upgrade_allowed(6, 1.0)
        assert not subject._spatial_speed_upgrade_allowed(5, 1.0)
        assert subject.spatial_speed_escalation_release_clearance_m == pytest.approx(0.22)


def test_confirmed_same_side_spatial_pulse_preserves_safe_commitment_speed():
    runtime = BenchmarkRuntimeAdapter()
    runtime.last_validated_action = 5
    runtime.commit_executed(5)
    assert runtime.committed_spatial_speed_mps == pytest.approx(0.4)
    runtime.last_validated_action = 7
    runtime.commit_executed(7)
    assert runtime.committed_spatial_side == -1
    assert runtime.committed_spatial_speed_mps == pytest.approx(0.4)


def test_only_centerline_intruding_static_geometry_extends_spatial_commitment():
    center = BenchmarkRuntimeAdapter(
        static_boxes=({"x": -0.15, "y": 0.0, "length": 0.8, "width": 0.9},)
    )
    offset = BenchmarkRuntimeAdapter(
        static_boxes=({"x": -0.10, "y": -1.65, "length": 0.8, "width": 0.6},)
    )
    assert center._central_static_route_release_x() == pytest.approx(1.24)
    assert offset._central_static_route_release_x() == float("-inf")


def test_centerline_obstacle_prevents_premature_straight_commitment_release():
    runtime = BenchmarkRuntimeAdapter(
        static_boxes=({"x": -0.15, "y": 0.0, "length": 0.8, "width": 0.9},)
    )
    runtime.last_validated_action = 5
    runtime.commit_executed(5)
    runtime.latest_pose_x_m = -1.0
    for _ in range(4):
        runtime.last_validated_action = 1
        runtime.commit_executed(1)
    assert runtime.committed_spatial_side == -1
    runtime.latest_pose_x_m = 1.25
    runtime.last_validated_action = 1
    runtime.commit_executed(1)
    assert runtime.committed_spatial_side == 0


def test_observed_pose_past_static_release_clears_commitment_before_ranking():
    runtime = BenchmarkRuntimeAdapter(
        static_boxes=({"x": -0.15, "y": 0.0, "length": 0.8, "width": 0.9},)
    )
    runtime.committed_spatial_side = 1
    runtime.committed_spatial_speed_mps = 0.4
    runtime.scan_history = np.ones((6, 72), dtype=np.float32) * runtime.maximum_range_m
    runtime.build((1.25, 1.4, 0.0), 0.4, 0.0, 10.0)
    assert runtime.committed_spatial_side == 0
    assert runtime.committed_spatial_speed_mps == 0.0


def test_suggestion_does_not_mutate_previous_execution():
    _, runtime = controlled_pair()
    suggestion = runtime.suggest((-2.6, 0.0, 0.0), 0.8, 0.0, 2.5, healthy())
    assert runtime.previous_action == 2
    validation = runtime.revalidate(
        suggestion, (-2.6, 0.0, 0.0), 0.8, 0.0, 2.5, healthy()
    )
    assert runtime.previous_action == 2
    receipt = runtime.commit_executed(validation["validated_action"])
    assert runtime.previous_action == receipt["executed_action"]
    assert runtime.execution_audit()["previous_action_feature_source"] == "last_confirmed_executed_action"


def test_stale_input_fails_closed_and_is_committable():
    _, runtime = controlled_pair()
    stale = RuntimeInputHealth(0.25, 0.01, 0.01)
    suggestion = runtime.suggest((-2.6, 0.0, 0.0), 0.8, 0.0, 2.5, stale)
    assert suggestion["suggested_action"] == 0
    assert suggestion["fallback_reason"] == "input_health_failure"
    validation = runtime.revalidate(
        suggestion, (-2.6, 0.0, 0.0), 0.8, 0.0, 2.5, stale
    )
    assert validation["validated_action"] == 0
    runtime.commit_executed(0)
    assert runtime.previous_action == 0


def test_scan_contract_rejects_nonfinite_and_empty_raw_input():
    runtime = BenchmarkRuntimeAdapter()
    with pytest.raises(ValueError, match="CARDINALITY"):
        runtime.resample_scan([], -3.14, 0.01, 30.0)
    with pytest.raises(ValueError, match="NONFINITE"):
        runtime.append_scan(np.full(72, np.nan, dtype=np.float32))


def test_goal_speed_envelope_stops_candidate_before_longitudinal_overshoot():
    runtime = BenchmarkRuntimeAdapter(goal_speed_envelope=True)
    preview = runtime._preview((4.0, 0.0, 0.0), 1.2, 0.0, 3, stop_tail=False)
    assert preview[-1][0] <= runtime.goal_x_m + runtime.goal_tolerance_m
    assert preview[-1][3] <= 0.05


def test_commit_rejects_execution_semantic_mismatch():
    _, runtime = controlled_pair()
    suggestion = runtime.suggest((-2.6, 0.0, 0.0), 0.8, 0.0, 2.5, healthy())
    validation = runtime.revalidate(
        suggestion, (-2.6, 0.0, 0.0), 0.8, 0.0, 2.5, healthy()
    )
    wrong = (validation["validated_action"] + 1) % 8
    with pytest.raises(ValueError, match="SEMANTIC_MISMATCH"):
        runtime.commit_executed(wrong)


def test_positive_backup_viability_margin_is_training_runtime_aligned():
    training, runtime = controlled_pair()
    deployed, _ = runtime.build((-2.6, 0.0, 0.0), 0.8, 0.0, 2.5)
    np.testing.assert_array_equal(deployed["action_mask"], training["action_mask"])
    assert runtime.shield_clearance_minimum_m == pytest.approx(0.20)
    assert training["action_mask"][0] == 1


def test_terminal_approach_preserves_stop_and_avoids_lookahead_deadlock():
    runtime = BenchmarkRuntimeAdapter(goal_speed_envelope=True)
    assert runtime.terminal_command_speed(0, 4.50, 0.7) == 0.0
    assert runtime.terminal_command_speed(2, 4.50, 0.0) == pytest.approx(0.20)
    assert runtime.terminal_command_speed(2, 5.06, 0.8) == 0.0


def test_terminal_pose_controller_removes_lateral_candidate_heading_bias():
    runtime = BenchmarkRuntimeAdapter(goal_speed_envelope=True)
    speed, yaw_rate = runtime.terminal_command(6, (4.40, 0.15, -0.45), 0.20, -0.20)
    assert speed == pytest.approx(0.20)
    assert yaw_rate > 0.0
    assert abs(yaw_rate / speed) <= math.tan(0.40) / runtime.wheelbase_m
    assert runtime.terminal_command(0, (4.40, 0.15, -0.45), 0.20, -0.20) == (0.0, 0.0)
    early_speed, early_yaw_rate = runtime.terminal_command(
        6, (3.90, 0.15, -0.45), 0.50, -0.20
    )
    assert early_speed == pytest.approx(0.50)
    assert early_yaw_rate > -0.20


def test_lateral_candidates_depart_before_policy_gate_and_recover_with_parity():
    env = BenchmarkSensorCandidateEnv(
        {"route_departure_x_m": -5.0, "route_recovery_x_m": 2.0}
    )
    runtime = BenchmarkRuntimeAdapter(route_departure_x_m=-5.0, route_recovery_x_m=2.0)
    for x_m in (-5.0, -4.0, -1.0, 0.0, 1.5, 2.0, 3.0):
        assert runtime._target_lateral(x_m, 1.2) == pytest.approx(
            env._target_lateral(x_m, 1.2)
        )
    assert runtime._target_lateral(-5.0, 1.2) == 0.0
    assert runtime._target_lateral(-4.0, 1.2) > 0.5
    assert runtime._target_lateral(2.0, 1.2) == 0.0
    assert runtime._target_lateral(-1.5, 1.2) == pytest.approx(1.2)


def test_benchmark_default_lateral_route_is_already_curving_at_policy_gate():
    env = BenchmarkSensorCandidateEnv()
    runtime = BenchmarkRuntimeAdapter()
    assert env.route_departure_x_m == pytest.approx(-5.5)
    assert runtime.route_departure_x_m == pytest.approx(-5.5)
    assert runtime.route_hold_start_x_m == pytest.approx(-3.0)
    assert runtime._target_lateral(-4.0, 1.4) > 0.9
    assert runtime._target_lateral(-1.0, 1.4) == pytest.approx(1.4)
    assert 0.0 < runtime._target_lateral(1.5, 1.4) < 1.4


def test_static_surface_association_accepts_noisy_box_edges_but_not_interior_actor():
    runtime = BenchmarkRuntimeAdapter(
        wall_half_width_m=2.35,
        static_boxes=({"x": 0.0, "y": 0.0, "length": 0.8, "width": 0.9},),
        static_surface_association_tolerance_m=0.15,
    )
    points = np.asarray(
        [
            [0.49, 0.20],
            [-0.46, -0.52],
            [1.20, 2.44],
            [0.0, 1.0],
        ],
        dtype=np.float64,
    )
    np.testing.assert_array_equal(
        runtime._known_static_point_mask(points), [True, True, True, False]
    )


def test_known_static_box_clearance_uses_complete_map_not_visible_surface_subset():
    runtime = BenchmarkRuntimeAdapter(
        wall_half_width_m=2.35,
        static_boxes=({"x": -0.15, "y": 0.0, "length": 0.8, "width": 0.9},),
        shield_inflation_m=0.33,
    )
    inside = runtime._known_static_geometry_clearance((-0.15, 0.0, 0.0, 0.0, 0.0))
    around = runtime._known_static_geometry_clearance((-1.34, 0.80, 0.0, 0.0, 0.0))
    assert inside == pytest.approx(-0.79)
    assert around > 0.0


def test_benchmark_wall_clearance_uses_evaluator_aligned_inflation_with_or_without_scan():
    runtime = BenchmarkRuntimeAdapter(
        wall_half_width_m=2.55,
        shield_inflation_m=0.33,
    )
    state = (0.0, 1.50, 0.0, 0.0, 0.0)
    expected = 2.55 - 1.50 - 0.46 - 0.33
    assert runtime._known_static_geometry_clearance(state) == pytest.approx(expected)

    env = BenchmarkSensorCandidateEnv(
        {
            "family": "narrow_corridor_yield",
            "shield_inflation_m": 0.33,
            "scan_noise_std_m": 0.0,
            "scan_dropout_probability": 0.0,
        }
    )
    env.reset(seed=196203, options={"actor_side": 1.0, "actor_speed_mps": 0.90})
    assert env._observable_clearance([state]) == pytest.approx(expected)


def test_benchmark_training_clearance_and_rank_oracle_share_evaluator_footprint():
    env = BenchmarkSensorCandidateEnv(
        {
            "family": "bidirectional_reactive_crossing",
            "shield_inflation_m": 0.33,
            "scan_noise_std_m": 0.0,
            "scan_dropout_probability": 0.0,
        }
    )
    env.reset(seed=196204, options={"actor_side": 1.0, "actor_speed_mps": 0.90})
    env.ego_x = env.actors[0]["x"]
    env.ego_y = env.actors[0]["y"] - 1.50
    expected = 1.50 - env.robot_radius_m - env.shield_inflation_m - env.actors[0]["radius"]
    assert env._physical_clearance() == pytest.approx(expected)


def test_backup_preview_accounts_for_bounded_response_delay():
    immediate = BenchmarkRuntimeAdapter(goal_speed_envelope=False, safety_response_delay_s=0.0)
    delayed = BenchmarkRuntimeAdapter(goal_speed_envelope=False, safety_response_delay_s=0.35)
    pose = (-1.5, 0.0, 0.0)
    immediate_preview = immediate._preview(pose, 0.8, 0.0, 0, stop_tail=True)
    delayed_preview = delayed._preview(pose, 0.8, 0.0, 0, stop_tail=True)
    assert len(delayed_preview) == len(immediate_preview) + 2
    assert max(state[0] for state in delayed_preview) > max(state[0] for state in immediate_preview)


def test_backup_preview_retains_post_stop_invariance_dwell():
    runtime = BenchmarkRuntimeAdapter(
        goal_speed_envelope=False,
        safety_response_delay_s=0.35,
        certified_stop_dwell_s=1.50,
    )
    preview = runtime._preview((-1.5, 0.0, 0.0), 0.8, 0.0, 0, stop_tail=True)
    assert preview[-1][3] == 0.0
    assert preview[-7][3] == 0.0


def test_directional_tracker_estimates_moving_scan_surface_without_world_truth():
    runtime = DirectionalBenchmarkRuntimeAdapter(
        wall_half_width_m=20.0,
        goal_speed_envelope=False,
        safety_response_delay_s=0.35,
        certified_stop_dwell_s=1.50,
    )
    pose = (-4.0, 0.0, 0.0)
    for index in range(5):
        actor = np.asarray([0.0, -3.0 + 0.20 * index])
        relative = actor - np.asarray(pose[:2])
        angle = np.arctan2(relative[1], relative[0])
        beam = int(np.argmin(np.abs(runtime.beam_angles - angle)))
        scan = np.full(72, 8.0, dtype=np.float32)
        surface_range = float(np.linalg.norm(relative)) - 0.36
        scan[max(0, beam - 1) : min(72, beam + 2)] = surface_range
        runtime.append_scan_with_pose(scan, pose, 0.25 * index)
    audit = runtime.tracker_audit()
    assert audit["valid"] is True
    assert audit["velocity_world_mps"][1] > 0.45
    assert audit["safety_prediction_speed_mps"] == pytest.approx(1.35)
    assert audit["fit_residual_m"] < 0.25


def test_directional_runtime_reuses_scan_timestamp_pose_for_world_projection():
    runtime = DirectionalBenchmarkRuntimeAdapter(wall_half_width_m=20.0)
    scan = np.full(72, 8.0, dtype=np.float32)
    scan[36] = 2.0
    stamped_pose = (-2.0, 0.3, 0.2)
    runtime.append_scan_with_pose(scan, stamped_pose, 1.0)
    expected = runtime._world_cloud(scan, stamped_pose)
    projected = runtime._scan_points_world((4.0, -3.0, -1.0))
    np.testing.assert_allclose(projected, expected)


def test_directional_tracker_rejects_unassociated_hijack_then_expires():
    runtime = DirectionalBenchmarkRuntimeAdapter(wall_half_width_m=20.0)
    pose = (-4.0, 0.0, 0.0)
    for index in range(4):
        scan = np.full(72, 8.0, dtype=np.float32)
        actor = np.asarray([0.0, -2.0 + 0.15 * index])
        relative = actor - np.asarray(pose[:2])
        beam = int(np.argmin(np.abs(runtime.beam_angles - np.arctan2(relative[1], relative[0]))))
        scan[beam] = float(np.linalg.norm(relative)) - 0.36
        runtime.append_scan_with_pose(scan, pose, 0.20 * index)
    original = runtime.dynamic_center.copy()

    unrelated = np.full(72, 8.0, dtype=np.float32)
    unrelated[60] = 1.0
    runtime.append_scan_with_pose(unrelated, pose, 0.80)
    np.testing.assert_allclose(runtime.dynamic_center, original)
    assert runtime.tracker_audit()["track_age_s"] == pytest.approx(0.20)

    empty = np.full(72, 8.0, dtype=np.float32)
    runtime.append_scan_with_pose(empty, pose, 1.20)
    assert runtime.dynamic_center is None
    assert runtime.dynamic_track_valid is False
