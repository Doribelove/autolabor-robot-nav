from pathlib import Path
import math

import yaml

from teb_mode_manager import (
    FeatureSnapshot,
    RuleContextSupervisor,
    RuntimeTrack,
    SupervisorHealth,
)


def _config():
    path = Path(__file__).resolve().parents[1] / "config/v2_03_rule_candidate.yaml"
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _features(stamp, **changes):
    values = dict(
        world_model_seq=int(stamp * 10) + 1,
        stamp_s=stamp,
        front_clearance_m=4.0,
        rear_clearance_m=4.0,
        obstacle_density=0.02,
        static_persistence=0.0,
        corridor_width_m=0.0,
        corridor_parallel_confidence=0.0,
        dead_end_score=0.0,
        path_curvature=0.15,
        goal_direction_stability=0.5,
        rear_covered=True,
        signed_heading_error_rad=0.0,
        left_clearance_m=4.0,
        right_clearance_m=4.0,
    )
    values.update(changes)
    return FeatureSnapshot(**values)


def _settle(changes, tracks=()):
    supervisor = RuleContextSupervisor(_config())
    decision = None
    for index in range(25):
        decision = supervisor.update(
            _features(index * 0.1, **changes), tracks, SupervisorHealth(True, False)
        )
    return supervisor, decision


def test_five_geometry_families_are_separated_without_labels():
    cases = {
        "BALANCED": {},
        "CRUISE": dict(front_clearance_m=20.0, obstacle_density=0.0,
                       path_curvature=0.0, goal_direction_stability=1.0),
        "STATIC_DENSE": dict(obstacle_density=0.16, static_persistence=0.8),
        "CORRIDOR": dict(front_clearance_m=8.0, corridor_width_m=2.0,
                         corridor_parallel_confidence=0.90),
        "MANEUVER": dict(front_clearance_m=0.5, dead_end_score=0.90,
                         rear_covered=True),
    }
    predictions = {label: _settle(features)[1].geometry_mode
                   for label, features in cases.items()}
    assert predictions == {label: label for label in cases}


def test_crossing_overlay_uses_relative_trajectory_and_releases_with_hysteresis():
    crossing = RuntimeTrack(
        track_id=7, motion_class="CROSSING", x=3.0, y=-1.0,
        vx=-1.0, vy=0.5, radius=0.3, confidence=0.9,
    )
    supervisor, decision = _settle({}, (crossing,))
    assert decision.dynamic_overlay == "CROSSING"
    first_clear = supervisor.update(_features(2.5), (), SupervisorHealth(True, False))
    assert first_clear.dynamic_overlay == "CROSSING"
    released = supervisor.update(_features(3.0), (), SupervisorHealth(True, False))
    assert released.dynamic_overlay == "NONE"


def test_fault_immediately_forces_invalid_balanced_none():
    supervisor, decision = _settle(
        dict(front_clearance_m=20.0, obstacle_density=0.0,
             path_curvature=0.0, goal_direction_stability=1.0)
    )
    assert decision.geometry_mode == "CRUISE"
    fault = supervisor.update(
        _features(2.6), (), SupervisorHealth(False, True, "tf_timeout")
    )
    assert not fault.valid
    assert fault.geometry_mode == "BALANCED"
    assert fault.dynamic_overlay == "NONE"
    assert fault.transition_state == "FAULTED"
    assert "tf_timeout" in fault.reason


def test_confirmation_and_dwell_block_mode_chatter():
    supervisor = RuleContextSupervisor(_config())
    observed = []
    for index in range(30):
        cruise = index % 2 == 0
        snapshot = _features(
            index * 0.1,
            front_clearance_m=20.0 if cruise else 3.0,
            obstacle_density=0.0 if cruise else 0.02,
            path_curvature=0.0 if cruise else 0.15,
            goal_direction_stability=1.0 if cruise else 0.5,
        )
        observed.append(
            supervisor.update(snapshot, (), SupervisorHealth(True, False)).geometry_mode
        )
    assert set(observed) == {"BALANCED"}


def test_low_confidence_defaults_to_balanced():
    _, decision = _settle(dict(front_clearance_m=3.0, obstacle_density=0.03,
                               path_curvature=0.12, goal_direction_stability=0.4))
    assert decision.geometry_mode == "BALANCED"
    assert decision.valid


def _repair_config():
    config = _config()
    config["geometry"]["static_dense"].update(
        persistence_density_full=0.10,
        exit_confidence=0.45,
    )
    config["geometry"]["cruise"]["exit_confidence"] = 0.45
    config["geometry"]["corridor"]["exit_confidence"] = 0.50
    config["geometry"]["maneuver"].update(
        exit_confidence=0.45,
        reverse_heading_error_min_rad=2.0,
        reverse_front_clearance_full_m=1.8,
        reverse_front_clearance_max_m=3.0,
        reverse_rear_clearance_full_m=3.0,
    )
    config["transition"]["switch_score_margin"] = 0.12
    return config


def _settle_with_config(config, changes, tracks=()):
    supervisor = RuleContextSupervisor(config)
    decision = None
    for index in range(35):
        decision = supervisor.update(
            _features(index * 0.1, **changes), tracks, SupervisorHealth(True, False)
        )
    return supervisor, decision


def test_repair_profile_does_not_treat_one_persistent_cluster_as_static_dense():
    _, decision = _settle_with_config(
        _repair_config(),
        dict(
            front_clearance_m=20.0,
            obstacle_density=0.01,
            static_persistence=1.0,
            path_curvature=0.0,
            goal_direction_stability=1.0,
        ),
    )
    assert decision.geometry_mode == "CRUISE"


def test_repair_profile_detects_reverse_path_maneuver_without_scene_label():
    _, decision = _settle_with_config(
        _repair_config(),
        dict(
            front_clearance_m=1.7,
            rear_clearance_m=4.0,
            dead_end_score=0.25,
            signed_heading_error_rad=math.pi,
            rear_covered=True,
        ),
    )
    assert decision.geometry_mode == "MANEUVER"
    assert "reverse_path" in decision.reason


def test_repair_profile_holds_active_mode_against_small_score_challenger():
    config = _repair_config()
    config["transition"]["minimum_dwell_s"] = 0.0
    config["transition"]["enter_confirmation_s"] = 0.0
    config["geometry"]["static_dense"]["obstacle_density_full"] = 0.06
    supervisor = RuleContextSupervisor(config)
    cruise = dict(
        front_clearance_m=20.0,
        obstacle_density=0.0,
        path_curvature=0.0,
        goal_direction_stability=1.0,
    )
    assert supervisor.update(
        _features(0.0, **cruise), (), SupervisorHealth(True, False)
    ).geometry_mode == "CRUISE"
    near_boundary = supervisor.update(
        _features(
            0.1,
            front_clearance_m=20.0,
            obstacle_density=0.036,
            static_persistence=0.0,
            path_curvature=0.0,
            goal_direction_stability=1.0,
        ),
        (),
        SupervisorHealth(True, False),
    )
    assert near_boundary.geometry_mode == "CRUISE"
    assert "hysteresis_hold" in near_boundary.reason


def test_pocket_geometry_triggers_maneuver_without_heading_or_scene_label():
    config = _repair_config()
    config["geometry"]["maneuver"].update(
        pocket_front_clearance_full_m=2.0,
        pocket_front_clearance_max_m=3.0,
        pocket_side_clearance_full_m=1.5,
        pocket_side_clearance_max_m=2.2,
        pocket_rear_clearance_full_m=3.0,
    )
    _, decision = _settle_with_config(
        config,
        dict(
            front_clearance_m=1.5,
            left_clearance_m=1.4,
            right_clearance_m=1.4,
            rear_clearance_m=5.0,
            dead_end_score=0.2,
            signed_heading_error_rad=0.0,
        ),
    )
    assert decision.geometry_mode == "MANEUVER"
    assert "pocket" in decision.reason


def test_exit_confirmation_is_longer_than_entry_confirmation():
    config = _repair_config()
    config["transition"].update(
        minimum_dwell_s=0.0,
        enter_confirmation_s=0.2,
        exit_confirmation_s=1.0,
    )
    supervisor = RuleContextSupervisor(config)
    cruise = dict(front_clearance_m=20.0, obstacle_density=0.0,
                  path_curvature=0.0, goal_direction_stability=1.0)
    for index in range(4):
        decision = supervisor.update(
            _features(index * 0.1, **cruise), (), SupervisorHealth(True, False)
        )
    assert decision.geometry_mode == "CRUISE"
    for index in range(4, 12):
        decision = supervisor.update(
            _features(index * 0.1), (), SupervisorHealth(True, False)
        )
    assert decision.geometry_mode == "CRUISE"
    for index in range(12, 16):
        decision = supervisor.update(
            _features(index * 0.1), (), SupervisorHealth(True, False)
        )
    assert decision.geometry_mode == "BALANCED"
