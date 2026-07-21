from teb_mode_manager.mechanism_controller import MechanismSnapshot, RuleMechanismController


def _config():
    return {
        "static_topology": {
            "unlock_front_clearance_m": 0.6,
            "residuals": {"max_vel_x": 0.2},
        },
        "corridor_centerline": {
            "correction_offset_m": 0.10,
            "centered_residuals": {"max_vel_x": 0.3},
            "correction_residuals": {"weight_viapoint": 0.7, "max_vel_x": -0.2},
        },
        "maneuver": {
            "reverse_rear_clearance_min_m": 1.5,
            "reverse_front_clearance_max_m": 3.5,
            "reverse_heading_error_min_rad": 2.0,
            "forward_residuals": {"max_vel_x": 0.2},
            "reverse_residuals": {"max_vel_x": 0.4},
        },
        "dynamic_release": {},
    }


def _snapshot(**changes):
    values = dict(
        front_clearance_m=4.0, rear_clearance_m=4.0,
        left_clearance_m=3.0, right_clearance_m=2.0,
        corridor_center_offset_m=0.0, signed_heading_error_rad=0.0,
        rear_covered=True,
    )
    values.update(changes)
    return MechanismSnapshot(**values)


def test_static_topology_preference_locks_until_infeasible_or_exit():
    controller = RuleMechanismController(_config())
    first = controller.update("STATIC_DENSE", "NONE", _snapshot())
    second = controller.update(
        "STATIC_DENSE", "NONE", _snapshot(left_clearance_m=1.0, right_clearance_m=4.0)
    )
    assert first.topology_preference == second.topology_preference == "LEFT"
    unlocked = controller.update(
        "STATIC_DENSE", "NONE", _snapshot(
            front_clearance_m=0.4, left_clearance_m=1.0, right_clearance_m=4.0
        )
    )
    assert unlocked.topology_preference == "RIGHT"
    assert controller.topology_switch_count == 1
    released = controller.update("BALANCED", "NONE", _snapshot())
    assert not released.topology_locked


def test_corridor_centerline_feedback_is_bounded_and_directional():
    controller = RuleMechanismController(_config())
    centered = controller.update("CORRIDOR", "NONE", _snapshot())
    correcting = controller.update(
        "CORRIDOR", "NONE", _snapshot(corridor_center_offset_m=0.2)
    )
    assert centered.corridor_centerline_active
    assert centered.residuals["max_vel_x"] > 0.0
    assert correcting.residuals["max_vel_x"] < 0.0
    assert correcting.residuals["weight_viapoint"] > 0.0


def test_maneuver_selects_reverse_only_with_geometry_and_rear_coverage():
    controller = RuleMechanismController(_config())
    reverse = controller.update(
        "MANEUVER", "NONE", _snapshot(
            front_clearance_m=2.0, rear_clearance_m=3.0,
            signed_heading_error_rad=3.0,
        )
    )
    assert reverse.maneuver_reverse
    no_rear = controller.update(
        "MANEUVER", "NONE", _snapshot(
            front_clearance_m=2.0, rear_clearance_m=3.0,
            signed_heading_error_rad=3.0, rear_covered=False,
        )
    )
    assert not no_rear.maneuver_reverse
