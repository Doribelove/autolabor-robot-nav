"""Rigid planar reference-point conversions; pure components, not live wiring.

World poses use a common world frame; twists use body-aligned axes at their
named reference point. Preserve measured lateral slip. Never use these helpers
to replace actual ODE states with a no-slip prediction. M2 base_link is midway
between axles, so rear axle is (-wheelbase/2, 0) in base coordinates.
"""
import math


def _finite(values, size):
    result = tuple(float(value) for value in values)
    if len(result) != size or not all(math.isfinite(v) for v in result):
        raise ValueError('wrong-sized or non-finite planar state')
    return result


def _half_wheelbase(wheelbase_m):
    wheelbase_m = float(wheelbase_m)
    if not math.isfinite(wheelbase_m) or wheelbase_m <= 0:
        raise ValueError('wheelbase must be finite and positive')
    return wheelbase_m / 2


def shift_pose(pose_xyyaw, offset_body_xy):
    """Pose at Q=P+R(yaw)*offset, with unchanged body orientation."""
    x, y, yaw = _finite(pose_xyyaw, 3)
    dx, dy = _finite(offset_body_xy, 2)
    c, s = math.cos(yaw), math.sin(yaw)
    return x + c*dx - s*dy, y + s*dx + c*dy, yaw


def shift_body_twist(twist_vx_vy_wz, offset_body_xy):
    """Exact rigid-body point velocity: v_Q=v_P+omega cross offset."""
    vx, vy, wz = _finite(twist_vx_vy_wz, 3)
    dx, dy = _finite(offset_body_xy, 2)
    return vx - wz*dy, vy + wz*dx, wz


def base_pose_to_rear(pose_xyyaw, wheelbase_m):
    return shift_pose(pose_xyyaw, (-_half_wheelbase(wheelbase_m), 0))


def rear_pose_to_base(pose_xyyaw, wheelbase_m):
    return shift_pose(pose_xyyaw, (_half_wheelbase(wheelbase_m), 0))


def base_twist_to_rear(twist_vx_vy_wz, wheelbase_m):
    return shift_body_twist(twist_vx_vy_wz, (-_half_wheelbase(wheelbase_m), 0))


def rear_twist_to_base(twist_vx_vy_wz, wheelbase_m):
    return shift_body_twist(twist_vx_vy_wz, (_half_wheelbase(wheelbase_m), 0))


def base_local_point_to_rear(point_body_xy, wheelbase_m):
    """Represent the same footprint point relative to the rear origin."""
    x, y = _finite(point_body_xy, 2)
    return x + _half_wheelbase(wheelbase_m), y


def rear_local_point_to_base(point_body_xy, wheelbase_m):
    x, y = _finite(point_body_xy, 2)
    return x - _half_wheelbase(wheelbase_m), y


LIVE_INTEGRATION_STATUS = 'component_only_not_wired_to_planner_or_controller'
