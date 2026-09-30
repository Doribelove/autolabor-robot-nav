"""Sensor-only directional reachable-occupancy verifier for V6-04."""

import math
from collections import deque

import numpy as np

from .runtime_adapter import SensorRuntimeAdapter


class DirectionalSafetyAdapter(SensorRuntimeAdapter):
    """Track moving scan surfaces and verify candidate-conditioned backups.

    Static geometry remains protected by the original 0.12 m inflation.  A
    moving cluster uses the evaluator-aligned 0.33 m inflation and a bounded
    constant-velocity reachable tube.  Only scan and odometry are consumed.
    """

    def __init__(self, actor=None, advantage_gate_margin=0.10):
        super().__init__(actor=actor, advantage_gate_margin=advantage_gate_margin)
        self.dynamic_surface_inflation_m = 0.33
        self.dynamic_acceleration_bound_mps2 = 0.40
        self.dynamic_uncertainty_rate_mps = 0.12
        self.dynamic_association_radius_m = 0.72
        self.cloud_history = deque(maxlen=6)
        self.track_history = deque(maxlen=6)
        self.dynamic_center = None
        self.dynamic_velocity = np.zeros(2, dtype=np.float64)
        self.dynamic_track_valid = False

    def _world_cloud(self, scan, pose):
        scan = np.asarray(scan, dtype=np.float64)
        valid = scan < self.maximum_range_m - 0.05
        angles = float(pose[2]) + self.beam_angles
        return np.stack(
            (
                float(pose[0]) + scan[valid] * np.cos(angles[valid]),
                float(pose[1]) + scan[valid] * np.sin(angles[valid]),
            ),
            axis=1,
        ) if np.any(valid) else np.empty((0, 2), dtype=np.float64)

    def append_scan_with_pose(self, scan, pose, stamp_s):
        super().append_scan(scan)
        cloud = self._world_cloud(scan, pose)
        stamp_s = float(stamp_s)
        candidate = None
        if len(cloud) and self.dynamic_center is not None:
            distances = np.linalg.norm(cloud - self.dynamic_center[None, :], axis=1)
            associated = cloud[distances <= self.dynamic_association_radius_m]
            if len(associated):
                candidate = np.median(associated, axis=0)
        if candidate is None and len(cloud) and self.cloud_history:
            old_stamp, old_cloud = self.cloud_history[0]
            if len(old_cloud) and stamp_s - old_stamp >= 0.35:
                pairwise = np.linalg.norm(cloud[:, None, :] - old_cloud[None, :, :], axis=2)
                novelty = np.min(pairwise, axis=1)
                ranges = np.linalg.norm(cloud - np.asarray(pose[:2])[None, :], axis=1)
                moving = cloud[(novelty >= 0.055) & (ranges <= 4.0)]
                if len(moving):
                    # Robustly retain the densest local group around the median.
                    centre = np.median(moving, axis=0)
                    local = moving[np.linalg.norm(moving - centre[None, :], axis=1) <= 0.65]
                    if len(local) >= 1:
                        candidate = np.median(local, axis=0)
        if candidate is not None:
            self.dynamic_center = np.asarray(candidate, dtype=np.float64)
            self.track_history.append((stamp_s, self.dynamic_center.copy()))
            if len(self.track_history) >= 2:
                old_stamp, old_center = self.track_history[0]
                dt = stamp_s - old_stamp
                if dt >= 0.20:
                    velocity = (self.dynamic_center - old_center) / dt
                    speed = float(np.linalg.norm(velocity))
                    if speed > 1.50:
                        velocity *= 1.50 / speed
                    self.dynamic_velocity = 0.45 * self.dynamic_velocity + 0.55 * velocity
                    self.dynamic_track_valid = True
        self.cloud_history.append((stamp_s, cloud))

    def _partition_points(self, scan_points):
        if not self.dynamic_track_valid or self.dynamic_center is None or not len(scan_points):
            return scan_points, np.empty((0, 2), dtype=np.float64)
        distances = np.linalg.norm(scan_points - self.dynamic_center[None, :], axis=1)
        dynamic = distances <= self.dynamic_association_radius_m
        return scan_points[~dynamic], scan_points[dynamic]

    def _clearance(self, preview, scan_points, dynamic_reachable=False):
        static_points, dynamic_points = self._partition_points(scan_points)
        minimum = float("inf")
        for index, state in enumerate(preview):
            time_s = index * self.preview_dt_s
            minimum = min(minimum, self.wall_half_width_m - abs(state[1]) - self.robot_radius_m)
            if len(static_points):
                distance = np.hypot(static_points[:, 0] - state[0], static_points[:, 1] - state[1])
                minimum = min(minimum, float(np.min(distance)) - self.robot_radius_m - self.shield_inflation_m)
            if len(dynamic_points):
                predicted = dynamic_points
                uncertainty = 0.0
                if dynamic_reachable:
                    predicted = dynamic_points + self.dynamic_velocity[None, :] * time_s
                    uncertainty = min(
                        self.maximum_dynamic_expansion_m,
                        self.dynamic_uncertainty_rate_mps * time_s
                        + 0.5 * self.dynamic_acceleration_bound_mps2 * time_s * time_s,
                    )
                distance = np.hypot(predicted[:, 0] - state[0], predicted[:, 1] - state[1])
                minimum = min(
                    minimum,
                    float(np.min(distance))
                    - self.robot_radius_m
                    - self.dynamic_surface_inflation_m
                    - uncertainty,
                )
        return minimum

    def tracker_audit(self):
        return {
            "valid": bool(self.dynamic_track_valid),
            "center_world_m": self.dynamic_center.tolist() if self.dynamic_center is not None else None,
            "velocity_world_mps": self.dynamic_velocity.tolist(),
            "speed_mps": float(np.linalg.norm(self.dynamic_velocity)),
            "source": "LaserScan_plus_Odometry_only",
        }
