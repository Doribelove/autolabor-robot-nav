import numpy as np

from v5_e2e_scope_rl.runtime_adapter import SensorRuntimeAdapter
from v5_e2e_scope_rl.directional_safety import DirectionalSafetyAdapter


def test_runtime_observation_schema_and_stop_invariant():
    planner = SensorRuntimeAdapter()
    planner.append_scan(np.full(72, 8.0, dtype=np.float32))
    observation, previews = planner.build((-2.6, 0.0, 0.0), 0.8, 0.0, 0.0)
    assert observation["scans"].shape == (6, 72)
    assert observation["robot"].shape == (10,)
    assert observation["candidates"].shape == (8, 7)
    assert observation["action_mask"].shape == (8,)
    assert observation["action_mask"][0] == 1
    assert len(previews) == 8


def test_runtime_scan_resampling_is_finite():
    planner = SensorRuntimeAdapter()
    ranges = np.linspace(0.5, 12.0, 1080)
    ranges[10] = np.inf
    scan = planner.resample_scan(ranges, -np.pi, 2.0 * np.pi / 1080.0, 12.0)
    assert scan.shape == (72,)
    assert np.isfinite(scan).all()
    assert np.all(scan >= 0.02)
    assert np.all(scan <= 8.0)


def test_directional_tracker_uses_scan_and_pose_motion():
    planner = DirectionalSafetyAdapter()
    angles = planner.beam_angles
    for index in range(6):
        scan = np.full(72, 8.0, dtype=np.float32)
        # Compact surface moves in +y in world coordinates.
        target_angle = np.arctan2(-0.9 + 0.12 * index, 2.0)
        beam = int(np.argmin(np.abs(angles - target_angle)))
        scan[max(0, beam - 1) : min(72, beam + 2)] = np.hypot(2.0, -0.9 + 0.12 * index)
        planner.append_scan_with_pose(scan, (0.0, 0.0, 0.0), 0.2 * index)
    audit = planner.tracker_audit()
    assert audit["valid"]
    assert audit["velocity_world_mps"][1] > 0.05
    assert audit["source"] == "LaserScan_plus_Odometry_only"
