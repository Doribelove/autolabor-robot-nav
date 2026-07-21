import math

import pytest

from teb_rl_tuner.state_builder import (
    HistoryWindow,
    ScanAngularMetadata,
    StateBuilder,
    StateError,
    lidar_sector_quantiles,
)


def test_lidar_sector_quantile_ignores_invalid_rays():
    sectors = lidar_sector_quantiles(
        [1.0, float("nan"), 2.0, float("inf"), 3.0, 4.0, 20.0, 5.0],
        range_min=0.1,
        range_max=10.0,
        sector_count=4,
        quantile=0.0,
    )
    assert sectors == pytest.approx((1.0, 2.0, 3.0, 5.0))
    assert all(math.isfinite(value) for value in sectors)


def test_builder_and_k4_history_are_ordered_and_fixed_dimension():
    builder = StateBuilder(
        feature_order=("goal_distance", "linear_velocity"),
        required_streams=("scan", "odom", "local_plan"),
        sector_count=4,
        max_sync_skew_s=0.05,
    )
    history = HistoryWindow(k=4)
    for index in range(4):
        stamp = 10.0 + index
        frame = builder.build(
            stamps={"scan": stamp, "odom": stamp + 0.01, "local_plan": stamp + 0.02},
            ranges=[1.0] * 8,
            range_min=0.1,
            range_max=10.0,
            features={"goal_distance": 5.0 - index, "linear_velocity": 0.2},
            validity={"scan": True, "tf": True, "localization": True, "interface": True},
        )
        history.append(frame)
    assert history.ready
    assert len(history.stacked()) == 4 * (4 + 2)

    newer = builder.build(
        stamps={"scan": 14.0, "odom": 14.01, "local_plan": 14.02},
        ranges=[2.0] * 8,
        range_min=0.1,
        range_max=10.0,
        features={"goal_distance": 1.0, "linear_velocity": 0.1},
    )
    history.append(newer)
    assert len(history.frames) == 4
    assert history.frames[0].timestamp == pytest.approx(11.02)


def test_invalid_history_cannot_be_used_for_policy_state():
    builder = StateBuilder(("x",), ("scan",), sector_count=1)
    history = HistoryWindow(k=1)
    frame = builder.build(
        {"scan": 1.0}, [1.0], 0.1, 10.0, {"x": 0.0}, {"tf": False}
    )
    assert frame.invalid_reasons == ("tf",)
    history.append(frame)
    with pytest.raises(StateError):
        history.stacked()


def test_scan_metadata_preserves_angles_and_audits_rear_coverage_without_v1_resize():
    builder = StateBuilder(("x",), ("scan",), sector_count=4)
    full = ScanAngularMetadata(
        stamp=1.0,
        frame_id="laser_link",
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=2.0 * math.pi / 8.0,
        ray_count=8,
    )
    frame = builder.build(
        {"scan": 1.0},
        [1.0] * 8,
        0.1,
        10.0,
        {"x": 0.0},
        scan_metadata=full,
    )
    assert len(frame.vector) == 5
    assert frame.scan_metadata == full
    assert frame.directional_scan_coverage.front is True
    assert frame.directional_scan_coverage.left is True
    assert frame.directional_scan_coverage.right is True
    assert frame.directional_scan_coverage.rear is True

    front_only = ScanAngularMetadata(
        stamp=2.0,
        frame_id="front_laser",
        angle_min=-math.pi / 2.0,
        angle_max=math.pi / 2.0,
        angle_increment=math.pi / 7.0,
        ray_count=8,
    )
    assert front_only.directional_coverage().front is True
    assert front_only.directional_coverage().rear is False


def test_scan_metadata_rejects_inconsistent_ray_count_and_stamp():
    builder = StateBuilder(("x",), ("scan",), sector_count=1, max_sync_skew_s=0.05)
    metadata = ScanAngularMetadata(
        stamp=1.0,
        frame_id="laser_link",
        angle_min=-1.0,
        angle_max=1.0,
        angle_increment=2.0 / 3.0,
        ray_count=4,
    )
    with pytest.raises(StateError, match="ray_count"):
        builder.build(
            {"scan": 1.0}, [1.0] * 3, 0.1, 10.0, {"x": 0.0},
            scan_metadata=metadata,
        )
    with pytest.raises(StateError, match="not synchronized"):
        builder.build(
            {"scan": 1.2}, [1.0] * 4, 0.1, 10.0, {"x": 0.0},
            scan_metadata=metadata,
        )
