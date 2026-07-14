import math
from pathlib import Path

import pytest
import yaml

from nav_world_model import (
    Detection,
    MultiObjectTracker,
    Point2,
    RobotState,
    ScanFrame,
    ScanValidationError,
    compute_local_geometry,
    extract_detections,
    scan_to_local_points,
    validate_scan,
)


def _scan(ranges=None):
    count = 720
    increment = 2.0 * math.pi / (count - 1)
    return ScanFrame(
        stamp_s=1.0,
        frame_id="laser_link",
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=increment,
        range_min=0.1,
        range_max=30.0,
        ranges=tuple(ranges if ranges is not None else [30.0] * count),
    )


def _config():
    path = Path(__file__).resolve().parents[1] / "config/v2_03_candidate.yaml"
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _tracker():
    config = _config()
    tracker, prediction = config["tracker"], config["prediction"]
    return MultiObjectTracker(
        association_gate_m=tracker["association_gate_m"],
        alpha=tracker["alpha"],
        beta=tracker["beta"],
        minimum_confirmed_hits=tracker["minimum_confirmed_hits"],
        maximum_misses=tracker["maximum_misses"],
        maximum_dt_s=tracker["maximum_dt_s"],
        stationary_speed_max_mps=tracker["stationary_speed_max_mps"],
        dynamic_speed_min_mps=tracker["dynamic_speed_min_mps"],
        prediction_horizon_s=prediction["horizon_s"],
        prediction_step_s=prediction["step_s"],
        confidence_decay_per_s=prediction["confidence_decay_per_s"],
        crossing_lateral_speed_min_mps=tracker["crossing_lateral_speed_min_mps"],
        crossing_path_half_width_m=tracker["crossing_path_half_width_m"],
    )


def test_scan_validation_is_strict_and_all_directions_are_covered():
    scan = _scan()
    validate_scan(scan, 720)
    bad = ScanFrame(**{**scan.__dict__, "angle_max": scan.angle_max + 0.2})
    with pytest.raises(ScanValidationError):
        validate_scan(bad, 720)
    points = scan_to_local_points(scan)
    estimate = compute_local_geometry(
        scan, points, (), (0.0, 0.0, 0.0, 1.0),
        sector_half_width_rad=math.pi / 12.0,
        density_radius_m=8.0,
        robot_radius_m=0.62,
        corridor_parameters=_config()["corridor"],
    )
    assert estimate.front_covered
    assert estimate.left_covered
    assert estimate.right_covered
    assert estimate.rear_covered


def test_ordered_clustering_rejects_wall_sized_objects():
    points = [None] * 30
    for index in range(6):
        points[2 + index] = Point2(5.0, -0.25 + index * 0.1)
    for index in range(15):
        points[12 + index] = Point2(2.0 + index * 0.3, 1.0)
    detections = extract_detections(
        points, maximum_gap_m=0.35, minimum_points=3,
        maximum_diameter_m=1.6, maximum_range_m=15.0, origin=Point2(0.0, 0.0)
    )
    assert len(detections) == 1
    assert detections[0].point_count == 6


def test_corridor_line_fit_recovers_width_and_center():
    scan = _scan()
    points = []
    for index in range(31):
        x = -1.0 + 0.2 * index
        points.extend((Point2(x, 1.0), Point2(x, -1.0)))
    estimate = compute_local_geometry(
        scan, tuple(points), (), (0.0, 0.0, 0.0, 1.0),
        sector_half_width_rad=math.pi / 12.0,
        density_radius_m=8.0,
        robot_radius_m=0.62,
        corridor_parameters=_config()["corridor"],
    )
    assert estimate.corridor_width_m == pytest.approx(2.0, abs=1.0e-6)
    assert estimate.corridor_center_offset_m == pytest.approx(0.0, abs=1.0e-6)
    assert estimate.corridor_parallel_confidence > 0.95


def test_tracker_position_prediction_and_identity_gates():
    tracker = _tracker()
    position_errors, prediction_errors, ids = [], [], []
    for index in range(50):
        stamp = index * 0.1
        truth_y = -2.0 + stamp
        noise = 0.03 * math.sin(index * 0.7)
        estimates = tracker.update(
            [Detection(10.0, truth_y + noise, 0.30, 6)], stamp, RobotState()
        )
        if not estimates:
            continue
        estimate = estimates[0]
        ids.append(estimate.track_id)
        position_errors.append(math.hypot(estimate.x - 10.0, estimate.y - truth_y))
        prediction = next(item for item in estimate.predictions
                          if item.time_from_start_s == pytest.approx(1.0))
        if index >= 12:
            prediction_errors.append(
                math.hypot(prediction.x - 10.0, prediction.y - (truth_y + 1.0))
            )
    position_rmse = math.sqrt(sum(value * value for value in position_errors) / len(position_errors))
    prediction_rmse = math.sqrt(sum(value * value for value in prediction_errors) / len(prediction_errors))
    assert position_rmse < 0.20
    assert prediction_rmse < 0.35
    assert len(set(ids)) == 1


def test_tracker_rejects_time_regression_and_deletes_missed_track():
    tracker = _tracker()
    for index in range(3):
        tracker.update([Detection(2.0, 0.0, 0.2, 5)], index * 0.1, RobotState())
    with pytest.raises(ValueError):
        tracker.update([], 0.1, RobotState())
    for index in range(3, 10):
        estimates = tracker.update([], index * 0.1, RobotState())
    assert estimates == ()
