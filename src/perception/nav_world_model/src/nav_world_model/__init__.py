"""Pure-Python core for the FAM-TEB V2 local world model."""

from .core import (
    Detection,
    GeometryEstimate,
    MultiObjectTracker,
    Point2,
    Prediction,
    RobotState,
    ScanFrame,
    ScanValidationError,
    TrackEstimate,
    compute_local_geometry,
    compute_path_metrics,
    extract_detections,
    scan_to_local_points,
    transform_points,
    validate_scan,
)
from .teb_bridge import FixedFrameObstacle, local_track_to_fixed
from .risk_evidence import (
    ClearanceEvidence,
    RelativeTrack,
    classify_ttc_evidence,
    earliest_relative_ttc,
    oriented_box_clearance,
    rectangular_footprint_clearance,
)

__all__ = [
    "Detection",
    "GeometryEstimate",
    "MultiObjectTracker",
    "Point2",
    "Prediction",
    "RobotState",
    "ScanFrame",
    "ScanValidationError",
    "TrackEstimate",
    "compute_local_geometry",
    "compute_path_metrics",
    "extract_detections",
    "scan_to_local_points",
    "transform_points",
    "validate_scan",
    "FixedFrameObstacle",
    "local_track_to_fixed",
]
