"""Pure R6 design reference for evaluator-aligned dynamic conflict evidence.

This module is deliberately not wired to a ROS node, launch file, runtime
profile, or execution authorization.  It leaves the frozen legacy supervisor
untouched and makes the proposed R6 categorical factor explicit.
"""

import copy
from dataclasses import dataclass
import math
from typing import Iterable, Optional, Sequence, Tuple

from nav_world_model.risk_evidence import RelativeTrack, relative_collision_ttc

from .rule_supervisor import RuleContextSupervisor, RuntimeTrack


LEGACY_ESTIMATOR_ID = "legacy_class_conditioned_geometry_v1"
ALIGNED_ESTIMATOR_ID = "shared_circle_envelope_first_contact_v1"
ESTIMATOR_IDS = (LEGACY_ESTIMATOR_ID, ALIGNED_ESTIMATOR_ID)
DEFAULT_TRACK_RADIUS_M = 0.25
FROZEN_HORIZON_S = 5.0
FROZEN_MINIMUM_TRACK_CONFIDENCE = 0.45
FROZEN_ROBOT_RADIUS_M = 0.62
FROZEN_MINIMUM_RELATIVE_SPEED_MPS = 0.05
FROZEN_LEGACY_CLOSEST_APPROACH_M = 1.35
FROZEN_OVERLAY_RELEASE_CONFIRMATION_S = 0.20


class R6SemanticError(ValueError):
    """Raised when the proposed R6 semantic boundary is ambiguous."""


@dataclass(frozen=True)
class ConflictDecision:
    """One pure overlay decision and its shared circle-contact evidence."""

    overlay: str
    reason: str
    track_id: Optional[int]
    ttc_s: Optional[float]
    conflict_present: bool


@dataclass(frozen=True)
class FootprintRuntimeTrack:
    """Pre-adapter track retaining the measured footprint geometry."""

    track_id: int
    motion_class: str
    x: float
    y: float
    vx: float
    vy: float
    footprint: Sequence
    confidence: float


def _require_finite_positive(value, label):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        or float(value) <= 0.0
    ):
        raise R6SemanticError("{} must be finite and positive".format(label))
    return float(value)


def footprint_radius(points: Iterable, estimator_id: str) -> float:
    """Derive the track radius as an atomic part of the estimator semantics.

    The legacy adapter used only the footprint x extent.  The aligned estimator
    uses the footprint circumradius, exactly matching the frozen evaluator
    producer.  Points may be ``(x, y)`` pairs or objects exposing ``x``/``y``.
    """

    if estimator_id not in ESTIMATOR_IDS:
        raise R6SemanticError("unknown conflict estimator: {}".format(estimator_id))
    values = []
    for point in points:
        if hasattr(point, "x") and hasattr(point, "y"):
            x, y = point.x, point.y
        else:
            try:
                x, y = point[0], point[1]
            except (TypeError, IndexError) as exc:
                raise R6SemanticError("footprint point must contain x/y") from exc
        if (
            isinstance(x, bool)
            or isinstance(y, bool)
            or not isinstance(x, (int, float))
            or not isinstance(y, (int, float))
            or not math.isfinite(float(x))
            or not math.isfinite(float(y))
        ):
            raise R6SemanticError("footprint coordinates must be finite")
        if estimator_id == LEGACY_ESTIMATOR_ID:
            values.append(abs(float(x)))
        else:
            values.append(math.hypot(float(x), float(y)))
    if not values:
        return DEFAULT_TRACK_RADIUS_M
    radius = max(values)
    if radius <= 0.0:
        raise R6SemanticError("tracked footprint radius must be positive")
    return radius


def runtime_track_from_footprint(
    track: FootprintRuntimeTrack, estimator_id: str
) -> RuntimeTrack:
    """Build the exact RuntimeTrack consumed by one estimator level.

    Requiring a footprint-bearing input prevents a future R6 node from silently
    carrying forward the frozen legacy node's x-extent radius when the aligned
    level is selected.
    """

    if not isinstance(track, FootprintRuntimeTrack):
        raise R6SemanticError(
            "R6 supervisor requires a footprint-bearing runtime track"
        )
    return RuntimeTrack(
        track_id=track.track_id,
        motion_class=track.motion_class,
        x=track.x,
        y=track.y,
        vx=track.vx,
        vy=track.vy,
        radius=footprint_radius(track.footprint, estimator_id),
        confidence=track.confidence,
    )


def evaluator_aligned_conflict(
    tracks: Sequence[RuntimeTrack],
    *,
    robot_radius_m: float,
    horizon_s: float,
    minimum_track_confidence: float,
    minimum_relative_speed_mps: float,
) -> ConflictDecision:
    """Select the first finite circle-envelope contact from runtime tracks.

    Motion class labels only select the overlay taxonomy after finite TTC has
    established conflict eligibility.  The conflict primitive is the frozen
    evaluator helper, not a reimplementation.
    """

    robot_radius = _require_finite_positive(robot_radius_m, "robot radius")
    horizon = _require_finite_positive(horizon_s, "TTC horizon")
    minimum_speed = _require_finite_positive(
        minimum_relative_speed_mps, "minimum relative speed"
    )
    if (
        isinstance(minimum_track_confidence, bool)
        or not isinstance(minimum_track_confidence, (int, float))
        or not math.isfinite(float(minimum_track_confidence))
        or not 0.0 <= float(minimum_track_confidence) <= 1.0
    ):
        raise R6SemanticError("minimum track confidence must be in [0, 1]")
    minimum_confidence = float(minimum_track_confidence)
    mapping = {
        "CROSSING": "CROSSING",
        "HEAD_ON": "HEAD_ON",
        "FOLLOWING": "FOLLOW",
        "UNKNOWN": "OVERTAKE_OR_YIELD",
    }
    priority = {
        "HEAD_ON": 0,
        "CROSSING": 1,
        "FOLLOW": 2,
        "OVERTAKE_OR_YIELD": 3,
    }
    candidates = []
    for track in tracks:
        if track.motion_class not in (
            "UNKNOWN",
            "STATIONARY",
            "CROSSING",
            "HEAD_ON",
            "FOLLOWING",
            "DEPARTING",
        ):
            raise R6SemanticError(
                "unsupported motion class: {}".format(track.motion_class)
            )
        relative = RelativeTrack(
            x=float(track.x),
            y=float(track.y),
            vx=float(track.vx),
            vy=float(track.vy),
            radius=float(track.radius),
            confidence=float(track.confidence),
            motion_class=track.motion_class,
        )
        ttc = relative_collision_ttc(
            relative,
            robot_radius_m=robot_radius,
            horizon_s=horizon,
            minimum_confidence=minimum_confidence,
            minimum_relative_speed_mps=minimum_speed,
        )
        overlay = mapping.get(track.motion_class)
        if ttc is not None and overlay is not None:
            candidates.append((ttc, priority[overlay], track.track_id, overlay))
    if not candidates:
        return ConflictDecision(
            overlay="NONE",
            reason="no_finite_circle_envelope_contact",
            track_id=None,
            ttc_s=None,
            conflict_present=False,
        )
    ttc, _, track_id, overlay = min(candidates)
    return ConflictDecision(
        overlay=overlay,
        reason="circle_envelope_first_contact_track_{}".format(track_id),
        track_id=track_id,
        ttc_s=ttc,
        conflict_present=True,
    )


class R6RelativeTTCSupervisor(RuleContextSupervisor):
    """Versioned, pure design reference with one categorical semantic factor."""

    def __init__(self, config):
        frozen_config = copy.deepcopy(config)
        super().__init__(frozen_config)
        dynamic = frozen_config["dynamic"]
        self.conflict_estimator_id = dynamic["conflict_estimator_id"]
        if self.conflict_estimator_id not in ESTIMATOR_IDS:
            raise R6SemanticError("R6 conflict estimator id is invalid")
        self.robot_radius_m = _require_finite_positive(
            dynamic["robot_radius_m"], "robot radius"
        )
        self.minimum_relative_speed_mps = _require_finite_positive(
            dynamic["minimum_relative_speed_mps"], "minimum relative speed"
        )
        self.horizon_s = _require_finite_positive(
            dynamic["predicted_ttc_max_s"], "TTC horizon"
        )
        self.minimum_track_confidence = float(
            dynamic["minimum_track_confidence"]
        )
        frozen_values = (
            (self.horizon_s, FROZEN_HORIZON_S, "TTC horizon"),
            (
                self.minimum_track_confidence,
                FROZEN_MINIMUM_TRACK_CONFIDENCE,
                "minimum track confidence",
            ),
            (self.robot_radius_m, FROZEN_ROBOT_RADIUS_M, "robot radius"),
            (
                self.minimum_relative_speed_mps,
                FROZEN_MINIMUM_RELATIVE_SPEED_MPS,
                "minimum relative speed",
            ),
            (
                float(dynamic["closest_approach_max_m"]),
                FROZEN_LEGACY_CLOSEST_APPROACH_M,
                "legacy closest approach",
            ),
            (
                float(
                    frozen_config["transition"][
                        "overlay_release_confirmation_s"
                    ]
                ),
                FROZEN_OVERLAY_RELEASE_CONFIRMATION_S,
                "overlay release confirmation",
            ),
        )
        for actual, expected, label in frozen_values:
            if actual != expected:
                raise R6SemanticError(
                    "{} is frozen at {}".format(label, expected)
                )

    def _overlay(
        self, tracks: Sequence[FootprintRuntimeTrack]
    ) -> Tuple[str, str]:
        adapted_tracks = [
            runtime_track_from_footprint(track, self.conflict_estimator_id)
            for track in tracks
        ]
        if self.conflict_estimator_id == LEGACY_ESTIMATOR_ID:
            return super()._overlay(adapted_tracks)
        decision = evaluator_aligned_conflict(
            adapted_tracks,
            robot_radius_m=self.robot_radius_m,
            horizon_s=self.horizon_s,
            minimum_track_confidence=self.minimum_track_confidence,
            minimum_relative_speed_mps=self.minimum_relative_speed_mps,
        )
        return decision.overlay, decision.reason
