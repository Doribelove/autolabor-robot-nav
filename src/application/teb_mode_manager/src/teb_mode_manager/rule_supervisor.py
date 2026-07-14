"""Label-free deterministic mode and dynamic-overlay supervisor."""

from dataclasses import dataclass
import math
from typing import Dict, Optional, Sequence, Tuple


GEOMETRY_MODES = ("BALANCED", "CRUISE", "STATIC_DENSE", "CORRIDOR", "MANEUVER")
DYNAMIC_OVERLAYS = ("NONE", "CROSSING", "HEAD_ON", "FOLLOW", "OVERTAKE_OR_YIELD")


def _clamp(value: float) -> float:
    return max(0.0, min(1.0, float(value)))


@dataclass(frozen=True)
class FeatureSnapshot:
    world_model_seq: int
    stamp_s: float
    front_clearance_m: float
    rear_clearance_m: float
    obstacle_density: float
    static_persistence: float
    corridor_width_m: float
    corridor_parallel_confidence: float
    dead_end_score: float
    path_curvature: float
    goal_direction_stability: float
    rear_covered: bool
    signed_heading_error_rad: float = 0.0
    left_clearance_m: float = 0.0
    right_clearance_m: float = 0.0


@dataclass(frozen=True)
class RuntimeTrack:
    track_id: int
    motion_class: str
    x: float
    y: float
    vx: float
    vy: float
    radius: float
    confidence: float


@dataclass(frozen=True)
class SupervisorHealth:
    valid: bool
    stale: bool
    fault_reason: str = ""


@dataclass(frozen=True)
class TransitionEvent:
    mode_seq: int
    from_mode: str
    to_mode: str
    overlay: str
    state: str
    progress: float
    minimum_dwell_remaining_s: float
    reason: str
    valid: bool


@dataclass(frozen=True)
class ContextDecision:
    world_model_seq: int
    mode_seq: int
    geometry_mode: str
    dynamic_overlay: str
    transition_state: str
    confidence: float
    mode_dwell_s: float
    minimum_dwell_remaining_s: float
    valid: bool
    reason: str
    transition: Optional[TransitionEvent] = None


class RuleContextSupervisor:
    """Runtime-label-free rules with confirmation, dwell, and fault priority."""

    def __init__(self, config: Dict):
        self.config = config
        self.minimum_confidence = float(config["transition"]["minimum_mode_confidence"])
        self.minimum_dwell_s = float(config["transition"]["minimum_dwell_s"])
        self.enter_confirmation_s = float(config["transition"]["enter_confirmation_s"])
        self.exit_confirmation_s = float(
            config["transition"].get("exit_confirmation_s", self.enter_confirmation_s)
        )
        self.blend_duration_s = float(config["transition"]["blend_duration_s"])
        self.overlay_release_confirmation_s = float(
            config["transition"]["overlay_release_confirmation_s"]
        )
        self.switch_score_margin = float(
            config["transition"].get("switch_score_margin", 0.0)
        )
        self.current_mode = "BALANCED"
        self.current_overlay = "NONE"
        self.mode_seq = 0
        self.mode_since_s: Optional[float] = None
        self.candidate_mode = "BALANCED"
        self.candidate_since_s: Optional[float] = None
        self.overlay_candidate = "NONE"
        self.overlay_candidate_since_s: Optional[float] = None
        self.transition_from = "BALANCED"
        self.transition_started_s: Optional[float] = None
        self.last_stamp_s: Optional[float] = None

    def reset(self) -> None:
        self.__init__(self.config)

    def update(
        self,
        snapshot: FeatureSnapshot,
        tracks: Sequence[RuntimeTrack],
        health: SupervisorHealth,
    ) -> ContextDecision:
        stamp_s = float(snapshot.stamp_s)
        if not math.isfinite(stamp_s) or stamp_s < 0.0:
            raise ValueError("supervisor stamp is invalid")
        if self.last_stamp_s is not None and stamp_s < self.last_stamp_s:
            raise ValueError("supervisor time moved backwards")
        self.last_stamp_s = stamp_s
        if self.mode_since_s is None:
            self.mode_since_s = stamp_s
        if not health.valid or health.stale:
            reason = health.fault_reason or "world_model_invalid_or_stale"
            transition = None
            if self.current_mode != "BALANCED" or self.current_overlay != "NONE":
                self.mode_seq += 1
                transition = TransitionEvent(
                    mode_seq=self.mode_seq,
                    from_mode=self.current_mode,
                    to_mode="BALANCED",
                    overlay="NONE",
                    state="FAULTED",
                    progress=0.0,
                    minimum_dwell_remaining_s=0.0,
                    reason=reason,
                    valid=False,
                )
            self.current_mode = "BALANCED"
            self.current_overlay = "NONE"
            self.mode_since_s = stamp_s
            self.candidate_mode = "BALANCED"
            self.candidate_since_s = stamp_s
            self.transition_started_s = None
            return ContextDecision(
                world_model_seq=snapshot.world_model_seq,
                mode_seq=self.mode_seq,
                geometry_mode="BALANCED",
                dynamic_overlay="NONE",
                transition_state="FAULTED",
                confidence=0.0,
                mode_dwell_s=0.0,
                minimum_dwell_remaining_s=0.0,
                valid=False,
                reason=reason,
                transition=transition,
            )

        desired_mode, mode_confidence, mode_reason, scores = self._geometry(snapshot)
        desired_mode, mode_confidence, mode_reason = self._apply_mode_hysteresis(
            desired_mode, mode_confidence, mode_reason, scores
        )
        desired_overlay, overlay_reason = self._overlay(tracks)
        committed_transition = self._advance_mode(
            desired_mode, mode_confidence, mode_reason, stamp_s
        )
        self._advance_overlay(desired_overlay, stamp_s)
        mode_since = self.mode_since_s if self.mode_since_s is not None else stamp_s
        dwell = max(0.0, stamp_s - mode_since)
        dwell_remaining = max(0.0, self.minimum_dwell_s - dwell)
        if self.transition_started_s is not None:
            elapsed = stamp_s - self.transition_started_s
            if elapsed < self.blend_duration_s:
                state = "ENTERING"
            elif dwell_remaining > 0.0:
                state = "HOLDING"
            else:
                state = "STABLE"
                self.transition_started_s = None
        elif dwell_remaining > 0.0:
            state = "HOLDING"
        else:
            state = "STABLE"
        reason = "{};{}".format(mode_reason, overlay_reason)
        return ContextDecision(
            world_model_seq=snapshot.world_model_seq,
            mode_seq=self.mode_seq,
            geometry_mode=self.current_mode,
            dynamic_overlay=self.current_overlay,
            transition_state=state,
            confidence=mode_confidence if self.current_mode == desired_mode else 0.5,
            mode_dwell_s=dwell,
            minimum_dwell_remaining_s=dwell_remaining,
            valid=True,
            reason=reason,
            transition=committed_transition,
        )

    def _geometry(
        self, snapshot: FeatureSnapshot
    ) -> Tuple[str, float, str, Dict[str, float]]:
        thresholds = self.config["geometry"]
        dead_end_maneuver = _clamp(
            (snapshot.dead_end_score - thresholds["maneuver"]["dead_end_score_min"])
            / max(1.0e-6, 1.0 - thresholds["maneuver"]["dead_end_score_min"])
        ) if snapshot.rear_covered else 0.0
        maneuver = dead_end_maneuver
        maneuver_reason = "dead_end_and_rear_coverage"
        maneuver_config = thresholds["maneuver"]
        if "reverse_heading_error_min_rad" in maneuver_config:
            heading_min = float(maneuver_config["reverse_heading_error_min_rad"])
            heading_score = _clamp(
                (abs(snapshot.signed_heading_error_rad) - heading_min)
                / max(1.0e-6, math.pi - heading_min)
            )
            front_full = float(
                maneuver_config.get("reverse_front_clearance_full_m", 0.0)
            )
            front_max = float(maneuver_config["reverse_front_clearance_max_m"])
            front_constraint = _clamp(
                (front_max - snapshot.front_clearance_m)
                / max(1.0e-6, front_max - front_full)
            )
            rear_escape = (
                _clamp(
                    snapshot.rear_clearance_m
                    / max(
                        1.0e-6,
                        float(maneuver_config["reverse_rear_clearance_full_m"]),
                    )
                )
                if snapshot.rear_covered else 0.0
            )
            reverse_path_maneuver = min(
                heading_score, front_constraint, rear_escape
            )
            if reverse_path_maneuver > maneuver:
                maneuver = reverse_path_maneuver
                maneuver_reason = "reverse_path_with_feasible_rear_escape"
        if "pocket_front_clearance_max_m" in maneuver_config:
            front_full = float(maneuver_config["pocket_front_clearance_full_m"])
            front_max = float(maneuver_config["pocket_front_clearance_max_m"])
            front_constraint = _clamp(
                (front_max - snapshot.front_clearance_m)
                / max(1.0e-6, front_max - front_full)
            )
            side_full = float(maneuver_config["pocket_side_clearance_full_m"])
            side_max = float(maneuver_config["pocket_side_clearance_max_m"])
            left_constraint = _clamp(
                (side_max - snapshot.left_clearance_m)
                / max(1.0e-6, side_max - side_full)
            )
            right_constraint = _clamp(
                (side_max - snapshot.right_clearance_m)
                / max(1.0e-6, side_max - side_full)
            )
            rear_escape = (
                _clamp(
                    snapshot.rear_clearance_m
                    / max(1.0e-6, float(maneuver_config["pocket_rear_clearance_full_m"]))
                )
                if snapshot.rear_covered else 0.0
            )
            pocket_maneuver = min(
                front_constraint, left_constraint, right_constraint, rear_escape
            )
            if pocket_maneuver > maneuver:
                maneuver = pocket_maneuver
                maneuver_reason = "front_and_side_pocket_with_rear_escape"
        corridor_width = snapshot.corridor_width_m
        corridor = snapshot.corridor_parallel_confidence
        if not (thresholds["corridor"]["width_min_m"] <= corridor_width
                <= thresholds["corridor"]["width_max_m"]
                and snapshot.front_clearance_m >= thresholds["corridor"]["front_clearance_min_m"]):
            corridor = 0.0
        static_density_score = _clamp(
            snapshot.obstacle_density / thresholds["static_dense"]["obstacle_density_full"]
        )
        persistence_density_full = thresholds["static_dense"].get(
            "persistence_density_full"
        )
        if persistence_density_full is None:
            # Backwards-compatible V2-03/V2-04D semantics. New repair profiles
            # must explicitly require density support for persistence evidence.
            supported_persistence = snapshot.static_persistence
        else:
            density_support = _clamp(
                snapshot.obstacle_density / float(persistence_density_full)
            )
            supported_persistence = snapshot.static_persistence * density_support
        static_score = max(static_density_score, supported_persistence)
        cruise_components = (
            _clamp(1.0 - snapshot.obstacle_density
                   / thresholds["cruise"]["obstacle_density_max"]),
            _clamp(snapshot.front_clearance_m
                   / thresholds["cruise"]["forward_clearance_full_m"]),
            _clamp(1.0 - abs(snapshot.path_curvature)
                   / thresholds["cruise"]["path_curvature_abs_max"]),
            _clamp(snapshot.goal_direction_stability),
        )
        cruise = min(cruise_components)
        priorities = (
            ("MANEUVER", maneuver, maneuver_reason),
            ("CORRIDOR", corridor, "parallel_side_boundaries"),
            ("STATIC_DENSE", static_score, "density_supported_static_geometry"),
            ("CRUISE", cruise, "clear_straight_stable_path"),
        )
        scores = {mode: _clamp(score) for mode, score, _ in priorities}
        scores["BALANCED"] = _clamp(1.0 - max(scores.values()))
        for mode, score, reason in priorities:
            enter = float(thresholds[mode.lower()]["confidence_min"])
            if score >= max(enter, self.minimum_confidence):
                return mode, _clamp(score), reason, scores
        return (
            "BALANCED",
            scores["BALANCED"],
            "insufficient_mode_confidence",
            scores,
        )

    def _apply_mode_hysteresis(
        self,
        desired: str,
        confidence: float,
        reason: str,
        scores: Dict[str, float],
    ) -> Tuple[str, float, str]:
        """Hold a still-plausible active mode unless a challenger wins clearly."""

        if desired == self.current_mode or self.current_mode == "BALANCED":
            return desired, confidence, reason
        current_config = self.config["geometry"].get(self.current_mode.lower(), {})
        exit_confidence = float(current_config.get("exit_confidence", 0.0))
        current_score = float(scores.get(self.current_mode, 0.0))
        desired_score = float(scores.get(desired, 0.0))
        current_is_plausible = current_score >= exit_confidence
        challenger_is_decisive = desired_score >= current_score + self.switch_score_margin
        if current_is_plausible and not challenger_is_decisive:
            return (
                self.current_mode,
                _clamp(current_score),
                "mode_exit_hysteresis_hold_{}_over_{}".format(
                    self.current_mode.lower(), desired.lower()
                ),
            )
        return desired, confidence, reason

    def _overlay(self, tracks: Sequence[RuntimeTrack]) -> Tuple[str, str]:
        thresholds = self.config["dynamic"]
        candidates = []
        for track in tracks:
            if track.confidence < thresholds["minimum_track_confidence"]:
                continue
            if track.motion_class == "CROSSING" and abs(track.vy) > 1.0e-6:
                crossing_time = -track.y / track.vy
                crossing_x = track.x + track.vx * crossing_time
                if 0.0 <= crossing_time <= thresholds["predicted_ttc_max_s"] and crossing_x > 0.0:
                    candidates.append(("CROSSING", crossing_time, track.track_id))
                    continue
            speed_squared = track.vx * track.vx + track.vy * track.vy
            closest_time = float("inf")
            closest_distance = float("inf")
            if speed_squared > 1.0e-6:
                closest_time = max(0.0, -(track.x * track.vx + track.y * track.vy) / speed_squared)
                closest_x = track.x + track.vx * closest_time
                closest_y = track.y + track.vy * closest_time
                closest_distance = math.hypot(closest_x, closest_y) - track.radius
            within_risk = (
                closest_time <= thresholds["predicted_ttc_max_s"]
                and closest_distance <= thresholds["closest_approach_max_m"]
            )
            mapping = {
                "HEAD_ON": "HEAD_ON",
                "FOLLOWING": "FOLLOW",
            }
            if track.motion_class in mapping and within_risk:
                candidates.append((mapping[track.motion_class], closest_time, track.track_id))
            elif track.motion_class == "UNKNOWN" and within_risk:
                candidates.append(("OVERTAKE_OR_YIELD", closest_time, track.track_id))
        priority = {"HEAD_ON": 0, "CROSSING": 1, "FOLLOW": 2, "OVERTAKE_OR_YIELD": 3}
        if not candidates:
            return "NONE", "no_predicted_dynamic_conflict"
        overlay, _, track_id = min(candidates, key=lambda item: (priority[item[0]], item[1]))
        return overlay, "predicted_conflict_track_{}".format(track_id)

    def _advance_mode(
        self, desired: str, confidence: float, reason: str, stamp_s: float
    ) -> Optional[TransitionEvent]:
        if desired != self.candidate_mode:
            self.candidate_mode = desired
            self.candidate_since_s = stamp_s
        if desired == self.current_mode:
            return None
        mode_since = self.mode_since_s if self.mode_since_s is not None else stamp_s
        candidate_since = (
            self.candidate_since_s if self.candidate_since_s is not None else stamp_s
        )
        dwell = stamp_s - mode_since
        confirmation = stamp_s - candidate_since
        confirmation_required = (
            self.exit_confirmation_s
            if desired == "BALANCED" and self.current_mode != "BALANCED"
            else self.enter_confirmation_s
        )
        if dwell < self.minimum_dwell_s or confirmation < confirmation_required:
            return None
        previous = self.current_mode
        self.current_mode = desired
        self.mode_seq += 1
        self.mode_since_s = stamp_s
        self.transition_from = previous
        self.transition_started_s = stamp_s
        return TransitionEvent(
            mode_seq=self.mode_seq,
            from_mode=previous,
            to_mode=desired,
            overlay=self.current_overlay,
            state="ENTERING",
            progress=0.0,
            minimum_dwell_remaining_s=self.minimum_dwell_s,
            reason=reason,
            valid=True,
        )

    def _advance_overlay(self, desired: str, stamp_s: float) -> None:
        if desired != self.overlay_candidate:
            self.overlay_candidate = desired
            self.overlay_candidate_since_s = stamp_s
        if desired == self.current_overlay:
            return
        confirmation_required = (
            self.overlay_release_confirmation_s if desired == "NONE" else 0.0
        )
        candidate_since = (
            self.overlay_candidate_since_s
            if self.overlay_candidate_since_s is not None else stamp_s
        )
        if stamp_s - candidate_since >= confirmation_required:
            self.current_overlay = desired
