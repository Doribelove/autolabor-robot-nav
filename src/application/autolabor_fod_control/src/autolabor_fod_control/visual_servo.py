"""ROS-independent visual-servo and blind-distance safety primitives.

The production node deliberately keeps target association and the transition
into the camera blind zone deterministic and unit-testable.  Pixel ``u`` grows
to the right and pixel ``v`` grows downwards.
"""

from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


ACQUIRE = "ACQUIRE"
APPROACH = "APPROACH"
TRACK_WAIT = "TRACK_WAIT"
BLIND_ADVANCE = "BLIND_ADVANCE"


@dataclass(frozen=True)
class PixelDetection:
    class_id: int
    class_name: str
    confidence: float
    x: float
    y: float
    width: float
    height: float
    anchor_u: float
    anchor_v: float
    track_id: int = 0
    depth_valid: bool = False
    depth_m: float = 0.0
    range_m: float = 0.0

    @property
    def area(self) -> float:
        return self.width * self.height


@dataclass(frozen=True)
class AssociationConfig:
    min_iou: float = 0.05
    max_anchor_distance_ratio: float = 0.18
    min_area_ratio: float = 0.35
    max_area_ratio: float = 2.80


@dataclass(frozen=True)
class TargetMachineConfig:
    acquire_frames: int = 6
    recovery_region_top_fraction: float = 0.50
    recovery_region_width_fraction: float = 0.60
    tracking_loss_grace_sec: float = 0.30
    recovery_loss_grace_sec: float = 0.0
    filter_alpha: float = 0.35
    roi_top_fraction: float = 0.0
    roi_bottom_fraction: float = 1.0
    select_nearest_by_depth: bool = False

    def __post_init__(self):
        if not 0.0 <= self.roi_top_fraction < self.roi_bottom_fraction <= 1.0:
            raise ValueError("visual ROI must satisfy 0 <= top < bottom <= 1")
        if not 0.0 <= self.recovery_region_top_fraction < 1.0:
            raise ValueError("recovery region top must be within the ground ROI")
        if not 0.0 < self.recovery_region_width_fraction <= 1.0:
            raise ValueError("recovery region width must be within the image")
        if not 0.0 <= self.tracking_loss_grace_sec <= 1.50:
            raise ValueError("tracking loss grace must be between 0 and 1.50 seconds")
        if not 0.0 <= self.recovery_loss_grace_sec <= 1.50:
            raise ValueError("recovery loss grace must be between 0 and 1.50 seconds")

    def recovery_region(self, image_width, image_height, target_u):
        half_width = self.recovery_region_width_fraction * image_width * 0.5
        top = self.roi_top_fraction + self.recovery_region_top_fraction * (
            self.roi_bottom_fraction - self.roi_top_fraction)
        return (max(0.0, target_u - half_width), top * (image_height - 1),
                min(image_width - 1.0, target_u + half_width),
                self.roi_bottom_fraction * (image_height - 1))

    def in_recovery_region(self, detection, image_width, image_height, target_u):
        left, top, right, bottom = self.recovery_region(image_width, image_height, target_u)
        return (left <= detection.anchor_u <= right
                and top <= detection.anchor_v <= bottom + 1.0)

    def vertical_fraction(self, anchor_v, image_height):
        # Full-frame coordinates remain authoritative on ROS topics. Only
        # approach speed and acquisition use the visible ground band.
        q = anchor_v / float(image_height - 1)
        return (q - self.roi_top_fraction) / (
            self.roi_bottom_fraction - self.roi_top_fraction)


@dataclass(frozen=True)
class TargetDecision:
    state: str
    reason: str
    target: Optional[PixelDetection]
    filtered_u: Optional[float]
    filtered_v: Optional[float]
    acquired: bool = False
    enter_blind_advance: bool = False
    fault: str = ""


@dataclass(frozen=True)
class BlindProgress:
    path_m: float
    forward_m: float
    lateral_m: float
    yaw_change_rad: float


@dataclass(frozen=True)
class ConfirmationWindow:
    """State of one feedback stream's continuous-safe confirmation window."""

    last_sequence: int
    seen_unsafe_sequence: int
    start_time: Optional[float]
    new_sample: bool


@dataclass(frozen=True)
class TerminalSensorFence:
    """Sensor state that must remain unchanged while committing COMPLETE."""

    odom_sequence: int
    wheel_sequence: int
    detection_sequence: int
    invalid_camera_generation: int
    invalid_odom_generation: int
    invalid_wheel_generation: int
    chassis_fault_generation: int
    raw_can_fault_generation: int
    m2_bypass_event_generation: int
    control_timeout_seen: bool
    detection_queue_size: int
    odom_queue_size: int
    detection_queue_overflow: bool
    odom_queue_overflow: bool


def terminal_sensor_fence_unchanged(
    expected: TerminalSensorFence, current: TerminalSensorFence
) -> bool:
    """Return true only when no unprocessed feedback crossed a terminal fence."""

    return current == expected


def terminal_feedback_is_fresh(
    now_monotonic: float,
    now_source_time: float,
    receipt_limits: Sequence[Tuple[float, float]],
    source_stamps: Sequence[float],
    source_timeout: float,
    absolute_deadlines: Sequence[float],
    future_tolerance: float = 0.20,
) -> bool:
    """Validate terminal feedback ages and deadlines at the commit instant."""

    if not _finite(
        (now_monotonic, now_source_time, source_timeout, future_tolerance)
    ):
        return False
    if source_timeout <= 0.0 or future_tolerance < 0.0:
        return False
    for receipt, timeout in receipt_limits:
        if not _finite((receipt, timeout)) or timeout <= 0.0:
            return False
        age = now_monotonic - receipt
        if age < 0.0 or age > timeout:
            return False
    for stamp in source_stamps:
        if not _finite((stamp,)):
            return False
        age = now_source_time - stamp
        if age < -future_tolerance or age > source_timeout:
            return False
    for deadline in absolute_deadlines:
        if not _finite((deadline,)) or now_monotonic >= deadline:
            return False
    return True


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def find_forbidden_publishers(
    system_publishers: Sequence[Tuple[str, Sequence[str]]],
    forbidden_topics: Sequence[str],
) -> Tuple[Tuple[str, Tuple[str, ...]], ...]:
    """Return deterministic topic/node pairs for forbidden ROS publishers.

    ROS master state is represented as ``[(topic, [node, ...]), ...]``.  Keep
    this small graph-policy primitive ROS-independent so the fail-closed M2
    bypass audit can be covered without starting a ROS master.
    """

    forbidden = set(forbidden_topics)
    active = {}
    for topic, nodes in system_publishers:
        if topic not in forbidden:
            continue
        active.setdefault(topic, set()).update(str(node) for node in nodes)
    return tuple(
        (topic, tuple(sorted(nodes)))
        for topic, nodes in sorted(active.items())
        if nodes
    )


def validate_detection(
    detection: PixelDetection, image_width: int, image_height: int
) -> None:
    """Raise ``ValueError`` if a detector result cannot be trusted."""

    if image_width <= 1 or image_height <= 1:
        raise ValueError("image dimensions must be greater than one pixel")
    values = (
        detection.confidence,
        detection.x,
        detection.y,
        detection.width,
        detection.height,
        detection.anchor_u,
        detection.anchor_v,
    )
    if not _finite(values):
        raise ValueError("detection contains a non-finite value")
    if detection.depth_valid and (
        not _finite((detection.depth_m, detection.range_m))
        or min(detection.depth_m, detection.range_m) <= 0.0
    ):
        raise ValueError("detection depth is marked valid but has no positive finite distance")
    if not 0.0 <= detection.confidence <= 1.0:
        raise ValueError("detection confidence is outside [0, 1]")
    if detection.width <= 0.0 or detection.height <= 0.0:
        raise ValueError("detection bounding box has non-positive size")
    tolerance = 1.0
    if (
        detection.x < -tolerance
        or detection.y < -tolerance
        or detection.x + detection.width > image_width + tolerance
        or detection.y + detection.height > image_height + tolerance
    ):
        raise ValueError("detection bounding box lies outside the image")
    if not (
        -tolerance <= detection.anchor_u <= image_width - 1 + tolerance
        and -tolerance <= detection.anchor_v <= image_height - 1 + tolerance
    ):
        raise ValueError("detection anchor lies outside the image")


def intersection_over_union(first: PixelDetection, second: PixelDetection) -> float:
    left = max(first.x, second.x)
    top = max(first.y, second.y)
    right = min(first.x + first.width, second.x + second.width)
    bottom = min(first.y + first.height, second.y + second.height)
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    union = first.area + second.area - intersection
    if union <= 0.0:
        return 0.0
    return intersection / union


def matching_detections(
    previous: PixelDetection,
    candidates: Sequence[PixelDetection],
    image_width: int,
    image_height: int,
    config: AssociationConfig,
) -> Tuple[PixelDetection, ...]:
    """Return all spatially plausible matches, best-scoring first.

    More than one match is intentionally left ambiguous; the caller must not
    silently jump between two nearby objects.
    """

    diagonal = math.hypot(float(image_width), float(image_height))
    scored = []
    for candidate in candidates:
        ratio = candidate.area / previous.area if previous.area > 0.0 else math.inf
        if ratio < config.min_area_ratio or ratio > config.max_area_ratio:
            continue
        distance_ratio = math.hypot(
            candidate.anchor_u - previous.anchor_u,
            candidate.anchor_v - previous.anchor_v,
        ) / diagonal
        overlap = intersection_over_union(previous, candidate)
        if overlap < config.min_iou and distance_ratio > config.max_anchor_distance_ratio:
            continue
        class_bonus = 0.05 if (
            candidate.class_id == previous.class_id
            or candidate.class_name == previous.class_name
        ) else 0.0
        score = 2.0 * overlap - distance_ratio + class_bonus
        scored.append((score, candidate))
    scored.sort(key=lambda item: item[0], reverse=True)
    return tuple(item[1] for item in scored)


def same_target_observations(previous, candidates, other_ids=()):
    """Reconnect one real observation after an ID/class change, not a bystander.

    Known neighbouring identities are excluded. A new identity must be the
    only plausible box near the last observation; no held box is generated.
    """
    exact = tuple(d for d in candidates if d.track_id == previous.track_id)
    if exact:
        return exact
    matches = []
    for d in candidates:
        if d.track_id <= 0 or d.track_id in other_ids:
            continue
        ratio = d.area / max(previous.area, 1.0)
        limit = max(48.0, min(100.0, 1.5 * max(
            math.hypot(previous.width, previous.height), math.hypot(d.width, d.height))))
        if not .2 <= ratio <= 5.0 or math.hypot(
                d.anchor_u-previous.anchor_u, d.anchor_v-previous.anchor_v) > limit:
            continue
        if (d.depth_valid and previous.depth_valid
                and abs(d.range_m-previous.range_m) > .75):
            continue
        matches.append(d)
    return tuple(matches) if len(matches) == 1 else ()


def horizontal_error(anchor_u: float, target_u: float, image_width: int) -> float:
    if image_width <= 1 or not _finite((anchor_u, target_u)):
        raise ValueError("horizontal error inputs are invalid")
    return (anchor_u - target_u) / (0.5 * float(image_width))


def curvature_from_pixel_error(
    error: float,
    gain: float,
    steering_sign: float,
    deadband: float,
    max_curvature: float,
) -> float:
    values = (error, gain, steering_sign, deadband, max_curvature)
    if not _finite(values) or gain < 0.0 or deadband < 0.0 or max_curvature <= 0.0:
        raise ValueError("curvature controller parameters are invalid")
    if steering_sign not in (-1.0, 1.0):
        raise ValueError("steering_sign must be either -1 or +1")
    effective_error = 0.0 if abs(error) <= deadband else error
    return max(
        -max_curvature,
        min(max_curvature, steering_sign * gain * effective_error),
    )


def approach_speed(
    vertical_fraction: float,
    horizontal_error_abs: float,
    far_speed: float,
    near_speed: float,
    slow_start_fraction: float,
    near_start_fraction: float,
    lateral_slowdown_error: float,
    minimum_lateral_scale: float,
) -> float:
    values = (
        vertical_fraction,
        horizontal_error_abs,
        far_speed,
        near_speed,
        slow_start_fraction,
        near_start_fraction,
        lateral_slowdown_error,
        minimum_lateral_scale,
    )
    if not _finite(values):
        raise ValueError("approach speed inputs are non-finite")
    if not 0.0 < near_speed <= far_speed:
        raise ValueError("approach speeds are invalid")
    if not 0.0 <= slow_start_fraction < near_start_fraction < 1.0:
        raise ValueError("vertical speed thresholds are invalid")
    if lateral_slowdown_error <= 0.0 or not 0.0 < minimum_lateral_scale <= 1.0:
        raise ValueError("lateral speed scaling parameters are invalid")

    if vertical_fraction <= slow_start_fraction:
        speed = far_speed
    elif vertical_fraction >= near_start_fraction:
        speed = near_speed
    else:
        ratio = (vertical_fraction - slow_start_fraction) / (
            near_start_fraction - slow_start_fraction
        )
        speed = far_speed + ratio * (near_speed - far_speed)

    lateral_scale = max(
        minimum_lateral_scale,
        1.0 - abs(horizontal_error_abs) / lateral_slowdown_error,
    )
    return speed * lateral_scale


def normalize_angle(angle: float) -> float:
    if not math.isfinite(angle):
        raise ValueError("angle must be finite")
    return math.atan2(math.sin(angle), math.cos(angle))


def local_displacement(
    start_x: float, start_y: float, start_yaw: float, end_x: float, end_y: float
) -> Tuple[float, float]:
    if not _finite((start_x, start_y, start_yaw, end_x, end_y)):
        raise ValueError("local displacement inputs must be finite")
    dx = end_x - start_x
    dy = end_y - start_y
    forward = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
    lateral = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
    return forward, lateral


def interpolate_planar_pose(
    before_stamp: float,
    before_x: float,
    before_y: float,
    before_yaw: float,
    after_stamp: float,
    after_x: float,
    after_y: float,
    after_yaw: float,
    target_stamp: float,
) -> Tuple[float, float, float, float]:
    """Interpolate x/y and shortest-path yaw at a bracketed source stamp.

    Returns ``(ratio, x, y, yaw)`` so callers can use the same ratio for other
    synchronized scalar feedback such as measured velocity.
    """

    values = (
        before_stamp,
        before_x,
        before_y,
        before_yaw,
        after_stamp,
        after_x,
        after_y,
        after_yaw,
        target_stamp,
    )
    if not _finite(values):
        raise ValueError("pose interpolation inputs must be finite")
    if after_stamp < before_stamp:
        raise ValueError("pose interpolation stamps move backwards")
    if target_stamp < before_stamp or target_stamp > after_stamp:
        raise ValueError("pose interpolation target is not bracketed")
    if abs(after_stamp - before_stamp) <= 1e-9:
        ratio = 0.0
    else:
        ratio = (target_stamp - before_stamp) / (after_stamp - before_stamp)
    yaw_delta = normalize_angle(after_yaw - before_yaw)
    return (
        ratio,
        before_x + ratio * (after_x - before_x),
        before_y + ratio * (after_y - before_y),
        normalize_angle(before_yaw + ratio * yaw_delta),
    )


class BlindDistanceTracker:
    """Accumulate real odometry motion after the target enters the blind zone."""

    def __init__(self, x: float, y: float, yaw: float):
        if not _finite((x, y, yaw)):
            raise ValueError("blind tracker start pose is invalid")
        self.start_x = float(x)
        self.start_y = float(y)
        self.start_yaw = float(yaw)
        self.last_x = float(x)
        self.last_y = float(y)
        self.path_m = 0.0

    def update(
        self, x: float, y: float, yaw: float, max_pose_step_m: float
    ) -> BlindProgress:
        if not _finite((x, y, yaw, max_pose_step_m)) or max_pose_step_m <= 0.0:
            raise ValueError("blind odometry update is invalid")
        step = math.hypot(x - self.last_x, y - self.last_y)
        if step > max_pose_step_m:
            raise ValueError(
                "odometry pose jump %.3f m exceeds %.3f m"
                % (step, max_pose_step_m)
            )
        self.path_m += step
        self.last_x = float(x)
        self.last_y = float(y)
        forward, lateral = local_displacement(
            self.start_x, self.start_y, self.start_yaw, x, y
        )
        return BlindProgress(
            path_m=self.path_m,
            forward_m=forward,
            lateral_m=lateral,
            yaw_change_rad=normalize_angle(yaw - self.start_yaw),
        )


def blind_goal_reached(progress: BlindProgress, target_forward_m: float) -> bool:
    """Return true only for net forward travel, never accumulated wandering."""

    values = (
        progress.path_m,
        progress.forward_m,
        progress.lateral_m,
        progress.yaw_change_rad,
        target_forward_m,
    )
    if not _finite(values) or target_forward_m <= 0.0:
        raise ValueError("blind completion inputs are invalid")
    return progress.forward_m >= target_forward_m


def advance_confirmation_window(
    last_sequence: int,
    seen_unsafe_sequence: int,
    start_time: Optional[float],
    sample_sequence: int,
    sample_time: float,
    sample_is_safe: bool,
    latest_unsafe_sequence: int,
) -> ConfirmationWindow:
    """Consume the newest sample without hiding unsafe samples in its batch.

    ``latest_unsafe_sequence`` is latched by the sensor callback.  It therefore
    advances even when an unsafe sample and a later safe sample both arrive
    between control ticks.  In that case the window restarts at the later safe
    sample instead of incorrectly retaining its earlier start time.
    """

    sequences = (
        last_sequence,
        seen_unsafe_sequence,
        sample_sequence,
        latest_unsafe_sequence,
    )
    if any(type(value) is not int or value < 0 for value in sequences):
        raise ValueError("confirmation sequences must be non-negative integers")
    if not math.isfinite(sample_time):
        raise ValueError("confirmation sample time must be finite")
    if start_time is not None and not math.isfinite(start_time):
        raise ValueError("confirmation start time must be finite")
    if latest_unsafe_sequence > seen_unsafe_sequence:
        seen_unsafe_sequence = latest_unsafe_sequence
        start_time = None
    new_sample = sample_sequence > last_sequence
    if new_sample:
        last_sequence = sample_sequence
        if sample_is_safe and sample_sequence > seen_unsafe_sequence:
            if start_time is None:
                start_time = sample_time
        else:
            start_time = None
    return ConfirmationWindow(
        last_sequence=last_sequence,
        seen_unsafe_sequence=seen_unsafe_sequence,
        start_time=start_time,
        new_sample=new_sample,
    )


class MotionLease:
    """Retain a command until an explicit stop or the task's absolute deadline.

    There is no control-update/heartbeat expiry. A delayed control tick does
    not erase the last command or prevent its next update.
    """

    def __init__(self):
        self.linear_x = 0.0
        self.curvature = 0.0
        self.absolute_deadline = None
        self.expired_reason = ""

    def stop(self) -> None:
        self.linear_x = 0.0
        self.curvature = 0.0
        self.absolute_deadline = None

    def set(
        self,
        linear_x: float,
        curvature: float,
        now: float,
        absolute_deadline: float,
    ) -> None:
        if not _finite((linear_x, curvature, now, absolute_deadline)):
            raise ValueError("motion lease command is non-finite")
        if linear_x <= 0.0:
            self.stop()
            return
        if absolute_deadline <= now:
            raise ValueError("motion command has no live absolute deadline")
        if self.expired_reason:
            raise RuntimeError(self.expired_reason)
        # The task deadline remains fixed even after a delayed update.
        if self.linear_x > 0.0:
            if self.absolute_deadline is None or now >= self.absolute_deadline:
                self.expired_reason = "absolute motion deadline expired"
            if self.expired_reason:
                self.stop()
                raise RuntimeError(self.expired_reason)
            # A live run may shorten, but never extend, its original absolute
            # deadline.  Starting a new run requires a new MotionLease object.
            absolute_deadline = min(absolute_deadline, self.absolute_deadline)
        self.linear_x = float(linear_x)
        self.curvature = float(curvature)
        self.absolute_deadline = float(absolute_deadline)

    def sample(self, now: float) -> Tuple[float, float, str]:
        if not math.isfinite(now):
            raise ValueError("lease sample time must be finite")
        if self.linear_x > 0.0:
            if self.absolute_deadline is None or now >= self.absolute_deadline:
                self.expired_reason = "absolute motion deadline expired"
            if self.expired_reason:
                self.stop()
        return self.linear_x, self.curvature, self.expired_reason


def renew_motion_lease_now(
    lease: MotionLease,
    linear_x: float,
    curvature: float,
    absolute_deadline: float,
    clock,
) -> float:
    """Renew a lease using a clock sampled at the renewal point.

    Callers should invoke this while holding the same lock used by the command
    publisher.  Sampling inside this helper prevents a timestamp captured
    before waiting for that lock from reviving an already-expired command.
    """

    now = float(clock())
    lease.set(linear_x, curvature, now, absolute_deadline)
    return now


class TargetPhaseMachine:
    """Lock one object, then finish one measured advance after its loss."""

    def __init__(
        self,
        config: TargetMachineConfig,
        association: Optional[AssociationConfig] = None,
    ):
        self.config = config
        self.association = association or AssociationConfig()
        self.reset()

    def reset(self) -> None:
        self.state = ACQUIRE
        self.reason = "waiting for one stable target"
        self.pending = None
        self.pending_hits = 0
        self.locked = None
        self.filtered_u = None
        self.filtered_v = None
        self.missing_frames = 0
        self.loss_started_stamp = None
        self.recovery_ready = False
        self.other_track_ids = set()

    def _decision(self, **kwargs) -> TargetDecision:
        return TargetDecision(
            state=self.state,
            reason=self.reason,
            target=self.locked,
            filtered_u=self.filtered_u,
            filtered_v=self.filtered_v,
            **kwargs,
        )

    def _set_filtered(self, target: PixelDetection, reset: bool = False) -> None:
        alpha = self.config.filter_alpha
        if reset or self.filtered_u is None or self.filtered_v is None:
            self.filtered_u = target.anchor_u
            self.filtered_v = target.anchor_v
        else:
            self.filtered_u += alpha * (target.anchor_u - self.filtered_u)
            self.filtered_v += alpha * (target.anchor_v - self.filtered_v)

    def _associate(
        self,
        candidates: Sequence[PixelDetection],
        image_width: int,
        image_height: int,
    ) -> Tuple[Optional[PixelDetection], str]:
        if self.config.select_nearest_by_depth:
            matches = self.target_candidates(candidates)
            if len(matches) > 1:
                return None, "duplicate observations for the locked target ID"
            return (matches[0] if matches else None), ""
        matches = matching_detections(
            self.locked, candidates, image_width, image_height, self.association
        )
        if len(matches) == 1:
            return matches[0], ""
        if len(matches) > 1:
            return None, "multiple detections match the locked target"
        return None, ""

    def target_candidates(self, candidates):
        """Keep the selected identity through approach, loss and blind travel."""
        if not self.config.select_nearest_by_depth or self.locked is None:
            return tuple(candidates)
        if any(d.track_id == self.locked.track_id for d in candidates):
            self.other_track_ids.update(d.track_id for d in candidates
                                        if d.track_id != self.locked.track_id)
        return same_target_observations(self.locked, candidates, self.other_track_ids)

    def _update_locked(self, target: PixelDetection) -> None:
        self.locked = target
        self._set_filtered(target)

    def process_frame(
        self,
        candidates: Sequence[PixelDetection],
        image_width: int,
        image_height: int,
        target_u: float,
        approach_distance_m: float,
        frame_stamp: float,
    ) -> TargetDecision:
        if not _finite((target_u, approach_distance_m, frame_stamp)):
            return self._decision(fault="target state-machine input is non-finite")
        if image_width <= 1 or image_height <= 1 or approach_distance_m < 0.0:
            return self._decision(fault="target state-machine input is invalid")

        if self.state == ACQUIRE:
            if self.config.select_nearest_by_depth:
                eligible = [d for d in candidates if d.track_id > 0 and d.depth_valid
                            and math.isfinite(d.range_m) and d.range_m > 0]
                # Choose nearest once, then confirm that same object across
                # frames. Range/confidence jitter must not alternate targets.
                matches = (same_target_observations(self.pending, eligible, self.other_track_ids)
                           if self.pending is not None else ())
                pending = matches[0] if len(matches) == 1 else None
                chosen = pending or min(eligible, key=lambda d: (d.range_m, d.track_id), default=None)
                if chosen is not None:
                    self.other_track_ids.update(d.track_id for d in eligible
                                                if d.track_id != chosen.track_id)
                candidates = (chosen,) if chosen is not None else ()
            if len(candidates) != 1:
                self.pending = None
                self.pending_hits = 0
                self.reason = (
                    ("waiting for a target with valid registered depth"
                     if self.config.select_nearest_by_depth else "no eligible target")
                    if not candidates
                    else "multiple eligible targets; motion remains inhibited"
                )
                return self._decision()
            candidate = candidates[0]
            if self.pending is None:
                self.pending = candidate
                self.pending_hits = 1
            else:
                if self.config.select_nearest_by_depth:
                    matches = same_target_observations(self.pending, (candidate,), self.other_track_ids)
                else:
                    matches = matching_detections(
                        self.pending, (candidate,), image_width, image_height, self.association)
                if len(matches) == 1:
                    self.pending = candidate
                    self.pending_hits += 1
                else:
                    self.pending = candidate
                    self.pending_hits = 1
            self.reason = "confirming target %d/%d" % (
                self.pending_hits,
                self.config.acquire_frames,
            )
            if self.pending_hits < self.config.acquire_frames:
                return self._decision()
            self.locked = self.pending
            self._set_filtered(self.locked, reset=True)
            self.recovery_ready = self.config.in_recovery_region(
                self.locked, image_width, image_height, target_u)
            self.state = APPROACH
            self.reason = "target locked; visual approach active"
            return self._decision(acquired=True)

        if self.state not in (APPROACH, TRACK_WAIT):
            return self._decision()

        candidates = self.target_candidates(candidates)
        matched, ambiguity = self._associate(candidates, image_width, image_height)
        if ambiguity:
            return self._decision(fault=ambiguity)

        if matched is not None:
            self._update_locked(matched)
            self.recovery_ready = self.recovery_ready or self.config.in_recovery_region(
                matched, image_width, image_height, target_u)
            self.missing_frames = 0
            self.loss_started_stamp = None
            if self.state == TRACK_WAIT:
                self.state = APPROACH
                self.reason = "same locked target reacquired; resuming visual approach"
            else:
                self.reason = ("target entered recovery region; disappearance will start recovery"
                               if self.recovery_ready else "approaching recovery region")
            return self._decision()

        # The region is latched for this identity, so edge jitter never disarms
        # it. Its first absence commits one odometry-only crossing; subsequent
        # boxes cannot restart distance, select another object or veto finish.
        self.missing_frames += 1
        if self.loss_started_stamp is None:
            self.loss_started_stamp = frame_stamp
        if (self.recovery_ready and frame_stamp-self.loss_started_stamp
                >= self.config.recovery_loss_grace_sec):
            self.state = BLIND_ADVANCE
            self.reason = "recovery-region target disappeared; completing one measured crossing"
            return self._decision(enter_blind_advance=True)
        if self.recovery_ready or frame_stamp - self.loss_started_stamp < self.config.tracking_loss_grace_sec:
            self.reason = "brief target dropout before recovery region; retaining approach"
        else:
            self.state = TRACK_WAIT
            self.reason = "target lost outside recovery region; waiting for the same target"
        return self._decision()
