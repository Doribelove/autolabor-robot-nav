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
REACQUIRE = "REACQUIRE"
EDGE_ARMED = "EDGE_ARMED"
LOSS_CONFIRM = "LOSS_CONFIRM"
STEER_SETTLE = "STEER_SETTLE"


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
    depth_valid: bool = False
    depth_m: float = float("nan")
    depth_mad_m: float = float("nan")
    depth_sample_count: int = 0
    depth_valid_fraction: float = 0.0

    @property
    def area(self) -> float:
        return self.width * self.height


@dataclass(frozen=True)
class AssociationConfig:
    min_iou: float = 0.05
    max_anchor_distance_ratio: float = 0.18
    min_area_ratio: float = 0.35
    max_area_ratio: float = 2.80
    duplicate_min_iou: float = 0.80
    duplicate_max_anchor_distance_ratio: float = 0.02
    duplicate_max_depth_delta_m: float = 0.15


@dataclass(frozen=True)
class TargetMachineConfig:
    acquire_frames: int = 6
    bottom_fraction: float = 0.88
    bottom_center_tolerance_fraction: float = 0.05
    bottom_confirm_frames: int = 6
    min_approach_distance_m: float = 0.10
    min_vertical_progress_fraction: float = 0.06
    loss_confirm_frames: int = 5
    loss_confirm_min_sec: float = 0.20
    early_loss_grace_frames: int = 20
    early_loss_max_frames: int = 60
    filter_alpha: float = 0.35


@dataclass(frozen=True)
class TargetDecision:
    state: str
    reason: str
    target: Optional[PixelDetection]
    filtered_u: Optional[float]
    filtered_v: Optional[float]
    acquired: bool = False
    enter_steer_settle: bool = False
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


def duplicate_observation_group(
    candidates: Sequence[PixelDetection],
    image_width: int,
    image_height: int,
    config: AssociationConfig,
) -> bool:
    """Return true only when every candidate is the same physical footprint.

    Class-aware detector NMS can occasionally publish two nearly identical
    boxes for one object under different class names. Treating that as two
    targets makes a locked controller abort even though its geometric target
    did not change. This gate deliberately requires pairwise high overlap,
    near-identical anchors, and motion-grade depths that agree; two merely
    nearby targets therefore remain ambiguous and fail closed.
    """

    if len(candidates) < 2 or image_width <= 1 or image_height <= 1:
        return False
    diagonal = math.hypot(float(image_width), float(image_height))
    for index, first in enumerate(candidates):
        if (
            not first.depth_valid
            or not math.isfinite(first.depth_m)
            or first.depth_m <= 0.0
        ):
            return False
        for second in candidates[index + 1 :]:
            if (
                not second.depth_valid
                or not math.isfinite(second.depth_m)
                or second.depth_m <= 0.0
            ):
                return False
            if intersection_over_union(first, second) < config.duplicate_min_iou:
                return False
            anchor_distance_ratio = math.hypot(
                first.anchor_u - second.anchor_u,
                first.anchor_v - second.anchor_v,
            ) / diagonal
            if anchor_distance_ratio > config.duplicate_max_anchor_distance_ratio:
                return False
            if (
                abs(first.depth_m - second.depth_m)
                > config.duplicate_max_depth_delta_m
            ):
                return False
    return True


def depth_rejection_reason(
    detection: PixelDetection,
    min_depth_m: float,
    max_depth_m: float,
    min_sample_count: int,
    min_valid_fraction: float,
    max_mad_m: float,
) -> str:
    """Return an empty string only for a motion-grade depth measurement."""

    if not detection.depth_valid:
        return "registered depth is unavailable"
    if not _finite(
        (detection.depth_m, detection.depth_mad_m, detection.depth_valid_fraction)
    ):
        return "depth measurement contains a non-finite value"
    if not min_depth_m <= detection.depth_m <= max_depth_m:
        return "depth %.3fm is outside %.3f..%.3fm" % (
            detection.depth_m,
            min_depth_m,
            max_depth_m,
        )
    if detection.depth_sample_count < min_sample_count:
        return "depth has %d samples, requires %d" % (
            detection.depth_sample_count,
            min_sample_count,
        )
    if detection.depth_valid_fraction < min_valid_fraction:
        return "depth valid fraction %.3f is below %.3f" % (
            detection.depth_valid_fraction,
            min_valid_fraction,
        )
    if detection.depth_mad_m > max_mad_m:
        return "depth MAD %.3fm exceeds %.3fm" % (
            detection.depth_mad_m,
            max_mad_m,
        )
    return ""


def nearest_depth_target(
    candidates: Sequence[PixelDetection],
    preferred: Optional[PixelDetection] = None,
    preferred_hysteresis_m: float = 0.0,
    image_width: int = 0,
    image_height: int = 0,
    association: Optional[AssociationConfig] = None,
) -> Optional[PixelDetection]:
    """Select the closest valid-depth target, stabilizing a pending lock.

    Hysteresis applies only while acquiring. Once the phase machine locks an
    object it continues normal spatial association and never jumps to another
    object merely because their depths cross.
    """

    ranked = sorted(
        (
            item
            for item in candidates
            if item.depth_valid and math.isfinite(item.depth_m) and item.depth_m > 0.0
        ),
        key=lambda item: item.depth_m,
    )
    if not ranked:
        return None
    closest = ranked[0]
    if (
        preferred is None
        or preferred_hysteresis_m <= 0.0
        or image_width <= 1
        or image_height <= 1
    ):
        return closest
    matches = matching_detections(
        preferred,
        ranked,
        image_width,
        image_height,
        association or AssociationConfig(),
    )
    if matches and matches[0].depth_m <= closest.depth_m + preferred_hysteresis_m:
        return matches[0]
    return closest


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
    """Fail closed when the healthy control loop stops renewing a command."""

    def __init__(self, lease_sec: float):
        if not math.isfinite(lease_sec) or lease_sec <= 0.0:
            raise ValueError("lease_sec must be finite and positive")
        self.lease_sec = float(lease_sec)
        self.linear_x = 0.0
        self.curvature = 0.0
        self.lease_deadline = None
        self.absolute_deadline = None
        self.expired_reason = ""

    def stop(self) -> None:
        self.linear_x = 0.0
        self.curvature = 0.0
        self.lease_deadline = None
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
        # The command timer normally detects lease expiry in sample().  Check
        # the old deadlines here as well so a delayed control loop cannot race
        # the timer and resurrect a command by renewing it after expiry.
        if self.linear_x > 0.0:
            if self.absolute_deadline is None or now >= self.absolute_deadline:
                self.expired_reason = "absolute motion deadline expired"
            elif self.lease_deadline is None or now >= self.lease_deadline:
                self.expired_reason = "control heartbeat lease expired"
            if self.expired_reason:
                self.stop()
                raise RuntimeError(self.expired_reason)
            # A live run may shorten, but never extend, its original absolute
            # deadline.  Starting a new run requires a new MotionLease object.
            absolute_deadline = min(absolute_deadline, self.absolute_deadline)
        self.linear_x = float(linear_x)
        self.curvature = float(curvature)
        self.absolute_deadline = float(absolute_deadline)
        self.lease_deadline = min(now + self.lease_sec, absolute_deadline)

    def sample(self, now: float) -> Tuple[float, float, str]:
        if not math.isfinite(now):
            raise ValueError("lease sample time must be finite")
        if self.linear_x > 0.0:
            if self.absolute_deadline is None or now >= self.absolute_deadline:
                self.expired_reason = "absolute motion deadline expired"
            elif self.lease_deadline is None or now >= self.lease_deadline:
                self.expired_reason = "control heartbeat lease expired"
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
    """Lock one object and make blind-zone entry a one-way guarded transition."""

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
        self.reason = "waiting for one stable depth-selected target"
        self.pending = None
        self.pending_hits = 0
        self.locked = None
        self.filtered_u = None
        self.filtered_v = None
        self.acquired_v_fraction = None
        self.maximum_v_fraction = None
        self.bottom_hits = 0
        self.missing_frames = 0
        self.loss_started_stamp = None

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
        matches = matching_detections(
            self.locked, candidates, image_width, image_height, self.association
        )
        if len(matches) == 1:
            return matches[0], ""
        if len(matches) > 1:
            if duplicate_observation_group(
                matches, image_width, image_height, self.association
            ):
                # matching_detections already ranks continuity with the
                # previous lock. All alternatives proved to be the same
                # physical footprint, so retaining its first match cannot
                # jump to a different geometric target.
                return matches[0], ""
            return None, "multiple detections match the locked target"
        return None, ""

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
            if len(candidates) != 1:
                self.pending = None
                self.pending_hits = 0
                self.reason = (
                    "no eligible target"
                    if not candidates
                    else "multiple eligible targets; motion remains inhibited"
                )
                return self._decision()
            candidate = candidates[0]
            if self.pending is None:
                self.pending = candidate
                self.pending_hits = 1
            else:
                matches = matching_detections(
                    self.pending,
                    (candidate,),
                    image_width,
                    image_height,
                    self.association,
                )
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
            self.acquired_v_fraction = self.filtered_v / float(image_height - 1)
            self.maximum_v_fraction = self.acquired_v_fraction
            self.state = APPROACH
            self.reason = "target locked; visual approach active"
            return self._decision(acquired=True)

        if self.state not in (
            APPROACH,
            REACQUIRE,
            EDGE_ARMED,
            LOSS_CONFIRM,
        ):
            return self._decision()

        matched, ambiguity = self._associate(candidates, image_width, image_height)
        if ambiguity:
            return self._decision(fault=ambiguity)

        if matched is not None:
            self._update_locked(matched)
            q = self.filtered_v / float(image_height - 1)
            raw_q = matched.anchor_v / float(image_height - 1)
            self.maximum_v_fraction = max(self.maximum_v_fraction, q)
            self.missing_frames = 0
            self.loss_started_stamp = None

            if self.state == LOSS_CONFIRM:
                center_error_px = abs(self.filtered_u - target_u)
                raw_center_error_px = abs(matched.anchor_u - target_u)
                if (
                    q < self.config.bottom_fraction
                    or raw_q < self.config.bottom_fraction
                    or center_error_px
                    > self.config.bottom_center_tolerance_fraction * image_width
                    or raw_center_error_px
                    > self.config.bottom_center_tolerance_fraction * image_width
                ):
                    return self._decision(
                        fault="target reappeared outside the armed bottom-center gate"
                    )
                self.state = EDGE_ARMED
                self.reason = "target reacquired before blind transition"
                return self._decision()

            if self.state == REACQUIRE:
                self.state = APPROACH
                self.reason = "locked target reacquired"

            center_error_px = abs(self.filtered_u - target_u)
            raw_center_error_px = abs(matched.anchor_u - target_u)
            vertical_progress = q - self.acquired_v_fraction
            ready = (
                q >= self.config.bottom_fraction
                and raw_q >= self.config.bottom_fraction
                and center_error_px
                <= self.config.bottom_center_tolerance_fraction * image_width
                and raw_center_error_px
                <= self.config.bottom_center_tolerance_fraction * image_width
                and approach_distance_m >= self.config.min_approach_distance_m
                and vertical_progress >= self.config.min_vertical_progress_fraction
            )
            if ready:
                self.bottom_hits += 1
            else:
                self.bottom_hits = 0

            if self.state == EDGE_ARMED:
                # EMA is useful for smooth steering, but the last real pixel
                # observation must independently remain in the blind-entry
                # gate.  Otherwise a single large raw jump could be hidden by
                # filtering immediately before disappearance.
                if raw_q < self.config.bottom_fraction:
                    return self._decision(
                        fault="armed target moved upward out of the bottom gate"
                    )
                if raw_center_error_px > (
                    self.config.bottom_center_tolerance_fraction * image_width
                ):
                    return self._decision(
                        fault="armed target moved laterally out of the safe gate"
                    )
                self.reason = "bottom gate armed; waiting for expected disappearance"
                return self._decision()

            if self.bottom_hits >= self.config.bottom_confirm_frames:
                self.state = EDGE_ARMED
                self.reason = "bottom gate armed; waiting for expected disappearance"
            else:
                self.reason = "visual approach active"
            return self._decision()

        # No spatial match.  A fresh detector frame reached this branch, so it
        # represents target absence rather than detector-topic timeout.
        if self.state == APPROACH:
            # A dropout may be tolerated for steering continuity, but it must
            # never count as part of the consecutive bottom-entry proof.
            self.bottom_hits = 0
            if not candidates:
                self.missing_frames += 1
                if self.missing_frames <= self.config.early_loss_grace_frames:
                    self.reason = (
                        "target briefly missing %d/%d; holding visual command"
                        % (
                            self.missing_frames,
                            self.config.early_loss_grace_frames,
                        )
                    )
                    return self._decision()
            else:
                # A different visible object is not a detector dropout and
                # must stop immediately rather than using the grace window.
                self.missing_frames = 1
            self.state = REACQUIRE
            self.reason = "target lost before bottom gate; stopped for reacquisition"
            return self._decision()

        if self.state == REACQUIRE:
            self.missing_frames += 1
            if self.missing_frames >= self.config.early_loss_max_frames:
                return self._decision(
                    fault="target was lost before reaching the bottom gate"
                )
            self.reason = "waiting to reacquire locked target %d/%d" % (
                self.missing_frames,
                self.config.early_loss_max_frames,
            )
            return self._decision()

        if self.state == EDGE_ARMED:
            if candidates:
                return self._decision(
                    fault="locked target disappeared while another target remained visible"
                )
            self.state = LOSS_CONFIRM
            self.missing_frames = 1
            self.loss_started_stamp = frame_stamp
            self.reason = "expected bottom disappearance; confirming with fresh frames"
            return self._decision()

        if self.state == LOSS_CONFIRM:
            if candidates:
                return self._decision(
                    fault="another target appeared during blind-zone confirmation"
                )
            self.missing_frames += 1
            elapsed = max(0.0, frame_stamp - self.loss_started_stamp)
            if (
                self.missing_frames >= self.config.loss_confirm_frames
                and elapsed >= self.config.loss_confirm_min_sec
            ):
                self.state = STEER_SETTLE
                self.reason = "target disappearance confirmed; settling steering"
                return self._decision(enter_steer_settle=True)
            self.reason = "confirming disappearance %d/%d" % (
                self.missing_frames,
                self.config.loss_confirm_frames,
            )
            return self._decision()

        return self._decision()
