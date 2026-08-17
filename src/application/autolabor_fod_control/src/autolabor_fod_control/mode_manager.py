"""Pure command-selection and GPS/FOD entry helpers."""

from dataclasses import dataclass
import math


GPS_SOURCE = "GPS"
FOD_SOURCE = "FOD"
STOP_SOURCE = "STOP"

WAIT_FOR_FOD = "WAIT_FOR_FOD"
ENTER_FOD = "ENTER_FOD"
KEEP_GPS = "KEEP_GPS"


@dataclass(frozen=True)
class TimedCommand:
    linear_x: float
    angular_z: float
    receipt_monotonic: float


@dataclass(frozen=True)
class FodEntryDecision:
    action: str
    reason: str
    nearest_depth_m: float = None


class FodEntryGate:
    """Allow GPS/FOD handoff only for a recent target strictly inside range."""

    def __init__(self, entry_distance_m, no_detection_timeout_sec):
        self.entry_distance_m = float(entry_distance_m)
        self.no_detection_timeout_sec = float(no_detection_timeout_sec)
        if (
            not math.isfinite(self.entry_distance_m)
            or self.entry_distance_m <= 0.0
        ):
            raise ValueError("entry_distance_m must be finite and positive")
        if (
            not math.isfinite(self.no_detection_timeout_sec)
            or self.no_detection_timeout_sec <= 0.0
        ):
            raise ValueError(
                "no_detection_timeout_sec must be finite and positive"
            )
        self.latest_nearest_depth_m = None
        self.latest_valid_receipt_monotonic = None

    def update(self, depths_m, receipt_monotonic):
        receipt = float(receipt_monotonic)
        if not math.isfinite(receipt):
            raise ValueError("FOD detection receipt time must be finite")
        valid_depths = [
            float(depth)
            for depth in depths_m
            if math.isfinite(float(depth)) and float(depth) > 0.0
        ]
        if not valid_depths:
            return
        self.latest_nearest_depth_m = min(valid_depths)
        self.latest_valid_receipt_monotonic = receipt

    def evaluate(self, now_monotonic, request_started_monotonic):
        now = float(now_monotonic)
        started = float(request_started_monotonic)
        if not math.isfinite(now) or not math.isfinite(started) or now < started:
            raise ValueError("FOD entry evaluation time is invalid")

        if (
            self.latest_nearest_depth_m is not None
            and self.latest_valid_receipt_monotonic is not None
        ):
            age = now - self.latest_valid_receipt_monotonic
            if 0.0 <= age <= self.no_detection_timeout_sec:
                depth = self.latest_nearest_depth_m
                if depth < self.entry_distance_m:
                    return FodEntryDecision(
                        ENTER_FOD,
                        "nearest FOD %.3fm is within the %.3fm entry distance"
                        % (depth, self.entry_distance_m),
                        depth,
                    )
                return FodEntryDecision(
                    KEEP_GPS,
                    "nearest FOD %.3fm is not within %.3fm; GPS remains active"
                    % (depth, self.entry_distance_m),
                    depth,
                )

        elapsed = now - started
        if elapsed >= self.no_detection_timeout_sec:
            return FodEntryDecision(
                KEEP_GPS,
                "no valid-depth FOD was observed for %.3fs; GPS remains active"
                % self.no_detection_timeout_sec,
            )
        return FodEntryDecision(
            WAIT_FOR_FOD,
            "waiting up to %.3fs for valid-depth FOD information"
            % self.no_detection_timeout_sec,
        )


class CommandArbiter:
    """Select one fresh, finite command source or return an explicit stop."""

    def __init__(self, command_timeout_sec):
        timeout = float(command_timeout_sec)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError("command_timeout_sec must be finite and positive")
        self.command_timeout_sec = timeout
        self._commands = {}

    def update(self, source, linear_x, angular_z, receipt_monotonic):
        if source not in (GPS_SOURCE, FOD_SOURCE):
            raise ValueError("unknown command source: {}".format(source))
        self._commands[source] = TimedCommand(
            float(linear_x),
            float(angular_z),
            float(receipt_monotonic),
        )

    def clear(self, source=None):
        if source is None:
            self._commands.clear()
            return
        self._commands.pop(source, None)

    def age(self, source, now_monotonic):
        command = self._commands.get(source)
        if command is None:
            return None
        return float(now_monotonic) - command.receipt_monotonic

    def sample(self, source, now_monotonic):
        if source == STOP_SOURCE:
            return 0.0, 0.0, "mode transition/fault stop"
        if source not in (GPS_SOURCE, FOD_SOURCE):
            return 0.0, 0.0, "unknown source"
        command = self._commands.get(source)
        if command is None:
            return 0.0, 0.0, "{} command has not arrived".format(source)
        age = float(now_monotonic) - command.receipt_monotonic
        if not math.isfinite(age) or age < 0.0 or age > self.command_timeout_sec:
            return 0.0, 0.0, "{} command is stale".format(source)
        if not all(
            math.isfinite(value)
            for value in (command.linear_x, command.angular_z)
        ):
            return 0.0, 0.0, "{} command is non-finite".format(source)
        return command.linear_x, command.angular_z, "{} command selected".format(
            source
        )


def stopped_sample_is_valid(
    linear_x,
    angular_z,
    age_sec,
    odom_timeout_sec,
    max_linear_speed,
    max_angular_speed,
):
    """Return whether one odometry sample is fresh, finite, and stopped."""
    values = (
        linear_x,
        angular_z,
        age_sec,
        odom_timeout_sec,
        max_linear_speed,
        max_angular_speed,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return False
    return (
        0.0 <= age_sec <= odom_timeout_sec
        and abs(linear_x) <= max_linear_speed
        and abs(angular_z) <= max_angular_speed
    )
