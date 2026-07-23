"""Pure command-selection helpers for GPS/FOD controller arbitration."""

from dataclasses import dataclass
import math


GPS_SOURCE = "GPS"
FOD_SOURCE = "FOD"
STOP_SOURCE = "STOP"


@dataclass(frozen=True)
class TimedCommand:
    linear_x: float
    angular_z: float
    receipt_monotonic: float


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
