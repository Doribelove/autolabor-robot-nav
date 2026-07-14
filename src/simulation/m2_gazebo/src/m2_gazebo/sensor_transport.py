"""Deterministic helpers for the V2 simulation transport contract."""

import hashlib
import math


class SensorTransportError(ValueError):
    """Raised when a simulation transport candidate is unsafe or ambiguous."""


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise SensorTransportError("{} must be numeric".format(name))
    result = float(value)
    if not math.isfinite(result):
        raise SensorTransportError("{} must be finite".format(name))
    return result


def deterministic_unit(seed, sequence, channel=0):
    """Return a reproducible value in the open interval (0, 1)."""

    if any(isinstance(value, bool) or not isinstance(value, int) or value < 0
           for value in (seed, sequence, channel)):
        raise SensorTransportError(
            "seed, sequence, and channel must be non-negative integers"
        )
    payload = "{}:{}:{}".format(seed, sequence, channel).encode("ascii")
    integer = int.from_bytes(hashlib.sha256(payload).digest()[:8], "big")
    return (integer + 0.5) / (2 ** 64)


def deterministic_jitter(seed, sequence, amplitude_s):
    amplitude = _finite(amplitude_s, "amplitude_s")
    if amplitude < 0.0:
        raise SensorTransportError("amplitude_s must be non-negative")
    return (2.0 * deterministic_unit(seed, sequence) - 1.0) * amplitude


def deterministic_gaussian(seed, sequence, channel, stddev):
    sigma = _finite(stddev, "stddev")
    if sigma < 0.0:
        raise SensorTransportError("stddev must be non-negative")
    if sigma == 0.0:
        return 0.0
    u1 = deterministic_unit(seed, sequence, 2 * channel + 1)
    u2 = deterministic_unit(seed, sequence, 2 * channel + 2)
    return sigma * math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)


def release_time(arrival_s, delay_s, jitter_s, seed, sequence):
    arrival = _finite(arrival_s, "arrival_s")
    delay = _finite(delay_s, "delay_s")
    jitter = _finite(jitter_s, "jitter_s")
    if arrival < 0.0 or delay < 0.0 or jitter < 0.0 or jitter > delay:
        raise SensorTransportError(
            "arrival/delay/jitter require arrival>=0 and 0<=jitter<=delay"
        )
    return arrival + delay + deterministic_jitter(seed, sequence, jitter)


def noisy_range(value, range_min, range_max, stddev, seed, sequence, ray_index):
    """Apply deterministic Gaussian noise while preserving non-finite returns."""

    measurement = float(value)
    if not math.isfinite(measurement):
        return measurement
    lower = _finite(range_min, "range_min")
    upper = _finite(range_max, "range_max")
    if lower < 0.0 or lower >= upper:
        raise SensorTransportError("range_min/range_max are invalid")
    perturbed = measurement + deterministic_gaussian(
        seed, sequence, ray_index, stddev
    )
    return max(lower, min(upper, perturbed))


def stopping_distance(speed_mps, command_latency_s, brake_deceleration_mps2):
    speed = abs(_finite(speed_mps, "speed_mps"))
    latency = _finite(command_latency_s, "command_latency_s")
    deceleration = _finite(brake_deceleration_mps2, "brake_deceleration_mps2")
    if latency < 0.0 or deceleration <= 0.0:
        raise SensorTransportError(
            "latency must be non-negative and deceleration positive"
        )
    return speed * latency + speed * speed / (2.0 * deceleration)
