"""ROS-free helpers for the Autolabor M2 Gazebo model."""

from .sensor_transport import (
    SensorTransportError,
    deterministic_gaussian,
    deterministic_jitter,
    deterministic_unit,
    noisy_range,
    release_time,
    stopping_distance,
)

__all__ = [
    "SensorTransportError",
    "deterministic_gaussian",
    "deterministic_jitter",
    "deterministic_unit",
    "noisy_range",
    "release_time",
    "stopping_distance",
]
