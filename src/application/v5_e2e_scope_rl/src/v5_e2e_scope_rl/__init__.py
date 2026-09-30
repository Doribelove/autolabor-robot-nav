"""V5/V6 SCOPE-RL package with lazy simulation-backend imports.

The ROS deployment adapter needs only NumPy and PyTorch.  Keeping Gymnasium
lazy prevents a training-only dependency from becoming part of the deployed
sensor-actor process.
"""

__all__ = ["ACTION_NAMES", "SensorCandidateEnv"]


def __getattr__(name):
    if name in __all__:
        from .sensor_env import ACTION_NAMES, SensorCandidateEnv

        return {"ACTION_NAMES": ACTION_NAMES, "SensorCandidateEnv": SensorCandidateEnv}[name]
    raise AttributeError(name)
