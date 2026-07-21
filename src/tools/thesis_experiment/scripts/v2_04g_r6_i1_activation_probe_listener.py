#!/usr/bin/env python3
"""R6-I1 readiness listener with direct tracker and context counts."""

import importlib.util
import os
from pathlib import Path
import sys
import tempfile

import rospy
import yaml

from nav_world_model.msg import TrackedObstacleArray


STAGE = "V2-04G-R6-I1"
SOURCE = Path(__file__).with_name(
    "v2_04g_r3_r1_activation_probe_listener.py"
)
_SPEC = importlib.util.spec_from_file_location(
    "v2_04g_r3_r1_listener_for_r6_i1", SOURCE
)
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)
_FROZEN = _BASE._FROZEN  # pylint: disable=protected-access
_BASE.STAGE = STAGE
_FROZEN.STAGE = STAGE


def _argument(name, cast=str):
    try:
        index = sys.argv.index(name)
        value = cast(sys.argv[index + 1])
    except (ValueError, IndexError) as exc:
        raise ValueError("{} is required".format(name)) from exc
    del sys.argv[index:index + 2]
    return value


def _atomic_yaml(path, value):
    target = Path(path)
    payload = yaml.safe_dump(value, sort_keys=False).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=target.name + ".tmp.", dir=str(target.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(target))
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


class R6DirectCountProbe(_FROZEN.TaxonomyProbe):
    """Add the tracker stream that the frozen activation probe omitted."""

    latest_instance = None

    def __init__(self, required_stable_count):
        super().__init__(required_stable_count)
        R6DirectCountProbe.latest_instance = self
        rospy.Subscriber(
            "/nav_world_model/tracks",
            TrackedObstacleArray,
            self._tracked_obstacles,
            queue_size=50,
        )

    def reset_measurement(self):
        super().reset_measurement()
        self.tracker_message_count = 0

    def _tracked_obstacles(self, _message):
        with self.lock:
            if self.measurement_enabled:
                self.tracker_message_count += 1


def main():
    scene_id = _argument("--scene-id")
    attempt = _argument("--attempt", int)
    output = Path(sys.argv[sys.argv.index("--output") + 1])
    minimum = int(
        sys.argv[sys.argv.index("--minimum-message-count") + 1]
    )
    _FROZEN.TaxonomyProbe = R6DirectCountProbe
    result = _BASE.main()
    report = yaml.safe_load(output.read_text(encoding="utf-8"))
    probe = R6DirectCountProbe.latest_instance
    tracker_count = (
        int(probe.tracker_message_count) if probe is not None else 0
    )
    report.update({
        "stage": STAGE,
        "profile_id": str(report["profile_id"]),
        "scene_id": scene_id,
        "seed": int(report["seed"]),
        "attempt": attempt,
        "tracker_message_count": tracker_count,
    })
    report.setdefault("hard_gates", {})
    report["hard_gates"].update({
        "direct_tracker_message_count": tracker_count >= minimum,
        "direct_context_message_count": (
            int(report.get("context_message_count", 0)) >= minimum
        ),
    })
    report["all_hard_gates_pass"] = all(
        report["hard_gates"].values()
    )
    report["status"] = (
        "pass" if report["all_hard_gates_pass"] else "fail"
    )
    _atomic_yaml(output, report)
    return 0 if result == 0 and report["all_hard_gates_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
