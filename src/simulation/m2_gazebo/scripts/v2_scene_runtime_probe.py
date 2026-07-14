#!/usr/bin/env python3
"""Read-only Gazebo probe for compiled V2 scene models and trajectory actors."""

import math
import os
import sys
import time

import rospy
import yaml
from gazebo_msgs.msg import ModelStates


class SceneRuntimeProbe:
    def __init__(self):
        self.expected_models = list(rospy.get_param("~expected_models", []))
        self.moving_model = rospy.get_param("~moving_model", "")
        self.minimum_motion_m = float(rospy.get_param("~minimum_motion_m", 0.0))
        self.sample_duration_s = float(rospy.get_param("~sample_duration_s", 1.5))
        self.report_path = rospy.get_param("~report_path", "/tmp/v2_scene_runtime_probe.yaml")
        self.latest = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._models, queue_size=20)

    def _models(self, message):
        self.latest = message

    def pose(self, name):
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.latest is not None and name in self.latest.name:
                return self.latest.pose[self.latest.name.index(name)]
            rospy.sleep(0.02)
        raise AssertionError("model unavailable: {}".format(name))

    def run(self):
        observed = sorted(self.latest.name if self.latest is not None else [])
        missing = sorted(set(self.expected_models) - set(observed))
        if missing:
            # Refresh once after all spawn callbacks have had time to arrive.
            for name in missing:
                self.pose(name)
            observed = sorted(self.latest.name)
            missing = sorted(set(self.expected_models) - set(observed))
        if missing:
            raise AssertionError("compiled scene models missing: {}".format(missing))
        motion = 0.0
        if self.moving_model:
            before = self.pose(self.moving_model)
            start = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - start < self.sample_duration_s:
                rospy.sleep(0.02)
            after = self.pose(self.moving_model)
            motion = math.hypot(
                after.position.x - before.position.x,
                after.position.y - before.position.y,
            )
            if motion < self.minimum_motion_m:
                raise AssertionError(
                    "trajectory actor moved {:.3f} m, expected >= {:.3f}".format(
                        motion, self.minimum_motion_m
                    )
                )
        report = {
            "schema_version": "2.0",
            "architecture_generation": "v2",
            "suite": "v2_02_scene_runtime_probe",
            "simulation_only": True,
            "formal_result": False,
            "runtime_ready": False,
            "training_started": False,
            "passed": True,
            "expected_models": self.expected_models,
            "observed_models": observed,
            "moving_model": self.moving_model or None,
            "observed_motion_m": motion,
            "minimum_motion_m": self.minimum_motion_m,
        }
        directory = os.path.dirname(self.report_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = self.report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
        os.replace(temporary, self.report_path)


def main():
    rospy.init_node("v2_scene_runtime_probe")
    probe = SceneRuntimeProbe()
    rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=15.0)
    probe.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
