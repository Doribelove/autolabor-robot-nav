#!/usr/bin/env python3
"""V6-04 directional sensor reachable-occupancy episode."""

import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml

from v5_e2e_scope_rl.directional_safety import DirectionalSafetyAdapter


R2_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v5_e2e_scope_rl/scripts/"
    "v6_sensor_gazebo_episode_r2.py"
)
SPEC = importlib.util.spec_from_file_location("v6_03_r2_frozen_episode", str(R2_PATH))
R2 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(R2)


class V604Episode(R2.V6SensorGazeboEpisodeR2):
    def __init__(self):
        super().__init__()
        actor = self.runtime_adapter.actor
        self.runtime_adapter = DirectionalSafetyAdapter(actor=actor, advantage_gate_margin=0.10)
        self.last_runtime_scan_stamp_s = None

    def _scan(self, message):
        R2.BASE.BASE.FairEpisode._scan(self, message)
        if self.runtime_adapter is None or self.odom is None:
            return
        stamp_s = message.header.stamp.to_sec()
        if stamp_s <= 0.0:
            stamp_s = rospy.Time.now().to_sec()
        if self.last_runtime_scan_stamp_s is None or stamp_s - self.last_runtime_scan_stamp_s >= 0.20:
            scan = self.runtime_adapter.resample_scan(
                message.ranges, message.angle_min, message.angle_increment, message.range_max
            )
            self.runtime_adapter.append_scan_with_pose(scan, self.policy_pose_from_odometry(), stamp_s)
            self.last_runtime_scan_stamp_s = stamp_s

    def run(self):
        report = super().run()
        report["stage"] = "V6-04"
        report["directional_reachable_occupancy"] = {
            "tracker": self.runtime_adapter.tracker_audit(),
            "dynamic_surface_inflation_m": self.runtime_adapter.dynamic_surface_inflation_m,
            "dynamic_acceleration_bound_mps2": self.runtime_adapter.dynamic_acceleration_bound_mps2,
            "dynamic_uncertainty_rate_mps": self.runtime_adapter.dynamic_uncertainty_rate_mps,
            "policy_world_state_access": False,
        }
        temporary = self.report_path + ".v604.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        return report


def main():
    rospy.init_node("v6_04_sensor_gazebo_episode")
    try:
        V604Episode().run()
        return 0
    except Exception as error:
        rospy.logerr("V6-04 episode failed: %s: %s", type(error).__name__, error)
        return 2


if __name__ == "__main__":
    sys.exit(main())
