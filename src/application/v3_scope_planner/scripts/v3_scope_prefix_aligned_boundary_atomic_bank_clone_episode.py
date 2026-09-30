#!/usr/bin/env python3
"""R19-R3 five-action clone with a 100 Hz common-prefix controller."""

import importlib.util
import os
from pathlib import Path
import sys

import rospy
import yaml

from v3_scope_planner import multi_actor_core


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/"
    "scripts/v3_scope_boundary_atomic_recovery_bank_clone_episode.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_r19_r3_boundary_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


class PrefixAlignedBoundaryAtomicBankClone(
    BASE.BoundaryAtomicRecoveryBankClone
):
    """Change only the prebranch warmup sampling rate from 20 to 100 Hz."""

    def warmup(self):
        rate = rospy.Rate(100)
        while (
            not rospy.is_shutdown()
            and rospy.Time.now().to_sec() < self.activation_time_s
        ):
            self.publish_command(0.0, 0.0)
            rate.sleep()
        activation_lateness_s = max(
            0.0, rospy.Time.now().to_sec() - self.activation_time_s
        )
        if activation_lateness_s > 0.075:
            raise RuntimeError("shared activation barrier missed")
        started_s = rospy.Time.now().to_sec()
        rate = rospy.Rate(100)
        trace = []
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            pose = self.robot_world_pose()
            tracks = self.actor_tracks()
            speed_mps = float(self.odom.twist.twist.linear.x)
            trace.append(
                {
                    "time_s": now_s,
                    "ego_world_x_m": pose["x_m"],
                    "ego_speed_mps": speed_mps,
                    "actors": [
                        {
                            "track_id": track["track_id"],
                            "coordinate_m": track["coordinate_m"],
                            "coordinate_speed_mps": track[
                                "coordinate_speed_mps"
                            ],
                        }
                        for track in tracks
                    ],
                }
            )
            if (
                pose["x_m"] >= self.gate_world_x - 0.018
                and speed_mps >= self.warmup_speed_mps - 0.035
            ):
                break
            if now_s - started_s > self.warmup_timeout_s:
                raise RuntimeError("shared warmup timed out")
            if pose["x_m"] > self.gate_world_x + 0.08:
                raise RuntimeError("shared warmup overshot gate")
            self.publish_command(self.warmup_speed_mps, 0.0)
            rate.sleep()
        return {
            "duration_s": rospy.Time.now().to_sec() - started_s,
            "activation_target_s": self.activation_time_s,
            "activation_lateness_s": activation_lateness_s,
            "sample_count": len(trace),
            "gate_pose": self.robot_world_pose(),
            "gate_speed_mps": float(self.odom.twist.twist.linear.x),
            "gate_actor_tracks": self.actor_tracks(),
            "trace_digest": multi_actor_core.digest(trace),
            "trace_tail": trace[-12:],
            "warmup_command_sampling_rate_hz": 100,
        }

    def run(self):
        report = super().run()
        report.update(
            {
                "stage": "V3-13-Q1-R19-R3",
                "method": (
                    "fresh_five_action_prefix_aligned_"
                    "boundary_atomic_preflight"
                ),
                "single_factor": {
                    "name": "prebranch_warmup_command_sampling_rate_hz",
                    "control": 20,
                    "treatment": 100,
                },
                "postbranch_execution_changed": False,
            }
        )
        temporary = self.report_path + ".r19_r3.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                report, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        return report


def main():
    rospy.init_node(
        "v3_scope_prefix_aligned_boundary_atomic_bank_clone"
    )
    try:
        PrefixAlignedBoundaryAtomicBankClone().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R19-R3 failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
