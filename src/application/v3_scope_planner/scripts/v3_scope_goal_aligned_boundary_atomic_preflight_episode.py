#!/usr/bin/env python3
"""R19-R1 preflight with boundary-atomic terminal evaluation."""

import importlib.util
import math
import os
from pathlib import Path
import sys

import rospy
import yaml

from v3_scope_planner import execution_core
from v3_scope_planner import multi_actor_core


BASE_PATH = Path(
    "/home/robot/robot_ws_base_rl/src/application/v3_scope_planner/"
    "scripts/v3_scope_goal_aligned_recovery_preflight_episode_r1.py"
)
SPEC = importlib.util.spec_from_file_location(
    "v3_13_r19_r1_preflight_base", BASE_PATH
)
BASE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(BASE)


class BoundaryAtomicPreflight(
    BASE.RepairedGoalAlignedRecoveryPreflight
):
    def __init__(self):
        super().__init__()
        self.boundary_terminal_evaluation_count = 0
        self.boundary_terminal_trigger_count = 0

    def execute(self, candidate):
        points = candidate["points"]
        started_s = rospy.Time.now().to_sec()
        previous_pose = self.robot_world_pose()
        path_index = 0
        path_length_m = 0.0
        minimum_clearance_m = float("inf")
        tracking_errors = []
        trace = []
        self.collecting = True
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            now_s = rospy.Time.now().to_sec()
            elapsed_s = now_s - started_s
            pose = self.robot_world_pose()
            tracks = self.actor_tracks()
            step_m = math.hypot(
                pose["x_m"] - previous_pose["x_m"],
                pose["y_m"] - previous_pose["y_m"],
            )
            if step_m <= 0.25:
                path_length_m += step_m
            previous_pose = pose
            sample_clearances = [
                math.hypot(
                    pose["x_m"] - track["world_x_m"],
                    pose["y_m"] - track["world_y_m"],
                )
                - self.combined_radius_m
                for track in tracks
            ]
            minimum_clearance_m = min(
                minimum_clearance_m, min(sample_clearances)
            )
            trace.append(
                {
                    "elapsed_s": elapsed_s,
                    "ego": {
                        "x_m": pose["x_m"],
                        "y_m": pose["y_m"],
                        "yaw_rad": pose["yaw_rad"],
                        "speed_mps": float(
                            self.odom.twist.twist.linear.x
                        ),
                    },
                    "actors": tracks,
                    "actor_clearances_m": sample_clearances,
                }
            )
            command = execution_core.pure_pursuit_command(
                points,
                pose["x_m"],
                pose["y_m"],
                pose["yaw_rad"],
                minimum_index=path_index,
                lookahead_m=self.lookahead_m,
            )
            path_index = max(path_index, command["nearest_index"])
            tracking_errors.append(command["tracking_error_m"])
            if elapsed_s >= self.horizon_s:
                self.boundary_terminal_evaluation_count += 1
                if command.get("terminal_reached_fail_closed", False):
                    self.boundary_terminal_trigger_count += 1
                    self.publish_command(
                        command["linear_x_mps"],
                        command["angular_z_radps"],
                    )
                else:
                    self.publish_command(0.0, 0.0)
                break
            self.publish_command(
                command["linear_x_mps"],
                command["angular_z_radps"],
            )
            rate.sleep()
        self.collecting = False
        for _ in range(6):
            self.publish_command(0.0, 0.0)
            rospy.sleep(0.02)
        trace_duration_s = (
            trace[-1]["elapsed_s"] - trace[0]["elapsed_s"]
            if len(trace) >= 2
            else 0.0
        )
        return {
            "completed_horizon": (
                trace_duration_s >= self.horizon_s - 0.15
            ),
            "trace_duration_s": trace_duration_s,
            "trace_sample_count": len(trace),
            "trace": trace,
            "trace_digest": multi_actor_core.digest(trace),
            "path_length_m": path_length_m,
            "minimum_conservative_clearance_m": minimum_clearance_m,
            "contact_state_count": self.contact_count,
            "minimum_scan_range_m": (
                self.minimum_scan_m
                if math.isfinite(self.minimum_scan_m)
                else None
            ),
            "command_count": self.command_count,
            "maximum_command_gap_s": self.maximum_command_gap_s,
            "tracking_error_max_m": (
                max(tracking_errors) if tracking_errors else None
            ),
            "tracking_error_mean_m": (
                sum(tracking_errors) / len(tracking_errors)
                if tracking_errors
                else None
            ),
            "final_pose": self.robot_world_pose(),
        }

    def run(self):
        report = super().run()
        report.update(
            {
                "stage": "V3-13-Q1-R19-R1",
                "method": (
                    "fresh_non_evidence_goal_aligned_"
                    "boundary_atomic_terminal_preflight"
                ),
                "boundary_terminal_evaluation_count": (
                    self.boundary_terminal_evaluation_count
                ),
                "boundary_terminal_trigger_count": (
                    self.boundary_terminal_trigger_count
                ),
                "boundary_atomic_terminal_pass": (
                    self.boundary_terminal_evaluation_count == 1
                    and self.boundary_terminal_trigger_count == 1
                    and report["terminal_guard_activation_count"] >= 1
                ),
            }
        )
        report["preflight_scientific_gate_pass"] = (
            report["preflight_scientific_gate_pass"]
            and report["boundary_atomic_terminal_pass"]
        )
        temporary = self.report_path + ".r19_r1.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                report, stream, sort_keys=False, allow_unicode=True
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        return report


def main():
    rospy.init_node("v3_scope_goal_aligned_boundary_atomic_preflight")
    try:
        BoundaryAtomicPreflight().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "V3-13-Q1-R19-R1 failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
