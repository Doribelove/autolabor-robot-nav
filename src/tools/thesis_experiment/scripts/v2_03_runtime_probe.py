#!/usr/bin/env python3
"""Evaluator-only Gazebo truth probe for V2-03 world-model outputs."""

import math
import os
import time

import rospy
import yaml
from gazebo_msgs.msg import ModelStates

from nav_world_model.msg import LocalGeometry, TrackedObstacleArray, WorldModelHealth
from teb_mode_manager.msg import ContextState


def _yaw(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


class RuntimeProbe:
    def __init__(self):
        self.report_path = rospy.get_param("~report_path")
        self.robot_name = rospy.get_param("~robot_name", "autolabor_m2")
        self.agent_name = rospy.get_param("~agent_name", "crossing-agent-1")
        self.sample_duration_s = float(rospy.get_param("~sample_duration_s", 6.0))
        self.minimum_track_samples = int(rospy.get_param("~minimum_track_samples", 12))
        self.maximum_position_rmse_m = float(rospy.get_param("~maximum_position_rmse_m", 0.8))
        self.maximum_id_switches = int(rospy.get_param("~maximum_id_switches", 1))
        self.truth = None
        self.errors = []
        self.track_ids = []
        self.health_valid_samples = 0
        self.health_fault_samples = 0
        self.geometry_valid_samples = 0
        self.context_valid_samples = 0
        self.crossing_overlay_samples = 0
        self.first_track_stamp = None
        self.last_track_stamp = None
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._truth, queue_size=2)
        rospy.Subscriber("/nav_world_model/tracks", TrackedObstacleArray, self._tracks, queue_size=5)
        rospy.Subscriber("/nav_world_model/health", WorldModelHealth, self._health, queue_size=5)
        rospy.Subscriber("/nav_world_model/local_geometry", LocalGeometry, self._geometry, queue_size=5)
        rospy.Subscriber("/teb_mode_manager/context", ContextState, self._context, queue_size=5)

    def _truth(self, message):
        if self.robot_name not in message.name or self.agent_name not in message.name:
            return
        robot_index = message.name.index(self.robot_name)
        agent_index = message.name.index(self.agent_name)
        robot, agent = message.pose[robot_index], message.pose[agent_index]
        yaw = _yaw(robot.orientation)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        dx, dy = agent.position.x - robot.position.x, agent.position.y - robot.position.y
        self.truth = (
            cosine * dx + sine * dy,
            -sine * dx + cosine * dy,
        )

    def _tracks(self, message):
        if self.truth is None or not message.obstacles:
            return
        truth_x, truth_y = self.truth
        nearest = min(
            message.obstacles,
            key=lambda item: math.hypot(
                item.pose.pose.position.x - truth_x,
                item.pose.pose.position.y - truth_y,
            ),
        )
        error = math.hypot(
            nearest.pose.pose.position.x - truth_x,
            nearest.pose.pose.position.y - truth_y,
        )
        if error <= 2.0:
            self.errors.append(error)
            self.track_ids.append(int(nearest.track_id))
            stamp = message.header.stamp.to_sec()
            self.first_track_stamp = stamp if self.first_track_stamp is None else self.first_track_stamp
            self.last_track_stamp = stamp

    def _health(self, message):
        if message.valid and not message.stale:
            self.health_valid_samples += 1
        else:
            self.health_fault_samples += 1

    def _geometry(self, message):
        if message.valid and not message.stale:
            self.geometry_valid_samples += 1

    def _context(self, message):
        if message.valid:
            self.context_valid_samples += 1
        if message.valid and message.dynamic_overlay == ContextState.DYNAMIC_CROSSING:
            self.crossing_overlay_samples += 1

    def run(self):
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.truth is not None and self.health_valid_samples > 0:
                break
            rospy.sleep(0.02)
        if self.truth is None:
            raise AssertionError("Gazebo evaluator truth unavailable")
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(20)
        while rospy.Time.now().to_sec() - start < self.sample_duration_s and not rospy.is_shutdown():
            rate.sleep()
        rmse = (
            math.sqrt(sum(error * error for error in self.errors) / len(self.errors))
            if self.errors else float("inf")
        )
        id_switches = sum(
            first != second for first, second in zip(self.track_ids, self.track_ids[1:])
        )
        checks = {
            "minimum_track_samples": len(self.errors) >= self.minimum_track_samples,
            "position_rmse": rmse <= self.maximum_position_rmse_m,
            "id_switches": id_switches <= self.maximum_id_switches,
            "valid_health": self.health_valid_samples >= self.minimum_track_samples,
            "valid_geometry": self.geometry_valid_samples >= self.minimum_track_samples,
            "valid_context": self.context_valid_samples >= 3,
            "crossing_overlay": self.crossing_overlay_samples > 0,
        }
        report = {
            "schema_version": "2.0",
            "architecture_generation": "v2",
            "suite": "v2_03_gazebo_world_model_runtime_probe",
            "simulation_only": True,
            "formal_result": False,
            "runtime_ready": False,
            "training_started": False,
            "real_vehicle_started": False,
            "truth_scope": "evaluator_only_gazebo_model_states",
            "runtime_policy_truth_access": False,
            "passed": all(checks.values()),
            "checks": checks,
            "tracking": {
                "sample_count": len(self.errors),
                "position_rmse_m": rmse,
                "id_switches": id_switches,
                "unique_track_ids": sorted(set(self.track_ids)),
                "first_track_stamp_s": self.first_track_stamp,
                "last_track_stamp_s": self.last_track_stamp,
            },
            "health": {
                "valid_samples": self.health_valid_samples,
                "fault_samples": self.health_fault_samples,
                "geometry_valid_samples": self.geometry_valid_samples,
            },
            "supervisor": {
                "valid_context_samples": self.context_valid_samples,
                "crossing_overlay_samples": self.crossing_overlay_samples,
            },
        }
        directory = os.path.dirname(self.report_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = self.report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
        os.replace(temporary, self.report_path)
        if not report["passed"]:
            raise AssertionError("V2-03 runtime checks failed: {}".format(checks))


def main():
    rospy.init_node("v2_03_runtime_probe")
    RuntimeProbe().run()


if __name__ == "__main__":
    main()
