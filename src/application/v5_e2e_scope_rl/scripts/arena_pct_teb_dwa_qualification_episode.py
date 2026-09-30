#!/usr/bin/env python3
"""One independent Arena PCT/analytic/Actual-TEB/DWA qualification episode."""

from __future__ import annotations

import importlib.util
import math
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rosgraph
import rospy
import yaml

from v3_scope_planner import execution_core
from v5_e2e_scope_rl.arena_benchmark_runtime import (
    ArenaMapDirectionalBenchmarkRuntimeAdapter,
)


def load_r2_source():
    workspace = Path(os.environ["THESIS_WS"]).resolve()
    source = workspace / (
        "src/application/v5_e2e_scope_rl/scripts/"
        "pct_teb_dwa_benchmark_r2_episode.py"
    )
    specification = importlib.util.spec_from_file_location(
        "arena_pct_teb_dwa_r2_source", str(source)
    )
    if specification is None or specification.loader is None:
        raise RuntimeError("R2_EPISODE_SOURCE_IMPORT_SPEC_INVALID")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module


r2 = load_r2_source()


class ArenaQualificationEpisode(r2.R2BenchmarkEpisode):
    def __init__(self):
        self._contact_lock = threading.Lock()
        self.contact_readiness_message_count = 0
        self.contact_monitor_active = False
        self.contact_monitor_start_s = None
        self.contact_monitor_stamps = []
        self.contact_monitor_frame_ids = []
        self.contact_monitor_state_counts = []
        self.contact_monitor_empty_message_count = 0
        self.contact_monitor_nonempty_message_count = 0
        self.contact_monitor_state_sample_count = 0
        self.contact_monitor_invalid_stamp_count = 0
        super().__init__()
        self.map_yaml_path = str(Path(rospy.get_param("~map_yaml")).resolve())
        self.route_origin_world_x_m = float(
            rospy.get_param("~route_origin_world_x_m")
        )
        self.route_origin_world_y_m = float(
            rospy.get_param("~route_origin_world_y_m")
        )
        self.route_yaw_world_rad = float(
            rospy.get_param("~route_yaw_world_rad")
        )
        self.map_static_surface_tolerance_m = float(
            rospy.get_param("~map_static_surface_tolerance_m", 0.35)
        )
        self.goal_yaw_tolerance_rad = 0.20
        self.release_sha256 = str(rospy.get_param("~release_sha256", "")) or None
        self.release_review_sha256 = (
            str(rospy.get_param("~release_review_sha256", "")) or None
        )

        actor = self.runtime.actor
        self.runtime = ArenaMapDirectionalBenchmarkRuntimeAdapter(
            actor=actor,
            start_x_m=self.policy_start_world_x,
            goal_x_m=self.goal_world_x,
            goal_tolerance_m=self.goal_tolerance_m,
            # Preserve the checkpoint's training-time feature semantics.  The
            # episode may run for 30 s, while the policy time feature saturates
            # at the historical 22 s horizon.
            maximum_episode_duration_s=22.0,
            shield_inflation_m=0.33,
            inference_deadline_ms=20.0,
            maximum_input_age_s=0.20,
            goal_speed_envelope=True,
            safety_response_delay_s=0.35,
            certified_stop_dwell_s=1.50,
            map_yaml_path=self.map_yaml_path,
            route_origin_world_x_m=self.route_origin_world_x_m,
            route_origin_world_y_m=self.route_origin_world_y_m,
            route_yaw_world_rad=self.route_yaw_world_rad,
            map_static_surface_tolerance_m=self.map_static_surface_tolerance_m,
        )
        self.candidate_target_speeds_mps = r2.apply_candidate_variant(
            self.runtime, self.candidate_variant
        )

    def _contacts(self, message):
        stamp = float(message.header.stamp.to_sec())
        if stamp <= 0.0:
            stamp = float(rospy.Time.now().to_sec())
        state_count = len(message.states)
        with self._contact_lock:
            self.contact_readiness_message_count += 1
            if self.contact_monitor_active:
                if not math.isfinite(stamp):
                    self.contact_monitor_invalid_stamp_count += 1
                    self.contact_monitor_stamps.append(None)
                else:
                    self.contact_monitor_stamps.append(stamp)
                self.contact_monitor_frame_ids.append(
                    str(message.header.frame_id).lstrip("/")
                )
                self.contact_monitor_state_counts.append(int(state_count))
                if state_count:
                    self.contact_monitor_nonempty_message_count += 1
                else:
                    self.contact_monitor_empty_message_count += 1
                self.contact_monitor_state_sample_count += state_count
        if self.collecting:
            self.contact_state_count += state_count

    @staticmethod
    def _contact_publisher_nodes():
        publishers, _subscribers, _services = rosgraph.Master(
            rospy.get_name()
        ).getSystemState()
        for topic, nodes in publishers:
            if topic == "/m2_gazebo/contacts":
                return sorted(str(node) for node in nodes)
        return []

    @staticmethod
    def _contact_topic_type():
        for topic, message_type in rospy.get_published_topics():
            if topic == "/m2_gazebo/contacts":
                return str(message_type)
        return None

    def wait_ready(self):
        super().wait_ready()
        deadline = time.monotonic() + 10.0
        readiness = None
        while time.monotonic() < deadline and not rospy.is_shutdown():
            with self._contact_lock:
                message_count = self.contact_readiness_message_count
            readiness = {
                "topic_type": self._contact_topic_type(),
                "publisher_nodes": self._contact_publisher_nodes(),
                "subscriber_connections": int(
                    self.contact_subscriber.get_num_connections()
                ),
                "message_count": int(message_count),
            }
            if (
                readiness["topic_type"] == "gazebo_msgs/ContactsState"
                and readiness["publisher_nodes"] == ["/gazebo"]
                and readiness["subscriber_connections"] >= 1
                and readiness["message_count"] >= 5
            ):
                break
            rospy.rostime.wallsleep(0.02)
        else:
            raise RuntimeError("contact sensor publisher/heartbeat did not become ready")
        with self._contact_lock:
            self.contact_monitor_start_s = float(rospy.Time.now().to_sec())
            self.contact_monitor_stamps = []
            self.contact_monitor_frame_ids = []
            self.contact_monitor_state_counts = []
            self.contact_monitor_empty_message_count = 0
            self.contact_monitor_nonempty_message_count = 0
            self.contact_monitor_state_sample_count = 0
            self.contact_monitor_invalid_stamp_count = 0
            self.contact_monitor_active = True
        self.contact_readiness = readiness

    def _finalize_contact_monitor(self):
        rospy.sleep(0.06)
        with self._contact_lock:
            self.contact_monitor_active = False
            monitor_end_s = float(rospy.Time.now().to_sec())
            raw_stamps = list(self.contact_monitor_stamps)
            stamps = [float(value) for value in raw_stamps if value is not None]
            frame_ids = list(self.contact_monitor_frame_ids)
            state_counts = list(self.contact_monitor_state_counts)
            empty_count = int(self.contact_monitor_empty_message_count)
            nonempty_count = int(self.contact_monitor_nonempty_message_count)
            state_count = int(self.contact_monitor_state_sample_count)
            invalid_stamp_count = int(self.contact_monitor_invalid_stamp_count)
            monitor_start_s = float(self.contact_monitor_start_s)
        if stamps:
            monitor_end_s = max(monitor_end_s, stamps[-1])
        intervals = [right - left for left, right in zip(stamps[:-1], stamps[1:])]
        regression_count = sum(value < -1.0e-9 for value in intervals)
        positive_intervals = [value for value in intervals if value >= 0.0]
        first_gap_s = stamps[0] - monitor_start_s if stamps else None
        last_gap_s = monitor_end_s - stamps[-1] if stamps else None
        gap_candidates = list(positive_intervals)
        if first_gap_s is not None:
            gap_candidates.append(max(0.0, first_gap_s))
        if last_gap_s is not None:
            gap_candidates.append(max(0.0, last_gap_s))
        maximum_gap_s = max(gap_candidates) if gap_candidates else None
        observed_rate_hz = (
            (len(stamps) - 1) / (stamps[-1] - stamps[0])
            if len(stamps) >= 2 and stamps[-1] > stamps[0]
            else None
        )
        end_publishers = self._contact_publisher_nodes()
        end_connections = int(self.contact_subscriber.get_num_connections())
        end_type = self._contact_topic_type()
        frame_mismatch_count = sum(frame != "base_link" for frame in frame_ids)
        publisher_presence_pass = bool(
            self.contact_readiness["topic_type"] == "gazebo_msgs/ContactsState"
            and self.contact_readiness["publisher_nodes"] == ["/gazebo"]
            and self.contact_readiness["subscriber_connections"] >= 1
            and end_type == "gazebo_msgs/ContactsState"
            and end_publishers == ["/gazebo"]
            and end_connections >= 1
        )
        heartbeat_pass = bool(
            publisher_presence_pass
            and len(stamps) >= 2
            and observed_rate_hz is not None
            and observed_rate_hz >= 80.0
            and maximum_gap_s is not None
            and maximum_gap_s <= 0.05
            and invalid_stamp_count == 0
            and regression_count == 0
            and frame_mismatch_count == 0
        )
        return {
            "topic": "/m2_gazebo/contacts",
            "expected_type": "gazebo_msgs/ContactsState",
            "expected_publisher_nodes": ["/gazebo"],
            "expected_frame_id": "base_link",
            "expected_update_rate_hz": 100.0,
            "minimum_observed_rate_hz": 80.0,
            "maximum_allowed_coverage_gap_s": 0.05,
            "readiness": self.contact_readiness,
            "end_topic_type": end_type,
            "end_publisher_nodes": end_publishers,
            "end_subscriber_connections": end_connections,
            "monitor_start_s": monitor_start_s,
            "monitor_end_s": monitor_end_s,
            "message_count": len(raw_stamps),
            "empty_message_count": empty_count,
            "nonempty_message_count": nonempty_count,
            "contact_state_sample_count": state_count,
            "first_stamp_s": stamps[0] if stamps else None,
            "last_stamp_s": stamps[-1] if stamps else None,
            "message_stamps_s": raw_stamps,
            "message_state_counts": state_counts,
            "frame_id_counts": {
                frame: frame_ids.count(frame) for frame in sorted(set(frame_ids))
            },
            "observed_rate_hz": observed_rate_hz,
            "first_coverage_gap_s": first_gap_s,
            "last_coverage_gap_s": last_gap_s,
            "maximum_coverage_gap_s": maximum_gap_s,
            "invalid_stamp_count": invalid_stamp_count,
            "timestamp_regression_count": regression_count,
            "frame_mismatch_count": frame_mismatch_count,
            "publisher_presence_pass": publisher_presence_pass,
            "heartbeat_pass": heartbeat_pass,
            "no_contact_state_samples_observed": state_count == 0,
            "collision_sensitivity_positive_control_qualified": False,
        }

    def _world_to_route(self, x_m, y_m, yaw_rad):
        point = self.runtime.arena_map.world_to_route(
            np.asarray([float(x_m), float(y_m)], dtype=np.float64)
        )
        route_yaw = execution_core.angle_delta(
            float(yaw_rad), self.route_yaw_world_rad
        )
        return float(point[0]), float(point[1]), float(route_yaw)

    def _scan(self, message):
        """Retain exact-stamp transport while allowing Arena mesh-load jitter.

        Hospital produces occasional 4--23 ms scan-before-TF ordering gaps.
        The historical 30 ms wait was marginal once the rich mesh world and
        move_base shared the process budget.  Q1 waits up to 120 ms for the
        *same requested timestamp*; it never falls back to latest-TF data.
        """
        self.scan = message
        stamp = message.header.stamp.to_sec() or rospy.Time.now().to_sec()
        self.scan_received_s = stamp
        self.scan_stamps.append(stamp)
        values = [
            float(value)
            for value in message.ranges
            if math.isfinite(float(value))
        ]
        if self.collecting and values:
            self.minimum_scan_m = min(self.minimum_scan_m, min(values))
        if (
            self.last_runtime_scan_stamp_s is None
            or stamp - self.last_runtime_scan_stamp_s >= 0.20
        ):
            try:
                scan = self.runtime.resample_scan(
                    message.ranges,
                    message.angle_min,
                    message.angle_increment,
                    message.range_max,
                )
                transform = self.tf_buffer.lookup_transform(
                    "odom",
                    str(message.header.frame_id).lstrip("/"),
                    message.header.stamp,
                    rospy.Duration(0.12),
                )
                scan_pose = (
                    float(transform.transform.translation.x) + self.spawn_world_x,
                    float(transform.transform.translation.y),
                    execution_core.quaternion_yaw(transform.transform.rotation),
                )
                self.runtime.append_scan_with_pose(scan, scan_pose, stamp)
                self.scan_transform_success_count += 1
                self.last_runtime_scan_stamp_s = stamp
            except Exception as error:
                self.scan_transform_failure_count += 1
                rospy.logwarn_throttle(
                    1.0, "Arena benchmark rejected exact-stamp scan: %s", error
                )

    def robot_world_pose(self):
        if self.model_states is None or self.ego_model not in self.model_index:
            raise RuntimeError("ego world pose unavailable")
        pose = self.model_states.pose[self.model_index[self.ego_model]]
        x_m, y_m, yaw_rad = self._world_to_route(
            pose.position.x,
            pose.position.y,
            execution_core.quaternion_yaw(pose.orientation),
        )
        return {"x_m": x_m, "y_m": y_m, "yaw_rad": yaw_rad}

    def raw_robot_world_pose(self):
        pose = self.model_states.pose[self.model_index[self.ego_model]]
        return {
            "x_m": float(pose.position.x),
            "y_m": float(pose.position.y),
            "yaw_rad": execution_core.quaternion_yaw(pose.orientation),
        }

    def actor_world_poses(self):
        result = []
        for item in self.scene.get("actors", []):
            name = str(item["model_name"])
            if name not in self.model_index:
                continue
            pose = self.model_states.pose[self.model_index[name]]
            twist = self.model_states.twist[self.model_index[name]]
            x_m, y_m, _ = self._world_to_route(
                pose.position.x,
                pose.position.y,
                execution_core.quaternion_yaw(pose.orientation),
            )
            result.append(
                {
                    "model_name": name,
                    "x_m": x_m,
                    "y_m": y_m,
                    "radius_m": float(item.get("radius_m", 0.36)),
                    "physical_speed_mps": math.sqrt(
                        float(twist.linear.x) ** 2
                        + float(twist.linear.y) ** 2
                        + float(twist.linear.z) ** 2
                    ),
                }
            )
        return result

    def warmup(self):
        result = super().warmup()
        result["branch_sim_time_s"] = rospy.Time.now().to_sec()
        result["branch_actor_states"] = self.actor_world_poses()
        return result

    def goal_state(self, pose):
        position_error = math.hypot(
            pose["x_m"] - self.goal_world_x, pose["y_m"]
        )
        yaw_error = abs(
            execution_core.angle_delta(0.0, pose["yaw_rad"])
        )
        return (
            position_error <= self.goal_tolerance_m
            and yaw_error <= self.goal_yaw_tolerance_rad,
            position_error,
            yaw_error,
        )

    def conservative_clearance(self, pose):
        robot_eval_radius = self.combined_radius_m - 0.36
        values = [
            self.runtime.arena_map.clearance_m(
                np.asarray([pose["x_m"], pose["y_m"]], dtype=np.float64)
            )
            - robot_eval_radius
        ]
        for actor in self.actor_world_poses():
            values.append(
                math.hypot(
                    pose["x_m"] - actor["x_m"],
                    pose["y_m"] - actor["y_m"],
                )
                - robot_eval_radius
                - actor["radius_m"]
            )
        return min(values)

    def run(self):
        report = super().run()
        contact_monitor = self._finalize_contact_monitor()
        result = report.get("result", {})
        result["contact_state_count"] = contact_monitor[
            "contact_state_sample_count"
        ]
        result["contact_state_sample_count"] = contact_monitor[
            "contact_state_sample_count"
        ]
        branch_states = report.get("warmup", {}).get(
            "branch_actor_states", []
        )
        expected_actor_count = len(self.scene.get("actors", []))
        branch_state_pass = len(branch_states) == expected_actor_count
        if expected_actor_count == 1 and branch_state_pass:
            actor = branch_states[0]
            branch_state_pass = bool(
                abs(float(actor.get("x_m", float("inf")))) <= 0.01
                and abs(float(actor.get("y_m", float("inf"))) + 2.7) <= 0.01
                and abs(float(actor.get("physical_speed_mps", float("inf"))))
                <= 0.01
            )
        method_runtime_ready = bool(
            int(result.get("local_plan_count", 0)) > 0
            if self.method in ("actual_teb", "dwa")
            else int(result.get("decision_count", 0)) > 0
        )
        platform_integrity = bool(
            report.get("gate_valid")
            and report.get("message_contract_pass")
            and contact_monitor["publisher_presence_pass"]
            and contact_monitor["heartbeat_pass"]
            and int(result.get("command_count", 0)) > 0
            and method_runtime_ready
            and branch_state_pass
        )
        minimum_clearance = result.get("minimum_conservative_clearance_m")
        safety_specification_pass = bool(
            contact_monitor["contact_state_sample_count"] == 0
            and minimum_clearance is not None
            and math.isfinite(float(minimum_clearance))
            and float(minimum_clearance) >= 0.0
            and int(result.get("mask_violation_count", 0)) == 0
        )
        static_clearances = []
        for row in report.get("control_trace", []):
            pose = row.get("ego_pose") or {}
            if "x_m" in pose and "y_m" in pose:
                static_clearances.append(
                    self.runtime.arena_map.clearance_m(
                        np.asarray([pose["x_m"], pose["y_m"]], dtype=np.float64)
                    )
                    - (self.combined_radius_m - 0.36)
                )
        report.update(
            {
                "arena_study_role": "qualification_only",
                "release_sha256": self.release_sha256,
                "release_review_sha256": self.release_review_sha256,
                "arena_claim_status": "descriptive_not_formal",
                "arena_split": "qualification",
                "legacy_split_parameter": report.get("split"),
                "pose_coordinate_frame": "frozen_straight_route_frame",
                "route_transform": {
                    "origin_world_m": [
                        self.route_origin_world_x_m,
                        self.route_origin_world_y_m,
                    ],
                    "yaw_world_rad": self.route_yaw_world_rad,
                    "spawn_route_x_m": self.spawn_world_x,
                    "policy_start_route_x_m": self.policy_start_world_x,
                    "goal_route_x_m": self.goal_world_x,
                },
                "arena_static_map_audit": self.runtime.arena_map.audit(),
                "contact_monitor": contact_monitor,
                "common_branch_state_pass": branch_state_pass,
                "platform_integrity_pass_excluding_algorithm_outcome": (
                    platform_integrity
                ),
                "method_native_goal_reached": bool(result.get("goal_reached")),
                "safety_specification_pass": safety_specification_pass,
                "safety_qualified_goal": bool(
                    platform_integrity
                    and result.get("goal_reached")
                    and safety_specification_pass
                ),
                "policy_time_feature_cap_s": 22.0,
                "execution_episode_cap_s": self.timeout_s,
                "common_geometric_goal_yaw_tolerance_rad": (
                    self.goal_yaw_tolerance_rad
                ),
                "final_world_pose": self.raw_robot_world_pose(),
                "minimum_static_map_clearance_m": (
                    min(static_clearances) if static_clearances else None
                ),
                "planning_truth_boundary": (
                    "All methods use LaserScan+Odometry+TF and the frozen map, "
                    "but with different system-stack semantics: PCT/analytic "
                    "use the map for scan static/dynamic separation and hard "
                    "candidate static-clearance verification; TEB/DWA use it "
                    "through map_server/global costmap and Navfn. Gazebo actor "
                    "truth is evaluator-only."
                ),
                "contact_claim_boundary": (
                    "The heartbeat-qualified bumper stream is used to reject "
                    "observed contact states. Because this Q1 has no frozen "
                    "positive collision-sensitivity control, zero samples are "
                    "diagnostic only and are not a standalone zero-contact or "
                    "formal-safety proof."
                ),
            }
        )
        temporary = self.report_path + ".arena.tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, self.report_path)
        return report


def main():
    rospy.init_node("arena_pct_teb_dwa_qualification_episode")
    try:
        ArenaQualificationEpisode().run()
        return 0
    except Exception as error:
        rospy.logerr(
            "Arena qualification episode failed: %s: %s",
            type(error).__name__,
            error,
        )
        return 2


if __name__ == "__main__":
    sys.exit(main())
