#!/usr/bin/env python3
"""Run one preregistered V2-04G calibration-only mechanism episode."""

import argparse
import importlib.util
import json
import math
from pathlib import Path

import rospy
import yaml
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String

from teb_mode_manager.msg import ContextState
from nav_world_model.msg import TrackedObstacle, TrackedObstacleArray, WorldModelHealth
from nav_world_model.risk_evidence import (
    RelativeTrack,
    classify_ttc_evidence,
    earliest_relative_ttc,
    oriented_box_clearance,
    rectangular_footprint_clearance,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
LEGACY_RUNNER = Path(__file__).with_name("v2_04d_validation_episode.py")
_SPEC = importlib.util.spec_from_file_location("v2_04d_episode_frozen", LEGACY_RUNNER)
_LEGACY = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_LEGACY)

DYNAMIC_NAMES = {
    ContextState.DYNAMIC_NONE: "NONE",
    ContextState.DYNAMIC_CROSSING: "CROSSING",
    ContextState.DYNAMIC_HEAD_ON: "HEAD_ON",
    ContextState.DYNAMIC_FOLLOW: "FOLLOW",
    ContextState.DYNAMIC_OVERTAKE_OR_YIELD: "OVERTAKE_OR_YIELD",
}


class SupervisorRepairEpisode(_LEGACY.ValidationEpisode):
    """Add mode occupancy evidence while retaining the frozen V2 evaluator."""

    def __init__(self, instance, method, output_dir, stage, split, profile_id):
        if stage not in (
            "V2-04E", "V2-04E2", "V2-04E3", "V2-04E4", "V2-04F", "V2-04G"
        ):
            raise RuntimeError("unsupported supervisor-repair stage")
        if split not in ("calibration", "validation"):
            raise RuntimeError("unsupported supervisor-repair split")
        if instance["scene"]["split"] != split:
            raise RuntimeError("episode split and compiled scene split disagree")
        self.stage = stage
        self.requested_split = split
        self.profile_id = profile_id
        self.latest_tracker_healthy = False
        self.model_states = None
        self.minimum_signed_scan_clearance_m = float("inf")
        self.minimum_truth_clearance_m = float("inf")
        self.minimum_clearance_detail = None
        self.minimum_truth_detail = None
        self.mechanism_message_count = 0
        self.mechanism_topology_locked_count = 0
        self.mechanism_corridor_count = 0
        self.mechanism_reverse_count = 0
        self.mechanism_topology_switch_max = 0
        self.context_geometry_counts = {name: 0 for name in _LEGACY.GEOMETRY_NAMES.values()}
        self.context_overlay_counts = {name: 0 for name in DYNAMIC_NAMES.values()}
        super().__init__(instance, method, output_dir)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states, queue_size=2)
        rospy.Subscriber(
            "/teb_mode_manager/mechanism_state", String,
            self._mechanism_state, queue_size=10,
        )

    def _health(self, message):
        super()._health(message)
        if self.stage == "V2-04G":
            with self.lock:
                self.latest_tracker_healthy = bool(
                    message.valid and not message.stale and message.tracker_valid
                )

    def _tracks(self, message):
        if self.stage != "V2-04G":
            return super()._tracks(message)
        motion_names = {
            TrackedObstacle.MOTION_UNKNOWN: "UNKNOWN",
            TrackedObstacle.MOTION_STATIONARY: "STATIONARY",
            TrackedObstacle.MOTION_CROSSING: "CROSSING",
            TrackedObstacle.MOTION_HEAD_ON: "HEAD_ON",
            TrackedObstacle.MOTION_FOLLOWING: "FOLLOWING",
            TrackedObstacle.MOTION_DEPARTING: "DEPARTING",
        }
        tracks = []
        for obstacle in message.obstacles:
            radius = max(
                (math.hypot(point.x, point.y) for point in obstacle.footprint.points),
                default=0.25,
            )
            tracks.append(RelativeTrack(
                x=float(obstacle.pose.pose.position.x),
                y=float(obstacle.pose.pose.position.y),
                vx=float(obstacle.velocity.twist.linear.x),
                vy=float(obstacle.velocity.twist.linear.y),
                radius=radius,
                confidence=float(obstacle.confidence),
                motion_class=motion_names.get(obstacle.motion_class, "UNKNOWN"),
            ))
        with self.lock:
            self.tracker_message_count += 1
            self.predicted_ttc_s = (
                earliest_relative_ttc(tracks) if self.latest_tracker_healthy else None
            )

    def _model_states(self, message):
        if self.stage == "V2-04G":
            with self.lock:
                self.model_states = message

    def _mechanism_state(self, message):
        if self.stage != "V2-04G":
            return
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            return
        with self.lock:
            self.mechanism_message_count += 1
            self.mechanism_topology_locked_count += int(bool(value.get("topology_locked")))
            self.mechanism_corridor_count += int(bool(value.get("corridor_centerline_active")))
            self.mechanism_reverse_count += int(bool(value.get("maneuver_reverse")))
            self.mechanism_topology_switch_max = max(
                self.mechanism_topology_switch_max,
                int(value.get("topology_switch_count", 0)),
            )

    def _truth_clearance(self):
        message = self.model_states
        if message is None or "autolabor_m2" not in message.name:
            return None
        by_name = {name: pose for name, pose in zip(message.name, message.pose)}
        robot_pose = by_name["autolabor_m2"]
        first = (
            robot_pose.position.x, robot_pose.position.y, _LEGACY._yaw(robot_pose.orientation)
        )
        candidates = []
        for item in self.scene["static_obstacles"] + self.scene["dynamic_agents"]:
            name = item.get("obstacle_id", item.get("agent_id"))
            if name not in by_name:
                continue
            pose = by_name[name]
            value = oriented_box_clearance(
                first, (1.04, 0.70),
                (pose.position.x, pose.position.y, _LEGACY._yaw(pose.orientation)),
                (float(item["size_m"][0]), float(item["size_m"][1])),
            )
            candidates.append((value, name, pose.position.x, pose.position.y))
        return min(candidates) if candidates else None

    def _sample(self, goal_reached=False):
        row = super()._sample(goal_reached=goal_reached)
        if self.stage != "V2-04G" or row is None:
            return row
        with self.lock:
            scan = self.scan
            evidence = rectangular_footprint_clearance(scan)
            row["clearance_m"] = evidence.clipped_clearance_m
            if evidence.signed_clearance_m < self.minimum_signed_scan_clearance_m:
                self.minimum_signed_scan_clearance_m = evidence.signed_clearance_m
                self.minimum_clearance_detail = {
                    "stamp_s": row["stamp_s"],
                    "robot_x_m": row["x_m"], "robot_y_m": row["y_m"],
                    "signed_scan_clearance_m": evidence.signed_clearance_m,
                    "clipped_scan_clearance_m": evidence.clipped_clearance_m,
                    "raw_range_m": evidence.raw_range_m,
                    "ray_index": evidence.ray_index,
                    "ray_angle_rad": evidence.ray_angle_rad,
                    "footprint_boundary_range_m": evidence.footprint_boundary_range_m,
                }
            truth = self._truth_clearance()
            if truth is not None and truth[0] < self.minimum_truth_clearance_m:
                self.minimum_truth_clearance_m = truth[0]
                self.minimum_truth_detail = {
                    "stamp_s": row["stamp_s"], "clearance_m": truth[0],
                    "closest_model": truth[1], "model_x_m": truth[2], "model_y_m": truth[3],
                }
        return row

    def _context(self, message):
        super()._context(message)
        if not message.valid:
            return
        with self.lock:
            geometry = _LEGACY.GEOMETRY_NAMES.get(message.geometry_mode, "UNKNOWN")
            overlay = DYNAMIC_NAMES.get(message.dynamic_overlay, "UNKNOWN")
            self.context_geometry_counts[geometry] = (
                self.context_geometry_counts.get(geometry, 0) + 1
            )
            self.context_overlay_counts[overlay] = (
                self.context_overlay_counts.get(overlay, 0) + 1
            )

    def _wait_ready(self):
        super()._wait_ready()
        # Readiness traffic proves interface health but is not navigation-time
        # behavior. Start occupancy/chatter measurement immediately before the
        # goal is sent by the inherited runner.
        with self.lock:
            self.context_message_count = 0
            self.context_valid_count = 0
            self.context_geometries = []
            self.context_geometry_counts = {
                name: 0 for name in _LEGACY.GEOMETRY_NAMES.values()
            }
            self.context_overlay_counts = {name: 0 for name in DYNAMIC_NAMES.values()}
            self.active_anchors = [self.active_anchor] if self.active_anchor else []

    def run(self):
        # The V2-04D ROS runner asserts a validation split before collecting a
        # trace. Adapt only that assertion; restore the original compiled scene
        # before the unchanged evaluator verifies its instance hash.
        original_split = self.scene["split"]
        original_evaluator = _LEGACY.evaluate_v2_episode

        def evaluate_original_instance(instance, rows, raw_trace_sha256):
            adapted_split = instance["scene"]["split"]
            instance["scene"]["split"] = original_split
            try:
                return original_evaluator(instance, rows, raw_trace_sha256)
            finally:
                instance["scene"]["split"] = adapted_split

        if self.requested_split == "calibration":
            self.scene["split"] = "validation"
            _LEGACY.evaluate_v2_episode = evaluate_original_instance
        try:
            evaluation = super().run()
        finally:
            self.scene["split"] = original_split
            _LEGACY.evaluate_v2_episode = original_evaluator
        total_geometry = sum(self.context_geometry_counts.values())
        total_overlay = sum(self.context_overlay_counts.values())
        evaluation.update({
            "stage": self.stage,
            "split": self.requested_split,
            "supervisor_profile_id": self.profile_id,
            "context_geometry_sample_counts": dict(self.context_geometry_counts),
            "context_geometry_sample_fractions": {
                key: (float(value) / total_geometry if total_geometry else 0.0)
                for key, value in self.context_geometry_counts.items()
            },
            "context_overlay_sample_counts": dict(self.context_overlay_counts),
            "context_overlay_sample_fractions": {
                key: (float(value) / total_overlay if total_overlay else 0.0)
                for key, value in self.context_overlay_counts.items()
            },
            "experiment_manager_calibration_manifest_access": (
                self.requested_split == "calibration"
            ),
            "experiment_manager_validation_manifest_access": (
                self.requested_split == "validation"
            ),
            "mode_measurement_window": "post_readiness_goal_execution_only",
        })
        if self.stage == "V2-04G":
            evaluation["ttc_status"] = classify_ttc_evidence(
                tracker_message_count=self.tracker_message_count,
                healthy_tracker_sample_count=self.health_valid_count,
                finite_ttc_sample_count=self.finite_ttc_sample_count,
            )
            signed = self.minimum_signed_scan_clearance_m
            truth = self.minimum_truth_clearance_m
            if not math.isfinite(signed):
                classification = "NO_VALID_SCAN_CLEARANCE"
            elif signed <= 0.0 and math.isfinite(truth) and truth <= 0.02:
                classification = "TRUE_GEOMETRIC_INTRUSION"
            elif signed <= 0.0 and math.isfinite(truth) and truth <= 0.10:
                classification = "PHYSICAL_NEAR_MISS_WITH_SCAN_INTRUSION"
            elif signed <= 0.0 and math.isfinite(truth):
                classification = "SENSOR_OR_FOOTPRINT_GEOMETRY_MISMATCH"
            elif signed <= 0.05:
                classification = "PHYSICAL_NEAR_MISS"
            else:
                classification = "CLEARANCE_POSITIVE"
            audit = {
                "schema_version": "2.0", "stage": "V2-04G",
                "evaluator_only_gazebo_truth_used": True,
                "runtime_policy_received_truth": False,
                "minimum_signed_scan_clearance_m": (
                    signed if math.isfinite(signed) else None
                ),
                "minimum_truth_box_clearance_m": truth if math.isfinite(truth) else None,
                "classification": classification,
                "minimum_scan_detail": self.minimum_clearance_detail,
                "minimum_truth_detail": self.minimum_truth_detail,
                "contact_count": self.contact_count,
            }
            audit_path = self.output_dir / "clearance_audit.yaml"
            audit_path.write_text(yaml.safe_dump(audit, sort_keys=False), encoding="utf-8")
            evaluation.update({
                "clearance_audit": audit,
                "mechanism_message_count": self.mechanism_message_count,
                "mechanism_topology_locked_sample_count": (
                    self.mechanism_topology_locked_count
                ),
                "mechanism_corridor_centerline_sample_count": self.mechanism_corridor_count,
                "mechanism_maneuver_reverse_sample_count": self.mechanism_reverse_count,
                "mechanism_topology_switch_count": self.mechanism_topology_switch_max,
            })
        (self.output_dir / "evaluation.yaml").write_text(
            yaml.safe_dump(evaluation, sort_keys=False), encoding="utf-8"
        )
        return evaluation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True)
    parser.add_argument("--method", choices=_LEGACY.METHODS, required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--stage", choices=(
            "V2-04E", "V2-04E2", "V2-04E3", "V2-04E4", "V2-04F", "V2-04G"
        ),
        required=True,
    )
    parser.add_argument("--split", choices=("calibration", "validation"), required=True)
    parser.add_argument("--profile-id", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    expected_root = WORKSPACE / "artifacts/v2" / args.split / args.stage.lower().replace("-", "_")
    output = Path(args.output_dir).resolve()
    output.relative_to(expected_root.resolve())
    instance = yaml.safe_load(Path(args.instance).read_text(encoding="utf-8"))
    rospy.init_node("{}_supervisor_episode".format(args.stage.lower().replace("-", "_")))
    report = SupervisorRepairEpisode(
        instance, args.method, output, args.stage, args.split, args.profile_id
    ).run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
