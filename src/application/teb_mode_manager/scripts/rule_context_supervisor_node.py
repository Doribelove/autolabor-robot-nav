#!/usr/bin/env python3
"""Simulation-gated label-free V2-03 rule context supervisor."""

from pathlib import Path
import threading

import rospy
import yaml

from nav_world_model.msg import LocalGeometry, TrackedObstacle, TrackedObstacleArray, WorldModelHealth
from teb_mode_manager.msg import ContextState, ModeTransition
from teb_mode_manager.rule_supervisor import (
    FeatureSnapshot,
    RuleContextSupervisor,
    RuntimeTrack,
    SupervisorHealth,
)


GEOMETRY_ENUM = {
    "BALANCED": ContextState.GEOMETRY_BALANCED,
    "CRUISE": ContextState.GEOMETRY_CRUISE,
    "STATIC_DENSE": ContextState.GEOMETRY_STATIC_DENSE,
    "CORRIDOR": ContextState.GEOMETRY_CORRIDOR,
    "MANEUVER": ContextState.GEOMETRY_MANEUVER,
}
DYNAMIC_ENUM = {
    "NONE": ContextState.DYNAMIC_NONE,
    "CROSSING": ContextState.DYNAMIC_CROSSING,
    "HEAD_ON": ContextState.DYNAMIC_HEAD_ON,
    "FOLLOW": ContextState.DYNAMIC_FOLLOW,
    "OVERTAKE_OR_YIELD": ContextState.DYNAMIC_OVERTAKE_OR_YIELD,
}
TRANSITION_ENUM = {
    "STABLE": ContextState.TRANSITION_STABLE,
    "ENTERING": ContextState.TRANSITION_ENTERING,
    "HOLDING": ContextState.TRANSITION_HOLDING,
    "FAULTED": ContextState.TRANSITION_FAULTED,
}
MOTION_NAME = {
    TrackedObstacle.MOTION_UNKNOWN: "UNKNOWN",
    TrackedObstacle.MOTION_STATIONARY: "STATIONARY",
    TrackedObstacle.MOTION_CROSSING: "CROSSING",
    TrackedObstacle.MOTION_HEAD_ON: "HEAD_ON",
    TrackedObstacle.MOTION_FOLLOWING: "FOLLOWING",
    TrackedObstacle.MOTION_DEPARTING: "DEPARTING",
}


def _load_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("rule-supervisor config root must be a mapping")
    required = {
        "schema_version", "architecture_generation", "profile_id", "status",
        "simulation_only", "runtime_ready", "training_allowed",
        "parameter_write_enabled", "real_vehicle_use_forbidden",
        "allow_unfrozen_simulation_candidate_required", "topics", "geometry",
        "dynamic", "transition", "health", "policy_boundary",
    }
    if set(data) != required:
        raise ValueError("rule-supervisor config keys drifted")
    if not (str(data["schema_version"]) == "2.0"
            and data["architecture_generation"] == "v2"
            and data["simulation_only"] is True
            and data["runtime_ready"] is False
            and data["training_allowed"] is False
            and data["parameter_write_enabled"] is False
            and data["real_vehicle_use_forbidden"] is True):
        raise ValueError("rule-supervisor candidate safety boundary drifted")
    boundary = data["policy_boundary"]
    if boundary["runtime_scene_labels_allowed"] is not False:
        raise ValueError("runtime scene labels are forbidden")
    if boundary["runtime_manifest_access"] is not False:
        raise ValueError("runtime manifest access is forbidden")
    if boundary["published_parameter_transactions"] is not False:
        raise ValueError("V2-03 cannot publish parameter transactions")
    if boundary["published_velocity_commands"] is not False:
        raise ValueError("V2-03 cannot publish velocity commands")
    return data


class RuleSupervisorNode:
    def __init__(self):
        self.config = _load_config(rospy.get_param("~config"))
        if not rospy.get_param("/m2_gazebo/simulation_only", False):
            raise RuntimeError("V2-03 rule supervisor requires simulation-only marker")
        if not rospy.get_param("~allow_unfrozen_simulation_candidate", False):
            raise RuntimeError("unfrozen V2-03 candidate requires explicit simulation opt-in")
        self.lock = threading.RLock()
        self.geometry = None
        self.tracks = None
        self.health = None
        self.supervisor = RuleContextSupervisor(self.config)
        topics = self.config["topics"]
        self.context_publisher = rospy.Publisher(
            topics["context"], ContextState, queue_size=2, latch=True
        )
        self.transition_publisher = rospy.Publisher(
            topics["transition"], ModeTransition, queue_size=5
        )
        rospy.Subscriber(topics["local_geometry"], LocalGeometry, self._geometry, queue_size=2)
        rospy.Subscriber(topics["tracks"], TrackedObstacleArray, self._tracks, queue_size=2)
        rospy.Subscriber(topics["health"], WorldModelHealth, self._health, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(0.20), self._tick)

    def _geometry(self, message):
        with self.lock:
            self.geometry = message

    def _tracks(self, message):
        with self.lock:
            self.tracks = message

    def _health(self, message):
        with self.lock:
            self.health = message

    def _tick(self, _event):
        with self.lock:
            now = rospy.Time.now()
            if self.geometry is None or self.tracks is None or self.health is None:
                self._publish_missing(now, "world_model_inputs_missing")
                return
            sequences = {
                self.geometry.world_model_seq,
                self.tracks.world_model_seq,
                self.health.world_model_seq,
            }
            matching = len(sequences) == 1
            age = max(0.0, (now - self.geometry.header.stamp).to_sec())
            valid = (
                self.geometry.valid and not self.geometry.stale
                and self.health.valid and not self.health.stale
                and matching
                and age <= self.config["health"]["maximum_input_age_s"]
            )
            reason = ""
            if not matching:
                reason = "world_model_sequence_mismatch"
            elif age > self.config["health"]["maximum_input_age_s"]:
                reason = "world_model_input_stale"
            elif not self.health.valid or self.health.stale:
                reason = self.health.fault_reason or "world_model_health_fault"
            elif not self.geometry.valid or self.geometry.stale:
                reason = "local_geometry_invalid"
            snapshot = FeatureSnapshot(
                world_model_seq=self.geometry.world_model_seq,
                stamp_s=now.to_sec(),
                front_clearance_m=self.geometry.front_clearance_m,
                rear_clearance_m=self.geometry.rear_clearance_m,
                obstacle_density=self.geometry.obstacle_density,
                static_persistence=self.geometry.static_persistence,
                corridor_width_m=self.geometry.corridor_width_m,
                corridor_parallel_confidence=self.geometry.corridor_parallel_confidence,
                dead_end_score=self.geometry.dead_end_score,
                path_curvature=self.geometry.path_curvature,
                goal_direction_stability=self.geometry.goal_direction_stability,
                rear_covered=self.geometry.rear_covered,
                signed_heading_error_rad=self.geometry.signed_heading_error_rad,
                left_clearance_m=self.geometry.left_clearance_m,
                right_clearance_m=self.geometry.right_clearance_m,
            )
            tracks = [
                RuntimeTrack(
                    track_id=item.track_id,
                    motion_class=MOTION_NAME.get(item.motion_class, "UNKNOWN"),
                    x=item.pose.pose.position.x,
                    y=item.pose.pose.position.y,
                    vx=item.velocity.twist.linear.x,
                    vy=item.velocity.twist.linear.y,
                    radius=max((abs(point.x) for point in item.footprint.points), default=0.25),
                    confidence=item.confidence,
                )
                for item in self.tracks.obstacles
            ]
            decision = self.supervisor.update(
                snapshot, tracks, SupervisorHealth(valid=valid, stale=not valid, fault_reason=reason)
            )
            self._publish_decision(now, decision)

    def _publish_decision(self, stamp, decision):
        message = ContextState()
        message.header.stamp = stamp
        message.header.frame_id = "base_link"
        message.schema_version = "2.0"
        message.world_model_seq = decision.world_model_seq
        message.mode_seq = decision.mode_seq
        message.geometry_mode = GEOMETRY_ENUM[decision.geometry_mode]
        message.dynamic_overlay = DYNAMIC_ENUM[decision.dynamic_overlay]
        message.transition_state = TRANSITION_ENUM[decision.transition_state]
        message.mode_confidence = decision.confidence
        message.mode_dwell = rospy.Duration(decision.mode_dwell_s)
        message.minimum_dwell_remaining = rospy.Duration(decision.minimum_dwell_remaining_s)
        message.valid = decision.valid
        message.reason = decision.reason
        self.context_publisher.publish(message)
        if decision.transition is not None:
            event = decision.transition
            transition = ModeTransition()
            transition.header = message.header
            transition.schema_version = "2.0"
            transition.mode_seq = event.mode_seq
            transition.from_geometry_mode = GEOMETRY_ENUM[event.from_mode]
            transition.to_geometry_mode = GEOMETRY_ENUM[event.to_mode]
            transition.dynamic_overlay = DYNAMIC_ENUM[event.overlay]
            transition.transition_state = TRANSITION_ENUM[event.state]
            transition.progress = event.progress
            transition.minimum_dwell_remaining = rospy.Duration(event.minimum_dwell_remaining_s)
            transition.reason = event.reason
            transition.valid = event.valid
            self.transition_publisher.publish(transition)

    def _publish_missing(self, stamp, reason):
        message = ContextState()
        message.header.stamp = stamp
        message.header.frame_id = "base_link"
        message.schema_version = "2.0"
        message.geometry_mode = ContextState.GEOMETRY_BALANCED
        message.dynamic_overlay = ContextState.DYNAMIC_NONE
        message.transition_state = ContextState.TRANSITION_FAULTED
        message.valid = False
        message.reason = reason
        self.context_publisher.publish(message)


def main():
    rospy.init_node("rule_context_supervisor")
    RuleSupervisorNode()
    rospy.spin()


if __name__ == "__main__":
    main()
