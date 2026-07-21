#!/usr/bin/env python3
"""Simulation-gated R6 supervisor retaining measured footprint point sets."""

import hashlib
import os
from pathlib import Path
import stat
import threading

import rospy
import yaml

from nav_world_model.msg import (
    LocalGeometry,
    TrackedObstacle,
    TrackedObstacleArray,
    WorldModelHealth,
)
from teb_mode_manager.msg import ContextState, ModeTransition
from teb_mode_manager.r6_relative_ttc_supervisor import (
    ALIGNED_ESTIMATOR_ID,
    ESTIMATOR_IDS,
    FROZEN_HORIZON_S,
    FROZEN_LEGACY_CLOSEST_APPROACH_M,
    FROZEN_MINIMUM_RELATIVE_SPEED_MPS,
    FROZEN_MINIMUM_TRACK_CONFIDENCE,
    FROZEN_OVERLAY_RELEASE_CONFIRMATION_S,
    FROZEN_ROBOT_RADIUS_M,
    FootprintRuntimeTrack,
    LEGACY_ESTIMATOR_ID,
    R6RelativeTTCSupervisor,
)
from teb_mode_manager.rule_supervisor import FeatureSnapshot, SupervisorHealth
from teb_mode_manager.world_model_input_join import BoundedWorldModelInputJoin


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
WORLD_MODEL_INPUT_JOIN_LIMITS = {
    "maximum_entries_per_stream": 32,
    "maximum_arrival_age_s": 1.0,
    "maximum_sequence_lag": 2,
    "maximum_timestamp_spread_s": 0.05,
}
EXPECTED_ESTIMATOR_BY_PROFILE = {
    "r6_semantics_legacy_control": LEGACY_ESTIMATOR_ID,
    "r6_semantics_circle_contact": ALIGNED_ESTIMATOR_ID,
}


def _read_config_once(path):
    source = Path(path)
    if source.is_symlink():
        raise ValueError("R6 supervisor config cannot be a symlink")
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    try:
        descriptor = os.open(str(source), flags)
    except OSError as exc:
        raise ValueError(
            "cannot open R6 supervisor config: {}".format(exc)
        ) from exc
    try:
        metadata = os.fstat(descriptor)
        if not stat.S_ISREG(metadata.st_mode):
            raise ValueError("R6 supervisor config must be a regular file")
        chunks = []
        while True:
            chunk = os.read(descriptor, 1024 * 1024)
            if not chunk:
                break
            chunks.append(chunk)
        return b"".join(chunks)
    finally:
        os.close(descriptor)


def _load_config(path, expected_sha256, attempt_profile_id):
    if (
        not isinstance(expected_sha256, str)
        or len(expected_sha256) != 64
        or any(character not in "0123456789abcdef"
               for character in expected_sha256)
    ):
        raise ValueError("R6 supervisor config SHA256 is invalid")
    if attempt_profile_id not in EXPECTED_ESTIMATOR_BY_PROFILE:
        raise ValueError("R6 attempt profile identity is invalid")
    payload = _read_config_once(path)
    if hashlib.sha256(payload).hexdigest() != expected_sha256:
        raise ValueError("R6 reviewed supervisor config hash drifted")
    try:
        data = yaml.safe_load(payload.decode("utf-8"))
    except (UnicodeDecodeError, yaml.YAMLError) as exc:
        raise ValueError(
            "cannot parse R6 supervisor config: {}".format(exc)
        ) from exc
    if not isinstance(data, dict):
        raise ValueError("R6 rule-supervisor config root must be a mapping")
    required = {
        "schema_version",
        "architecture_generation",
        "stage",
        "profile_id",
        "status",
        "simulation_only",
        "runtime_ready",
        "training_allowed",
        "parameter_write_enabled",
        "real_vehicle_use_forbidden",
        "allow_unfrozen_simulation_candidate_required",
        "topics",
        "geometry",
        "dynamic",
        "transition",
        "health",
        "policy_boundary",
    }
    if set(data) != required:
        raise ValueError("R6 rule-supervisor config keys drifted")
    if not (
        str(data["schema_version"]) == "2.0"
        and data["architecture_generation"] == "v2"
        and data["stage"] == "V2-04G-R6-I1"
        and isinstance(data["profile_id"], str)
        and data["profile_id"]
        and data["status"] == "calibration_candidate"
        and data["simulation_only"] is True
        and data["runtime_ready"] is False
        and data["training_allowed"] is False
        and data["parameter_write_enabled"] is False
        and data["real_vehicle_use_forbidden"] is True
        and data["allow_unfrozen_simulation_candidate_required"] is True
    ):
        raise ValueError("R6 rule-supervisor safety boundary drifted")
    dynamic = data["dynamic"]
    if set(dynamic) != {
        "minimum_track_confidence",
        "predicted_ttc_max_s",
        "closest_approach_max_m",
        "conflict_estimator_id",
        "robot_radius_m",
        "minimum_relative_speed_mps",
    }:
        raise ValueError("R6 dynamic estimator fields drifted")
    if dynamic["conflict_estimator_id"] not in ESTIMATOR_IDS:
        raise ValueError("R6 conflict estimator identity is invalid")
    if (
        dynamic["conflict_estimator_id"]
        != EXPECTED_ESTIMATOR_BY_PROFILE[attempt_profile_id]
    ):
        raise ValueError(
            "R6 attempt profile does not match conflict estimator"
        )
    frozen = (
        (
            dynamic["minimum_track_confidence"],
            FROZEN_MINIMUM_TRACK_CONFIDENCE,
            "minimum track confidence",
        ),
        (
            dynamic["predicted_ttc_max_s"],
            FROZEN_HORIZON_S,
            "TTC horizon",
        ),
        (
            dynamic["closest_approach_max_m"],
            FROZEN_LEGACY_CLOSEST_APPROACH_M,
            "legacy closest approach",
        ),
        (
            dynamic["robot_radius_m"],
            FROZEN_ROBOT_RADIUS_M,
            "robot radius",
        ),
        (
            dynamic["minimum_relative_speed_mps"],
            FROZEN_MINIMUM_RELATIVE_SPEED_MPS,
            "minimum relative speed",
        ),
        (
            data["transition"]["overlay_release_confirmation_s"],
            FROZEN_OVERLAY_RELEASE_CONFIRMATION_S,
            "overlay release confirmation",
        ),
    )
    for actual, expected, label in frozen:
        if type(actual) not in (int, float) or float(actual) != expected:
            raise ValueError("{} is frozen at {}".format(label, expected))
    boundary = data["policy_boundary"]
    if boundary["runtime_scene_labels_allowed"] is not False:
        raise ValueError("runtime scene labels are forbidden")
    if boundary["runtime_manifest_access"] is not False:
        raise ValueError("runtime manifest access is forbidden")
    if boundary["published_parameter_transactions"] is not False:
        raise ValueError("R6 supervisor cannot publish parameter transactions")
    if boundary["published_velocity_commands"] is not False:
        raise ValueError("R6 supervisor cannot publish velocity commands")
    forbidden_topics = set(boundary.get("forbidden_runtime_topics", ()))
    if not {
        "/gazebo/model_states",
        "/pedsim_simulator/simulated_agents",
    }.issubset(forbidden_topics):
        raise ValueError("R6 runtime truth-topic boundary drifted")
    return data


def _footprint_tracks(message):
    """Adapt tracks without pre-compressing their measured footprint."""

    return [
        FootprintRuntimeTrack(
            track_id=item.track_id,
            motion_class=MOTION_NAME.get(item.motion_class, "UNKNOWN"),
            x=item.pose.pose.position.x,
            y=item.pose.pose.position.y,
            vx=item.velocity.twist.linear.x,
            vy=item.velocity.twist.linear.y,
            footprint=tuple(
                (point.x, point.y) for point in item.footprint.points
            ),
            confidence=item.confidence,
        )
        for item in message.obstacles
    ]


class R6RuleSupervisorNode:
    def __init__(self):
        self.attempt_profile_id = rospy.get_param("~attempt_profile_id")
        self.config = _load_config(
            rospy.get_param("~config"),
            rospy.get_param("~config_sha256"),
            self.attempt_profile_id,
        )
        if not rospy.get_param("/m2_gazebo/simulation_only", False):
            raise RuntimeError(
                "R6 rule supervisor requires simulation-only marker"
            )
        if not rospy.get_param(
            "~allow_unfrozen_simulation_candidate", False
        ):
            raise RuntimeError(
                "unfrozen R6 candidate requires explicit simulation opt-in"
            )
        self.lock = threading.RLock()
        self.input_join = BoundedWorldModelInputJoin(
            **WORLD_MODEL_INPUT_JOIN_LIMITS
        )
        self.supervisor = R6RelativeTTCSupervisor(self.config)
        topics = self.config["topics"]
        self.context_publisher = rospy.Publisher(
            topics["context"], ContextState, queue_size=2, latch=True
        )
        self.transition_publisher = rospy.Publisher(
            topics["transition"], ModeTransition, queue_size=5
        )
        rospy.Subscriber(
            topics["local_geometry"],
            LocalGeometry,
            self._geometry,
            queue_size=32,
        )
        rospy.Subscriber(
            topics["tracks"],
            TrackedObstacleArray,
            self._tracks,
            queue_size=32,
        )
        rospy.Subscriber(
            topics["health"],
            WorldModelHealth,
            self._health,
            queue_size=32,
        )
        self.timer = rospy.Timer(rospy.Duration(0.20), self._tick)

    def _geometry(self, message):
        with self.lock:
            self._add_input("geometry", message)

    def _tracks(self, message):
        with self.lock:
            self._add_input("tracks", message)

    def _health(self, message):
        with self.lock:
            self._add_input("health", message)

    def _add_input(self, stream, message):
        self.input_join.add(
            stream,
            int(message.world_model_seq),
            message.header.stamp.to_sec(),
            rospy.Time.now().to_sec(),
            message,
        )

    def _tick(self, event):
        try:
            self._tick_checked(event)
        except Exception as exc:
            # rospy.Timer logs callback exceptions but otherwise leaves the
            # process alive, which could preserve a stale latched context.
            # Publish an explicit fault and terminate the required node.
            try:
                with self.lock:
                    self._publish_missing(
                        rospy.Time.now(),
                        "r6_supervisor_runtime_fault_{}".format(
                            type(exc).__name__.lower()
                        ),
                    )
            except Exception as publish_exc:
                rospy.logerr(
                    "R6 supervisor could not publish terminal fault: %s",
                    publish_exc,
                )
            rospy.logfatal("R6 supervisor runtime fault: %s", exc)
            rospy.signal_shutdown(
                "R6 supervisor runtime fault: {}".format(exc)
            )

    def _tick_checked(self, _event):
        with self.lock:
            now = rospy.Time.now()
            joined = self.input_join.resolve(now.to_sec())
            if not joined.valid:
                self._publish_missing(
                    now,
                    "world_model_input_join_{}".format(
                        joined.reason.lower()
                    ),
                )
                return
            geometry = joined.payloads["geometry"]
            tracks_message = joined.payloads["tracks"]
            health = joined.payloads["health"]
            age = max(0.0, (now - geometry.header.stamp).to_sec())
            valid = (
                geometry.valid
                and not geometry.stale
                and health.valid
                and not health.stale
                and age <= self.config["health"]["maximum_input_age_s"]
            )
            reason = ""
            if age > self.config["health"]["maximum_input_age_s"]:
                reason = "world_model_input_stale"
            elif not health.valid or health.stale:
                reason = health.fault_reason or "world_model_health_fault"
            elif not geometry.valid or geometry.stale:
                reason = "local_geometry_invalid"
            snapshot = FeatureSnapshot(
                world_model_seq=geometry.world_model_seq,
                stamp_s=now.to_sec(),
                front_clearance_m=geometry.front_clearance_m,
                rear_clearance_m=geometry.rear_clearance_m,
                obstacle_density=geometry.obstacle_density,
                static_persistence=geometry.static_persistence,
                corridor_width_m=geometry.corridor_width_m,
                corridor_parallel_confidence=(
                    geometry.corridor_parallel_confidence
                ),
                dead_end_score=geometry.dead_end_score,
                path_curvature=geometry.path_curvature,
                goal_direction_stability=geometry.goal_direction_stability,
                rear_covered=geometry.rear_covered,
                signed_heading_error_rad=geometry.signed_heading_error_rad,
                left_clearance_m=geometry.left_clearance_m,
                right_clearance_m=geometry.right_clearance_m,
            )
            decision = self.supervisor.update(
                snapshot,
                _footprint_tracks(tracks_message),
                SupervisorHealth(
                    valid=valid, stale=not valid, fault_reason=reason
                ),
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
        message.transition_state = TRANSITION_ENUM[
            decision.transition_state
        ]
        message.mode_confidence = decision.confidence
        message.mode_dwell = rospy.Duration(decision.mode_dwell_s)
        message.minimum_dwell_remaining = rospy.Duration(
            decision.minimum_dwell_remaining_s
        )
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
            transition.minimum_dwell_remaining = rospy.Duration(
                event.minimum_dwell_remaining_s
            )
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
    rospy.init_node("r6_rule_context_supervisor")
    try:
        R6RuleSupervisorNode()
    except (RuntimeError, ValueError) as exc:
        rospy.logfatal("R6 rule supervisor denied: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
