#!/usr/bin/env python3
"""V2-04G-R1 typed transaction with bounded asynchronous context join."""

import json
from pathlib import Path
import threading

import rospy
import yaml
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

from teb_mode_manager.action_pipeline import ActionPipelineError, AnchorBank
from teb_mode_manager.bounded_context_join import BoundedContextJoin
from teb_mode_manager.mechanism_action_pipeline import MechanismAnchorTransactionLoop
from teb_mode_manager.mechanism_controller import (
    MechanismSnapshot,
    RuleMechanismController,
)
from teb_mode_manager.msg import ContextState, ParameterTransaction
from nav_world_model.msg import LocalGeometry
from teb_mode_manager.typed_teb_transaction import (
    EXPECTED_TEB_NAMESPACE,
    RosTypedDynamicReconfigureAdapter,
    SimulationWriteContext,
    TypedTebTransactionBackend,
    require_simulation_write,
)


GEOMETRY_NAME = {
    ContextState.GEOMETRY_BALANCED: "BALANCED",
    ContextState.GEOMETRY_CRUISE: "CRUISE",
    ContextState.GEOMETRY_STATIC_DENSE: "STATIC_DENSE",
    ContextState.GEOMETRY_CORRIDOR: "CORRIDOR",
    ContextState.GEOMETRY_MANEUVER: "MANEUVER",
}
DYNAMIC_NAME = {
    ContextState.DYNAMIC_NONE: "NONE",
    ContextState.DYNAMIC_CROSSING: "CROSSING",
    ContextState.DYNAMIC_HEAD_ON: "HEAD_ON",
    ContextState.DYNAMIC_FOLLOW: "FOLLOW",
    ContextState.DYNAMIC_OVERTAKE_OR_YIELD: "OVERTAKE_OR_YIELD",
}
TRANSITION_NAME = {
    ContextState.TRANSITION_STABLE: "STABLE",
    ContextState.TRANSITION_ENTERING: "ENTERING",
    ContextState.TRANSITION_EXITING: "EXITING",
    ContextState.TRANSITION_HOLDING: "HOLDING",
    ContextState.TRANSITION_SAFE_OVERRIDE: "SAFE_OVERRIDE",
    ContextState.TRANSITION_FAULTED: "FAULTED",
}


def load_r1_mechanism_config(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    required = {
        "schema_version", "architecture_generation", "stage", "profile_id",
        "status", "simulation_only", "runtime_ready", "training_allowed",
        "real_vehicle_use_forbidden", "static_topology", "corridor_centerline",
        "maneuver", "dynamic_release", "policy_boundary",
    }
    if not isinstance(data, dict) or set(data) != required:
        raise ValueError("V2-04G-R1 mechanism config keys drifted")
    if not (
        str(data["schema_version"]) == "2.0"
        and data["architecture_generation"] == "v2"
        and data["stage"] == "V2-04G-R1"
        and data["status"] == "calibration_candidate"
        and data["simulation_only"] is True
        and data["runtime_ready"] is False
        and data["training_allowed"] is False
        and data["real_vehicle_use_forbidden"] is True
    ):
        raise ValueError("V2-04G-R1 mechanism safety boundary drifted")
    if data["policy_boundary"] != {
        "runtime_scene_labels_allowed": False,
        "runtime_manifest_access": False,
        "published_velocity_commands": False,
        "learned_policy_loaded": False,
    }:
        raise ValueError("V2-04G-R1 mechanism policy boundary drifted")
    return data


class SimulationTypedAnchorTransactionNode:
    def __init__(self):
        namespace = rospy.get_param("~teb_namespace", EXPECTED_TEB_NAMESPACE)
        clock_timeout_s = float(rospy.get_param("~clock_timeout_s", 10.0))
        try:
            clock = rospy.wait_for_message("/clock", Clock, timeout=clock_timeout_s)
            clock_active = clock.clock.to_sec() > 0.0
        except rospy.ROSException:
            clock_active = False
        write_context = SimulationWriteContext(
            explicit_simulation_write=rospy.get_param(
                "~allow_simulation_teb_parameter_write", False
            ) is True,
            use_sim_time=rospy.get_param("/use_sim_time", False) is True,
            simulation_marker=rospy.get_param(
                "/m2_gazebo/simulation_only", False
            ) is True,
            gazebo_clock_active=clock_active,
            teb_namespace=namespace,
        )
        require_simulation_write(write_context)
        if not rospy.get_param("~allow_unfrozen_simulation_candidate", False):
            raise RuntimeError("simulation Anchor Bank requires explicit opt-in")
        bank = AnchorBank.from_file(rospy.get_param("~anchor_bank"))
        self.force_geometry_balanced = bool(
            rospy.get_param("~force_geometry_balanced", False)
        )
        timeout_s = float(rospy.get_param("~transaction_timeout_s", 2.0))
        adapter = RosTypedDynamicReconfigureAdapter(namespace, timeout_s)
        self.backend = TypedTebTransactionBackend(
            bank,
            adapter,
            write_context,
            timeout_s=timeout_s,
            equality_tolerance=float(rospy.get_param("~equality_tolerance", 1.0e-9)),
            time_source=lambda: rospy.Time.now().to_sec(),
        )
        self.backend.initialize()
        rospy.on_shutdown(self._shutdown)
        self.loop = MechanismAnchorTransactionLoop(bank, backend=self.backend)
        self.context = None
        self.lock = threading.RLock()
        mechanism_path = str(rospy.get_param("~mechanism_config", "")).strip()
        self.mechanism = (
            RuleMechanismController(load_r1_mechanism_config(mechanism_path))
            if mechanism_path else None
        )
        self.geometry_join = BoundedContextJoin(
            maximum_entries=int(rospy.get_param("~join_maximum_entries", 32)),
            maximum_arrival_age_s=float(
                rospy.get_param("~join_maximum_arrival_age_s", 1.0)
            ),
            maximum_sequence_delta=int(
                rospy.get_param("~join_maximum_sequence_delta", 2)
            ),
            maximum_timestamp_delta_s=float(
                rospy.get_param("~join_maximum_timestamp_delta_s", 0.45)
            ),
        )
        self.context_maximum_age_s = float(rospy.get_param("~context_maximum_age_s", 0.40))
        if self.context_maximum_age_s <= 0.0:
            raise RuntimeError("context_maximum_age_s must be positive")
        self.publisher = rospy.Publisher(
            rospy.get_param("~action_trace_topic", "/teb_rl_v2/action_trace"),
            ParameterTransaction,
            queue_size=10,
        )
        self.mechanism_publisher = rospy.Publisher(
            rospy.get_param("~mechanism_state_topic", "/teb_mode_manager/mechanism_state"),
            String, queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~context_topic", "/teb_mode_manager/context"),
            ContextState,
            self._context,
            queue_size=2,
        )
        rospy.Subscriber(
            rospy.get_param("~geometry_topic", "/nav_world_model/local_geometry"),
            LocalGeometry,
            self._geometry,
            queue_size=2,
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.loop.frequency_hz), self._tick)

    def _shutdown(self):
        try:
            self.backend.close()
        except Exception as exc:
            rospy.logerr("failed to restore startup typed TEB profile: %s", exc)

    def _context(self, message):
        with self.lock:
            self.context = message

    def _geometry(self, message):
        with self.lock:
            self.geometry_join.add(
                int(message.world_model_seq),
                message.header.stamp.to_sec(),
                rospy.Time.now().to_sec(),
                message,
            )

    def _tick(self, _event):
        with self.lock:
            now = rospy.Time.now()
            context = self.context
            join_result = None
            mechanism_command = None
            if context is None:
                trace = self.loop.update(
                    now.to_sec(), 0, 0, "BALANCED", "NONE", "FAULTED", False
                )
            else:
                geometry = GEOMETRY_NAME.get(context.geometry_mode)
                overlay = DYNAMIC_NAME.get(context.dynamic_overlay)
                transition = TRANSITION_NAME.get(context.transition_state)
                enum_valid = geometry is not None and overlay is not None and transition is not None
                age_s = max(0.0, (now - context.header.stamp).to_sec())
                valid = bool(context.valid and enum_valid and age_s <= self.context_maximum_age_s)
                if not enum_valid:
                    geometry, overlay, transition = "BALANCED", "NONE", "FAULTED"
                elif self.force_geometry_balanced:
                    # V2-04D single-Anchor comparator: retain the label-free
                    # dynamic overlay while removing geometry-mode selection.
                    geometry = "BALANCED"
                if self.mechanism is not None:
                    join_result = self.geometry_join.resolve(
                        int(context.world_model_seq),
                        context.header.stamp.to_sec(),
                        now.to_sec(),
                    )
                    geometry_message = join_result.payload
                    mechanism_valid = bool(
                        join_result.valid and geometry_message is not None
                        and geometry_message.valid and not geometry_message.stale
                    )
                    valid = valid and mechanism_valid
                    if mechanism_valid:
                        mechanism_command = self.mechanism.update(
                            geometry, overlay,
                            MechanismSnapshot(
                                front_clearance_m=geometry_message.front_clearance_m,
                                rear_clearance_m=geometry_message.rear_clearance_m,
                                left_clearance_m=geometry_message.left_clearance_m,
                                right_clearance_m=geometry_message.right_clearance_m,
                                corridor_center_offset_m=(
                                    geometry_message.corridor_center_offset_m
                                ),
                                signed_heading_error_rad=(
                                    geometry_message.signed_heading_error_rad
                                ),
                                rear_covered=geometry_message.rear_covered,
                            ),
                        )
                trace = self.loop.update(
                    now.to_sec(), context.world_model_seq, context.mode_seq,
                    geometry, overlay, transition, valid,
                    maneuver_reverse=(
                        mechanism_command.maneuver_reverse if mechanism_command else False
                    ),
                    residuals=(mechanism_command.residuals if mechanism_command else {}),
                )
            if self.mechanism is not None:
                self.mechanism_publisher.publish(String(data=json.dumps({
                    "world_model_seq": int(context.world_model_seq) if context else 0,
                    "mode_seq": int(context.mode_seq) if context else 0,
                    "geometry_mode": trace.geometry_mode,
                    "dynamic_overlay": trace.dynamic_overlay,
                    "join_valid": bool(join_result and join_result.valid),
                    "join_reason": (
                        join_result.reason if join_result else "CONTEXT_MISSING"
                    ),
                    "joined_geometry_sequence": (
                        join_result.geometry_sequence if join_result else None
                    ),
                    "join_sequence_delta": (
                        join_result.sequence_delta if join_result else None
                    ),
                    "join_timestamp_delta_s": (
                        join_result.timestamp_delta_s if join_result else None
                    ),
                    "join_arrival_age_s": (
                        join_result.arrival_age_s if join_result else None
                    ),
                    "join_cache_size": (
                        join_result.cache_size if join_result else self.geometry_join.size
                    ),
                    "transaction_activated": bool(trace.activated),
                    "transaction_valid": bool(trace.valid),
                    "transaction_fault_reason": trace.fault_reason,
                    "topology_preference": (
                        mechanism_command.topology_preference
                        if mechanism_command else self.mechanism.topology_preference
                    ),
                    "topology_locked": bool(
                        mechanism_command and mechanism_command.topology_locked
                    ),
                    "topology_switch_count": self.mechanism.topology_switch_count,
                    "corridor_centerline_active": bool(
                        mechanism_command and mechanism_command.corridor_centerline_active
                    ),
                    "maneuver_reverse": bool(
                        mechanism_command and mechanism_command.maneuver_reverse
                    ),
                    "residuals": mechanism_command.residuals if mechanism_command else {},
                    "reason": (
                        mechanism_command.reason
                        if mechanism_command else "mechanism_not_applied"
                    ),
                }, sort_keys=True)))
            self._publish(now, trace)

    def _publish(self, stamp, trace):
        message = ParameterTransaction()
        message.header.stamp = stamp
        message.header.frame_id = "base_link"
        message.schema_version = "2.0"
        message.world_model_seq = trace.world_model_seq
        message.mode_seq = trace.mode_seq
        message.config_seq = trace.config_seq
        message.geometry_mode = {name: value for value, name in GEOMETRY_NAME.items()}[trace.geometry_mode]
        message.dynamic_overlay = {name: value for value, name in DYNAMIC_NAME.items()}[trace.dynamic_overlay]
        message.transition_state = {name: value for value, name in TRANSITION_NAME.items()}[trace.transition_state]
        message.anchor_id = trace.anchor_id
        message.profile_id = trace.profile_id
        message.execution_backend = trace.execution_backend
        message.parameter_names = list(trace.parameter_names)
        message.parameter_types = list(trace.parameter_types)
        message.commanded = list(trace.commanded)
        message.feasible = list(trace.feasible)
        message.safe = list(trace.safe)
        message.executed = list(trace.executed)
        message.projection_reason_mask = trace.projection_reason_mask
        message.safety_reason_mask = trace.safety_reason_mask
        message.t_request = rospy.Time.from_sec(trace.t_request_s)
        message.t_ack = rospy.Time.from_sec(trace.t_ack_s)
        message.t_readback = rospy.Time.from_sec(trace.t_readback_s)
        message.t_active = rospy.Time.from_sec(trace.t_active_s)
        message.activated = trace.activated
        message.slow_profile_committed = trace.slow_profile_committed
        message.training_used = trace.training_used
        message.valid = trace.valid
        message.fault_reason = trace.fault_reason
        self.publisher.publish(message)


def main():
    rospy.init_node("v2_04g_r1_typed_anchor_transaction")
    try:
        SimulationTypedAnchorTransactionNode()
    except (ActionPipelineError, RuntimeError, ValueError) as exc:
        rospy.logfatal("V2-04G-R1 simulation typed transaction denied: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
