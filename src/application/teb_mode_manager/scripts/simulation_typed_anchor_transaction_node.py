#!/usr/bin/env python3
"""Gazebo-only rule Anchor loop backed by typed TEB dynamic_reconfigure."""

import threading

import rospy
from rosgraph_msgs.msg import Clock

from teb_mode_manager.action_pipeline import ActionPipelineError, AnchorBank, RuleAnchorTransactionLoop
from teb_mode_manager.msg import ContextState, ParameterTransaction
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
        self.loop = RuleAnchorTransactionLoop(bank, backend=self.backend)
        self.context = None
        self.lock = threading.RLock()
        self.context_maximum_age_s = float(rospy.get_param("~context_maximum_age_s", 0.40))
        if self.context_maximum_age_s <= 0.0:
            raise RuntimeError("context_maximum_age_s must be positive")
        self.publisher = rospy.Publisher(
            rospy.get_param("~action_trace_topic", "/teb_rl_v2/action_trace"),
            ParameterTransaction,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~context_topic", "/teb_mode_manager/context"),
            ContextState,
            self._context,
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

    def _tick(self, _event):
        with self.lock:
            now = rospy.Time.now()
            context = self.context
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
                trace = self.loop.update(
                    now.to_sec(), context.world_model_seq, context.mode_seq,
                    geometry, overlay, transition, valid,
                )
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
    rospy.init_node("simulation_typed_anchor_transaction")
    try:
        SimulationTypedAnchorTransactionNode()
    except (ActionPipelineError, RuntimeError) as exc:
        rospy.logfatal("V2-04B simulation typed transaction denied: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
