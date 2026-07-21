#!/usr/bin/env python3
"""Simulation-gated V2-04 rule Anchor/profile shadow transaction loop."""

import threading

import rospy

from teb_mode_manager.action_pipeline import ActionPipelineError, AnchorBank, RuleAnchorTransactionLoop
from teb_mode_manager.msg import ContextState, ParameterTransaction


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


class RuleAnchorTransactionNode:
    def __init__(self):
        if not rospy.get_param("/m2_gazebo/simulation_only", False):
            raise RuntimeError("V2-04 transaction loop requires simulation-only marker")
        if not rospy.get_param("~allow_unfrozen_simulation_candidate", False):
            raise RuntimeError("unfrozen V2-04 candidate requires explicit simulation opt-in")
        if rospy.get_param("~allow_dynamic_reconfigure", False):
            raise RuntimeError("V2-04 real/dynamic_reconfigure execution is not implemented")
        self.bank = AnchorBank.from_file(rospy.get_param("~anchor_bank"))
        self.loop = RuleAnchorTransactionLoop(self.bank)
        self.context = None
        self.lock = threading.RLock()
        self.context_maximum_age_s = float(rospy.get_param("~context_maximum_age_s", 0.40))
        if self.context_maximum_age_s <= 0.0:
            raise RuntimeError("context_maximum_age_s must be positive")
        input_topic = rospy.get_param("~context_topic", "/teb_mode_manager/context")
        output_topic = rospy.get_param("~action_trace_topic", "/teb_rl_v2/action_trace")
        self.publisher = rospy.Publisher(output_topic, ParameterTransaction, queue_size=10)
        rospy.Subscriber(input_topic, ContextState, self._context, queue_size=2)
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.loop.frequency_hz), self._tick)

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
                self._publish(now, trace)
                return
            geometry = GEOMETRY_NAME.get(context.geometry_mode)
            overlay = DYNAMIC_NAME.get(context.dynamic_overlay)
            transition = TRANSITION_NAME.get(context.transition_state)
            enum_valid = geometry is not None and overlay is not None and transition is not None
            age_s = max(0.0, (now - context.header.stamp).to_sec())
            context_valid = bool(context.valid and enum_valid and age_s <= self.context_maximum_age_s)
            if not enum_valid:
                geometry, overlay, transition = "BALANCED", "NONE", "FAULTED"
            trace = self.loop.update(
                now.to_sec(), context.world_model_seq, context.mode_seq,
                geometry, overlay, transition, context_valid,
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
        message.geometry_mode = {name: value for value, name in GEOMETRY_NAME.items()}[
            trace.geometry_mode
        ]
        message.dynamic_overlay = {name: value for value, name in DYNAMIC_NAME.items()}[
            trace.dynamic_overlay
        ]
        message.transition_state = {name: value for value, name in TRANSITION_NAME.items()}[
            trace.transition_state
        ]
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
    rospy.init_node("rule_anchor_transaction")
    try:
        RuleAnchorTransactionNode()
    except ActionPipelineError as exc:
        rospy.logfatal("V2-04 action pipeline rejected configuration: %s", exc)
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
