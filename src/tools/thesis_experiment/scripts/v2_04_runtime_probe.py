#!/usr/bin/env python3
"""Probe the V2-04 ROS ContextState -> ParameterTransaction shadow loop."""

import argparse
from pathlib import Path
import threading
import time

import rospy
import yaml

from teb_mode_manager.msg import ContextState, ParameterTransaction


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


class Probe:
    def __init__(self):
        self.lock = threading.RLock()
        self.messages = []
        self.publisher = rospy.Publisher(
            "/teb_mode_manager/context_probe", ContextState, queue_size=2, latch=True
        )
        rospy.Subscriber(
            "/teb_rl_v2/action_trace_probe", ParameterTransaction,
            self._transaction, queue_size=50,
        )

    def _transaction(self, message):
        with self.lock:
            self.messages.append(message)

    def publish_phase(self, mode_seq, geometry, overlay, transition, valid, duration_s=1.0):
        deadline = time.monotonic() + duration_s
        world_seq = mode_seq * 1000
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            world_seq += 1
            message = ContextState()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "base_link"
            message.schema_version = "2.0"
            message.world_model_seq = world_seq
            message.mode_seq = mode_seq
            message.geometry_mode = geometry
            message.dynamic_overlay = overlay
            message.transition_state = transition
            message.mode_confidence = 0.9 if valid else 0.0
            message.valid = valid
            message.reason = "runtime_probe"
            self.publisher.publish(message)
            rospy.sleep(0.05)


def main():
    args = parse_args()
    rospy.init_node("v2_04_runtime_probe", anonymous=True)
    probe = Probe()
    deadline = time.monotonic() + 3.0
    while probe.publisher.get_num_connections() == 0 and time.monotonic() < deadline:
        rospy.sleep(0.05)
    if probe.publisher.get_num_connections() == 0:
        raise RuntimeError("V2-04 transaction node did not subscribe to probe context")
    probe.publish_phase(
        1, ContextState.GEOMETRY_CRUISE, ContextState.DYNAMIC_NONE,
        ContextState.TRANSITION_ENTERING, True,
    )
    probe.publish_phase(
        2, ContextState.GEOMETRY_CRUISE, ContextState.DYNAMIC_CROSSING,
        ContextState.TRANSITION_STABLE, True,
    )
    probe.publish_phase(
        3, ContextState.GEOMETRY_CORRIDOR, ContextState.DYNAMIC_NONE,
        ContextState.TRANSITION_ENTERING, True,
    )
    probe.publish_phase(
        4, ContextState.GEOMETRY_BALANCED, ContextState.DYNAMIC_NONE,
        ContextState.TRANSITION_FAULTED, False, duration_s=0.6,
    )
    rospy.sleep(0.3)
    with probe.lock:
        messages = list(probe.messages)
    valid = [item for item in messages if item.valid and item.activated]
    invalid = [item for item in messages if not item.valid and not item.activated]
    complete = all(
        len(item.parameter_names) == len(item.parameter_types)
        == len(item.commanded) == len(item.feasible) == len(item.safe)
        == len(item.executed) == 20
        for item in messages
    )
    zero_projection = all(item.projection_reason_mask == 0 for item in valid)
    no_training = all(item.training_used is False for item in messages)
    shadow_only = all(item.execution_backend == "deterministic_shadow" for item in messages)
    max_velocity_index = None
    maximum_delta = 0.0
    previous = None
    for item in valid:
        if max_velocity_index is None:
            max_velocity_index = list(item.parameter_names).index("max_vel_x")
        value = item.executed[max_velocity_index]
        if previous is not None:
            maximum_delta = max(maximum_delta, abs(value - previous))
        previous = value
    passed = (
        len(valid) >= 10 and len(invalid) >= 2 and complete and zero_projection
        and no_training and shadow_only and maximum_delta <= 0.11
    )
    report = {
        "schema_version": "2.0",
        "stage": "V2-04",
        "probe": "ros_context_to_shadow_parameter_transaction",
        "status": "passed" if passed else "failed",
        "simulation_only": True,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "real_teb_parameter_write_used": False,
        "received_transaction_count": len(messages),
        "valid_activated_count": len(valid),
        "invalid_hold_count": len(invalid),
        "complete_twenty_parameter_trace": complete,
        "normal_projection_count": sum(
            int(item.projection_reason_mask != 0) for item in valid
        ),
        "all_training_used_false": no_training,
        "all_execution_backend_shadow": shadow_only,
        "maximum_consecutive_max_vel_x_delta": maximum_delta,
        "observed_anchor_ids": sorted(set(item.anchor_id for item in valid)),
        "observed_mode_sequences": sorted(set(int(item.mode_seq) for item in messages)),
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(report, sort_keys=False), encoding="utf-8"
    )
    print(args.output)
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
