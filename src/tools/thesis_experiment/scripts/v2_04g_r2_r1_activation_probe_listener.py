#!/usr/bin/env python3
"""Fault-taxonomy-aware readiness listener for V2-04G-R2-R1."""

import argparse
from collections import Counter
import json
from pathlib import Path
import threading

import rospy
import yaml
from std_msgs.msg import String

from teb_mode_manager.msg import ContextState, ParameterTransaction


STAGE = "V2-04G-R2-R1"
EXPECTED_CONTEXT_HOLD_REASON = (
    "invalid_or_faulted_context_hold_previous_executed"
)
BACKEND_FAULT_PATTERNS = (
    "acknowledgement",
    "readback",
    "restore",
    "dynamic_reconfigure",
    "timed out",
    "timeout",
    "outside live bounds",
    "typed request",
)
GEOMETRY_NAMES = {
    ContextState.GEOMETRY_BALANCED: "BALANCED",
    ContextState.GEOMETRY_CRUISE: "CRUISE",
    ContextState.GEOMETRY_STATIC_DENSE: "STATIC_DENSE",
    ContextState.GEOMETRY_CORRIDOR: "CORRIDOR",
    ContextState.GEOMETRY_MANEUVER: "MANEUVER",
}
DYNAMIC_NAMES = {
    ContextState.DYNAMIC_NONE: "NONE",
    ContextState.DYNAMIC_CROSSING: "CROSSING",
    ContextState.DYNAMIC_HEAD_ON: "HEAD_ON",
    ContextState.DYNAMIC_FOLLOW: "FOLLOW",
    ContextState.DYNAMIC_OVERTAKE_OR_YIELD: "OVERTAKE_OR_YIELD",
}
TRANSITION_NAMES = {
    ContextState.TRANSITION_STABLE: "STABLE",
    ContextState.TRANSITION_ENTERING: "ENTERING",
    ContextState.TRANSITION_EXITING: "EXITING",
    ContextState.TRANSITION_HOLDING: "HOLDING",
    ContextState.TRANSITION_SAFE_OVERRIDE: "SAFE_OVERRIDE",
    ContextState.TRANSITION_FAULTED: "FAULTED",
}


def classify_fault_reason(reason):
    """Return the preregistered disjoint fault taxonomy class."""
    value = str(reason or "")
    if not value:
        return "CLEAN"
    if value == EXPECTED_CONTEXT_HOLD_REASON:
        return "EXPECTED_FAIL_CLOSED_CONTEXT_HOLD"
    lower = value.lower()
    if any(pattern in lower for pattern in BACKEND_FAULT_PATTERNS):
        return "BACKEND_TRANSACTION_FAULT"
    return "UNKNOWN_TRANSACTION_FAULT"


class ConsecutiveStableWindow:
    """Track a strict consecutive stable-message readiness barrier."""

    def __init__(self, required_count):
        if isinstance(required_count, bool) or int(required_count) <= 0:
            raise ValueError("required stable count must be positive")
        self.required_count = int(required_count)
        self.current_count = 0
        self.maximum_count = 0
        self.ready = False

    def update(self, stable):
        self.current_count = self.current_count + 1 if stable else 0
        self.maximum_count = max(self.maximum_count, self.current_count)
        self.ready = self.current_count >= self.required_count
        return self.ready


class TaxonomyProbe:
    def __init__(self, required_stable_count):
        self.lock = threading.RLock()
        self.window = ConsecutiveStableWindow(required_stable_count)
        self.measurement_enabled = False
        self.latest_context = None
        self.latest_mechanism = None
        self.warmup_mechanism_count = 0
        self.warmup_instability_counts = Counter()
        self.reset_measurement()
        rospy.Subscriber(
            "/teb_mode_manager/context", ContextState,
            self._context, queue_size=50,
        )
        rospy.Subscriber(
            "/teb_mode_manager/mechanism_state", String,
            self._mechanism, queue_size=50,
        )
        rospy.Subscriber(
            "/teb_rl_v2/action_trace", ParameterTransaction,
            self._transaction, queue_size=50,
        )

    @property
    def ready(self):
        return self.window.ready

    def reset_measurement(self):
        self.transaction_count = 0
        self.transaction_valid_count = 0
        self.transaction_activated_count = 0
        self.transaction_training_count = 0
        self.transaction_backend_counts = Counter()
        self.fault_reason_counts = Counter()
        self.fault_taxonomy_counts = Counter()
        self.fault_samples = []
        self.mechanism_count = 0
        self.join_valid_count = 0
        self.join_reason_counts = Counter()
        self.context_sequences = set()
        self.maximum_sequence_delta = 0
        self.maximum_timestamp_delta_s = 0.0
        self.context_count = 0
        self.context_valid_count = 0
        self.context_reason_counts = Counter()

    def begin_measurement(self):
        with self.lock:
            if not self.ready:
                raise RuntimeError("stable readiness window is incomplete")
            self.reset_measurement()
            self.measurement_enabled = True

    def _context(self, message):
        snapshot = {
            "source_stamp_s": message.header.stamp.to_sec(),
            "arrival_stamp_s": rospy.Time.now().to_sec(),
            "world_model_seq": int(message.world_model_seq),
            "mode_seq": int(message.mode_seq),
            "geometry_mode": GEOMETRY_NAMES.get(
                message.geometry_mode, "UNKNOWN_{}".format(message.geometry_mode)
            ),
            "dynamic_overlay": DYNAMIC_NAMES.get(
                message.dynamic_overlay, "UNKNOWN_{}".format(message.dynamic_overlay)
            ),
            "transition_state": TRANSITION_NAMES.get(
                message.transition_state, "UNKNOWN_{}".format(message.transition_state)
            ),
            "valid": bool(message.valid),
            "reason": str(message.reason),
        }
        snapshot["arrival_age_s"] = max(
            0.0, snapshot["arrival_stamp_s"] - snapshot["source_stamp_s"]
        )
        with self.lock:
            self.latest_context = snapshot
            if self.measurement_enabled:
                self.context_count += 1
                self.context_valid_count += int(snapshot["valid"])
                self.context_reason_counts[snapshot["reason"] or "CLEAN"] += 1

    def _mechanism(self, message):
        try:
            value = json.loads(message.data)
        except (TypeError, ValueError):
            value = {"decode_error": True, "raw": str(message.data)}
        with self.lock:
            self.latest_mechanism = value
            if not self.measurement_enabled:
                self.warmup_mechanism_count += 1
                stable = bool(
                    value.get("join_valid")
                    and value.get("transaction_valid")
                    and value.get("transaction_activated")
                    and not value.get("transaction_fault_reason")
                )
                if not stable:
                    reasons = []
                    if not value.get("join_valid"):
                        reasons.append("JOIN_INVALID")
                    if not value.get("transaction_valid"):
                        reasons.append("TRANSACTION_INVALID")
                    if not value.get("transaction_activated"):
                        reasons.append("TRANSACTION_NOT_ACTIVATED")
                    if value.get("transaction_fault_reason"):
                        reasons.append("TRANSACTION_FAULT")
                    self.warmup_instability_counts[
                        "+".join(reasons) or "MECHANISM_DECODE_ERROR"
                    ] += 1
                self.window.update(stable)
                return
            self.mechanism_count += 1
            self.join_valid_count += int(bool(value.get("join_valid")))
            reason = str(value.get("join_reason", "MISSING_JOIN_REASON"))
            self.join_reason_counts[reason] += 1
            sequence_delta = value.get("join_sequence_delta")
            timestamp_delta = value.get("join_timestamp_delta_s")
            if sequence_delta is not None:
                self.maximum_sequence_delta = max(
                    self.maximum_sequence_delta, int(sequence_delta)
                )
            if timestamp_delta is not None:
                self.maximum_timestamp_delta_s = max(
                    self.maximum_timestamp_delta_s, float(timestamp_delta)
                )

    def _transaction(self, message):
        with self.lock:
            if not self.measurement_enabled:
                return
            self.transaction_count += 1
            self.transaction_valid_count += int(bool(message.valid))
            self.transaction_activated_count += int(bool(message.activated))
            self.transaction_training_count += int(bool(message.training_used))
            self.transaction_backend_counts[str(message.execution_backend)] += 1
            self.context_sequences.add(int(message.world_model_seq))
            reason = str(message.fault_reason or "")
            taxonomy = classify_fault_reason(reason)
            self.fault_taxonomy_counts[taxonomy] += 1
            if reason:
                self.fault_reason_counts[reason] += 1
                self.fault_samples.append({
                    "transaction_stamp_s": message.header.stamp.to_sec(),
                    "world_model_seq": int(message.world_model_seq),
                    "mode_seq": int(message.mode_seq),
                    "config_seq": int(message.config_seq),
                    "valid": bool(message.valid),
                    "activated": bool(message.activated),
                    "geometry_mode": GEOMETRY_NAMES.get(
                        message.geometry_mode,
                        "UNKNOWN_{}".format(message.geometry_mode),
                    ),
                    "dynamic_overlay": DYNAMIC_NAMES.get(
                        message.dynamic_overlay,
                        "UNKNOWN_{}".format(message.dynamic_overlay),
                    ),
                    "transition_state": TRANSITION_NAMES.get(
                        message.transition_state,
                        "UNKNOWN_{}".format(message.transition_state),
                    ),
                    "execution_backend": str(message.execution_backend),
                    "fault_reason": reason,
                    "fault_taxonomy": taxonomy,
                    "latest_context": dict(self.latest_context or {}),
                    "latest_mechanism": dict(self.latest_mechanism or {}),
                })


def _fraction(numerator, denominator):
    return float(numerator) / denominator if denominator else 0.0


def _write_report(path, report):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--repeat", type=int, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--warmup-timeout-s", type=float, required=True)
    parser.add_argument("--measurement-duration-s", type=float, required=True)
    parser.add_argument("--minimum-message-count", type=int, required=True)
    parser.add_argument("--minimum-valid-fraction", type=float, required=True)
    parser.add_argument("--required-consecutive-stable-count", type=int, required=True)
    parser.add_argument("--maximum-expected-context-hold-count", type=int, required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("v2_04g_r2_r1_activation_probe_listener")
    probe = TaxonomyProbe(args.required_consecutive_stable_count)
    wall_start_s = rospy.get_time()
    deadline = rospy.Time.now().to_sec() + args.warmup_timeout_s
    rate = rospy.Rate(50.0)
    while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
        with probe.lock:
            ready = probe.ready
        if ready:
            break
        rate.sleep()
    else:
        ready = False
    warmup_elapsed_s = max(0.0, rospy.get_time() - wall_start_s)
    if not ready:
        report = {
            "schema_version": "2.0", "stage": STAGE,
            "status": "fail", "simulation_only": True,
            "runtime_ready": False, "training_used": False,
            "real_vehicle_used": False, "profile_id": args.profile_id,
            "repeat": args.repeat, "seed": args.seed,
            "failure_phase": "STABLE_READINESS_WARMUP",
            "warmup_elapsed_s": warmup_elapsed_s,
            "required_consecutive_stable_count": args.required_consecutive_stable_count,
            "maximum_consecutive_stable_count": probe.window.maximum_count,
            "warmup_mechanism_message_count": probe.warmup_mechanism_count,
            "warmup_instability_counts": dict(probe.warmup_instability_counts),
            "hard_gates": {"consecutive_stable_readiness": False},
            "all_hard_gates_pass": False,
        }
        _write_report(args.output, report)
        print(yaml.safe_dump(report, sort_keys=False))
        return 1
    probe.begin_measurement()
    start = rospy.Time.now().to_sec()
    while (
        not rospy.is_shutdown()
        and rospy.Time.now().to_sec() - start < args.measurement_duration_s
    ):
        rate.sleep()
    with probe.lock:
        transaction_valid_fraction = _fraction(
            probe.transaction_valid_count, probe.transaction_count
        )
        transaction_activated_fraction = _fraction(
            probe.transaction_activated_count, probe.transaction_count
        )
        join_valid_fraction = _fraction(
            probe.join_valid_count, probe.mechanism_count
        )
        context_valid_fraction = _fraction(
            probe.context_valid_count, probe.context_count
        )
        expected_holds = probe.fault_taxonomy_counts[
            "EXPECTED_FAIL_CLOSED_CONTEXT_HOLD"
        ]
        backend_faults = probe.fault_taxonomy_counts[
            "BACKEND_TRANSACTION_FAULT"
        ]
        unknown_faults = probe.fault_taxonomy_counts[
            "UNKNOWN_TRANSACTION_FAULT"
        ]
        classified_faults = expected_holds + backend_faults + unknown_faults
        gates = {
            "consecutive_stable_readiness": probe.window.ready,
            "transaction_message_count": (
                probe.transaction_count >= args.minimum_message_count
            ),
            "mechanism_message_count": (
                probe.mechanism_count >= args.minimum_message_count
            ),
            "transaction_activated_fraction": (
                transaction_activated_fraction >= args.minimum_valid_fraction
            ),
            "transaction_valid_fraction": (
                transaction_valid_fraction >= args.minimum_valid_fraction
            ),
            "join_valid_fraction": (
                join_valid_fraction >= args.minimum_valid_fraction
            ),
            "expected_context_hold_count": (
                expected_holds <= args.maximum_expected_context_hold_count
            ),
            "backend_transaction_fault_count": backend_faults == 0,
            "unknown_transaction_fault_count": unknown_faults == 0,
            "fault_taxonomy_complete": (
                classified_faults == sum(probe.fault_reason_counts.values())
            ),
            "join_reason_domain": set(probe.join_reason_counts).issubset({
                "EXACT_SEQUENCE_JOIN", "BOUNDED_SEQUENCE_TIME_JOIN",
            }),
            "execution_backend_domain": set(
                probe.transaction_backend_counts
            ) == {"simulation_teb_dynamic_reconfigure"},
            "training_unused": probe.transaction_training_count == 0,
        }
        report = {
            "schema_version": "2.0", "stage": STAGE,
            "status": "pass" if all(gates.values()) else "fail",
            "simulation_only": True, "runtime_ready": False,
            "training_used": False, "real_vehicle_used": False,
            "profile_id": args.profile_id, "repeat": args.repeat,
            "seed": args.seed,
            "warmup_elapsed_s": warmup_elapsed_s,
            "required_consecutive_stable_count": args.required_consecutive_stable_count,
            "maximum_consecutive_stable_count": probe.window.maximum_count,
            "warmup_mechanism_message_count": probe.warmup_mechanism_count,
            "warmup_instability_counts": dict(probe.warmup_instability_counts),
            "measurement_duration_s": args.measurement_duration_s,
            "transaction_message_count": probe.transaction_count,
            "transaction_valid_count": probe.transaction_valid_count,
            "transaction_activated_count": probe.transaction_activated_count,
            "transaction_valid_fraction": transaction_valid_fraction,
            "transaction_activated_fraction": transaction_activated_fraction,
            "transaction_backend_counts": dict(probe.transaction_backend_counts),
            "fault_reason_counts": dict(probe.fault_reason_counts),
            "fault_taxonomy_counts": dict(probe.fault_taxonomy_counts),
            "fault_samples": list(probe.fault_samples),
            "mechanism_message_count": probe.mechanism_count,
            "join_valid_count": probe.join_valid_count,
            "join_valid_fraction": join_valid_fraction,
            "join_reason_counts": dict(probe.join_reason_counts),
            "unique_context_sequence_count": len(probe.context_sequences),
            "maximum_join_sequence_delta": probe.maximum_sequence_delta,
            "maximum_join_timestamp_delta_s": probe.maximum_timestamp_delta_s,
            "context_message_count": probe.context_count,
            "context_valid_count": probe.context_valid_count,
            "context_valid_fraction": context_valid_fraction,
            "context_reason_counts": dict(probe.context_reason_counts),
            "hard_gates": gates,
            "all_hard_gates_pass": all(gates.values()),
        }
    _write_report(args.output, report)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0 if report["all_hard_gates_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
