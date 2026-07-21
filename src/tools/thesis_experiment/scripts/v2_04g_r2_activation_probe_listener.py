#!/usr/bin/env python3
"""Measure one deterministic V2-04G-R2 activation-readiness window."""

import argparse
import importlib.util
from pathlib import Path

import rospy
import yaml


R1_LISTENER = Path(__file__).with_name("v2_04g_r1_activation_probe_listener.py")
_SPEC = importlib.util.spec_from_file_location("v2_04g_r1_frozen_probe", R1_LISTENER)
_R1 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_R1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--repeat", type=int, required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--warmup-timeout-s", type=float, default=8.0)
    parser.add_argument("--measurement-duration-s", type=float, default=6.0)
    parser.add_argument("--minimum-message-count", type=int, default=20)
    parser.add_argument("--minimum-valid-fraction", type=float, default=0.95)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("v2_04g_r2_activation_probe_listener")
    probe = _R1.Probe()
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
    if not ready:
        raise RuntimeError("R2 activation probe did not observe joined activation")
    with probe.lock:
        probe.reset()
        probe.ready = True
    start = rospy.Time.now().to_sec()
    while (
        not rospy.is_shutdown()
        and rospy.Time.now().to_sec() - start < args.measurement_duration_s
    ):
        rate.sleep()
    with probe.lock:
        transaction_fraction = (
            float(probe.transaction_activated_count) / probe.transaction_count
            if probe.transaction_count else 0.0
        )
        transaction_valid_fraction = (
            float(probe.transaction_valid_count) / probe.transaction_count
            if probe.transaction_count else 0.0
        )
        join_fraction = (
            float(probe.join_valid_count) / probe.mechanism_count
            if probe.mechanism_count else 0.0
        )
        gates = {
            "transaction_message_count": probe.transaction_count >= args.minimum_message_count,
            "mechanism_message_count": probe.mechanism_count >= args.minimum_message_count,
            "transaction_activated_fraction": transaction_fraction >= args.minimum_valid_fraction,
            "transaction_valid_fraction": transaction_valid_fraction >= args.minimum_valid_fraction,
            "join_valid_fraction": join_fraction >= args.minimum_valid_fraction,
            "transaction_fault_count": probe.transaction_fault_count == 0,
            "join_reason_domain": set(probe.join_reason_counts).issubset({
                "EXACT_SEQUENCE_JOIN", "BOUNDED_SEQUENCE_TIME_JOIN",
            }),
        }
        report = {
            "schema_version": "2.0", "stage": "V2-04G-R2",
            "status": "pass" if all(gates.values()) else "fail",
            "simulation_only": True, "runtime_ready": False,
            "training_used": False, "real_vehicle_used": False,
            "profile_id": args.profile_id, "repeat": args.repeat,
            "seed": args.seed, "measurement_duration_s": args.measurement_duration_s,
            "transaction_message_count": probe.transaction_count,
            "transaction_valid_count": probe.transaction_valid_count,
            "transaction_activated_count": probe.transaction_activated_count,
            "transaction_fault_count": probe.transaction_fault_count,
            "transaction_valid_fraction": transaction_valid_fraction,
            "transaction_activated_fraction": transaction_fraction,
            "mechanism_message_count": probe.mechanism_count,
            "join_valid_count": probe.join_valid_count,
            "join_valid_fraction": join_fraction,
            "join_reason_counts": dict(probe.join_reason_counts),
            "unique_context_sequence_count": len(probe.context_sequences),
            "maximum_join_sequence_delta": probe.maximum_sequence_delta,
            "maximum_join_timestamp_delta_s": probe.maximum_timestamp_delta_s,
            "hard_gates": gates, "all_hard_gates_pass": all(gates.values()),
        }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    temporary.replace(args.output)
    print(yaml.safe_dump(report, sort_keys=False))
    return 0 if report["all_hard_gates_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
