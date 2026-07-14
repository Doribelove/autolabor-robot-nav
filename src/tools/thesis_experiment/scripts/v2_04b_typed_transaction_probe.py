#!/usr/bin/env python3
"""Exercise real typed TEB transactions in Gazebo and restore startup state."""

import argparse
from pathlib import Path

import rospy
import yaml
from rosgraph_msgs.msg import Clock

from teb_mode_manager.action_pipeline import AnchorBank, RuleAnchorTransactionLoop
from teb_mode_manager.typed_teb_transaction import (
    EXPECTED_TEB_NAMESPACE,
    RosTypedDynamicReconfigureAdapter,
    SimulationWriteContext,
    TypedTebTransactionBackend,
    require_simulation_write,
)


SCHEDULE = (
    ("CRUISE", False),
    ("STATIC_DENSE", False),
    ("CORRIDOR", False),
    ("MANEUVER", False),
    ("MANEUVER", True),
    ("BALANCED", False),
)


def _same(bank, left, right, tolerance=1.0e-8):
    for name, definition in bank.definitions.items():
        if definition.parameter_type == "double":
            if abs(float(left[name]) - float(right[name])) > tolerance:
                return False
        elif type(left[name]) is not type(right[name]) or left[name] != right[name]:
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--anchor-bank", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--timeout-s", type=float, default=2.0)
    parser.add_argument("--maximum-ticks-per-segment", type=int, default=100)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("v2_04b_typed_transaction_probe")
    output = Path(args.output).resolve()
    allowed = Path("/home/robot/robot_ws_base_rl/artifacts/v2").resolve()
    try:
        output.relative_to(allowed)
    except ValueError:
        raise RuntimeError("probe output must remain under artifacts/v2")
    clock = rospy.wait_for_message("/clock", Clock, timeout=10.0)
    bank = AnchorBank.from_file(args.anchor_bank)
    write_context = SimulationWriteContext(
        explicit_simulation_write=True,
        use_sim_time=rospy.get_param("/use_sim_time", False) is True,
        simulation_marker=rospy.get_param("/m2_gazebo/simulation_only", False) is True,
        gazebo_clock_active=clock.clock.to_sec() > 0.0,
        teb_namespace=EXPECTED_TEB_NAMESPACE,
    )
    require_simulation_write(write_context)
    adapter = RosTypedDynamicReconfigureAdapter(EXPECTED_TEB_NAMESPACE, args.timeout_s)
    backend = TypedTebTransactionBackend(
        bank,
        adapter,
        write_context,
        timeout_s=args.timeout_s,
        time_source=lambda: rospy.Time.now().to_sec(),
    )
    backend.initialize()
    startup = dict(backend.startup.values)
    loop = RuleAnchorTransactionLoop(bank, backend=backend)
    rate = rospy.Rate(loop.frequency_hz)
    traces = []
    segments = []
    mode_seq = 0
    failure = ""
    try:
        for geometry_mode, reverse in SCHEDULE:
            mode_seq += 1
            target = loop.decoder.decode(
                geometry_mode, "NONE", residuals={}, maneuver_reverse=reverse
            ).feasible.values
            reached = False
            for local_tick in range(args.maximum_ticks_per_segment):
                trace = loop.update(
                    rospy.Time.now().to_sec(), len(traces) + 1, mode_seq,
                    geometry_mode, "NONE",
                    "ENTERING" if local_tick < 3 else "STABLE", True,
                    maneuver_reverse=reverse,
                )
                traces.append(trace)
                if not trace.valid or not trace.activated:
                    raise RuntimeError("typed transaction failed: {}".format(trace.fault_reason))
                if _same(bank, trace.typed_stage_mapping("executed"), target):
                    reached = True
                    segments.append({
                        "geometry_mode": geometry_mode,
                        "maneuver_reverse": reverse,
                        "ticks": local_tick + 1,
                        "slow_profile_committed": trace.slow_profile_committed,
                        "profile_id": trace.profile_id,
                    })
                    break
                rate.sleep()
            if not reached:
                raise RuntimeError("segment {} failed to converge".format(geometry_mode))
    except Exception as exc:
        failure = str(exc)
    finally:
        try:
            backend.close()
        except Exception as exc:
            failure = failure or "startup restore failed: {}".format(exc)
    restored = backend.current is not None and _same(bank, backend.current.values, startup)
    timestamps_valid = all(
        trace.t_request_s <= trace.t_ack_s <= trace.t_readback_s <= trace.t_active_s
        for trace in traces
    )
    types_valid = all(
        trace.parameter_types == bank.parameter_types
        and type(trace.typed_stage_mapping("executed")["include_dynamic_obstacles"]) is bool
        and type(trace.typed_stage_mapping("executed")["max_number_classes"]) is int
        for trace in traces
    )
    passed = not failure and len(segments) == len(SCHEDULE) and restored and timestamps_valid and types_valid
    report = {
        "schema_version": "2.0",
        "stage": "V2-04B",
        "status": "passed" if passed else "failed",
        "formal_result": False,
        "simulation_only": True,
        "runtime_ready": False,
        "training_started": False,
        "real_vehicle_used": False,
        "execution_backend": backend.backend_id,
        "teb_namespace": EXPECTED_TEB_NAMESPACE,
        "parameter_count": len(bank.parameter_names),
        "parameter_type_counts": {
            kind: bank.parameter_types.count(kind) for kind in ("double", "int", "bool")
        },
        "single_call_complete_profile": all(
            len(record.get("request", {})) == len(bank.parameter_names)
            for record in backend.audit_records
        ),
        "transaction_count": len(traces),
        "segments": segments,
        "timestamps_ordered": timestamps_valid,
        "typed_trace_reconstructible": types_valid,
        "startup_snapshot_restored": restored,
        "audit_operation_count": len(backend.audit_records),
        "fault_reason": failure,
        "claims": {
            "anchor_values_calibrated": False,
            "navigation_performance_improved": False,
            "sac_training_used": False,
            "real_vehicle_validated": False,
        },
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(report, sort_keys=False))
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
