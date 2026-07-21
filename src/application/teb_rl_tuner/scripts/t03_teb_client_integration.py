#!/usr/bin/env python3
"""Run the T03 parameter transaction against the T02 Gazebo move_base only."""

import os
import sys
import time
import unittest
from pathlib import Path

import rospy
import yaml

from teb_rl_tuner import (
    RosDynamicReconfigureBackend,
    SimulationWriteContext,
    TebParameterClient,
    specs_as_dict,
)


def _write_report(path, report):
    class NoAliasSafeDumper(yaml.SafeDumper):
        def ignore_aliases(self, data):
            return True

    destination = Path(path)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    temporary.write_text(
        yaml.dump(report, Dumper=NoAliasSafeDumper, sort_keys=False), encoding="utf-8"
    )
    os.replace(str(temporary), str(destination))


def main():
    rospy.init_node("t03_teb_client_integration", anonymous=False)
    namespace = rospy.get_param("~teb_namespace", "/move_base/TebLocalPlannerROS")
    timeout_s = float(rospy.get_param("~timeout_s", 5.0))
    report_path = rospy.get_param(
        "~report_path",
        "/home/robot/robot_ws_base_rl/artifacts/t03/teb_parameter_client_acceptance.yaml",
    )
    probe = rospy.get_param("~probe_theta")
    context = SimulationWriteContext(
        explicit_simulation=rospy.get_param("~simulation", False),
        use_sim_time=rospy.get_param("/use_sim_time", False),
        simulation_marker=rospy.get_param("/m2_gazebo/simulation_only", False),
        teb_namespace=namespace,
    )
    report = {
        "schema_version": 1,
        "task": "T03",
        "suite": "teb_parameter_client_acceptance",
        "simulation_only": True,
        "real_driver_started": False,
        "serial_or_can_accessed": False,
        "real_vehicle_motion": False,
        "namespace": namespace,
        "safety_gate": {
            "explicit_simulation": context.explicit_simulation,
            "use_sim_time": context.use_sim_time,
            "simulation_marker": context.simulation_marker,
        },
        "probe_status": "simulation_candidate_not_real_calibration",
        "passed": False,
    }
    client = None
    try:
        backend = RosDynamicReconfigureBackend(namespace, timeout_s)
        client = TebParameterClient(backend, context, timeout_s=timeout_s)
        rospy.on_shutdown(client.close)
        specs = client.initialize()
        report["parameter_specs"] = specs_as_dict(specs)
        report["startup_snapshot"] = client.snapshot

        transaction = client.apply(probe)
        report["transaction"] = transaction
        restoration = client.restore()
        report["restoration"] = restoration
        report["final_values"] = restoration["readback"]
        report["snapshot_restored"] = restoration["readback"] == report["startup_snapshot"]
        report["atomic_request_parameter_count"] = len(transaction["request"])
        report["passed"] = bool(
            transaction["passed"]
            and restoration["passed"]
            and report["snapshot_restored"]
            and report["atomic_request_parameter_count"] == 9
        )
    except Exception as exc:
        report["failure_type"] = type(exc).__name__
        report["failure_message"] = str(exc)
        if client is not None:
            report["audit_records"] = client.audit_records
        _write_report(report_path, report)
        rospy.logerr("T03 acceptance failed: %s", exc)
        return 1
    finally:
        if client is not None:
            try:
                client.close()
            except Exception as exc:
                report["close_restore_failure"] = str(exc)
                report["passed"] = False

    report["audit_records"] = client.audit_records
    _write_report(report_path, report)
    if not report["passed"]:
        rospy.logerr("T03 acceptance did not meet all checks")
        return 1
    rospy.loginfo("T03 parameter client acceptance passed")
    return 0


if __name__ == "__main__":
    if "--rostest" in sys.argv:
        import rostest

        class T03GazeboIntegrationTest(unittest.TestCase):
            def test_parameter_transaction_and_restore(self):
                self.assertEqual(main(), 0)

        rostest.rosrun(
            "teb_rl_tuner",
            "t03_teb_client_integration",
            T03GazeboIntegrationTest,
        )
    else:
        sys.exit(main())
