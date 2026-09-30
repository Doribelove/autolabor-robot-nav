#!/usr/bin/env python3
"""Build the pure-offline V3-05-R1 master-port repair evidence."""

import argparse
import hashlib
import importlib.util
from pathlib import Path
import socket

import yaml


W = Path("/home/robot/robot_ws_base_rl")
CONTRACT = Path(
    "config/thesis_experiments/v3/v3_05_r1_master_port_repair_contract.yaml"
)
PREREGISTRATION = Path(
    "experiments/manifests/v3/"
    "v3_05_r1_master_port_repair_preregistration.yaml"
)
RUNNER = (
    W
    / "src/tools/thesis_experiment/scripts/"
    "run_v3_03_c1_closed_loop.py"
)
ROOT = Path("artifacts/v3/formal/v3_05_r1")
REPORT = ROOT / "v3_05_r1_port_repair_report.yaml"
CLOSURE = ROOT / "v3_05_r1_dependency_closure.yaml"
EVIDENCE_FREEZE = ROOT / "v3_05_r1_evidence_freeze.sha256"
INDEPENDENT_REVIEW = ROOT / "v3_05_r1_independent_review.yaml"

SOURCES = (
    CONTRACT,
    PREREGISTRATION,
    Path("artifacts/v3/formal/v3_05_a0/v3_05_a0_terminal_freeze.sha256"),
    Path(
        "artifacts/v3/formal/v3_05_a0/"
        "v3_05_a0_terminal_failure_record.yaml"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "run_v3_03_c1_closed_loop.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "build_v3_05_r1_port_repair.py"
    ),
    Path(
        "src/tools/thesis_experiment/scripts/"
        "review_v3_05_r1_port_repair.py"
    ),
    Path("src/tools/thesis_experiment/tests/test_v3_05_r1.py"),
)


def load(path):
    return yaml.safe_load((W / path).read_bytes())


def sha(payload):
    return hashlib.sha256(payload).hexdigest()


def yaml_bytes(value):
    return yaml.safe_dump(
        value, sort_keys=False, allow_unicode=True, width=1000000
    ).encode("utf-8")


def write_once(path, payload):
    absolute = W / path
    if absolute.exists():
        if absolute.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        absolute.parent.mkdir(parents=True, exist_ok=True)
        absolute.write_bytes(payload)


def runner():
    spec = importlib.util.spec_from_file_location("v3_05_r1_runner", RUNNER)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def build():
    contract = load(CONTRACT)
    prereg = load(PREREGISTRATION)
    factor = contract["single_factor_repair"]
    if (
        contract["stage"] != "V3-05-R1"
        or prereg["stage"] != "V3-05-R1"
        or factor["factor_count"] != 1
        or factor["factor_id"]
        != "gazebo_master_port_allocation_below_ephemeral_range"
    ):
        raise RuntimeError("contract/preregistration differs")
    if sha((W / contract["parent"]["terminal_freeze_path"]).read_bytes()) != (
        contract["parent"]["terminal_freeze_sha256"]
    ):
        raise RuntimeError("parent terminal freeze differs")
    if sha((W / contract["parent"]["failure_record_path"]).read_bytes()) != (
        contract["parent"]["failure_record_sha256"]
    ):
        raise RuntimeError("parent failure record differs")
    allocation = {
        "base_port": factor["base_port"],
        "sequence_stride": factor["sequence_stride"],
        "maximum_decision_index": factor["maximum_decision_index"],
        "minimum_allowed_port": factor["minimum_allowed_port"],
        "maximum_allowed_port": factor["maximum_allowed_port"],
    }
    run = runner()
    ports = [
        run.gazebo_master_port(
            {"runtime_port_allocation": allocation}, sequence, decision
        )
        for sequence in range(1, int(factor["maximum_sequence"]) + 1)
        for decision in range(
            int(factor["maximum_decision_index"]) + 1
        )
    ]
    ephemeral = factor["audited_linux_ephemeral_range"]
    bindable = 0
    for port in ports:
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            probe.bind(("127.0.0.1", port))
            bindable += 1
        finally:
            probe.close()
    metrics = {
        "allocated_port_count": len(ports),
        "minimum_allocated_port": min(ports),
        "maximum_allocated_port": max(ports),
        "unique_port_rate": len(set(ports)) / len(ports),
        "below_ephemeral_rate": sum(
            port < int(ephemeral[0]) for port in ports
        )
        / len(ports),
        "static_bind_probe_rate": bindable / len(ports),
        "historical_failure_port": 15000 + 206 * 100,
        "historical_failure_port_in_ephemeral_range": (
            int(ephemeral[0])
            <= 15000 + 206 * 100
            <= int(ephemeral[1])
        ),
    }
    gates = contract["offline_gates"]
    results = {
        "port_count_pass": metrics["allocated_port_count"]
        == int(gates["allocated_port_count"]),
        "unique_ports_pass": metrics["unique_port_rate"]
        >= float(gates["unique_port_rate_min"]),
        "below_ephemeral_pass": metrics["below_ephemeral_rate"]
        >= float(gates["below_ephemeral_rate_min"]),
        "bind_probe_pass": metrics["static_bind_probe_rate"]
        >= float(gates["static_bind_probe_rate_min"]),
        "historical_failure_identified_pass": (
            metrics["historical_failure_port"]
            == int(gates["historical_failure_port_identified"])
            and metrics["historical_failure_port_in_ephemeral_range"]
        ),
    }
    report = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-05-R1",
        "status": (
            "v3_05_r1_master_port_repair_offline_pass"
            if all(results.values())
            else "v3_05_r1_master_port_repair_offline_fail"
        ),
        "formal_result": False,
        "metrics": metrics,
        "gates": results,
        "overall_pass": all(results.values()),
        "simulation_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
    }
    resources = []
    for path in SOURCES:
        payload = (W / path).read_bytes()
        resources.append(
            {
                "path": path.as_posix(),
                "sha256": sha(payload),
                "size_bytes": len(payload),
            }
        )
    closure = {
        "schema_version": "1.0",
        "stage": "V3-05-R1",
        "status": "dependency_closure_complete",
        "unresolved_count": 0,
        "resources": resources,
    }
    outputs = {REPORT: yaml_bytes(report), CLOSURE: yaml_bytes(closure)}
    outputs[EVIDENCE_FREEZE] = (
        "\n".join(
            "{}  {}".format(sha(payload), path.as_posix())
            for path, payload in outputs.items()
        )
        + "\n"
    ).encode("utf-8")
    return outputs


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    outputs = build()
    for path, payload in outputs.items():
        if args.check:
            if not (W / path).is_file() or (W / path).read_bytes() != payload:
                raise RuntimeError("persisted output differs: {}".format(path))
        else:
            write_once(path, payload)
    report = yaml.safe_load(outputs[REPORT])
    print(
        "status={} report_sha256={}".format(
            report["status"], sha(outputs[REPORT])
        )
    )


if __name__ == "__main__":
    main()
