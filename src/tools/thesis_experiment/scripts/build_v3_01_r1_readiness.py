#!/usr/bin/env python3
"""Materialize V3-01-R1 single-factor repair readiness."""

import argparse
import importlib.util
from pathlib import Path
import shutil


W = Path("/home/robot/robot_ws_base_rl")
M = W / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_r1.py"


def module():
    spec = importlib.util.spec_from_file_location("v3_01_r1", M)
    value = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(value)
    return value


def write_once(path, payload):
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing overwrite: {}".format(path))
    else:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)


def outputs(v):
    contract = v.load_yaml_relative(W, v.CONTRACT_RELATIVE)
    prereg = v.load_yaml_relative(W, v.PREREGISTRATION_RELATIVE)
    v.validate_contract(contract, prereg)
    schedule = v.exact_schedule(contract)
    gzserver = Path(shutil.which("gzserver") or "")
    v.require(gzserver.is_file(), "gzserver unavailable")
    sources = [v.receipt(W, item) for item in v.SOURCE_RESOURCES]
    binaries = [v.receipt(W, item) for item in v.BINARY_RESOURCES]
    executables = [v.absolute_receipt(gzserver)]
    bindings = {
        "schema_version": "1.0",
        "stage": contract["stage"],
        "status": "hermetic_runtime_bindings_frozen",
        "process_model": "env_i_non_login_no_ros_master",
        "gazebo_executable": executables[0],
        "gazebo_arguments": ["--verbose", "WORLD"],
        "environment": {
            "PATH": "/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin",
            "LD_LIBRARY_PATH": (
                "/home/robot/robot_ws_base_rl/devel/lib:/opt/ros/noetic/lib:"
                "/usr/lib/x86_64-linux-gnu"
            ),
            "GAZEBO_PLUGIN_PATH": "/home/robot/robot_ws_base_rl/devel/lib",
            "GAZEBO_MODEL_PATH": "",
            "ROS_DISTRO": "noetic",
        },
        "ros_master_required": False,
        "move_base_required": False,
        "hardware_device_access_required": False,
    }
    closure = {
        "schema_version": "1.0",
        "stage": contract["stage"],
        "status": "dependency_closure_complete",
        "unresolved_count": 0,
        "source_resources": sources,
        "binary_resources": binaries,
        "executable_resources": executables,
    }
    schedule_bytes = v.yaml_bytes(schedule)
    bindings_bytes = v.yaml_bytes(bindings)
    closure_bytes = v.yaml_bytes(closure)
    release = {
        "schema_version": "1.0",
        "stage": contract["stage"],
        "release_id": "{}_crossing_execution_release_1".format(
            contract["stage"].lower().replace("-", "_")
        ),
        "status": "release_candidate_requires_review_and_authorization",
        "single_factor_repair": contract["single_factor_repair"]["factor_id"],
        "execution_ready": False,
        "execution_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
        "schedule": {
            "path": v.SCHEDULE_RELATIVE.as_posix(),
            "sha256": v.sha256_bytes(schedule_bytes),
            "row_count": 36,
        },
        "runtime_bindings": {
            "path": v.BINDINGS_RELATIVE.as_posix(),
            "sha256": v.sha256_bytes(bindings_bytes),
        },
        "dependency_closure": {
            "path": v.CLOSURE_RELATIVE.as_posix(),
            "sha256": v.sha256_bytes(closure_bytes),
            "unresolved_count": 0,
        },
        "budget": {
            "authorized_after_separate_authorization": 36,
            "consumed": 0,
            "retry": 0,
        },
    }
    return {
        v.SCHEDULE_RELATIVE: schedule_bytes,
        v.BINDINGS_RELATIVE: bindings_bytes,
        v.CLOSURE_RELATIVE: closure_bytes,
        v.RELEASE_RELATIVE: v.yaml_bytes(release),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    v = module()
    built = outputs(v)
    for relative, payload in built.items():
        path = W / relative
        if args.check:
            v.require(path.is_file() and path.read_bytes() == payload, "output differs")
        else:
            write_once(path, payload)
    print(
        "release_sha256={}".format(
            v.sha256_bytes(built[v.RELEASE_RELATIVE])
        )
    )


if __name__ == "__main__":
    main()
