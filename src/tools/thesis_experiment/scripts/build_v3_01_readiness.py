#!/usr/bin/env python3
"""Materialize the V3-01 exact schedule, bindings, closure and release."""

import argparse
import importlib.util
from pathlib import Path
import shutil
import sys


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
MODULE_PATH = (
    WORKSPACE
    / "src/tools/thesis_experiment/src/thesis_experiment/v3_01_qualification.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("v3_01_qualification", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_once(path: Path, payload: bytes) -> None:
    if path.exists():
        if path.read_bytes() != payload:
            raise RuntimeError("refusing to overwrite different file: {}".format(path))
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(payload)


def build(module):
    contract = module.load_yaml_relative(WORKSPACE, module.CONTRACT_RELATIVE)
    preregistration = module.load_yaml_relative(
        WORKSPACE, module.PREREGISTRATION_RELATIVE
    )
    module.validate_contract(contract, preregistration)
    schedule = module.exact_schedule(contract)

    gzserver = Path(shutil.which("gzserver") or "")
    if not gzserver.is_file():
        raise module.V301Error("gzserver executable is unavailable")
    source_receipts = [
        module.receipt(WORKSPACE, path) for path in module.SOURCE_RESOURCES
    ]
    binary_receipts = [
        module.receipt(WORKSPACE, path) for path in module.BINARY_RESOURCES
    ]
    executable_receipts = [module.absolute_receipt(gzserver)]
    bindings = {
        "schema_version": "1.0",
        "stage": "V3-01",
        "status": "hermetic_runtime_bindings_frozen",
        "process_model": "env_i_non_login_no_ros_master",
        "gazebo_executable": executable_receipts[0],
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
        "forbidden_environment_key_fragments": [
            "TOKEN",
            "SECRET",
            "PASSWORD",
            "CREDENTIAL",
            "SSH_AUTH_SOCK",
        ],
        "ros_master_required": False,
        "move_base_required": False,
        "hardware_device_access_required": False,
    }
    closure = {
        "schema_version": "1.0",
        "stage": "V3-01",
        "status": "dependency_closure_complete",
        "unresolved_count": 0,
        "source_resource_count": len(source_receipts),
        "binary_resource_count": len(binary_receipts),
        "executable_resource_count": len(executable_receipts),
        "source_resources": source_receipts,
        "binary_resources": binary_receipts,
        "executable_resources": executable_receipts,
    }
    schedule_payload = module.yaml_bytes(schedule)
    bindings_payload = module.yaml_bytes(bindings)
    closure_payload = module.yaml_bytes(closure)
    release = {
        "schema_version": "1.0",
        "architecture_generation": "v3",
        "stage": "V3-01",
        "release_id": "v3_scope_v3_01_crossing_execution_release_1",
        "status": "release_candidate_requires_independent_review_and_authorization",
        "execution_ready": False,
        "execution_authorized": False,
        "training_authorized": False,
        "real_vehicle_authorized": False,
        "schedule": {
            "path": module.SCHEDULE_RELATIVE.as_posix(),
            "sha256": module.sha256_bytes(schedule_payload),
            "row_count": 36,
        },
        "runtime_bindings": {
            "path": module.BINDINGS_RELATIVE.as_posix(),
            "sha256": module.sha256_bytes(bindings_payload),
        },
        "dependency_closure": {
            "path": module.CLOSURE_RELATIVE.as_posix(),
            "sha256": module.sha256_bytes(closure_payload),
            "unresolved_count": 0,
        },
        "budget": {
            "authorized_after_separate_authorization": 36,
            "consumed": 0,
            "forfeited": 0,
            "retry": 0,
        },
        "execution_command_contract": {
            "runner": (
                "src/tools/thesis_experiment/scripts/"
                "run_v3_01_qualification.py"
            ),
            "required_flags": ["--execute", "--release-sha", "--authorization-sha"],
            "resume_allowed": False,
        },
    }
    return {
        module.SCHEDULE_RELATIVE: schedule_payload,
        module.BINDINGS_RELATIVE: bindings_payload,
        module.CLOSURE_RELATIVE: closure_payload,
        module.RELEASE_RELATIVE: module.yaml_bytes(release),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    module = load_module()
    outputs = build(module)
    for relative, payload in outputs.items():
        path = WORKSPACE / relative
        if args.check:
            if not path.is_file() or path.read_bytes() != payload:
                raise module.V301Error("readiness output differs: {}".format(relative))
        else:
            write_once(path, payload)
    release_sha = module.sha256_bytes(outputs[module.RELEASE_RELATIVE])
    print(
        module.yaml_bytes(
            {
                "status": "v3_01_release_candidate_materialized",
                "release_sha256": release_sha,
                "row_count": 36,
                "execution_authorized": False,
            }
        ).decode("utf-8").strip()
    )


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("V3-01 readiness failed closed: {}".format(exc), file=sys.stderr)
        raise
