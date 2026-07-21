#!/usr/bin/env python3
"""Capture or control the live R6-I1 transaction boundary."""

import argparse
import json
import os
from pathlib import Path
import tempfile

import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml

from teb_mode_manager.action_pipeline import AnchorBank
from teb_mode_manager.r6_execution_integration import (
    canonical_attempt_identity,
    canonical_profile_bytes,
    sha256_bytes,
)
from teb_mode_manager.typed_teb_transaction import (
    RosTypedDynamicReconfigureAdapter,
    _extract_values,
)


STAGE = "V2-04G-R6-I1"
TEB_NAMESPACE = "/move_base/TebLocalPlannerROS"


def _identity(args):
    return canonical_attempt_identity({
        "stage": args.stage,
        "profile_id": args.profile_id,
        "scene_id": args.scene_id,
        "seed": args.seed,
        "attempt": args.attempt,
    })


def _atomic_yaml(path, value):
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml.safe_dump(value, sort_keys=False).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=target.name + ".tmp.", dir=str(target.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(str(temporary), str(target))
    except BaseException:
        try:
            temporary.unlink()
        except OSError:
            pass
        raise


def _initial_readback(args, identity):
    bank = AnchorBank.from_file(args.anchor_bank)
    adapter = RosTypedDynamicReconfigureAdapter(
        TEB_NAMESPACE, args.timeout_s
    )
    try:
        raw = adapter.get_configuration(args.timeout_s)
        if raw is None:
            raise RuntimeError("initial independent readback timed out")
        values = _extract_values(raw, bank, "R6-I1 initial readback")
        payload = canonical_profile_bytes(bank, values)
    finally:
        adapter.close()
    return {
        "schema_version": "2.0",
        "record_type": "r6_i1_initial_independent_readback",
        **identity,
        "identity": identity,
        "teb_namespace": TEB_NAMESPACE,
        "startup_profile_sha256": sha256_bytes(payload),
        "startup_profile_canonical_json": payload.decode("utf-8"),
        "readback_complete": True,
    }


def _wait_startup(args, identity):
    message = rospy.wait_for_message(
        args.startup_topic, String, timeout=args.timeout_s
    )
    document = json.loads(message.data)
    if document.get("identity") != identity:
        raise RuntimeError("transaction startup identity mismatch")
    payload = document.get("startup_profile_canonical_json", "").encode(
        "utf-8"
    )
    if (
        not payload
        or sha256_bytes(payload) != document.get("startup_profile_sha256")
    ):
        raise RuntimeError("transaction startup profile hash mismatch")
    return document


def _trigger(args, identity, service_name, record_type):
    rospy.wait_for_service(service_name, timeout=args.timeout_s)
    response = rospy.ServiceProxy(service_name, Trigger)()
    if record_type == "r6_i1_arm_receipt":
        document = json.loads(response.message)
        if document.get("identity") != identity:
            raise RuntimeError("arm receipt identity mismatch")
        if not (
            document.get("execution_armed") is True
            and isinstance(document.get("startup_profile_sha256"), str)
            and len(document["startup_profile_sha256"]) == 64
        ):
            raise RuntimeError("arm receipt startup binding is incomplete")
        document.update({
            **identity,
            "service": service_name,
            "service_response_success": bool(response.success),
        })
    else:
        document = json.loads(response.message)
        if document.get("identity") != identity:
            raise RuntimeError("teardown receipt identity mismatch")
        document["service_response_success"] = bool(response.success)
    if not response.success:
        _atomic_yaml(args.output, document)
        raise RuntimeError("{} failed".format(record_type))
    return document


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("initial-readback", "transaction-startup", "arm", "restore"),
        required=True,
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--anchor-bank")
    parser.add_argument("--stage", choices=(STAGE,), required=True)
    parser.add_argument("--profile-id", required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--attempt", type=int, choices=(1,), required=True)
    parser.add_argument("--timeout-s", type=float, default=15.0)
    parser.add_argument(
        "--startup-topic",
        default="/teb_mode_manager/r6/startup_profile",
    )
    parser.add_argument(
        "--arm-service",
        default="/teb_mode_manager/r6/arm_execution",
    )
    parser.add_argument(
        "--restore-service",
        default="/teb_mode_manager/r6/restore_startup",
    )
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node(
        "v2_04g_r6_i1_runtime_control_{}".format(
            args.mode.replace("-", "_")
        ),
        anonymous=True,
    )
    identity = _identity(args)
    if args.mode == "initial-readback":
        if not args.anchor_bank:
            parser.error("--anchor-bank is required for initial-readback")
        document = _initial_readback(args, identity)
    elif args.mode == "transaction-startup":
        document = _wait_startup(args, identity)
    elif args.mode == "arm":
        document = _trigger(
            args, identity, args.arm_service, "r6_i1_arm_receipt"
        )
    else:
        document = _trigger(
            args,
            identity,
            args.restore_service,
            "r6_two_phase_teardown_receipt",
        )
    _atomic_yaml(args.output, document)
    print(yaml.safe_dump(document, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
