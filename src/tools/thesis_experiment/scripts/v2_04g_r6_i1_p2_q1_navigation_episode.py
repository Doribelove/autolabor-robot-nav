#!/usr/bin/env python3
"""Record one future P2-Q1 calibration navigation trace.

The adapter reuses the frozen P1 evaluator only for navigation mechanics and
adds a Q1 raw sample document.  It is never invoked by the offline Q1 stage;
only a later exact release may run it in Gazebo.
"""

import argparse
import hashlib
import importlib.util
import os
from pathlib import Path
import tempfile

import rospy
from std_msgs.msg import Bool
import yaml

from teb_mode_manager.msg import ContextState


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P2-Q1"
SOURCE = Path(__file__).with_name("v2_04g_r6_i1_p1_e1_episode.py")
_SPEC = importlib.util.spec_from_file_location("p2_q1_frozen_p1_episode", SOURCE)
_P1 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_P1)

CIRCLE_METHODS = {
    "circle_contact_v120",
    "circle_contact_v125",
    "circle_contact_v130",
    "circle_contact_v135",
    "circle_contact_v140",
}
FIXED_METHODS = {
    "fixed_teb_v120__TEB-Default_alias",
    "fixed_teb_v125",
    "fixed_teb_v130",
    "fixed_teb_v135",
    "fixed_teb_v140",
    "TEB-Tuned-V1",
}
METHODS = tuple(sorted(CIRCLE_METHODS | FIXED_METHODS))


def _atomic_yaml(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".{}-".format(path.name), dir=str(path.parent)
    )
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    finally:
        if os.path.exists(temporary_name):
            os.unlink(temporary_name)


class Q1NavigationEpisode(_P1.P1Episode):
    """Add raw per-sample projection, emergency, and transaction evidence."""

    def __init__(self, *args, actual_method_id, sequence, **kwargs):
        self.q1_actual_method_id = actual_method_id
        self.q1_sequence = sequence
        self.q1_latest_projection = False
        self.q1_latest_transaction_valid = actual_method_id in FIXED_METHODS
        self.q1_emergency = False
        self.q1_samples = []
        super().__init__(*args, **kwargs)
        rospy.Subscriber(
            "/m2_driver/emergency_stop",
            Bool,
            self._emergency_topic,
            queue_size=5,
        )

    def _emergency_topic(self, message):
        if message.data:
            with self.lock:
                self.q1_emergency = True

    def _context(self, message):
        super()._context(message)
        if (
            message.valid
            and message.transition_state
            in {
                ContextState.TRANSITION_SAFE_OVERRIDE,
                ContextState.TRANSITION_FAULTED,
            }
        ):
            with self.lock:
                self.q1_emergency = True

    def _transaction(self, message):
        super()._transaction(message)
        with self.lock:
            self.q1_latest_projection = bool(message.projection_reason_mask)
            self.q1_latest_transaction_valid = bool(
                message.valid and message.activated and not message.fault_reason
            )
            if message.safety_reason_mask:
                self.q1_emergency = True

    def _sample(self, goal_reached=False):
        row = super()._sample(goal_reached=goal_reached)
        if row is None:
            return None
        with self.lock:
            normalized = {
                "stamp_s": float(row["stamp_s"]),
                "x_m": float(row["x_m"]),
                "y_m": float(row["y_m"]),
                "linear_velocity_mps": float(row["linear_velocity_mps"]),
                "yaw_rate_radps": float(row["angular_velocity_radps"]),
                "clearance_m": float(row["clearance_m"]),
                "predicted_ttc_s": (
                    None
                    if row["predicted_ttc_s"] is None
                    else float(row["predicted_ttc_s"])
                ),
                "collision": bool(row["collision"]),
                "goal_reached": bool(row["goal_reached"]),
                "emergency": bool(self.q1_emergency),
                "projection_applied": bool(self.q1_latest_projection),
                "transaction_valid": bool(self.q1_latest_transaction_valid),
                "tracker_valid": bool(self.latest_tracker_healthy),
            }
            if (
                not self.q1_samples
                or normalized["stamp_s"] > self.q1_samples[-1]["stamp_s"]
            ):
                self.q1_samples.append(normalized)
        return row

    def run(self):
        evaluation = super().run()
        if len(self.q1_samples) < 2:
            raise RuntimeError("Q1 navigation raw trace is too short")
        identity = {
            "sequence": self.q1_sequence,
            "row_type": "calibration_navigation",
            "scene_id": self.scene["scene_id"],
            "seed": int(self.scene["seed"]),
            "attempt": self.r6_attempt,
            "method_id": self.q1_actual_method_id,
            "block_id": "p2-q1-cal-block-s{}".format(int(self.scene["seed"])),
        }
        raw_path = self.output_dir / "raw_trace.yaml"
        _atomic_yaml(
            raw_path,
            {
                "schema_version": "1.0",
                "stage": STAGE,
                "identity": identity,
                "samples": self.q1_samples,
            },
        )
        evaluation.update(
            {
                "stage": STAGE,
                "method_id": self.q1_actual_method_id,
                "sequence": self.q1_sequence,
                "block_id": identity["block_id"],
                "raw_trace_path": str(raw_path),
                "raw_trace_sha256": hashlib.sha256(raw_path.read_bytes()).hexdigest(),
                "raw_trace_sample_count": len(self.q1_samples),
                "runtime_scene_labels_available": False,
                "training_used": False,
            }
        )
        _atomic_yaml(self.output_dir / "evaluation.yaml", evaluation)
        return evaluation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True)
    parser.add_argument("--instance-sha256", required=True)
    parser.add_argument("--method-id", choices=METHODS, required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--stage", choices=(STAGE,), required=True)
    parser.add_argument("--sequence", type=int, choices=range(1, 331), required=True)
    parser.add_argument("--attempt", type=int, choices=(1,), required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    output = Path(args.output_dir).resolve()
    output.relative_to(
        (
            WORKSPACE
            / "artifacts/v2/integration/v2_04g_r6_i1/p2_q1_execution"
        ).resolve()
    )
    instance_path = Path(args.instance).resolve()
    payload = instance_path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != args.instance_sha256:
        raise RuntimeError("Q1 source instance SHA256 differs")
    instance = yaml.safe_load(payload)
    if (
        instance.get("scene", {}).get("split") != "calibration"
        or instance["scene"].get("seed") not in range(5301, 5331)
    ):
        raise RuntimeError("Q1 navigation instance identity differs")
    semantic = args.method_id in CIRCLE_METHODS
    p1_alias = (
        "r6_semantics_circle_contact" if semantic else "fixed_teb_reference"
    )
    method = "rule_multi_anchor" if semantic else "fixed_teb"
    source_binding = {
        "path": str(instance_path.relative_to(WORKSPACE)),
        "sha256": digest,
    }
    rospy.init_node("v2_04g_p2_q1_navigation_episode")
    report = Q1NavigationEpisode(
        instance,
        method,
        output,
        "calibration",
        p1_alias,
        attempt=args.attempt,
        source_binding=source_binding,
        method_id=p1_alias,
        actual_method_id=args.method_id,
        sequence=args.sequence,
    ).run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
