#!/usr/bin/env python3
"""Run one P1-E1 episode and record a timestamped, behavior-read-only overlay trace."""

import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile

import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from teb_mode_manager.msg import ContextState


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
STAGE = "V2-04G-P1-E1"
SOURCE = Path(__file__).with_name("v2_04g_r6_i1_mechanism_episode.py")
_SPEC = importlib.util.spec_from_file_location("p1_e1_frozen_r6_i1_episode", SOURCE)
_BASE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_BASE)
_BASE.STAGE = STAGE

DYNAMIC_NAMES = {
    ContextState.DYNAMIC_NONE: "NONE",
    ContextState.DYNAMIC_CROSSING: "CROSSING",
    ContextState.DYNAMIC_HEAD_ON: "HEAD_ON",
    ContextState.DYNAMIC_FOLLOW: "FOLLOW",
    ContextState.DYNAMIC_OVERTAKE_OR_YIELD: "OVERTAKE_OR_YIELD",
}
SEMANTIC_METHODS = {
    "r6_semantics_legacy_control",
    "r6_semantics_circle_contact",
}


def _canonical_sha(value):
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def _atomic_yaml(path, value):
    target = Path(path)
    payload = yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")
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


class P1Episode(_BASE.R6I1MechanismEpisode):
    """Add exact goal-window overlay evidence without changing policy inputs."""

    def __init__(self, *args, source_binding, method_id, **kwargs):
        self.p1_method_id = method_id
        self.p1_source_binding = source_binding
        self.p1_goal_dispatch_stamp_s = None
        self.p1_terminal_stamp_s = None
        self.p1_terminal_reason = None
        self.p1_measurement_active = False
        self.p1_latest_raw_overlay = None
        self.p1_latest_context_receipt_s = None
        self.p1_overlay_events = []
        super().__init__(*args, **kwargs)

    def _mapped_overlay(self, raw_state):
        if raw_state == "NONE":
            return "NONE"
        if self.p1_method_id == "r6_semantics_legacy_control":
            return "LEGACY_DYNAMIC"
        if self.p1_method_id == "r6_semantics_circle_contact":
            return "CIRCLE_CONTACT_DYNAMIC"
        raise RuntimeError("fixed-TEB received a non-NONE supervisor overlay")

    def _append_overlay_event(self, stamp_s, raw_state, source):
        mapped = self._mapped_overlay(raw_state)
        if self.p1_overlay_events:
            previous = self.p1_overlay_events[-1]
            if stamp_s < previous["stamp_s"] - 1.0e-9:
                raise RuntimeError("overlay receipt time moved backwards")
            if mapped == previous["mapped_state"]:
                return
        self.p1_overlay_events.append({
            "stamp_s": float(stamp_s),
            "raw_state": raw_state,
            "mapped_state": mapped,
            "source": source,
        })

    def _context(self, message):
        super()._context(message)
        if not message.valid:
            return
        receipt = rospy.Time.now().to_sec()
        raw_state = DYNAMIC_NAMES.get(message.dynamic_overlay, "UNKNOWN")
        if raw_state == "UNKNOWN":
            raise RuntimeError("unknown ContextState dynamic overlay")
        with self.lock:
            self.p1_latest_raw_overlay = raw_state
            self.p1_latest_context_receipt_s = receipt
            if (
                self.p1_measurement_active
                and self.p1_terminal_stamp_s is None
                and receipt >= self.p1_goal_dispatch_stamp_s
            ):
                self._append_overlay_event(receipt, raw_state, "context_receipt")

    def _record_terminal(self, stamp_s, reason):
        with self.lock:
            if self.p1_terminal_stamp_s is None:
                value = max(float(stamp_s), float(self.p1_goal_dispatch_stamp_s or stamp_s))
                self.p1_terminal_stamp_s = value
                self.p1_terminal_reason = reason
                self.p1_measurement_active = False

    def _sample(self, goal_reached=False):
        row = super()._sample(goal_reached=goal_reached)
        if row is not None and row.get("collision"):
            self._record_terminal(row["stamp_s"], "collision_sample")
        return row

    def _install_goal_window_hooks(self):
        original_send_goal = self.move_base.send_goal
        original_get_state = self.move_base.get_state
        original_cancel_goal = self.move_base.cancel_goal

        def send_goal(goal, *args, **kwargs):
            stamp = rospy.Time.now().to_sec()
            with self.lock:
                if self.p1_goal_dispatch_stamp_s is not None:
                    raise RuntimeError("move_base goal dispatched more than once")
                if self.p1_method_id == "fixed_teb_reference":
                    initial = "NONE"
                    source = "fixed_mode_structural_none"
                else:
                    initial = self.p1_latest_raw_overlay
                    source = "last_valid_context_at_dispatch"
                    if initial is None:
                        raise RuntimeError("no valid ContextState before goal dispatch")
                self.p1_goal_dispatch_stamp_s = stamp
                self.p1_measurement_active = True
                self._append_overlay_event(stamp, initial, source)
            return original_send_goal(goal, *args, **kwargs)

        def get_state():
            state = original_get_state()
            if state in {
                GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                GoalStatus.REJECTED, GoalStatus.LOST,
            }:
                self._record_terminal(
                    rospy.Time.now().to_sec(), "move_base_state_{}".format(state)
                )
            return state

        def cancel_goal():
            self._record_terminal(rospy.Time.now().to_sec(), "goal_cancel_requested")
            return original_cancel_goal()

        self.move_base.send_goal = send_goal
        self.move_base.get_state = get_state
        self.move_base.cancel_goal = cancel_goal
        return original_send_goal, original_get_state, original_cancel_goal

    def _overlay_document(self):
        start = self.p1_goal_dispatch_stamp_s
        end = self.p1_terminal_stamp_s
        if not (
            isinstance(start, float) and isinstance(end, float)
            and math.isfinite(start) and math.isfinite(end) and end > start
        ):
            raise RuntimeError("goal-window timestamps are incomplete")
        events = []
        for event in self.p1_overlay_events:
            if event["stamp_s"] < start - 1.0e-9 or event["stamp_s"] > end + 1.0e-9:
                continue
            current = dict(event)
            current["offset_s"] = max(0.0, min(end - start, event["stamp_s"] - start))
            if events and current["mapped_state"] == events[-1]["mapped_state"]:
                continue
            events.append(current)
        if not events or abs(events[0]["offset_s"]) > 1.0e-9:
            raise RuntimeError("overlay trace lacks dispatch-bound initial state")
        return {
            "schema_version": "1.0",
            "stage": STAGE,
            "method_id": self.p1_method_id,
            "scene_id": self.scene["scene_id"],
            "seed": int(self.scene["seed"]),
            "attempt": self.r6_attempt,
            "behavior_read_only": True,
            "runtime_policy_received_trace": False,
            "timestamp_source": "ros_sim_time_context_receipt",
            "window_definition": "first_goal_dispatch_through_first_terminal_event",
            "goal_dispatch_stamp_s": start,
            "terminal_event_stamp_s": end,
            "terminal_reason": self.p1_terminal_reason,
            "observation_duration_s": end - start,
            "source_instance": dict(self.p1_source_binding),
            "events": events,
        }

    def run(self):
        hooks = self._install_goal_window_hooks()
        try:
            evaluation = super().run()
        finally:
            (
                self.move_base.send_goal,
                self.move_base.get_state,
                self.move_base.cancel_goal,
            ) = hooks
        if self.p1_terminal_stamp_s is None:
            self._record_terminal(rospy.Time.now().to_sec(), "episode_returned")
        overlay = self._overlay_document()
        overlay_path = self.output_dir / "overlay_trace.yaml"
        _atomic_yaml(overlay_path, overlay)
        evaluation.update({
            "stage": STAGE,
            "p1_method_id": self.p1_method_id,
            "source_instance_path": self.p1_source_binding["path"],
            "source_instance_sha256": self.p1_source_binding["sha256"],
            "adapted_instance_sha256": self.instance["instance_sha256"],
            "p1_split_adapter": "test_id_to_evaluator_calibration_only",
            "overlay_trace_path": str(overlay_path),
            "overlay_trace_sha256": hashlib.sha256(overlay_path.read_bytes()).hexdigest(),
            "goal_dispatch_stamp_s": overlay["goal_dispatch_stamp_s"],
            "terminal_event_stamp_s": overlay["terminal_event_stamp_s"],
            "observation_duration_s": overlay["observation_duration_s"],
        })
        audit = evaluation.get("clearance_audit")
        if isinstance(audit, dict):
            audit["stage"] = STAGE
            _atomic_yaml(self.output_dir / "clearance_audit.yaml", audit)
        _atomic_yaml(self.output_dir / "evaluation.yaml", evaluation)
        return evaluation


def _adapt_instance(source):
    if source.get("scene", {}).get("split") != "test_id":
        raise RuntimeError("P1 source instance must retain frozen test_id split")
    adapted = copy.deepcopy(source)
    adapted["scene"]["split"] = "calibration"
    unhashed = {
        key: adapted[key] for key in ("schema_version", "generator", "scene")
    }
    adapted["instance_sha256"] = _canonical_sha(unhashed)
    return adapted


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True)
    parser.add_argument("--instance-sha256", required=True)
    parser.add_argument(
        "--method-id",
        choices=(
            "r6_semantics_legacy_control",
            "r6_semantics_circle_contact",
            "fixed_teb_reference",
        ),
        required=True,
    )
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--stage", choices=(STAGE,), required=True)
    parser.add_argument("--attempt", type=int, choices=(1,), required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    output = Path(args.output_dir).resolve()
    output.relative_to(
        (WORKSPACE / "artifacts/v2/integration/v2_04g_r6_i1/p1_e1_execution").resolve()
    )
    instance_path = Path(args.instance).resolve()
    payload = instance_path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != args.instance_sha256:
        raise RuntimeError("P1 source instance SHA256 differs")
    source = yaml.safe_load(payload)
    adapted = _adapt_instance(source)
    method = "fixed_teb" if args.method_id == "fixed_teb_reference" else "rule_multi_anchor"
    source_binding = {
        "path": str(instance_path.relative_to(WORKSPACE)),
        "sha256": digest,
    }
    rospy.init_node("v2_04g_p1_e1_episode")
    report = P1Episode(
        adapted,
        method,
        output,
        "calibration",
        args.method_id,
        attempt=args.attempt,
        source_binding=source_binding,
        method_id=args.method_id,
    ).run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
