#!/usr/bin/env python3
"""Run one V2-04E calibration or V2-04F held-out validation episode."""

import argparse
import importlib.util
from pathlib import Path

import rospy
import yaml

from teb_mode_manager.msg import ContextState


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
LEGACY_RUNNER = Path(__file__).with_name("v2_04d_validation_episode.py")
_SPEC = importlib.util.spec_from_file_location("v2_04d_episode_frozen", LEGACY_RUNNER)
_LEGACY = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_LEGACY)

DYNAMIC_NAMES = {
    ContextState.DYNAMIC_NONE: "NONE",
    ContextState.DYNAMIC_CROSSING: "CROSSING",
    ContextState.DYNAMIC_HEAD_ON: "HEAD_ON",
    ContextState.DYNAMIC_FOLLOW: "FOLLOW",
    ContextState.DYNAMIC_OVERTAKE_OR_YIELD: "OVERTAKE_OR_YIELD",
}


class SupervisorRepairEpisode(_LEGACY.ValidationEpisode):
    """Add mode occupancy evidence while retaining the frozen V2 evaluator."""

    def __init__(self, instance, method, output_dir, stage, split, profile_id):
        if stage not in ("V2-04E", "V2-04E2", "V2-04E3", "V2-04E4", "V2-04F"):
            raise RuntimeError("unsupported supervisor-repair stage")
        if split not in ("calibration", "validation"):
            raise RuntimeError("unsupported supervisor-repair split")
        if instance["scene"]["split"] != split:
            raise RuntimeError("episode split and compiled scene split disagree")
        self.stage = stage
        self.requested_split = split
        self.profile_id = profile_id
        self.context_geometry_counts = {name: 0 for name in _LEGACY.GEOMETRY_NAMES.values()}
        self.context_overlay_counts = {name: 0 for name in DYNAMIC_NAMES.values()}
        super().__init__(instance, method, output_dir)

    def _context(self, message):
        super()._context(message)
        if not message.valid:
            return
        with self.lock:
            geometry = _LEGACY.GEOMETRY_NAMES.get(message.geometry_mode, "UNKNOWN")
            overlay = DYNAMIC_NAMES.get(message.dynamic_overlay, "UNKNOWN")
            self.context_geometry_counts[geometry] = (
                self.context_geometry_counts.get(geometry, 0) + 1
            )
            self.context_overlay_counts[overlay] = (
                self.context_overlay_counts.get(overlay, 0) + 1
            )

    def _wait_ready(self):
        super()._wait_ready()
        # Readiness traffic proves interface health but is not navigation-time
        # behavior. Start occupancy/chatter measurement immediately before the
        # goal is sent by the inherited runner.
        with self.lock:
            self.context_message_count = 0
            self.context_valid_count = 0
            self.context_geometries = []
            self.context_geometry_counts = {
                name: 0 for name in _LEGACY.GEOMETRY_NAMES.values()
            }
            self.context_overlay_counts = {name: 0 for name in DYNAMIC_NAMES.values()}
            self.active_anchors = [self.active_anchor] if self.active_anchor else []

    def run(self):
        # The V2-04D ROS runner asserts a validation split before collecting a
        # trace. Adapt only that assertion; restore the original compiled scene
        # before the unchanged evaluator verifies its instance hash.
        original_split = self.scene["split"]
        original_evaluator = _LEGACY.evaluate_v2_episode

        def evaluate_original_instance(instance, rows, raw_trace_sha256):
            adapted_split = instance["scene"]["split"]
            instance["scene"]["split"] = original_split
            try:
                return original_evaluator(instance, rows, raw_trace_sha256)
            finally:
                instance["scene"]["split"] = adapted_split

        if self.requested_split == "calibration":
            self.scene["split"] = "validation"
            _LEGACY.evaluate_v2_episode = evaluate_original_instance
        try:
            evaluation = super().run()
        finally:
            self.scene["split"] = original_split
            _LEGACY.evaluate_v2_episode = original_evaluator
        total_geometry = sum(self.context_geometry_counts.values())
        total_overlay = sum(self.context_overlay_counts.values())
        evaluation.update({
            "stage": self.stage,
            "split": self.requested_split,
            "supervisor_profile_id": self.profile_id,
            "context_geometry_sample_counts": dict(self.context_geometry_counts),
            "context_geometry_sample_fractions": {
                key: (float(value) / total_geometry if total_geometry else 0.0)
                for key, value in self.context_geometry_counts.items()
            },
            "context_overlay_sample_counts": dict(self.context_overlay_counts),
            "context_overlay_sample_fractions": {
                key: (float(value) / total_overlay if total_overlay else 0.0)
                for key, value in self.context_overlay_counts.items()
            },
            "experiment_manager_calibration_manifest_access": (
                self.requested_split == "calibration"
            ),
            "experiment_manager_validation_manifest_access": (
                self.requested_split == "validation"
            ),
            "mode_measurement_window": "post_readiness_goal_execution_only",
        })
        (self.output_dir / "evaluation.yaml").write_text(
            yaml.safe_dump(evaluation, sort_keys=False), encoding="utf-8"
        )
        return evaluation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True)
    parser.add_argument("--method", choices=_LEGACY.METHODS, required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--stage", choices=("V2-04E", "V2-04E2", "V2-04E3", "V2-04E4", "V2-04F"),
        required=True,
    )
    parser.add_argument("--split", choices=("calibration", "validation"), required=True)
    parser.add_argument("--profile-id", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    expected_root = WORKSPACE / "artifacts/v2" / args.split / args.stage.lower().replace("-", "_")
    output = Path(args.output_dir).resolve()
    output.relative_to(expected_root.resolve())
    instance = yaml.safe_load(Path(args.instance).read_text(encoding="utf-8"))
    rospy.init_node("{}_supervisor_episode".format(args.stage.lower().replace("-", "_")))
    report = SupervisorRepairEpisode(
        instance, args.method, output, args.stage, args.split, args.profile_id
    ).run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
