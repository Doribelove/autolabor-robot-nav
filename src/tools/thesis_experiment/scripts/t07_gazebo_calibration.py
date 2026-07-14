#!/usr/bin/env python3
"""Execute paired T07 theta perturbations in one persistent T02 Gazebo."""

import csv
import math
import sys
import time
from pathlib import Path

import rospy
import yaml
from actionlib_msgs.msg import GoalStatus

from teb_rl_tuner.safety_gate import SimulationWriteContext
from teb_rl_tuner.teb_parameter_client import RosDynamicReconfigureBackend, TebParameterClient
from thesis_experiment.calibration import (
    CalibrationError,
    analyze_sensitivity,
    build_mapping_document,
    load_observations,
    validate_frozen_mapping,
)
from thesis_experiment.gazebo_training_adapter import GazeboTrainingAdapter
from thesis_experiment.scenario import (
    build_perturbation_plan,
    canonical_sha256,
    load_scenario_manifest,
)


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
CSV_FIELDS = (
    "run_id", "scene_id", "seed", "theta_name", "parameter", "direction",
    "delta", "theta_value", "baseline_theta_value", "success", "navigation_time",
    "path_length", "min_obstacle_distance", "near_obstacle_risk_integral",
    "path_error_integral", "smoothness_integral", "planner_failure_count",
    "speed", "obstacle_conservatism", "clearance", "path_tracking", "smoothness",
)


def _trapz(samples, key):
    total = 0.0
    for left, right in zip(samples, samples[1:]):
        dt = right["stamp"] - left["stamp"]
        total += 0.5 * dt * (left[key] + right[key])
    return total


class CalibrationRunner:
    def __init__(self):
        self.manifest_path = Path(rospy.get_param(
            "~manifest", str(WORKSPACE / "experiments/manifests/t07/calibration_pilot.yaml")
        ))
        self.output_dir = Path(rospy.get_param(
            "~output_dir", str(WORKSPACE / "artifacts/t07/calibration_pilot")
        ))
        # One seed across three geometries already supplies three independent
        # central-difference pairs per matrix entry. The checked-in full plan
        # retains two seeds for a later expanded rerun.
        self.seeds_per_scene = int(rospy.get_param("~seeds_per_scene", 1))
        self.manifest = load_scenario_manifest(self.manifest_path, WORKSPACE)
        self.full_plan = build_perturbation_plan(self.manifest, WORKSPACE)
        allowed = {
            (scene["scene_id"], seed)
            for scene in self.manifest["scenes"]
            for seed in scene["seeds"][: self.seeds_per_scene]
        }
        self.plan = [run for run in self.full_plan if (run["scene_id"], run["seed"]) in allowed]
        self.baseline = self.manifest["theta"]["baseline"]
        self.bounds = self.manifest["theta"]["simulation_candidate_bounds"]
        self.client = None
        self.adapter = None

    def _connect(self):
        namespace = "/move_base/TebLocalPlannerROS"
        context = SimulationWriteContext(
            explicit_simulation=True,
            use_sim_time=rospy.get_param("/use_sim_time", False),
            simulation_marker=rospy.get_param("/m2_gazebo/simulation_only", False),
            teb_namespace=namespace,
        )
        self.client = TebParameterClient(
            RosDynamicReconfigureBackend(namespace, 5.0), context, timeout_s=5.0
        )
        self.client.initialize()
        first = self.plan[0]
        self.adapter = GazeboTrainingAdapter(
            self.client,
            [{
                "scene_id": first["scene_id"],
                "layout": first["layout"],
                "goal": [
                    first["goal"]["x_m"], first["goal"]["y_m"],
                    first["goal"]["yaw_rad"],
                ],
            }],
            self.bounds,
        )

    def _one_episode(self, run, sequence):
        receipt = self.adapter.write_parameters(run["theta"], sequence)
        del receipt
        scenario = {
            "scene_id": run["scene_id"], "layout": run["layout"],
            "goal": [run["goal"]["x_m"], run["goal"]["y_m"], run["goal"]["yaw_rad"]],
        }
        self.adapter.scenarios = (scenario,)
        self.adapter._scenario_index = -1
        self.adapter.reset(run["seed"])
        started = rospy.Time.now().to_sec()
        samples = []
        previous_angular = 0.0
        previous_stamp = started
        timeout = float(run["timeout_s"])
        collision_threshold = float(run["collision"]["threshold_m"])
        collision = False
        while rospy.Time.now().to_sec() - started < timeout:
            metrics = self.adapter._metrics()
            dt = max(metrics["stamp"] - previous_stamp, 1e-6)
            angular_acceleration = (metrics["angular_velocity"] - previous_angular) / dt
            near_risk = max(0.0, 1.0 - metrics["clearance"] / 1.0) ** 2
            samples.append({
                "stamp": metrics["stamp"],
                "near_risk": near_risk,
                "path_error_sq": metrics["path_error"] ** 2,
                "smoothness": metrics["linear_acceleration"] ** 2 + angular_acceleration ** 2,
            })
            previous_angular, previous_stamp = metrics["angular_velocity"], metrics["stamp"]
            if metrics["clearance"] < collision_threshold:
                collision = True
                self.adapter.request_stop("calibration_collision_threshold")
                break
            state = self.adapter.move_base.get_state()
            if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                         GoalStatus.REJECTED, GoalStatus.LOST):
                break
            rospy.sleep(0.05)
        elapsed = rospy.Time.now().to_sec() - started
        state = self.adapter.move_base.get_state()
        success = state == GoalStatus.SUCCEEDED and not collision
        if state not in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                         GoalStatus.REJECTED, GoalStatus.LOST):
            self.adapter.request_stop("calibration_timeout")
        near_integral = _trapz(samples, "near_risk") if len(samples) > 1 else 0.0
        path_integral = _trapz(samples, "path_error_sq") if len(samples) > 1 else 0.0
        smooth_integral = _trapz(samples, "smoothness") if len(samples) > 1 else 0.0
        result = {
            "success": success, "navigation_time": elapsed,
            "path_length": self.adapter.path_length,
            "min_obstacle_distance": self.adapter.minimum_clearance,
            "near_obstacle_risk_integral": near_integral,
            "path_error_integral": path_integral,
            "smoothness_integral": smooth_integral,
            "planner_failure_count": int(state in (GoalStatus.ABORTED,
                                                     GoalStatus.REJECTED, GoalStatus.LOST)),
            "speed": -elapsed,
            "obstacle_conservatism": -near_integral,
            "clearance": self.adapter.minimum_clearance,
            "path_tracking": -path_integral,
            "smoothness": -smooth_integral,
        }
        return result

    def run(self):
        self._connect()
        rows = []
        run_reports = []
        try:
            for index, run in enumerate(self.plan, 1):
                rospy.loginfo("T07 calibration %d/%d: %s", index, len(self.plan), run["run_id"])
                metrics = self._one_episode(run, index)
                run_reports.append({
                    "run_id": run["run_id"], "scene_id": run["scene_id"],
                    "seed": run["seed"], "condition": run["condition"],
                    "success": metrics["success"],
                    "navigation_time": metrics["navigation_time"],
                })
                if run["condition"] == "baseline":
                    targets = [(name, "baseline", 0.0) for name in self.manifest["theta"]["order"]]
                else:
                    name = run["perturbed_parameter"]
                    direction = "plus" if run["perturbation_sign"] > 0 else "minus"
                    physical_delta = run["theta"][name] - self.baseline[name]
                    low, high = self.bounds[name]
                    targets = [(name, direction, 2.0 * physical_delta / (high - low))]
                for name, direction, normalized_delta in targets:
                    row = {
                        "run_id": run["run_id"], "scene_id": run["scene_id"],
                        "seed": run["seed"], "theta_name": name, "parameter": name,
                        "direction": direction, "delta": normalized_delta,
                        "theta_value": run["theta"][name],
                        "baseline_theta_value": self.baseline[name],
                    }
                    row.update(metrics)
                    rows.append(row)
            return self._write_and_analyze(rows, run_reports)
        finally:
            if self.adapter is not None:
                self.adapter.close()

    def _write_and_analyze(self, rows, run_reports):
        self.output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = self.output_dir / "sensitivity_observations.csv"
        temporary = csv_path.with_suffix(".csv.tmp")
        with temporary.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        temporary.replace(csv_path)
        observations = load_observations([csv_path])
        analysis = analyze_sensitivity(
            observations, min_pairs=2, min_sign_consistency=0.5,
            min_abs_sensitivity=1e-8, top_k_per_eta=3,
        )
        candidate = build_mapping_document(
            analysis, "A_TEB_v1_gazebo_calibration", freeze=False
        )
        candidate_path = self.output_dir / "A_TEB_v1.candidate.yaml"
        candidate_path.write_text(yaml.safe_dump(candidate, sort_keys=False), encoding="utf-8")
        frozen = None
        freeze_error = None
        try:
            frozen = build_mapping_document(
                analysis, "A_TEB_v1_gazebo_calibration", freeze=True
            )
            validate_frozen_mapping(frozen, verify_sources=True)
            frozen_path = self.output_dir / "A_TEB_v1.yaml"
            frozen_path.write_text(yaml.safe_dump(frozen, sort_keys=False), encoding="utf-8")
        except CalibrationError as exc:
            freeze_error = str(exc)
        report = {
            "schema_version": 1, "task": "T07", "simulation_only": True,
            "formal_experiment": False, "real_vehicle_use_forbidden": True,
            "full_plan_run_count": len(self.full_plan),
            "executed_run_count": len(self.plan), "observation_row_count": len(rows),
            "full_plan_sha256": canonical_sha256(self.full_plan),
            "executed_plan_sha256": canonical_sha256(self.plan),
            "all_runs_success": all(item["success"] for item in run_reports),
            "incomplete_pair_count": len(analysis["incomplete_pairs"]),
            "frozen": frozen is not None,
            "mapping_sha256": frozen["sha256"] if frozen else None,
            "freeze_error": freeze_error,
            "candidate_sha256": candidate["sha256"],
            "passed": not analysis["incomplete_pairs"],
            "runs": run_reports,
        }
        report_path = self.output_dir / "t07_calibration_report.yaml"
        report_path.write_text(yaml.safe_dump(report, sort_keys=False), encoding="utf-8")
        return report


def main():
    rospy.init_node("t07_gazebo_calibration", anonymous=False)
    report = CalibrationRunner().run()
    if not report["passed"]:
        return 1
    if not report["frozen"]:
        rospy.logwarn("T07 candidate generated but freeze gate stayed closed: %s",
                      report["freeze_error"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
