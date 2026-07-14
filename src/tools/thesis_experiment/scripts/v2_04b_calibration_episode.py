#!/usr/bin/env python3
"""Run one no-training calibration-split episode with a typed Anchor candidate."""

import argparse
import csv
import hashlib
import math
from pathlib import Path
import threading

import actionlib
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path as NavPath
from nav_world_model.msg import TrackedObstacle, TrackedObstacleArray, WorldModelHealth
import rospy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
import yaml

from teb_mode_manager.action_pipeline import AnchorBank, TypedProfile
from teb_mode_manager.typed_teb_transaction import (
    EXPECTED_TEB_NAMESPACE,
    RosTypedDynamicReconfigureAdapter,
    SimulationWriteContext,
    TypedTebTransactionBackend,
    require_simulation_write,
)
from thesis_experiment.v2_evaluator import TRACE_COLUMNS, evaluate_v2_episode, trace_sha256
from thesis_experiment.v2_scene import canonical_sha256
from thesis_experiment.v2_04b_calibration import apply_candidate_overlay


WORKSPACE = Path("/home/robot/robot_ws_base_rl")


def _yaw(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def _bool_text(value):
    return "true" if value else "false"


def _predicted_ttc(obstacles, interaction_radius_m=1.0, horizon_s=5.0):
    """Return the earliest constant-relative-velocity interaction time."""

    earliest = None
    for obstacle in obstacles:
        if obstacle.motion_class in (
            TrackedObstacle.MOTION_UNKNOWN,
            TrackedObstacle.MOTION_STATIONARY,
            TrackedObstacle.MOTION_DEPARTING,
        ):
            continue
        px = float(obstacle.pose.pose.position.x)
        py = float(obstacle.pose.pose.position.y)
        vx = float(obstacle.velocity.twist.linear.x)
        vy = float(obstacle.velocity.twist.linear.y)
        a = vx * vx + vy * vy
        if a <= 1.0e-8:
            continue
        b = 2.0 * (px * vx + py * vy)
        c = px * px + py * py - interaction_radius_m * interaction_radius_m
        discriminant = b * b - 4.0 * a * c
        if discriminant < 0.0 or b >= 0.0:
            continue
        root = (-b - math.sqrt(discriminant)) / (2.0 * a)
        if 0.0 <= root <= horizon_s and (earliest is None or root < earliest):
            earliest = root
    return earliest


def _footprint_clearance(scan):
    """Ray-to-rectangle clearance for the frozen 1.04 m x 0.70 m chassis."""

    clearances = []
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        cosine, sine = math.cos(angle), math.sin(angle)
        x_limit = 0.52 / abs(cosine) if abs(cosine) > 1.0e-9 else float("inf")
        y_limit = 0.35 / abs(sine) if abs(sine) > 1.0e-9 else float("inf")
        boundary = min(x_limit, y_limit)
        clearances.append(max(0.0, float(distance) - boundary))
    return min(clearances) if clearances else float(scan.range_max)


class EpisodeRunner:
    def __init__(self, instance, candidate, bank, output_dir, timeout_s,
                 allow_timeout_override=False):
        self.instance = instance
        self.scene = instance["scene"]
        self.candidate = candidate
        self.bank = bank
        self.output_dir = output_dir
        requested_timeout = float(timeout_s)
        self.timeout_s = (
            requested_timeout if allow_timeout_override
            else min(requested_timeout, float(self.scene["timeout_s"]))
        )
        self.lock = threading.RLock()
        self.odom = None
        self.scan = None
        self.commanded_speed = 0.0
        self.contact_count = 0
        self.global_replan_count = 0
        self.predicted_ttc_s = None
        self.tracker_message_count = 0
        self.tracker_health_message_count = 0
        self.tracker_health_valid_count = 0
        self.finite_ttc_sample_count = 0
        self.evaluation_entry = None
        rospy.Subscriber("/odom", Odometry, self._odom, queue_size=5)
        rospy.Subscriber("/scan", LaserScan, self._scan, queue_size=5)
        rospy.Subscriber("/cmd_vel", Twist, self._command, queue_size=5)
        rospy.Subscriber("/m2_gazebo/contacts", ContactsState, self._contact, queue_size=20)
        rospy.Subscriber("/move_base/NavfnROS/plan", NavPath, self._global_plan, queue_size=5)
        rospy.Subscriber(
            "/nav_world_model/tracks", TrackedObstacleArray, self._tracks, queue_size=5
        )
        rospy.Subscriber(
            "/nav_world_model/health", WorldModelHealth, self._world_model_health,
            queue_size=5,
        )
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

    def _odom(self, message):
        with self.lock:
            self.odom = message

    def _scan(self, message):
        with self.lock:
            self.scan = message

    def _command(self, message):
        with self.lock:
            self.commanded_speed = float(message.linear.x)

    def _contact(self, message):
        if message.states:
            with self.lock:
                self.contact_count += len(message.states)

    def _global_plan(self, message):
        if message.poses:
            with self.lock:
                self.global_replan_count += 1

    def _tracks(self, message):
        with self.lock:
            self.tracker_message_count += 1
            self.predicted_ttc_s = _predicted_ttc(message.obstacles)

    def _world_model_health(self, message):
        with self.lock:
            self.tracker_health_message_count += 1
            if message.valid and not message.stale and message.tracker_valid:
                self.tracker_health_valid_count += 1

    def _connect_backend(self):
        clock = rospy.wait_for_message("/clock", Clock, timeout=10.0)
        write_context = SimulationWriteContext(
            explicit_simulation_write=True,
            use_sim_time=rospy.get_param("/use_sim_time", False) is True,
            simulation_marker=rospy.get_param("/m2_gazebo/simulation_only", False) is True,
            gazebo_clock_active=clock.clock.to_sec() > 0.0,
            teb_namespace=EXPECTED_TEB_NAMESPACE,
        )
        require_simulation_write(write_context)
        adapter = RosTypedDynamicReconfigureAdapter(EXPECTED_TEB_NAMESPACE, 3.0)
        backend = TypedTebTransactionBackend(
            self.bank,
            adapter,
            write_context,
            timeout_s=3.0,
            time_source=lambda: rospy.Time.now().to_sec(),
        )
        backend.initialize()
        return backend

    def _ramp_candidate(self, backend):
        base = self.bank.validate_values(self.candidate["values"], "calibration candidate")
        if canonical_sha256(base) != self.candidate["profile_sha256"]:
            raise RuntimeError("candidate profile hash mismatch")
        target = apply_candidate_overlay(
            self.bank, base, self.evaluation_entry["dynamic_overlay"]
        )
        if canonical_sha256(target) != self.evaluation_entry["effective_profile_sha256"]:
            raise RuntimeError("effective candidate/overlay profile hash mismatch")
        rate_hz = float(self.bank.transaction["decision_frequency_hz"])
        dt = 1.0 / rate_hz
        rate = rospy.Rate(rate_hz)
        receipts = []
        for _ in range(200):
            current = backend.current.values
            values = dict(current)
            continuous_converged = True
            for name, definition in self.bank.definitions.items():
                if not definition.continuous:
                    continue
                delta = float(target[name]) - float(current[name])
                step = max(-definition.max_rate_per_s * dt,
                           min(definition.max_rate_per_s * dt, delta))
                values[name] = float(current[name]) + step
                if abs(float(target[name]) - values[name]) > 1.0e-6:
                    continuous_converged = False
            if continuous_converged:
                for name, definition in self.bank.definitions.items():
                    if not definition.continuous:
                        values[name] = target[name]
            profile = TypedProfile(
                self.candidate["anchor_id"], self.candidate["candidate_id"],
                self.bank.validate_values(values, "calibration ramp"),
            )
            receipts.append(backend.apply(profile, rospy.Time.now().to_sec()))
            if all(
                abs(float(profile.values[name]) - float(target[name])) <= 1.0e-8
                if definition.parameter_type == "double"
                else profile.values[name] == target[name]
                for name, definition in self.bank.definitions.items()
            ):
                return receipts
            rate.sleep()
        raise RuntimeError("candidate ramp did not converge")

    def _sample(self, goal_reached=False):
        with self.lock:
            if self.odom is None or self.scan is None:
                return None
            odom, scan = self.odom, self.scan
            clearance = _footprint_clearance(scan)
            pose = odom.pose.pose
            twist = odom.twist.twist
            goal = self.scene["goal"]
            distance = math.hypot(goal["x_m"] - pose.position.x,
                                  goal["y_m"] - pose.position.y)
            velocity = float(twist.linear.x)
            gear = "REVERSE" if velocity < -0.01 else ("FORWARD" if velocity > 0.01 else "NEUTRAL")
            if self.predicted_ttc_s is not None:
                self.finite_ttc_sample_count += 1
            return {
                "stamp_s": rospy.Time.now().to_sec(),
                "x_m": float(pose.position.x),
                "y_m": float(pose.position.y),
                "yaw_rad": _yaw(pose.orientation),
                "linear_velocity_mps": velocity,
                "angular_velocity_radps": float(twist.angular.z),
                "commanded_speed_mps": self.commanded_speed,
                "clearance_m": clearance,
                "goal_distance_m": distance,
                "collision": self.contact_count > 0,
                "goal_reached": bool(goal_reached),
                "contact_count": self.contact_count,
                "topology_id": "single_teb",
                "global_replan_count": self.global_replan_count,
                "recovery_count": 0,
                "gear": gear,
                "predicted_ttc_s": self.predicted_ttc_s,
            }

    def run(self):
        if self.scene["split"] != "calibration":
            raise RuntimeError("episode runner refuses non-calibration split")
        self.evaluation_entry = next((
            row for row in self.candidate["evaluations"]
            if row["scene_id"] == self.scene["scene_id"] and row["split"] == "calibration"
        ), None)
        if self.evaluation_entry is None:
            raise RuntimeError("candidate is not preregistered for this calibration scene")
        backend = self._connect_backend()
        rows = []
        failure = ""
        terminal_state = GoalStatus.PENDING
        ramp_receipts = []
        try:
            ramp_receipts = self._ramp_candidate(backend)
            if not self.move_base.wait_for_server(rospy.Duration(15.0)):
                raise RuntimeError("move_base action server unavailable")
            rospy.wait_for_message("/odom", Odometry, timeout=10.0)
            rospy.wait_for_message("/scan", LaserScan, timeout=10.0)
            goal_data = self.scene["goal"]
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "odom"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = goal_data["x_m"]
            goal.target_pose.pose.position.y = goal_data["y_m"]
            goal.target_pose.pose.orientation.z = math.sin(goal_data["yaw_rad"] / 2.0)
            goal.target_pose.pose.orientation.w = math.cos(goal_data["yaw_rad"] / 2.0)
            self.move_base.send_goal(goal)
            start = rospy.Time.now().to_sec()
            rate = rospy.Rate(10.0)
            while rospy.Time.now().to_sec() - start < self.timeout_s:
                terminal_state = self.move_base.get_state()
                row = self._sample(goal_reached=terminal_state == GoalStatus.SUCCEEDED)
                if row is not None and (not rows or row["stamp_s"] > rows[-1]["stamp_s"]):
                    rows.append(row)
                if row is not None and row["collision"]:
                    failure = "collision"
                    self.move_base.cancel_goal()
                    break
                if terminal_state == GoalStatus.SUCCEEDED:
                    break
                if terminal_state in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.LOST):
                    failure = "move_base_state_{}".format(terminal_state)
                    break
                rate.sleep()
            else:
                failure = "timeout"
                self.move_base.cancel_goal()
            if terminal_state == GoalStatus.SUCCEEDED:
                rospy.sleep(0.3)
                final = self._sample(goal_reached=True)
                if final is not None and final["stamp_s"] > rows[-1]["stamp_s"]:
                    rows.append(final)
        finally:
            try:
                self.move_base.cancel_all_goals()
            except Exception:
                pass
            backend.close()
        if len(rows) < 2:
            raise RuntimeError("episode produced fewer than two trace rows")
        self.output_dir.mkdir(parents=True, exist_ok=True)
        trace_path = self.output_dir / "trace.csv"
        with trace_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=TRACE_COLUMNS)
            writer.writeheader()
            for row in rows:
                encoded = dict(row)
                encoded["collision"] = _bool_text(row["collision"])
                encoded["goal_reached"] = _bool_text(row["goal_reached"])
                writer.writerow(encoded)
        evaluation = evaluate_v2_episode(
            self.instance, rows, trace_sha256(trace_path)
        )
        if self.tracker_message_count <= 0 or self.tracker_health_valid_count <= 0:
            ttc_status = "TRACKER_INVALID"
        elif self.finite_ttc_sample_count > 0:
            ttc_status = "OBSERVED_CONFLICT"
        else:
            ttc_status = "NO_CONFLICT_IN_HORIZON"
        health_coverage = (
            float(self.tracker_health_valid_count) / self.tracker_health_message_count
            if self.tracker_health_message_count > 0 else 0.0
        )
        evaluation.update({
            "candidate_id": self.candidate["candidate_id"],
            "candidate_profile_sha256": self.candidate["profile_sha256"],
            "dynamic_overlay": self.evaluation_entry["dynamic_overlay"],
            "effective_profile_sha256": self.evaluation_entry["effective_profile_sha256"],
            "typed_ramp_transaction_count": len(ramp_receipts),
            "typed_startup_snapshot_restored": backend.current.values == backend.startup.values,
            "tracker_message_count": self.tracker_message_count,
            "tracker_health_message_count": self.tracker_health_message_count,
            "tracker_health_valid_count": self.tracker_health_valid_count,
            "tracker_health_valid_fraction": health_coverage,
            "finite_ttc_sample_count": self.finite_ttc_sample_count,
            "ttc_status": ttc_status,
            "episode_timeout_s": self.timeout_s,
            "scene_manifest_timeout_s": float(self.scene["timeout_s"]),
            "timeout_override_used": bool(
                self.timeout_s > float(self.scene["timeout_s"])
            ),
            "training_used": False,
            "runtime_policy_manifest_access": False,
            "experiment_manager_calibration_manifest_access": True,
            "runner_fault_reason": failure,
        })
        evaluation_path = self.output_dir / "evaluation.yaml"
        evaluation_path.write_text(
            yaml.safe_dump(evaluation, sort_keys=False), encoding="utf-8"
        )
        progress = {
            "schema_version": "2.0",
            "stage": "V2-04B",
            "status": "calibration_in_progress",
            "formal_result": False,
            "runtime_ready": False,
            "training_started": False,
            "completed_navigation_episode_count": 1,
            "planned_navigation_episode_count": 90,
            "episode": {
                "scene_id": self.scene["scene_id"],
                "split": self.scene["split"],
                "seed": self.scene["seed"],
                "candidate_id": self.candidate["candidate_id"],
                "termination": evaluation["termination"],
                "success": evaluation["metrics"]["common"]["success"],
            },
            "calibration_complete": False,
            "anchor_values_frozen": False,
        }
        (self.output_dir / "progress.yaml").write_text(
            yaml.safe_dump(progress, sort_keys=False), encoding="utf-8"
        )
        return evaluation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True)
    parser.add_argument("--candidate-plan", required=True)
    parser.add_argument("--candidate-id", required=True)
    parser.add_argument("--anchor-bank", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--timeout-s", type=float, default=55.0)
    parser.add_argument("--allow-timeout-override", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("v2_04b_calibration_episode")
    instance = yaml.safe_load(Path(args.instance).read_text(encoding="utf-8"))
    plan = yaml.safe_load(Path(args.candidate_plan).read_text(encoding="utf-8"))
    candidate = next(
        (row for row in plan["candidates"] if row["candidate_id"] == args.candidate_id), None
    )
    if candidate is None:
        raise RuntimeError("candidate ID is not in the preregistered plan")
    output = Path(args.output_dir).resolve()
    output.relative_to((WORKSPACE / "artifacts/v2/calibration").resolve())
    report = EpisodeRunner(
        instance, candidate, AnchorBank.from_file(args.anchor_bank), output, args.timeout_s,
        allow_timeout_override=args.allow_timeout_override,
    ).run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
