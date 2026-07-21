#!/usr/bin/env python3
"""Run one V2-04D no-training paired-validation episode."""

import argparse
import csv
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
from sensor_msgs.msg import LaserScan
import yaml

from teb_mode_manager.msg import ContextState, ParameterTransaction
from thesis_experiment.v2_evaluator import TRACE_COLUMNS, evaluate_v2_episode, trace_sha256


WORKSPACE = Path("/home/robot/robot_ws_base_rl")
METHODS = ("fixed_teb", "balanced_anchor", "rule_multi_anchor")
GEOMETRY_NAMES = {
    ContextState.GEOMETRY_BALANCED: "BALANCED",
    ContextState.GEOMETRY_CRUISE: "CRUISE",
    ContextState.GEOMETRY_STATIC_DENSE: "STATIC_DENSE",
    ContextState.GEOMETRY_CORRIDOR: "CORRIDOR",
    ContextState.GEOMETRY_MANEUVER: "MANEUVER",
}


def _yaw(quaternion):
    siny = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny, cosy)


def _bool_text(value):
    return "true" if value else "false"


def _predicted_ttc(obstacles, interaction_radius_m=1.0, horizon_s=5.0):
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
    clearances = []
    for index, distance in enumerate(scan.ranges):
        if not math.isfinite(distance) or not scan.range_min <= distance <= scan.range_max:
            continue
        angle = scan.angle_min + index * scan.angle_increment
        cosine, sine = math.cos(angle), math.sin(angle)
        x_limit = 0.52 / abs(cosine) if abs(cosine) > 1.0e-9 else float("inf")
        y_limit = 0.35 / abs(sine) if abs(sine) > 1.0e-9 else float("inf")
        clearances.append(max(0.0, float(distance) - min(x_limit, y_limit)))
    return min(clearances) if clearances else float(scan.range_max)


class ValidationEpisode:
    def __init__(self, instance, method, output_dir):
        if method not in METHODS:
            raise RuntimeError("unknown V2-04D method")
        self.instance = instance
        self.scene = instance["scene"]
        self.method = method
        self.output_dir = output_dir
        self.timeout_s = float(self.scene["timeout_s"])
        self.lock = threading.RLock()
        self.odom = None
        self.scan = None
        self.commanded_speed = 0.0
        self.contact_count = 0
        self.global_replan_count = 0
        self.predicted_ttc_s = None
        self.tracker_message_count = 0
        self.health_message_count = 0
        self.health_valid_count = 0
        self.finite_ttc_sample_count = 0
        self.context_message_count = 0
        self.context_valid_count = 0
        self.context_geometries = []
        self.transaction_message_count = 0
        self.transaction_valid_count = 0
        self.transaction_activated_count = 0
        self.transaction_fault_count = 0
        self.transaction_training_used = False
        self.transaction_backends = set()
        self.active_anchor = "fixed_teb" if method == "fixed_teb" else ""
        self.active_anchors = []
        rospy.Subscriber("/odom", Odometry, self._odom, queue_size=5)
        rospy.Subscriber("/scan", LaserScan, self._scan, queue_size=5)
        rospy.Subscriber("/cmd_vel", Twist, self._command, queue_size=5)
        rospy.Subscriber("/m2_gazebo/contacts", ContactsState, self._contact, queue_size=20)
        rospy.Subscriber("/move_base/NavfnROS/plan", NavPath, self._global_plan, queue_size=5)
        rospy.Subscriber("/nav_world_model/tracks", TrackedObstacleArray,
                         self._tracks, queue_size=5)
        rospy.Subscriber("/nav_world_model/health", WorldModelHealth,
                         self._health, queue_size=5)
        rospy.Subscriber("/teb_mode_manager/context", ContextState,
                         self._context, queue_size=10)
        rospy.Subscriber("/teb_rl_v2/action_trace", ParameterTransaction,
                         self._transaction, queue_size=20)
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

    def _health(self, message):
        with self.lock:
            self.health_message_count += 1
            if message.valid and not message.stale and message.tracker_valid:
                self.health_valid_count += 1

    def _context(self, message):
        with self.lock:
            self.context_message_count += 1
            if message.valid:
                self.context_valid_count += 1
                name = GEOMETRY_NAMES.get(message.geometry_mode, "UNKNOWN")
                if not self.context_geometries or self.context_geometries[-1] != name:
                    self.context_geometries.append(name)

    def _transaction(self, message):
        with self.lock:
            self.transaction_message_count += 1
            self.transaction_training_used |= bool(message.training_used)
            self.transaction_backends.add(message.execution_backend)
            if message.valid:
                self.transaction_valid_count += 1
            if message.activated:
                self.transaction_activated_count += 1
                self.active_anchor = message.anchor_id
                if not self.active_anchors or self.active_anchors[-1] != message.anchor_id:
                    self.active_anchors.append(message.anchor_id)
            if message.fault_reason:
                self.transaction_fault_count += 1

    def _wait_ready(self):
        if not self.move_base.wait_for_server(rospy.Duration(15.0)):
            raise RuntimeError("move_base action server unavailable")
        rospy.wait_for_message("/odom", Odometry, timeout=10.0)
        rospy.wait_for_message("/scan", LaserScan, timeout=10.0)
        deadline = rospy.Time.now().to_sec() + 15.0
        rate = rospy.Rate(10.0)
        while rospy.Time.now().to_sec() < deadline:
            with self.lock:
                world_ready = self.health_valid_count > 0
                transaction_ready = (
                    self.method == "fixed_teb" or self.transaction_activated_count > 0
                )
            if world_ready and transaction_ready:
                return
            rate.sleep()
        raise RuntimeError("V2-04D runtime readiness timeout")

    def _sample(self, goal_reached=False):
        with self.lock:
            if self.odom is None or self.scan is None:
                return None
            pose = self.odom.pose.pose
            twist = self.odom.twist.twist
            goal = self.scene["goal"]
            velocity = float(twist.linear.x)
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
                "clearance_m": _footprint_clearance(self.scan),
                "goal_distance_m": math.hypot(
                    goal["x_m"] - pose.position.x, goal["y_m"] - pose.position.y
                ),
                "collision": self.contact_count > 0,
                "goal_reached": bool(goal_reached),
                "contact_count": self.contact_count,
                "topology_id": self.active_anchor or "transaction_pending",
                "global_replan_count": self.global_replan_count,
                "recovery_count": 0,
                "gear": "REVERSE" if velocity < -0.01 else (
                    "FORWARD" if velocity > 0.01 else "NEUTRAL"
                ),
                "predicted_ttc_s": self.predicted_ttc_s,
            }

    def run(self):
        if self.scene["split"] != "validation":
            raise RuntimeError("V2-04D runner refuses a non-validation split")
        self._wait_ready()
        goal_data = self.scene["goal"]
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = goal_data["x_m"]
        goal.target_pose.pose.position.y = goal_data["y_m"]
        goal.target_pose.pose.orientation.z = math.sin(goal_data["yaw_rad"] / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(goal_data["yaw_rad"] / 2.0)
        self.move_base.send_goal(goal)
        rows = []
        failure = ""
        terminal_state = GoalStatus.PENDING
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
            if final is not None and (not rows or final["stamp_s"] > rows[-1]["stamp_s"]):
                rows.append(final)
        self.move_base.cancel_all_goals()
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
        evaluation = evaluate_v2_episode(self.instance, rows, trace_sha256(trace_path))
        if self.tracker_message_count <= 0 or self.health_valid_count <= 0:
            ttc_status = "TRACKER_INVALID"
        elif self.finite_ttc_sample_count > 0:
            ttc_status = "OBSERVED_CONFLICT"
        else:
            ttc_status = "NO_CONFLICT_IN_HORIZON"
        typed_expected = self.method != "fixed_teb"
        typed_valid = bool(
            not typed_expected or (
                self.transaction_activated_count > 0
                and not self.transaction_training_used
                and self.transaction_backends == {"simulation_teb_dynamic_reconfigure"}
            )
        )
        evaluation.update({
            "stage": "V2-04D",
            "method": self.method,
            "episode_timeout_s": self.timeout_s,
            "tracker_message_count": self.tracker_message_count,
            "tracker_health_valid_fraction": (
                float(self.health_valid_count) / self.health_message_count
                if self.health_message_count else 0.0
            ),
            "finite_ttc_sample_count": self.finite_ttc_sample_count,
            "ttc_status": ttc_status,
            "context_message_count": self.context_message_count,
            "context_valid_count": self.context_valid_count,
            "context_geometry_sequence": list(self.context_geometries),
            "context_geometry_switch_count": max(0, len(self.context_geometries) - 1),
            "transaction_message_count": self.transaction_message_count,
            "transaction_valid_count": self.transaction_valid_count,
            "transaction_activated_count": self.transaction_activated_count,
            "transaction_fault_count": self.transaction_fault_count,
            "transaction_backends": sorted(self.transaction_backends),
            "active_anchor_sequence": list(self.active_anchors),
            "active_anchor_switch_count": max(0, len(self.active_anchors) - 1),
            "typed_transaction_expected": typed_expected,
            "typed_transaction_valid": typed_valid,
            "training_used": self.transaction_training_used,
            "runtime_policy_manifest_access": False,
            "runtime_scene_labels_available": False,
            "experiment_manager_validation_manifest_access": True,
            "runner_fault_reason": failure,
        })
        (self.output_dir / "evaluation.yaml").write_text(
            yaml.safe_dump(evaluation, sort_keys=False), encoding="utf-8"
        )
        return evaluation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--instance", required=True)
    parser.add_argument("--method", choices=METHODS, required=True)
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    output = Path(args.output_dir).resolve()
    output.relative_to((WORKSPACE / "artifacts/v2/validation/v2_04d").resolve())
    instance = yaml.safe_load(Path(args.instance).read_text(encoding="utf-8"))
    rospy.init_node("v2_04d_validation_episode")
    report = ValidationEpisode(instance, args.method, output).run()
    print(yaml.safe_dump(report, sort_keys=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
