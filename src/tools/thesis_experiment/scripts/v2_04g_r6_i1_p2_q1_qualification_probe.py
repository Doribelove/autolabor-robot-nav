#!/usr/bin/env python3
"""Record one future P2-Q1 raw qualification trace in Gazebo.

This adapter is release-gated by the Q1 runner and is not invoked by the
offline-readiness stage.  It records raw samples only; pass/fail is recomputed
later by the independent replay module.
"""

import argparse
import math
import os
from pathlib import Path
import tempfile
import threading

import actionlib
import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from nav_world_model.risk_evidence import rectangular_footprint_clearance
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from teb_mode_manager.msg import ContextState, ParameterTransaction


STAGE = "V2-04G-P2-Q1"
DYNAMIC_NAMES = {
    ContextState.DYNAMIC_NONE: "NONE",
    ContextState.DYNAMIC_CROSSING: "CROSSING",
    ContextState.DYNAMIC_HEAD_ON: "HEAD_ON",
    ContextState.DYNAMIC_FOLLOW: "FOLLOW",
    ContextState.DYNAMIC_OVERTAKE_OR_YIELD: "OVERTAKE_OR_YIELD",
}


def atomic_yaml(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = yaml.safe_dump(value, sort_keys=False, allow_unicode=True).encode("utf-8")
    descriptor, temporary = tempfile.mkstemp(prefix=".{}-".format(path.name), dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class QualificationProbe:
    def __init__(self, arguments):
        self.arguments = arguments
        self.lock = threading.RLock()
        self.odom = None
        self.scan = None
        self.collision = False
        self.context_overlay = "NONE"
        self.executed_overlay = "NONE"
        self.emergency = False
        self.projection_applied = False
        self.transaction_valid = (
            arguments.probe != "high_speed_to_conflict_overlay_transition"
        )
        self.samples = []
        self.command = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.brake = rospy.Publisher("/m2_driver/brake_set", Bool, queue_size=2)
        rospy.Subscriber("/odom", Odometry, self._odom, queue_size=50)
        rospy.Subscriber("/scan", LaserScan, self._scan, queue_size=10)
        rospy.Subscriber("/m2_gazebo/contacts", ContactsState, self._contact, queue_size=10)
        rospy.Subscriber("/teb_mode_manager/context", ContextState, self._context, queue_size=10)
        rospy.Subscriber("/teb_rl_v2/action_trace", ParameterTransaction, self._transaction, queue_size=10)
        rospy.Subscriber(
            "/m2_driver/emergency_stop", Bool, self._emergency, queue_size=5
        )
        self.move_base = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

    def _odom(self, message):
        with self.lock:
            self.odom = message

    def _scan(self, message):
        with self.lock:
            self.scan = message

    def _contact(self, message):
        if message.states:
            with self.lock:
                self.collision = True

    def _context(self, message):
        if message.valid:
            with self.lock:
                self.context_overlay = DYNAMIC_NAMES.get(message.dynamic_overlay, "UNKNOWN")
                if message.transition_state in {
                    ContextState.TRANSITION_SAFE_OVERRIDE,
                    ContextState.TRANSITION_FAULTED,
                }:
                    self.emergency = True

    def _transaction(self, message):
        with self.lock:
            self.projection_applied = bool(message.projection_reason_mask)
            self.transaction_valid = bool(
                message.valid and message.activated and not message.fault_reason
            )
            if message.safety_reason_mask:
                self.emergency = True
            if message.valid and message.activated:
                self.executed_overlay = DYNAMIC_NAMES.get(message.dynamic_overlay, "UNKNOWN")

    def _emergency(self, message):
        if message.data:
            with self.lock:
                self.emergency = True

    def wait_ready(self):
        rospy.wait_for_message("/odom", Odometry, timeout=20.0)
        rospy.wait_for_message("/scan", LaserScan, timeout=20.0)

    def sample(self):
        with self.lock:
            if self.odom is None or self.scan is None:
                return
            pose = self.odom.pose.pose.position
            twist = self.odom.twist.twist
            clearance = rectangular_footprint_clearance(
                self.scan
            ).clipped_clearance_m
            self.samples.append(
                {
                    "stamp_s": float(rospy.Time.now().to_sec()),
                    "x_m": float(pose.x),
                    "y_m": float(pose.y),
                    "linear_velocity_mps": float(twist.linear.x),
                    "yaw_rate_radps": float(twist.angular.z),
                    "clearance_m": float(clearance),
                    "collision": bool(self.collision),
                    "dynamic_overlay": self.executed_overlay,
                    "conflict_triggered": self.context_overlay != "NONE",
                    "emergency": bool(self.emergency),
                    "projection_applied": bool(self.projection_applied),
                    "transaction_valid": bool(self.transaction_valid),
                }
            )

    def publish_for(self, linear, angular, duration_s, brake=False):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        self.brake.publish(Bool(data=brake))
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(20)
        while (
            rospy.Time.now().to_sec() - start < duration_s
            and not rospy.is_shutdown()
            and not self.collision
        ):
            self.command.publish(message)
            self.sample()
            rate.sleep()

    def run(self):
        self.wait_ready()
        speed = self.arguments.candidate_speed_mps
        if self.arguments.probe == "braking":
            self.publish_for(speed, 0.0, 3.0, brake=False)
            self.publish_for(0.0, 0.0, 3.0, brake=True)
            self.brake.publish(Bool(data=False))
        elif self.arguments.probe == "ackermann_circle":
            self.publish_for(speed, speed / 1.2, 6.0, brake=False)
            self.publish_for(0.0, 0.0, 1.0, brake=False)
        else:
            if not self.move_base.wait_for_server(rospy.Duration(20.0)):
                raise RuntimeError("move_base action server unavailable")
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "odom"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = self.arguments.goal_x_m
            goal.target_pose.pose.position.y = self.arguments.goal_y_m
            goal.target_pose.pose.orientation.z = math.sin(self.arguments.goal_yaw_rad / 2.0)
            goal.target_pose.pose.orientation.w = math.cos(self.arguments.goal_yaw_rad / 2.0)
            self.move_base.send_goal(goal)
            start = rospy.Time.now().to_sec()
            rate = rospy.Rate(20)
            while (
                rospy.Time.now().to_sec() - start < 30.0
                and not rospy.is_shutdown()
                and not self.collision
            ):
                self.sample()
                if (
                    any(row["conflict_triggered"] for row in self.samples)
                    and self.executed_overlay != "NONE"
                    and len(self.samples) >= 2
                ):
                    for _ in range(10):
                        self.sample()
                        rate.sleep()
                    break
                if self.move_base.get_state() in {
                    GoalStatus.ABORTED,
                    GoalStatus.REJECTED,
                    GoalStatus.RECALLED,
                    GoalStatus.LOST,
                }:
                    break
                rate.sleep()
            self.move_base.cancel_goal()
        self.command.publish(Twist())
        self.sample()
        if len(self.samples) < 2:
            raise RuntimeError("qualification raw trace is too short")
        identity = {
            "sequence": self.arguments.sequence,
            "row_type": "qualification_probe",
            "scene_id": self.arguments.scene_id,
            "seed": self.arguments.seed,
            "attempt": 1,
            "probe_type": self.arguments.probe,
            "candidate_speed_mps": speed,
        }
        atomic_yaml(
            self.arguments.output,
            {
                "schema_version": "1.0",
                "stage": STAGE,
                "identity": identity,
                "samples": self.samples,
            },
        )


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--probe", required=True, choices=[
        "braking", "high_speed_to_conflict_overlay_transition", "ackermann_circle"
    ])
    parser.add_argument("--candidate-speed-mps", type=float, required=True)
    parser.add_argument("--sequence", type=int, required=True)
    parser.add_argument("--scene-id", required=True)
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--goal-x-m", type=float, default=18.0)
    parser.add_argument("--goal-y-m", type=float, default=0.0)
    parser.add_argument("--goal-yaw-rad", type=float, default=0.0)
    parsed, _ = parser.parse_known_args()
    if parsed.candidate_speed_mps not in {1.25, 1.30, 1.35, 1.40}:
        parser.error("candidate speed is outside the frozen grid")
    return parsed


def main():
    parsed = arguments()
    rospy.init_node("p2_q1_qualification_probe", anonymous=False)
    probe = QualificationProbe(parsed)
    try:
        probe.run()
        return 0
    finally:
        probe.command.publish(Twist())
        probe.brake.publish(Bool(data=True))


if __name__ == "__main__":
    raise SystemExit(main())
