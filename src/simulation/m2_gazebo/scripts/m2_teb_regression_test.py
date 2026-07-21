#!/usr/bin/env python3
"""Machine-readable fixed-TEB navigation matrix for the M2 simulation."""

import math
import os
import sys
import time

import actionlib
import rospy
import yaml
from actionlib_msgs.msg import GoalStatus
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry, Path
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty
from std_srvs.srv import Empty as EmptyService


STATUS_NAMES = {
    GoalStatus.PENDING: "PENDING", GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.PREEMPTED: "PREEMPTED", GoalStatus.SUCCEEDED: "SUCCEEDED",
    GoalStatus.ABORTED: "ABORTED", GoalStatus.REJECTED: "REJECTED",
    GoalStatus.PREEMPTING: "PREEMPTING", GoalStatus.RECALLING: "RECALLING",
    GoalStatus.RECALLED: "RECALLED", GoalStatus.LOST: "LOST",
}


def yaw_of(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


def angle_delta(after, before):
    return math.atan2(math.sin(after - before), math.cos(after - before))


class TebRegression:
    def __init__(self):
        self.seed = rospy.get_param("~seed", 42)
        self.report_path = rospy.get_param("~report_path", "/tmp/m2_fixed_teb_regression.yaml")
        self.timeout = rospy.get_param("~goal_timeout_s", 45.0)
        self.results = []
        self.odom = None
        self.scan = None
        self.path_length = 0.0
        self.previous_xy = None
        self.collecting = False
        self.minimum_scan = float("inf")
        self.global_plan_received = False
        self.local_plan_received = False
        self.last_cmd_stamp = None
        self.max_cmd_gap = 0.0
        self.cmd_count = 0
        self.planner_errors = []
        self.control_deadline_misses = []
        self.reset_odom_pub = rospy.Publisher("/m2_driver/reset_odom", Empty, queue_size=1)
        self.brake = rospy.Publisher("/m2_driver/brake_set", Bool, queue_size=1)
        self.cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom, queue_size=50)
        rospy.Subscriber("/scan", LaserScan, self._scan, queue_size=5)
        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel, queue_size=20)
        rospy.Subscriber("/rosout_agg", Log, self._rosout, queue_size=50)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/global_plan", Path, self._global_plan, queue_size=2)
        rospy.Subscriber("/move_base/TebLocalPlannerROS/local_plan", Path, self._local_plan, queue_size=2)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=15.0)
        rospy.wait_for_service("/move_base/clear_costmaps", timeout=20.0)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        self.clear_costmaps = rospy.ServiceProxy("/move_base/clear_costmaps", EmptyService)
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        if not self.client.wait_for_server(rospy.Duration(20.0)):
            raise RuntimeError("move_base action server unavailable")
        rospy.wait_for_message("/odom", Odometry, timeout=10.0)
        rospy.wait_for_message("/scan", LaserScan, timeout=10.0)

    def _odom(self, message):
        if self.collecting and self.previous_xy is not None:
            current = (message.pose.pose.position.x, message.pose.pose.position.y)
            step = math.hypot(current[0] - self.previous_xy[0], current[1] - self.previous_xy[1])
            if step < 0.25:
                self.path_length += step
            self.previous_xy = current
        self.odom = message

    def _scan(self, message):
        self.scan = message
        if self.collecting:
            finite = [value for value in message.ranges if math.isfinite(value)]
            if finite:
                self.minimum_scan = min(self.minimum_scan, min(finite))

    def _global_plan(self, message):
        if message.poses:
            self.global_plan_received = True

    def _local_plan(self, message):
        if message.poses:
            self.local_plan_received = True

    def _cmd_vel(self, _message):
        if not self.collecting:
            return
        now = rospy.Time.now().to_sec()
        if self.last_cmd_stamp is not None:
            self.max_cmd_gap = max(self.max_cmd_gap, now - self.last_cmd_stamp)
        self.last_cmd_stamp = now
        self.cmd_count += 1

    def _rosout(self, message):
        if not self.collecting or "move_base" not in message.name:
            return
        if message.level >= Log.ERROR:
            self.planner_errors.append(message.msg)
        if "Control loop missed its desired rate" in message.msg:
            self.control_deadline_misses.append(message.msg)

    @staticmethod
    def assert_limit(condition, message):
        if not condition:
            raise AssertionError(message)

    def place_model(self, name, x, y, z, yaw=0.0):
        state = ModelState(model_name=name, reference_frame="world")
        state.pose.position.x, state.pose.position.y, state.pose.position.z = x, y, z
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)
        response = self.set_model_state(state)
        self.assert_limit(response.success, response.status_message)

    def reset_robot(self):
        self.client.cancel_all_goals()
        self.brake.publish(Bool(data=False))
        self.cmd.publish(Twist())
        self.place_model("autolabor_m2", 0.0, 0.0, 0.02)
        for _ in range(3):
            self.reset_odom_pub.publish(Empty())
            rospy.sleep(0.05)
        self.clear_costmaps()
        rospy.sleep(1.0)
        self.assert_limit(math.hypot(self.odom.pose.pose.position.x,
                                     self.odom.pose.pose.position.y) < 0.05,
                          "robot reset failed")

    def layout_clear(self):
        self.place_model("front_box", 100.0, 0.0, 0.5)
        self.place_model("left_wall", 100.0, 10.0, 0.5)
        self.place_model("right_wall", 100.0, -10.0, 0.5)

    def layout_obstacle(self):
        self.layout_clear()
        self.place_model("front_box", 2.5, 0.0, 0.5)

    def layout_corridor(self):
        self.layout_clear()
        # Wall thickness is 0.2 m: inner faces at y=+/-0.9, corridor width 1.8 m.
        self.place_model("left_wall", 3.0, 1.0, 0.5)
        self.place_model("right_wall", 3.0, -1.0, 0.5)

    def send_goal(self, name, x, y, yaw, layout, minimum_clearance=0.20):
        layout()
        self.reset_robot()
        self.path_length = 0.0
        self.previous_xy = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y)
        self.minimum_scan = float("inf")
        self.global_plan_received = False
        self.local_plan_received = False
        self.last_cmd_stamp = None
        self.max_cmd_gap = 0.0
        self.cmd_count = 0
        self.planner_errors = []
        self.control_deadline_misses = []
        self.collecting = True
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        started = rospy.Time.now().to_sec()
        self.client.send_goal(goal)
        finished = self.client.wait_for_result(rospy.Duration(self.timeout))
        elapsed = rospy.Time.now().to_sec() - started
        state = self.client.get_state()
        if not finished:
            self.client.cancel_goal()
        self.collecting = False
        self.cmd.publish(Twist())
        final_x, final_y = self.odom.pose.pose.position.x, self.odom.pose.pose.position.y
        position_error = math.hypot(final_x - x, final_y - y)
        yaw_error = abs(angle_delta(yaw_of(self.odom.pose.pose.orientation), yaw))
        metrics = {
            "goal": {"x_m": x, "y_m": y, "yaw_rad": yaw},
            "action_state": state,
            "action_state_name": STATUS_NAMES.get(state, "UNKNOWN"),
            "elapsed_s": elapsed,
            "final_position_error_m": position_error,
            "final_yaw_error_rad": yaw_error,
            "path_length_m": self.path_length,
            "minimum_scan_range_m": self.minimum_scan if math.isfinite(self.minimum_scan) else None,
            "cmd_vel_message_count": self.cmd_count,
            "max_cmd_vel_gap_s": self.max_cmd_gap,
            "planner_error_count": len(self.planner_errors),
            "control_deadline_miss_count": len(self.control_deadline_misses),
            "planner_errors": self.planner_errors,
            "global_plan_received": self.global_plan_received,
            "local_plan_received": self.local_plan_received,
        }
        thresholds = {
            "required_action_state": GoalStatus.SUCCEEDED,
            "max_elapsed_s": self.timeout,
            "max_position_error_m": 0.30,
            "max_yaw_error_rad": 0.20,
            "minimum_scan_range_m": minimum_clearance,
            "max_cmd_vel_gap_s": 0.50,
            "max_planner_error_count": 0,
            "max_control_deadline_miss_count": 0,
        }
        self.assert_limit(finished and state == GoalStatus.SUCCEEDED,
                          "{} did not succeed: {}".format(name, metrics["action_state_name"]))
        self.assert_limit(position_error <= 0.30, "position error {:.3f}".format(position_error))
        self.assert_limit(yaw_error <= 0.20, "yaw error {:.3f}".format(yaw_error))
        self.assert_limit(self.minimum_scan >= minimum_clearance,
                          "scan clearance {:.3f}".format(self.minimum_scan))
        self.assert_limit(self.global_plan_received and self.local_plan_received, "TEB plans missing")
        self.assert_limit(self.cmd_count > 0 and self.max_cmd_gap <= 0.50,
                          "unstable cmd_vel gap {:.3f} s".format(self.max_cmd_gap))
        self.assert_limit(not self.planner_errors,
                          "move_base errors: {}".format(self.planner_errors))
        self.assert_limit(not self.control_deadline_misses,
                          "move_base controller deadline miss")
        return metrics, thresholds

    def record(self, name, function):
        result = {"name": name, "passed": False, "metrics": {}, "thresholds": {}, "message": ""}
        try:
            result["metrics"], result["thresholds"] = function()
            result["passed"] = True
            result["message"] = "passed"
            rospy.loginfo("Fixed TEB regression %s PASSED", name)
        except Exception as error:
            result["message"] = "{}: {}".format(type(error).__name__, error)
            rospy.logerr("Fixed TEB regression %s FAILED: %s", name, result["message"])
        self.results.append(result)

    def write_report(self):
        report = {
            "schema_version": 1,
            "suite": "m2_fixed_teb_regression",
            "simulation_only": True,
            "rl_enabled": False,
            "seed": self.seed,
            "passed": all(item["passed"] for item in self.results),
            "summary": {"passed": sum(item["passed"] for item in self.results),
                        "failed": sum(not item["passed"] for item in self.results),
                        "total": len(self.results)},
            "cases": self.results,
        }
        directory = os.path.dirname(self.report_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(self.report_path, "w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
        rospy.loginfo("Fixed TEB machine-readable report: %s", self.report_path)
        return report["passed"]


def main():
    rospy.init_node("m2_teb_regression_test")
    test = TebRegression()
    cases = [
        ("straight_goal", lambda: test.send_goal("straight_goal", 4.0, 0.0, 0.0, test.layout_clear)),
        ("left_turn_goal", lambda: test.send_goal("left_turn_goal", 3.0, 3.0, math.pi / 2.0, test.layout_clear)),
        ("right_turn_goal", lambda: test.send_goal("right_turn_goal", 3.0, -3.0, -math.pi / 2.0, test.layout_clear)),
        ("single_obstacle_detour", lambda: test.send_goal("single_obstacle_detour", 5.0, 0.0, 0.0, test.layout_obstacle, 0.25)),
        ("narrow_corridor", lambda: test.send_goal("narrow_corridor", 5.0, 0.0, 0.0, test.layout_corridor, 0.45)),
    ]
    try:
        for name, function in cases:
            rospy.loginfo("Fixed TEB quantitative regression: %s", name)
            test.record(name, function)
        passed = test.write_report()
        return 0 if passed else 1
    finally:
        test.client.cancel_all_goals()
        test.cmd.publish(Twist())
        test.brake.publish(Bool(data=True))


if __name__ == "__main__":
    sys.exit(main())
