#!/usr/bin/env python3
"""Quantitative, simulation-only M2 chassis regression with YAML results."""

import math
import os
import sys
import time
from collections import OrderedDict

import rospy
import tf2_ros
import yaml
from autolabor_canbus_driver.srv import ChassisParameterServer
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Empty


def yaw_of(orientation):
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2),
    )


def roll_pitch_of(orientation):
    sinr = 2.0 * (orientation.w * orientation.x + orientation.y * orientation.z)
    cosr = 1.0 - 2.0 * (orientation.x ** 2 + orientation.y ** 2)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (orientation.w * orientation.y - orientation.z * orientation.x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


def angle_delta(after, before):
    return math.atan2(math.sin(after - before), math.cos(after - before))


class Regression:
    def __init__(self):
        self.seed = rospy.get_param("~seed", 42)
        self.report_path = rospy.get_param("~report_path", "/tmp/m2_chassis_regression.yaml")
        self.results = []
        self.latest_odom = None
        self.latest_scan = None
        self.latest_models = None
        self.cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.brake = rospy.Publisher("/m2_driver/brake_set", Bool, queue_size=1)
        self.reset_odom_pub = rospy.Publisher("/m2_driver/reset_odom", Empty, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=20)
        rospy.Subscriber("/scan", LaserScan, self._scan_callback, queue_size=2)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self._models_callback, queue_size=2)
        rospy.wait_for_service("/gazebo/set_model_state", timeout=15.0)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def _odom_callback(self, message):
        self.latest_odom = message

    def _scan_callback(self, message):
        self.latest_scan = message

    def _models_callback(self, message):
        self.latest_models = message

    def wait_ready(self):
        rospy.wait_for_message("/odom", Odometry, timeout=10.0)
        rospy.wait_for_message("/scan", LaserScan, timeout=10.0)
        rospy.wait_for_message("/joint_states", JointState, timeout=10.0)

    @staticmethod
    def assert_limit(condition, message):
        if not condition:
            raise AssertionError(message)

    def record(self, name, function):
        started = rospy.Time.now().to_sec()
        result = OrderedDict(name=name, passed=False, metrics={}, thresholds={}, message="")
        try:
            metrics, thresholds = function()
            result["metrics"] = metrics
            result["thresholds"] = thresholds
            result["passed"] = True
            result["message"] = "passed"
        except Exception as error:  # Keep all cases machine-readable.
            result["message"] = "{}: {}".format(type(error).__name__, error)
            rospy.logerr("M2 chassis regression %s FAILED: %s", name, result["message"])
        result["duration_sim_s"] = max(0.0, rospy.Time.now().to_sec() - started)
        self.results.append(dict(result))
        if result["passed"]:
            rospy.loginfo("M2 chassis regression %s PASSED", name)

    def model_pose(self, name="autolabor_m2"):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.latest_models and name in self.latest_models.name:
                return self.latest_models.pose[self.latest_models.name.index(name)]
            rospy.sleep(0.02)
        raise AssertionError("model pose unavailable: {}".format(name))

    def place_model(self, name, x, y, z, yaw=0.0):
        state = ModelState(model_name=name, reference_frame="world")
        state.pose.position.x = x
        state.pose.position.y = y
        state.pose.position.z = z
        state.pose.orientation.z = math.sin(yaw / 2.0)
        state.pose.orientation.w = math.cos(yaw / 2.0)
        response = self.set_model_state(state)
        self.assert_limit(response.success, response.status_message)

    def reset_robot(self):
        self.brake.publish(Bool(data=False))
        self.cmd.publish(Twist())
        self.place_model("autolabor_m2", 0.0, 0.0, 0.02)
        for _ in range(3):
            self.reset_odom_pub.publish(Empty())
            rospy.sleep(0.04)
        rospy.sleep(0.15)
        odom = self.latest_odom
        self.assert_limit(odom is not None, "odom unavailable after reset")
        self.assert_limit(math.hypot(odom.pose.pose.position.x, odom.pose.pose.position.y) < 0.03,
                          "odom reset error")
        return odom

    def publish_for_sim_time(self, linear, angular, duration):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(30)
        while rospy.Time.now().to_sec() - start < duration and not rospy.is_shutdown():
            self.cmd.publish(message)
            rate.sleep()
        self.cmd.publish(Twist())
        rospy.sleep(0.08)

    def drive_distance(self, distance, speed):
        start = self.latest_odom
        sx, sy = start.pose.pose.position.x, start.pose.pose.position.y
        message = Twist()
        message.linear.x = speed
        rate = rospy.Rate(30)
        deadline = rospy.Time.now().to_sec() + abs(distance / speed) * 1.4 + 2.0
        while not rospy.is_shutdown():
            odom = self.latest_odom
            travelled = math.hypot(odom.pose.pose.position.x - sx, odom.pose.pose.position.y - sy)
            if travelled >= abs(distance) or rospy.Time.now().to_sec() > deadline:
                break
            self.cmd.publish(message)
            rate.sleep()
        self.cmd.publish(Twist())
        rospy.sleep(0.08)
        return self.latest_odom

    def interfaces_and_reset(self):
        self.wait_ready()
        odom = self.reset_robot()
        joints = rospy.wait_for_message("/joint_states", JointState, timeout=3.0)
        rospy.wait_for_service("/m2_driver/chassis_parameter", timeout=3.0)
        parameters = rospy.ServiceProxy("/m2_driver/chassis_parameter", ChassisParameterServer)()
        required = {"front_left_steer_joint", "front_right_steer_joint",
                    "rear_left_wheel_joint", "rear_right_wheel_joint"}
        self.assert_limit(required.issubset(set(joints.name)), "required joints missing")
        self.assert_limit(parameters.success, "candidate parameter query failed")
        return ({"reset_xy_error_m": math.hypot(odom.pose.pose.position.x, odom.pose.pose.position.y),
                 "joint_count": len(joints.name), "parameter_query": parameters.success},
                {"max_reset_xy_error_m": 0.03, "minimum_joint_count": 6})

    def static_stability(self):
        self.reset_robot()
        first = self.model_pose()
        start = rospy.Time.now().to_sec()
        max_roll = max_pitch = 0.0
        while rospy.Time.now().to_sec() - start < 3.0:
            pose = self.model_pose()
            roll, pitch = roll_pitch_of(pose.orientation)
            max_roll, max_pitch = max(max_roll, abs(roll)), max(max_pitch, abs(pitch))
            rospy.sleep(0.05)
        last = self.model_pose()
        drift = math.hypot(last.position.x - first.position.x, last.position.y - first.position.y)
        self.assert_limit(drift <= 0.01, "static drift {:.4f} m".format(drift))
        self.assert_limit(max_roll <= 0.02 and max_pitch <= 0.02, "model tilted")
        return ({"duration_s": 3.0, "xy_drift_m": drift,
                 "max_abs_roll_rad": max_roll, "max_abs_pitch_rad": max_pitch},
                {"max_xy_drift_m": 0.01, "max_abs_roll_pitch_rad": 0.02})

    def straight_case(self, target):
        self.place_model("scan_target", 100.0, 0.0, 0.75)
        self.reset_robot()
        final = self.drive_distance(target, 1.0)
        x, y = final.pose.pose.position.x, final.pose.pose.position.y
        yaw = yaw_of(final.pose.pose.orientation)
        error = abs(x - target)
        self.assert_limit(error <= 0.08, "distance error {:.3f}".format(error))
        self.assert_limit(abs(y) <= 0.03 and abs(yaw) <= 0.02, "straight drift")
        return ({"target_m": target, "final_x_m": x, "distance_error_m": error,
                 "lateral_error_m": y, "yaw_error_rad": yaw},
                {"max_distance_error_m": 0.08, "max_lateral_error_m": 0.03,
                 "max_yaw_error_rad": 0.02})

    def low_speed_reverse(self):
        self.reset_robot()
        self.publish_for_sim_time(-0.20, 0.0, 5.0)
        odom = self.latest_odom
        error = abs(odom.pose.pose.position.x + 1.0)
        self.assert_limit(error <= 0.06, "reverse distance error {:.3f}".format(error))
        self.assert_limit(abs(odom.pose.pose.position.y) <= 0.03, "reverse lateral drift")
        return ({"speed_mps": -0.20, "duration_s": 5.0,
                 "final_x_m": odom.pose.pose.position.x, "distance_error_m": error},
                {"max_distance_error_m": 0.06, "max_lateral_error_m": 0.03})

    def circle_case(self, direction):
        self.reset_robot()
        speed, yaw_rate = 0.60, direction * 0.40
        radius = abs(speed / yaw_rate)
        duration = 2.0 * math.pi / abs(yaw_rate)
        self.publish_for_sim_time(speed, yaw_rate, duration)
        odom = self.latest_odom
        closure = math.hypot(odom.pose.pose.position.x, odom.pose.pose.position.y)
        yaw_error = abs(angle_delta(yaw_of(odom.pose.pose.orientation), 0.0))
        self.assert_limit(closure <= 0.18, "circle closure {:.3f}".format(closure))
        self.assert_limit(yaw_error <= 0.12, "circle yaw closure {:.3f}".format(yaw_error))
        return ({"direction": "left" if direction > 0 else "right", "command_radius_m": radius,
                 "duration_s": duration, "closure_error_m": closure,
                 "yaw_closure_error_rad": yaw_error},
                {"max_closure_error_m": 0.18, "max_yaw_closure_error_rad": 0.12})

    def braking_response(self):
        self.reset_robot()
        command = Twist()
        command.linear.x = 1.0
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(30)
        while rospy.Time.now().to_sec() - start < 1.0:
            self.cmd.publish(command)
            rate.sleep()
        before = self.latest_odom
        stop_time = rospy.Time.now().to_sec()
        self.cmd.publish(Twist())
        deadline = stop_time + 0.5
        stopped = None
        while rospy.Time.now().to_sec() < deadline:
            odom = self.latest_odom
            if abs(odom.twist.twist.linear.x) < 0.01:
                stopped = odom
                break
            rospy.sleep(0.005)
        self.assert_limit(stopped is not None, "did not stop within 0.5 s")
        response = rospy.Time.now().to_sec() - stop_time
        distance = math.hypot(stopped.pose.pose.position.x - before.pose.pose.position.x,
                              stopped.pose.pose.position.y - before.pose.pose.position.y)
        self.assert_limit(response <= 0.08, "stop response {:.3f} s".format(response))
        self.assert_limit(distance <= 0.05, "stop distance {:.3f} m".format(distance))
        return ({"initial_speed_mps": 1.0, "response_time_s": max(0.0, response),
                 "braking_distance_m": distance},
                {"max_response_time_s": 0.08, "max_braking_distance_m": 0.05})

    def scan_accuracy(self):
        self.reset_robot()
        self.place_model("scan_target", 5.0, 0.0, 0.75)
        rospy.sleep(0.4)
        scan = self.latest_scan
        center = min(range(len(scan.ranges)), key=lambda i: abs(scan.angle_min + i * scan.angle_increment))
        window = [value for value in scan.ranges[center - 2:center + 3] if math.isfinite(value)]
        measured = sum(window) / len(window)
        expected = 4.75
        error = abs(measured - expected)
        self.assert_limit(error <= 0.06, "scan error {:.3f} m".format(error))
        return ({"expected_range_m": expected, "measured_range_m": measured,
                 "absolute_error_m": error, "sample_count": len(scan.ranges)},
                {"max_absolute_error_m": 0.06, "expected_sample_count": 720})

    def tf_integrity(self):
        buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        listener = tf2_ros.TransformListener(buffer)
        rospy.sleep(0.3)
        transform = buffer.lookup_transform("odom", "base_link", rospy.Time(0), rospy.Duration(3.0))
        frames = yaml.safe_load(buffer.all_frames_as_yaml()) or {}
        parent = {name: data.get("parent") for name, data in frames.items()}
        for frame in parent:
            seen, current = set(), frame
            while current in parent and parent[current]:
                self.assert_limit(current not in seen, "TF cycle at {}".format(current))
                seen.add(current)
                current = parent[current]
        age = max(0.0, (rospy.Time.now() - transform.header.stamp).to_sec())
        self.assert_limit(age <= 0.15, "TF timestamp age {:.3f}".format(age))
        self.assert_limit(parent.get("base_link") == "odom", "base_link parent is not odom")
        return ({"frame_count": len(frames), "base_link_parent": parent.get("base_link"),
                 "latest_transform_age_s": age, "cycle_found": False},
                {"max_transform_age_s": 0.15, "required_base_link_parent": "odom"})

    def reset_repeatability(self):
        samples = []
        for _ in range(3):
            odom = self.reset_robot()
            scan = self.latest_scan
            center = len(scan.ranges) // 2
            samples.append((odom.pose.pose.position.x, odom.pose.pose.position.y,
                            yaw_of(odom.pose.pose.orientation), scan.ranges[center]))
        spans = [max(values) - min(values) for values in zip(*samples)]
        self.assert_limit(max(spans[:3]) <= 0.01, "initial pose is not repeatable")
        self.assert_limit(spans[3] <= 0.03, "initial scan is not repeatable")
        return ({"seed": self.seed, "reset_count": len(samples), "x_span_m": spans[0],
                 "y_span_m": spans[1], "yaw_span_rad": spans[2], "center_scan_span_m": spans[3]},
                {"max_pose_span": 0.01, "max_scan_span_m": 0.03})

    def write_report(self):
        report = {
            "schema_version": 1,
            "suite": "m2_chassis_regression",
            "simulation_only": True,
            "move_base_started": False,
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
        rospy.loginfo("M2 chassis machine-readable report: %s", self.report_path)
        return report["passed"]


def main():
    rospy.init_node("m2_regression_test")
    test = Regression()
    mode = rospy.get_param("~mode", "all")
    cases = OrderedDict([
        ("spawn_and_reset", test.interfaces_and_reset),
        ("static_stability", test.static_stability),
        ("straight_5m", lambda: test.straight_case(5.0)),
        ("straight_10m", lambda: test.straight_case(10.0)),
        ("low_speed_reverse", test.low_speed_reverse),
        ("left_fixed_radius_circle", lambda: test.circle_case(1)),
        ("right_fixed_radius_circle", lambda: test.circle_case(-1)),
        ("stop_response", test.braking_response),
        ("scan_fixed_obstacle", test.scan_accuracy),
        ("tf_integrity", test.tf_integrity),
        ("seed_reset_repeatability", test.reset_repeatability),
    ])
    try:
        test.wait_ready()
        selected = cases.items() if mode == "all" else [(mode, cases[mode])]
        for name, function in selected:
            rospy.loginfo("M2 quantitative chassis regression: %s", name)
            test.record(name, function)
        passed = test.write_report()
        rospy.loginfo("M2 quantitative chassis regression %s", "PASSED" if passed else "FAILED")
        return 0 if passed else 1
    finally:
        test.cmd.publish(Twist())
        test.brake.publish(Bool(data=True))


if __name__ == "__main__":
    sys.exit(main())
