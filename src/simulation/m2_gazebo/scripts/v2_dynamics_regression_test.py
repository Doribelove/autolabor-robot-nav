#!/usr/bin/env python3
"""Simulation-only V2 actuator, brake, delay, reverse, and sensor regression."""

import math
import os
import sys
import time
import unittest
from collections import OrderedDict, deque

import rospy
import yaml
from gazebo_msgs.msg import ContactsState, ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, Float64


class V2DynamicsRegression:
    def __init__(self):
        self.seed = int(rospy.get_param("~seed", 42))
        self.report_path = rospy.get_param(
            "~report_path", "/tmp/v2_dynamics_regression.yaml"
        )
        self.results = []
        self.latest_odom = None
        self.odom_history = deque(maxlen=3000)
        self.activation_latencies = []
        self.scan_ages = []
        self.scan_ray_counts = []
        self.contact_messages = 0
        self.cmd = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.brake = rospy.Publisher("/m2_driver/brake_set", Bool, queue_size=2)
        self.emergency = rospy.Publisher("/m2_driver/emergency_stop", Bool, queue_size=2)
        self.reset_odom = rospy.Publisher("/m2_driver/reset_odom", Empty, queue_size=2)
        rospy.Subscriber("/odom", Odometry, self._odom, queue_size=100)
        rospy.Subscriber(
            "/m2_driver/command_activation_latency", Float64, self._latency, queue_size=100
        )
        rospy.Subscriber("/scan", LaserScan, self._scan, queue_size=20)
        rospy.Subscriber(
            "/m2_gazebo/contacts", ContactsState, self._contacts, queue_size=20
        )
        rospy.wait_for_service("/gazebo/set_model_state", timeout=20.0)
        self.set_model_state = rospy.ServiceProxy(
            "/gazebo/set_model_state", SetModelState
        )

    def _odom(self, message):
        self.latest_odom = message
        self.odom_history.append(message)

    def _latency(self, message):
        self.activation_latencies.append(float(message.data))

    def _scan(self, message):
        self.scan_ages.append(max(0.0, (rospy.Time.now() - message.header.stamp).to_sec()))
        self.scan_ray_counts.append(len(message.ranges))

    def _contacts(self, _message):
        self.contact_messages += 1

    @staticmethod
    def require(condition, message):
        if not condition:
            raise AssertionError(message)

    def record(self, name, function):
        result = OrderedDict(name=name, passed=False, metrics={}, thresholds={}, message="")
        started = rospy.Time.now().to_sec()
        try:
            metrics, thresholds = function()
            result["metrics"] = metrics
            result["thresholds"] = thresholds
            result["passed"] = True
            result["message"] = "passed"
        except Exception as exc:
            result["message"] = "{}: {}".format(type(exc).__name__, exc)
            rospy.logerr("V2 dynamics regression %s FAILED: %s", name, result["message"])
        result["duration_sim_s"] = max(0.0, rospy.Time.now().to_sec() - started)
        self.results.append(dict(result))

    def wait_ready(self):
        rospy.wait_for_message("/odom", Odometry, timeout=15.0)
        rospy.wait_for_message("/scan", LaserScan, timeout=15.0)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            topics = dict(rospy.get_published_topics())
            if topics.get("/m2_gazebo/contacts") == "gazebo_msgs/ContactsState":
                return
            rospy.sleep(0.02)
        raise AssertionError("contact topic is not published")

    def place_robot(self):
        state = ModelState(model_name="autolabor_m2", reference_frame="world")
        state.pose.position.z = 0.02
        state.pose.orientation.w = 1.0
        response = self.set_model_state(state)
        self.require(response.success, response.status_message)

    def reset_robot(self):
        self.brake.publish(Bool(data=False))
        self.emergency.publish(Bool(data=False))
        self.place_robot()
        for _ in range(3):
            self.reset_odom.publish(Empty())
            rospy.sleep(0.04)
        rospy.sleep(0.10)
        self.odom_history.clear()
        self.activation_latencies[:] = []
        self.require(self.latest_odom is not None, "odom unavailable after reset")

    def publish_command(self, linear, angular, duration, rate_hz=30):
        message = Twist()
        message.linear.x = linear
        message.angular.z = angular
        start = rospy.Time.now().to_sec()
        rate = rospy.Rate(rate_hz)
        while rospy.Time.now().to_sec() - start < duration and not rospy.is_shutdown():
            self.cmd.publish(message)
            rate.sleep()
        return start

    def straight(self):
        self.reset_robot()
        self.publish_command(1.0, 0.0, 4.0)
        odom = self.latest_odom
        x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
        speed = odom.twist.twist.linear.x
        self.require(speed >= 0.90, "steady speed {:.3f}".format(speed))
        self.require(abs(y) <= 0.05, "lateral error {:.3f}".format(y))
        self.require(2.8 <= x <= 4.1, "longitudinal response {:.3f}".format(x))
        return ({"duration_s": 4.0, "final_x_m": x, "lateral_error_m": y,
                 "steady_speed_mps": speed},
                {"minimum_steady_speed_mps": 0.90, "maximum_lateral_error_m": 0.05,
                 "final_x_range_m": [2.8, 4.1]})

    def circle(self):
        self.reset_robot()
        start = self.publish_command(0.60, 0.40, 4.0)
        steady = [message for message in self.odom_history
                  if message.header.stamp.to_sec() >= start + 3.0]
        self.require(len(steady) >= 10, "insufficient steady circle samples")
        mean_speed = sum(message.twist.twist.linear.x for message in steady) / len(steady)
        mean_yaw = sum(message.twist.twist.angular.z for message in steady) / len(steady)
        radius = abs(mean_speed / mean_yaw)
        self.require(abs(mean_yaw - 0.40) <= 0.06,
                     "steady yaw-rate error {:.3f}".format(abs(mean_yaw - 0.40)))
        self.require(abs(radius - 1.50) <= 0.20, "radius error {:.3f}".format(abs(radius - 1.50)))
        return ({"mean_steady_speed_mps": mean_speed, "mean_steady_yaw_rate_radps": mean_yaw,
                 "estimated_radius_m": radius},
                {"maximum_steady_yaw_rate_error_radps": 0.06,
                 "maximum_radius_error_m": 0.20})

    def braking(self):
        self.reset_robot()
        self.publish_command(1.0, 0.0, 2.2)
        before = self.latest_odom
        initial_speed = before.twist.twist.linear.x
        start_x, start_y = before.pose.pose.position.x, before.pose.pose.position.y
        self.require(initial_speed >= 0.90, "initial braking speed {:.3f}".format(initial_speed))
        trigger = rospy.Time.now().to_sec()
        self.brake.publish(Bool(data=True))
        command = Twist()
        command.linear.x = 1.0
        rate = rospy.Rate(50)
        stopped = None
        while rospy.Time.now().to_sec() - trigger < 1.0 and not rospy.is_shutdown():
            self.cmd.publish(command)
            if abs(self.latest_odom.twist.twist.linear.x) <= 0.02:
                stopped = self.latest_odom
                break
            rate.sleep()
        self.brake.publish(Bool(data=False))
        self.require(stopped is not None, "brake did not stop within 1.0 s")
        stop_time = rospy.Time.now().to_sec() - trigger
        distance = math.hypot(stopped.pose.pose.position.x - start_x,
                              stopped.pose.pose.position.y - start_y)
        self.require(0.10 <= distance <= 0.45,
                     "braking distance {:.3f} is not physical candidate".format(distance))
        return ({"initial_speed_mps": initial_speed, "stop_time_s": stop_time,
                 "stopping_distance_m": distance},
                {"maximum_stop_time_s": 1.0, "minimum_stopping_distance_m": 0.10,
                 "maximum_stopping_distance_m": 0.45})

    def reverse(self):
        self.reset_robot()
        self.publish_command(-0.20, 0.0, 4.0)
        odom = self.latest_odom
        speed = odom.twist.twist.linear.x
        x, y = odom.pose.pose.position.x, odom.pose.pose.position.y
        self.require(speed <= -0.16, "reverse steady speed {:.3f}".format(speed))
        self.require(x < -0.45, "reverse displacement {:.3f}".format(x))
        self.require(abs(y) <= 0.05, "reverse lateral error {:.3f}".format(y))
        return ({"steady_speed_mps": speed, "final_x_m": x, "lateral_error_m": y},
                {"minimum_abs_steady_speed_mps": 0.16,
                 "maximum_lateral_error_m": 0.05})

    def delay_and_repeatability(self):
        samples = []
        for _ in range(2):
            self.reset_robot()
            self.publish_command(0.4, 0.0, 0.8, rate_hz=10)
            rospy.sleep(0.20)
            observed = list(self.activation_latencies[:8])
            self.require(len(observed) >= 6, "insufficient activation latency samples")
            self.require(min(observed) >= 0.060 and max(observed) <= 0.120,
                         "activation latency range [{:.3f}, {:.3f}]".format(
                             min(observed), max(observed)))
            samples.append(observed)
        pair_count = min(len(samples[0]), len(samples[1]))
        maximum_pair_error = max(abs(samples[0][index] - samples[1][index])
                                 for index in range(pair_count))
        self.require(maximum_pair_error <= 0.025,
                     "same-seed delay repeatability error {:.3f}".format(maximum_pair_error))
        return ({"sample_count": pair_count, "minimum_observed_s": min(samples[0]),
                 "maximum_observed_s": max(samples[0]),
                 "maximum_repeat_error_s": maximum_pair_error},
                {"minimum_observed_s": 0.060, "maximum_observed_s": 0.120,
                 "scheduler_resolution_tolerance_s": 0.025,
                 "maximum_repeat_error_s": 0.025})

    def sensor_and_contact_contract(self):
        deadline = rospy.Time.now().to_sec() + 1.0
        while len(self.scan_ages) < 8 and rospy.Time.now().to_sec() < deadline:
            rospy.sleep(0.02)
        recent = self.scan_ages[-8:]
        self.require(len(recent) >= 5, "insufficient delayed scans")
        mean_age = sum(recent) / len(recent)
        self.require(0.045 <= mean_age <= 0.100,
                     "scan transport age {:.3f}".format(mean_age))
        self.require(set(self.scan_ray_counts[-8:]) == {720}, "LaserScan ray count drifted")
        topics = dict(rospy.get_published_topics())
        self.require(topics.get("/m2_gazebo/contacts") == "gazebo_msgs/ContactsState",
                     "contact collision topic/type drifted")
        return ({"mean_scan_age_s": mean_age, "ray_count": 720,
                 "contact_topic_type": topics["/m2_gazebo/contacts"],
                 "contact_message_count": self.contact_messages},
                {"minimum_mean_scan_age_s": 0.045, "maximum_mean_scan_age_s": 0.100,
                 "required_ray_count": 720})

    def write_report(self):
        report = {
            "schema_version": "2.0",
            "architecture_generation": "v2",
            "suite": "v2_02_dynamics_regression",
            "simulation_only": True,
            "formal_result": False,
            "runtime_ready": False,
            "training_started": False,
            "real_vehicle_use_forbidden": True,
            "seed": self.seed,
            "passed": all(item["passed"] for item in self.results),
            "summary": {
                "passed": sum(item["passed"] for item in self.results),
                "failed": sum(not item["passed"] for item in self.results),
                "total": len(self.results),
            },
            "cases": self.results,
        }
        directory = os.path.dirname(self.report_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        temporary = self.report_path + ".tmp"
        with open(temporary, "w", encoding="utf-8") as stream:
            yaml.safe_dump(report, stream, sort_keys=False, allow_unicode=True)
        os.replace(temporary, self.report_path)
        return report["passed"]


def execute_suite():
    test = V2DynamicsRegression()
    cases = OrderedDict([
        ("straight_response", test.straight),
        ("fixed_radius_circle", test.circle),
        ("nonzero_braking_distance", test.braking),
        ("low_speed_reverse", test.reverse),
        ("command_delay_repeatability", test.delay_and_repeatability),
        ("sensor_delay_and_contact", test.sensor_and_contact_contract),
    ])
    try:
        test.wait_ready()
        for name, function in cases.items():
            test.record(name, function)
        return 0 if test.write_report() else 1
    finally:
        test.cmd.publish(Twist())
        test.brake.publish(Bool(data=True))


class V2DynamicsRegressionRostest(unittest.TestCase):
    def test_regression_suite(self):
        if not rospy.core.is_initialized():
            rospy.init_node("v2_dynamics_regression_rostest", anonymous=True)
        self.assertEqual(
            execute_suite(), 0, "V2 dynamics regression report contains failures"
        )


def main():
    if any(argument.startswith("--gtest_output") for argument in sys.argv[1:]):
        import rostest
        rostest.rosrun(
            "m2_gazebo",
            "v2_dynamics_regression_rostest",
            V2DynamicsRegressionRostest,
        )
        return 0
    rospy.init_node("v2_dynamics_regression_test")
    return execute_suite()


if __name__ == "__main__":
    sys.exit(main())
