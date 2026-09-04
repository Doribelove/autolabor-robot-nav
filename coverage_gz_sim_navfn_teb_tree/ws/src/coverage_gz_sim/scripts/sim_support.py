#!/usr/bin/env python3
"""Publish deterministic readiness heartbeats required by coverage_manager."""

import json
import math
import threading

import rospy
from autolabor_coverage.srv import SetCoverageOwner, SetCoverageOwnerResponse
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String


class SimulationSupport:
    def __init__(self):
        self.lock = threading.RLock()
        self.owner_token = ""
        self.localization_pub = rospy.Publisher(
            "/fast_lio/localization_status", String, queue_size=2, latch=True
        )
        self.watchdog_pub = rospy.Publisher(
            "/nvidia_cmd_vel_watchdog/status", String, queue_size=2, latch=True
        )
        self.mode_pub = rospy.Publisher(
            "/fod_navigation_mode/status", String, queue_size=2, latch=True
        )
        self.pause_pub = rospy.Publisher(
            "/navigation_pause/paused", Bool, queue_size=2, latch=True
        )
        self.dual_lidar_pub = rospy.Publisher(
            "/avoidance/dual_lidar_active", Bool, queue_size=2, latch=True
        )
        self.scan_pub = rospy.Publisher("/scan", LaserScan, queue_size=2)
        self.owner_service = rospy.Service(
            "/navigation_goal/set_coverage_owner",
            SetCoverageOwner,
            self._owner_callback,
        )
        rospy.Timer(rospy.Duration(0.10), self._heartbeat)

    def _owner_callback(self, request):
        with self.lock:
            if request.claim:
                if not self.owner_token or self.owner_token == request.owner_token:
                    self.owner_token = request.owner_token
                    return SetCoverageOwnerResponse(
                        True, True, self.owner_token, "simulation owner claimed"
                    )
                return SetCoverageOwnerResponse(
                    False, True, self.owner_token, "another simulation owner is active"
                )
            if not self.owner_token or self.owner_token == request.owner_token:
                self.owner_token = ""
                return SetCoverageOwnerResponse(
                    True, False, "", "simulation owner released"
                )
            return SetCoverageOwnerResponse(
                False, True, self.owner_token, "owner token does not match"
            )

    def _heartbeat(self, _event):
        now = rospy.Time.now()
        self.localization_pub.publish(
            String(data="state=LOCALIZED; source=gazebo_truth; fitness=0")
        )
        self.watchdog_pub.publish(String(data=json.dumps({
            "motion_enabled": True,
            "max_linear_speed": 1.60,
            "max_angular_speed": 1.00,
            "source": "coverage_gz_sim",
        }, sort_keys=True)))
        self.mode_pub.publish(String(data=json.dumps({
            "state": "GPS_ACTIVE",
            "move_base_goals_allowed": True,
            "source": "coverage_gz_sim",
        }, sort_keys=True)))
        self.pause_pub.publish(Bool(data=False))
        self.dual_lidar_pub.publish(Bool(data=False))

        # This is a readiness heartbeat, not a simulated range sensor.  The
        # navigation costmaps use only the immutable static map in this lab.
        scan = LaserScan()
        scan.header.stamp = now
        scan.header.frame_id = "base_link"
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / 360.0
        scan.time_increment = 0.0
        scan.scan_time = 0.10
        scan.range_min = 0.10
        scan.range_max = 30.0
        scan.ranges = [float("inf")] * 361
        self.scan_pub.publish(scan)


if __name__ == "__main__":
    rospy.init_node("coverage_sim_support")
    SimulationSupport()
    rospy.spin()
