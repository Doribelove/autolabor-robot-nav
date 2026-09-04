#!/usr/bin/env python3
"""Online coverage-navigation monitor and machine-readable experiment logger."""

import csv
import json
import math
import os
import threading
from collections import deque

import rospy
from autolabor_coverage.msg import CoverageStatus, EnforcedPath
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


class CoverageMonitor:
    def __init__(self):
        self.result_dir = os.path.abspath(rospy.get_param("~result_dir"))
        self.minimum_radius = float(rospy.get_param("~minimum_turning_radius", 1.35))
        self.io_lock = threading.RLock()
        self.closing = False
        os.makedirs(self.result_dir, exist_ok=True)
        self.events = open(
            os.path.join(self.result_dir, "events.jsonl"), "a", encoding="utf-8"
        )
        self.samples_file = open(
            os.path.join(self.result_dir, "samples.csv"), "w", newline="", encoding="utf-8"
        )
        self.samples = csv.writer(self.samples_file)
        self.samples.writerow([
            "stamp", "state", "segment", "x", "y", "v_cmd", "omega_cmd",
            "command_radius", "v_actual", "omega_actual",
        ])
        self.state = ""
        self.segment = 0
        self.active = False
        self.pose = None
        self.actual = (0.0, 0.0)
        self.cmd = (0.0, 0.0)
        self.history = deque()
        self.last_no_progress_key = None
        self.curvature_samples = 0
        self.curvature_violations = 0
        self.path_updates = 0
        self.hybrid_updates = 0
        self.status_updates = 0
        rospy.Subscriber("/coverage/status", CoverageStatus, self._status, queue_size=10)
        rospy.Subscriber("/coverage/enforced_path", EnforcedPath, self._enforced, queue_size=10)
        rospy.Subscriber("/coverage/hybrid_transition_path", Path, self._hybrid, queue_size=10)
        rospy.Subscriber("/cmd_vel_sim", Twist, self._command, queue_size=20)
        rospy.Subscriber("/odom", Odometry, self._odom, queue_size=20)
        self.timer = rospy.Timer(rospy.Duration(0.10), self._sample)
        rospy.on_shutdown(self._finish)

    def _event(self, kind, **payload):
        with self.io_lock:
            if self.closing:
                return
            record = {"stamp": rospy.Time.now().to_sec(), "kind": kind}
            record.update(payload)
            self.events.write(json.dumps(record, ensure_ascii=False, sort_keys=True) + "\n")
            self.events.flush()

    def _status(self, message):
        self.status_updates += 1
        changed = message.state != self.state or int(message.current_segment) != self.segment
        self.state = message.state
        self.segment = int(message.current_segment)
        self.active = bool(message.active)
        if changed:
            self._event(
                "coverage_state",
                state=message.state,
                segment=self.segment,
                total=int(message.total_segments),
                detail=message.detail,
                region=message.current_region_name,
            )
            rospy.loginfo(
                "MONITOR state=%s segment=%d/%d region=%s detail=%s",
                message.state, self.segment, int(message.total_segments),
                message.current_region_name, message.detail,
            )

    def _enforced(self, message):
        self.path_updates += 1

    def _hybrid(self, message):
        self.hybrid_updates += 1
        if message.poses:
            self._event("hybrid_path", poses=len(message.poses))

    def _command(self, message):
        self.cmd = (float(message.linear.x), float(message.angular.z))
        v, omega = self.cmd
        if abs(v) > 0.01 and abs(omega) > 0.01:
            self.curvature_samples += 1
            radius = abs(v / omega)
            if radius + 1.0e-6 < self.minimum_radius:
                self.curvature_violations += 1
                if self.curvature_violations <= 5 or self.curvature_violations % 100 == 0:
                    self._event(
                        "command_curvature_violation", v=v, omega=omega, radius=radius
                    )
                    rospy.logwarn(
                        "MONITOR command radius %.3fm below %.3fm (v=%.3f omega=%.3f)",
                        radius, self.minimum_radius, v, omega,
                    )

    def _odom(self, message):
        self.pose = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
        )
        self.actual = (
            float(message.twist.twist.linear.x),
            float(message.twist.twist.angular.z),
        )

    def _sample(self, _event):
        with self.io_lock:
            if self.closing or self.pose is None:
                return
            stamp = rospy.Time.now().to_sec()
            v, omega = self.cmd
            radius = abs(v / omega) if abs(v) > 0.01 and abs(omega) > 0.01 else float("inf")
            self.samples.writerow([
                "{:.6f}".format(stamp), self.state, self.segment,
                "{:.6f}".format(self.pose[0]), "{:.6f}".format(self.pose[1]),
                "{:.6f}".format(v), "{:.6f}".format(omega),
                "{:.6f}".format(radius), "{:.6f}".format(self.actual[0]),
                "{:.6f}".format(self.actual[1]),
            ])
            self.samples_file.flush()
        if not self.active:
            self.history.clear()
            self.last_no_progress_key = None
            return
        self.history.append((stamp, self.pose[0], self.pose[1]))
        while self.history and stamp - self.history[0][0] > 8.0:
            self.history.popleft()
        if self.history and stamp - self.history[0][0] >= 7.8:
            displacement = math.hypot(
                self.pose[0] - self.history[0][1],
                self.pose[1] - self.history[0][2],
            )
            key = (self.state, self.segment)
            if displacement < 0.10 and key != self.last_no_progress_key:
                self.last_no_progress_key = key
                self._event(
                    "no_progress", state=self.state, segment=self.segment,
                    displacement=displacement,
                )
                rospy.logerr(
                    "MONITOR no progress: state=%s segment=%d displacement=%.3fm/8s",
                    self.state, self.segment, displacement,
                )
            elif displacement >= 0.10:
                self.last_no_progress_key = None

    def _finish(self):
        with self.io_lock:
            if self.closing:
                return
            self.closing = True
            self.timer.shutdown()
            summary = {
                "curvature_samples": self.curvature_samples,
                "curvature_violations": self.curvature_violations,
                "enforced_path_updates": self.path_updates,
                "hybrid_path_updates": self.hybrid_updates,
                "status_updates": self.status_updates,
                "final_state": self.state,
                "final_segment": self.segment,
            }
            with open(os.path.join(self.result_dir, "monitor_summary.json"), "w", encoding="utf-8") as stream:
                json.dump(summary, stream, indent=2, sort_keys=True)
                stream.write("\n")
            self.events.close()
            self.samples_file.close()


if __name__ == "__main__":
    rospy.init_node("coverage_monitor")
    CoverageMonitor()
    rospy.spin()
