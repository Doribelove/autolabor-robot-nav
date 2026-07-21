#!/usr/bin/env python3
"""Deterministic delayed/noisy LaserScan transport for V2 simulation only."""

import copy
import heapq
import threading

import rospy
from sensor_msgs.msg import LaserScan

from m2_gazebo.sensor_transport import noisy_range, release_time


class V2ScanTransport:
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/v2/scan_raw")
        self.output_topic = rospy.get_param("~output_topic", "/scan")
        self.delay_s = float(rospy.get_param("~delay_s", 0.06))
        self.jitter_s = float(rospy.get_param("~jitter_s", 0.01))
        self.noise_stddev_m = float(rospy.get_param("~noise_stddev_m", 0.01))
        self.seed = int(rospy.get_param("~seed", 42))
        release_time(0.0, self.delay_s, self.jitter_s, self.seed, 0)
        self.publisher = rospy.Publisher(self.output_topic, LaserScan, queue_size=10)
        self.pending = []
        self.sequence = 0
        self.last_now = None
        self.lock = threading.Lock()
        self.subscriber = rospy.Subscriber(
            self.input_topic, LaserScan, self._receive, queue_size=10
        )
        self.timer = rospy.Timer(rospy.Duration(0.005), self._release)

    def _receive(self, source):
        now = rospy.Time.now().to_sec()
        with self.lock:
            sequence = self.sequence
            self.sequence += 1
            message = copy.deepcopy(source)
            message.ranges = [
                noisy_range(
                    value,
                    message.range_min,
                    message.range_max,
                    self.noise_stddev_m,
                    self.seed,
                    sequence,
                    index,
                )
                for index, value in enumerate(message.ranges)
            ]
            due = release_time(
                now, self.delay_s, self.jitter_s, self.seed, sequence
            )
            heapq.heappush(self.pending, (due, sequence, message))

    def _release(self, _event):
        now = rospy.Time.now().to_sec()
        ready = []
        with self.lock:
            if self.last_now is not None and now < self.last_now:
                self.pending.clear()
                self.sequence = 0
            self.last_now = now
            while self.pending and self.pending[0][0] <= now:
                ready.append(heapq.heappop(self.pending)[2])
        for message in ready:
            self.publisher.publish(message)


def main():
    rospy.init_node("v2_scan_transport")
    V2ScanTransport()
    rospy.spin()


if __name__ == "__main__":
    main()
