#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
from collections import deque

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64, String


def is_finite_xy(msg):
    p = msg.pose.pose.position
    return math.isfinite(p.x) and math.isfinite(p.y)


def stddev(values, mean):
    if not values:
        return 0.0
    return math.sqrt(sum((value - mean) ** 2 for value in values) / len(values))


class GpsStaticErrorMonitor:
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/gps/odom")
        self.window_seconds = float(rospy.get_param("~window_seconds", 120.0))
        self.warmup_seconds = float(rospy.get_param("~warmup_seconds", 5.0))
        self.warn_error = float(rospy.get_param("~warn_error", 0.5))
        self.error_prefix = rospy.get_param("~error_prefix", "/gps/static_error").rstrip("/")

        self.start_time = rospy.Time.now()
        self.reference_x = None
        self.reference_y = None
        self.samples = deque()

        self.current_error_pub = rospy.Publisher(
            self.error_prefix + "/current",
            Float64,
            queue_size=10,
        )
        self.rms_error_pub = rospy.Publisher(
            self.error_prefix + "/rms",
            Float64,
            queue_size=10,
        )
        self.max_error_pub = rospy.Publisher(
            self.error_prefix + "/max",
            Float64,
            queue_size=10,
        )
        self.std_x_pub = rospy.Publisher(
            self.error_prefix + "/std_x",
            Float64,
            queue_size=10,
        )
        self.std_y_pub = rospy.Publisher(
            self.error_prefix + "/std_y",
            Float64,
            queue_size=10,
        )
        self.summary_pub = rospy.Publisher(
            self.error_prefix + "/summary",
            String,
            queue_size=10,
        )

        self.reset_sub = rospy.Subscriber(
            self.error_prefix + "/reset",
            Empty,
            self.reset_cb,
            queue_size=1,
        )
        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_cb,
            queue_size=100,
        )

        rospy.loginfo(
            "GPS static error monitor: odom=%s prefix=%s window=%.1fs warmup=%.1fs warn=%.2fm",
            self.odom_topic,
            self.error_prefix,
            self.window_seconds,
            self.warmup_seconds,
            self.warn_error,
        )

    def reset_cb(self, _msg):
        self.reference_x = None
        self.reference_y = None
        self.samples.clear()
        self.start_time = rospy.Time.now()
        rospy.loginfo("GPS static error monitor reset; waiting %.1fs warmup", self.warmup_seconds)

    def odom_cb(self, msg):
        if not is_finite_xy(msg):
            rospy.logwarn_throttle(2.0, "Ignoring non-finite odom sample on %s", self.odom_topic)
            return

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()
        p = msg.pose.pose.position

        if (rospy.Time.now() - self.start_time).to_sec() < self.warmup_seconds:
            rospy.loginfo_throttle(
                2.0,
                "GPS static error monitor warming up: %.1fs remaining",
                self.warmup_seconds - (rospy.Time.now() - self.start_time).to_sec(),
            )
            return

        if self.reference_x is None or self.reference_y is None:
            self.reference_x = p.x
            self.reference_y = p.y
            rospy.loginfo(
                "GPS static error reference set from %s: x=%.3f y=%.3f frame=%s",
                self.odom_topic,
                self.reference_x,
                self.reference_y,
                msg.header.frame_id,
            )

        dx = p.x - self.reference_x
        dy = p.y - self.reference_y
        error = math.hypot(dx, dy)
        self.samples.append((stamp.to_sec(), dx, dy, error))
        self.trim_samples(stamp.to_sec())
        self.publish_metrics(dx, dy, error)

    def trim_samples(self, now_sec):
        if self.window_seconds <= 0.0:
            return
        oldest = now_sec - self.window_seconds
        while self.samples and self.samples[0][0] < oldest:
            self.samples.popleft()

    def publish_metrics(self, dx, dy, error):
        errors = [sample[3] for sample in self.samples]
        xs = [sample[1] for sample in self.samples]
        ys = [sample[2] for sample in self.samples]
        count = len(errors)
        if count == 0:
            return

        mean_x = sum(xs) / count
        mean_y = sum(ys) / count
        rms = math.sqrt(sum(value * value for value in errors) / count)
        max_error = max(errors)
        std_x = stddev(xs, mean_x)
        std_y = stddev(ys, mean_y)

        self.current_error_pub.publish(Float64(data=error))
        self.rms_error_pub.publish(Float64(data=rms))
        self.max_error_pub.publish(Float64(data=max_error))
        self.std_x_pub.publish(Float64(data=std_x))
        self.std_y_pub.publish(Float64(data=std_y))

        summary = (
            "samples=%d current=%.3fm dx=%.3f dy=%.3f "
            "rms=%.3fm max=%.3fm std_x=%.3fm std_y=%.3fm"
            % (count, error, dx, dy, rms, max_error, std_x, std_y)
        )
        self.summary_pub.publish(String(data=summary))

        if self.warn_error > 0.0 and error >= self.warn_error:
            rospy.logwarn_throttle(2.0, "GPS static drift high: %s", summary)
        else:
            rospy.loginfo_throttle(2.0, "GPS static drift: %s", summary)


def main():
    rospy.init_node("gps_static_error_monitor")
    GpsStaticErrorMonitor()
    rospy.spin()


if __name__ == "__main__":
    main()
