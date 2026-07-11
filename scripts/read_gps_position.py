#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import sys

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion


def yaw_from_odom(msg):
    q = msg.pose.pose.orientation
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


class GpsPositionReader:
    def __init__(self, fix_topic, odom_topic, include_local):
        self.fix_topic = fix_topic
        self.odom_topic = odom_topic
        self.include_local = include_local
        self.latest_fix = None
        self.latest_odom = None
        self.last_printed_fix_stamp = None

        self.fix_sub = rospy.Subscriber(
            self.fix_topic,
            NavSatFix,
            self.fix_cb,
            queue_size=10,
        )
        self.odom_sub = None
        if self.include_local:
            self.odom_sub = rospy.Subscriber(
                self.odom_topic,
                Odometry,
                self.odom_cb,
                queue_size=10,
            )

    def fix_cb(self, msg):
        self.latest_fix = msg

    def odom_cb(self, msg):
        self.latest_odom = msg

    def wait_for_first_fix(self, timeout):
        rospy.loginfo("Waiting for GPS position on %s ...", self.fix_topic)
        self.latest_fix = rospy.wait_for_message(
            self.fix_topic,
            NavSatFix,
            timeout=timeout,
        )
        if self.include_local:
            try:
                self.latest_odom = rospy.wait_for_message(
                    self.odom_topic,
                    Odometry,
                    timeout=1.0,
                )
            except rospy.ROSException:
                rospy.logwarn(
                    "No local pose received on %s yet; printing lat/lon only",
                    self.odom_topic,
                )

    def format_position(self):
        fix = self.latest_fix
        if fix is None:
            return None

        parts = [
            "stamp=%.3f" % fix.header.stamp.to_sec(),
            "lat=%.12f" % fix.latitude,
            "lon=%.12f" % fix.longitude,
            "alt=%.3f" % fix.altitude,
            "status=%d" % fix.status.status,
        ]

        if self.include_local and self.latest_odom is not None:
            pose = self.latest_odom.pose.pose
            yaw = yaw_from_odom(self.latest_odom)
            parts.extend(
                [
                    "x=%.3f" % pose.position.x,
                    "y=%.3f" % pose.position.y,
                    "yaw=%.3fdeg" % math.degrees(yaw),
                ]
            )

        return "GPS_POSITION " + " ".join(parts)

    def print_once(self):
        text = self.format_position()
        if text:
            print(text)
            sys.stdout.flush()
            self.last_printed_fix_stamp = self.latest_fix.header.stamp

    def spin(self, rate_hz):
        rate = rospy.Rate(rate_hz)
        while not rospy.is_shutdown():
            if self.latest_fix is not None:
                stamp = self.latest_fix.header.stamp
                if stamp != self.last_printed_fix_stamp:
                    self.print_once()
                    self.last_printed_fix_stamp = stamp
            rate.sleep()


def main():
    parser = argparse.ArgumentParser(
        description="Print only GPS position information from GPS mode ROS topics."
    )
    parser.add_argument("--fix-topic", default="/gps/fix")
    parser.add_argument("--odom-topic", default="/gps/odom")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--lat-lon-only", action="store_true")
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("read_gps_position", anonymous=True)
    reader = GpsPositionReader(
        args.fix_topic,
        args.odom_topic,
        include_local=not args.lat_lon_only,
    )

    try:
        reader.wait_for_first_fix(args.timeout)
    except rospy.ROSException as exc:
        rospy.logerr("Timed out waiting for %s: %s", args.fix_topic, exc)
        return 1

    reader.print_once()
    if args.once:
        return 0

    reader.spin(max(args.rate, 0.1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
