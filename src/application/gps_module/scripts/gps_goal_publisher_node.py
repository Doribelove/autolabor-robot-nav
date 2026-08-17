#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus


def get_float_param(names, default=None):
    for name in names:
        if rospy.has_param(name):
            value = rospy.get_param(name)
            if value == "":
                continue
            return float(value)
    return default


class GpsGoalPublisherNode:
    def __init__(self):
        self.goal_fix_topic = rospy.get_param("~goal_fix_topic", "/gps/goal_fix")
        self.frame_id = rospy.get_param("~frame_id", "gps")
        self.altitude = float(rospy.get_param("~altitude", 0.0))
        self.publish_rate = float(rospy.get_param("~publish_rate", 1.0))
        self.publish_once = bool(rospy.get_param("~publish_once", True))
        self.wait_for_subscriber = bool(rospy.get_param("~wait_for_subscriber", True))

        self.latitude = get_float_param(["~lat", "~latitude", "~target_lat"])
        self.longitude = get_float_param(["~lon", "~longitude", "~target_lon"])
        if self.latitude is None or self.longitude is None:
            rospy.logerr(
                "GPS goal publisher needs lat/lon parameters, for example: "
                "rosrun gps_module gps_goal_publisher_node.py _lat:=31.0 _lon:=121.0"
            )
            rospy.signal_shutdown("missing GPS goal")
            return

        if math.isnan(self.latitude) or math.isnan(self.longitude):
            rospy.logerr("GPS goal latitude/longitude cannot be NaN")
            rospy.signal_shutdown("invalid GPS goal")
            return

        self.pub = rospy.Publisher(self.goal_fix_topic, NavSatFix, queue_size=1, latch=True)

    def make_msg(self):
        msg = NavSatFix()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = self.latitude
        msg.longitude = self.longitude
        msg.altitude = self.altitude
        return msg

    def wait_until_connected(self):
        if not self.wait_for_subscriber:
            return

        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and self.pub.get_num_connections() == 0:
            rospy.loginfo_throttle(
                2.0,
                "Waiting for a subscriber on %s before publishing GPS goal",
                self.goal_fix_topic,
            )
            rate.sleep()

    def spin(self):
        if rospy.is_shutdown():
            return

        self.wait_until_connected()
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.pub.publish(self.make_msg())
            rospy.loginfo(
                "Published GPS goal lat=%.8f lon=%.8f to %s",
                self.latitude,
                self.longitude,
                self.goal_fix_topic,
            )
            if self.publish_once:
                rospy.signal_shutdown("GPS goal published")
                return
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("gps_goal_publisher")
    node = GpsGoalPublisherNode()
    node.spin()
