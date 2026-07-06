#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix


def gps_to_xy(lat, lon, origin_lat, origin_lon):
    radius = 6378137.0
    d_lat = math.radians(lat - origin_lat)
    d_lon = math.radians(lon - origin_lon)
    ref_lat = math.radians(origin_lat)
    x = radius * d_lon * math.cos(ref_lat)
    y = radius * d_lat
    return x, y


def rotate_xy(x, y, yaw):
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return (
        cos_yaw * x - sin_yaw * y,
        sin_yaw * x + cos_yaw * y,
    )


class GpsGoalNode:
    def __init__(self):
        self.frame_id = rospy.get_param("~frame_id", "camera_init")
        self.goal_topic = rospy.get_param("~goal_topic", "/move_base_simple/goal")
        self.fix_topic = rospy.get_param("~fix_topic", "/gps/fix")
        self.goal_fix_topic = rospy.get_param("~goal_fix_topic", "/gps/goal_fix")
        self.publish_once = bool(rospy.get_param("~publish_once", False))

        self.origin_lat = rospy.get_param("~origin_lat", None)
        self.origin_lon = rospy.get_param("~origin_lon", None)
        if self.origin_lat == "":
            self.origin_lat = None
        if self.origin_lon == "":
            self.origin_lon = None
        if self.origin_lat is not None:
            self.origin_lat = float(self.origin_lat)
        if self.origin_lon is not None:
            self.origin_lon = float(self.origin_lon)
        if self.origin_lat is None or self.origin_lon is None:
            shared_origin_lat = rospy.get_param("/gps/origin_lat", None)
            shared_origin_lon = rospy.get_param("/gps/origin_lon", None)
            if shared_origin_lat is not None and shared_origin_lon is not None:
                self.origin_lat = float(shared_origin_lat)
                self.origin_lon = float(shared_origin_lon)

        self.target_lat = rospy.get_param("~target_lat", None)
        self.target_lon = rospy.get_param("~target_lon", None)
        if self.target_lat == "":
            self.target_lat = None
        if self.target_lon == "":
            self.target_lon = None
        if self.target_lat is not None:
            self.target_lat = float(self.target_lat)
        if self.target_lon is not None:
            self.target_lon = float(self.target_lon)

        self.yaw_offset = float(rospy.get_param("~yaw_offset", 0.0))
        yaw_offset_deg = float(rospy.get_param("~yaw_offset_deg", 0.0))
        if yaw_offset_deg != 0.0:
            self.yaw_offset += math.radians(yaw_offset_deg)

        self.pending_goals = []
        self.goal_pub = rospy.Publisher(self.goal_topic, PoseStamped, queue_size=1, latch=True)
        self.fix_sub = rospy.Subscriber(self.fix_topic, NavSatFix, self.fix_cb, queue_size=1)
        self.goal_fix_sub = rospy.Subscriber(self.goal_fix_topic, NavSatFix, self.goal_fix_cb, queue_size=1)

        rospy.Timer(rospy.Duration(0.5), self.publish_configured_goal, oneshot=True)
        rospy.loginfo(
            "GPS goal converter publishing %s in %s; yaw_offset=%.3f rad",
            self.goal_topic,
            self.frame_id,
            self.yaw_offset,
        )
        if self.origin_lat is not None and self.origin_lon is not None:
            rospy.loginfo(
                "GPS goal origin loaded: lat=%.8f lon=%.8f",
                self.origin_lat,
                self.origin_lon,
            )

    def fix_cb(self, msg):
        if self.origin_lat is not None and self.origin_lon is not None:
            return
        shared_origin_lat = rospy.get_param("/gps/origin_lat", None)
        shared_origin_lon = rospy.get_param("/gps/origin_lon", None)
        if shared_origin_lat is not None and shared_origin_lon is not None:
            self.origin_lat = float(shared_origin_lat)
            self.origin_lon = float(shared_origin_lon)
            rospy.loginfo(
                "GPS goal origin set from /gps/origin_*: lat=%.8f lon=%.8f",
                self.origin_lat,
                self.origin_lon,
            )
            self.publish_pending_goals()
            return
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        self.origin_lat = msg.latitude
        self.origin_lon = msg.longitude
        rospy.loginfo(
            "GPS goal origin set from %s: lat=%.8f lon=%.8f",
            self.fix_topic,
            self.origin_lat,
            self.origin_lon,
        )
        self.publish_pending_goals()

    def publish_pending_goals(self):
        while self.pending_goals:
            lat, lon = self.pending_goals.pop(0)
            rospy.loginfo("Publishing pending GPS goal after origin became available")
            self.publish_goal(lat, lon)

    def goal_fix_cb(self, msg):
        self.publish_goal(msg.latitude, msg.longitude)

    def publish_configured_goal(self, _event):
        if self.target_lat is None or self.target_lon is None:
            return
        self.publish_goal(self.target_lat, self.target_lon)
        if self.publish_once:
            rospy.signal_shutdown("configured GPS goal published")

    def publish_goal(self, lat, lon):
        if self.origin_lat is None or self.origin_lon is None:
            self.pending_goals.append((lat, lon))
            rospy.logwarn(
                "GPS goal delayed until origin is available: lat=%.8f lon=%.8f",
                lat,
                lon,
            )
            return False

        gps_x, gps_y = gps_to_xy(lat, lon, self.origin_lat, self.origin_lon)
        x, y = rotate_xy(gps_x, gps_y, self.yaw_offset)
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        rospy.loginfo(
            "Published GPS goal lat=%.8f lon=%.8f as x=%.3f y=%.3f in %s; raw_enu=(%.3f, %.3f)",
            lat,
            lon,
            x,
            y,
            self.frame_id,
            gps_x,
            gps_y,
        )
        return True


if __name__ == "__main__":
    rospy.init_node("gps_goal_node")
    GpsGoalNode()
    rospy.spin()
