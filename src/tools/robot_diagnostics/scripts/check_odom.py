#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import sys

import rospy
from nav_msgs.msg import Odometry


def finite_pose(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
    return all(math.isfinite(value) for value in values)


def main():
    rospy.init_node("check_odom", anonymous=True)
    topic = rospy.get_param("~topic", "/Odometry")
    timeout = float(rospy.get_param("~timeout", 10.0))
    required_frame = rospy.get_param("~required_frame", "")
    required_child_frame = rospy.get_param("~required_child_frame", "")

    try:
        msg = rospy.wait_for_message(topic, Odometry, timeout=timeout)
    except rospy.ROSException as exc:
        rospy.logerr("Odom check failed: %s", exc)
        return 2

    if required_frame and msg.header.frame_id != required_frame:
        rospy.logerr(
            "Odom check failed: %s frame_id=%s expected=%s",
            topic,
            msg.header.frame_id,
            required_frame,
        )
        return 3
    if required_child_frame and msg.child_frame_id != required_child_frame:
        rospy.logerr(
            "Odom check failed: %s child_frame_id=%s expected=%s",
            topic,
            msg.child_frame_id,
            required_child_frame,
        )
        return 4
    if not finite_pose(msg):
        rospy.logerr("Odom check failed: %s contains non-finite pose values", topic)
        return 5

    rospy.loginfo(
        "Odom check passed: %s frame=%s child=%s x=%.3f y=%.3f",
        topic,
        msg.header.frame_id,
        msg.child_frame_id,
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
