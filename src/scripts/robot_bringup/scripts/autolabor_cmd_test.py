#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time

import rospy
from geometry_msgs.msg import Twist


def main():
    rospy.init_node("autolabor_cmd_test")
    cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
    linear = float(rospy.get_param("~linear", 0.2))
    angular = float(rospy.get_param("~angular", 0.0))
    duration = float(rospy.get_param("~duration", 2.0))
    rate_hz = float(rospy.get_param("~rate", 10.0))

    pub = rospy.Publisher(cmd_topic, Twist, queue_size=1)
    deadline = time.time() + 5.0
    while pub.get_num_connections() == 0 and time.time() < deadline and not rospy.is_shutdown():
        rospy.loginfo_throttle(1.0, "Waiting for a subscriber on %s", cmd_topic)
        rospy.sleep(0.1)

    if pub.get_num_connections() == 0:
        rospy.logerr("No subscriber connected on %s", cmd_topic)
        return 2

    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular

    rospy.loginfo("Publishing %.3f m/s, %.3f rad/s to %s for %.1fs", linear, angular, cmd_topic, duration)
    rate = rospy.Rate(rate_hz)
    end_time = time.time() + duration
    while time.time() < end_time and not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

    stop = Twist()
    for _ in range(10):
        pub.publish(stop)
        rate.sleep()

    rospy.loginfo("Stop command sent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
