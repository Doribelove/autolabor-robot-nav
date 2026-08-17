#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys

import rospy
import tf2_ros


def main():
    rospy.init_node("check_tf", anonymous=True)
    target_frame = rospy.get_param("~target_frame", "camera_init")
    source_frame = rospy.get_param("~source_frame", "base_link")
    timeout = float(rospy.get_param("~timeout", 10.0))
    poll_period = float(rospy.get_param("~poll_period", 0.2))

    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)

    deadline = rospy.Time.now() + rospy.Duration(timeout)
    last_error = None
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        try:
            transform = buffer.lookup_transform(
                target_frame,
                source_frame,
                rospy.Time(0),
                rospy.Duration(poll_period),
            )
            rospy.loginfo(
                "TF check passed: %s -> %s via stamp %.3f",
                target_frame,
                source_frame,
                transform.header.stamp.to_sec(),
            )
            return 0
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as exc:
            last_error = exc
            rospy.logwarn_throttle(
                2.0,
                "Waiting for TF %s -> %s: %s",
                target_frame,
                source_frame,
                exc,
            )

    rospy.logerr(
        "TF check failed: %s -> %s unavailable after %.1fs: %s",
        target_frame,
        source_frame,
        timeout,
        last_error,
    )
    return 3


if __name__ == "__main__":
    sys.exit(main())
