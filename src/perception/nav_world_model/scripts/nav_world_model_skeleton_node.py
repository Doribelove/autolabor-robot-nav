#!/usr/bin/env python3
"""Publish an explicit invalid heartbeat until the V2 world model is implemented."""

import rospy

from nav_world_model.msg import WorldModelHealth


def main():
    rospy.init_node("nav_world_model_skeleton")
    rate_hz = float(rospy.get_param("~publish_rate_hz", 1.0))
    if rate_hz <= 0.0:
        raise ValueError("publish_rate_hz must be positive")
    publisher = rospy.Publisher(
        "/nav_world_model/health", WorldModelHealth, queue_size=1, latch=True)
    rate = rospy.Rate(rate_hz)
    sequence = 0
    while not rospy.is_shutdown():
        sequence += 1
        message = WorldModelHealth()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = ""
        message.schema_version = "2.0"
        message.world_model_seq = sequence
        message.valid = False
        message.stale = True
        message.scan_valid = False
        message.tf_valid = False
        message.localization_valid = False
        message.costmap_valid = False
        message.tracker_valid = False
        message.scan_age_s = float("inf")
        message.tf_age_s = float("inf")
        message.tracker_age_s = float("inf")
        message.fault_reason = "v2_world_model_skeleton_not_implemented"
        publisher.publish(message)
        rate.sleep()


if __name__ == "__main__":
    main()
