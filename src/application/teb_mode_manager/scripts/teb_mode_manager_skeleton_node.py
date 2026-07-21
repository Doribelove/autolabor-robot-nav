#!/usr/bin/env python3
"""Publish BALANCED/NONE/FAULTED until the V2 mode manager is implemented."""

import rospy

from teb_mode_manager.msg import ContextState


def main():
    rospy.init_node("teb_mode_manager_skeleton")
    rate_hz = float(rospy.get_param("~publish_rate_hz", 1.0))
    if rate_hz <= 0.0:
        raise ValueError("publish_rate_hz must be positive")
    publisher = rospy.Publisher(
        "/teb_mode_manager/context", ContextState, queue_size=1, latch=True)
    rate = rospy.Rate(rate_hz)
    sequence = 0
    while not rospy.is_shutdown():
        sequence += 1
        message = ContextState()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = ""
        message.schema_version = "2.0"
        message.world_model_seq = 0
        message.mode_seq = sequence
        message.geometry_mode = ContextState.GEOMETRY_BALANCED
        message.dynamic_overlay = ContextState.DYNAMIC_NONE
        message.transition_state = ContextState.TRANSITION_FAULTED
        message.mode_confidence = 0.0
        message.mode_dwell = rospy.Duration(0.0)
        message.minimum_dwell_remaining = rospy.Duration(0.0)
        message.valid = False
        message.reason = "v2_mode_manager_skeleton_not_implemented"
        publisher.publish(message)
        rate.sleep()


if __name__ == "__main__":
    main()
