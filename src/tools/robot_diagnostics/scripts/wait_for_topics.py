#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import time

import rosgraph
import rospy


def topic_exists(topic_name):
    master = rosgraph.Master("/wait_for_topics")
    published = master.getPublishedTopics("")
    return any(name == topic_name for name, _topic_type in published)


def main():
    rospy.init_node("wait_for_topics", anonymous=True)
    topics = rospy.get_param("~topics", [])
    timeout = float(rospy.get_param("~timeout", 30.0))
    poll_period = float(rospy.get_param("~poll_period", 0.2))

    if isinstance(topics, str):
        topics = [item.strip() for item in topics.split(",") if item.strip()]

    if not topics:
        rospy.logerr("No topics configured for wait_for_topics")
        return 2

    deadline = time.time() + timeout
    pending = set(topics)
    while pending and not rospy.is_shutdown():
        for topic in list(pending):
            try:
                ready = topic_exists(topic)
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "Waiting for ROS master/topics: %s", exc)
                ready = False
            if ready:
                rospy.loginfo("Topic ready: %s", topic)
                pending.remove(topic)
        if not pending:
            break
        if time.time() > deadline:
            rospy.logerr("Timed out waiting for topics: %s", ", ".join(sorted(pending)))
            return 3
        rospy.sleep(poll_period)

    return 0


if __name__ == "__main__":
    sys.exit(main())
