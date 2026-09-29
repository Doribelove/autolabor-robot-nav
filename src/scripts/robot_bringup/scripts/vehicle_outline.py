#!/usr/bin/env python3
"""Display the physical footprint in the moving chassis frame."""

import rospy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def make_outline(footprint, frame):
    xs, ys = zip(*footprint)
    center_x = (min(xs) + max(xs)) / 2.0
    center_y = (min(ys) + max(ys)) / 2.0
    result = MarkerArray()

    def marker(identifier, kind):
        item = Marker()
        item.header.frame_id = frame
        item.ns = "vehicle_outline"
        item.id = identifier
        item.type = kind
        item.action = Marker.ADD
        item.pose.orientation.w = 1.0
        item.frame_locked = True
        item.color.a = 1.0
        result.markers.append(item)
        return item

    border = marker(0, Marker.LINE_STRIP)
    border.scale.x = 0.08
    border.color.r, border.color.g, border.color.b = 1.0, 0.85, 0.0
    border.points = [Point(x=x, y=y, z=0.08) for x, y in footprint + footprint[:1]]

    forward = marker(1, Marker.ARROW)
    forward.points = [Point(x=center_x, y=center_y, z=0.10),
                      Point(x=max(xs) - 0.10, y=center_y, z=0.10)]
    forward.scale.x, forward.scale.y, forward.scale.z = 0.045, 0.14, 0.12
    forward.color.r, forward.color.g, forward.color.b = 0.1, 1.0, 0.3

    label = marker(2, Marker.TEXT_VIEW_FACING)
    label.pose.position = Point(x=center_x, y=min(ys) - 0.25, z=0.12)
    label.scale.z = 0.28
    label.color.r, label.color.g, label.color.b = 1.0, 0.95, 0.4
    label.text = "{:.2f} m x {:.2f} m".format(max(xs) - min(xs), max(ys) - min(ys))
    return result


def main():
    rospy.init_node("vehicle_outline")
    outline = make_outline(rospy.get_param("~footprint"),
                           rospy.get_param("~frame_id", "base_link"))
    publisher = rospy.Publisher("/vehicle/outline", MarkerArray, queue_size=1, latch=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        publisher.publish(outline)
        rate.sleep()


if __name__ == "__main__":
    main()
