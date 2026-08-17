#!/usr/bin/env python3

import math
import time
import unittest

import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Empty


def cost_at(message, world_x, world_y):
    origin = message.info.origin.position
    mx = int(math.floor((world_x - origin.x) / message.info.resolution))
    my = int(math.floor((world_y - origin.y) / message.info.resolution))
    if not 0 <= mx < message.info.width or not 0 <= my < message.info.height:
        raise AssertionError("world point is outside received costmap")
    return message.data[my * message.info.width + mx]


class GpsGeofenceCostmapIntegrationTest(unittest.TestCase):
    def test_fence_marks_both_costmaps_and_global_plan_detours(self):
        global_map = rospy.wait_for_message(
            "/move_base/global_costmap/costmap", OccupancyGrid, timeout=20.0
        )
        local_map = rospy.wait_for_message(
            "/move_base/local_costmap/costmap", OccupancyGrid, timeout=20.0
        )
        for costmap in (global_map, local_map):
            self.assertEqual(cost_at(costmap, 4.0, 0.0), 100)
            self.assertEqual(cost_at(costmap, 2.25, 0.0), 100)
            self.assertLess(cost_at(costmap, 0.0, 0.0), 100)

        rospy.wait_for_service("/move_base/make_plan", timeout=15.0)
        request = GetPlanRequest()
        request.start.header.frame_id = "camera_init"
        request.start.pose.orientation.w = 1.0
        request.goal.header.frame_id = "camera_init"
        request.goal.pose.position.x = 8.0
        request.goal.pose.orientation.w = 1.0
        request.tolerance = 0.0
        response = rospy.ServiceProxy("/move_base/make_plan", GetPlan)(request)
        self.assertGreater(len(response.plan.poses), 2)
        self.assertGreater(
            max(abs(pose.pose.position.y) for pose in response.plan.poses),
            2.0,
        )

        for namespace in (
            "/move_base/global_costmap/gps_geofence_layer",
            "/move_base/local_costmap/gps_geofence_layer",
        ):
            rospy.set_param(namespace + "/regions", [])
        reload_publisher = rospy.Publisher(
            "/gps/geofence/reload", Empty, queue_size=1
        )
        deadline = time.monotonic() + 3.0
        while reload_publisher.get_num_connections() < 1 and time.monotonic() < deadline:
            rospy.sleep(0.05)
        self.assertGreaterEqual(reload_publisher.get_num_connections(), 1)
        for _ in range(3):
            reload_publisher.publish(Empty())
            rospy.sleep(0.05)

        cleared = False
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            global_map = rospy.wait_for_message(
                "/move_base/global_costmap/costmap", OccupancyGrid, timeout=2.0
            )
            local_map = rospy.wait_for_message(
                "/move_base/local_costmap/costmap", OccupancyGrid, timeout=2.0
            )
            cleared = all(
                cost_at(costmap, 4.0, 0.0) < 100
                for costmap in (global_map, local_map)
            )
            if not cleared:
                rospy.sleep(0.1)
        self.assertTrue(cleared, "disabled fence remained in a running costmap")


if __name__ == "__main__":
    rospy.init_node("test_gps_geofence_costmap")
    import rostest

    rostest.rosrun(
        "robot_bringup",
        "gps_geofence_costmap_integration",
        GpsGeofenceCostmapIntegrationTest,
    )
