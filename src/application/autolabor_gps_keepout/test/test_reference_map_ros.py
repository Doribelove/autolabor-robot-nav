#!/usr/bin/env python3
"""Check the real map's markers on a private ROS master, never vehicle ROS."""
import json
from pathlib import Path
import time
import unittest

import rospy
import rostest
from visualization_msgs.msg import MarkerArray, Marker


class ReferenceMapROSTest(unittest.TestCase):
    def test_actual_data_projection_and_display_only_publishers(self):
        rospy.init_node('test_reference_map')
        path = rospy.get_param('/gps_reference_map/map_file')
        data = json.loads(Path(path).read_text())
        received = []
        sub = rospy.Subscriber('/gps/reference_map/markers',MarkerArray,received.append,queue_size=2)
        time.sleep(.2)
        self.assertEqual(received,[])  # No invented GPS origin.
        lon,lat = data['features'][0]['geometry']['coordinates'][0]
        rospy.set_param('/gps/origin_lat',lat); rospy.set_param('/gps/origin_lon',lon)
        deadline = time.monotonic()+8
        while not received and time.monotonic()<deadline: time.sleep(.05)
        self.assertTrue(received)
        lines = [m for m in received[-1].markers if m.action==Marker.ADD and m.type==Marker.LINE_STRIP]
        self.assertEqual(len(lines),len(data['features']))
        self.assertEqual(sum(len(m.points) for m in lines),data['metadata']['display_points'])
        self.assertAlmostEqual(lines[0].points[0].x,0.,places=5)
        self.assertAlmostEqual(lines[0].points[0].y,0.,places=5)
        old = lines[0].points[0].x; received.clear()
        rospy.set_param('/gps/origin_lon',lon+.00001)
        deadline = time.monotonic()+5
        while not received and time.monotonic()<deadline: time.sleep(.05)
        self.assertTrue(received)
        lines = [m for m in received[-1].markers if m.action==Marker.ADD and m.type==Marker.LINE_STRIP]
        self.assertLess(lines[0].points[0].x,old-.9)
        code,message,state = rospy.get_master().getSystemState()
        self.assertEqual(code,1)
        published = {topic for topic,nodes in state[0] if '/gps_reference_map' in nodes}
        self.assertEqual(published,{'/rosout','/gps/reference_map/markers','/gps/reference_map/status'})
        sub.unregister()


if __name__ == '__main__':
    rostest.rosrun('autolabor_gps_keepout','reference_map',ReferenceMapROSTest)
