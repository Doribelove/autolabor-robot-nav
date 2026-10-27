#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path
import unittest

import rospy
from sensor_msgs.msg import LaserScan

path=Path(__file__).parents[1]/'scripts/mid360_scan_guard.py'
spec=importlib.util.spec_from_file_location('mid360_guard',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

def scan():
    m=LaserScan();m.header.frame_id='base_link';m.header.stamp=rospy.Time.from_sec(100.)
    m.angle_min=-math.pi;m.angle_max=math.pi;m.angle_increment=math.pi/2
    m.range_min=.5;m.range_max=12.;m.ranges=[1.,2.,float('inf'),2.,1.]
    return m

class Mid360GuardTest(unittest.TestCase):
    def test_full_azimuth_scan_is_accepted_without_any_auxiliary_sensor(self):
        self.assertTrue(module.validate_scan(scan(),100.1)[0])
    def test_all_infinite_ranges_still_represent_a_valid_sensor_scan(self):
        m=scan();m.ranges=[float('inf')]*5
        self.assertTrue(module.validate_scan(m,100.1)[0])
    def test_stale_and_future_timestamps_are_rejected(self):
        self.assertFalse(module.validate_scan(scan(),100.6)[0])
        self.assertFalse(module.validate_scan(scan(),99.7)[0])
    def test_zero_timestamp_is_rejected(self):
        m=scan();m.header.stamp=rospy.Time()
        self.assertFalse(module.validate_scan(m,100.)[0])
    def test_wrong_frame_is_rejected(self):
        m=scan();m.header.frame_id='unconfirmed_sensor_frame'
        self.assertFalse(module.validate_scan(m,100.1)[0])
    def test_malformed_geometry_is_rejected(self):
        for value in (0.,-1.,float('nan')):
            m=scan();m.angle_increment=value
            self.assertFalse(module.validate_scan(m,100.1)[0])
    def test_invalid_ranges_are_rejected(self):
        for value in (float('nan'),-1.,-float('inf')):
            m=scan();m.ranges[0]=value
            self.assertFalse(module.validate_scan(m,100.1)[0])
    def test_inconsistent_beam_count_is_rejected(self):
        m=scan();m.ranges=[1.]*30
        self.assertFalse(module.validate_scan(m,100.1)[0])

if __name__=='__main__':unittest.main()
