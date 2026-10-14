#!/usr/bin/env python3
"""Exercise the real converter, scan projection and rolling obstacle costmap.

Run with: rostest robot_bringup livox_obstacle_scan.test
rostest creates a private master; this launch contains no hardware drivers.
"""
import math
import threading
import time
import unittest

import rospy
import rostest
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from geometry_msgs.msg import TransformStamped
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Header


MOUNT = (-0.011, -0.02329, 0.64412)  # lidar origin in base_link


class ObstacleScanTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_livox_obstacle_scan', anonymous=True)
        cls.lock = threading.Lock()
        cls.pose = (0., 0., 0.)
        cls.twist = (0., 0.)
        cls.odom_age = 0.
        cls.points = []
        cls.latest = {}
        cls.lidar = rospy.Publisher('/livox/lidar', CustomMsg, queue_size=2)
        cls.odom = rospy.Publisher('/gps/odom', Odometry, queue_size=2)
        cls.bad_lio = rospy.Publisher('/cloud_registered_body', PointCloud2, queue_size=2)
        cls.tf = tf2_ros.TransformBroadcaster()
        cls.subs = [rospy.Subscriber(topic, kind, lambda msg, key=key: cls.latest.update({key: msg}),
                                    queue_size=2) for topic, kind, key in [
            ('/scan', LaserScan, 'scan'), ('/cloud_livox_body', PointCloud2, 'cloud'),
            ('/move_base/local_costmap/costmap', OccupancyGrid, 'map')]]
        cls.timer = rospy.Timer(rospy.Duration(.05), cls.publish)
        time.sleep(1.)

    @classmethod
    def tearDownClass(cls):
        cls.timer.shutdown()

    @classmethod
    def publish(cls, _):
        now = rospy.Time.now()
        with cls.lock:
            px, py, yaw = cls.pose
            vx, wz = cls.twist
            age, points = cls.odom_age, list(cls.points)
        odom = Odometry(header=Header(stamp=now - rospy.Duration(age), frame_id='camera_init'),
                        child_frame_id='base_link')
        odom.pose.pose.position.x, odom.pose.pose.position.y = px, py
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = vx, wz
        cls.odom.publish(odom)
        tf = TransformStamped(header=Header(stamp=now, frame_id='camera_init'), child_frame_id='base_link')
        tf.transform.translation.x, tf.transform.translation.y = px, py
        tf.transform.rotation = odom.pose.pose.orientation
        cls.tf.sendTransform(tf)
        cloud = CustomMsg(header=Header(stamp=now-rospy.Duration(.1), frame_id='livox_frame'))
        cloud.points = points
        cloud.point_num = len(points)
        cls.lidar.publish(cloud)

    @staticmethod
    def point(x, y, z, offset=100000000, tag=0):
        return CustomPoint(offset_time=offset, x=x-MOUNT[0], y=y-MOUNT[1],
                           z=z-MOUNT[2], reflectivity=80, tag=tag, line=0)

    def configure(self, points, pose=(0., 0., 0.), twist=(0., 0.), odom_age=0.):
        with self.lock:
            type(self).points, type(self).pose = points, pose
            type(self).twist, type(self).odom_age = twist, odom_age
        time.sleep(.35)  # let already queued frames drain

    def wait_for(self, predicate, timeout=6.):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            time.sleep(.05)
        self.fail('Timed out waiting for obstacle pipeline/costmap')

    def scan_ranges(self):
        self.wait_for(lambda: 'scan' in self.latest)
        return [r for r in self.latest['scan'].ranges if math.isfinite(r)]

    def cloud_points(self):
        self.wait_for(lambda: 'cloud' in self.latest)
        return list(pc2.read_points(self.latest['cloud'], field_names=('x', 'y', 'z')))

    def test_01_range_height_and_physical_footprint(self):
        points = [self.point(3, 0, .4), self.point(2, 1.5, 1.3), self.point(-2, -1, .6),
                  self.point(6, -4.5, .5),  # distinct ray at the window's far edge
                  self.point(0, 10, .5), self.point(0, -30, .5),  # far clutter
                  self.point(1, 1, .1), self.point(2, -2, 2.),  # ground/above window
                  self.point(.5, .2, .5), self.point(-.79, -.39, 1.),  # vehicle
                  CustomPoint(), CustomPoint(x=float('nan'), y=1, z=0),
                  self.point(2, -1, .6, tag=0x20)]
        self.configure(points)
        values = sorted(self.scan_ranges())
        self.assertEqual(len(values), 4, values)
        for actual, expected in zip(values, sorted([3., 2.5, math.sqrt(5), 7.5])):
            self.assertAlmostEqual(actual, expected, places=4)
        self.assertAlmostEqual(self.latest['scan'].range_max, 8.1, places=5)
        self.assertEqual(self.latest['scan'].header.frame_id, 'base_link')

    def test_02_long_distance_and_heading_keep_obstacles_in_costmap(self):
        reference = None
        for px, py, yaw in [(0., 0., 0.), (600., 0., math.pi/2),
                            (1200., -400., math.pi), (-700., 800., -.7)]:
            with self.subTest(pose=(px, py, yaw)):
                self.configure([self.point(3, 0, .5), self.point(2, 1.5, 1.3)],
                               pose=(px, py, yaw))
                values = sorted(self.scan_ranges())
                if reference is None:
                    reference = values
                self.assertEqual(values, reference)
                wx, wy = px + 3*math.cos(yaw), py + 3*math.sin(yaw)

                def mapped_obstacle():
                    grid = self.latest.get('map')
                    if grid is None:
                        return False
                    info = grid.info
                    mx = int((wx-info.origin.position.x)/info.resolution)
                    my = int((wy-info.origin.position.y)/info.resolution)
                    # Scan angular bin quantization can shift a hit by <2 cm.
                    return any(0 <= x < info.width and 0 <= y < info.height and
                               grid.data[y*info.width+x] == 100
                               for x in range(mx-1, mx+2) for y in range(my-1, my+2))
                self.wait_for(mapped_obstacle)

    def test_03_lio_failure_does_not_change_scan(self):
        self.configure([self.point(3, 0, .5)])
        original = self.scan_ranges()
        bad = pc2.create_cloud_xyz32(Header(stamp=rospy.Time.now(), frame_id='body'),
                                    [(5000., 1000., -900.), (float('nan'), 0., 0.)])
        for _ in range(5):
            self.bad_lio.publish(bad)
            time.sleep(.1)
        self.assertEqual(self.scan_ranges(), original)

    def test_04_forward_reverse_and_turn_motion_compensation(self):
        for speed, yaw_rate in [(1.6, 0.), (-1.6, 0.), (1.6, .6), (-.6, -.5)]:
            with self.subTest(twist=(speed, yaw_rate)):
                # Integrate a chassis over 100 small steps, independently of
                # the converter's closed-form single-frame compensation.
                x = y = yaw = 0.
                points = []
                for i in range(101):
                    dx, dy = 5.-x, 1.-y
                    bx = math.cos(yaw)*dx + math.sin(yaw)*dy
                    by = -math.sin(yaw)*dx + math.cos(yaw)*dy
                    points.append(self.point(bx, by, .5, i*1000000))
                    if i < 100:
                        x += speed*math.cos(yaw + yaw_rate*.0005)*.001
                        y += speed*math.sin(yaw + yaw_rate*.0005)*.001
                        yaw += yaw_rate*.001
                self.configure(points, twist=(speed, yaw_rate))
                cloud = self.cloud_points()
                self.assertEqual(len(cloud), 101)
                for cx, cy, _ in cloud:
                    self.assertAlmostEqual(cx, bx, places=4)
                    self.assertAlmostEqual(cy, by, places=4)

    def test_05_missing_twist_uses_raw_local_observations(self):
        self.configure([self.point(3, 0, .5, 0), self.point(3, 0, .5)],
                       twist=(1.6, .5), odom_age=10.)
        for x, y, _ in self.cloud_points():
            self.assertAlmostEqual(x, 3., places=5)
            self.assertAlmostEqual(y, 0., places=5)
        self.assertEqual(len(self.scan_ranges()), 1)

    def test_06_empty_frame_publishes_no_phantom_obstacles(self):
        self.configure([CustomPoint(), CustomPoint(x=float('inf'))])
        self.assertEqual(self.cloud_points(), [])
        self.assertEqual(self.scan_ranges(), [])

    def test_07_empty_surroundings_do_not_form_costmap_ring(self):
        self.configure([])
        self.wait_for(lambda: 'map' in self.latest and
                      self.latest['map'].data.count(100) == 0)

    def road_points(self):
        # Measured field case: road rises in the sensor frame by about 3.3 deg;
        # fixed z filtering incorrectly marks the ground around 7-8 m ahead.
        return [self.point(ix*.3, iy*.3, .0585*ix*.3-.155 + .003*math.sin(ix+iy))
                for ix in range(-32, 33, 2) for iy in range(-32, 33, 2)
                if ix*ix+iy*iy >= 25]

    def test_08_tilted_road_is_not_an_obstacle_arc(self):
        self.configure(self.road_points())
        self.assertEqual(self.scan_ranges(), [])
        self.wait_for(lambda: 'map' in self.latest and
                      self.latest['map'].data.count(100) == 0)

    def test_09_obstacles_above_road_survive_ground_removal(self):
        obstacles = [(6., 1., .35), (5., -2., .9), (4., .5, .35), (7., 0., .08)]
        points = self.road_points() + [self.point(x, y, .0585*x-.155+height)
                                      for x, y, height in obstacles]
        self.configure(points)
        values = sorted(self.scan_ranges())
        self.assertEqual(len(values), len(obstacles), values)
        for actual, expected in zip(values, sorted(math.hypot(x,y) for x,y,_ in obstacles)):
            self.assertAlmostEqual(actual, expected, places=4)

    def test_10_wall_and_tabletop_are_not_used_as_ground(self):
        surfaces = [
            [self.point(3., iy*.1, iz*.1) for iy in range(-20,21) for iz in range(4,14)],
            [self.point(ix*.1, iy*.1, .7) for ix in range(20,61) for iy in range(-20,21)],
        ]
        for points in surfaces:
            with self.subTest(surface_points=len(points)):
                self.configure(points)
                self.assertGreaterEqual(len(self.scan_ranges()), 30)


if __name__ == '__main__':
    rostest.rosrun('robot_bringup', 'livox_obstacle_scan', ObstacleScanTest)
