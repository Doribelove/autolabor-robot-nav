#!/usr/bin/env python3
"""Resolve actual launch/URDF geometry without starting ROS or hardware."""
import math
import os
from pathlib import Path
import shlex
import subprocess
import unittest
import xml.etree.ElementTree as ET

import numpy as np
import roslaunch.config
import rospkg
import yaml
from tf.transformations import euler_matrix


ROOT = Path(__file__).resolve().parents[1]


def launch(name, args=()):
    return roslaunch.config.load_config_default(
        [(str(ROOT/'launch'/name), list(args))], None, verbose=False)


def fixed(node):
    values = shlex.split(node.args)
    x, y, z, yaw, pitch, roll = map(float, values[:6])
    result = euler_matrix(roll, pitch, yaw)
    result[:3, 3] = [x, y, z]
    return result


class InstallationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.gps = launch('scan_livox_direct.launch', ['start_driver:=false'])
        cls.lio = launch('scan_fast_lio.launch')
        # scripts/zed2_camera.sh deliberately runs the installed ARM wrapper in
        # its own environment; it is not on the AGX catkin package path.
        driver_setup = os.environ.get('ZED_DRIVER_SETUP', str(Path.home()/'robot_ws/.deps/setup.bash'))
        cls.zed_urdf = subprocess.check_output([
            'bash', '-c', 'source "$1" && exec /usr/bin/python3 -c "$2" "$3"',
            'zed-installation-test', driver_setup,
            "import roslaunch.config,sys; cfg=roslaunch.config.load_config_default([sys.argv[1]],None,verbose=False); sys.stdout.write(cfg.params['/zed2/zed2_description'].value)",
            str(ROOT/'launch/zed2_camera.launch')], text=True)
        cls.nav = launch('navigation_arena.launch', ['localization_source:=gps', 'start_rviz:=false'])
        cls.footprint = yaml.safe_load((ROOT/'config/vehicle_footprint.yaml').read_text())

    def nodes(self, config):
        return {node.name: node for node in config.nodes}

    def test_lidar_mount_in_both_localization_modes(self):
        nodes = self.nodes(self.gps)
        transform = fixed(nodes['body_to_base_link']) @ fixed(nodes['body_to_livox_frame'])
        expected = np.eye(4); expected[:3, 3] = [0, 0, .65]
        np.testing.assert_allclose(transform, expected, atol=1e-10)
        nodes = self.nodes(self.lio)
        transform = np.linalg.inv(fixed(nodes['body_to_base_link'])) @ fixed(nodes['body_to_livox_frame'])
        np.testing.assert_allclose(transform, expected, atol=1e-10)

    def test_side_and_rear_optical_rays_point_outward_and_down(self):
        nodes = self.nodes(self.gps)
        for index, xyz, yaw, pitch in [(0,[0,.215,.55],math.pi/2,math.radians(22)),
                                       (1,[0,-.215,.55],-math.pi/2,math.radians(22)),
                                       (2,[-.24,0,.55],math.pi,math.radians(15))]:
            with self.subTest(camera=index):
                matrix = fixed(nodes['base_to_rx1_'+str(index)]) @ fixed(nodes['rx1_'+str(index)+'_to_optical'])
                np.testing.assert_allclose(matrix[:3,3], xyz, atol=1e-10)
                direction = [math.cos(pitch)*math.cos(yaw),math.cos(pitch)*math.sin(yaw),-math.sin(pitch)]
                np.testing.assert_allclose(matrix[:3,2], direction, atol=1e-10)
                self.assertAlmostEqual(matrix[2,3] + .25, .8)

    def test_zed_left_image_uses_measured_position_and_is_level(self):
        # This is the URDF expanded by the real camera launch, including the
        # vendor mounting screw, 0.05 rad slope and stereo baseline.
        xml = self.zed_urdf
        joints = {}
        for joint in ET.fromstring(xml).findall('joint'):
            origin = joint.find('origin')
            xyz = list(map(float, origin.get('xyz','0 0 0').split())) if origin is not None else [0]*3
            rpy = list(map(float, origin.get('rpy','0 0 0').split())) if origin is not None else [0]*3
            matrix = euler_matrix(*rpy); matrix[:3,3] = xyz
            joints[joint.find('child').get('link')] = (joint.find('parent').get('link'), matrix)
        child = 'zed2_left_camera_optical_frame'; matrix = np.eye(4)
        while child != 'base_link':
            child, transform = joints[child]; matrix = transform @ matrix
        np.testing.assert_allclose(matrix[:3,3], [.96,0,.07], atol=1e-10)
        # Optical forward/right/down must be chassis forward/right/down.
        np.testing.assert_allclose(matrix[:3,:3], [[0,0,1],[-1,0,0],[0,-1,0]], atol=1e-10)
        self.assertAlmostEqual(matrix[2,3] + .25, .32)

    def test_navigation_margin_is_applied_once_to_asymmetric_body(self):
        for name in ('global_costmap','local_costmap'):
            actual = self.nav.params['/move_base/'+name+'/footprint'].value
            margin = self.nav.params['/move_base/'+name+'/footprint_padding'].value
            self.assertEqual(actual, self.footprint)
            padded = [(x+math.copysign(margin,x),y+math.copysign(margin,y)) for x,y in actual]
            xs,ys = zip(*padded)
            np.testing.assert_allclose([min(xs),max(xs),min(ys),max(ys)],[-.62,1.32,-.5,.5])
            self.assertAlmostEqual(max(xs)-min(xs),1.94)
            self.assertAlmostEqual(max(ys)-min(ys),1.)
        self.assertEqual(self.nav.params['/move_base/TebLocalPlannerROS/footprint_model/vertices'].value,self.footprint)
        self.assertEqual(self.nav.params['/vehicle_outline/footprint'].value,self.footprint)
        self.assertEqual(self.gps.params['/pointcloud_self_filter/self_footprint'].value,self.footprint)

    def test_recovery_gets_physical_body_before_gui_or_navigation_starts(self):
        # The mode manager starts before navigation/vehicle_outline in bringup.
        package = rospkg.RosPack().get_path('autolabor_fod_control')
        cfg = roslaunch.config.load_config_default(
            [str(Path(package)/'launch/gps_visual_recovery_standby.launch')],None,verbose=False)
        self.assertEqual(cfg.params['/fod_navigation_mode/vehicle_footprint'].value,self.footprint)

    def test_no_duplicate_child_frames_in_fixed_mount_tree(self):
        children = [shlex.split(node.args)[7] for node in self.gps.nodes if node.type=='static_transform_publisher']
        self.assertEqual(len(children),len(set(children)))
        self.assertNotIn('base_link',children)  # GPS already owns this child.


if __name__ == '__main__':
    unittest.main()
