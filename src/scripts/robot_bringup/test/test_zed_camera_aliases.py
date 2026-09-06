#!/usr/bin/env python3
"""Static routing contracts; the installed-library fixture is in scripts/."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


class ZedCameraAliases(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(Path(__file__).resolve().parents[1] /
                             'launch' / 'zed2_camera.launch').getroot()
        self.aliases = self.root.find("group[@if='$(arg publish_fod_aliases)']")

    def test_only_image_endpoints_are_remapped(self):
        remaps = {node.attrib['from']: node.attrib['to']
                  for node in self.aliases.findall('remap')}
        self.assertEqual(remaps, {
            '/$(arg camera_name)/$(arg node_name)/rgb/image_rect_color': '/fod_camera/image_raw',
            '/$(arg camera_name)/$(arg node_name)/depth/depth_registered': '/fod_camera/depth_registered',
        })

    def test_rgb_and_depth_keep_same_derived_info_endpoint(self):
        for node in self.aliases.findall('remap'):
            derived_info = node.attrib['to'].rsplit('/', 1)[0] + '/camera_info'
            self.assertEqual(derived_info, '/fod_camera/camera_info')

    def test_native_camera_debug_entry_is_retained(self):
        native = self.root.find("group[@unless='$(arg publish_fod_aliases)']")
        self.assertIsNotNone(native)
        self.assertEqual(native.find('include').attrib['file'],
                         '$(find zed_wrapper)/launch/zed2.launch')
        self.assertEqual(native.findall('remap'), [])

    def test_camera_tf_and_depth_defaults_are_not_relaxed(self):
        defaults = {node.attrib['name']: node.attrib.get('default')
                    for node in self.root.findall('arg')}
        self.assertEqual(defaults['publish_tf'], 'false')
        self.assertEqual(defaults['publish_map_tf'], 'false')
        self.assertEqual(defaults['depth_mode'], 'QUALITY')


if __name__ == '__main__':
    unittest.main()
