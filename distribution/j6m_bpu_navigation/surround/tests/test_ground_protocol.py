"""Actual BG01 JPEG packets: four-camera handoff, missing rear and old input compatibility."""
import json
import struct
import unittest

import cv2
import numpy as np

from surround.ground_protocol import decode_ground_frame


class GroundProtocolTests(unittest.TestCase):
    def packet(self, cameras, sources=None, configured=True):
        meta = dict(protocol='conbine.ground_bev.frame.v1', fusion_session='fixture',
            produced_epoch_s=1000, image_width=600, image_height=600,
            sources={n: dict(sequence=10) for n in (sources or cameras)})
        if configured:
            meta['configured_cameras'] = cameras
        header = json.dumps(meta).encode()
        ok, jpeg = cv2.imencode('.jpg', np.full((600, 600, 3), [11, 79, 153], np.uint8))
        self.assertTrue(ok)
        return struct.pack('!4sII', b'BG01', len(header), len(jpeg))+header+jpeg.tobytes()

    def test_four_camera_packet_reaches_the_yolo_input(self):
        meta, image, _ = decode_ground_frame(self.packet(['front', 'left', 'right', 'rear']), 1001)
        self.assertIn('rear', meta['sources'])
        self.assertEqual(image.shape, (600, 600, 3))

    def test_four_camera_packet_missing_rear_is_rejected(self):
        data = self.packet(['front', 'left', 'right', 'rear'], ['front', 'left', 'right'])
        with self.assertRaisesRegex(ValueError, 'missing or unexpected'):
            decode_ground_frame(data, 1001)

    def test_existing_three_camera_packets_remain_supported(self):
        decode_ground_frame(self.packet(['front', 'left', 'right'], configured=False), 1001)

    def test_duplicate_unknown_stale_and_truncated_inputs_are_rejected(self):
        for cameras in (['front', 'left', 'right', 'rear', 'rear'], ['front', 'left', 'right', 'unknown']):
            with self.assertRaises(ValueError):
                decode_ground_frame(self.packet(cameras), 1001)
        data = self.packet(['front', 'left', 'right', 'rear'])
        for payload, now in ((data, 1003), (data, 999), (data[:-1], 1001)):
            with self.assertRaises(ValueError):
                decode_ground_frame(payload, now)


if __name__ == '__main__':
    unittest.main()
