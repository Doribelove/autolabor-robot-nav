import io
import os
import unittest
from unittest.mock import patch
import numpy as np
from live_protocol import HEADER, PAYLOAD_BYTES, crop_geometry, frame_packet, header, read_payload, unpack
from qt_bpu_bridge import prepare


class CroppedProtocolTest(unittest.TestCase):
    def decode(self, packet):
        with patch('live_protocol.read_exact', side_effect=lambda stream, size, *_: stream.read(size)):
            return read_payload(io.BytesIO(packet[HEADER.size:]), packet[:HEADER.size])

    def test_byte_exact_landscape_portrait_odd_edges_and_square(self):
        random = np.random.default_rng(24)
        for width, height in ((640, 360), (810, 1080), (643, 361), (1920, 1080), (64, 1080), (896, 896)):
            frame = random.integers(0, 256, (height, width, 3), dtype=np.uint8)
            payload, _ = prepare(frame)
            packet = frame_packet(5, 12345, width, height, payload)
            self.assertEqual(unpack(packet[:HEADER.size]), (5, 12345, width, height))
            self.assertEqual(self.decode(packet), payload)
            if width != height:
                self.assertEqual(packet[:4], b'FC03')
                self.assertLess(len(packet), PAYLOAD_BYTES)

    def test_unknown_padding_and_explicit_legacy_fallback(self):
        payload = os.urandom(PAYLOAD_BYTES)
        for crop in (True, False):
            packet = frame_packet(1, 2, 640, 360, payload, crop)
            self.assertEqual(packet[:4], b'FC01')
            self.assertEqual(self.decode(packet), payload)

    def test_truncated_and_invalid_source_geometry_rejected(self):
        prefix = b'FC03' + header(1, 2, 640, 360)[4:]
        with self.assertRaises(ValueError):
            self.decode(prefix + b'bad')
        for width, height in ((0, 360), (640, 0), (1921, 360), (640, 1081)):
            with self.subTest(width=width, height=height), self.assertRaises(ValueError):
                frame_packet(1, 2, width, height, bytes(PAYLOAD_BYTES))

    def test_sender_exact_size_and_fixed_camera_bandwidth(self):
        with self.assertRaises(ValueError):
            frame_packet(1, 2, 640, 360, b'bad')
        self.assertEqual(crop_geometry(640, 360), (896, 504, 896, 252))


if __name__ == '__main__':
    unittest.main()
