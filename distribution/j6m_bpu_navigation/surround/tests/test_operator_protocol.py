import copy
import json
import unittest

from distribution.j6m_bpu_navigation.surround.operator_protocol import (
    capture_identity, SourceGuard, packet, PREFIX, MAX_JPEG)


class OperatorProtocolTest(unittest.TestCase):
    def setUp(self):
        self.meta = dict(camera_port=0, gdc_applied=1, layout='NV12', vendor_format=2,
                         width=960, height=540, bytes=777600, lut_sha256='a'*64,
                         session='camera-session', received_monotonic=100., sequence=3,
                         raw_stamp=123456789, hal_received_ns=123456790)

    def source(self, port=0, **changes):
        meta = dict(self.meta, camera_port=port, **changes)
        return capture_identity(meta, port, 'a'*64, now=100.1)

    def test_source_device_time_is_not_replaced_with_wall_time(self):
        source = self.source()
        self.assertEqual(source['raw_stamp'], '123456789')
        self.assertEqual(source['hal_received_ns'], '123456790')
        self.assertNotIn('source_stamp_ns', source)

    def test_no_raw_image_or_wrong_lut_fallback(self):
        for changes in [{'gdc_applied':0}, {'lut_sha256':'b'*64}, {'layout':'NV16'}, {'width':1920}]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.source(**changes)

    def test_cross_port_reply_is_rejected(self):
        with self.assertRaises(ValueError):
            capture_identity(self.meta, 2, 'a'*64, now=100.1)

    def test_stale_and_future_read_rejected(self):
        for stamp in [99.59, 100.11, float('nan')]:
            with self.subTest(stamp=stamp), self.assertRaises(ValueError):
                self.source(received_monotonic=stamp)

    def test_duplicate_does_not_become_new_inference(self):
        guard = SourceGuard()
        self.assertTrue(guard.accept(self.source()))
        self.assertFalse(guard.accept(self.source()))

    def test_ports_have_independent_sequence_guards(self):
        guard = SourceGuard()
        for port in range(3):
            self.assertTrue(guard.accept(self.source(port)))
        self.assertTrue(guard.accept(self.source(1, sequence=4, raw_stamp=123456800)))
        self.assertFalse(guard.accept(self.source(2)))

    def test_reordered_or_mutated_duplicate_rejected(self):
        guard = SourceGuard()
        guard.accept(self.source())
        for changes in [{'sequence':2}, {'sequence':4,'raw_stamp':123456788}, {'hal_received_ns':3}]:
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                guard.accept(self.source(**changes))

    def test_retired_capture_session_rejected(self):
        guard = SourceGuard()
        guard.accept(self.source())
        self.assertTrue(guard.accept(self.source(session='new', sequence=0)))
        with self.assertRaises(ValueError):
            guard.accept(self.source(sequence=10, raw_stamp=999999999))

    def test_atomic_packet_contains_exact_json_and_image(self):
        meta = {'source':self.source(), 'image_source':self.source()}
        jpeg = b'fixture-not-real-jpeg'
        payload = packet(meta, jpeg)
        magic, nj, ni = PREFIX.unpack_from(payload)
        self.assertEqual(magic, b'SG01')
        self.assertEqual(json.loads(payload[12:12+nj]), meta)
        self.assertEqual(payload[12+nj:], jpeg)
        self.assertEqual(ni, len(jpeg))

    def test_oversize_and_nonfinite_output_rejected(self):
        for meta, image in [({'x':float('nan')},b'a'), ({},b'a'*(MAX_JPEG+1)), ({},b'')]:
            with self.assertRaises(ValueError):
                packet(meta, image)


if __name__ == '__main__':
    unittest.main()
