import ctypes as C
import os
from pathlib import Path
import unittest

import numpy as np

from distribution.j6m_bpu_navigation.surround.capture import Meta


@unittest.skipUnless(os.environ.get("SURROUND_NATIVE_TEST_DIR"), "Set test library directory for offline SDK double")
class NativeCaptureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(os.environ["SURROUND_NATIVE_TEST_DIR"]).resolve()
        cls.path = str(root / "libsurround_fake_vio.so").encode()
        cls.fake = C.CDLL(cls.path.decode())
        cls.api = C.CDLL(str(root / "libj6m_surround_capture.so"))
        cls.api.sc_error.restype = C.c_char_p
        cls.api.sc_open.argtypes = [C.c_char_p] * 4
        cls.api.sc_open.restype = C.c_void_p
        cls.api.sc_start.argtypes = [C.c_void_p, C.c_uint]
        cls.api.sc_close.argtypes = [C.c_void_p]
        cls.api.sc_read.argtypes = [C.c_void_p, C.c_uint, C.c_int, C.c_uint,
                                   C.c_void_p, C.c_size_t, C.POINTER(Meta)]
        cls.api.sc_set_gdc.argtypes = [C.c_void_p, C.c_uint, C.c_void_p, C.c_size_t]

    def setUp(self):
        self.fake.fake_reset()
        self.ctx = self.api.sc_open(b"unused", b"unused", self.path, self.path)
        self.assertTrue(self.ctx, self.api.sc_error())

    def tearDown(self):
        if self.ctx:
            self.api.sc_close(self.ctx)

    def test_stride_removed_and_original_timestamp_preserved(self):
        data = np.zeros(16, np.uint8)
        meta = Meta()
        self.assertEqual(self.api.sc_read(self.ctx, 0, 0, 0, data.ctypes.data, data.size, C.byref(meta)), 0)
        np.testing.assert_array_equal(data, np.arange(16))
        self.assertEqual(meta.raw_stamp, 123456789)
        self.assertEqual(meta.sequence, 42)
        self.assertEqual(meta.hal_received_ns, 1700000000000123000)
        self.assertEqual(self.fake.fake_releases(), 1)

    def test_small_destination_releases_sdk_buffer(self):
        data = np.zeros(1, np.uint8)
        self.assertEqual(self.api.sc_read(self.ctx, 0, 0, 0, data.ctypes.data, 1, C.byref(Meta())), -1)
        self.assertEqual(self.fake.fake_releases(), 1)

    def test_pym_and_gdc_both_released(self):
        cfg = C.create_string_buffer(b"1234")
        self.assertEqual(self.api.sc_set_gdc(self.ctx, 0, cfg, 4), 0)
        data = np.zeros(12, np.uint8)
        meta = Meta()
        self.assertEqual(self.api.sc_read(self.ctx, 0, 1, 0, data.ctypes.data, data.size, C.byref(meta)), 0)
        np.testing.assert_array_equal(data[:8], np.arange(8))
        np.testing.assert_array_equal(data[8:], [128] * 4)
        self.assertEqual(meta.gdc_applied, 1)
        self.assertEqual(self.fake.fake_releases(), 2)

    def test_cim_cannot_be_sent_directly_to_gdc(self):
        cfg = C.create_string_buffer(b"1234")
        self.api.sc_set_gdc(self.ctx, 0, cfg, 4)
        data = np.zeros(16, np.uint8)
        self.assertEqual(self.api.sc_read(self.ctx, 0, 0, 0, data.ctypes.data, data.size, C.byref(Meta())), -1)
        self.assertEqual(self.fake.fake_releases(), 1)

    def test_gdc_failure_releases_input_without_returning_raw_frame(self):
        cfg = C.create_string_buffer(b"1234")
        self.assertEqual(self.api.sc_set_gdc(self.ctx, 0, cfg, 4), 0)
        self.fake.fake_fail_gdc()
        data, meta = np.zeros(12, np.uint8), Meta()
        self.assertEqual(self.api.sc_read(self.ctx, 0, 1, 0, data.ctypes.data, data.size, C.byref(meta)), -1)
        self.assertIn(b"hb_vio_run_gdc: -88", self.api.sc_error())
        self.assertEqual(self.fake.fake_releases(), 1)
        self.assertFalse(data.any())
        self.assertEqual(meta.bytes, 0)

    def test_bad_gdc_output_returns_both_buffers(self):
        cfg = C.create_string_buffer(b"1234")
        self.assertEqual(self.api.sc_set_gdc(self.ctx, 0, cfg, 4), 0)
        self.fake.fake_bad_gdc_shape()
        data = np.zeros(12, np.uint8)
        self.assertEqual(self.api.sc_read(self.ctx, 0, 1, 0, data.ctypes.data, data.size, C.byref(Meta())), -1)
        self.assertEqual(self.fake.fake_releases(), 2)

    def test_gdc_output_format_and_size_keep_input_exposure_identity(self):
        cfg = C.create_string_buffer(b"1234")
        self.assertEqual(self.api.sc_set_gdc(self.ctx, 0, cfg, 4), 0)
        self.fake.fake_gdc_metadata()
        data, meta = np.zeros(12, np.uint8), Meta()
        self.assertEqual(self.api.sc_read(self.ctx, 0, 1, 0, data.ctypes.data, data.size, C.byref(meta)), 0)
        self.assertEqual((meta.width, meta.height, meta.bytes, meta.vendor_format), (2, 2, 6, 31))
        self.assertEqual((meta.raw_stamp, meta.sequence), (123456789, 42))
        self.assertEqual(meta.hal_received_ns, 1700000000000123000)
        self.assertEqual(self.fake.fake_releases(), 2)

    def test_cim_nv16_keeps_both_chroma_rows(self):
        self.fake.fake_cim_nv16()
        data = np.zeros(16, np.uint8)
        self.assertEqual(self.api.sc_read(self.ctx, 0, 2, 0, data.ctypes.data, data.size, C.byref(Meta())), 0)
        np.testing.assert_array_equal(data[:8], np.arange(8))
        np.testing.assert_array_equal(data[8:], [128] * 8)
        self.assertEqual(self.fake.fake_releases(), 1)

    def test_partial_start_cleanup(self):
        self.fake.fake_fail_start()
        self.assertEqual(self.api.sc_start(self.ctx, 0), -1)
        self.api.sc_close(self.ctx)
        self.ctx = None
        self.assertEqual(self.fake.fake_stops(), 1)
        self.assertEqual(self.fake.fake_deinit(), 2)


if __name__ == "__main__":
    unittest.main()
