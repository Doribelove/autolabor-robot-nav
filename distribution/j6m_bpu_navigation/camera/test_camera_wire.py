"""Failure and numeric-integrity checks; no camera, BPU or vehicle connection."""
import ctypes
import json
import socket
import struct
import threading
import unittest
import zlib
from camera_wire import RAW_BYTES, LIB, decode, receive, SequenceGuard, validate_header, imu_source_timing


class WireTests(unittest.TestCase):
    def header(self):
        return dict(protocol=1, token='test', serial=23748636, width=1280, height=720,
                    color_encoding='bgra8',depth_encoding='32FC1',depth_units='m',
                    color_raw_bytes=RAW_BYTES,depth_raw_bytes=RAW_BYTES,codec='lz4-block',
                    byte_order='little',frame_id='zed2_left_camera_optical_frame',
                    session='one',sequence=1,color_stamp_ns=100_000_000,depth_stamp_ns=100_000_000,
                    color_lz4_bytes=20,depth_lz4_bytes=20,
                    calibration=dict(serial=23748636,width=1280,height=720,depth_units='m',
                                     left_rectified={},T_left_from_imu_row_major=[0]*16))

    def test_float_bit_patterns_and_corruption(self):
        # Distinct NaN payload, -Inf, finite 1, and signed zero must survive bit for bit.
        raw=struct.pack('<IIII',0x7fc01234,0xff800000,0x3f800000,0x80000000)*(RAW_BYTES//16)
        LIB.LZ4_compress_default.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_int]
        source=ctypes.create_string_buffer(raw); target=ctypes.create_string_buffer(RAW_BYTES+RAW_BYTES//255+16)
        n=LIB.LZ4_compress_default(source,target,RAW_BYTES,len(target))
        packed=bytearray(target.raw[:n]); crc=zlib.crc32(raw)
        self.assertEqual(bytes(decode(packed,crc)),raw)
        with self.assertRaises(ValueError):decode(packed,crc^1)
        with self.assertRaises(ValueError):decode(packed[:3],crc)

    def test_pairing_threshold_and_layout(self):
        h=self.header();h['depth_stamp_ns']+=20_000_000;validate_header(h,'test')
        h['depth_stamp_ns']+=1
        with self.assertRaises(ValueError):validate_header(h,'test')
        for key,value in [('width',640),('depth_encoding','16UC1'),('token','foreign'),('sequence',True)]:
            h=self.header();h[key]=value
            with self.assertRaises(ValueError):validate_header(h,'test')

    def test_sequence_restart_and_replay(self):
        guard=SequenceGuard();self.assertEqual(guard.accept('a',1,10),0)
        self.assertEqual(guard.accept('a',4,20),2)
        for item in [('a',4,30),('a',5,19)]:
            with self.assertRaises(ValueError):guard.accept(*item)
        self.assertEqual(guard.accept('b',1,30),0)
        with self.assertRaises(ValueError):guard.accept('a',6,40)

    def test_oversized_frame_rejected_before_payload(self):
        a,b=socket.socketpair()
        try:
            a.sendall(struct.pack('!4sII',b'ZB01',1,0xffffffff))
            with self.assertRaises(ValueError):receive(b,'test')
        finally:a.close();b.close()

    def test_truncated_header_fails_closed(self):
        a,b=socket.socketpair()
        try:
            a.sendall(struct.pack('!4sII',b'ZB01',100,40)+b'{}');a.shutdown(socket.SHUT_WR)
            with self.assertRaises(EOFError):receive(b,'test')
        finally:a.close();b.close()

    def test_raw_imu_future_time_preserved_and_fusion_rejected(self):
        j=dict(stamp_ns=1_020_000_000,host_read_begin_ns=999_000_000,host_read_end_ns=1_000_000_000)
        result=imu_source_timing(j,1_002_000_000)
        self.assertFalse(result['fusion_time_valid'])
        self.assertIsNone(result['corrected_acquisition_stamp_ns'])
        self.assertEqual(j['stamp_ns'],1_020_000_000)
        self.assertEqual(result['host_read_age_ms'],2)
        with self.assertRaises(ValueError):imu_source_timing(j,1_600_000_000)
        j['stamp_ns']=1_001_000_000
        self.assertTrue(imu_source_timing(j,1_002_000_000)['fusion_time_valid'])


if __name__=='__main__':unittest.main()
