"""Bounded, lossless RGB-D framing; original acquisition time is immutable."""
import ctypes
import json
import struct
import zlib

RAW_BYTES = 1280 * 720 * 4
MAX_BODY = 2 * (RAW_BYTES + RAW_BYTES // 255 + 16)
LIB = ctypes.CDLL('liblz4.so.1')
LIB.LZ4_decompress_safe.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
LIB.LZ4_decompress_safe.restype = ctypes.c_int


def exact(sock, size):
    result = bytearray(size)
    view = memoryview(result)
    offset = 0
    while offset < size:
        count = sock.recv_into(view[offset:])
        if count == 0:
            raise EOFError('RGB-D peer closed mid-message')
        offset += count
    return result


def validate_header(h, token):
    expected = dict(protocol=1, token=token, serial=23748636, width=1280, height=720,
                    color_encoding='bgra8', depth_encoding='32FC1', depth_units='m',
                    color_raw_bytes=RAW_BYTES, depth_raw_bytes=RAW_BYTES,
                    codec='lz4-block', byte_order='little',
                    frame_id='zed2_left_camera_optical_frame')
    if any(h.get(k) != v for k, v in expected.items()):
        raise ValueError('RGB-D identity/layout mismatch')
    if not isinstance(h.get('session'), str) or not 1 <= len(h['session']) <= 80:
        raise ValueError('Invalid capture session')
    for key in ['sequence', 'color_stamp_ns', 'depth_stamp_ns']:
        if type(h.get(key)) is not int or h[key] <= 0:
            raise ValueError('Invalid sequence or timestamp')
    if abs(h['color_stamp_ns'] - h['depth_stamp_ns']) > 20_000_000:
        raise ValueError('RGB-D pairing exceeds 20 ms')
    cal = h.get('calibration', {})
    if (cal.get('serial') != h['serial'] or cal.get('width') != 1280 or cal.get('height') != 720
            or cal.get('depth_units') != 'm' or 'left_rectified' not in cal
            or len(cal.get('T_left_from_imu_row_major', [])) != 16):
        raise ValueError('Missing/mismatched calibration')
    for k in ['color_lz4_bytes', 'depth_lz4_bytes']:
        if type(h.get(k)) is not int or not 1 <= h[k] <= MAX_BODY // 2:
            raise ValueError('Invalid compressed length')


def decode(block, crc):
    source = (ctypes.c_char * len(block)).from_buffer(block)
    target = bytearray(RAW_BYTES)
    buffer = (ctypes.c_char * RAW_BYTES).from_buffer(target)
    count = LIB.LZ4_decompress_safe(source, buffer, len(block), RAW_BYTES)
    if count != RAW_BYTES or zlib.crc32(target) != crc:
        raise ValueError('Lossless decode length/CRC mismatch')
    return target


def receive(sock, token):
    prefix = exact(sock, 12)
    magic, header_size, body_size = struct.unpack('!4sII', prefix)
    if magic != b'ZB01' or not 1 <= header_size <= 65536 or not 1 <= body_size <= MAX_BODY:
        raise ValueError('Invalid RGB-D wire bounds')
    header = json.loads(exact(sock, header_size))
    validate_header(header, token)
    nc, nd = header['color_lz4_bytes'], header['depth_lz4_bytes']
    if nc + nd != body_size:
        raise ValueError('Payload lengths disagree')
    color = decode(exact(sock, nc), header['color_crc32'])
    depth = decode(exact(sock, nd), header['depth_crc32'])
    del header['token']
    return header, color, depth, 12 + header_size + body_size


class SequenceGuard:
    def __init__(self):
        self.session = None
        self.retired = set()
        self.sequence = 0
        self.stamp = 0

    def accept(self, session, sequence, stamp):
        if session != self.session:
            if session in self.retired:
                raise ValueError('Retired capture session')
            if self.session is not None:
                self.retired.add(self.session)
            if len(self.retired) > 4096:
                raise ValueError('Session history full; restart receiver explicitly')
            self.session, self.sequence, self.stamp = session, 0, 0
        if sequence <= self.sequence or stamp <= self.stamp:
            raise ValueError('Duplicate/regressing source frame')
        gap = sequence - self.sequence - 1 if self.sequence else 0
        self.sequence, self.stamp = sequence, stamp
        return gap


def imu_source_timing(packet, source_now_ns):
    """Keep raw SDK time, and separately declare whether it is usable for fusion.

    A live sensor read is not a replacement acquisition timestamp. In particular,
    SDK future timestamps are delivered as diagnostic data with fusion disabled,
    instead of reconnecting and losing the continuous raw sensor stream.
    """
    for key in ['stamp_ns','host_read_begin_ns','host_read_end_ns']:
        if type(packet.get(key)) is not int or packet[key] <= 0:
            raise ValueError('Missing original IMU/host read timestamp')
    begin, end, stamp = (packet[k] for k in ['host_read_begin_ns','host_read_end_ns','stamp_ns'])
    if not 0 <= end-begin <= 100_000_000:
        raise ValueError('Invalid sensor read interval')
    read_age = (source_now_ns-end)/1e6
    sdk_age = (source_now_ns-stamp)/1e6
    if not -5 <= read_age <= 500 or abs(stamp-end) > 500_000_000:
        raise ValueError('IMU transport/source time out of bounds')
    return dict(host_read_age_ms=read_age,sdk_source_age_ms=sdk_age,
                sdk_minus_host_read_ms=(stamp-end)/1e6,
                fusion_time_valid=(-5 <= sdk_age <= 500),
                fusion_eligible=False,source_clock_alignment_accepted=False,
                corrected_acquisition_stamp_ns=None)
