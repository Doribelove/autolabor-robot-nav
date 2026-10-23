"""Frame identity and bounded display protocol for numeric-port GDC/BPU preview.

Source device timestamps are retained verbatim. Freshness uses the J6M SDK read
clock; no claim that it is calibrated exposure time or a ROS acquisition stamp.
"""
import json
import math
import struct
import time

PREFIX = struct.Struct('!4sII')
MAX_JSON = 128*1024
MAX_JPEG = 2*1024*1024
MAX_AGE_MS = 500
DETECTOR_SHA = 'f2f1f7e9753b53b0494495be44af92a0603b3ee6c71e69e0f58a0d50544a8cb8'
CLASSIFIER_SHA = 'd4cb80f5f7364d1156ea2275eba9e522991dec309f2c397e74649254fe36a89f'


def capture_identity(meta, port, lut_sha, now=None):
    now = time.monotonic() if now is None else now
    if (meta.get('camera_port') != port or meta.get('gdc_applied') != 1
            or meta.get('layout') != 'NV12' or meta.get('vendor_format') != 2
            or (meta.get('width'), meta.get('height'), meta.get('bytes')) != (960, 540, 777600)
            or meta.get('lut_sha256') != lut_sha
            or not isinstance(meta.get('session'), str) or not 1 <= len(meta['session']) <= 128):
        raise ValueError('GDC camera identity, LUT or image format mismatch')
    received = meta.get('received_monotonic')
    if not isinstance(received, (float, int)) or not math.isfinite(received):
        raise ValueError('Missing SDK read time')
    if not 0 <= (now-received)*1000 <= MAX_AGE_MS:
        raise ValueError('GDC frame read expired')
    for name in ('sequence', 'raw_stamp', 'hal_received_ns'):
        if type(meta.get(name)) is not int or not 0 <= meta[name] < 2**64:
            raise ValueError('Invalid source '+name)
    return dict(session=meta['session'], port=port, sequence=meta['sequence'],
                raw_stamp=str(meta['raw_stamp']), hal_received_ns=str(meta['hal_received_ns']),
                frame_id='surround_rx1_port%d_rectified_optical'%port)


class SourceGuard:
    def __init__(self):
        self.current = {}
        self.retired = {p: set() for p in range(3)}

    def accept(self, source):
        p = source['port']
        if p not in self.retired:
            raise ValueError('Unknown port')
        old = self.current.get(p)
        if source['session'] in self.retired[p]:
            raise ValueError('Retired camera session')
        if old and source['session'] == old['session']:
            if source == old:
                return False
            if source['sequence'] <= old['sequence'] or int(source['raw_stamp']) <= int(old['raw_stamp']):
                raise ValueError('Camera frame reordered')
        elif old:
            if len(self.retired[p]) >= 1024:
                raise ValueError('Camera session history full')
            self.retired[p].add(old['session'])
        self.current[p] = dict(source)
        return True


def packet(meta, jpeg):
    encoded = json.dumps(meta, allow_nan=False, separators=(',', ':')).encode()
    if not 0 < len(encoded) <= MAX_JSON or not 0 < len(jpeg) <= MAX_JPEG:
        raise ValueError('Display packet exceeds bounds')
    return PREFIX.pack(b'SG01', len(encoded), len(jpeg)) + encoded + jpeg
