"""Decode actual three/four-camera ground images for the existing YOLO consumer."""
import json
import hashlib
import math
import struct
import time

import cv2
import numpy as np


def decode_ground_frame(data, now=None):
    now = time.time() if now is None else now
    if len(data) < 12:
        raise ValueError('Truncated ground frame')
    magic, nj, ni = struct.unpack('!4sII', data[:12])
    if (magic != b'BG01' or not 0 < nj <= 128*1024 or
            not 0 < ni <= 2*1024*1024 or len(data) != 12+nj+ni):
        raise ValueError('Invalid ground frame envelope')
    meta = json.loads(data[12:12+nj])
    configured = meta.get('configured_cameras', ['front', 'left', 'right'])
    valid_sets = ({'front', 'left', 'right'}, {'front', 'left', 'right', 'rear'})
    if (not isinstance(configured, list) or any(not isinstance(n, str) for n in configured)
            or len(configured) != len(set(configured)) or set(configured) not in valid_sets
            or not isinstance(meta.get('sources'), dict)
            or set(meta['sources']) != set(configured)):
        raise ValueError('Ground frame has missing or unexpected camera inputs')
    produced = meta.get('produced_epoch_s')
    if (meta.get('protocol') != 'conbine.ground_bev.frame.v1' or
            not meta.get('fusion_session') or not isinstance(produced, (int, float)) or
            not math.isfinite(produced) or not -.2 <= now-produced <= 2):
        raise ValueError('Ground frame is stale or has an invalid time')
    jpeg = data[12+nj:]
    image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if image is None or list(image.shape[1::-1]) != [meta['image_width'], meta['image_height']]:
        raise ValueError('Ground image dimensions mismatch')
    return meta, image, jpeg


def encode_view_frame(ground_packet, upright_images):
    """Bundle exactly the upright views used to produce this BEV frame."""
    parts, views = [ground_packet], []
    for name, image in sorted(upright_images.items()):
        ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 94])
        if not ok:
            raise ValueError('Cannot encode camera '+name)
        value = jpeg.tobytes()
        parts.append(value)
        views.append(dict(camera=name, bytes=len(value), size=list(image.shape[1::-1]),
                          sha256=hashlib.sha256(value).hexdigest()))
    header = json.dumps(dict(ground_bytes=len(ground_packet), views=views)).encode()
    body = b''.join(parts)
    if len(header) > 128*1024 or len(body) > 10*1024*1024:
        raise ValueError('Oversized camera/BEV bundle')
    return struct.pack('!4sII', b'MV01', len(header), len(body))+header+body


def decode_view_frame(data, now=None):
    if len(data) < 12:
        raise ValueError('Truncated multi-view frame')
    magic, nh, nb = struct.unpack('!4sII', data[:12])
    if magic != b'MV01' or not 0 < nh <= 128*1024 or not 0 < nb <= 10*1024*1024 or len(data) != 12+nh+nb:
        raise ValueError('Invalid multi-view envelope')
    header = json.loads(data[12:12+nh])
    body = memoryview(data)[12+nh:]
    cursor = header['ground_bytes']
    if type(cursor) is not int or not 0 < cursor <= len(body):
        raise ValueError('Invalid embedded ground packet')
    meta, bev, raw = decode_ground_frame(bytes(body[:cursor]), now)
    rows = header['views']
    if (not isinstance(rows, list) or len(rows) != len(meta['sources']) or
            {v['camera'] for v in rows} != set(meta['sources'])):
        raise ValueError('Missing or duplicated camera view')
    images = {}
    for row in rows:
        count, name = row['bytes'], row['camera']
        if type(count) is not int or not 0 < count <= 2*1024*1024 or cursor+count > len(body):
            raise ValueError('Invalid camera image length')
        jpeg = bytes(body[cursor:cursor+count]); cursor += count
        if hashlib.sha256(jpeg).hexdigest() != row['sha256']:
            raise ValueError('Camera image digest mismatch')
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
        if image is None or list(image.shape[1::-1]) != row['size'] or row['size'] != meta['camera_image_sizes'][name]:
            raise ValueError('Camera geometry/image size mismatch')
        images[name] = image
    if cursor != len(body):
        raise ValueError('Trailing multi-view data')
    return meta, bev, raw, images
