"""Bounded binary frames over SSH stdio; source identity survives the round trip."""
import json
import math
import os
import select
import struct
import time

MAGIC = b'J6T1'
PREFIX = struct.Struct('!4sII')
MAX_HEADER = 32768
MAX_PAYLOAD = 64 * 1024 * 1024


def _transfer(stream, data=None, count=None, timeout=3.0):
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('Finite positive transport deadline required')
    fd = stream.fileno()
    blocking = os.get_blocking(fd)
    os.set_blocking(fd, False)
    deadline = time.monotonic() + timeout
    result = bytearray() if data is None else memoryview(data)
    cursor = 0
    total = count if data is None else len(data)
    try:
        while cursor < total:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Tensor transport deadline exceeded')
            readable, writable, _ = select.select(
                [fd] if data is None else [], [fd] if data is not None else [], [], remaining)
            if not readable and not writable:
                raise TimeoutError('Tensor transport deadline exceeded')
            try:
                if data is None:
                    chunk = os.read(fd, min(total - cursor, 65536))
                    if not chunk:
                        raise EOFError('Tensor worker disconnected')
                    result.extend(chunk)
                    cursor += len(chunk)
                else:
                    cursor += os.write(fd, result[cursor:cursor + 65536])
            except BlockingIOError:
                continue
        return bytes(result) if data is None else None
    finally:
        os.set_blocking(fd, blocking)


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError('Duplicate JSON key')
        result[key] = value
    return result


def send(stream, header, payload=b'', timeout=3.0):
    if not isinstance(header, dict) or not isinstance(payload, bytes):
        raise ValueError('Expected JSON object and bytes')
    encoded = json.dumps(header, allow_nan=False, separators=(',', ':')).encode()
    if not 0 < len(encoded) <= MAX_HEADER or len(payload) > MAX_PAYLOAD:
        raise ValueError('Oversized tensor frame')
    _transfer(stream, PREFIX.pack(MAGIC, len(encoded), len(payload)) + encoded + payload,
              timeout=timeout)


def receive(stream, timeout=3.0):
    started = time.monotonic()
    magic, header_size, payload_size = PREFIX.unpack(_transfer(stream, count=PREFIX.size, timeout=timeout))
    if magic != MAGIC or not 0 < header_size <= MAX_HEADER or payload_size > MAX_PAYLOAD:
        raise ValueError('Invalid tensor frame prefix')
    def read(count):
        remaining = timeout - (time.monotonic() - started)
        if remaining <= 0:
            raise TimeoutError('Tensor transport deadline exceeded')
        return _transfer(stream, count=count, timeout=remaining)
    header = json.loads(read(header_size), object_pairs_hook=_unique_object,
                        parse_constant=lambda value: (_ for _ in ()).throw(ValueError('Nonfinite JSON')))
    if not isinstance(header, dict):
        raise ValueError('Expected tensor header object')
    return header, read(payload_size) if payload_size else b''


def identity(header):
    for name in ['sequence', 'stamp_ns']:
        if type(header.get(name)) is not int or not 0 < header[name] < 2**63:
            raise ValueError('Invalid source identity: ' + name)
    frame = header.get('frame_id')
    if not isinstance(frame, str) or not frame or len(frame) > 256:
        raise ValueError('Invalid source frame ID')
    return {key: header[key] for key in ['sequence', 'stamp_ns', 'frame_id']}


def pack_arrays(arrays):
    import numpy as np
    descriptors, chunks = [], []
    for array in arrays:
        if not isinstance(array, np.ndarray) or array.dtype not in [np.dtype('uint8'), np.dtype('float32')]:
            raise ValueError('Only dense U8/F32 arrays are supported')
        if len(chunks) >= 32 or array.nbytes > MAX_PAYLOAD - sum(d['bytes'] for d in descriptors):
            raise ValueError('Oversized tensor list')
        if (not 0 < array.ndim <= 10 or any(n <= 0 for n in array.shape)
                or not np.isfinite(array).all()):
            raise ValueError('Invalid tensor shape/values')
        dtype = 'u1' if array.dtype == np.uint8 else '<f4'
        chunks.append(np.ascontiguousarray(array, dtype=dtype).tobytes())
        descriptors.append({'shape': list(array.shape), 'dtype': dtype, 'bytes': len(chunks[-1])})
    if not 0 < len(descriptors) <= 32 or sum(d['bytes'] for d in descriptors) > MAX_PAYLOAD:
        raise ValueError('Oversized or empty tensor list')
    return descriptors, b''.join(chunks)


def unpack_arrays(descriptors, payload, specs, outputs=False):
    import numpy as np
    if not isinstance(descriptors, list) or len(descriptors) != len(specs):
        raise ValueError('Tensor count mismatch')
    arrays, cursor = [], 0
    for descriptor, spec in zip(descriptors, specs):
        if (not isinstance(descriptor, dict) or not isinstance(descriptor.get('shape'), list)
                or any(type(n) is not int for n in descriptor['shape'])
                or type(descriptor.get('bytes')) is not int):
            raise ValueError('Malformed tensor descriptor')
        dtype = '<f4' if outputs or spec['type'] == 7 else 'u1' if spec['type'] == 3 else None
        count = math.prod(spec['shape'])
        byte_count = count * (4 if dtype == '<f4' else 1)
        if (dtype is None or descriptor != {'shape': spec['shape'], 'dtype': dtype, 'bytes': byte_count}
                or byte_count > MAX_PAYLOAD or cursor + byte_count > len(payload)):
            raise ValueError('Tensor shape/dtype/length differs from loaded HBM')
        value = np.frombuffer(payload, dtype=dtype, count=count, offset=cursor).reshape(spec['shape'])
        if not np.isfinite(value).all():
            raise ValueError('Nonfinite tensor payload')
        arrays.append(value)
        cursor += byte_count
    if cursor != len(payload):
        raise ValueError('Unaccounted payload bytes')
    return arrays
