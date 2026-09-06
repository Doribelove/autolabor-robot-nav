"""Bounded protocol for an SSH-only, one-request-at-a-time visual demo."""
import json
import os
import select
import struct
import time

HEADER = struct.Struct("!4sIQII")
Y_BYTES = 896 * 896
PAYLOAD_BYTES = Y_BYTES * 3 // 2
MAX_JSON = 65536


def header(sequence, stamp_ns, width, height):
    if not 1 <= sequence < 2 ** 32 or not 1 <= stamp_ns < 2 ** 64:
        raise ValueError("Invalid sequence/timestamp")
    if not 64 <= width <= 1920 or not 64 <= height <= 1080:
        raise ValueError("Invalid source dimensions")
    return HEADER.pack(b"FC01", sequence, stamp_ns, width, height)


def unpack(data):
    magic, seq, stamp, width, height = HEADER.unpack(data)
    if magic not in (b"FC01", b"FC03"):
        raise ValueError("Invalid protocol magic")
    header(seq, stamp, width, height)
    return seq, stamp, width, height


def crop_geometry(width, height):
    scale = min(896 / width, 896 / height)
    w, h = round(width * scale), round(height * scale)
    return w, h, (w + 1) // 2 * 2, (h + 1) // 2


def frame_packet(sequence, stamp_ns, width, height, payload, crop=True):
    """FC03 omits only verified constant letterbox padding; FC01 stays supported.

    Chroma includes the entire boundary pair for odd resized dimensions. This
    is byte-exact NV12 transport, with no codec or changed model preprocessing.
    """
    prefix = header(sequence, stamp_ns, width, height)
    if not isinstance(payload, bytes) or len(payload) != PAYLOAD_BYTES:
        raise ValueError("Malformed NV12 payload")
    if crop:
        import numpy as np
        w, h, uvw, uvh = crop_geometry(width, height)
        y = np.frombuffer(payload, dtype=np.uint8, count=Y_BYTES).reshape(896, 896)
        uv = np.frombuffer(payload, dtype=np.uint8, offset=Y_BYTES).reshape(448, 896)
        # Fallback for callers that did not use our fixed black letterbox.
        if (w * h + uvw * uvh < PAYLOAD_BYTES and
                np.all(y[h:] == 16) and np.all(y[:h, w:] == 16) and
                np.all(uv[uvh:] == 128) and np.all(uv[:uvh, uvw:] == 128)):
            return b"FC03" + prefix[4:] + y[:h, :w].tobytes() + uv[:uvh, :uvw].tobytes()
    return prefix + payload


def read_payload(stream, prefix, timeout=8.0):
    _, _, width, height = unpack(prefix)
    if prefix[:4] == b"FC01":
        return read_exact(stream, PAYLOAD_BYTES, timeout)
    import numpy as np
    w, h, uvw, uvh = crop_geometry(width, height)
    packed = read_exact(stream, w * h + uvw * uvh, timeout)
    values = np.frombuffer(packed, dtype=np.uint8)
    result = np.empty(PAYLOAD_BYTES, np.uint8)
    y, uv = result[:Y_BYTES].reshape(896, 896), result[Y_BYTES:].reshape(448, 896)
    y.fill(16)
    uv.fill(128)
    y[:h, :w] = values[:w * h].reshape(h, w)
    uv[:uvh, :uvw] = values[w * h:].reshape(uvh, uvw)
    return result.tobytes()


def read_exact(stream, size, timeout=8.0):
    # Streams must be unbuffered so select observes every pending byte.
    deadline = time.monotonic() + timeout
    parts, count = [], 0
    while count < size:
        remaining = deadline - time.monotonic()
        if remaining <= 0 or not select.select([stream], [], [], remaining)[0]:
            raise TimeoutError("Preview connection timed out")
        part = stream.read(size - count)
        if not part:
            raise EOFError("Preview connection closed")
        parts.append(part)
        count += len(part)
    return b"".join(parts)


def send_json(stream, value):
    data = json.dumps(value, allow_nan=False, separators=(",", ":")).encode()
    if len(data) > MAX_JSON:
        raise ValueError("Oversized preview result")
    stream.write(struct.pack("!I", len(data)) + data)
    stream.flush()


def write_all(stream, data, timeout=8.0):
    fd = stream.fileno()
    os.set_blocking(fd, False)
    deadline = time.monotonic() + timeout
    remaining = memoryview(data)
    while remaining:
        seconds = deadline - time.monotonic()
        if seconds <= 0 or not select.select([], [stream], [], seconds)[1]:
            raise TimeoutError("Preview send timed out")
        try:
            count = os.write(fd, remaining[:65536])
        except BlockingIOError:
            continue
        if count <= 0:
            raise EOFError("Preview send closed")
        remaining = remaining[count:]


def receive_json(stream, timeout=8.0):
    size = struct.unpack("!I", read_exact(stream, 4, timeout))[0]
    if not 1 <= size <= MAX_JSON:
        raise ValueError("Oversized/empty preview result")
    return json.loads(read_exact(stream, size, timeout))
