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
    if magic != b"FC01":
        raise ValueError("Invalid protocol magic")
    header(seq, stamp, width, height)
    return seq, stamp, width, height


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
