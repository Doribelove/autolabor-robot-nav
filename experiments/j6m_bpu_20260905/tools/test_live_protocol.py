import io
import json
import os
import struct
import unittest
from live_protocol import header, unpack, read_exact, write_all, receive_json, send_json


class ProtocolTest(unittest.TestCase):
    def test_identity(self):
        self.assertEqual(unpack(header(7, 123456, 640, 360)), (7, 123456, 640, 360))

    def test_bounds(self):
        for args in ((0, 3, 640, 360), (1, 0, 640, 360), (1, 2, 999999, 360)):
            with self.assertRaises(ValueError):
                header(*args)

    def test_magic(self):
        with self.assertRaises(ValueError):
            unpack(b"BAD!" + header(1, 2, 640, 360)[4:])

    def test_json_finite_bound(self):
        for value in ({"x": float("nan")}, {"x": "A" * 70000}):
            with self.assertRaises(ValueError):
                send_json(io.BytesIO(), value)

    def test_pipe_json(self):
        r, w = os.pipe()
        with os.fdopen(r, "rb", buffering=0) as reader, os.fdopen(w, "wb", buffering=0) as writer:
            send_json(writer, {"ok": True})
            self.assertEqual(receive_json(reader), {"ok": True})

    def test_oversized_response(self):
        r, w = os.pipe()
        with os.fdopen(r, "rb", buffering=0) as reader, os.fdopen(w, "wb", buffering=0) as writer:
            writer.write(struct.pack("!I", 999999))
            with self.assertRaises(ValueError):
                receive_json(reader)

    def test_eof(self):
        r, w = os.pipe()
        os.close(w)
        with os.fdopen(r, "rb", buffering=0) as reader:
            with self.assertRaises(EOFError):
                read_exact(reader, 4)

    def test_timeout(self):
        r, w = os.pipe()
        with os.fdopen(r, "rb", buffering=0) as reader, os.fdopen(w, "wb", buffering=0):
            with self.assertRaises(TimeoutError):
                read_exact(reader, 4, timeout=0.01)

    def test_write_all(self):
        r, w = os.pipe()
        with os.fdopen(r, "rb", buffering=0) as reader, os.fdopen(w, "wb", buffering=0) as writer:
            write_all(writer, b"abc")
            self.assertEqual(read_exact(reader, 3), b"abc")
