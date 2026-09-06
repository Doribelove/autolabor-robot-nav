"""Small Python binding to the private, natively built J6M inference adapter."""
import ctypes as C
from pathlib import Path
import time
import numpy as np
from decode_fcos_demo import SIZES
from live_protocol import PAYLOAD_BYTES

SHAPES = [(80 if i < 5 else 4 if i < 10 else 1, SIZES[i % 5], SIZES[i % 5])
          for i in range(15)]
OUTPUT_FLOATS = sum(int(np.prod(shape)) for shape in SHAPES)


class ResidentFCOS:
    """One model load, reusable cached device buffers, one task at a time.

    Returned tensors are views valid until the next infer() or close().
    The caller owns model SHA validation and stdout redirection before loading.
    """
    def __init__(self, lab, model):
        self.context = None
        self.library = C.CDLL(str(Path(lab) / 'bin/libfcos_resident.so'))
        self.library.fcos_create.argtypes = [C.c_char_p, C.c_void_p, C.c_size_t]
        self.library.fcos_create.restype = C.c_void_p
        self.library.fcos_infer.argtypes = [C.c_void_p, C.c_void_p, C.c_size_t,
                                          C.c_void_p, C.c_size_t, C.c_void_p,
                                          C.c_void_p, C.c_size_t]
        self.library.fcos_infer.restype = C.c_int
        self.library.fcos_destroy.argtypes = [C.c_void_p]
        self.library.fcos_destroy.restype = None
        self.error = C.create_string_buffer(512)
        self.timings = (C.c_double * 3)()
        self.buffer = np.empty(OUTPUT_FLOATS, dtype=np.float32)
        self.outputs, offset = [], 0
        for shape in SHAPES:
            count = int(np.prod(shape))
            self.outputs.append(self.buffer[offset:offset + count].reshape(shape))
            offset += count
        started = time.monotonic()
        self.context = self.library.fcos_create(str(model).encode(), self.error, len(self.error))
        if not self.context:
            raise RuntimeError(self.error.value.decode(errors='replace'))
        self.model_load_ms = (time.monotonic() - started) * 1000

    def infer(self, payload):
        if not self.context or not isinstance(payload, bytes) or len(payload) != PAYLOAD_BYTES:
            raise ValueError('Closed session or invalid NV12 payload')
        if self.library.fcos_infer(self.context, C.c_char_p(payload), len(payload),
                                   self.buffer.ctypes.data, self.buffer.size, self.timings,
                                   self.error, len(self.error)):
            raise RuntimeError(self.error.value.decode(errors='replace'))
        return self.outputs, dict(input_copy_ms=self.timings[0], vendor_infer_ms=self.timings[1],
                                  dequantize_ms=self.timings[2])

    def close(self):
        if self.context:
            self.library.fcos_destroy(self.context)
            self.context = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
