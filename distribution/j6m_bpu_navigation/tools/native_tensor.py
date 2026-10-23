"""Binding to the private J6M executor; model identity is mandatory."""
import ctypes as C
import hashlib
import json
import re
from pathlib import Path
import time

import numpy as np


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def check_model_file(path, expected):
    path = Path(path)
    if (path.is_symlink() or not path.is_file() or path.suffix != '.hbm'
            or not isinstance(expected, str) or not re.fullmatch('[0-9a-f]{64}', expected)
            or sha256(path) != expected):
        raise ValueError('Expected a regular HBM with the pinned SHA256')
    return path.resolve()


class TensorEngine:
    """Reusable device buffers, dense input packing, stride-aware F32 outputs.

    This execution API never grants production acceptance or motion eligibility.
    Output arrays are owned by the caller, and remain valid after the next call.
    """
    def __init__(self, library, model, model_name, model_sha256):
        model = check_model_file(model, model_sha256)
        if not isinstance(model_name, str) or not model_name:
            raise ValueError('An explicit compiler model name is required')
        self.context = None
        self.library = C.CDLL(str(Path(library).resolve(strict=True)))
        self.error = C.create_string_buffer(1024)
        declarations = {
            'create': ([C.c_char_p, C.c_char_p, C.c_void_p, C.c_size_t], C.c_void_p),
            'metadata': ([C.c_void_p, C.c_void_p, C.c_size_t, C.c_void_p, C.c_size_t], C.c_int),
            'input': ([C.c_void_p, C.c_int, C.c_void_p, C.c_size_t, C.c_void_p, C.c_size_t], C.c_int),
            'infer': ([C.c_void_p, C.c_int, C.c_void_p, C.c_void_p, C.c_size_t], C.c_int),
            'output': ([C.c_void_p, C.c_int, C.c_void_p, C.c_size_t, C.c_void_p, C.c_size_t], C.c_int),
            'destroy': ([C.c_void_p], None),
        }
        for name, (arguments, result) in declarations.items():
            function = getattr(self.library, 'j6m_tensor_' + name)
            function.argtypes, function.restype = arguments, result
        started = time.monotonic()
        self.context = self.library.j6m_tensor_create(
            str(model).encode(), model_name.encode(), self.error, len(self.error))
        if not self.context:
            raise RuntimeError(self.error.value.decode(errors='replace'))
        try:
            buffer = C.create_string_buffer(65536)
            self._check(self.library.j6m_tensor_metadata(
                self.context, buffer, len(buffer), self.error, len(self.error)))
            self.metadata = json.loads(buffer.value)
            if self.metadata['model_name'] != model_name or sha256(model) != model_sha256:
                raise RuntimeError('Model identity changed during load')
            self.model_sha256 = model_sha256
            self.load_ms = (time.monotonic() - started) * 1000
        except Exception:
            self.close()
            raise

    def _check(self, code):
        if code:
            raise RuntimeError(self.error.value.decode(errors='replace'))

    def infer(self, tensors, timeout_ms=2000):
        if not self.context or len(tensors) != len(self.metadata['inputs']):
            raise ValueError('Closed engine or input count mismatch')
        if isinstance(timeout_ms, bool) or not isinstance(timeout_ms, int) or not 1 <= timeout_ms <= 8000:
            raise ValueError('Device deadline must be 1..8000 ms')
        prepared = []
        for value, spec in zip(tensors, self.metadata['inputs']):
            dtype = np.dtype('float32') if spec['type'] == 7 else np.dtype('uint8') if spec['type'] == 3 else None
            if (dtype is None or not isinstance(value, np.ndarray) or value.dtype != dtype
                    or list(value.shape) != spec['shape'] or not np.isfinite(value).all()):
                raise ValueError('Input dtype/shape/finite-value mismatch')
            prepared.append(np.ascontiguousarray(value))
        started = time.monotonic()
        for index, value in enumerate(prepared):
            self._check(self.library.j6m_tensor_input(
                self.context, index, value.ctypes.data, value.nbytes, self.error, len(self.error)))
        input_ms = (time.monotonic() - started) * 1000
        device_ms = C.c_double()
        self._check(self.library.j6m_tensor_infer(
            self.context, timeout_ms, C.byref(device_ms), self.error, len(self.error)))
        started = time.monotonic()
        outputs = []
        for index, spec in enumerate(self.metadata['outputs']):
            value = np.empty(spec['shape'], dtype=np.float32)
            self._check(self.library.j6m_tensor_output(
                self.context, index, value.ctypes.data, value.size, self.error, len(self.error)))
            outputs.append(value)
        return outputs, {'input_copy_ms': input_ms, 'vendor_infer_ms': device_ms.value,
                         'output_copy_ms': (time.monotonic() - started) * 1000}

    def close(self):
        if self.context:
            self.library.j6m_tensor_destroy(self.context)
            self.context = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
